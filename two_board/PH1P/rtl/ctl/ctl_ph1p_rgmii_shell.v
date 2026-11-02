//=============================================================================
//  ctl_ph1p_rgmii_shell  --  PH1P RGMII/GMII physical shell and reset release
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk_50m, recovered RXC, 125 MHz TX/TX90
//  复位     : I_rst_n
//  参数     : P_PHY_RESET_HOLD_CYCLES
//  验证记录 : docs/ethernet-phy-repair-20261018.md；上板未做
//=============================================================================
`timescale 1ns / 1ps
// PH1P master-board Ethernet physical shell.
// The common ctl_lpr_packet_link owns raw-Ethernet framing/FCS and connects
// to the GMII client ports below.  This module owns only PLL, RGMII DDR I/O,
// RXC buffering, and reset release.  No TEMAC is instantiated here.

module ctl_ph1p_rgmii_shell (
    input  wire       I_clk_50m,
    input  wire       I_rst_n,

    output wire [3:0] O_eth_txd,
    output wire       O_eth_txc,
    output wire       O_eth_tx_ctl,
    input  wire [3:0] I_eth_rxd,
    input  wire       I_eth_rxc,
    input  wire       I_eth_rx_ctl,
    output wire       O_eth_mdc,
    inout  wire       IO_eth_mdio,
    output wire       O_eth_rst_n,

    input  wire [7:0] I_gmii_txd,
    input  wire       I_gmii_tx_en,
    input  wire       I_gmii_tx_er,
    output wire [7:0] O_gmii_rxd,
    output wire       O_gmii_rx_dv,
    output wire       O_gmii_rx_er,
    output wire       O_gmii_rx_clk,
    output wire       O_gmii_tx_clk,
    output wire       O_gmii_tx_clk_90,
    output wire       O_pll_lock,
    output wire       O_reset_ready,
    output wire       O_tx_reset_n,
    output wire       O_rx_reset_n,
    output wire       O_link_status,
    // PHY in-band status is sampled in the recovered RXC domain.  The
    // reference RGMII block reports 2'b10 for 1000 Mb/s.
    output wire [1:0] O_clock_speed,
    output wire       O_duplex_status,
    output wire       O_gigabit_ready
);

    wire W_clk_tx;
    wire W_clk_tx_90;
    wire W_pll_lock;
    wire W_rx_pll_lock;
    wire W_rx_clk_ddr;
    wire W_rxc_raw_clk;
    wire W_rx_clk;
    wire W_rx_pll_clk_c0;
    wire [7:0] W_gmii_rxd;
    wire W_gmii_rx_dv;
    wire W_gmii_rx_er;
    wire W_gmii_crs;
    wire W_gmii_col;
    wire W_pll_reset_release_n;
    wire W_phy_reset_n;
    wire W_rx_pll_reset;
    wire W_rx_reset_async_n;
    wire W_rxc_present;

    reg [1:0] S_tx_rel_q;
    reg [1:0] S_tx_rel_90_q;
    reg [1:0] S_rx_rel_q;
    reg [1:0] S_rx_retime_rel_q;
    reg [7:0] S_gmii_rxd_retime_q;
    reg       S_gmii_rx_dv_retime_q;
    reg       S_gmii_rx_er_retime_q;
    reg [7:0] S_gmii_rxd_fabric_q;
    reg       S_gmii_rx_dv_fabric_q;
    reg       S_gmii_rx_er_fabric_q;
    reg       S_link_status_fabric_q;
    reg [1:0] S_clock_speed_fabric_q;
    reg       S_duplex_status_fabric_q;
    // The PHY reset is driven only from the stable 50 MHz reference and PLL
    // lock.  YT8511 requires at least 10 ms and recommends 100 ms during
    // power-up.  One extra edge covers an asynchronous first partial cycle;
    // this hold deliberately does not depend on recovered RXC.
    localparam [23:0] P_PHY_RESET_HOLD_CYCLES = 24'd5000001;
    reg [23:0] S_phy_reset_hold_count_q;
    reg        S_phy_reset_hold_done_q;

    // RXC presence is observed in the stable 50 MHz reference domain.  The
    // 3-bit source counter is deliberately divided before synchronization:
    // its MSB clock frequency is 125 MHz/8, avoiding a toggle on every RXC
    // edge that could alias the 50 MHz observer.  A recovered clock can stop
    // while the PH1P PLL VCO/output still oscillates, so this heartbeat is
    // also part of the RX reset gate and is not inferred from PLL lock alone.
    localparam [2:0] P_RXC_QUALIFY_CHANGES  = 3'd4;
    localparam [5:0] P_RXC_MISSING_50M_CYCLES = 6'd63;
    reg [2:0] S_rxc_count_q;
    reg [1:0] S_rxc_count_sync_q;
    reg       S_rxc_count_seen_q;
    reg [2:0] S_rxc_qualify_q;
    reg [5:0] S_rxc_missing_count_q;
    reg       S_rxc_present_q;

    assign W_pll_reset_release_n = I_rst_n & W_pll_lock;

    always @(posedge I_clk_50m or negedge W_pll_reset_release_n) begin
        if (!W_pll_reset_release_n) begin
            S_phy_reset_hold_count_q <= 24'd0;
            S_phy_reset_hold_done_q <= 1'b0;
        end else if (!S_phy_reset_hold_done_q) begin
            if (S_phy_reset_hold_count_q >= P_PHY_RESET_HOLD_CYCLES - 1'b1) begin
                S_phy_reset_hold_done_q <= 1'b1;
            end else begin
                S_phy_reset_hold_count_q <= S_phy_reset_hold_count_q + 1'b1;
            end
        end
    end

    // This reset is intentionally independent of RXC.  RXC may be absent
    // while the PHY is held in reset, so making PHY reset depend on rx_rel_q
    // would create a permanent reset loop.
    assign W_phy_reset_n = I_rst_n & W_pll_lock & S_phy_reset_hold_done_q;
    assign O_eth_rst_n = W_phy_reset_n;

    // PHY reset remains independent of RXC.  The RX PLL reset is local to
    // this reference domain: no RXC heartbeat means no attempt is made to
    // use a stale/residual PLL output as a receive clock.
    assign W_rx_pll_reset = !W_pll_reset_release_n || !W_rxc_present;
    assign W_rx_reset_async_n = W_phy_reset_n & W_rxc_present & W_rx_pll_lock;

    // PLLF6F1D0P0_0 is the ETHtest PH1P IP: 50 MHz -> 125 MHz at 0°/90°.
    PLLF6F1D0P0_0 u_eth_pll (
        .refclk   (I_clk_50m),
        .reset    (!I_rst_n),
        .clk0_out (W_clk_tx),
        .clk1_out (W_clk_tx_90),
        .lock     (W_pll_lock)
    );

    // Keep one raw RXC buffer for the heartbeat monitor.  The generated RX
    // PLL takes the pad RXC directly as its refclk; its C1 output is already
    // promoted by the TD-generated wrapper and is the receive capture clock.
    PH1P_LOGIC_BUFG u_eth_rxc_bufg (
        .i (I_eth_rxc),
        .o (W_rxc_raw_clk)
    );

    // TD Catalog-generated PH1P RX PLL: 125 MHz RXC -> 125 MHz C0/C1.
    // C0 is the generated external-feedback clock.  C1 is 4.0909 degrees
    // with 45.45% high time, balancing the two HDIO input capture windows.
    // The lock output is used only by the RX reset gate.
    eth_rx_pll u_eth_rx_pll (
        .refclk   (I_eth_rxc),
        .clk0_out (W_rx_pll_clk_c0),
        .clk1_out (W_rx_clk),
        .lock     (W_rx_pll_lock),
        .reset    (W_rx_pll_reset)
    );

    // Source heartbeat counter.  W_pll_reset_release_n is the independent
    // local reset for both this RXC-domain state and its 50 MHz observer.
    always @(posedge W_rxc_raw_clk or negedge W_pll_reset_release_n) begin
        if (!W_pll_reset_release_n)
            S_rxc_count_q <= 3'b000;
        else
            S_rxc_count_q <= S_rxc_count_q + 1'b1;
    end

    // Two-flop RXC-heartbeat synchronization and bounded qualification in
    // the 50 MHz reference domain.  Four observed changes qualify the input;
    // 64 consecutive reference cycles without a change drop it again.
    always @(posedge I_clk_50m or negedge W_pll_reset_release_n) begin
        if (!W_pll_reset_release_n) begin
            S_rxc_count_sync_q    <= 2'b00;
            S_rxc_count_seen_q    <= 1'b0;
            S_rxc_qualify_q       <= 3'b000;
            S_rxc_missing_count_q <= 6'd0;
            S_rxc_present_q       <= 1'b0;
        end else begin
            S_rxc_count_sync_q <= {S_rxc_count_sync_q[0], S_rxc_count_q[2]};
            if (S_rxc_count_sync_q[1] != S_rxc_count_seen_q) begin
                S_rxc_count_seen_q    <= S_rxc_count_sync_q[1];
                S_rxc_missing_count_q <= 6'd0;
                if (S_rxc_qualify_q < P_RXC_QUALIFY_CHANGES)
                    S_rxc_qualify_q <= S_rxc_qualify_q + 1'b1;
                if (S_rxc_qualify_q >= P_RXC_QUALIFY_CHANGES - 1'b1)
                    S_rxc_present_q <= 1'b1;
            end else if (S_rxc_missing_count_q == P_RXC_MISSING_50M_CYCLES) begin
                S_rxc_missing_count_q <= S_rxc_missing_count_q;
                S_rxc_qualify_q       <= 3'b000;
                S_rxc_present_q       <= 1'b0;
            end else begin
                S_rxc_missing_count_q <= S_rxc_missing_count_q + 1'b1;
            end
        end
    end

    assign W_rxc_present = S_rxc_present_q;

    rgmii_interface #(
        .DEVICE   ("PH1P"),
        .ASYNCRST ("ENABLE"),
        .PIPEMODE ("PIPED")
    ) u_rgmii (
        .tx_reset           (~(S_tx_rel_q[1] & S_tx_rel_90_q[1])),
        .rx_reset           (~(W_rx_reset_async_n & S_rx_rel_q[1])),
        .speed_10_100      (1'b0),
        .rgmii_txd         (O_eth_txd),
        .rgmii_tx_ctl      (O_eth_tx_ctl),
        .rgmii_txc         (O_eth_txc),
        .rgmii_rxd         (I_eth_rxd),
        .rgmii_rx_ctl      (I_eth_rx_ctl),
        .rgmii_rxc         (W_rx_clk),
        // Decode status from the same hand-off byte as the client data.
        .link_status       (),
        .clock_speed       (),
        .duplex_status     (),
        .gmii_txc_en       (1'b1),
        .gmii_txd          (I_gmii_txd),
        .gmii_tx_en        (I_gmii_tx_en),
        .gmii_tx_er        (I_gmii_tx_er),
        .gmii_crs          (W_gmii_crs),
        .gmii_col          (W_gmii_col),
        .gmii_rxd         (W_gmii_rxd),
        .gmii_rx_dv        (W_gmii_rx_dv),
        .gmii_rx_er        (W_gmii_rx_er),
        .tx_clk            (W_clk_tx),
        .tx_clk_90         (W_clk_tx_90),
        // 2026-10-18 RGMII 发送时钟修复（与 EG4D top_eg4d_slave.v 同一处）：
        // TXC 由 rgmii_interface 的 ODDR 用 (d0,d1) 交替产生，两个数据输入必须
        // 一个 1、一个 0。官方参考 tx_clk_en_gen.v 的 1Gbps 分支：
        // rgmii_txc_en=0、rgmii_txc_en_shift=1（ODDR d1=0 / d0=1）。
        // 原来两者都接 1'b1 -> O_eth_txc 恒高 -> 无发送时钟 -> 该板 TX 全死。
        .rgmii_txc_en      (1'b0),
        .rgmii_txc_en_shift(1'b1),
        .rx_rgmii_clk      (W_rx_clk_ddr)
    );

    // Reset deassertion is synchronized independently in the 0° TX, 90° TX
    // and buffered RX domains.  TX reset is held until both TX domains have
    // released, avoiding a half-cycle ODDR enable during PLL startup.
    always @(posedge W_clk_tx or negedge W_phy_reset_n) begin
        if (!W_phy_reset_n)
            S_tx_rel_q <= 2'b00;
        else
            S_tx_rel_q <= {S_tx_rel_q[0], 1'b1};
    end

    always @(posedge W_clk_tx_90 or negedge W_phy_reset_n) begin
        if (!W_phy_reset_n)
            S_tx_rel_90_q <= 2'b00;
        else
            S_tx_rel_90_q <= {S_tx_rel_90_q[0], 1'b1};
    end

    // Release DDR capture in its own buffered RX clock domain.  The later
    // hand-off retains its existing two-edge reset release.
    always @(posedge W_rx_clk_ddr or negedge W_rx_reset_async_n) begin
        if (!W_rx_reset_async_n)
            S_rx_rel_q <= 2'b00;
        else
            S_rx_rel_q <= {S_rx_rel_q[0], 1'b1};
    end

    // The PH1P HDIO IDDR with PIPEMODE="PIPED" presents q0/q1 as one complete
    // GMII byte on a rising RX PLL C1 edge.  Capture that byte and DV/ER
    // on the falling edge of the same clock.  All exposed RX signals
    // are therefore launched from the same half-cycle boundary.  A second
    // register at the next buffered rising edge drives the packet link,
    // preserving a full RXC period for its CRC/control logic.  In-band status
    // is decoded from this retimed GMII value in the fabric hand-off below;
    // the unused vendor status outputs remain unconnected above.
    always @(negedge W_rx_clk or negedge W_rx_reset_async_n) begin
        if (!W_rx_reset_async_n) begin
            S_rx_retime_rel_q     <= 2'b00;
            S_gmii_rxd_retime_q   <= 8'h00;
            S_gmii_rx_dv_retime_q <= 1'b0;
            S_gmii_rx_er_retime_q <= 1'b0;
        end else begin
            S_rx_retime_rel_q <= {S_rx_retime_rel_q[0], S_rx_rel_q[1]};
            if (!S_rx_retime_rel_q[1]) begin
                S_gmii_rxd_retime_q    <= 8'h00;
                S_gmii_rx_dv_retime_q  <= 1'b0;
                S_gmii_rx_er_retime_q  <= 1'b0;
            end else begin
                S_gmii_rxd_retime_q    <= W_gmii_rxd;
                S_gmii_rx_dv_retime_q  <= W_gmii_rx_dv;
                S_gmii_rx_er_retime_q  <= W_gmii_rx_er;
            end
        end
    end

    // Fabric hand-off register after the HDIO capture.  This stage is deliberately separate from
    // the negedge capture: the packet-link GMII inputs are all launched on
    // the same buffered rising edge and therefore retain a full-cycle setup
    // budget.  The negedge stage gives the DDR byte a half cycle to settle.
    always @(posedge W_rx_clk or negedge W_rx_reset_async_n) begin
        if (!W_rx_reset_async_n) begin
            S_gmii_rxd_fabric_q       <= 8'h00;
            S_gmii_rx_dv_fabric_q     <= 1'b0;
            S_gmii_rx_er_fabric_q     <= 1'b0;
            S_link_status_fabric_q    <= 1'b0;
            S_clock_speed_fabric_q    <= 2'b00;
            S_duplex_status_fabric_q  <= 1'b0;
        end else if (!S_rx_retime_rel_q[1]) begin
            S_gmii_rxd_fabric_q       <= 8'h00;
            S_gmii_rx_dv_fabric_q     <= 1'b0;
            S_gmii_rx_er_fabric_q     <= 1'b0;
            S_link_status_fabric_q    <= 1'b0;
            S_clock_speed_fabric_q    <= 2'b00;
            S_duplex_status_fabric_q  <= 1'b0;
        end else begin
            S_gmii_rxd_fabric_q       <= S_gmii_rxd_retime_q;
            S_gmii_rx_dv_fabric_q     <= S_gmii_rx_dv_retime_q;
            S_gmii_rx_er_fabric_q     <= S_gmii_rx_er_retime_q;
            // Match rgmii_interface.v's in-band contract: only an idle GMII
            // cycle updates status, while data or RX error holds the prior
            // value.  Decode the previous fabric byte (the nonblocking
            // value from the prior hand-off) to retain the vendor block's
            // one-RXC-cycle status latency after the GMII byte, without
            // recreating the raw IDDR-to-status-register hold path.
            if (!S_gmii_rx_dv_fabric_q && !S_gmii_rx_er_fabric_q) begin
                S_link_status_fabric_q   <= S_gmii_rxd_fabric_q[0];
                S_clock_speed_fabric_q   <= S_gmii_rxd_fabric_q[2:1];
                S_duplex_status_fabric_q <= S_gmii_rxd_fabric_q[3];
            end
        end
    end

    assign O_gmii_rx_clk   = W_rx_clk;
    assign O_gmii_rxd      = S_gmii_rxd_fabric_q;
    assign O_gmii_rx_dv    = S_gmii_rx_dv_fabric_q;
    assign O_gmii_rx_er    = S_gmii_rx_er_fabric_q;
    assign O_gmii_tx_clk   = W_clk_tx;
    assign O_gmii_tx_clk_90 = W_clk_tx_90;
    assign O_pll_lock      = W_pll_lock;
    // The common TX client reset is released only by the TX-clock-domain
    // synchronizer.  The 90-degree synchronizer remains part of the RGMII
    // primitive reset and of the aggregate ready indication below.
    assign O_tx_reset_n    = W_phy_reset_n & S_tx_rel_q[1];
    assign O_rx_reset_n    = W_rx_reset_async_n & S_rx_rel_q[1];
    assign O_reset_ready   = O_tx_reset_n & S_tx_rel_90_q[1] & O_rx_reset_n;
    assign O_link_status   = S_link_status_fabric_q;
    assign O_clock_speed   = S_clock_speed_fabric_q;
    assign O_duplex_status = S_duplex_status_fabric_q;
    assign O_gigabit_ready = S_link_status_fabric_q &&
                             (S_clock_speed_fabric_q == 2'b10) &&
                             S_duplex_status_fabric_q && O_reset_ready;

    // MDIO management remains outside this physical shell until the board
    // communication contract assigns a management owner.
    assign O_eth_mdc  = 1'b0;
    assign IO_eth_mdio = 1'bz;

endmodule
