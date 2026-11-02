`timescale 1ns/1ps
// ============================================================================
// top_eg4d_slave
//  所属域 : top
//  时钟域 : 25 MHz compute, 75 MHz ink/mode, 125 MHz TX, PHY RX
//  复位   : legacy roots preserved; INK common asynchronous assertion
//  参数   : fixed board MAC addresses and EtherTypes
//  验证状态 : integration candidate, simulation and physical signoff pending
//
// EG4D20EG176 board shell for the telemetry/key endpoint and remote line modes.
// The three raw-Ethernet client slots (0x88B5 telemetry, 0x88B6 line, 0x88B7
// reserved) share only the GMII TX pins through ctl_gmii_tx_scheduler3; each
// active client retains an independent RX mailbox.  PLL_0 is the frozen 125 MHz
// RGMII clock source and PLL_75 is an independent, explicitly generated 75 MHz
// mode/application clock.  The PHY RXC is compensated by the native RX PLL and
// promoted through EG_LOGIC_BUFG before it clocks the IDDRs and RX-domain clients.
//
// The common transport owns Ethernet preamble/FCS/header and all packet RAM.
// This top owns clock/reset/PHY conversion, telemetry/key control, audio, and
// the line-mode adapter/filter path. The three-wire 74HC595 interface displays
// received distance on the left four digits and speed on the right four digits.
// 固定 1 Gb/s full-duplex：RGMII speed_10_100=0。LED1..4 保留原有按键/原始
// 活动窗口；LED4 额外并入周期 FIFO overflow 诊断。
// ============================================================================
module top_eg4d_slave #(
    parameter integer P_CORE_BUSINESS = 1
) (
    input  wire        I_clk_25m,
    output wire [3:0]  O_led,
    output wire        O_seg_sclk,
    output wire        O_seg_latch,
    output wire        O_seg_data,
    // MLK-S200 4x4 key matrix (ADC-verified): group A 112/109/104/91,
    // group B 119/118/115/114.  Both groups are bidirectional; the scanner
    // drives one group and senses the other, alternating every scan.
    inout  wire [3:0]  IO_key_row,
    inout  wire [3:0]  IO_key_col,

    output wire        O_eth_txc,
    output wire [3:0]  O_eth_txd,
    output wire        O_eth_tx_ctl,
    input  wire        I_eth_rxc,
    input  wire [3:0]  I_eth_rxd,
    input  wire        I_eth_rx_ctl,
    output wire        O_eth_mdc,
    inout  wire        IO_eth_mdio,
    output wire        O_eth_rst_n,

    // ---- TF card (SPI mode) + MAX98357A I2S audio (KEY5..16 trigger) ----
    output wire        O_tf_clk,
    output wire        O_tf_cs_n,
    output wire        O_tf_mosi,
    input  wire        I_tf_miso,
    output wire        O_aud_bclk,
    output wire        O_aud_lrclk,
    output wire        O_aud_din,
    output wire        O_aud_sd,
    output wire        O_aud_gain
);

    localparam [47:0] P_LOCAL_MAC = 48'h02_00_00_65_43_21;
    localparam [47:0] P_PEER_MAC  = 48'h02_00_00_12_34_56;
    localparam [15:0] P_OCR_ETHERTYPE = 16'h88b5;
    localparam [15:0] P_MODE_ETHERTYPE = 16'h88b6;

    wire W_clk_tx;
    wire W_clk_tx_90;
    wire W_pll_lock;
    wire W_clk_mode;
    wire W_clk_mode_90;
    wire W_mode_pll_lock;
    wire W_clk_rx;
    wire W_rx_pll_lock;
    wire W_rx_pll_clock;
    wire W_eth_rxc_g;
    wire W_mdio_done;
    wire W_mdio_error;

    // YT8531 RESET_N is held low for 20 ms after the local 125 MHz PLL locks.
    // MDIO qualification and the native RX PLL are also reset gates for all
    // packet clients; this prevents an RX mailbox from starting on an
    // unqualified recovered clock.
    localparam [18:0] P_PHY_RESET_HOLD_CYCLES = 19'd500001;
    reg [18:0] S_phy_reset_hold_count;
    wire       W_phy_reset_hold_done =
                   (S_phy_reset_hold_count == P_PHY_RESET_HOLD_CYCLES);
    wire       W_phy_reset_n = W_pll_lock & W_phy_reset_hold_done;
    wire       W_phy_config_ready = W_mdio_done & ~W_mdio_error;

    // Explicit 25 MHz POR.  The board has no required user-reset input.
    reg [15:0] S_por_25;
    wire       W_por_done = &S_por_25;
    // Keep the frozen 25/125 MHz telemetry/key path alive only after PHY
    // reset, MDIO delay configuration, and RX PLL lock are all qualified.
    // Only the mode application pipeline additionally depends on W_mode_pll_lock.
    wire       W_rst_base_async =
                   ~(W_por_done & W_phy_reset_n & W_phy_config_ready & W_rx_pll_lock);
    wire       W_rst_mode_async =
                   ~(W_por_done & W_phy_reset_n & W_phy_config_ready &
                     W_rx_pll_lock & W_mode_pll_lock);
    initial S_por_25 = 16'd0;
    initial S_phy_reset_hold_count = 19'd0;

    always @(posedge I_clk_25m) begin
        if (!W_pll_lock)
            S_por_25 <= 16'd0;
        else if (!W_por_done)
            S_por_25 <= S_por_25 + 16'd1;

        if (!W_pll_lock)
            S_phy_reset_hold_count <= 19'd0;
        else if (!W_phy_reset_hold_done)
            S_phy_reset_hold_count <= S_phy_reset_hold_count + 19'd1;
    end

    PLL_0 u_pll (
        .refclk   (I_clk_25m),
        .reset    (1'b0),
        .extlock  (W_pll_lock),
        .clk0_out (W_clk_tx),
        .clk1_out (W_clk_tx_90)
    );

    // This is a real EG_PHY_PLL output, not a fabric divider.  Keeping the
    // line engine in its own clock domain avoids asking the 125 MHz GMII
    // clock to meet the 75 MHz 1280-pixel pipeline timing and lets TD report a
    // generated 75 MHz clock in the Clock Summary.
    PLL_75 u_mode_pll (
        .refclk   (I_clk_25m),
        .reset    (1'b0),
        .extlock  (W_mode_pll_lock),
        .clk0_out (W_clk_mode),
        .clk1_out (W_clk_mode_90)
    );

    // The PHY-recognized RXC is first compensated by the native EG4 RX PLL,
    // then promoted on the global clock network before feeding the DDR inputs.
    eg4d_eth_rx_pll u_rx_pll (
        .refclk   (I_eth_rxc),
        .reset    (~W_phy_reset_n),
        .extlock  (W_rx_pll_lock),
        .clk0_out (),
        .clk1_out (W_rx_pll_clock)
    );

    EG_LOGIC_BUFG u_rxc_bufg (
        .i (W_rx_pll_clock),
        .o (W_eth_rxc_g)
    );

    // Synchronous reset release in each clock domain.  Assertion is common
    // and asynchronous; deassertion is local to the receiving clock.
    reg [2:0] S_app_rst;
    reg [2:0] S_tx_rst;
    reg [2:0] S_tx90_rst;
    reg [2:0] S_rx_rst;
    reg [2:0] S_mode_rst;
    reg [2:0] S_mode_tx_rst;
    reg [2:0] S_mode_rx_rst;
    initial begin
        S_app_rst  = 3'd0;
        S_tx_rst   = 3'd0;
        S_tx90_rst = 3'd0;
        S_rx_rst   = 3'd0;
        S_mode_rst = 3'd0;
        S_mode_tx_rst = 3'd0;
        S_mode_rx_rst = 3'd0;
    end

    always @(posedge I_clk_25m or posedge W_rst_base_async) begin
        if (W_rst_base_async)
            S_app_rst <= 3'd0;
        else
            S_app_rst <= {S_app_rst[1:0], 1'b1};
    end

    always @(posedge W_clk_tx or posedge W_rst_base_async) begin
        if (W_rst_base_async)
            S_tx_rst <= 3'd0;
        else
            S_tx_rst <= {S_tx_rst[1:0], 1'b1};
    end

    always @(posedge W_clk_tx_90 or posedge W_rst_base_async) begin
        if (W_rst_base_async)
            S_tx90_rst <= 3'd0;
        else
            S_tx90_rst <= {S_tx90_rst[1:0], 1'b1};
    end

    always @(posedge W_clk_rx or posedge W_rst_base_async) begin
        if (W_rst_base_async)
            S_rx_rst <= 3'd0;
        else
            S_rx_rst <= {S_rx_rst[1:0], 1'b1};
    end

    always @(posedge W_clk_mode or posedge W_rst_mode_async) begin
        if (W_rst_mode_async)
            S_mode_rst <= 3'd0;
        else
            S_mode_rst <= {S_mode_rst[1:0], 1'b1};
    end

    // The mode PLL lock is asynchronous to both the GMII TX and PHY RX
    // clocks.  Assert the mode endpoint reset immediately on loss of lock, but
    // release it through a local three-flop chain in each endpoint domain.
    // This keeps a lock return from making the stream TX/RX FSMs observe a
    // partially released reset, while the OCR/PHY base reset remains alive.
    wire W_rst_mode_tx_async = ~(S_tx_rst[2] & W_mode_pll_lock);
    wire W_rst_mode_rx_async = ~(S_rx_rst[2] & W_mode_pll_lock);
    always @(posedge W_clk_tx or posedge W_rst_mode_tx_async) begin
        if (W_rst_mode_tx_async)
            S_mode_tx_rst <= 3'd0;
        else
            S_mode_tx_rst <= {S_mode_tx_rst[1:0], 1'b1};
    end
    always @(posedge W_clk_rx or posedge W_rst_mode_rx_async) begin
        if (W_rst_mode_rx_async)
            S_mode_rx_rst <= 3'd0;
        else
            S_mode_rx_rst <= {S_mode_rx_rst[1:0], 1'b1};
    end

    wire W_rst_app_n = S_app_rst[2];
    wire W_rst_tx_n  = S_tx_rst[2];
    wire W_rst_tx90_n= S_tx90_rst[2];
    wire W_rst_rx_n  = S_rx_rst[2];
    wire W_rst_mode_n= S_mode_rst[2];
    // The 0x88B6 endpoint is itself part of the mode partition.  Reset its
    // GMII/RX FSMs on a mode-PLL loss, while the OCR endpoint and shared
    // scheduler keep using the qualified 125 MHz base reset.  The scheduler
    // sees a synthetic mode-done pulse during the loss so an interrupted mode
    // frame cannot hold the shared owner forever.
    wire W_rst_mode_tx_n = S_mode_tx_rst[2];
    wire W_rst_mode_rx_n = S_mode_rx_rst[2];
    // rgmii_interface has one TX reset input feeding both TX DDR phases.  It
    // is released only after both local phase-domain synchronizers are high.
    wire W_rst_rgmii_tx_n = W_rst_tx_n & W_rst_tx90_n;

    // Per-client GMII TX buses are muxed only by the scheduler below.  There
    // is deliberately no combinational OR of two MAC outputs on the PHY bus.
    wire [7:0] W_gmii_txd;
    wire       W_gmii_tx_en;
    wire       W_gmii_tx_er;
    wire [7:0] W_ocr_gmii_txd;
    wire       W_ocr_gmii_tx_en;
    wire       W_ocr_gmii_tx_er;
    wire [7:0] W_mode_gmii_txd;
    wire       W_mode_gmii_tx_en;
    wire       W_mode_gmii_tx_er;
    wire [7:0] W_gmii_rxd;
    wire       W_gmii_rx_dv;
    wire       W_gmii_rx_er;
    wire       W_link_status;
    wire [1:0] W_clock_speed;
    wire       W_duplex_status;
    wire       W_gmii_crs;
    wire       W_gmii_col;
    wire       W_compute_busy;
    wire       W_compute_response_valid;
    wire [3:0] W_ink_diag_status_app;

    wire        W_ocr_tx_request;
    wire        W_ocr_tx_grant;
    wire        W_ocr_tx_done_tx;
    wire        W_mode_tx_request;
    wire        W_mode_tx_grant;
    wire        W_mode_tx_done_tx;

    // 0x88B5 telemetry/key endpoint, app clock = 25 MHz.
    wire        W_ocr_tx_write_en;
    wire [9:0]  W_ocr_tx_write_addr;
    wire [7:0]  W_ocr_tx_write_data;
    wire        W_ocr_tx_commit;
    wire [10:0] W_ocr_tx_length;
    wire        W_ocr_tx_busy;
    wire        W_ocr_tx_done;
    wire        W_ocr_rx_valid;
    wire [10:0] W_ocr_rx_length;
    wire [9:0]  W_ocr_rx_read_addr;
    wire [7:0]  W_ocr_rx_read_data;
    wire        W_ocr_rx_release;
    wire [31:0] W_ocr_bad_frame_count;
    wire [31:0] W_ocr_busy_drop_count;

    // 0x88B5 telemetry snapshot retained in the 25 MHz application domain.
    // ctl_telemetry_rx owns the one-cycle mailbox read/release contract.
    wire [9:0]  W_telemetry_rx_read_addr;
    wire        W_telemetry_rx_release;
    wire [15:0] W_telemetry_ag /*synthesis keep*/;
    wire [15:0] W_telemetry_ae /*synthesis keep*/;
    wire [19:0] W_telemetry_speed_mmps /*synthesis keep*/;
    wire [19:0] W_telemetry_dist_mm /*synthesis keep*/;
    wire        W_telemetry_metrics_valid /*synthesis keep*/;
    wire [7:0]  W_telemetry_metrics_state /*synthesis keep*/;
    wire [31:0] W_telemetry_sample_seq /*synthesis keep*/;
    wire        W_telemetry_sample_valid /*synthesis keep*/;
    wire        W_telemetry_update /*synthesis keep*/;
    wire        W_telemetry_cycle_active;
    wire        W_telemetry_cycle_reset_ack;
    wire        W_cycle_reset_request;
    wire        W_cycle_active;
    wire [3:0]  W_cycle_index;
    wire        W_cycle_auto_busy;
    wire [4:0]  W_cycle_osd_key;
    wire        W_cycle_queue_overflow;

    ctl_telemetry_rx u_telemetry_rx (
        .clk                  (I_clk_25m),
        .rst_n                (W_rst_app_n),
        .I_rx_valid           (W_ocr_rx_valid),
        .I_rx_length          (W_ocr_rx_length),
        .O_rx_addr            (W_telemetry_rx_read_addr),
        .I_rx_data            (W_ocr_rx_read_data),
        .O_rx_release         (W_telemetry_rx_release),
        .O_ag                 (W_telemetry_ag),
        .O_ae                 (W_telemetry_ae),
        .O_speed_mmps         (W_telemetry_speed_mmps),
        .O_dist_mm            (W_telemetry_dist_mm),
        .O_metrics_valid      (W_telemetry_metrics_valid),
        .O_metrics_state      (W_telemetry_metrics_state),
        .O_sample_seq         (W_telemetry_sample_seq),
        .O_sample_valid       (W_telemetry_sample_valid),
        .O_update             (W_telemetry_update),
        .O_cycle_active       (W_telemetry_cycle_active),
        .O_cycle_reset_ack    (W_telemetry_cycle_reset_ack)
    );

    // Match the PH1P OSD: state 1 is measured, state 2 is a held measurement.
    // Both the decoder and display run in the same 25 MHz application domain.
    wire W_seg_metrics_valid = W_telemetry_metrics_valid &&
        ((W_telemetry_metrics_state == 8'd1) ||
         (W_telemetry_metrics_state == 8'd2));

    dsp_telemetry_seg u_telemetry_seg (
        .I_clk_25m       (I_clk_25m),
        .I_rst_n         (W_rst_app_n),
        .I_update        (W_telemetry_update),
        .I_metrics_valid (W_seg_metrics_valid),
        .I_dist_mm       (W_telemetry_dist_mm),
        .I_speed_mmps    (W_telemetry_speed_mmps),
        .O_seg_sclk      (O_seg_sclk),
        .O_seg_latch     (O_seg_latch),
        .O_seg_data      (O_seg_data)
    );

    // Explicit parity anchor for retained telemetry snapshot state.  This is
    // observation-only; LED4 qualifies its key-TX window with snapshot parity
    // the cycle FIFO overflow diagnostic below.
    (* keep = "true" *) wire W_telemetry_snapshot_parity = ^{
        W_telemetry_ag, W_telemetry_ae, W_telemetry_speed_mmps,
        W_telemetry_dist_mm, W_telemetry_metrics_valid,
        W_telemetry_metrics_state, W_telemetry_sample_seq,
        W_telemetry_sample_valid, W_telemetry_cycle_active
    };

    // W3: 4x4 key matrix scan + 0x88B5 key-state packet producer.
    wire [4:0] W_key_code;
    wire       W_key_valid;
    wire       W_key_changed;
    wire [1:0] W_key_raw_act;
    wire       W_key_tx_done;

    // During an automatic cycle, the key-state packet carries the OSD key and
    // flags instead of a raw matrix key.  Outside the cycle it remains the
    // original manual key producer. KEY4 is a local speaker switch only.
    wire        W_key_tx_auto = W_cycle_active || W_cycle_auto_busy;
    wire [4:0]  W_key_tx_code = W_key_tx_auto ? W_cycle_osd_key :
                                  ((W_key_code == 5'd4) ? 5'd0 : W_key_code);
    wire        W_key_tx_valid = W_key_tx_auto ?
                                  (W_cycle_osd_key != 5'd0) :
                                  (W_key_valid && (W_key_code != 5'd4));

    // 0x88B6 line endpoint, app/pixel clock = 75 MHz.
    wire        W_mode_tx_ready;
    wire        W_mode_tx_wbuf;
    wire        W_mode_tx_we;
    wire [10:0] W_mode_tx_addr;
    wire [7:0]  W_mode_tx_wdata;
    wire        W_mode_tx_commit;
    wire        W_mode_rx_valid;
    wire        W_mode_rx_rbuf;
    wire [15:0] W_mode_rx_len;
    wire [10:0] W_mode_rx_addr;
    wire [7:0]  W_mode_rx_rdata;
    wire        W_mode_rx_release;
    wire [31:0] W_mode_bad_frame_count;
    wire [31:0] W_mode_busy_drop_count;

    // Raster stream between the line adapter and the second-pass filter.
    wire        W_fil_vsync;
    wire        W_fil_hsync;
    wire        W_fil_de;
    wire        W_fil_user;
    wire        W_fil_last;
    wire [7:0]  W_fil_y;
    wire [4:0]  W_fil_mode;
    wire [4:0]  W_mode_out_mode;
    wire        W_mode_out_vsync;
    wire        W_mode_out_hsync;
    wire        W_mode_out_de;
    wire        W_mode_out_user;
    wire        W_mode_out_last;
    wire [7:0]  W_mode_out_y;
    wire        W_mode_out_edge;
    wire        W_mode_out_mask;
    wire        W_mode_window_valid;

    assign W_clk_rx = W_eth_rxc_g;

    rgmii_interface #(
        .DEVICE   ("EG4"),
        .ASYNCRST ("ENABLE"),
        .PIPEMODE ("PIPED")
    ) u_rgmii (
        .tx_reset           (~W_rst_rgmii_tx_n),
        .rx_reset           (~W_rst_rx_n),
        .speed_10_100      (1'b0),
        .rgmii_txd         (O_eth_txd),
        .rgmii_tx_ctl      (O_eth_tx_ctl),
        .rgmii_txc         (O_eth_txc),
        .rgmii_rxd         (I_eth_rxd),
        .rgmii_rx_ctl      (I_eth_rx_ctl),
        .rgmii_rxc         (W_eth_rxc_g),
        .link_status       (W_link_status),
        .clock_speed       (W_clock_speed),
        .duplex_status     (W_duplex_status),
        .gmii_txc_en       (1'b1),
        .gmii_txd          (W_gmii_txd),
        .gmii_tx_en        (W_gmii_tx_en),
        .gmii_tx_er        (W_gmii_tx_er),
        .gmii_crs          (W_gmii_crs),
        .gmii_col          (W_gmii_col),
        .gmii_rxd          (W_gmii_rxd),
        .gmii_rx_dv        (W_gmii_rx_dv),
        .gmii_rx_er        (W_gmii_rx_er),
        .tx_clk            (W_clk_tx),
        .tx_clk_90         (W_clk_tx_90),
        // ODDR d0/d1 are the two TXC phases.  d0 must be low and d1 high;
        // driving both high makes O_eth_txc a constant level.
        .rgmii_txc_en      (1'b0),
        .rgmii_txc_en_shift(1'b1),
        .rx_rgmii_clk      ()
    );

    // ---------------------------------------------------------------------
    // W2: INK OCR recognition cascade removed.  The C13/C14 char recognition
    // (char_shape_feature_engine + char_match) and its sampler/compute session
    // are gone; tie off the former INK TX-arbitration and diagnostic signals so
    // the 3-client scheduler and the fault LED keep their contract.
    // ---------------------------------------------------------------------
    wire W_ink_rst_tx_n = 1'b1;
    wire W_ink_tx_grant;
    wire W_ink_tx_request = 1'b0;
    wire W_ink_tx_done = 1'b1;
    wire [7:0] W_ink_gmii_txd = 8'd0;
    wire W_ink_gmii_tx_en = 1'b0;
    wire W_ink_gmii_tx_er = 1'b0;
    assign W_compute_busy = 1'b0;
    assign W_compute_response_valid = 1'b0;
    assign W_ink_diag_status_app = 4'd0;

    // ---------------------------------------------------------------------
    // W3: 0x88B5 key-state endpoint (25 MHz application domain).  The same
    // ctl_lpr_packet_link carries key TX to PH1P and telemetry RX back here.
    // ---------------------------------------------------------------------
    ctl_lpr_packet_link #(
        .P_LOCAL_MAC (P_LOCAL_MAC),
        .P_PEER_MAC  (P_PEER_MAC),
        .P_ETHERTYPE (P_OCR_ETHERTYPE),
        .P_TX_ARB_EN (1)
    ) u_ocr_packet_link (
        .I_clk_app      (I_clk_25m),
        .I_rst_app_n    (W_rst_app_n),
        .I_clk_tx       (W_clk_tx),
        .I_rst_tx_n     (W_rst_tx_n),
        .I_clk_rx       (W_clk_rx),
        .I_rst_rx_n     (W_rst_rx_n),
        .I_tx_write_en  (W_ocr_tx_write_en),
        .I_tx_write_addr(W_ocr_tx_write_addr),
        .I_tx_write_data(W_ocr_tx_write_data),
        .I_tx_commit    (W_ocr_tx_commit),
        .I_tx_length    (W_ocr_tx_length),
        .O_tx_busy      (W_ocr_tx_busy),
        .O_tx_done      (W_ocr_tx_done),
        .I_tx_grant     (W_ocr_tx_grant),
        .O_tx_request   (W_ocr_tx_request),
        .O_tx_done_tx   (W_ocr_tx_done_tx),
        .O_rx_valid     (W_ocr_rx_valid),
        .O_rx_length    (W_ocr_rx_length),
        .I_rx_read_addr (W_ocr_rx_read_addr),
        .O_rx_read_data (W_ocr_rx_read_data),
        .I_rx_release   (W_ocr_rx_release),
        .O_gmii_txd     (W_ocr_gmii_txd),
        .O_gmii_tx_en   (W_ocr_gmii_tx_en),
        .O_gmii_tx_er   (W_ocr_gmii_tx_er),
        .I_gmii_rxd     (W_gmii_rxd),
        .I_gmii_rx_dv   (W_gmii_rx_dv),
        .I_gmii_rx_er   (W_gmii_rx_er),
        .O_tx_busy_drop_count(),
        .O_tx_invalid_drop_count(),
        .O_rx_bad_frame_count(W_ocr_bad_frame_count),
        .O_rx_busy_drop_count(W_ocr_busy_drop_count),
        .O_rx_oversize_drop_count(),
        .O_bad_frame_count(),
        .O_busy_drop_count()
    );

    // The 0x88B5 RX mailbox is a live telemetry consumer.  The key-state TX
    // packet and the telemetry RX packet share the same bidirectional link;
    // each direction has its own mailbox contract.
    assign W_ocr_rx_read_addr = W_telemetry_rx_read_addr;
    assign W_ocr_rx_release   = W_telemetry_rx_release;

    // ------------------------------------------------------------------
    // 4x4 key matrix scanner and 0x88B5 key-state packet producer
    // ------------------------------------------------------------------
    ctl_key_matrix_scan u_key_scan (
        .clk          (I_clk_25m        ),
        .rst_n        (W_rst_app_n      ),
        .IO_key_row   (IO_key_row       ),
        .IO_key_col   (IO_key_col       ),
        .O_key_code   (W_key_code       ),
        .O_key_valid  (W_key_valid      ),
        .O_key_changed(W_key_changed    ),
        .O_raw_act    (W_key_raw_act    )
    );

    ctl_key_state_tx u_key_tx (
        .clk        (I_clk_25m           ),
        .rst_n      (W_rst_app_n         ),
        .I_key_code (W_key_tx_code       ),
        .I_key_valid(W_key_tx_valid      ),
        .I_key_auto (W_key_tx_auto       ),
        .I_cycle_reset_request(W_cycle_reset_request),
        .I_tx_busy  (W_ocr_tx_busy       ),
        .O_tx_done  (W_key_tx_done       ),
        .O_wr_addr  (W_ocr_tx_write_addr ),
        .O_wr_data  (W_ocr_tx_write_data ),
        .O_wr_en    (W_ocr_tx_write_en   ),
        .O_commit   (W_ocr_tx_commit     ),
        .O_length   (W_ocr_tx_length     )
    );
    // ---------------------------------------------------------------------
    // 0x88B6 line-mode endpoint (75 MHz application/pixel domain)
    // ---------------------------------------------------------------------
    generate
    if (P_CORE_BUSINESS == 0) begin : g_mode_client_full
        ctl_stream_link #(
            .P_LOCAL_MAC(P_LOCAL_MAC),
            .P_PEER_MAC (P_PEER_MAC),
            .P_ETHERTYPE(P_MODE_ETHERTYPE),
            .P_TX_ARB_EN(1)
        ) u_mode_stream_link (
            .I_clk_tx     (W_clk_tx),
            .I_rst_tx_n   (W_rst_mode_tx_n),
            .O_gmii_txd   (W_mode_gmii_txd),
            .O_gmii_tx_en (W_mode_gmii_tx_en),
            .O_gmii_tx_er (W_mode_gmii_tx_er),
            .I_tx_grant   (W_mode_tx_grant),
            .O_tx_request (W_mode_tx_request),
            .O_tx_done_tx (W_mode_tx_done_tx),
            .I_clk_rx     (W_clk_rx),
            .I_rst_rx_n   (W_rst_mode_rx_n),
            .I_gmii_rxd   (W_gmii_rxd),
            .I_gmii_rx_dv (W_gmii_rx_dv),
            .I_gmii_rx_er (W_gmii_rx_er),
            .I_clk_app    (W_clk_mode),
            .I_rst_app_n  (W_rst_mode_n),
            .O_tx_ready   (W_mode_tx_ready),
            .O_tx_wbuf    (W_mode_tx_wbuf),
            .I_tx_we      (W_mode_tx_we),
            .I_tx_addr    (W_mode_tx_addr),
            .I_tx_wdata   (W_mode_tx_wdata),
            .I_tx_commit  (W_mode_tx_commit),
            .O_rx_valid   (W_mode_rx_valid),
            .O_rx_rbuf    (W_mode_rx_rbuf),
            .O_rx_len     (W_mode_rx_len),
            .I_rx_addr    (W_mode_rx_addr),
            .O_rx_rdata   (W_mode_rx_rdata),
            .I_rx_release (W_mode_rx_release),
            .O_tx_frame_count(),
            .O_rx_frame_count(),
            .O_rx_bad_count(W_mode_bad_frame_count),
            .O_rx_busy_drop_count(W_mode_busy_drop_count),
            .O_rx_last_line(),
            .O_bad_frame_count(),
            .O_busy_drop_count()
        );

        eg4d_line_adapter #(
            .IMG_W(1280), .P_HDR_BYTES(16), .P_APP_MAGIC(16'h01A5)
        ) u_line_adapter (
            .I_clk        (W_clk_mode),
            .I_rst_n      (W_rst_mode_n),
            .I_rx_valid   (W_mode_rx_valid),
            .I_rx_len     (W_mode_rx_len),
            .O_rx_addr    (W_mode_rx_addr),
            .I_rx_rdata   (W_mode_rx_rdata),
            .O_rx_release (W_mode_rx_release),
            .I_tx_ready   (W_mode_tx_ready),
            // ctl_stream_link chooses the free ping-pong buffer internally;
            // O_tx_wbuf is retained on the adapter API for legacy masters.
            .O_tx_wbuf    (),
            .O_tx_we      (W_mode_tx_we),
            .O_tx_addr    (W_mode_tx_addr),
            .O_tx_wdata   (W_mode_tx_wdata),
            .O_tx_commit  (W_mode_tx_commit),
            .O_fil_vsync  (W_fil_vsync),
            .O_fil_hsync  (W_fil_hsync),
            .O_fil_de     (W_fil_de),
            .O_fil_user   (W_fil_user),
            .O_fil_last   (W_fil_last),
            .O_fil_y      (W_fil_y),
            .O_fil_mode   (W_fil_mode),
            .I_fil_vsync  (W_mode_out_vsync),
            .I_fil_hsync  (W_mode_out_hsync),
            .I_fil_de     (W_mode_out_de),
            .I_fil_user   (W_mode_out_user),
            .I_fil_last   (W_mode_out_last),
            .I_fil_y      (W_mode_out_y),
            .I_fil_edge   (W_mode_out_edge),
            .I_fil_window_valid(W_mode_window_valid)
        );

        eg4d_mode_pipeline #(
            .IMG_W(1280), .EDGE_RGB_TH(8'd80), .CARTOON_OFF(8'd0)
        ) u_mode_pipeline (
            .I_clk        (W_clk_mode),
            .I_rst_n      (W_rst_mode_n),
            .I_mode       (W_fil_mode),
            .I_bin_th     (8'd128),
            .I_cartoon_th (8'd80),
            .I_vsync      (W_fil_vsync),
            .I_hsync      (W_fil_hsync),
            .I_de         (W_fil_de),
            .I_user       (W_fil_user),
            .I_last       (W_fil_last),
            .I_y          (W_fil_y),
            .O_mode       (W_mode_out_mode),
            .O_vsync      (W_mode_out_vsync),
            .O_hsync      (W_mode_out_hsync),
            .O_de         (W_mode_out_de),
            .O_user       (W_mode_out_user),
            .O_last       (W_mode_out_last),
            .O_y          (W_mode_out_y),
            .O_edge       (W_mode_out_edge),
            .O_mask       (W_mode_out_mask),
            .O_window_valid(W_mode_window_valid)
        );

    end else begin : g_mode_client_disabled
        assign W_mode_gmii_txd = 8'd0;
        assign W_mode_gmii_tx_en = 1'b0;
        assign W_mode_gmii_tx_er = 1'b0;
        assign W_mode_tx_request = 1'b0;
        assign W_mode_tx_done_tx = 1'b1;
        assign W_mode_tx_ready = 1'b0;
        assign W_mode_tx_wbuf = 1'b0;
        assign W_mode_tx_we = 1'b0;
        assign W_mode_tx_addr = 11'd0;
        assign W_mode_tx_wdata = 8'd0;
        assign W_mode_tx_commit = 1'b0;
        assign W_mode_rx_valid = 1'b0;
        assign W_mode_rx_rbuf = 1'b0;
        assign W_mode_rx_len = 16'd0;
        assign W_mode_rx_addr = 11'd0;
        assign W_mode_rx_rdata = 8'd0;
        assign W_mode_rx_release = 1'b0;
        assign W_mode_bad_frame_count = 32'd0;
        assign W_mode_busy_drop_count = 32'd0;
        assign W_fil_vsync = 1'b0;
        assign W_fil_hsync = 1'b0;
        assign W_fil_de = 1'b0;
        assign W_fil_user = 1'b0;
        assign W_fil_last = 1'b0;
        assign W_fil_y = 8'd0;
        assign W_fil_mode = 5'd0;
        assign W_mode_out_mode = 5'd0;
        assign W_mode_out_vsync = 1'b0;
        assign W_mode_out_hsync = 1'b0;
        assign W_mode_out_de = 1'b0;
        assign W_mode_out_user = 1'b0;
        assign W_mode_out_last = 1'b0;
        assign W_mode_out_y = 8'd0;
        assign W_mode_out_edge = 1'b0;
        assign W_mode_out_mask = 1'b0;
        assign W_mode_window_valid = 1'b0;
    end
    endgenerate

    // Frame owners include the full IFG: B5 telemetry/key, preserved B6, then
    // the reserved B7 slot (currently tied off).
    wire [2:0] W_network_grant;
    ctl_gmii_tx_scheduler3 #(.P_TX_SCHEDULE(1)) u_network_scheduler (
        .I_clk_tx(W_clk_tx), .I_rst_tx_n(W_rst_tx_n),
        .I_tx_request({W_ink_tx_request && W_ink_rst_tx_n,W_mode_tx_request && W_mode_pll_lock,W_ocr_tx_request}),
        .I_tx_done({W_ink_tx_done || !W_ink_rst_tx_n,W_mode_tx_done_tx || !W_mode_pll_lock,W_ocr_tx_done_tx}),
        .O_tx_grant(W_network_grant),
        .I_tx0_txd(W_ocr_gmii_txd), .I_tx1_txd(W_mode_gmii_txd), .I_tx2_txd(W_ink_gmii_txd),
        .I_tx_en({W_ink_gmii_tx_en,W_mode_gmii_tx_en,W_ocr_gmii_tx_en}),
        .I_tx_er({W_ink_gmii_tx_er,W_mode_gmii_tx_er,W_ocr_gmii_tx_er}),
        .O_txd(W_gmii_txd), .O_tx_en(W_gmii_tx_en), .O_tx_er(W_gmii_tx_er), .O_owner_valid(), .O_owner()
    );
    assign W_ocr_tx_grant = W_network_grant[0];
    assign W_mode_tx_grant = W_network_grant[1];
    assign W_ink_tx_grant = W_network_grant[2];

    // MDIO configures the PHY delay contract before packet-domain reset is
    // released.  Keep the existing diagnostic synchronizers, but make the
    // physical reset and MDIO completion part of the base reset gate.
    wire        W_mdio_oe;
    wire        W_mdio_out;
    wire [15:0] W_mdio_ext_val;
    wire        W_mdio_rxc_delay;
    wire [15:0] W_mdio_rgmii_cfg;
    reg         S_diag_error_rx;
    reg         S_diag_sync1_app;
    reg         S_diag_sync2_app;
    reg         S_link_ready_rx;
    reg         S_link_sync1_app;
    reg         S_link_sync2_app;
    initial begin
        S_diag_error_rx = 1'b0;
        S_diag_sync1_app = 1'b0;
        S_diag_sync2_app = 1'b0;
        S_link_ready_rx = 1'b0;
        S_link_sync1_app = 1'b0;
        S_link_sync2_app = 1'b0;
    end

    // Packet-link counters are owned by the RX domain.  Export only a sticky
    // error indication through a two-flop synchronizer; the 32-bit counters
    // remain internal probe signals and never become unplanned board pins.
    always @(posedge W_clk_rx or posedge W_rst_base_async) begin
        if (W_rst_base_async) begin
            S_diag_error_rx <= 1'b0;
            S_link_ready_rx <= 1'b0;
        end else begin
            if ((|W_ocr_bad_frame_count) || (|W_ocr_busy_drop_count) ||
                (|W_mode_bad_frame_count) || (|W_mode_busy_drop_count))
                S_diag_error_rx <= 1'b1;
            S_link_ready_rx <= W_link_status &&
                               (W_clock_speed == 2'b10) &&
                               W_duplex_status;
        end
    end

    always @(posedge I_clk_25m or posedge W_rst_base_async) begin
        if (W_rst_base_async) begin
            S_diag_sync1_app <= 1'b0;
            S_diag_sync2_app <= 1'b0;
            S_link_sync1_app <= 1'b0;
            S_link_sync2_app <= 1'b0;
        end else begin
            S_diag_sync1_app <= S_diag_error_rx;
            S_diag_sync2_app <= S_diag_sync1_app;
            S_link_sync1_app <= S_link_ready_rx;
            S_link_sync2_app <= S_link_sync1_app;
        end
    end
    assign IO_eth_mdio = W_mdio_oe ? W_mdio_out : 1'bz;

    ctl_mdio_strap_read #(
        // ETHA PHYAD[2:0]=001 on the MLK-S200 schematic.
        .P_PHY_ADDR(5'd1),
        .P_EXT_ADDR(16'hA001),
        .P_REG_OFF (5'h1E),
        .P_REG_DAT (5'h1F),
        .P_HALF    (8'd13),
        .P_STARTUP_CYCLES(2500001),
        .P_CONFIGURE_DELAYS(1)
    ) u_mdio_diag (
        .I_clk_25m (I_clk_25m),
        .I_rst_n   (W_phy_reset_n),
        .O_mdc    (O_eth_mdc),
        .O_mdio_oe(W_mdio_oe),
        .O_mdio_out(W_mdio_out),
        .I_mdio_in(IO_eth_mdio),
        .O_ext_val(W_mdio_ext_val),
        .O_rxc_dly_en(W_mdio_rxc_delay),
        .O_done(W_mdio_done),
        .O_error(W_mdio_error),
        .O_rgmii_cfg(W_mdio_rgmii_cfg)
    );

    assign O_eth_rst_n = W_phy_reset_n;
    // MLK-S200 LEDs sink current: low turns the lamp on (schematic sheet 11).
    // LED1 = a key packet was sent (1 s stretch), LED2/3 = raw key matrix
    // activity (orientation A/B), LED4 = a key is currently held (debounced)
    // plus a sticky automatic-cycle FIFO overflow/MDIO diagnostic.
    reg [24:0] S_tx_led_cnt;
    reg        S_tx_led;
    always @(posedge I_clk_25m or negedge W_rst_app_n) begin
        if (!W_rst_app_n) begin
            S_tx_led_cnt <= 25'd0;
            S_tx_led     <= 1'b0;
        end else if (W_key_tx_done) begin
            S_tx_led_cnt <= 25'd25_000_000;
            S_tx_led     <= 1'b1;
        end else if (S_tx_led_cnt != 25'd0) begin
            S_tx_led_cnt <= S_tx_led_cnt - 1'b1;
        end else begin
            S_tx_led <= 1'b0;
        end
    end
    wire W_led4_diag = (S_tx_led & ~W_telemetry_snapshot_parity) |
        W_cycle_queue_overflow | W_mdio_error;
    assign O_led = ~{W_led4_diag, W_key_raw_act[1], W_key_raw_act[0], W_key_valid};

    // =====================================================================
    // TF 卡（SPI 模式）+ MAX98357A I2S 音频播放。KEY1/2 为遥测告警，
    // KEY3 为周期软复位，KEY4 为扬声器开关，KEY5..16 为车牌语音。
    //   TF 卡：MLK-S200 底板 TF 接口 P25/P35/P39/P19（手册 p15，LVCMOS25）
    //   音频：MAX98357A 模块接 GPIO 排针 P37/P31/P22/P28/P26（手册 p16-17）
    //   时钟：全部在 25 MHz 域；BCLK = 25/8 = 3.125 MHz，
    //         LRCLK = 25/512 = 48 828.125 Hz（无 PLL、无跨时钟域）
    // =====================================================================
    wire        W_aud_buf_we_unused;
    wire [7:0]  W_aud_buf_wdata_unused;
    wire        W_aud_play_req;
    wire        W_aud_fifo_empty;
    wire        W_aud_i2s_active;
    wire        W_aud_fifo_space;
    wire [15:0] W_aud_fifo_dout;
    wire        W_aud_fifo_re;
    wire        W_aud_fifo_we;
    wire [15:0] W_aud_fifo_wdata;
    wire        W_aud_fifo_clr;
    wire [8:0]  W_aud_buf_addr;
    wire [7:0]  W_aud_buf_rdata;
    wire [1:0]  W_aud_op;
    wire [31:0] W_aud_sector;
    wire        W_aud_start;
    wire        W_tf_ready;
    wire        W_tf_busy;
    wire        W_tf_done;
    wire [3:0]  W_tf_err;
    wire        W_aud_playing;
    wire [3:0]  W_aud_status;
    // 播放器 / 失败提示音 对 FIFO 的写口仲裁
    wire        W_aud_player_we;
    wire [15:0] W_aud_player_wdata;
    wire        W_tone_we;
    wire [15:0] W_tone_data;
    wire        W_tone_start;
    wire [3:0]  W_tone_nbeeps;
    wire        W_tone_busy;
    reg  [3:0]  S_tone_beeps;
    reg         W_key_edge_d;
    reg         W_aud_playing_d;
    reg         W_play_fell_d;
    reg         W_beep_fail;        // 本次播放是否失败（状态 4..8）
    // ------------------------------------------------------------------
    // 周期派发与手动音频仲裁。
    //   KEY1 -> P01.WAV 速度告警；KEY2 -> P02.WAV 距离告警；
    //   KEY3 -> 周期软复位；KEY4 -> 扬声器开/关；KEY5..KEY16 -> 车牌 WAV。
    // 自动队列未清空时只记住最后一次手动请求，避免打断 64-entry 队列。
    // ------------------------------------------------------------------
    wire [4:0] W_key_press = (W_key_valid && (W_key_code != 5'd0)) ? W_key_code : 5'd0;
    reg  [4:0] W_key_press_d;
    wire       W_key_edge  = (W_key_press != 5'd0) && (W_key_press != W_key_press_d);
    wire       W_cycle_reset = W_key_edge && (W_key_press == 5'd3);
    wire       W_audio_rst_n = W_rst_app_n && !W_cycle_reset;
    reg  [4:0] S_aud_track;
    reg        S_speaker_enable;
    wire        W_telemetry_aud_req;
    wire [4:0]  W_telemetry_aud_track;
    reg         S_manual_pending;
    reg  [4:0]  S_manual_track;

    wire W_cycle_start_packet = W_telemetry_update &&
        W_telemetry_cycle_active &&
        (W_telemetry_cycle_reset_ack == W_cycle_reset_request) &&
        !W_cycle_active;
    wire W_manual_press = W_key_edge && (W_key_press != 5'd3) &&
        (W_key_press != 5'd4) &&
        !W_cycle_active && !W_cycle_start_packet;
    wire W_manual_now = W_manual_press && !W_cycle_auto_busy;
    wire W_manual_late = S_manual_pending && !W_cycle_auto_busy &&
        !W_cycle_active && !W_cycle_start_packet;
    wire W_manual_req = W_manual_now || W_manual_late;
    wire [4:0] W_manual_track = W_manual_now ? W_key_press : S_manual_track;
    wire W_aud_trigger = (W_telemetry_aud_req || W_manual_req) && !W_cycle_reset;

    wire W_play_fell = W_aud_playing_d && !W_aud_playing;

    // Include the final FIFO/I2S tail frame and the one-cycle trigger pipeline
    // in the busy contract.  This is also the dispatcher back-pressure signal.
    wire W_telemetry_aud_busy = W_aud_playing || !W_aud_fifo_empty ||
        W_aud_i2s_active || W_tone_busy || W_manual_req || W_aud_trigger ||
        W_telemetry_aud_req || W_play_fell || (W_play_fell_d && W_beep_fail);

    ctl_plate_cycle_dispatch u_plate_cycle_dispatch (
        .I_clk              (I_clk_25m),
        .I_rst_n            (W_rst_app_n),
        .I_update           (W_telemetry_update),
        .I_cycle_active     (W_telemetry_cycle_active),
        .I_cycle_reset_ack  (W_telemetry_cycle_reset_ack),
        .I_metrics_valid    (W_telemetry_metrics_valid),
        .I_speed_mmps       (W_telemetry_speed_mmps),
        .I_dist_mm          (W_telemetry_dist_mm),
        .I_reset_cycle      (W_cycle_reset),
        .I_audio_busy       (W_telemetry_aud_busy),
        .I_audio_sample     (W_aud_fifo_re && !W_aud_fifo_empty),
        .O_reset_request    (W_cycle_reset_request),
        .O_cycle_active     (W_cycle_active),
        .O_cycle_index      (W_cycle_index),
        .O_auto_busy        (W_cycle_auto_busy),
        .O_play_req         (W_telemetry_aud_req),
        .O_track            (W_telemetry_aud_track),
        .O_osd_key          (W_cycle_osd_key),
        .O_queue_overflow   (W_cycle_queue_overflow)
    );

    always @(posedge I_clk_25m or negedge W_rst_app_n) begin
        if (!W_rst_app_n) begin
            S_manual_pending <= 1'b0;
            S_manual_track   <= 5'd0;
        end else if (W_cycle_reset) begin
            S_manual_pending <= 1'b0;
            S_manual_track   <= 5'd0;
        end else begin
            if (W_manual_late || W_manual_now)
                S_manual_pending <= 1'b0;
            if (W_manual_press && W_cycle_auto_busy) begin
                S_manual_pending <= 1'b1;
                S_manual_track   <= W_key_press;
            end
        end
    end

    always @(posedge I_clk_25m or negedge W_rst_app_n) begin
        if (!W_rst_app_n) W_key_press_d <= 5'd0;
        else              W_key_press_d <= W_key_press;
    end

    // Default on. One debounced KEY4 press toggles all speaker output,
    // including automatic alerts and failure beeps, inside or outside a cycle.
    // Muting does not stop playback timing; KEY3 leaves this setting unchanged.
    always @(posedge I_clk_25m or negedge W_rst_app_n) begin
        if (!W_rst_app_n) S_speaker_enable <= 1'b1;
        else if (W_key_edge && (W_key_press == 5'd4))
            S_speaker_enable <= ~S_speaker_enable;
    end

    always @(posedge I_clk_25m or negedge W_rst_app_n) begin
        if (!W_rst_app_n) S_aud_track <= 5'd0;
        else if (W_cycle_reset) S_aud_track <= 5'd0;
        else if (W_telemetry_aud_req) S_aud_track <= W_telemetry_aud_track;
        else if (W_manual_req) S_aud_track <= W_manual_track;
    end

    // 沿延迟一拍再触发，避免播放器采到上一个按键的曲目号。
    always @(posedge I_clk_25m or negedge W_rst_app_n) begin
        if (!W_rst_app_n) W_key_edge_d <= 1'b0;
        else              W_key_edge_d <= W_aud_trigger;
    end

    wire W_track_wav = (S_aud_track == 5'd1) || (S_aud_track == 5'd2) ||
                       ((S_aud_track >= 5'd5) && (S_aud_track <= 5'd16));
    assign W_aud_play_req   = W_key_edge_d && W_track_wav && !W_cycle_reset;

    // 播放失败提示音：状态码 4..8（无卡 / 找不到文件 / WAV 非法 / TF 错误 /
    // 文件系统不支持）响 N 声“嘀”；成功(status=3)不响。
    always @(posedge I_clk_25m or negedge W_rst_app_n) begin
        if (!W_rst_app_n) begin
            W_aud_playing_d <= 1'b0;
            W_play_fell_d   <= 1'b0;
            W_beep_fail     <= 1'b0;
            S_tone_beeps    <= 4'd0;
        end
        else if (W_cycle_reset) begin
            W_aud_playing_d <= 1'b0;
            W_play_fell_d   <= 1'b0;
            W_beep_fail     <= 1'b0;
            S_tone_beeps    <= 4'd0;
        end else begin
            W_aud_playing_d <= W_aud_playing;
            W_play_fell_d   <= W_play_fell;
            if (W_play_fell) begin
                W_beep_fail  <= (W_aud_status >= 4'd4) && (W_aud_status <= 4'd8);
                S_tone_beeps <= W_aud_status;
            end
        end
    end

    assign W_tone_start  = W_play_fell_d && W_beep_fail;
    assign W_tone_nbeeps = S_tone_beeps;

    // FIFO 写口：播放器优先，空闲时给失败提示音
    assign W_aud_fifo_we    = W_aud_player_we | (W_tone_we & ~W_aud_playing);
    assign W_aud_fifo_wdata = W_aud_playing ? W_aud_player_wdata : W_tone_data;

    tf_card_ctrl #(
        .SPI_W       (16),
        .DIV_INIT    (16'd31),      // 25 MHz/(2*32) = 390.6 kHz（识别，<400 kHz）
        .DIV_RUN     (16'd1),       // 25 MHz/(2*2)  = 6.25 MHz（数据传输）
        .TO_BYTES    (16'd60000),
        .ACMD41_RETRY(16'd2000),
        // 真卡上必须关掉读 CRC 校验（SPI 模式数据块 CRC 可选）。
        .CHECK_RD_CRC(1'b0),
        .AW          (9)
    ) u_aud_tf (
        .I_clk        (I_clk_25m),
        .I_rst_n      (W_rst_app_n),
        .I_buf_we     (1'b0),
        .I_buf_addr   (W_aud_buf_addr),
        .I_buf_wdata  (8'd0),
        .O_buf_rdata  (W_aud_buf_rdata),
        .I_op         (W_aud_op),
        .I_sector     (W_aud_sector),
        .I_start      (W_aud_start),
        .O_ready      (W_tf_ready),
        .O_busy       (W_tf_busy),
        .O_done       (W_tf_done),
        .O_err        (W_tf_err),
        .O_card_type  (),
        .O_block_mode (),
        .O_state      (),
        .O_ocr        (),
        .O_tf_clk     (O_tf_clk),
        .O_tf_cs_n    (O_tf_cs_n),
        .O_tf_mosi    (O_tf_mosi),
        .I_tf_miso    (I_tf_miso)
    );

    aud_pcm_fifo #(.AW(9)) u_aud_fifo (
        .I_clk      (I_clk_25m),
        .I_rst_n    (W_rst_app_n),
        .I_clr      (W_aud_fifo_clr || W_cycle_reset),
        .I_we       (W_aud_fifo_we),
        .I_wdata    (W_aud_fifo_wdata),
        .I_re       (W_aud_fifo_re),
        .O_rdata    (W_aud_fifo_dout),
        .O_count    (),
        .O_empty    (W_aud_fifo_empty),
        .O_full     (),
        .O_space256 (W_aud_fifo_space)
    );

    aud_i2s_tx u_aud_i2s (
        .I_clk        (I_clk_25m),
        .I_rst_n      (W_rst_app_n),
        .I_fifo_empty (W_aud_fifo_empty),
        .I_fifo_dout  (W_aud_fifo_dout),
        .O_fifo_re    (W_aud_fifo_re),
        .O_bclk       (O_aud_bclk),
        .O_lrclk      (O_aud_lrclk),
        .O_din        (O_aud_din),
        .O_active     (W_aud_i2s_active)
    );

    aud_wav_player #(.AW(9)) u_aud_player (
        .I_clk       (I_clk_25m),
        .I_rst_n     (W_audio_rst_n),
        .I_play_req  (W_aud_play_req),
        .I_track     (S_aud_track),
        .O_buf_addr  (W_aud_buf_addr),
        .O_buf_we    (W_aud_buf_we_unused),
        .O_buf_wdata (W_aud_buf_wdata_unused),
        .I_buf_rdata (W_aud_buf_rdata),
        .O_op        (W_aud_op),
        .O_sector    (W_aud_sector),
        .O_start     (W_aud_start),
        .I_tf_ready  (W_tf_ready),
        .I_tf_busy   (W_tf_busy),
        .I_tf_done   (W_tf_done),
        .I_tf_err    (W_tf_err),
        .O_pcm_we    (W_aud_player_we),
        .O_pcm_data  (W_aud_player_wdata),
        .I_pcm_space (W_aud_fifo_space),
        .O_pcm_clr   (W_aud_fifo_clr),
        .O_playing   (W_aud_playing),
        .O_status    (W_aud_status)
    );

    aud_tone u_aud_tone (
        .I_clk    (I_clk_25m),
        .I_rst_n  (W_rst_app_n),
        .I_start  (W_tone_start),
        .I_nbeeps (W_tone_nbeeps),
        .I_abort  (W_aud_playing || W_cycle_reset),
        .I_space  (W_aud_fifo_space),
        .O_we     (W_tone_we),
        .O_data   (W_tone_data),
        .O_busy   (W_tone_busy)
    );

    // KEY4 总开关关闭时将功放置于 shutdown；开启时保留原尾音使能时序。
    // Keep the amplifier enabled through the last I2S frame; FIFO empty alone
    // is not a safe end-of-audio indication because aud_i2s_tx may still hold
    // the current sample in its tail frame.
    assign O_aud_sd   = S_speaker_enable &&
                       (W_aud_playing || !W_aud_fifo_empty ||
                        W_aud_i2s_active || W_tone_busy);
    // 增益配置脚：静态电平（接高 = 最低增益档，最稳妥）
    assign O_aud_gain = 1'b1;

endmodule
