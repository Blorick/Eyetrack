`timescale 1ns/1ps
//=============================================================================
//  ctl_stream_link  --  raw-Ethernet pixel-stream endpoint (v1.19 two-board link)
//-----------------------------------------------------------------------------
//  Domain   : application, GMII TX, GMII RX (three independent clocks)
//  Reset    : one board reset per domain; assertion async, release sync
//  Verify   : sim/tb_ctl_stream_link.v
//  Spec     : common/ctl_stream_link_spec.md
//=============================================================================
//  Why this sibling of ctl_lpr_packet_link exists
//  ----------------------------------------------
//  ctl_lpr_packet_link (v2.4 two_board/common) holds ONE 1024-byte mailbox per
//  direction.  At 1 Gb/s a packet arrives every ~8.5 us, so the application
//  would have to drain 1024 bytes in 8.5 us = 120 MB/s.  The v1.19 application
//  clock is 75.002 MHz = 75 MB/s, so it can never keep up and every other packet
//  is lost.  That is structural, not tunable: a pixel stream needs concurrent
//  fill and drain.
//
//  Structure
//  ---------
//  Two ping-pong frame buffers per direction.  Each is a simple dual-port RAM
//  with one port per clock domain, so no byte crosses a domain combinationally
//  and no asynchronous FIFO is required.  Ownership uses toggles (2-FF
//  synchronised), the same level-handshake idea as the packet link:
//
//    TX buffer i : app toggles S_tx_tog_app[i] on commit
//                  tx  toggles S_tx_done_tog[i] after FCS + IFG
//                  app may reuse buffer i when S_tx_done_s2[i] == S_tx_tog_app[i]
//    RX buffer i : rx  toggles S_rx_tog_rx[i] once a *valid* frame is complete
//                  app toggles S_rx_rel_tog[i] after copying the frame out
//                  rx may reuse buffer i when S_rx_rel_s2[i] == S_rx_tog_rx[i]
//
//  A received frame is stored while it arrives but is published to the
//  application only after MAC address, EtherType, application magic, length and
//  FCS all pass, so a bad frame is never visible to the application.
//
//  Wire format (spec section 2)
//  ----------------------------
//    14B Ethernet header (dst = peer MAC, src = local MAC, EtherType 0x88B6)
//    16B application header (payload length at bytes 6..7 little endian;
//        magic at bytes 0..1; line number at bytes 2..3)
//    N B payload
//    0..pad zero bytes (Ethernet minimum data field is 46 bytes)
//    4B FCS = ones-complement of the reflected CRC-32
//
//  The application-header layout is part of this module's contract: the receive
//  side must learn the frame length before the frame has finished arriving.
//=============================================================================
module ctl_stream_link #(
    parameter [47:0]  P_LOCAL_MAC = 48'h02_00_00_12_34_56,
    parameter [47:0]  P_PEER_MAC  = 48'h02_00_00_65_43_21,
    parameter [15:0]  P_ETHERTYPE = 16'h88B6,
    parameter [15:0]  P_APP_MAGIC = 16'h01A5,   // byte0 = A5, byte1 = 01
    parameter integer P_MAX_BYTES = 2048,
    parameter integer P_AW        = 11,
    parameter integer P_TX_SCHEDULE = 0,
    parameter integer P_TX_ARB_EN = 0
)(
    //--------------------------------------------------------------- GMII TX
    input  wire        I_clk_tx,
    input  wire        I_rst_tx_n,
    output reg  [7:0]  O_gmii_txd,
    output reg         O_gmii_tx_en,
    output reg         O_gmii_tx_er,
    input  wire         I_tx_grant,
    output wire         O_tx_request,
    output reg          O_tx_done_tx,

    //--------------------------------------------------------------- GMII RX
    input  wire        I_clk_rx,
    input  wire        I_rst_rx_n,
    input  wire [7:0]  I_gmii_rxd,
    input  wire        I_gmii_rx_dv,
    input  wire        I_gmii_rx_er,

    //------------------------------------------------------ application (TX)
    input  wire        I_clk_app,
    input  wire        I_rst_app_n,
    output wire        O_tx_ready,
    output wire        O_tx_wbuf,
    input  wire        I_tx_we,
    input  wire [P_AW-1:0] I_tx_addr,
    input  wire [7:0]  I_tx_wdata,
    input  wire        I_tx_commit,

    //------------------------------------------------------ application (RX)
    output wire        O_rx_valid,
    output wire        O_rx_rbuf,
    output wire [15:0] O_rx_len,
    input  wire [P_AW-1:0] I_rx_addr,
    output wire [7:0]  O_rx_rdata,
    input  wire        I_rx_release,

    //--------------------------------------------------------------- counters
    output reg  [31:0] O_tx_frame_count,
    output reg  [31:0] O_rx_frame_count,
    output reg  [31:0] O_rx_bad_count,
    output reg  [31:0] O_rx_busy_drop_count,
    output reg  [15:0] O_rx_last_line,
    output wire [31:0] O_bad_frame_count,
    output wire [31:0] O_busy_drop_count
);

    //=========================================================================
    // 0) constants and functions
    //=========================================================================
    localparam integer P_HDR_BYTES = 16;
    localparam integer P_ETH_HDR   = 14;
    localparam integer P_MIN_DATA  = 46;

    localparam [2:0] T_IDLE = 3'd0, T_PRE = 3'd1, T_HDR = 3'd2,
                     T_PAY  = 3'd3, T_PAD = 3'd4, T_FCS = 3'd5, T_IFG = 3'd6,
                     T_ETH  = 3'd7;

    localparam [2:0] R_IDLE = 3'd0, R_PRE = 3'd1, R_HDR = 3'd2,
                     R_APP  = 3'd3, R_PAD = 3'd4, R_FCS = 3'd5;

    function [7:0] f_mac_byte;
        input [47:0] I_mac;
        input [3:0]  I_idx;
        begin
            case (I_idx)
                4'd0: f_mac_byte = I_mac[47:40];
                4'd1: f_mac_byte = I_mac[39:32];
                4'd2: f_mac_byte = I_mac[31:24];
                4'd3: f_mac_byte = I_mac[23:16];
                4'd4: f_mac_byte = I_mac[15:8];
                default: f_mac_byte = I_mac[7:0];
            endcase
        end
    endfunction

    function [31:0] f_crc32_nibble;
        input [3:0] I_idx;
        begin
            case (I_idx)
                4'h0: f_crc32_nibble = 32'h0000_0000;
                4'h1: f_crc32_nibble = 32'h1DB7_1064;
                4'h2: f_crc32_nibble = 32'h3B6E_20C8;
                4'h3: f_crc32_nibble = 32'h26D9_30AC;
                4'h4: f_crc32_nibble = 32'h76DC_4190;
                4'h5: f_crc32_nibble = 32'h6B6B_51F4;
                4'h6: f_crc32_nibble = 32'h4DB2_6158;
                4'h7: f_crc32_nibble = 32'h5005_713C;
                4'h8: f_crc32_nibble = 32'hEDB8_8320;
                4'h9: f_crc32_nibble = 32'hF00F_9344;
                4'hA: f_crc32_nibble = 32'hD6D6_A3E8;
                4'hB: f_crc32_nibble = 32'hCB61_B38C;
                4'hC: f_crc32_nibble = 32'h9B64_C2B0;
                4'hD: f_crc32_nibble = 32'h86D3_D2D4;
                4'hE: f_crc32_nibble = 32'hA00A_E278;
                default: f_crc32_nibble = 32'hBDBD_F21C;
            endcase
        end
    endfunction

    function [31:0] f_crc32_byte;
        input [31:0] I_crc;
        input [7:0]  I_data;
        reg [31:0] W_mid;
        begin
            W_mid = (I_crc >> 4) ^ f_crc32_nibble(I_crc[3:0] ^ I_data[3:0]);
            f_crc32_byte = (W_mid >> 4) ^ f_crc32_nibble(W_mid[3:0] ^ I_data[7:4]);
        end
    endfunction

    function [31:0] f_eth_header_crc;
        input I_unused;
        integer i;
        reg [31:0] c;
        begin
            c = 32'hFFFF_FFFF;
            for (i = 0; i < 6; i = i + 1)
                c = f_crc32_byte(c, f_mac_byte(P_PEER_MAC, i[3:0]));
            for (i = 0; i < 6; i = i + 1)
                c = f_crc32_byte(c, f_mac_byte(P_LOCAL_MAC, i[3:0]));
            c = f_crc32_byte(c, P_ETHERTYPE[15:8]);
            c = f_crc32_byte(c, P_ETHERTYPE[7:0]);
            f_eth_header_crc = c;
        end
    endfunction

    localparam [31:0] P_ETH_CRC_SEED = f_eth_header_crc(1'b0);

    // ★ The RECEIVE seed is a different constant: a frame that arrives here
    //   carries dst = THIS board's MAC (= P_LOCAL_MAC) and src = the peer's,
    //   i.e. local-then-peer, while a frame we transmit carries peer-then-
    //   local.  Using the transmit seed on the receive side made every frame
    //   fail its FCS check (the CRC came out 0x256BAF96 for a frame whose FCS
    //   was 0x3B7F2246).
    function [31:0] f_eth_header_crc_rx;
        input I_unused;
        integer i;
        reg [31:0] c;
        begin
            c = 32'hFFFF_FFFF;
            for (i = 0; i < 6; i = i + 1)
                c = f_crc32_byte(c, f_mac_byte(P_LOCAL_MAC, i[3:0]));
            for (i = 0; i < 6; i = i + 1)
                c = f_crc32_byte(c, f_mac_byte(P_PEER_MAC, i[3:0]));
            c = f_crc32_byte(c, P_ETHERTYPE[15:8]);
            c = f_crc32_byte(c, P_ETHERTYPE[7:0]);
            f_eth_header_crc_rx = c;
        end
    endfunction

    localparam [31:0] P_ETH_CRC_SEED_RX = f_eth_header_crc_rx(1'b0);

    assign O_bad_frame_count = O_rx_bad_count;
    assign O_busy_drop_count = O_rx_busy_drop_count;
    //=========================================================================
    // 1) ALL storage declarations (one block, before any always block: Verilog
    //    treats an identifier used before its declaration as an implicit 1-bit
    //    net, which silently changes behaviour and breaks TD's RAM inference)
    //=========================================================================
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [7:0] S_tx_mem [0:2*P_MAX_BYTES-1];
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [7:0] S_rx_mem [0:2*P_MAX_BYTES-1];

    // TX ownership
    reg  [1:0]  S_tx_tog_app;
    reg  [1:0]  S_tx_tog_tx_s1, S_tx_tog_tx_s2;
    reg  [1:0]  S_tx_done_tog_tx;
    reg  [1:0]  S_tx_done_s1, S_tx_done_s2;
    // Application writes and the later commit can be separated by several
    // thousand pixel clocks while a line filter drains.  The old combinational
    // O_tx_wbuf followed whichever ping-pong buffer happened to be free at
    // each instant, so a long packet could put its payload in buffer 1 and its
    // header in buffer 0 after buffer 0 completed on the wire.  Lock the
    // application-side choice at the first write and release it only at
    // commit; the TX domain still owns the independent done toggles.
    reg        S_tx_app_sel_valid;
    reg        S_tx_app_sel_buf;

    // RX ownership
    reg  [1:0]  S_rx_tog_rx;
    reg  [1:0]  S_rx_tog_app_s1, S_rx_tog_app_s2;
    reg  [1:0]  S_rx_rel_tog_app;
    reg  [1:0]  S_rx_rel_s1, S_rx_rel_s2;

    // RX length per buffer, crossed with the publish toggle
    reg  [15:0] S_rx_len_rx_0, S_rx_len_rx_1;
    reg  [15:0] S_rx_len_s1_0, S_rx_len_s1_1;
    reg  [15:0] S_rx_len_s2_0, S_rx_len_s2_1;

    // TX datapath
    reg  [2:0]  S_t_state;
    // TX-domain request for the shared GMII arbiter.  The toggle comparison
    // covers a committed ping-pong buffer; the FSM term covers the full
    // preamble/frame/FCS/IFG ownership interval.
    assign O_tx_request =
        (S_t_state != T_IDLE) ||
        (S_tx_tog_tx_s2 != S_tx_done_tog_tx);
    reg  [2:0]  S_t_pre;
    reg  [3:0]  S_t_eth_idx;
    reg  [4:0]  S_t_hdr_idx;
    reg  [15:0] S_t_pay_len;
    reg  [15:0] S_t_pay_idx;
    reg  [15:0] S_t_pad_len;
    reg  [15:0] S_t_pad_idx;
    reg  [2:0]  S_t_fcs_idx;
    reg  [3:0]  S_t_ifg;
    reg  [31:0] S_t_crc;
    reg  [31:0] S_t_crc_final;
    reg         S_t_buf;
    reg  [P_AW-1:0] S_t_raddr;
    reg  [7:0]  S_t_rdata;
    reg         S_t_done_pulse;
    reg  [31:0] S_t_frame_count;

    // RX datapath
    reg  [2:0]  S_r_state;
    reg  [4:0]  S_r_pre;
    reg  [15:0] S_r_idx;
    reg  [15:0] S_r_len_cur;
    reg  [15:0] S_r_pad;
    reg  [2:0]  S_r_fcs_idx;
    reg  [31:0] S_r_crc;
    reg  [7:0]  S_r_fcs0, S_r_fcs1, S_r_fcs2, S_r_fcs3;
    reg  [1:0]  S_r_magic;
    reg         S_r_err;
    reg         S_r_buf;
    // FSM-owned pulse.  The verdict block below is the ONLY writer of
    // S_r_check; TD rejected the design ("net S_r_check is constantly driven
    // from multiple places") when the receive FSM also assigned it.
    reg         S_r_fcs_end;
    reg         S_r_len_ld;      // set once header bytes 6..7 have been latched
    // Down-counter of application-frame bytes still to come.  The end-of-frame
    // test used to be a 16-bit add + compare against W_r_flen, and that chain
    // (S_r_idx -> adder -> compare -> state decode -> register CE) was the worst
    // path of the whole slave in the 125 MHz receive domain: -254 ps, 10 failing
    // endpoints.  Counting down makes it a 12-bit equality instead.
    reg  [11:0] S_r_rem;
    // Application-byte counter: every field extraction below used to test
    // (S_r_idx - 14) == k, i.e. a 16-bit subtract plus a 16-bit compare,
    // repeated eight times.  That chain was the worst path of the 125 MHz
    // receive domain; a 12-bit counter makes each test a 12-bit equality.
    reg  [11:0] S_rapp_i;
    // Length-range validity, registered once when W_r_flen becomes complete.
    // Left combinational it sat on the verdict path as length -> +16 adder ->
    // two comparators -> publish toggle, which was the worst path of the RX
    // domain (8.681 ns of an 8.000 ns budget, 54% of it routing).
    reg         S_r_len_ok;
    reg  [15:0] S_r_line_rx;
    reg  [31:0] S_r_frame_count;
    reg  [31:0] S_r_bad_count;
    reg  [31:0] S_r_drop_count;
    reg  [15:0] S_r_last_line_rx;

    // RX application-side read register and counter synchronisers
    reg  [7:0]  S_rx_rdata;
    reg  [31:0] S_rx_bad_s1, S_rx_bad_s2;
    reg  [31:0] S_rx_frame_s1, S_rx_frame_s2;
    reg  [31:0] S_rx_drop_s1, S_rx_drop_s2;
    reg  [15:0] S_rx_line_s1, S_rx_line_s2;

    integer k;

    wire [7:0]  W_r_byte  = I_gmii_rxd;
    wire [31:0] W_r_fcs_rx = {S_r_fcs3, S_r_fcs2, S_r_fcs1, S_r_fcs0};
    // S_r_len_cur is the *payload* length read from header bytes 6..7; the
    // application frame is that plus the 16-byte application header.
    wire [15:0] W_r_flen = S_r_len_cur + P_HDR_BYTES;

    //=========================================================================
    // 2) TX ownership handshake
    //=========================================================================
    always @(posedge I_clk_tx or negedge I_rst_tx_n) begin
        if (!I_rst_tx_n) begin
            S_tx_tog_tx_s1 <= 2'd0;
            S_tx_tog_tx_s2 <= 2'd0;
        end
        else begin
            S_tx_tog_tx_s1 <= S_tx_tog_app;
            S_tx_tog_tx_s2 <= S_tx_tog_tx_s1;
        end
    end

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            S_tx_done_s1 <= 2'd0;
            S_tx_done_s2 <= 2'd0;
        end
        else begin
            S_tx_done_s1 <= S_tx_done_tog_tx;
            S_tx_done_s2 <= S_tx_done_s1;
        end
    end

    wire W_tx_busy0 = (S_tx_tog_app[0] != S_tx_done_s2[0]);
    wire W_tx_busy1 = (S_tx_tog_app[1] != S_tx_done_s2[1]);

    // A ping-pong endpoint can accept a new packet while the other buffer is
    // still on the wire.  Returning ready only when *both* buffers were free
    // serialized the 75 MHz line engine behind the 125 MHz transmitter and
    // caused a deterministic every-other-line drop during coexist traffic.
    // The commit-side mux below still chooses buffer 0 first and buffer 1
    // while buffer 0 is held, so this level is safe for the application writer.
    assign O_tx_ready = !(W_tx_busy0 && W_tx_busy1);
    assign O_tx_wbuf  = S_tx_app_sel_valid ? S_tx_app_sel_buf :
                        (W_tx_busy0 ? 1'b1 : 1'b0);   // prefer buffer 0

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) S_tx_tog_app <= 2'd0;
        else if (I_tx_commit) begin
            if (S_tx_app_sel_valid) begin
                if (S_tx_app_sel_buf == 1'b0)
                    S_tx_tog_app[0] <= ~S_tx_tog_app[0];
                else
                    S_tx_tog_app[1] <= ~S_tx_tog_app[1];
            end
            else if (!W_tx_busy0) S_tx_tog_app[0] <= ~S_tx_tog_app[0];
            else                  S_tx_tog_app[1] <= ~S_tx_tog_app[1];
        end
    end

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            S_tx_app_sel_valid <= 1'b0;
            S_tx_app_sel_buf   <= 1'b0;
        end
        else begin
            // A write is the first unambiguous indication that a new packet
            // is being filled.  Keep the chosen buffer through the commit,
            // even if the other ping-pong buffer becomes free meanwhile.
            if (!S_tx_app_sel_valid && I_tx_we) begin
                S_tx_app_sel_valid <= 1'b1;
                S_tx_app_sel_buf   <= W_tx_busy0 ? 1'b1 : 1'b0;
            end
            if (I_tx_commit) begin
                S_tx_app_sel_valid <= 1'b0;
                S_tx_app_sel_buf   <= 1'b0;
            end
        end
    end

    always @(posedge I_clk_app) begin
        if (I_tx_we)
            S_tx_mem[O_tx_wbuf*P_MAX_BYTES + I_tx_addr] <= I_tx_wdata;
    end

    //=========================================================================
    // 3) transmit FSM (tx clock domain)
    //=========================================================================
    always @(posedge I_clk_tx) S_t_rdata <= S_tx_mem[S_t_buf*P_MAX_BYTES + S_t_raddr];
    wire [7:0] W_t_byte = S_t_rdata;

    wire [7:0] W_t_eth_byte =
          (S_t_eth_idx < 4'd6)   ? f_mac_byte(P_PEER_MAC,  S_t_eth_idx)
        : (S_t_eth_idx < 4'd12)  ? f_mac_byte(P_LOCAL_MAC, S_t_eth_idx - 4'd6)
        : (S_t_eth_idx == 4'd12) ? P_ETHERTYPE[15:8]
        :                          P_ETHERTYPE[7:0];

    always @(posedge I_clk_tx or negedge I_rst_tx_n) begin
        if (!I_rst_tx_n) begin
            S_t_state <= T_IDLE;
            O_gmii_txd <= 8'h00; O_gmii_tx_en <= 1'b0; O_gmii_tx_er <= 1'b0;
            S_t_pre <= 3'd0; S_t_hdr_idx <= 5'd0; S_t_pay_len <= 16'd0;
            S_t_eth_idx <= 4'd0;
            S_t_pay_idx <= 16'd0; S_t_pad_len <= 16'd0; S_t_pad_idx <= 16'd0;
            S_t_fcs_idx <= 3'd0; S_t_ifg <= 4'd0;
            S_t_crc <= 32'hFFFF_FFFF; S_t_crc_final <= 32'd0;
            S_t_buf <= 1'b0; S_t_raddr <= {P_AW{1'b0}};
            S_t_done_pulse <= 1'b0;
            O_tx_done_tx <= 1'b0;
        end
        else begin
            S_t_done_pulse <= 1'b0;
            O_tx_done_tx <= 1'b0;
            O_gmii_tx_en   <= 1'b0;
            O_gmii_tx_er   <= 1'b0;

            case (S_t_state)
            T_IDLE: begin
                if ((S_tx_tog_tx_s2[0] != S_tx_done_tog_tx[0]) &&
                    ((P_TX_ARB_EN == 0) || I_tx_grant)) begin
                    S_t_buf   <= 1'b0;
                    S_t_raddr <= {P_AW{1'b0}};
                    S_t_crc   <= P_ETH_CRC_SEED;
                    S_t_pre   <= 3'd0;
                    S_t_state <= T_PRE;
                end
                else if ((S_tx_tog_tx_s2[1] != S_tx_done_tog_tx[1]) &&
                         ((P_TX_ARB_EN == 0) || I_tx_grant)) begin
                    S_t_buf   <= 1'b1;
                    S_t_raddr <= {P_AW{1'b0}};
                    S_t_crc   <= P_ETH_CRC_SEED;
                    S_t_pre   <= 3'd0;
                    S_t_state <= T_PRE;
                end
            end

            T_PRE: begin
                O_gmii_tx_en <= 1'b1;
                O_gmii_txd   <= (S_t_pre == 3'd7) ? 8'hD5 : 8'h55;
                if (S_t_pre == 3'd7) begin
                    S_t_pre     <= 3'd0;
                    S_t_eth_idx <= 4'd0;
                    S_t_state   <= T_ETH;
                end
                else S_t_pre <= S_t_pre + 3'd1;
            end

            // The 14-byte Ethernet header is generated here and its CRC is
            // already folded into the running CRC as the seed, so these bytes
            // are emitted WITHOUT updating the CRC (the receive side does the
            // same thing in the other direction).
            T_ETH: begin
                O_gmii_tx_en <= 1'b1;
                O_gmii_txd   <= W_t_eth_byte;
                if (S_t_eth_idx == 4'd13) begin
                    S_t_eth_idx <= 4'd0;
                    S_t_hdr_idx <= 5'd0;
                    // ★ The frame memory has a REGISTERED read, so the byte
                    //    presented in cycle n is mem[address in cycle n-1].
                    //    The address must therefore lead by one: leave T_ETH
                    //    with address 1 so that T_HDR cycle 1 shows mem[0].
                    //    (Setting it to 0 made every byte arrive one position
                    //     late: the payload length read as 16384 instead of 64
                    //     and no frame was ever published.)
                    S_t_raddr   <= {{(P_AW-1){1'b0}}, 1'b1};
                    S_t_state   <= T_HDR;
                end
                else S_t_eth_idx <= S_t_eth_idx + 4'd1;
            end

            T_HDR: begin
                O_gmii_tx_en <= 1'b1;
                O_gmii_txd   <= W_t_byte;
                S_t_crc      <= f_crc32_byte(S_t_crc, W_t_byte);
                if (S_t_hdr_idx == 5'd6) S_t_pay_len[7:0]  <= W_t_byte;
                if (S_t_hdr_idx == 5'd7) S_t_pay_len[15:8] <= W_t_byte;
                S_t_raddr <= S_t_raddr + 1'b1;
                if (S_t_hdr_idx == (P_HDR_BYTES-1)) begin
                    S_t_hdr_idx <= 5'd0;
                    S_t_pay_idx <= 16'd0;
                    S_t_pad_len <= ((P_HDR_BYTES + S_t_pay_len) >= P_MIN_DATA)
                                   ? 16'd0
                                   : (P_MIN_DATA - (P_HDR_BYTES + S_t_pay_len));
                    S_t_state   <= T_PAY;
                end
                else S_t_hdr_idx <= S_t_hdr_idx + 5'd1;
            end

            T_PAY: begin
                O_gmii_tx_en <= 1'b1;
                O_gmii_txd   <= W_t_byte;
                S_t_crc      <= f_crc32_byte(S_t_crc, W_t_byte);
                S_t_raddr    <= S_t_raddr + 1'b1;
                if ((S_t_pay_idx + 16'd1) >= S_t_pay_len) begin
                    S_t_pay_idx <= 16'd0;
                    S_t_pad_idx <= 16'd0;
                    // ★ No padding needed: go straight to the FCS.  Entering
                    //   T_PAD with pad_len==0 emitted one extra 0x00 (and folded
                    //   it into the CRC), which shifted the whole FCS by a byte
                    //   and made the receiver reject every frame.
                    if (S_t_pad_len == 16'd0) begin
                        // ★ Fold the LAST payload byte combinationally: the
                        //   registered S_t_crc only receives it at the end of
                        //   this cycle, so ~S_t_crc here would be missing one
                        //   byte (FCS came out 3B7F2246 instead of 18835225;
                        //   an independent Python model of the same byte
                        //   stream confirmed the RECEIVE side was right).
                        S_t_crc_final <= ~f_crc32_byte(S_t_crc, W_t_byte);
                        S_t_fcs_idx   <= 3'd0;
                        S_t_state     <= T_FCS;
                    end
                    else S_t_state <= T_PAD;
                end
                else S_t_pay_idx <= S_t_pay_idx + 16'd1;
            end

            T_PAD: begin
                O_gmii_tx_en <= 1'b1;
                O_gmii_txd   <= 8'h00;
                S_t_crc      <= f_crc32_byte(S_t_crc, 8'h00);
                if ((S_t_pad_idx + 16'd1) >= S_t_pad_len) begin
                    // same reason as in T_PAY: fold the last pad byte here
                    S_t_crc_final <= ~f_crc32_byte(S_t_crc, 8'h00);
                    S_t_fcs_idx   <= 3'd0;
                    S_t_state     <= T_FCS;
                end
                else S_t_pad_idx <= S_t_pad_idx + 16'd1;
            end

            T_FCS: begin
                O_gmii_tx_en <= 1'b1;
                case (S_t_fcs_idx)
                    3'd0:    O_gmii_txd <= S_t_crc_final[7:0];
                    3'd1:    O_gmii_txd <= S_t_crc_final[15:8];
                    3'd2:    O_gmii_txd <= S_t_crc_final[23:16];
                    default: O_gmii_txd <= S_t_crc_final[31:24];
                endcase
                if (S_t_fcs_idx == 3'd3) begin
                    S_t_fcs_idx <= 3'd0;
                    S_t_ifg     <= 4'd0;
                    S_t_state   <= T_IFG;
                end
                else S_t_fcs_idx <= S_t_fcs_idx + 3'd1;
            end

            T_IFG: begin
                O_gmii_tx_en <= 1'b0;
                O_gmii_txd   <= 8'h00;
                if (S_t_ifg == 4'd11) begin
                    S_t_ifg        <= 4'd0;
                    S_t_done_pulse <= 1'b1;
                    O_tx_done_tx   <= 1'b1;
                    S_t_state      <= T_IDLE;
                end
                else S_t_ifg <= S_t_ifg + 4'd1;
            end

            default: S_t_state <= T_IDLE;
            endcase
        end
    end

    // ★ The completion toggle must land on the SAME edge that returns the FSM to
    //   T_IDLE.  Driving it from the registered S_t_done_pulse made it one cycle
    //   late, so T_IDLE re-evaluated "tog_tx_s2 != done_tog" while done_tog was
    //   still stale and immediately retransmitted the same buffer -- one commit
    //   produced three identical frames on the wire.
    wire W_t_frame_done = (S_t_state == T_IFG) && (S_t_ifg == 4'd11);

    always @(posedge I_clk_tx or negedge I_rst_tx_n) begin
        if (!I_rst_tx_n) begin
            S_tx_done_tog_tx <= 2'd0;
            S_t_frame_count  <= 32'd0;
        end
        else if (W_t_frame_done) begin
            S_tx_done_tog_tx[S_t_buf] <= ~S_tx_done_tog_tx[S_t_buf];
            S_t_frame_count           <= S_t_frame_count + 32'd1;
        end
    end

    always @(posedge I_clk_app) O_tx_frame_count <= S_t_frame_count;

    //=========================================================================
    // 4) receive FSM (rx clock domain)
    //=========================================================================
    always @(posedge I_clk_rx or negedge I_rst_rx_n) begin
        if (!I_rst_rx_n) begin
            S_r_state <= R_IDLE;
            S_r_pre <= 5'd0; S_r_idx <= 16'd0; S_r_len_cur <= 16'd0; S_r_pad <= 16'd0;
            S_r_fcs_idx <= 3'd0; S_r_crc <= 32'hFFFF_FFFF;
            S_r_fcs0 <= 8'd0; S_r_fcs1 <= 8'd0; S_r_fcs2 <= 8'd0; S_r_fcs3 <= 8'd0;
            S_r_magic <= 2'd0; S_r_err <= 1'b0;
            S_r_len_ld <= 1'b0;
            S_r_rem    <= 12'd0;
            S_rapp_i   <= 12'd0;
            S_r_len_ok <= 1'b0;
            S_r_fcs_end <= 1'b0;
            S_r_line_rx <= 16'd0;
        end
        else begin
            case (S_r_state)
            R_IDLE: begin
                S_r_idx   <= 16'd0;
                S_r_magic <= 2'd0;
                S_r_err   <= 1'b0;
                S_r_len_ld <= 1'b0;
                S_r_rem    <= 12'd0;
                S_rapp_i   <= 12'd0;
                S_r_len_ok <= 1'b0;
                S_r_fcs_end <= 1'b0;
                if (I_gmii_rx_dv && (W_r_byte == 8'h55)) begin
                    S_r_pre   <= 5'd1;
                    S_r_err   <= I_gmii_rx_er;
                    S_r_state <= R_PRE;
                end
            end

            R_PRE: begin
                // Ethernet requires exactly seven 0x55 bytes followed by the
                // D5 SFD.  Keep malformed preambles attached to the frame so
                // a complete bad frame reaches the normal FCS verdict path;
                // otherwise an RX_ER on the preamble could silently disappear
                // without a bad-frame count.
                if (!I_gmii_rx_dv) begin
                    S_r_state <= R_IDLE;
                end
                else begin
                    if (I_gmii_rx_er)
                        S_r_err <= 1'b1;
                    if (W_r_byte == 8'hD5) begin
                        if (S_r_pre != 5'd7)
                            S_r_err <= 1'b1;
                        S_r_crc   <= P_ETH_CRC_SEED_RX;  // SFD is not part of the CRC
                        S_r_idx   <= 16'd0;
                        S_r_state <= R_HDR;
                    end
                    else if (W_r_byte == 8'h55) begin
                        if (S_r_pre < 5'd7)
                            S_r_pre <= S_r_pre + 5'd1;
                        else
                            S_r_err <= 1'b1;
                    end
                    else begin
                        S_r_err   <= 1'b1;
                        S_r_state <= R_IDLE;
                    end
                end
            end

            R_HDR: begin
                // RX_ER marks this frame bad but does not end it.  Consume the
                // remaining RX_DV bytes so the FCS verdict records one rejected
                // frame and the next preamble cannot be mistaken for a tail.
                if (!I_gmii_rx_dv) begin
                    S_r_err <= 1'b1; S_r_state <= R_IDLE;
                end
                else begin
                    if (I_gmii_rx_er)
                        S_r_err <= 1'b1;
                    if (S_r_idx < P_ETH_HDR) begin
                    // The running CRC was already seeded with the Ethernet
                    // header CRC at the SFD, so these 14 bytes are validated but
                    // NOT folded into the CRC again (doing so double-counts the
                    // header and rejects every frame).
                    if ((S_r_idx == 16'd0)  && (W_r_byte != f_mac_byte(P_LOCAL_MAC,4'd0))) S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd1)  && (W_r_byte != f_mac_byte(P_LOCAL_MAC,4'd1))) S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd2)  && (W_r_byte != f_mac_byte(P_LOCAL_MAC,4'd2))) S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd3)  && (W_r_byte != f_mac_byte(P_LOCAL_MAC,4'd3))) S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd4)  && (W_r_byte != f_mac_byte(P_LOCAL_MAC,4'd4))) S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd5)  && (W_r_byte != f_mac_byte(P_LOCAL_MAC,4'd5))) S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd6)  && (W_r_byte != f_mac_byte(P_PEER_MAC,4'd0)))  S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd7)  && (W_r_byte != f_mac_byte(P_PEER_MAC,4'd1)))  S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd8)  && (W_r_byte != f_mac_byte(P_PEER_MAC,4'd2)))  S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd9)  && (W_r_byte != f_mac_byte(P_PEER_MAC,4'd3)))  S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd10) && (W_r_byte != f_mac_byte(P_PEER_MAC,4'd4)))  S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd11) && (W_r_byte != f_mac_byte(P_PEER_MAC,4'd5)))  S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd12) && (W_r_byte != P_ETHERTYPE[15:8]))            S_r_err <= 1'b1;
                    if ((S_r_idx == 16'd13) && (W_r_byte != P_ETHERTYPE[7:0]))             S_r_err <= 1'b1;
                    S_r_idx   <= S_r_idx + 16'd1;
                    S_rapp_i  <= S_rapp_i + 12'd1;
                        if (S_r_idx == (P_ETH_HDR-1)) begin
                            S_rapp_i <= 12'd0;
                            S_r_state <= R_APP;
                        end
                    end
                end
            end

            R_APP: begin
                if (!I_gmii_rx_dv) begin
                    S_r_err <= 1'b1; S_r_state <= R_IDLE;
                end
                else begin
                    if (I_gmii_rx_er)
                        S_r_err <= 1'b1;
                    if (S_rapp_i >= P_MAX_BYTES[11:0]) begin
                    S_r_err <= 1'b1; S_r_state <= R_IDLE;
                    end
                    else begin
                    S_rx_mem[S_r_buf*P_MAX_BYTES + S_rapp_i[10:0]] <= W_r_byte;
                    S_r_crc <= f_crc32_byte(S_r_crc, W_r_byte);
                    if (S_rapp_i == 16'd0)  S_r_magic[0] <= (W_r_byte == P_APP_MAGIC[7:0]);
                    if (S_rapp_i == 16'd1)  S_r_magic[1] <= (W_r_byte == P_APP_MAGIC[15:8]);
                    if (S_rapp_i == 16'd2)  S_r_line_rx[7:0]   <= W_r_byte;
                    if (S_rapp_i == 16'd3)  S_r_line_rx[15:8]  <= W_r_byte;
                    if (S_rapp_i == 16'd6)  S_r_len_cur[7:0]   <= W_r_byte;
                    if (S_rapp_i == 16'd7)  S_r_len_cur[15:8]  <= W_r_byte;
                    // Loaded at byte 8, not byte 7: W_r_flen is built from
                    // S_r_len_cur, and byte 7 only ASSIGNS its high half, so
                    // during byte 7 W_r_flen still holds the PREVIOUS frame's
                    // length.  By byte 8 it is complete.
                    // S_r_rem then equals (flen - k) at byte k, so the frame ends
                    // when it reaches 1 at byte flen-1.  flen >= 16 by contract
                    // (16-byte header + payload), so byte 8 is never the last byte
                    // and no special case is needed.
                    if (S_rapp_i == 16'd8) begin
                        S_r_len_ld <= 1'b1;
                        S_r_rem    <= W_r_flen[11:0] - 12'd9;
                        // Length-range validity is registered HERE, while the
                        // length is already being consumed.  Left combinational
                        // it sat on the verdict path as `length -> +16 adder ->
                        // two comparators -> publish toggle`, the worst path of
                        // the RX domain (8.681 ns of an 8.000 ns budget).
                        S_r_len_ok <= (W_r_flen >= P_HDR_BYTES) &&
                                      (W_r_flen <= P_MAX_BYTES);
                    end
                    else if (S_r_len_ld) S_r_rem <= S_r_rem - 12'd1;

                    S_r_idx   <= S_r_idx + 16'd1;

                    S_rapp_i  <= S_rapp_i + 12'd1;
                    // Only after the length has been latched can the end of the
                    // application frame be recognised (otherwise S_r_len_cur is
                    // still the previous frame's value and the frame ends after
                    // the first byte).
                        if (S_r_len_ld && (S_r_rem == 12'd1)) begin
                        // When the data field already meets the 46-byte
                        //   Ethernet minimum there is NO padding, and the FCS
                        //   must start immediately.  Going through R_PAD with
                        //   pad==0 still consumed one byte (the first FCS byte)
                        //   and folded it into the CRC, so every real frame
                        //   failed its FCS check.
                        if (W_r_flen >= P_MIN_DATA) begin
                            S_r_fcs_idx <= 3'd0;
                            S_r_state   <= R_FCS;
                        end
                        else begin
                            S_r_pad   <= P_MIN_DATA - W_r_flen;
                            S_r_state <= R_PAD;
                        end
                        end
                    end
                end
            end

            R_PAD: begin
                if (!I_gmii_rx_dv) begin
                    S_r_err <= 1'b1; S_r_state <= R_IDLE;
                end
                else begin
                    if (I_gmii_rx_er)
                        S_r_err <= 1'b1;
                    S_r_crc <= f_crc32_byte(S_r_crc, W_r_byte);
                    if (S_r_pad <= 16'd1) begin
                        S_r_pad     <= 16'd0;
                        S_r_fcs_idx <= 3'd0;
                        S_r_state   <= R_FCS;
                    end
                    else S_r_pad <= S_r_pad - 16'd1;
                end
            end

            R_FCS: begin
                if (!I_gmii_rx_dv) begin
                    S_r_err <= 1'b1; S_r_state <= R_IDLE;
                end
                else begin
                    if (I_gmii_rx_er)
                        S_r_err <= 1'b1;
                    case (S_r_fcs_idx)
                        3'd0: S_r_fcs0 <= W_r_byte;
                        3'd1: S_r_fcs1 <= W_r_byte;
                        3'd2: S_r_fcs2 <= W_r_byte;
                        default: S_r_fcs3 <= W_r_byte;
                    endcase
                    if (S_r_fcs_idx == 3'd3) begin
                        S_r_fcs_idx <= 3'd0;
                        S_r_fcs_end <= 1'b1;
                        S_r_state   <= R_IDLE;
                    end
                    else S_r_fcs_idx <= S_r_fcs_idx + 3'd1;
                end
            end

            default: S_r_state <= R_IDLE;
            endcase
        end
    end

    //=========================================================================
    // 5) FCS verdict, publication and counters (rx domain)
    //=========================================================================
    always @(posedge I_clk_rx or negedge I_rst_rx_n) begin
        if (!I_rst_rx_n) begin
            S_r_bad_count <= 32'd0;
            S_r_drop_count <= 32'd0;
            S_r_frame_count <= 32'd0;
            S_r_last_line_rx <= 16'd0;
            S_rx_len_rx_0 <= 16'd0;
            S_rx_len_rx_1 <= 16'd0;
            S_rx_tog_rx   <= 2'd0;
            S_r_buf       <= 1'b0;
        end
        else begin
            if (S_r_fcs_end) begin
                if (((~S_r_crc) == W_r_fcs_rx) &&
                    (S_r_magic == 2'b11) && !S_r_err &&
                    S_r_len_ok) begin
                    // the buffer the frame was written into must still be free
                    if (S_rx_tog_rx[S_r_buf] == S_rx_rel_s2[S_r_buf]) begin
                        if (S_r_buf) S_rx_len_rx_1 <= W_r_flen;
                        else         S_rx_len_rx_0 <= W_r_flen;
                        S_rx_tog_rx[S_r_buf] <= ~S_rx_tog_rx[S_r_buf];
                        S_r_buf              <= ~S_r_buf;
                        S_r_last_line_rx     <= S_r_line_rx;
                        S_r_frame_count      <= S_r_frame_count + 32'd1;
                    end
                    else begin
                        S_r_drop_count <= S_r_drop_count + 32'd1;
                        S_r_buf        <= ~S_r_buf;   // frame is discarded
                    end
                end
                else begin
                    S_r_bad_count <= S_r_bad_count + 32'd1;
                end
            end
        end
    end

    //=========================================================================
    // 6) application-side synchronisers and read ports
    //=========================================================================
    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            S_rx_tog_app_s1 <= 2'd0; S_rx_tog_app_s2 <= 2'd0;
            S_rx_len_s1_0 <= 16'd0; S_rx_len_s1_1 <= 16'd0;
            S_rx_len_s2_0 <= 16'd0; S_rx_len_s2_1 <= 16'd0;
        end
        else begin
            S_rx_tog_app_s1 <= S_rx_tog_rx;
            S_rx_tog_app_s2 <= S_rx_tog_app_s1;
            S_rx_len_s1_0 <= S_rx_len_rx_0;  S_rx_len_s2_0 <= S_rx_len_s1_0;
            S_rx_len_s1_1 <= S_rx_len_rx_1;  S_rx_len_s2_1 <= S_rx_len_s1_1;
        end
    end

    always @(posedge I_clk_rx or negedge I_rst_rx_n) begin
        if (!I_rst_rx_n) begin S_rx_rel_s1 <= 2'd0; S_rx_rel_s2 <= 2'd0; end
        else begin
            S_rx_rel_s1 <= S_rx_rel_tog_app;
            S_rx_rel_s2 <= S_rx_rel_s1;
        end
    end

    // ★ Both sides of this comparison must live in the APPLICATION domain.
    //   Using S_rx_rel_s2 (the rx-domain synchroniser of the release toggle)
    //   made O_rx_valid a combinational function of an rx-domain register:
    //   the application could observe a stale "held" state (and hence a stale
    //   buffer and length) for a frame it had already released, which showed
    //   up as recv_frame() reading the previous frame's data.
    wire W_rx_held0 = (S_rx_tog_app_s2[0] != S_rx_rel_tog_app[0]);
    wire W_rx_held1 = (S_rx_tog_app_s2[1] != S_rx_rel_tog_app[1]);

    assign O_rx_valid = W_rx_held0 | W_rx_held1;
    assign O_rx_rbuf  = W_rx_held0 ? 1'b0 : 1'b1;
    assign O_rx_len   = W_rx_held0 ? S_rx_len_s2_0 : S_rx_len_s2_1;

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) S_rx_rel_tog_app <= 2'd0;
        else if (I_rx_release) S_rx_rel_tog_app[O_rx_rbuf] <= ~S_rx_rel_tog_app[O_rx_rbuf];
    end

    always @(posedge I_clk_app)
        S_rx_rdata <= S_rx_mem[O_rx_rbuf*P_MAX_BYTES + I_rx_addr];
    assign O_rx_rdata = S_rx_rdata;

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            S_rx_bad_s1 <= 32'd0; S_rx_bad_s2 <= 32'd0;
            S_rx_frame_s1 <= 32'd0; S_rx_frame_s2 <= 32'd0;
            S_rx_drop_s1 <= 32'd0; S_rx_drop_s2 <= 32'd0;
            S_rx_line_s1 <= 16'd0; S_rx_line_s2 <= 16'd0;
        end
        else begin
            S_rx_bad_s1   <= S_r_bad_count;    S_rx_bad_s2   <= S_rx_bad_s1;
            S_rx_frame_s1 <= S_r_frame_count;  S_rx_frame_s2 <= S_rx_frame_s1;
            S_rx_drop_s1  <= S_r_drop_count;   S_rx_drop_s2  <= S_rx_drop_s1;
            S_rx_line_s1  <= S_r_last_line_rx; S_rx_line_s2  <= S_rx_line_s1;
        end
    end

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            O_rx_frame_count     <= 32'd0;
            O_rx_bad_count       <= 32'd0;
            O_rx_busy_drop_count <= 32'd0;
            O_rx_last_line       <= 16'd0;
        end
        else begin
            O_rx_frame_count     <= S_rx_frame_s2;
            O_rx_bad_count       <= S_rx_bad_s2;
            O_rx_busy_drop_count <= S_rx_drop_s2;
            O_rx_last_line       <= S_rx_line_s2;
        end
    end

endmodule
