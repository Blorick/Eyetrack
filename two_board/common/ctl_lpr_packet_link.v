`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// Module: ctl_lpr_packet_link
// Function: raw Ethernet request/response link with bounded packet RAM
// Domain: ctl; application, GMII TX, and GMII RX clock domains
// Clock: I_clk_app, I_clk_tx, and I_clk_rx are independent
// Reset: common active-low board reset; assertion is asynchronous, release is synchronous
// Parameters: local/peer MAC addresses and the fixed application EtherType
// Verification: tb_ctl_lpr_packet_link.sv plus fixed golden GMII/FCS vectors
// -----------------------------------------------------------------------------
module ctl_lpr_packet_link #(
    parameter [47:0] P_LOCAL_MAC = 48'h02_00_00_12_34_56,
    parameter [47:0] P_PEER_MAC = 48'h02_00_00_65_43_21,
    parameter [15:0] P_ETHERTYPE = 16'h88B5,
    // Kept on the endpoint for a common PH1P/EG4D instantiation contract.
    // Arbitration policy is implemented by ctl_gmii_tx_scheduler.
    parameter integer P_TX_SCHEDULE = 0,
    // Legacy standalone users leave this at zero and need no grant wiring.
    parameter integer P_TX_ARB_EN = 0
) (
    input I_clk_app,
    input I_rst_app_n,
    input I_clk_tx,
    input I_rst_tx_n,
    input I_clk_rx,
    input I_rst_rx_n,

    input I_tx_write_en,
    input [9:0] I_tx_write_addr,
    input [7:0] I_tx_write_data,
    input I_tx_commit,
    input [10:0] I_tx_length,
    output O_tx_busy,
    output reg O_tx_done,
    input I_tx_grant,
    output O_tx_request,
    output reg O_tx_done_tx,

    output O_rx_valid,
    output reg [10:0] O_rx_length,
    input [9:0] I_rx_read_addr,
    output reg [7:0] O_rx_read_data,
    input I_rx_release,

    output [7:0] O_gmii_txd,
    output O_gmii_tx_en,
    output O_gmii_tx_er,
    input [7:0] I_gmii_rxd,
    input I_gmii_rx_dv,
    input I_gmii_rx_er,

    output reg [31:0] O_tx_busy_drop_count,
    output reg [31:0] O_tx_invalid_drop_count,
    output reg [31:0] O_rx_bad_frame_count,
    output reg [31:0] O_rx_busy_drop_count,
    output reg [31:0] O_rx_oversize_drop_count,
    output [31:0] O_bad_frame_count,
    output [31:0] O_busy_drop_count,

    // 2026-10-18 上板观测口（只读，不参与任何收发逻辑）。位定义：
    //   [0] 前导码/帧首检查失败   [1] MAC/EtherType 头检查失败  [2] FCS 检查失败
    //   [3] 长度/形状检查失败     [4] 应用头契约失败            [5] 补齐字节非 0
    //   [6] 帧内出现过 RX_ER      [7] 至少有一帧完整通过
    // 伴随输出为最近一帧收到的应用字节数（正常 kind-2 帧应为 46）。
    output reg  [7:0]  O_rx_dbg_flags,
    output reg  [10:0] O_rx_dbg_app_count
);

    localparam integer P_MAX_APP_BYTES_N = 1024;
    localparam [10:0] P_MAX_APP_BYTES_W = 11'd1024;
    localparam [10:0] P_ETH_HEADER_BYTES_N = 11'd14;
    localparam [10:0] P_MIN_ETH_DATA_BYTES_N = 11'd46;
    localparam [3:0] P_PREAMBLE_BYTES_N = 4'd8;
    localparam [3:0] P_IFG_BYTES_N = 4'd12;

    localparam [2:0] ST_TX_IDLE = 3'd0;
    localparam [2:0] ST_TX_PREAMBLE = 3'd1;
    localparam [2:0] ST_TX_FRAME = 3'd2;
    localparam [2:0] ST_TX_FCS = 3'd3;
    localparam [2:0] ST_TX_IFG = 3'd4;

    localparam [1:0] ST_RX_IDLE = 2'd0;
    localparam [1:0] ST_RX_PREAMBLE = 2'd1;
    localparam [1:0] ST_RX_BODY = 2'd2;

    // Both payload mailboxes are 1024x8 simple dual-port memories: one port is
    // written in the peer clock domain, the other is read back registered in
    // the application clock domain.  The attribute is the TD RAM candidate
    // marker already used by this project (see char_match.v / char_locate.v);
    // without it TD maps the two arrays to 8192 flip-flops plus 1632 f7/f8
    // muxes, which alone overflows the PH1P35 slice budget of the two-board
    // master build.  Read-during-write collision is impossible by protocol:
    // the mailbox publishes busy/hold before the peer can touch the same
    // address, so the RAM write mode does not change behaviour.
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [7:0] S_tx_mem [0:P_MAX_APP_BYTES_N - 1];
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [7:0] S_rx_mem [0:P_MAX_APP_BYTES_N - 1];

    reg S_tx_busy_app;
    reg S_tx_request_app;
    reg [10:0] S_tx_length_app;
    reg S_tx_ack_s1_app;
    reg S_tx_ack_s2_app;
    reg S_tx_ack_seen_app;
    reg S_tx_completion_wait_app;

    reg S_rx_release_app;
    reg S_rx_hold_s1_app;
    reg S_rx_hold_s2_app;
    reg S_rx_hold_s3_app;
    reg S_rx_hold_s4_app;
    reg [10:0] S_rx_length_s1_app;
    reg [10:0] S_rx_length_s2_app;
    reg S_rx_hold_valid;
    reg [10:0] S_rx_length_hold;

    reg S_tx_request_s1_tx;
    reg S_tx_request_s2_tx;
    reg S_tx_request_seen_tx;
    reg [10:0] S_tx_length_s1_tx;
    reg [10:0] S_tx_length_s2_tx;
    reg S_tx_ack_tx;
    reg S_tx_start_wait_tx;
    reg [10:0] S_tx_length_latched_tx;
    reg [10:0] S_tx_frame_length_tx;
    reg [2:0] S_tx_state;
    reg [3:0] S_tx_preamble_pos;
    reg [10:0] S_tx_frame_pos;
    reg [1:0] S_tx_fcs_pos;
    reg [3:0] S_tx_ifg_pos;
    reg [31:0] S_tx_crc;
    reg [31:0] S_tx_fcs;

    reg S_rx_release_s1_rx;
    reg S_rx_release_s2_rx;
    reg [1:0] S_rx_state;
    reg S_rx_active;
    reg [3:0] S_rx_preamble_pos;
    reg S_rx_preamble_ok;
    reg S_rx_header_ok;
    reg S_rx_error_seen;
    reg S_rx_overflow;
    reg S_rx_accept_frame;
    reg [2:0] S_rx_tail_count;
    reg [7:0] S_rx_tail0;
    reg [7:0] S_rx_tail1;
    reg [7:0] S_rx_tail2;
    reg [7:0] S_rx_tail3;
    reg [31:0] S_rx_crc;
    reg [10:0] S_rx_body_pos;
    reg [10:0] S_rx_app_count;
    reg [7:0] S_rx_app_magic0;
    reg [7:0] S_rx_app_magic1;
    reg [7:0] S_rx_app_version;
    reg [7:0] S_rx_app_kind;
    reg [7:0] S_rx_app_payload_hi;
    reg [7:0] S_rx_app_payload_lo;
    reg S_rx_app_mask_msb;
    reg [7:0] S_rx_app_flags;
    reg S_rx_padding_ok;
    // 2026-10-11: registered copy of the header-contract verdict.  Its inputs
    // (S_rx_app_magic0/1, version, kind, payload_hi/lo, mask_msb, flags) are
    // written at fixed byte indices 0..3 and 12..15 and never change again
    // within the frame, so the registered value equals the combinational one
    // at every cycle from byte 16 to frame end.  Registering it removes the
    // payload-length compare from the frame-complete cone; that cone
    // (W_rx_complete_ok -> S_rx_hold_valid / the drop counters) was the
    // eg4d_eth_rxc 125 MHz critical path once the EG4D netlist changed.
    reg S_rx_header_contract_ok_q;
    // Payload length settles after application-header byte 13.  Register its
    // padded wire length before the frame-complete check: any frame shorter
    // than 46 application bytes fails shape validation in either version.
    // Longer frames have already had this value stable for over 30 cycles.
    reg [15:0] S_rx_wire_data_length_q;

    reg [7:0] W_tx_data_mux;
    // TX payload mailbox read and pipeline.  The read address leads the
    // transmitted byte by two TX cycles, so the ERAM output register is
    // followed by a fabric pipeline register before the GMII data mux and the
    // CRC.  Post-route STA on the EG4D20 showed the ERAM clock-to-out alone at
    // 3.245 ns of the 8 ns GMII cycle, leaving too little for the CRC when the
    // ERAM output drove it directly.  Frame position p carries payload index
    // k = p - 14, so address k is presented at position k + 12.
    wire [9:0] W_tx_payload_addr = S_tx_frame_pos[9:0] - 10'd12;
    reg  [7:0] S_tx_mem_q;
    reg        S_tx_pad_q;
    reg  [7:0] S_tx_mem_data_q;
    // Evaluated one cycle ahead: W_tx_next_pad(x) is the pad condition for the
    // byte transmitted at position x + 2.
    wire       W_tx_next_pad =
        (S_tx_frame_pos >= 11'd12) &&
        (S_tx_frame_pos <= S_tx_frame_length_tx - 11'd3) &&
        ((S_tx_frame_pos - 11'd12) >= S_tx_length_latched_tx);
    wire [7:0] S_tx_mem_data = S_tx_mem_data_q;
    wire [9:0] W_rx_mem_addr = S_rx_app_count[9:0];
    wire [15:0] W_rx_payload_length = {S_rx_app_payload_hi, S_rx_app_payload_lo};
    wire [15:0] W_rx_app_length = 16'd16 + W_rx_payload_length;
    wire [15:0] W_rx_wire_data_length =
        (W_rx_app_length < 16'd46) ? 16'd46 : W_rx_app_length;
    wire W_rx_header_contract_ok =
        (S_rx_app_magic0 == 8'h4C) &&
        (S_rx_app_magic1 == 8'h50) &&
        (S_rx_app_version == 8'd1) &&
        ((S_rx_app_kind == 8'd1) || (S_rx_app_kind == 8'd2)) &&
        ((S_rx_app_kind == 8'd1 && W_rx_payload_length == 16'd896) ||
         (S_rx_app_kind == 8'd2 && W_rx_payload_length == 16'd16)) &&
        !S_rx_app_mask_msb &&
        ((S_rx_app_flags & 8'hFE) == 8'h00);
    // Registered form used by the frame-complete cone (see the declaration
    // comment).  Same value, one cycle of extra latency that is free here.
    wire W_rx_header_contract_ok_q = S_rx_header_contract_ok_q;
    wire W_rx_padding_ok = S_rx_padding_ok;
    wire W_rx_crc_ok =
        (S_rx_tail_count == 3'd4) &&
        ((~S_rx_crc) == {S_rx_tail3, S_rx_tail2, S_rx_tail1, S_rx_tail0});
    wire W_rx_shape_ok =
        (S_rx_tail_count == 3'd4) &&
        (S_rx_body_pos >= P_ETH_HEADER_BYTES_N) &&
        ({5'd0, S_rx_app_count} == S_rx_wire_data_length_q);
    wire W_rx_complete_ok =
        S_rx_accept_frame && S_rx_preamble_ok && S_rx_header_ok &&
        !S_rx_error_seen && !S_rx_overflow && W_rx_crc_ok &&
        W_rx_shape_ok && W_rx_header_contract_ok_q && W_rx_padding_ok;

    function [7:0] f_mac_byte;
        input [47:0] I_mac;
        input [10:0] I_index;
        begin
            case (I_index)
                11'd0: f_mac_byte = I_mac[47:40];
                11'd1: f_mac_byte = I_mac[39:32];
                11'd2: f_mac_byte = I_mac[31:24];
                11'd3: f_mac_byte = I_mac[23:16];
                11'd4: f_mac_byte = I_mac[15:8];
                11'd5: f_mac_byte = I_mac[7:0];
                default: f_mac_byte = 8'h00;
            endcase
        end
    endfunction

    // Ethernet CRC-32 (reflected polynomial 0xEDB88320), nibble-table form.
    // The original eight-step per-bit loop put eight dependent XOR stages
    // between the payload-RAM output register and the CRC/FCS registers.  On
    // the EG4D20 slave that path missed the 125 MHz GMII TX clock by 0.914 ns
    // after place and route, so the low nibble and then the high nibble are
    // now folded through a 16-entry constant table.  This is the standard
    // CRC-32 nibble table and is arithmetically identical to the bit loop;
    // f_crc32_byte keeps its interface, so no call site changes.
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
        input [7:0] I_data;
        reg [31:0] W_crc_mid;
        begin
            W_crc_mid = (I_crc >> 4) ^
                f_crc32_nibble(I_crc[3:0] ^ I_data[3:0]);
            f_crc32_byte = (W_crc_mid >> 4) ^
                f_crc32_nibble(W_crc_mid[3:0] ^ I_data[7:4]);
        end
    endfunction

    // The 14 Ethernet header bytes are constants derived from the MAC and
    // EtherType parameters, so their CRC is an elaboration-time constant.
    // Seeding the running CRC with it keeps the frame-position decode and the
    // transmit byte mux out of the CRC datapath: on the EG4D20 that combined
    // path (seven LUT levels, mostly routing) missed the 125 MHz GMII TX clock
    // by 0.680 ns after place and route.  The payload/pad bytes still go
    // through f_crc32_byte one at a time, and the frame length is
    // 14 + max(46, length) >= 60, so the last transmitted byte is never a
    // header byte.
    function [31:0] f_tx_header_crc;
        input I_seed_unused;
        integer I_hdr_byte;
        reg [31:0] W_hdr_crc;
        begin
            W_hdr_crc = 32'hFFFF_FFFF;
            for (I_hdr_byte = 0; I_hdr_byte < 6; I_hdr_byte = I_hdr_byte + 1) begin
                W_hdr_crc = f_crc32_byte(W_hdr_crc,
                    f_mac_byte(P_PEER_MAC, I_hdr_byte[10:0]));
            end
            for (I_hdr_byte = 0; I_hdr_byte < 6; I_hdr_byte = I_hdr_byte + 1) begin
                W_hdr_crc = f_crc32_byte(W_hdr_crc,
                    f_mac_byte(P_LOCAL_MAC, I_hdr_byte[10:0]));
            end
            W_hdr_crc = f_crc32_byte(W_hdr_crc, P_ETHERTYPE[15:8]);
            W_hdr_crc = f_crc32_byte(W_hdr_crc, P_ETHERTYPE[7:0]);
            f_tx_header_crc = W_hdr_crc;
        end
    endfunction
    // Default parameters (peer 02:00:00:65:43:21, local 02:00:00:12:34:56,
    // EtherType 0x88B5) evaluate to 32'h0982_F987.  TD requires every function
    // to carry at least one input port, hence the unused seed argument.
    localparam [31:0] P_TX_HDR_CRC = f_tx_header_crc(1'b0);

    assign O_tx_busy = S_tx_busy_app;
    // This is in the TX clock domain.  A request stays high from the first
    // synchronized commit until the frame's IFG has completed.  It drops on
    // the same edge as O_tx_done_tx, before the app-domain acknowledge has
    // returned, so the shared scheduler cannot re-grant a completed packet.
    assign O_tx_request =
        (S_tx_state != ST_TX_IDLE) ||
        (S_tx_start_wait_tx && S_tx_request_s2_tx) ||
        (S_tx_request_s2_tx && !S_tx_ack_tx);
    // Short board-facing aliases for the receive diagnostics.  Detailed
    // counters remain available above for separate TX/RX probes.
    assign O_bad_frame_count = O_rx_bad_frame_count;
    assign O_busy_drop_count = O_rx_busy_drop_count;
    assign O_rx_valid = S_rx_hold_s4_app;
    assign O_gmii_tx_en =
        (S_tx_state == ST_TX_PREAMBLE) ||
        (S_tx_state == ST_TX_FRAME) ||
        (S_tx_state == ST_TX_FCS);
    assign O_gmii_tx_er = 1'b0;
    assign O_gmii_txd = W_tx_data_mux;

    always @(*) begin
        W_tx_data_mux = 8'h00;
        case (S_tx_state)
            ST_TX_PREAMBLE: begin
                if (S_tx_preamble_pos == 4'd7) begin
                    W_tx_data_mux = 8'hD5;
                end else begin
                    W_tx_data_mux = 8'h55;
                end
            end
            ST_TX_FRAME: begin
                if (S_tx_frame_pos < 11'd6) begin
                    W_tx_data_mux = f_mac_byte(P_PEER_MAC, S_tx_frame_pos);
                end else if (S_tx_frame_pos < 11'd12) begin
                    W_tx_data_mux = f_mac_byte(P_LOCAL_MAC, S_tx_frame_pos - 11'd6);
                end else if (S_tx_frame_pos == 11'd12) begin
                    W_tx_data_mux = P_ETHERTYPE[15:8];
                end else if (S_tx_frame_pos == 11'd13) begin
                    W_tx_data_mux = P_ETHERTYPE[7:0];
                end else begin
                    W_tx_data_mux = S_tx_mem_data;
                end
            end
            ST_TX_FCS: begin
                case (S_tx_fcs_pos)
                    2'd0: W_tx_data_mux = S_tx_fcs[7:0];
                    2'd1: W_tx_data_mux = S_tx_fcs[15:8];
                    2'd2: W_tx_data_mux = S_tx_fcs[23:16];
                    default: W_tx_data_mux = S_tx_fcs[31:24];
                endcase
            end
            default: begin
                W_tx_data_mux = 8'h00;
            end
        endcase
    end

    // The three reset inputs are intended to be driven from one board reset
    // synchronizer: asynchronous assertion is safe here, while each clock
    // domain must release reset only on its own clock edge.
    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            S_tx_busy_app <= 1'b0;
            S_tx_request_app <= 1'b0;
            S_tx_length_app <= 11'd0;
            S_tx_ack_s1_app <= 1'b0;
            S_tx_ack_s2_app <= 1'b0;
            S_tx_ack_seen_app <= 1'b0;
            S_tx_completion_wait_app <= 1'b0;
            S_rx_release_app <= 1'b0;
            S_rx_hold_s1_app <= 1'b0;
            S_rx_hold_s2_app <= 1'b0;
            S_rx_hold_s3_app <= 1'b0;
            S_rx_hold_s4_app <= 1'b0;
            S_rx_length_s1_app <= 11'd0;
            S_rx_length_s2_app <= 11'd0;
            O_tx_done <= 1'b0;
            O_tx_busy_drop_count <= 32'd0;
            O_tx_invalid_drop_count <= 32'd0;
            O_rx_length <= 11'd0;
            O_rx_read_data <= 8'h00;
        end else begin
            S_tx_ack_s1_app <= S_tx_ack_tx;
            S_tx_ack_s2_app <= S_tx_ack_s1_app;
            S_rx_hold_s1_app <= S_rx_hold_valid;
            S_rx_hold_s2_app <= S_rx_hold_s1_app;
            S_rx_hold_s3_app <= S_rx_hold_s2_app;
            S_rx_hold_s4_app <= S_rx_hold_s3_app;
            S_rx_length_s1_app <= S_rx_length_hold;
            S_rx_length_s2_app <= S_rx_length_s1_app;
            O_tx_done <= 1'b0;

            if (I_tx_write_en && !S_tx_busy_app) begin
                S_tx_mem[I_tx_write_addr] <= I_tx_write_data;
            end

            if (S_tx_ack_s2_app && !S_tx_ack_seen_app) begin
                S_tx_ack_seen_app <= 1'b1;
                S_tx_request_app <= 1'b0;
                S_tx_completion_wait_app <= 1'b1;
            end else if (!S_tx_ack_s2_app && S_tx_completion_wait_app) begin
                S_tx_ack_seen_app <= 1'b0;
                S_tx_completion_wait_app <= 1'b0;
                S_tx_busy_app <= 1'b0;
                O_tx_done <= 1'b1;
            end else if (!S_tx_ack_s2_app) begin
                S_tx_ack_seen_app <= 1'b0;
            end

            if (I_tx_commit) begin
                if (S_tx_busy_app) begin
                    O_tx_busy_drop_count <= O_tx_busy_drop_count + 32'd1;
                end else if (I_tx_length <= P_MAX_APP_BYTES_W) begin
                    S_tx_length_app <= I_tx_length;
                    S_tx_request_app <= 1'b1;
                    S_tx_busy_app <= 1'b1;
                end else begin
                    O_tx_invalid_drop_count <= O_tx_invalid_drop_count + 32'd1;
                end
            end

            if (I_rx_release && S_rx_hold_s2_app) begin
                S_rx_release_app <= 1'b1;
            end else if (!S_rx_hold_s2_app) begin
                S_rx_release_app <= 1'b0;
            end

            if (S_rx_hold_s3_app && !S_rx_hold_s4_app) begin
                O_rx_length <= S_rx_length_s2_app;
            end
            O_rx_read_data <= S_rx_mem[I_rx_read_addr];
        end
    end

    always @(posedge I_clk_tx or negedge I_rst_tx_n) begin
        if (!I_rst_tx_n) begin
            S_tx_request_s1_tx <= 1'b0;
            S_tx_request_s2_tx <= 1'b0;
            S_tx_request_seen_tx <= 1'b0;
            S_tx_length_s1_tx <= 11'd0;
            S_tx_length_s2_tx <= 11'd0;
            S_tx_ack_tx <= 1'b0;
            S_tx_start_wait_tx <= 1'b0;
            S_tx_length_latched_tx <= 11'd0;
            S_tx_frame_length_tx <= 11'd0;
            S_tx_state <= ST_TX_IDLE;
            S_tx_preamble_pos <= 4'd0;
            S_tx_frame_pos <= 11'd0;
            S_tx_fcs_pos <= 2'd0;
            S_tx_ifg_pos <= 4'd0;
            S_tx_crc <= 32'hFFFF_FFFF;
            S_tx_fcs <= 32'd0;
            S_tx_mem_q <= 8'h00;
            S_tx_pad_q <= 1'b0;
            S_tx_mem_data_q <= 8'h00;
            O_tx_done_tx <= 1'b0;
        end else begin
            S_tx_request_s1_tx <= S_tx_request_app;
            S_tx_request_s2_tx <= S_tx_request_s1_tx;
            S_tx_length_s1_tx <= S_tx_length_app;
            S_tx_length_s2_tx <= S_tx_length_s1_tx;
            S_tx_mem_q <= S_tx_mem[W_tx_payload_addr];
            S_tx_pad_q <= W_tx_next_pad;
            S_tx_mem_data_q <= S_tx_pad_q ? 8'h00 : S_tx_mem_q;
            O_tx_done_tx <= 1'b0;

            if (!S_tx_request_s2_tx) begin
                S_tx_request_seen_tx <= 1'b0;
                S_tx_start_wait_tx <= 1'b0;
                S_tx_ack_tx <= 1'b0;
                S_tx_state <= ST_TX_IDLE;
            end else begin
                if (!S_tx_request_seen_tx) begin
                    S_tx_request_seen_tx <= 1'b1;
                    S_tx_start_wait_tx <= 1'b1;
                end

                if (S_tx_start_wait_tx && (S_tx_state == ST_TX_IDLE) &&
                    ((P_TX_ARB_EN == 0) || I_tx_grant)) begin
                    S_tx_start_wait_tx <= 1'b0;
                    // The request has been synchronized for one full TX
                    // cycle before this bundled length is consumed.  Keep
                    // the app-side length frozen until the ack returns.
                    S_tx_length_latched_tx <= S_tx_length_s2_tx;
                    S_tx_frame_length_tx <= P_ETH_HEADER_BYTES_N +
                        ((S_tx_length_s2_tx < P_MIN_ETH_DATA_BYTES_N) ?
                         P_MIN_ETH_DATA_BYTES_N : S_tx_length_s2_tx);
                    S_tx_preamble_pos <= 4'd0;
                    S_tx_frame_pos <= 11'd0;
                    S_tx_crc <= 32'hFFFF_FFFF;
                    S_tx_state <= ST_TX_PREAMBLE;
                end else begin
                    case (S_tx_state)
                        ST_TX_PREAMBLE: begin
                            if (S_tx_preamble_pos == P_PREAMBLE_BYTES_N - 1'b1) begin
                                S_tx_preamble_pos <= 4'd0;
                                S_tx_frame_pos <= 11'd0;
                                S_tx_crc <= 32'hFFFF_FFFF;
                                S_tx_state <= ST_TX_FRAME;
                            end else begin
                                S_tx_preamble_pos <= S_tx_preamble_pos + 4'd1;
                            end
                        end
                        ST_TX_FRAME: begin
                            if (S_tx_frame_pos == S_tx_frame_length_tx - 1'b1) begin
                                S_tx_fcs <= ~f_crc32_byte(S_tx_crc, S_tx_mem_data);
                                S_tx_fcs_pos <= 2'd0;
                                S_tx_state <= ST_TX_FCS;
                            end else begin
                                // Positions 0..12 are already covered by the
                                // header CRC seed loaded at position 13; from
                                // position 14 on the transmitted byte is the
                                // registered mailbox byte.
                                if (S_tx_frame_pos == 11'd13) begin
                                    S_tx_crc <= P_TX_HDR_CRC;
                                end else if (S_tx_frame_pos >= 11'd14) begin
                                    S_tx_crc <= f_crc32_byte(S_tx_crc, S_tx_mem_data);
                                end
                                S_tx_frame_pos <= S_tx_frame_pos + 11'd1;
                            end
                        end
                        ST_TX_FCS: begin
                            if (S_tx_fcs_pos == 2'd3) begin
                                S_tx_fcs_pos <= 2'd0;
                                S_tx_ifg_pos <= 4'd0;
                                S_tx_state <= ST_TX_IFG;
                            end else begin
                                S_tx_fcs_pos <= S_tx_fcs_pos + 2'd1;
                            end
                        end
                        ST_TX_IFG: begin
                            if (S_tx_ifg_pos == P_IFG_BYTES_N - 1'b1) begin
                                S_tx_ifg_pos <= 4'd0;
                                S_tx_ack_tx <= 1'b1;
                                O_tx_done_tx <= 1'b1;
                                S_tx_state <= ST_TX_IDLE;
                            end else begin
                                S_tx_ifg_pos <= S_tx_ifg_pos + 4'd1;
                            end
                        end
                        default: begin
                            S_tx_state <= ST_TX_IDLE;
                        end
                    endcase
                end
            end
        end
    end

    // Header comparison and verdict accumulation are separate RX stages.
    // Complete-frame/public-mailbox latency remains unchanged.
    reg S_rx_header_bad_q;
    function [7:0] f_rx_eth_header_byte;
        input [10:0] index;
        begin
            case(index)
                0: f_rx_eth_header_byte=P_LOCAL_MAC[47:40];
                1: f_rx_eth_header_byte=P_LOCAL_MAC[39:32];
                2: f_rx_eth_header_byte=P_LOCAL_MAC[31:24];
                3: f_rx_eth_header_byte=P_LOCAL_MAC[23:16];
                4: f_rx_eth_header_byte=P_LOCAL_MAC[15:8];
                5: f_rx_eth_header_byte=P_LOCAL_MAC[7:0];
                6: f_rx_eth_header_byte=P_PEER_MAC[47:40];
                7: f_rx_eth_header_byte=P_PEER_MAC[39:32];
                8: f_rx_eth_header_byte=P_PEER_MAC[31:24];
                9: f_rx_eth_header_byte=P_PEER_MAC[23:16];
                10: f_rx_eth_header_byte=P_PEER_MAC[15:8];
                11: f_rx_eth_header_byte=P_PEER_MAC[7:0];
                12: f_rx_eth_header_byte=P_ETHERTYPE[15:8];
                default: f_rx_eth_header_byte=P_ETHERTYPE[7:0];
            endcase
        end
    endfunction
    always @(posedge I_clk_rx or negedge I_rst_rx_n) begin
        if(!I_rst_rx_n) begin
            S_rx_header_bad_q <= 1'b0;
            S_rx_header_ok <= 1'b1;
        end else begin
            S_rx_header_bad_q <= I_gmii_rx_dv && (S_rx_state == ST_RX_BODY) &&
                (S_rx_tail_count == 3'd4) && (S_rx_body_pos < 11'd14) &&
                (S_rx_tail0 != f_rx_eth_header_byte(S_rx_body_pos));
            if(I_gmii_rx_dv && !S_rx_active) begin
                S_rx_header_bad_q <= 1'b0;
                S_rx_header_ok <= 1'b1;
            end else if(S_rx_header_bad_q) begin
                S_rx_header_ok <= 1'b0;
            end
        end
    end

    always @(posedge I_clk_rx or negedge I_rst_rx_n) begin
        if (!I_rst_rx_n) begin
            S_rx_release_s1_rx <= 1'b0;
            S_rx_release_s2_rx <= 1'b0;
            S_rx_state <= ST_RX_IDLE;
            S_rx_active <= 1'b0;
            S_rx_preamble_pos <= 4'd0;
            S_rx_preamble_ok <= 1'b1;
            S_rx_error_seen <= 1'b0;
            S_rx_overflow <= 1'b0;
            S_rx_accept_frame <= 1'b0;
            S_rx_tail_count <= 3'd0;
            S_rx_tail0 <= 8'h00;
            S_rx_tail1 <= 8'h00;
            S_rx_tail2 <= 8'h00;
            S_rx_tail3 <= 8'h00;
            S_rx_crc <= 32'hFFFF_FFFF;
            S_rx_body_pos <= 11'd0;
            S_rx_app_count <= 11'd0;
            S_rx_app_magic0 <= 8'h00;
            S_rx_app_magic1 <= 8'h00;
            S_rx_app_version <= 8'h00;
            S_rx_app_kind <= 8'h00;
            S_rx_app_payload_hi <= 8'h00;
            S_rx_app_payload_lo <= 8'h00;
            S_rx_app_mask_msb <= 1'b0;
            S_rx_app_flags <= 8'h00;
            S_rx_padding_ok <= 1'b1;
            S_rx_header_contract_ok_q <= 1'b0;
            S_rx_wire_data_length_q <= 16'd46;
            S_rx_hold_valid <= 1'b0;
            S_rx_length_hold <= 11'd0;
            O_rx_bad_frame_count <= 32'd0;
            O_rx_busy_drop_count <= 32'd0;
            O_rx_oversize_drop_count <= 32'd0;
        end else begin
            S_rx_release_s1_rx <= S_rx_release_app;
            S_rx_release_s2_rx <= S_rx_release_s1_rx;
            // Header-contract verdict is constant from byte 16 onward, so this
            // register is already settled long before the frame-complete cone
            // samples it.
            S_rx_header_contract_ok_q <= W_rx_header_contract_ok;
            S_rx_wire_data_length_q <= W_rx_wire_data_length;

            if (S_rx_release_s2_rx) begin
                S_rx_hold_valid <= 1'b0;
            end

            if (I_gmii_rx_dv) begin
                if (!S_rx_active) begin
                    S_rx_active <= 1'b1;
                    S_rx_state <= ST_RX_PREAMBLE;
                    S_rx_preamble_pos <= 4'd1;
                    S_rx_preamble_ok <= (I_gmii_rxd == 8'h55);
                    S_rx_error_seen <= I_gmii_rx_er;
                    S_rx_overflow <= 1'b0;
                    S_rx_accept_frame <= !S_rx_hold_valid && !S_rx_release_s2_rx;
                    S_rx_tail_count <= 3'd0;
                    S_rx_crc <= 32'hFFFF_FFFF;
                    S_rx_body_pos <= 11'd0;
                    S_rx_app_count <= 11'd0;
                    S_rx_app_magic0 <= 8'h00;
                    S_rx_app_magic1 <= 8'h00;
                    S_rx_app_version <= 8'h00;
                    S_rx_app_kind <= 8'h00;
                    S_rx_app_payload_hi <= 8'h00;
                    S_rx_app_payload_lo <= 8'h00;
                    S_rx_app_mask_msb <= 1'b0;
                    S_rx_app_flags <= 8'h00;
                    S_rx_padding_ok <= 1'b1;
                    S_rx_wire_data_length_q <= 16'd46;
                end else if (S_rx_state == ST_RX_PREAMBLE) begin
                    if (I_gmii_rx_er) begin
                        S_rx_error_seen <= 1'b1;
                    end
                    if (S_rx_preamble_pos < 4'd7) begin
                        if (I_gmii_rxd != 8'h55) begin
                            S_rx_preamble_ok <= 1'b0;
                        end
                        S_rx_preamble_pos <= S_rx_preamble_pos + 4'd1;
                    end else begin
                        if (I_gmii_rxd != 8'hD5) begin
                            S_rx_preamble_ok <= 1'b0;
                        end
                        S_rx_state <= ST_RX_BODY;
                        S_rx_tail_count <= 3'd0;
                        S_rx_crc <= 32'hFFFF_FFFF;
                        S_rx_body_pos <= 11'd0;
                        S_rx_app_count <= 11'd0;
                    end
                end else begin
                    if (I_gmii_rx_er) begin
                        S_rx_error_seen <= 1'b1;
                    end
                    if (S_rx_tail_count < 3'd4) begin
                        case (S_rx_tail_count)
                            3'd0: S_rx_tail0 <= I_gmii_rxd;
                            3'd1: S_rx_tail1 <= I_gmii_rxd;
                            3'd2: S_rx_tail2 <= I_gmii_rxd;
                            default: S_rx_tail3 <= I_gmii_rxd;
                        endcase
                        S_rx_tail_count <= S_rx_tail_count + 3'd1;
                    end else begin
                        if ((S_rx_body_pos >= 11'd14) && S_rx_accept_frame) begin
                            case (S_rx_app_count)
                                11'd0: S_rx_app_magic0 <= S_rx_tail0;
                                11'd1: S_rx_app_magic1 <= S_rx_tail0;
                                11'd2: S_rx_app_version <= S_rx_tail0;
                                11'd3: S_rx_app_kind <= S_rx_tail0;
                                11'd12: S_rx_app_payload_hi <= S_rx_tail0;
                                11'd13: S_rx_app_payload_lo <= S_rx_tail0;
                                11'd14: S_rx_app_mask_msb <= S_rx_tail0[7];
                                11'd15: S_rx_app_flags <= S_rx_tail0;
                                default: begin
                                end
                            endcase
                            if ((S_rx_app_kind == 8'd2) &&
                                (S_rx_app_count >= 11'd32) &&
                                (S_rx_app_count < 11'd46) &&
                                (S_rx_tail0 != 8'h00)) begin
                                S_rx_padding_ok <= 1'b0;
                            end
                            if (S_rx_app_count < P_MAX_APP_BYTES_W) begin
                                S_rx_mem[W_rx_mem_addr] <= S_rx_tail0;
                                S_rx_app_count <= S_rx_app_count + 11'd1;
                            end else begin
                                S_rx_overflow <= 1'b1;
                            end
                        end
                        S_rx_crc <= f_crc32_byte(S_rx_crc, S_rx_tail0);
                        if (S_rx_body_pos != 11'h7FF) begin
                            S_rx_body_pos <= S_rx_body_pos + 11'd1;
                        end
                        S_rx_tail0 <= S_rx_tail1;
                        S_rx_tail1 <= S_rx_tail2;
                        S_rx_tail2 <= S_rx_tail3;
                        S_rx_tail3 <= I_gmii_rxd;
                    end
                end
            end else if (S_rx_active) begin
                S_rx_active <= 1'b0;
                S_rx_state <= ST_RX_IDLE;
                if (!S_rx_accept_frame) begin
                    O_rx_busy_drop_count <= O_rx_busy_drop_count + 32'd1;
                end else if (W_rx_complete_ok) begin
                    S_rx_length_hold <= S_rx_app_count;
                    S_rx_hold_valid <= 1'b1;
                end else begin
                    if (S_rx_overflow) begin
                        O_rx_oversize_drop_count <= O_rx_oversize_drop_count + 32'd1;
                    end
                    O_rx_bad_frame_count <= O_rx_bad_frame_count + 32'd1;
                end
            end
        end
    end

    //==========================================================================
    // 2026-10-18 上板观测：逐项检查的粘滞失败位 + 最近一帧应用字节数
    //   只在帧结束那一拍采样（与 W_rx_complete_ok 的判据同一时刻），
    //   随后经两级同步送进应用域。纯观测，不回灌任何收发逻辑。
    //==========================================================================
    reg [7:0]  S_rx_dbg_flags_q;
    reg [10:0] S_rx_dbg_cnt_q;
    reg [7:0]  S_rx_dbg_flags_s1_app;
    reg [7:0]  S_rx_dbg_flags_s2_app;
    reg [10:0] S_rx_dbg_cnt_s1_app;
    reg [10:0] S_rx_dbg_cnt_s2_app;

    always @(posedge I_clk_rx or negedge I_rst_rx_n) begin
        if (!I_rst_rx_n) begin
            S_rx_dbg_flags_q <= 8'd0;
            S_rx_dbg_cnt_q   <= 11'd0;
        end else if (S_rx_active && !I_gmii_rx_dv) begin
            S_rx_dbg_flags_q[0] <= S_rx_dbg_flags_q[0] | ~S_rx_preamble_ok;
            S_rx_dbg_flags_q[1] <= S_rx_dbg_flags_q[1] | ~S_rx_header_ok;
            S_rx_dbg_flags_q[2] <= S_rx_dbg_flags_q[2] | ~W_rx_crc_ok;
            S_rx_dbg_flags_q[3] <= S_rx_dbg_flags_q[3] | ~W_rx_shape_ok;
            S_rx_dbg_flags_q[4] <= S_rx_dbg_flags_q[4] | ~W_rx_header_contract_ok_q;
            S_rx_dbg_flags_q[5] <= S_rx_dbg_flags_q[5] | ~W_rx_padding_ok;
            S_rx_dbg_flags_q[6] <= S_rx_dbg_flags_q[6] |  S_rx_error_seen;
            S_rx_dbg_flags_q[7] <= S_rx_dbg_flags_q[7] |  W_rx_complete_ok;
            S_rx_dbg_cnt_q      <= S_rx_app_count;
        end
    end

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            S_rx_dbg_flags_s1_app <= 8'd0;
            S_rx_dbg_flags_s2_app <= 8'd0;
            S_rx_dbg_cnt_s1_app   <= 11'd0;
            S_rx_dbg_cnt_s2_app   <= 11'd0;
            O_rx_dbg_flags        <= 8'd0;
            O_rx_dbg_app_count    <= 11'd0;
        end else begin
            S_rx_dbg_flags_s1_app <= S_rx_dbg_flags_q;
            S_rx_dbg_flags_s2_app <= S_rx_dbg_flags_s1_app;
            S_rx_dbg_cnt_s1_app   <= S_rx_dbg_cnt_q;
            S_rx_dbg_cnt_s2_app   <= S_rx_dbg_cnt_s1_app;
            O_rx_dbg_flags        <= S_rx_dbg_flags_s2_app;
            O_rx_dbg_app_count    <= S_rx_dbg_cnt_s2_app;
        end
    end

endmodule
