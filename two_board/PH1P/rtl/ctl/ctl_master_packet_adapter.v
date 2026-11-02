//=============================================================================
//  ctl_master_packet_adapter  --  PH1P snapshot request/response adapter
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk_pix (video/application)
//  复位     : I_rst_pix_n
//  参数     : P_SESSION_ID, P_TIMEOUT_CYCLES
//  验证状态 : 仿真 待 Root 重跑 | 综合 未做 | 布线 未做 | 上板 未做
//=============================================================================
`timescale 1ns / 1ps
// PH1P application adapter for ctl_lpr_packet_link.
// This module stores the exported rows, emits the frozen 912-byte request,
// and atomically applies a complete 32-byte response.  It does not implement
// Ethernet framing, FCS, MAC filtering, or RGMII.

module ctl_master_packet_adapter #(
    parameter [31:0] P_SESSION_ID = 32'h2026_1011,
    parameter [23:0] P_TIMEOUT_CYCLES = 24'd7500000
) (
    input  wire        I_clk_pix,
    input  wire        I_rst_pix_n,

    input  wire        I_capture_write_en,
    input  wire [8:0]  I_capture_write_addr,
    input  wire [31:0] I_capture_write_data,
    input  wire        I_snapshot_valid,
    input  wire        I_snapshot_drop,
    input  wire        I_snapshot_bank,
    input  wire [6:0]  I_snapshot_mask,
    input  wire [31:0] I_snapshot_frame_index,
    input  wire        I_plate_valid,
    input  wire        I_link_ready,

    output wire        O_capture_export_busy,

    output reg         O_tx_write_en,
    output reg  [9:0]  O_tx_write_addr,
    output reg  [7:0]  O_tx_write_data,
    output reg         O_tx_commit,
    output reg  [10:0] O_tx_length,
    input  wire        I_tx_busy,
    input  wire        I_tx_done,

    input  wire        I_rx_valid,
    input  wire [10:0] I_rx_length,
    output reg  [9:0]  O_rx_read_addr,
    input  wire [7:0]  I_rx_read_data,
    output reg        O_rx_release,

    output reg  [55:0] O_res,
    output reg  [3:0]  O_res_n,
    output reg         O_res_valid,
    output reg  [6:0]  O_raw_valid,
    output reg  [55:0] O_raw_candidate_code,
    output reg  [6:0]  O_raw_candidate_mask,
    output reg         O_frame_result,
    // Capture can finish a frame while the previous transaction owns the
    // exporter.  Keep that event observable instead of silently treating it
    // as an OCR/PHY failure.
    output reg  [31:0] O_snapshot_drop_count
);

    localparam [3:0] ST_IDLE      = 4'd0;
    localparam [3:0] ST_TX_WRITE  = 4'd1;
    localparam [3:0] ST_TX_COMMIT = 4'd2;
    localparam [3:0] ST_WAIT_RX   = 4'd3;
    localparam [3:0] ST_RX_READ   = 4'd4;
    localparam [3:0] ST_RX_CHECK  = 4'd5;
    localparam [3:0] ST_TX_READ   = 4'd6;
    localparam [3:0] ST_RX_DRAIN  = 4'd7;

    reg [3:0] ST_state_q;
    reg [23:0] S_timeout_q;
    reg [9:0] S_tx_offset_q;
    // The wire protocol carries a 32-bit frame identifier.  Capture owns a
    // 32-bit monotonically increasing index, and this latched value is used
    // identically for TX and RX checks across the 16-bit wrap boundary.
    reg [31:0] S_frame_id_q;
    reg [6:0] S_snapshot_mask_q;
    reg S_snapshot_bank_q;
    reg S_plate_valid_q;
    // One registered word is read from bank_ram per four payload bytes.  The
    // first word is prefetched in ST_TX_READ; subsequent words are issued on
    // the last byte of the preceding word, allowing a synchronous ERAM read
    // without a bubble in the 896-byte payload stream.
    reg [31:0] S_tx_read_data_q;
    reg [7:0] S_tx_ram_row_q;
    reg [2:0] S_tx_ram_slot_q;
    reg [10:0] S_rx_length_q;
    reg [5:0] S_rx_index_q;
    reg [7:0] S_rx_drain_q;
    reg       S_rx_release_issued_q;
    // Internal epoch changes on timeout/link recovery.  The frozen wire
    // response remains 32-byte/session-compatible; frame_id plus the drain
    // state are the stale-response discriminator.
    reg       S_transaction_epoch_q;
    reg [31:0] S_session_id_q;
    reg       S_link_ready_q;
    reg       S_link_seen_q;

    reg [31:0] S_bank_ram [0:447];
    reg [7:0] S_rx_buf [0:31];
    reg [7:0] W_tx_byte_value;
    integer W_byte_index;
    integer W_row_index;
    integer W_byte_lane;
    integer W_slot_index;
    integer S_reset_index;

    assign O_capture_export_busy = (ST_state_q != ST_IDLE);

    // Network byte order header plus the fixed 896-byte row payload.
    always @(*) begin
        W_tx_byte_value = 8'h00;
        W_byte_index = 0;
        W_row_index = 0;
        W_byte_lane = 0;
        W_slot_index = 0;
        if (S_tx_offset_q == 10'd0)       W_tx_byte_value = 8'h4c;
        else if (S_tx_offset_q == 10'd1)  W_tx_byte_value = 8'h50;
        else if (S_tx_offset_q == 10'd2)  W_tx_byte_value = 8'h01;
        else if (S_tx_offset_q == 10'd3)  W_tx_byte_value = 8'h01;
        else if (S_tx_offset_q >= 10'd4 && S_tx_offset_q <= 10'd7)
            W_tx_byte_value = S_session_id_q[31 - ((S_tx_offset_q - 10'd4) * 8) -: 8];
        else if (S_tx_offset_q >= 10'd8 && S_tx_offset_q <= 10'd11)
            W_tx_byte_value = S_frame_id_q[31 - ((S_tx_offset_q - 10'd8) * 8) -: 8];
        else if (S_tx_offset_q == 10'd12) W_tx_byte_value = 8'h03;
        else if (S_tx_offset_q == 10'd13) W_tx_byte_value = 8'h80;
        else if (S_tx_offset_q == 10'd14) W_tx_byte_value = {1'b0, S_snapshot_mask_q[6:0]};
        else if (S_tx_offset_q == 10'd15) W_tx_byte_value = {7'd0, S_plate_valid_q};
        else if (S_tx_offset_q >= 10'd16 && S_tx_offset_q < 10'd912) begin
            W_byte_index = S_tx_offset_q - 10'd16;
            W_row_index = W_byte_index / 4;
            W_byte_lane = W_byte_index % 4;
            W_slot_index = W_row_index / 32;
            // bank_ram is intentionally not reset so the 448x32 storage can
            // remain eligible for ERAM inference.  A slot not present in the
            // latched snapshot mask must never expose stale RAM contents.
            if ((W_slot_index >= 7) || !S_snapshot_mask_q[W_slot_index]) begin
                W_tx_byte_value = 8'h00;
            end else begin
                case (W_byte_lane)
                    0: W_tx_byte_value = S_tx_read_data_q[7:0];
                    1: W_tx_byte_value = S_tx_read_data_q[15:8];
                    2: W_tx_byte_value = S_tx_read_data_q[23:16];
                    default: W_tx_byte_value = S_tx_read_data_q[31:24];
                endcase
            end
        end
    end

    always @(posedge I_clk_pix or negedge I_rst_pix_n) begin
        if (!I_rst_pix_n) begin
            ST_state_q <= ST_IDLE;
            S_timeout_q <= 24'd0;
            S_tx_offset_q <= 10'd0;
            S_frame_id_q <= 32'd0;
            S_snapshot_mask_q <= 7'd0;
            S_snapshot_bank_q <= 1'b0;
            S_plate_valid_q <= 1'b0;
            S_tx_read_data_q <= 32'd0;
            S_tx_ram_row_q <= 8'd0;
            S_tx_ram_slot_q <= 3'd0;
            S_rx_length_q <= 11'd0;
            S_rx_index_q <= 6'd0;
            S_rx_drain_q <= 8'd0;
            S_rx_release_issued_q <= 1'b0;
            S_transaction_epoch_q <= 1'b0;
            S_session_id_q <= P_SESSION_ID;
            S_link_ready_q <= 1'b0;
            S_link_seen_q <= 1'b0;
            O_tx_write_en <= 1'b0;
            O_tx_write_addr <= 10'd0;
            O_tx_write_data <= 8'd0;
            O_tx_commit <= 1'b0;
            O_tx_length <= 11'd912;
            O_rx_read_addr <= 10'd0;
            O_rx_release <= 1'b0;
            O_res <= 56'd0;
            O_res_n <= 4'd0;
            O_res_valid <= 1'b0;
            O_raw_valid <= 7'd0;
            O_raw_candidate_code <= 56'd0;
            O_raw_candidate_mask <= 7'd0;
            O_frame_result <= 1'b0;
            O_snapshot_drop_count <= 32'd0;
            // bank_ram intentionally has no reset loop: clearing the whole
            // 14,336-bit array here would inhibit ERAM inference.  Invalid
            // snapshot slots are emitted as zero in the TX mux above.
            for (S_reset_index = 0; S_reset_index < 32; S_reset_index = S_reset_index + 1)
                S_rx_buf[S_reset_index] <= 8'd0;
        end else begin
            O_tx_write_en <= 1'b0;
            O_tx_commit <= 1'b0;
            O_rx_release <= 1'b0;
            O_frame_result <= 1'b0;

            // A capture drop is an explicit telemetry event.  The exporter
            // remains busy with its current transaction, while this counter
            // lets diagnostics distinguish a dropped snapshot from an OCR or
            // PHY failure.
            if (I_snapshot_drop)
                O_snapshot_drop_count <= O_snapshot_drop_count + 32'd1;

            // Advance the session exactly once on a link transition.  The
            // initial ready observation intentionally retains P_SESSION_ID;
            // every loss/recovery thereafter gets a new identity so a late
            // packet cannot be accepted by a later transaction.
            if (I_link_ready != S_link_ready_q) begin
                if (S_link_seen_q)
                    S_session_id_q <= S_session_id_q + 32'd1;
                S_link_ready_q <= I_link_ready;
                S_link_seen_q <= 1'b1;
            end

            // A PHY that is merely electrically linked at 10/100 Mb/s is
            // not a valid board-to-board transport.  Abort an in-flight
            // transaction and clear the public result until the synchronized
            // RX in-band status reports 1 Gb/s/full duplex again.
            if (!I_link_ready) begin
                ST_state_q <= ST_RX_DRAIN;
                S_timeout_q <= 24'd0;
                S_rx_drain_q <= 8'd0;
                S_rx_release_issued_q <= 1'b0;
                S_transaction_epoch_q <= ~S_transaction_epoch_q;
                O_res_valid <= 1'b0;
                if (I_rx_valid) begin
                    O_rx_release <= 1'b1;
                    S_rx_release_issued_q <= 1'b1;
                end
            end

            if (I_capture_write_en && (I_capture_write_addr < 9'd448))
                S_bank_ram[I_capture_write_addr] <= I_capture_write_data;

            if (I_link_ready && (ST_state_q == ST_IDLE)) begin
                S_timeout_q <= 24'd0;
                if (I_snapshot_valid) begin
                    S_frame_id_q <= I_snapshot_frame_index;
                    S_snapshot_mask_q <= I_snapshot_mask;
                    S_snapshot_bank_q <= I_snapshot_bank;
                    S_plate_valid_q <= I_plate_valid;
                    S_tx_offset_q <= 10'd0;
                    S_tx_read_data_q <= 32'd0;
                    S_tx_ram_row_q <= 8'd0;
                    S_tx_ram_slot_q <= 3'd0;
                    O_tx_length <= 11'd912;
                    ST_state_q <= ST_TX_WRITE;
                end
            end else if (I_link_ready) begin
                if (S_timeout_q >= P_TIMEOUT_CYCLES - 1'b1) begin
                    // Timeout is a complete invalid transaction.  The result
                    // pulse lets the video side clear stale OSD validity.
                    S_timeout_q <= 24'd0;
                    ST_state_q <= ST_RX_DRAIN;
                    S_rx_drain_q <= 8'd0;
                    S_rx_release_issued_q <= 1'b0;
                    S_transaction_epoch_q <= ~S_transaction_epoch_q;
                    S_session_id_q <= S_session_id_q + 32'd1;
                    O_res_valid <= 1'b0;
                    O_frame_result <= 1'b1;
                    if (I_rx_valid) begin
                        O_rx_release <= 1'b1;
                        S_rx_release_issued_q <= 1'b1;
                    end
                end else begin
                    S_timeout_q <= S_timeout_q + 1'b1;
                end
            end

            if (I_link_ready && (S_timeout_q < P_TIMEOUT_CYCLES - 1'b1)) case (ST_state_q)
                ST_IDLE: begin
                    // Snapshot admission above owns the ST_IDLE transition;
                    // do not let the dispatch case overwrite it with a
                    // default assignment in the same NBA block.
                end

                ST_TX_WRITE: begin
                    if (!I_tx_busy) begin
                        O_tx_write_en <= 1'b1;
                        O_tx_write_addr <= S_tx_offset_q;
                        O_tx_write_data <= W_tx_byte_value;
                        if (S_tx_offset_q == 10'd15) begin
                            // Header byte 15 is written now.  Read payload
                            // row 0 on the next clock in ST_TX_READ so the
                            // first payload byte samples a registered word.
                            S_tx_offset_q <= 10'd16;
                            S_tx_ram_row_q <= 8'd0;
                            S_tx_ram_slot_q <= 3'd0;
                            ST_state_q <= ST_TX_READ;
                        end else if (S_tx_offset_q == 10'd911) begin
                            ST_state_q <= ST_TX_COMMIT;
                        end else begin
                            // At each completed 32-bit word, issue the next
                            // synchronous RAM read.  Invalid slots do not
                            // read bank_ram and instead load an explicit zero.
                            if ((S_tx_offset_q >= 10'd16) &&
                                (((S_tx_offset_q - 10'd16) % 4) == 3)) begin
                                S_tx_ram_row_q <= S_tx_ram_row_q + 1'b1;
                                if (S_tx_ram_row_q[4:0] == 5'd31) begin
                                    S_tx_ram_slot_q <= S_tx_ram_slot_q + 3'd1;
                                    if (S_snapshot_mask_q[S_tx_ram_slot_q + 3'd1])
                                        S_tx_read_data_q <= S_bank_ram[(S_snapshot_bank_q ? 224 : 0) +
                                                                    S_tx_ram_row_q + 1'b1];
                                    else
                                        S_tx_read_data_q <= 32'd0;
                                end else if (S_snapshot_mask_q[S_tx_ram_slot_q]) begin
                                    S_tx_read_data_q <= S_bank_ram[(S_snapshot_bank_q ? 224 : 0) +
                                                                S_tx_ram_row_q + 1'b1];
                                end else begin
                                    S_tx_read_data_q <= 32'd0;
                                end
                            end
                            S_tx_offset_q <= S_tx_offset_q + 1'b1;
                        end
                    end
                end

                ST_TX_READ: begin
                    // Registered first payload word.  Subsequent reads are
                    // launched by the byte-3 path in ST_TX_WRITE.
                    if (S_snapshot_mask_q[0])
                        S_tx_read_data_q <= S_bank_ram[(S_snapshot_bank_q ? 224 : 0)];
                    else
                        S_tx_read_data_q <= 32'd0;
                    ST_state_q <= ST_TX_WRITE;
                end

                ST_TX_COMMIT: begin
                    if (!I_tx_busy) begin
                        O_tx_commit <= 1'b1;
                        O_tx_length <= 11'd912;
                        // Keep address zero stable while the common link
                        // waits for the response.  Its synchronous read
                        // port then has a deterministic first-byte value.
                        O_rx_read_addr <= 10'd0;
                        ST_state_q <= ST_WAIT_RX;
                    end
                end

                ST_WAIT_RX: begin
                    // Keep address zero stable while waiting.  On the valid
                    // edge, pre-issue address one: the common link registers
                    // byte zero from the prior address and byte one from this
                    // address for the next application edge.
                    O_rx_read_addr <= 10'd0;
                    if (I_rx_valid) begin
                        S_rx_length_q <= I_rx_length;
                        S_rx_index_q <= 6'd0;
                        S_rx_release_issued_q <= 1'b0;
                        O_rx_read_addr <= 10'd1;
                        ST_state_q <= ST_RX_READ;
                    end
                end

                ST_RX_READ: begin
                    S_rx_buf[S_rx_index_q] <= I_rx_read_data;
                    if (S_rx_index_q == 6'd31) begin
                        ST_state_q <= ST_RX_CHECK;
                    end else begin
                        S_rx_index_q <= S_rx_index_q + 1'b1;
                        O_rx_read_addr <= S_rx_index_q + 2'd2;
                    end
                end

                ST_RX_CHECK: begin
                    // Release exactly once, then drain until the link drops
                    // RX_VALID.  Parsing while RX_VALID remains high used to
                    // re-apply one response every cycle and left malformed
                    // responses stuck in ST_RX_CHECK forever.
                    O_rx_release <= 1'b1;
                    S_rx_release_issued_q <= 1'b1;
                    ST_state_q <= ST_RX_DRAIN;
                    S_rx_drain_q <= 8'd0;
                    if ((S_rx_length_q >= 11'd46) &&
                        (S_rx_buf[0] == 8'h4c) && (S_rx_buf[1] == 8'h50) &&
                        (S_rx_buf[2] == 8'h01) && (S_rx_buf[3] == 8'h02) &&
                        (S_rx_buf[4] == S_session_id_q[31:24]) &&
                        (S_rx_buf[5] == S_session_id_q[23:16]) &&
                        (S_rx_buf[6] == S_session_id_q[15:8]) &&
                        (S_rx_buf[7] == S_session_id_q[7:0]) &&
                        (S_rx_buf[8] == S_frame_id_q[31:24]) &&
                        (S_rx_buf[9] == S_frame_id_q[23:16]) &&
                        (S_rx_buf[10] == S_frame_id_q[15:8]) &&
                        (S_rx_buf[11] == S_frame_id_q[7:0]) &&
                        (S_rx_buf[12] == 8'h00) && (S_rx_buf[13] == 8'h10) &&
                        (((S_rx_buf[14] == 8'h7f) && S_rx_buf[15][0]) ||
                         ((S_rx_buf[14] == 8'h00) && !S_rx_buf[15][0])) &&
                        ((S_rx_buf[15] & 8'hfe) == 8'h00)) begin
                        O_res <= {S_rx_buf[22], S_rx_buf[21], S_rx_buf[20],
                                  S_rx_buf[19], S_rx_buf[18], S_rx_buf[17], S_rx_buf[16]};
                        O_res_n <= S_rx_buf[23][3:0];
                        O_raw_valid <= S_rx_buf[24][6:0];
                        O_raw_candidate_code <= {S_rx_buf[31], S_rx_buf[30],
                                                  S_rx_buf[29], S_rx_buf[28],
                                                  S_rx_buf[27], S_rx_buf[26], S_rx_buf[25]};
                        O_raw_candidate_mask <= S_rx_buf[24][6:0];
                        O_res_valid <= S_rx_buf[15][0];
                        O_frame_result <= 1'b1;
                        S_timeout_q <= 24'd0;
                    end else begin
                        // A bad identity/shape is consumed and reported as an
                        // invalid result; the next snapshot is admitted only
                        // after the current RX hold has drained.
                        O_res_valid <= 1'b0;
                        O_frame_result <= 1'b1;
                    end
                end

                ST_RX_DRAIN: begin
                    // A common link may keep O_rx_valid high for several app
                    // clocks after O_rx_release.  Never re-read that buffer;
                    // wait for the falling edge before returning to IDLE.
                    if (!I_rx_valid) begin
                        ST_state_q <= ST_IDLE;
                        S_rx_drain_q <= 8'd0;
                        S_rx_release_issued_q <= 1'b0;
                        S_timeout_q <= 24'd0;
                    end
                    else if (!S_rx_release_issued_q) begin
                        O_rx_release <= 1'b1;
                        S_rx_release_issued_q <= 1'b1;
                        S_rx_drain_q <= 8'd0;
                    end
                    else if (S_rx_drain_q == 8'hFF) begin
                        // Bounded recovery for a wedged link.  The next
                        // transaction still carries a new frame_id and will
                        // reject any stale packet that arrives later.
                        ST_state_q <= ST_IDLE;
                        S_rx_drain_q <= 8'd0;
                        S_rx_release_issued_q <= 1'b0;
                        S_timeout_q <= 24'd0;
                    end
                    else begin
                        S_rx_drain_q <= S_rx_drain_q + 1'b1;
                    end
                end

                default: ST_state_q <= ST_IDLE;
            endcase
        end
    end
endmodule
