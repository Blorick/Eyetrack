`timescale 1ns/1ps
// ============================================================================
// ctl_eg4d_compute_adapter
//
// 25 MHz application-domain adapter for the EG4D slave.  The Ethernet client
// (ctl_lpr_packet_link) owns the wire protocol and its packet RAMs.  This
// block only consumes that client's fixed app-side RAM interface, writes the
// C13 two-bank tile engine, schedules C13 feature extraction and C14 matching,
// then serializes the frozen C14 result back into the TX app RAM.
//
// The request reader deliberately uses two states per byte.  The address is
// presented for one clock and the returned byte is sampled on the following
// clock, matching the frozen one-clock synchronous-read contract.
//
// No packet, payload, or result is shared with another transaction while this
// state machine is busy.  A malformed/old request is released without a TX
// response.  A valid request always gets a 32-byte response, including when
// C14 has no stable update on this frame.
// ============================================================================
module ctl_eg4d_compute_adapter #(
    // Bound a malformed request, a matcher that never asserts done, or a
    // stuck TX client so the 0x88B6 stream endpoint cannot be starved.
    parameter integer P_WATCHDOG_CYCLES = 25_000_000,
    parameter integer P_DRAIN_CYCLES = 32
) (
    input  wire        I_clk_app,
    input  wire        I_rst_app_n,

    // ctl_lpr_packet_link TX app-side interface.
    output reg         O_tx_write_en,
    output reg  [9:0]  O_tx_write_addr,
    output reg  [7:0]  O_tx_write_data,
    output reg         O_tx_commit,
    output reg  [10:0] O_tx_length,
    input  wire        I_tx_busy,
    input  wire        I_tx_done,

    // ctl_lpr_packet_link RX app-side interface.
    input  wire        I_rx_valid,
    input  wire [10:0] I_rx_length,
    output reg  [9:0]  O_rx_read_addr,
    input  wire [7:0]  I_rx_read_data,
    output reg         O_rx_release,

    output wire        O_busy,
    output wire        O_response_valid,
    output reg         O_frame_result_pulse,
    output wire [3:0]  O_debug_state
);

    localparam [4:0] S_IDLE          = 5'd0;
    localparam [4:0] S_RX_HEADER_REQ = 5'd1;
    localparam [4:0] S_RX_HEADER_WAIT= 5'd2;
    localparam [4:0] S_RX_TILE_REQ   = 5'd3;
    localparam [4:0] S_RX_TILE_WAIT  = 5'd4;
    localparam [4:0] S_RX_TILE_WRITE = 5'd5;
    localparam [4:0] S_SESSION_CHECK = 5'd6;
    localparam [4:0] S_MATCHER_RESET = 5'd7;
    localparam [4:0] S_FRAME_START   = 5'd8;
    localparam [4:0] S_GLYPH_START   = 5'd9;
    localparam [4:0] S_GLYPH_WAIT    = 5'd10;
    localparam [4:0] S_FRAME_WAIT    = 5'd11;
    localparam [4:0] S_TX_WAIT       = 5'd12;
    localparam [4:0] S_TX_WRITE      = 5'd13;
    localparam [4:0] S_TX_COMMIT     = 5'd14;
    localparam [4:0] S_TX_DONE       = 5'd15;
    localparam [4:0] S_RELEASE       = 5'd16;
    localparam [4:0] S_DROP          = 5'd17;
    localparam [4:0] S_HEADER_ID_CHECK= 5'd18;
    localparam [4:0] S_RX_DRAIN      = 5'd19;

    localparam [10:0] REQUEST_LENGTH = 11'd912;
    localparam [15:0] REQUEST_PAYLOAD_LENGTH = 16'd896;
    localparam [10:0] RESPONSE_LENGTH = 11'd32;

    reg [4:0] state;
    // Canonical local-style state probe; the legacy state name is retained
    // for the frozen directed TB hierarchy and has identical semantics.
    wire [4:0] ST_state = state;

    // Header and transaction identity captured from the request.
    reg [9:0]  rx_index_q;
    reg [7:0]  header_magic0_q;
    reg [7:0]  header_magic1_q;
    reg [7:0]  header_version_q;
    reg [7:0]  header_kind_q;
    reg [31:0] request_session_q;
    reg [31:0] request_frame_q;
    reg [15:0] request_payload_len_q;
    reg [6:0]  request_mask_q;
    reg        request_mask_reserved_q;
    reg        request_plate_valid_q;

    // A new session resets C14 stability.  Within a session frames must move
    // strictly forward; stale/replayed requests are dropped before compute.
    reg        session_seen_q;
    reg [31:0] active_session_q;
    reg        retired_session_valid_q;
    reg [31:0] retired_session_q;
    reg [31:0] last_frame_q;
    reg        new_session_q;
    reg        matcher_rst_n_q;

    // 896-byte payload reader and one complete 32-bit row at a time.
    reg [9:0]  tile_byte_q;
    reg [8:0]  tile_row_q;
    reg [31:0] tile_word_q;

    reg [2:0] glyph_slot_q;

    // Response is latched only after C14's one-cycle o_frame_result pulse is
    // observed.  The persistent o_res_valid is intentionally distinct from
    // that pulse.
    reg [31:0] response_session_q;
    reg [31:0] response_frame_q;
    reg [55:0] response_res_q;
    reg [3:0]  response_res_n_q;
    reg        response_res_valid_q;
    reg [6:0]  response_raw_valid_q;
    reg [55:0] response_raw_code_q;
    reg [5:0]  tx_index_q;
    reg [31:0] watchdog_q;
    reg [7:0]  drain_q;

    // ---------------------------------------------------------------------
    // C13 engine and C14 matcher connections.
    // ---------------------------------------------------------------------
    wire        engine_feature_valid;
    wire [7:0]  engine_feature_data;
    wire        engine_feature_ready;
    wire        engine_done;
    wire        engine_busy;
    wire [6:0]  engine_feature_index;

    wire        engine_tile_write_en = (state == S_RX_TILE_WRITE);
    wire        engine_process_start = (state == S_GLYPH_START);
    wire [2:0]  engine_process_slot = glyph_slot_q;
    wire        engine_process_valid = request_plate_valid_q &&
                                       request_mask_q[glyph_slot_q];
    wire [8:0]  engine_tile_write_addr = tile_row_q;

    char_shape_feature_engine #(
        .TILE_WORDS(448)
    ) u_c13_engine (
        .clk              (I_clk_app),
        .rst_n            (I_rst_app_n),
        .i_tile_write_en  (engine_tile_write_en),
        .i_tile_write_addr(engine_tile_write_addr),
        .i_tile_write_data(tile_word_q),
        .i_process_start  (engine_process_start),
        .i_process_slot   (engine_process_slot),
        .i_process_bank   (1'b0),
        .i_process_valid  (engine_process_valid),
        .o_feature_valid  (engine_feature_valid),
        .o_feature_data   (engine_feature_data),
        .i_feature_ready  (engine_feature_ready),
        .o_engine_done    (engine_done),
        .o_busy           (engine_busy),
        .o_feature_index  (engine_feature_index)
    );

    wire        matcher_glyph_start = (state == S_GLYPH_START);
    wire        matcher_frame_start = (state == S_FRAME_START);
    wire        matcher_glyph_done;
    wire        matcher_frame_done =
        (state == S_GLYPH_WAIT) && (glyph_slot_q == 3'd6) &&
        matcher_glyph_done;
    wire        matcher_result_valid;
    wire [7:0]  matcher_result_code;
    wire        matcher_result_accepted;
    wire [2:0]  matcher_result_slot;
    wire        matcher_res_valid;
    wire [55:0] matcher_res;
    wire [3:0]  matcher_res_n;
    wire [6:0]  matcher_raw_valid;
    wire        matcher_frame_result;
    wire [55:0] matcher_raw_candidate_code;
    wire [6:0]  matcher_raw_candidate_mask;
    wire [3:0]  matcher_state;

    // 2026-10-11：恢复跨帧稳定识别配置。
    //   STABLE_EN=1 要求 7 个字形全部被接受，并连续 HOLD_K=5 帧完全一致，
    //   才把 matcher_res_valid 置起；该状态经响应字节 14/15 送到主板 PH1P，
    //   是 PH1P OSD 第 3 行（识别结果行）的有效使能来源。
    //   当前配置保留时间确认，避免把单帧或部分字形结果直接显示为最终车牌。
    //   此前的 STABLE_EN=0 仅用于历史上板 bypass 调试，旁路跨帧确认，
    //   可能显示部分或错误字符，不属于当前交付配置。
    //   当前参数：STABLE_EN=1、HOLD_K=5、LOST_K=8；主板只在 res_valid=1 时点亮第 3 行。
    char_match #(
        .STABLE_EN(1),
        .HOLD_K(5),
        .LOST_K(8)
    ) u_c14_matcher (
        .clk                 (I_clk_app),
        .rst_n               (matcher_rst_n_q),
        .i_glyph_start       (matcher_glyph_start),
        .i_glyph_slot        (glyph_slot_q),
        .i_glyph_valid       (engine_process_valid),
        .i_feature_valid     (engine_feature_valid),
        .i_feature_data      (engine_feature_data),
        .o_feature_ready     (engine_feature_ready),
        .o_result_valid      (matcher_result_valid),
        .o_result_code       (matcher_result_code),
        .o_result_accepted   (matcher_result_accepted),
        .o_glyph_done        (matcher_glyph_done),
        .o_result_slot       (matcher_result_slot),
        .i_frame_start       (matcher_frame_start),
        .i_frame_done        (matcher_frame_done),
        .o_res               (matcher_res),
        .o_res_n             (matcher_res_n),
        .o_res_valid         (matcher_res_valid),
        .o_raw_valid         (matcher_raw_valid),
        .o_frame_result      (matcher_frame_result),
        .o_raw_candidate_code(matcher_raw_candidate_code),
        .o_raw_candidate_mask(matcher_raw_candidate_mask),
        .o_match_state       (matcher_state)
    );

    // ---------------------------------------------------------------------
    // Fixed response serializer.  Bytes 16..22 and 25..31 are explicitly
    // spelled out to avoid tool-dependent variable part-select behavior.
    // ---------------------------------------------------------------------
    function [7:0] response_byte;
        input [5:0] index;
        begin
            case (index)
                6'd0:  response_byte = 8'h4c;
                6'd1:  response_byte = 8'h50;
                6'd2:  response_byte = 8'h01;
                6'd3:  response_byte = 8'h02;
                6'd4:  response_byte = response_session_q[31:24];
                6'd5:  response_byte = response_session_q[23:16];
                6'd6:  response_byte = response_session_q[15:8];
                6'd7:  response_byte = response_session_q[7:0];
                6'd8:  response_byte = response_frame_q[31:24];
                6'd9:  response_byte = response_frame_q[23:16];
                6'd10: response_byte = response_frame_q[15:8];
                6'd11: response_byte = response_frame_q[7:0];
                6'd12: response_byte = 8'h00;
                6'd13: response_byte = 8'h10;
                6'd14: response_byte = response_res_valid_q ? 8'h7f : 8'h00;
                6'd15: response_byte = response_res_valid_q ? 8'h01 : 8'h00;
                6'd16: response_byte = response_res_q[7:0];
                6'd17: response_byte = response_res_q[15:8];
                6'd18: response_byte = response_res_q[23:16];
                6'd19: response_byte = response_res_q[31:24];
                6'd20: response_byte = response_res_q[39:32];
                6'd21: response_byte = response_res_q[47:40];
                6'd22: response_byte = response_res_q[55:48];
                6'd23: response_byte = {4'b0000, response_res_n_q};
                6'd24: response_byte = {1'b0, response_raw_valid_q};
                6'd25: response_byte = response_raw_code_q[7:0];
                6'd26: response_byte = response_raw_code_q[15:8];
                6'd27: response_byte = response_raw_code_q[23:16];
                6'd28: response_byte = response_raw_code_q[31:24];
                6'd29: response_byte = response_raw_code_q[39:32];
                6'd30: response_byte = response_raw_code_q[47:40];
                6'd31: response_byte = response_raw_code_q[55:48];
                default: response_byte = 8'h00;
            endcase
        end
    endfunction

    // App-side outputs are deliberately simple registered-state decodes.  A
    // TX write/commit is only asserted in a state that has already observed
    // O_tx_busy low.  The RX address is held through both halves of the
    // synchronous-read transaction.
    always @(*) begin
        O_rx_read_addr  = 10'd0;
        O_tx_write_en   = 1'b0;
        O_tx_write_addr = 10'd0;
        O_tx_write_data = 8'd0;
        O_tx_commit     = 1'b0;
        O_tx_length     = RESPONSE_LENGTH;
        O_rx_release    = 1'b0;

        case (state)
            S_RX_HEADER_REQ,
            S_RX_HEADER_WAIT: O_rx_read_addr = rx_index_q;
            S_RX_TILE_REQ,
            S_RX_TILE_WAIT:   O_rx_read_addr = 10'd16 + tile_byte_q;
            S_RX_TILE_WRITE: begin
                O_tx_length = RESPONSE_LENGTH;
            end
            S_TX_WRITE: begin
                O_tx_write_en   = !I_tx_busy;
                O_tx_write_addr = {4'd0, tx_index_q};
                O_tx_write_data = response_byte(tx_index_q);
            end
            S_TX_COMMIT: begin
                O_tx_commit = !I_tx_busy;
            end
            S_RELEASE: begin
                O_rx_release = I_rx_valid;
            end
            default: begin
            end
        endcase
    end

    assign O_busy = (state != S_IDLE);
    assign O_response_valid = response_res_valid_q;
    assign O_debug_state = state[3:0];

    // ---------------------------------------------------------------------
    // Request/compute/response control.
    // ---------------------------------------------------------------------
    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            state                 <= S_IDLE;
            rx_index_q            <= 10'd0;
            header_magic0_q       <= 8'd0;
            header_magic1_q       <= 8'd0;
            header_version_q      <= 8'd0;
            header_kind_q         <= 8'd0;
            request_session_q     <= 32'd0;
            request_frame_q       <= 32'd0;
            request_payload_len_q <= 16'd0;
            request_mask_q        <= 7'd0;
            request_mask_reserved_q <= 1'b0;
            request_plate_valid_q <= 1'b0;
            session_seen_q        <= 1'b0;
            active_session_q      <= 32'd0;
            retired_session_valid_q <= 1'b0;
            retired_session_q     <= 32'd0;
            last_frame_q          <= 32'd0;
            new_session_q         <= 1'b0;
            matcher_rst_n_q       <= 1'b0;
            tile_byte_q           <= 10'd0;
            tile_row_q            <= 9'd0;
            tile_word_q           <= 32'd0;
            glyph_slot_q          <= 3'd0;
            response_session_q    <= 32'd0;
            response_frame_q      <= 32'd0;
            response_res_q        <= 56'd0;
            response_res_n_q      <= 4'd0;
            response_res_valid_q  <= 1'b0;
            response_raw_valid_q  <= 7'd0;
            response_raw_code_q   <= 56'd0;
            tx_index_q            <= 6'd0;
            watchdog_q            <= 32'd0;
            drain_q               <= 8'd0;
            O_frame_result_pulse  <= 1'b0;
        end else begin
            O_frame_result_pulse <= 1'b0;

            if (state == S_IDLE)
                watchdog_q <= 32'd0;
            else if (watchdog_q != 32'hFFFF_FFFF)
                watchdog_q <= watchdog_q + 32'd1;

            case (state)
                S_IDLE: begin
                    // Keep C14 in reset until the first accepted session has
                    // passed the header/payload checks.
                    if (I_rx_valid) begin
                        rx_index_q <= 10'd0;
                        state <= S_RX_HEADER_REQ;
                    end
                end

                S_RX_HEADER_REQ: begin
                    state <= S_RX_HEADER_WAIT;
                end

                S_RX_HEADER_WAIT: begin
                    case (rx_index_q)
                        10'd0:  header_magic0_q  <= I_rx_read_data;
                        10'd1:  header_magic1_q  <= I_rx_read_data;
                        10'd2:  header_version_q <= I_rx_read_data;
                        10'd3:  header_kind_q    <= I_rx_read_data;
                        10'd4:  request_session_q[31:24] <= I_rx_read_data;
                        10'd5:  request_session_q[23:16] <= I_rx_read_data;
                        10'd6:  request_session_q[15:8]  <= I_rx_read_data;
                        10'd7:  request_session_q[7:0]   <= I_rx_read_data;
                        10'd8:  request_frame_q[31:24]   <= I_rx_read_data;
                        10'd9:  request_frame_q[23:16]   <= I_rx_read_data;
                        10'd10: request_frame_q[15:8]    <= I_rx_read_data;
                        10'd11: request_frame_q[7:0]     <= I_rx_read_data;
                        10'd12: request_payload_len_q[15:8] <= I_rx_read_data;
                        10'd13: request_payload_len_q[7:0]  <= I_rx_read_data;
                        10'd14: begin
                            request_mask_q <= I_rx_read_data[6:0];
                            request_mask_reserved_q <= I_rx_read_data[7];
                        end
                        10'd15: request_plate_valid_q <= I_rx_read_data[0];
                        default: begin end
                    endcase

                    if (rx_index_q == 10'd15) begin
                        // At byte 15 all earlier header fields are already
                        // registered.  Check reserved bits before touching
                        // tile RAM; no partial/invalid request may compute.
                        if ((I_rx_length == REQUEST_LENGTH) &&
                            (header_magic0_q == 8'h4c) &&
                            (header_magic1_q == 8'h50) &&
                            (header_version_q == 8'h01) &&
                            (header_kind_q == 8'h01) &&
                            (request_payload_len_q == REQUEST_PAYLOAD_LENGTH) &&
                            (request_mask_reserved_q == 1'b0) &&
                            (I_rx_read_data[7:1] == 7'd0) &&
                            (rx_index_q == 10'd15)) begin
                            state <= S_HEADER_ID_CHECK;
                        end else begin
                            state <= S_DROP;
                        end
                    end else begin
                        rx_index_q <= rx_index_q + 10'd1;
                        state <= S_RX_HEADER_REQ;
                    end
                end

                S_HEADER_ID_CHECK: begin
                    // Reject a replay before touching tile RAM.  A different
                    // opaque session ID is treated as a new application epoch
                    // and is checked/reset after its payload is copied.
                    if (session_seen_q &&
                        (((request_session_q == active_session_q) &&
                          (request_frame_q <= last_frame_q)) ||
                         (retired_session_valid_q &&
                          (request_session_q == retired_session_q)))) begin
                        state <= S_DROP;
                    end else begin
                        tile_byte_q <= 10'd0;
                        tile_row_q <= 9'd0;
                        tile_word_q <= 32'd0;
                        state <= S_RX_TILE_REQ;
                    end
                end

                S_RX_TILE_REQ: begin
                    state <= S_RX_TILE_WAIT;
                end

                S_RX_TILE_WAIT: begin
                    case (tile_byte_q[1:0])
                        2'd0: tile_word_q[7:0]   <= I_rx_read_data;
                        2'd1: tile_word_q[15:8]  <= I_rx_read_data;
                        2'd2: tile_word_q[23:16] <= I_rx_read_data;
                        2'd3: tile_word_q[31:24] <= I_rx_read_data;
                        default: begin end
                    endcase
                    if (tile_byte_q[1:0] == 2'd3)
                        state <= S_RX_TILE_WRITE;
                    else begin
                        tile_byte_q <= tile_byte_q + 10'd1;
                        state <= S_RX_TILE_REQ;
                    end
                end

                S_RX_TILE_WRITE: begin
                    // The fourth byte was captured in the preceding state;
                    // the complete little-byte-first row is now presented to
                    // C13 for one clock.
                    if (tile_byte_q == 10'd895) begin
                        state <= S_SESSION_CHECK;
                    end else begin
                        tile_byte_q <= tile_byte_q + 10'd1;
                        tile_row_q  <= tile_row_q + 9'd1;
                        state <= S_RX_TILE_REQ;
                    end
                end

                S_SESSION_CHECK: begin
                    if (!session_seen_q || (request_session_q != active_session_q)) begin
                        if (session_seen_q) begin
                            retired_session_valid_q <= 1'b1;
                            retired_session_q <= active_session_q;
                        end
                        session_seen_q   <= 1'b1;
                        active_session_q <= request_session_q;
                        last_frame_q     <= request_frame_q;
                        new_session_q    <= 1'b1;
                        matcher_rst_n_q  <= 1'b0;
                        state            <= S_MATCHER_RESET;
                    end else if (request_frame_q <= last_frame_q) begin
                        // Replay/old frame: release it without feeding either
                        // matcher or response RAM.
                        state <= S_DROP;
                    end else begin
                        last_frame_q    <= request_frame_q;
                        new_session_q   <= 1'b0;
                        state           <= S_FRAME_START;
                    end
                end

                S_MATCHER_RESET: begin
                    matcher_rst_n_q <= 1'b1;
                    state <= S_FRAME_START;
                end

                S_FRAME_START: begin
                    glyph_slot_q <= 3'd0;
                    state <= S_GLYPH_START;
                end

                S_GLYPH_START: begin
                    state <= S_GLYPH_WAIT;
                end

                S_GLYPH_WAIT: begin
                    if (matcher_glyph_done) begin
                        if (glyph_slot_q == 3'd6) begin
                            state <= S_FRAME_WAIT;
                        end else begin
                            glyph_slot_q <= glyph_slot_q + 3'd1;
                            state <= S_GLYPH_START;
                        end
                    end
                end

                S_FRAME_WAIT: begin
                    // C14 updates persistent stable outputs and pulses
                    // o_frame_result at the same edge as frame completion.
                    // Sampling here one cycle later captures the committed
                    // stable state, including a frame with no update.
                    if (matcher_frame_result) begin
                        response_session_q   <= request_session_q;
                        response_frame_q     <= request_frame_q;
                        response_res_q       <= matcher_res;
                        response_res_n_q     <= matcher_res_n;
                        response_res_valid_q <= matcher_res_valid;
                        response_raw_valid_q <= matcher_raw_valid;
                        response_raw_code_q  <= matcher_raw_candidate_code;
                        O_frame_result_pulse <= 1'b1;
                        state <= S_TX_WAIT;
                    end
                end

                S_TX_WAIT: begin
                    if (!I_tx_busy) begin
                        tx_index_q <= 6'd0;
                        state <= S_TX_WRITE;
                    end
                end

                S_TX_WRITE: begin
                    if (!I_tx_busy) begin
                        if (tx_index_q == 6'd31) begin
                            state <= S_TX_COMMIT;
                        end else begin
                            tx_index_q <= tx_index_q + 6'd1;
                        end
                    end
                end

                S_TX_COMMIT: begin
                    if (!I_tx_busy)
                        state <= S_TX_DONE;
                end

                S_TX_DONE: begin
                    if (I_tx_done)
                        state <= S_RELEASE;
                end

                S_DROP: begin
                    state <= S_RELEASE;
                end

                S_RELEASE: begin
                    // Pulse release once, then wait for the common link's
                    // valid hold to fall before accepting another request.
                    drain_q <= 8'd0;
                    state <= S_RX_DRAIN;
                end

                S_RX_DRAIN: begin
                    if (!I_rx_valid) begin
                        drain_q <= 8'd0;
                        state <= S_IDLE;
                    end else if (drain_q >= P_DRAIN_CYCLES - 1) begin
                        // A wedged peer cannot block the stream forever.
                        drain_q <= 8'd0;
                        state <= S_IDLE;
                    end else begin
                        drain_q <= drain_q + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase

            // Keep this after the normal FSM case so it is the final state
            // decision only when an active request exceeded its budget.
            if ((state != S_IDLE) && (state != S_RELEASE) &&
                (state != S_RX_DRAIN) && (P_WATCHDOG_CYCLES > 0) &&
                (watchdog_q >= P_WATCHDOG_CYCLES - 1)) begin
                response_res_valid_q <= 1'b0;
                matcher_rst_n_q <= 1'b0;
                O_frame_result_pulse <= 1'b1;
                drain_q <= 8'd0;
                state <= S_RELEASE;
            end
        end
    end

endmodule
