`timescale 1ns/1ps
`define CHAR_SHAPE_DEBUG
// ============================================================================
// Desktop-blue H64 native integration: parameterized C14 shape matcher.
// Source baseline: two_board/EG4D/rtl/candidate14/char_match.v
// Production RTL is intentionally untouched.  This local integration defaults
// to the trained Desktop-14 H64 model while keeping the C14 scheduling,
// quantization, masks, gates and stability interfaces.
// ============================================================================
`timescale 1ns/1ps
// ============================================================================
// Company: Anlogic contest isolated candidate13 integration
// Engineer: C13 integration worker
// Create Date: 2026-10-10
// Design Name: candidate13 serial shape matcher wrapper
// Module Name: char_match
// Description: description/candidate13/char_match_spec.md
// Simulations: tb/tb_plate_image_eval.v and wrapper/tb_char_match_smoke.v
// Referrences: matcher/char_shape_match_spec.md; stability/char_plate_stability_spec.md
// Dependencies: embedded integer MLP, model weights and plate stability logic
// Version: V1.0
// Revision Date: 2026-10-10
// History: 2026-10-10 V1.0 serial classifier and whole-plate event wrapper.
//
// 版权归属：候选13隔离集成；开发人员：C13 integration worker；创建日期：2026-10-10
// 设计名称：候选13串行字形匹配封装；模块名称：char_match
// 模块说明：公开真实特征握手、结果原子脉冲、整牌确认和状态探针。
// 仿真工程：tb/tb_plate_image_eval.v；参考资料：candidate13_spec.md
// 依赖文件：本文件内嵌 char_shape_match、模型权重与 char_plate_stability
// 当前版本：V1.0；修订日期：2026-10-10；修订历史：V1.0 串行分类和整牌事件封装。
// ============================================================================
module char_match #(
    parameter STABLE_EN = 1,
    parameter HOLD_K = 5,
    parameter LOST_K = 8,
    parameter integer HIDDEN_COUNT = 64
)(
    input wire clk,
    input wire rst_n,
    input wire i_glyph_start,
    input wire [2:0] i_glyph_slot,
    input wire i_glyph_valid,
    input wire i_feature_valid,
    input wire [7:0] i_feature_data,
    output wire o_feature_ready,
    output wire o_result_valid,
    output wire [7:0] o_result_code,
    output wire o_result_accepted,
    output wire o_glyph_done,
    output wire [2:0] o_result_slot,
    input wire i_frame_start,
    input wire i_frame_done,
    output wire [55:0] o_res,
    output wire [3:0] o_res_n,
    output wire o_res_valid,
    output wire [6:0] o_raw_valid,
    output wire o_frame_result,
    output wire [55:0] o_raw_candidate_code,
    output wire [6:0] o_raw_candidate_mask,
    output wire [3:0] o_match_state
);

    wire match_result_valid;
    wire [7:0] match_result_code;
    wire match_result_accepted;
    wire match_glyph_done;
    wire [2:0] match_result_slot;
    wire [55:0] stability_candidate_code;
    wire [6:0] stability_candidate_mask;

    char_shape_match_hidden_param #(.HIDDEN_COUNT(HIDDEN_COUNT)) u_shape_match (
        .clk(clk),
        .rst_n(rst_n),
        .glyph_start(i_glyph_start),
        .glyph_slot(i_glyph_slot),
        .glyph_valid(i_glyph_valid),
        .feature_valid(i_feature_valid),
        .feature_data(i_feature_data),
        .feature_ready(o_feature_ready),
        .result_valid(match_result_valid),
        .result_code(match_result_code),
        .result_accepted(match_result_accepted),
        .glyph_done(match_glyph_done),
        .result_slot(match_result_slot),
        .state_public(o_match_state)
    );

    char_plate_stability_hidden_param #(
        .STABLE_EN(STABLE_EN),
        .HOLD_K(HOLD_K),
        .LOST_K(LOST_K)
    ) u_plate_stability (
        .clk(clk),
        .rst_n(rst_n),
        .i_frame_start(i_frame_start),
        .i_glyph_result_valid(match_result_valid),
        .i_glyph_slot(match_result_slot),
        .i_glyph_code(match_result_code),
        .i_glyph_accepted(match_result_accepted),
        .i_frame_done(i_frame_done),
        .o_res(o_res),
        .o_res_n(o_res_n),
        .o_res_valid(o_res_valid),
        .o_raw_valid(o_raw_valid),
        .o_frame_result(o_frame_result),
        .o_candidate_code(stability_candidate_code),
        .o_candidate_valid(stability_candidate_mask)
    );

    assign o_result_valid = match_result_valid;
    assign o_result_code = match_result_code;
    assign o_result_accepted = match_result_accepted;
    assign o_glyph_done = match_glyph_done;
    assign o_result_slot = match_result_slot;
    assign o_raw_candidate_code = stability_candidate_code;
    assign o_raw_candidate_mask = stability_candidate_mask;
endmodule
module char_shape_match_hidden_param #(
    parameter integer HIDDEN_COUNT = 32
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       glyph_start,
    input  wire [2:0] glyph_slot,
    input  wire       glyph_valid,
    input  wire       feature_valid,
    input  wire [7:0] feature_data,
    output wire       feature_ready,
    output reg        result_valid,
    output reg  [7:0] result_code,
    output reg        result_accepted,
    output reg        glyph_done,
    output reg  [2:0] result_slot,
    // C14 integration probe: same public state contract as the verified 622 wrapper.
    output wire [3:0] state_public
);

    localparam integer FEATURE_COUNT_FIXED = 128;
    localparam integer PROV_CLASSES = 33;
    localparam integer ALNUM_CLASSES = 41;
    localparam integer W1_DEPTH = HIDDEN_COUNT * FEATURE_COUNT_FIXED;
    localparam integer W2_PROV_DEPTH = PROV_CLASSES * HIDDEN_COUNT;
    localparam integer W2_ALNUM_DEPTH = ALNUM_CLASSES * HIDDEN_COUNT;
    localparam integer HIDDEN_LAST = HIDDEN_COUNT - 1;

    localparam [3:0] S_IDLE          = 4'd0;
    localparam [3:0] S_CAPTURE       = 4'd1;
    localparam [3:0] S_NORM_READ     = 4'd2;
    localparam [3:0] S_NORM_DIV_INIT = 4'd3;
    localparam [3:0] S_DIV_RUN       = 4'd4;
    localparam [3:0] S_NORM_DIV_SAVE = 4'd5;
    localparam [3:0] S_HIDDEN_INIT   = 4'd6;
    localparam [3:0] S_HIDDEN_MAC    = 4'd7;
    localparam [3:0] S_HIDDEN_SAVE   = 4'd8;
    localparam [3:0] S_HIDDEN_DIV_SAVE=4'd9;
    localparam [3:0] S_OUT_INIT      = 4'd10;
    localparam [3:0] S_OUT_MAC       = 4'd11;
    localparam [3:0] S_OUT_SAVE      = 4'd12;
    localparam [3:0] S_DECIDE        = 4'd13;
    localparam [3:0] S_REJECT        = 4'd14;

    // The integer reference clips to the complete signed int8 range.  The
    // floating training path's +/-4.0 bound is not used by this RTL model.
    localparam signed [63:0] SHAPE_MIN_QX = -64'sd127;
    localparam signed [63:0] SHAPE_MAX_QX = 64'sd127;
    localparam signed [31:0] SHAPE_SCORE_MIN_SENTINEL = -32'sd2147483647 - 32'sd1;

    reg [3:0] state;
    reg [2:0] slot_q;
    reg       glyph_valid_q;
    reg [7:0] raw_feature [0:127];
    reg signed [7:0] q_feature [0:127];
    // Hidden activations are unsigned Q5.5 values in 0..224.  Keeping this
    // byte unsigned is required for the upper half of the ReLU range.
    reg [7:0] hidden_q [0:HIDDEN_COUNT-1];

    // The generated include supplies one model head for province and one for
    // alphanumeric classes.  The arrays are initialized from the bound model
    // snapshot and are marked as ROM candidates for implementation tools.
    (* force_ram *) (* ram_style = "bram" *) reg [63:0] shape_mean8_q24_prov [0:127];
    (* force_ram *) (* ram_style = "bram" *) reg [63:0] shape_std8_q24_prov [0:127];
    (* force_ram *) (* ram_style = "bram" *) reg [63:0] shape_mean8_q24_alnum [0:127];
    (* force_ram *) (* ram_style = "bram" *) reg [63:0] shape_std8_q24_alnum [0:127];
    (* force_ram *) (* ram_style = "bram" *) reg signed [7:0] shape_w1_prov [0:W1_DEPTH-1];
    (* force_ram *) (* ram_style = "bram" *) reg signed [7:0] shape_w1_alnum [0:W1_DEPTH-1];
    (* force_ram *) (* ram_style = "bram" *) reg signed [7:0] shape_w2_prov [0:W2_PROV_DEPTH-1];
    (* force_ram *) (* ram_style = "bram" *) reg signed [7:0] shape_w2_alnum [0:W2_ALNUM_DEPTH-1];
    (* force_ram *) (* ram_style = "bram" *) reg signed [31:0] shape_b1_prov [0:HIDDEN_COUNT-1];
    (* force_ram *) (* ram_style = "bram" *) reg signed [31:0] shape_b1_alnum [0:HIDDEN_COUNT-1];
    (* force_ram *) (* ram_style = "bram" *) reg signed [31:0] shape_b2_prov [0:32];
    (* force_ram *) (* ram_style = "bram" *) reg signed [31:0] shape_b2_alnum [0:40];
    reg [7:0] shape_code_prov [0:32];
    reg [7:0] shape_code_alnum [0:40];
`ifdef CHAR_SHAPE_DEBUG
    // Simulation-only score taps.  The production interface remains unchanged
    // when CHAR_SHAPE_DEBUG is not defined; the focused full-row bench uses
    // these taps to compare every class score before checking the final gate.
    reg signed [31:0] debug_scores [0:40];
`endif


    // Model constants are generated separately.  H64 must select the explicit
    // Local selector binds the trained Desktop-14 H64 JSON export.
`include "matcher_model_select.vh"

`ifndef SYNTHESIS
    initial begin
        if (SHAPE_MODEL_HIDDEN_COUNT != HIDDEN_COUNT)
            $fatal(1, "matcher model/RTL hidden count mismatch");
    end
`endif

    reg [6:0] feature_index;
    reg [6:0] hidden_index;
    reg [5:0] output_index;
    // The first layer consumes all 128 features; the same counter is reused
    // for the 32-wide output layer, so it must represent 0..127.
    reg [6:0] inner_index;
    reg signed [31:0] hidden_acc;
    reg signed [31:0] output_acc;
    reg signed [31:0] best_score;
    reg signed [31:0] second_score;
    reg [5:0] best_index;
    reg       best_found;
    // Synchronous ROM outputs are kept per model head.  They intentionally
    // have no reset: reset belongs to the control/data registers, while the
    // ROM read ports follow the vendor inference template.
    reg signed [7:0] w1_read_q_prov;
    reg signed [7:0] w1_read_q_alnum;
    reg signed [7:0] w2_read_q_prov;
    reg signed [7:0] w2_read_q_alnum;
    reg [63:0] mean_read_q_prov;
    reg [63:0] mean_read_q_alnum;
    reg [63:0] std_read_q_prov;
    reg [63:0] std_read_q_alnum;
    reg [7:0] raw_read_q;
    // Shared 64-cycle restoring divider.  The numerator is prepared with
    // shifts only; divider state is reused for Q24 input ties-to-even and
    // positive hidden half-up rescaling.
    reg [63:0] div_num_abs;
    reg [63:0] div_den;
    reg [64:0] div_rem;
    reg [63:0] div_quot;
    reg [6:0]  div_bit;
    reg        div_negative;
    reg        div_hidden_mode;

    // HIDDEN_COUNT is parameterized.  H64 needs 13 W1 address bits and
    // 12 W2 address bits; the row strides are 128 and HIDDEN_COUNT.
    wire [13:0] w1_address_wide = ({7'd0, hidden_index} << 7) + {7'd0, inner_index};
    wire [12:0] w1_address = w1_address_wide[12:0];
    wire [12:0] w1_next_address = w1_address + 13'd1;
    wire [17:0] w2_address_wide = (output_index * HIDDEN_COUNT) + inner_index;
    wire [11:0] w2_address = w2_address_wide[11:0];
    wire [11:0] w2_next_address = w2_address + 12'd1;
    wire signed [31:0] b1_read = (slot_q == 3'd0) ? shape_b1_prov[hidden_index] : shape_b1_alnum[hidden_index];
    wire signed [31:0] b2_read = (slot_q == 3'd0) ? shape_b2_prov[output_index] : shape_b2_alnum[output_index];
    wire signed [7:0] w1_read_q = (slot_q == 3'd0) ? w1_read_q_prov : w1_read_q_alnum;
    wire signed [7:0] w2_read_q = (slot_q == 3'd0) ? w2_read_q_prov : w2_read_q_alnum;
    wire [63:0] mean_read_q = (slot_q == 3'd0) ? mean_read_q_prov : mean_read_q_alnum;
    wire [63:0] std_read_q = (slot_q == 3'd0) ? std_read_q_prov : std_read_q_alnum;
    wire signed [15:0] w1_qx_ext = {{8{q_feature[inner_index][7]}}, q_feature[inner_index]};
    wire signed [15:0] w1_weight_ext = {{8{w1_read_q[7]}}, w1_read_q};
    wire signed [15:0] w1_product = w1_qx_ext * w1_weight_ext;
    // Extend the unsigned hidden value with a zero before multiplying by the
    // signed int8 output weight.  Both operands are 17 bits so the product is
    // not truncated by Verilog expression sizing.
    wire signed [16:0] w2_hidden_ext = $signed({1'b0, hidden_q[inner_index]});
    wire signed [16:0] w2_weight_ext = {{9{w2_read_q[7]}}, w2_read_q};
    wire signed [16:0] w2_product = w2_hidden_ext * w2_weight_ext;
    wire [5:0] last_output_index = (slot_q == 3'd0) ? 6'd32 : 6'd40;
    wire [31:0] active_w1_scale_q16 = (slot_q == 3'd0) ? SHAPE_PROV_W1_SCALE_Q16 : SHAPE_ALNUM_W1_SCALE_Q16;
    wire [63:0] active_w1_den = {32'd0, active_w1_scale_q16} <<< 4;

    assign feature_ready = (state == S_CAPTURE) && glyph_valid_q;
    assign state_public = state;

    function shape_allow_class;
        input [2:0] slot;
        input [5:0] class_index;
        begin
            case (slot)
                3'd0: shape_allow_class = (class_index <= 6'd32);
                3'd1: shape_allow_class = (class_index >= 6'd17);
                3'd2, 3'd3, 3'd4, 3'd5: shape_allow_class = (class_index >= 6'd7);
                3'd6: shape_allow_class = 1'b1;
                default: shape_allow_class = 1'b0;
            endcase
        end
    endfunction

    function signed [63:0] shape_norm_numerator;
        input [7:0] raw_value;
        input [63:0] mean_value;
        reg signed [63:0] difference;
        begin
            difference = $signed({1'b0, raw_value});
            difference = difference <<< 24;
            difference = difference - $signed(mean_value);
            shape_norm_numerator = difference <<< 4;
        end
    endfunction

    function signed [63:0] shape_qx_from_div;
        input [63:0] quotient;
        input [63:0] remainder;
        input [63:0] denominator;
        input        negative;
        reg [63:0] rounded;
        begin
            rounded = quotient;
            if ((remainder > (denominator >> 1)) ||
                ((remainder << 1) == denominator && quotient[0]))
                rounded = quotient + 1'b1;
            if (negative) shape_qx_from_div = -$signed(rounded);
            else shape_qx_from_div = $signed(rounded);
        end
    endfunction

    function [63:0] shape_hidden_from_div;
        input [63:0] quotient;
        input [63:0] remainder;
        input [63:0] denominator;
        begin
            // The serialized model uses (n + floor(d/2)) / d for this
            // positive quantity, so an exact half rounds upward.
            shape_hidden_from_div = quotient;
            if ((remainder << 1) >= denominator)
                shape_hidden_from_div = quotient + 1'b1;
        end
    endfunction

    wire [64:0] div_trial_rem = {div_rem[63:0], div_num_abs[div_bit]};
    wire        div_trial_ge  = (div_trial_rem >= {1'b0, div_den});
    wire signed [63:0] norm_numerator = shape_norm_numerator(raw_read_q, mean_read_q);
    wire signed [63:0] qx_div_value = shape_qx_from_div(div_quot, div_rem[63:0], div_den, div_negative);
    wire [63:0] hidden_div_value = shape_hidden_from_div(div_quot, div_rem[63:0], div_den);

    wire signed [32:0] score_margin = $signed({best_score[31], best_score}) - $signed({second_score[31], second_score});
    wire signed [31:0] active_score_min = (slot_q == 3'd0) ? SHAPE_PROV_SCORE_MIN : SHAPE_ALNUM_SCORE_MIN;
    wire signed [31:0] active_margin_min = (slot_q == 3'd0) ? SHAPE_PROV_MARGIN_MIN : SHAPE_ALNUM_MARGIN_MIN;
    wire active_gate = best_found && (best_score >= active_score_min) &&
                       (score_margin >= $signed({active_margin_min[31], active_margin_min}));

    // Read each ROM head synchronously.  The current FSM consumes the value
    // already in q and advances the address at the same edge, so INIT loads
    // address zero and each MAC cycle prefetches the next address without a
    // pipeline or handshake change.
    wire mean_rom_read_en = (state == S_NORM_READ);
    wire w1_rom_read_en = (state == S_HIDDEN_INIT) || (state == S_HIDDEN_MAC);
    wire w2_rom_read_en = (state == S_OUT_INIT) || (state == S_OUT_MAC);
    wire [12:0] w1_rom_address = (state == S_HIDDEN_INIT) ? w1_address : w1_next_address;
    wire [11:0] w2_rom_address = (state == S_OUT_INIT) ? w2_address : w2_next_address;

    always @(posedge clk) begin
        if (mean_rom_read_en) begin
            mean_read_q_prov <= shape_mean8_q24_prov[feature_index];
            mean_read_q_alnum <= shape_mean8_q24_alnum[feature_index];
            std_read_q_prov <= shape_std8_q24_prov[feature_index];
            std_read_q_alnum <= shape_std8_q24_alnum[feature_index];
        end
    end

    always @(posedge clk) begin
        if (w1_rom_read_en) begin
            w1_read_q_prov <= shape_w1_prov[w1_rom_address];
            w1_read_q_alnum <= shape_w1_alnum[w1_rom_address];
        end
    end

    always @(posedge clk) begin
        if (w2_rom_read_en) begin
            w2_read_q_prov <= shape_w2_prov[w2_rom_address];
            w2_read_q_alnum <= shape_w2_alnum[w2_rom_address];
        end
    end

    integer reset_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            slot_q <= 3'd0;
            glyph_valid_q <= 1'b0;
            feature_index <= 7'd0;
            hidden_index <= 7'd0;
            output_index <= 6'd0;
            inner_index <= 6'd0;
            hidden_acc <= 32'sd0;
            output_acc <= 32'sd0;
            best_score <= SHAPE_SCORE_MIN_SENTINEL;
            second_score <= SHAPE_SCORE_MIN_SENTINEL;
            best_index <= 6'd0;
            best_found <= 1'b0;
            raw_read_q <= 8'd0;
            div_num_abs <= 64'd0;
            div_den <= 64'd1;
            div_rem <= 65'd0;
            div_quot <= 64'd0;
            div_bit <= 7'd0;
            div_negative <= 1'b0;
            div_hidden_mode <= 1'b0;
            result_valid <= 1'b0;
            result_code <= 8'd0;
            result_accepted <= 1'b0;
            glyph_done <= 1'b0;
            result_slot <= 3'd0;
            for (reset_index = 0; reset_index < 128; reset_index = reset_index + 1) begin
                raw_feature[reset_index] <= 8'd0;
                q_feature[reset_index] <= 8'sd0;
            end
            for (reset_index = 0; reset_index < HIDDEN_COUNT; reset_index = reset_index + 1)
                hidden_q[reset_index] <= 8'd0;
`ifdef CHAR_SHAPE_DEBUG
            for (reset_index = 0; reset_index < 41; reset_index = reset_index + 1)
                debug_scores[reset_index] <= 32'sd0;
`endif
        end else begin
            result_valid <= 1'b0;
            result_accepted <= 1'b0;
            glyph_done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (glyph_start) begin
                        slot_q <= glyph_slot;
                        glyph_valid_q <= glyph_valid && (glyph_slot <= 3'd6);
                        feature_index <= 7'd0;
                        best_found <= 1'b0;
                        best_score <= SHAPE_SCORE_MIN_SENTINEL;
                        second_score <= SHAPE_SCORE_MIN_SENTINEL;
                        best_index <= 6'd0;
                        if (glyph_valid && (glyph_slot <= 3'd6)) state <= S_CAPTURE;
                        else state <= S_REJECT;
                    end
                end
                S_CAPTURE: begin
                    if (feature_valid) begin
                        raw_feature[feature_index] <= feature_data;
                        if (feature_index == 7'd127) begin
                            feature_index <= 7'd0;
                            state <= S_NORM_READ;
                        end else feature_index <= feature_index + 1'b1;
                    end
                end
                S_NORM_READ: begin
                    raw_read_q <= raw_feature[feature_index];
                    state <= S_NORM_DIV_INIT;
                end
                S_NORM_DIV_INIT: begin
                    div_negative <= (norm_numerator < 0);
                    if (norm_numerator < 0) div_num_abs <= -norm_numerator;
                    else div_num_abs <= norm_numerator;
                    div_den <= (std_read_q == 64'd0) ? 64'd1 : std_read_q;
                    div_rem <= 65'd0;
                    div_quot <= 64'd0;
                    div_bit <= 7'd63;
                    div_hidden_mode <= 1'b0;
                    state <= S_DIV_RUN;
                end
                S_DIV_RUN: begin
                    if (div_trial_ge) begin
                        div_rem <= div_trial_rem - {1'b0, div_den};
                        div_quot[div_bit] <= 1'b1;
                    end else begin
                        div_rem <= div_trial_rem;
                        div_quot[div_bit] <= 1'b0;
                    end
                    if (div_bit == 7'd0) begin
                        state <= div_hidden_mode ? S_HIDDEN_DIV_SAVE : S_NORM_DIV_SAVE;
                    end else div_bit <= div_bit - 1'b1;
                end
                S_NORM_DIV_SAVE: begin
                    if (qx_div_value < SHAPE_MIN_QX)
                        q_feature[feature_index] <= -8'sd127;
                    else if (qx_div_value > SHAPE_MAX_QX)
                        q_feature[feature_index] <= 8'sd127;
                    else q_feature[feature_index] <= qx_div_value[7:0];
                    if (feature_index == 7'd127) begin
                        hidden_index <= 7'd0;
                        inner_index <= 6'd0;
                        state <= S_HIDDEN_INIT;
                    end else begin
                        feature_index <= feature_index + 1'b1;
                        state <= S_NORM_READ;
                    end
                end
                S_HIDDEN_INIT: begin
                    hidden_acc <= b1_read;
                    inner_index <= 6'd0;
                    state <= S_HIDDEN_MAC;
                end
                S_HIDDEN_MAC: begin
                    hidden_acc <= hidden_acc + {{16{w1_product[15]}}, w1_product};
                    if (inner_index == 7'd127) begin
                        state <= S_HIDDEN_SAVE;
                    end else begin
                        inner_index <= inner_index + 1'b1;
                    end
                end
                S_HIDDEN_SAVE: begin
                    if (hidden_acc <= 0) begin
                        hidden_q[hidden_index] <= 8'd0;
                        if (hidden_index == HIDDEN_LAST) begin
                            output_index <= 6'd0;
                            inner_index <= 6'd0;
                            state <= S_OUT_INIT;
                        end else begin
                            hidden_index <= hidden_index + 1'b1;
                            inner_index <= 6'd0;
                            state <= S_HIDDEN_INIT;
                        end
                    end else begin
                        div_num_abs <= {32'd0, hidden_acc} <<< 21;
                        div_den <= active_w1_den;
                        div_rem <= 65'd0;
                        div_quot <= 64'd0;
                        div_bit <= 7'd63;
                        div_negative <= 1'b0;
                        div_hidden_mode <= 1'b1;
                        state <= S_DIV_RUN;
                    end
                end
                S_HIDDEN_DIV_SAVE: begin
                    if (hidden_div_value > 64'd224) hidden_q[hidden_index] <= 8'd224;
                    else hidden_q[hidden_index] <= hidden_div_value[7:0];
                    if (hidden_index == HIDDEN_LAST) begin
                        output_index <= 6'd0;
                        inner_index <= 6'd0;
                        state <= S_OUT_INIT;
                    end else begin
                        hidden_index <= hidden_index + 1'b1;
                        inner_index <= 6'd0;
                        state <= S_HIDDEN_INIT;
                    end
                end
                S_OUT_INIT: begin
                    output_acc <= b2_read;
                    inner_index <= 6'd0;
                    state <= S_OUT_MAC;
                end
                S_OUT_MAC: begin
                    output_acc <= output_acc + {{15{w2_product[16]}}, w2_product};
                    if (inner_index == HIDDEN_LAST) begin
                        state <= S_OUT_SAVE;
                    end else begin
                        inner_index <= inner_index + 1'b1;
                    end
                end
                S_OUT_SAVE: begin
`ifdef CHAR_SHAPE_DEBUG
                    debug_scores[output_index] <= output_acc;
`endif
                    if (shape_allow_class(slot_q, output_index)) begin
                        if (!best_found || (output_acc > best_score)) begin
                            second_score <= best_score;
                            best_score <= output_acc;
                            best_index <= output_index;
                            best_found <= 1'b1;
                        end else if (output_acc > second_score) begin
                            second_score <= output_acc;
                        end
                    end
                    if (output_index == last_output_index) state <= S_DECIDE;
                    else begin
                        output_index <= output_index + 1'b1;
                        inner_index <= 6'd0;
                        state <= S_OUT_INIT;
                    end
                end
                S_DECIDE: begin
                    result_valid <= 1'b1;
                    glyph_done <= 1'b1;
                    result_slot <= slot_q;
                    if (active_gate) begin
                        result_accepted <= 1'b1;
                        result_code <= (slot_q == 3'd0) ? shape_code_prov[best_index] : shape_code_alnum[best_index];
                    end else begin
                        result_accepted <= 1'b0;
                        result_code <= 8'd0;
                    end
                    state <= S_IDLE;
                end
                S_REJECT: begin
                    result_valid <= 1'b1;
                    result_accepted <= 1'b0;
                    result_code <= 8'd0;
                    glyph_done <= 1'b1;
                    result_slot <= slot_q;
                    state <= S_IDLE;
                end
                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
module char_plate_stability_hidden_param #(
    parameter STABLE_EN = 1,
    parameter HOLD_K = 5,
    parameter LOST_K = 8
)(
    input wire clk,
    input wire rst_n,
    input wire i_frame_start,
    input wire i_glyph_result_valid,
    input wire [2:0] i_glyph_slot,
    input wire [7:0] i_glyph_code,
    input wire i_glyph_accepted,
    input wire i_frame_done,
    output reg [55:0] o_res,
    output reg [3:0] o_res_n,
    output reg o_res_valid,
    output reg [6:0] o_raw_valid,
    output reg o_frame_result,
    output wire [55:0] o_candidate_code,
    output wire [6:0] o_candidate_valid
);
    reg frame_open;
    reg [55:0] candidate_plate;
    reg [6:0] candidate_valid;
    reg [55:0] pending_plate;
    reg [7:0] pending_count;
    reg [7:0] lost_count;
    reg [55:0] plate_next;
    reg [6:0] valid_next;
    reg [3:0] valid_count;
    integer slot_index;

    // Forward a last-glyph result arriving with frame_done. A new frame
    // always starts from zero, so a missed slot cannot inherit an old code.
    always @(*) begin
        plate_next = i_frame_start ? 56'd0 : candidate_plate;
        valid_next = i_frame_start ? 7'd0 : candidate_valid;
        if ((frame_open || i_frame_start) && i_glyph_result_valid &&
            (i_glyph_slot < 7)) begin
            plate_next[i_glyph_slot*8 +: 8] =
                (i_glyph_accepted && i_glyph_code != 0) ? i_glyph_code : 8'd0;
            valid_next[i_glyph_slot] = i_glyph_accepted && i_glyph_code != 0;
        end
        valid_count = 0;
        for (slot_index=0; slot_index<7; slot_index=slot_index+1)
            valid_count = valid_count + {3'd0,valid_next[slot_index]};
    end

    assign o_candidate_code = plate_next;
    assign o_candidate_valid = valid_next;

    wire complete_trusted = valid_next == 7'h7f;
    wire same_pending = pending_count != 0 && plate_next == pending_plate;
    wire confirmation_done = HOLD_K == 1 ||
        (same_pending && pending_count >= HOLD_K-1);
    wire loss_expired = LOST_K == 1 || lost_count >= LOST_K-1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_open <= 0;
            candidate_plate <= 0;
            candidate_valid <= 0;
            pending_plate <= 0;
            pending_count <= 0;
            lost_count <= 0;
            o_res <= 0;
            o_res_n <= 0;
            o_res_valid <= 0;
            o_raw_valid <= 0;
            o_frame_result <= 0;
        end else begin
            o_frame_result <= 0;
            if (frame_open || i_frame_start) begin
                candidate_plate <= plate_next;
                candidate_valid <= valid_next;
            end
            if (i_frame_start) frame_open <= 1;
            if (i_frame_done && (frame_open || i_frame_start)) begin
                frame_open <= 0;
                o_frame_result <= 1;
                o_raw_valid <= valid_next;
                if (STABLE_EN == 0) begin
                    o_res <= plate_next;
                    o_res_n <= valid_count;
                    o_res_valid <= valid_next != 0;
                end else if (complete_trusted) begin
                    if (o_res_valid && plate_next == o_res) begin
                        lost_count <= 0;
                        pending_plate <= 0;
                        pending_count <= 0;
                    end else if (confirmation_done) begin
                        o_res <= plate_next;
                        o_res_n <= 7;
                        o_res_valid <= 1;
                        lost_count <= 0;
                        pending_plate <= 0;
                        pending_count <= 0;
                    end else begin
                        pending_plate <= plate_next;
                        pending_count <= same_pending ? pending_count + 1'b1 : 8'd1;
                        if (!o_res_valid) lost_count <= 0;
                        else if (loss_expired) begin
                            o_res <= 0;
                            o_res_n <= 0;
                            o_res_valid <= 0;
                            lost_count <= 0;
                        end else lost_count <= lost_count + 1'b1;
                    end
                end else begin
                    pending_plate <= 0;
                    pending_count <= 0;
                    if (!o_res_valid) lost_count <= 0;
                    else if (loss_expired) begin
                        o_res <= 0;
                        o_res_n <= 0;
                        o_res_valid <= 0;
                        lost_count <= 0;
                    end else lost_count <= lost_count + 1'b1;
                end
            end
        end
    end

    // synthesis translate_off
    initial begin
        if (HOLD_K < 1 || HOLD_K > 255 || LOST_K < 1 || LOST_K > 255)
            $fatal(1,"HOLD_K and LOST_K must be in 1..255");
    end
    // synthesis translate_on
endmodule
