//=============================================================================
//  cap_master_tile_capture  --  C32-compatible streamed character tile capture
//-----------------------------------------------------------------------------
//  所属域   : cap
//  时钟域   : I_clk_pix (HDMI/application)
//  复位     : I_rst_pix_n
//  参数     : H_ACTIVE, V_ACTIVE
//  验证状态 : isolated pipeline candidate; simulation/STA pending
//=============================================================================
`timescale 1ns / 1ps
// Candidate13 streaming shape frontend / 候选 C13 流式字形前端
//
// The input contract intentionally matches the existing char_recognize block:
// one binary I_ink bit with I_ink_de, a vertical I_vsync interval, and the
// real char_locate geometry.  The frontend never stores a full video frame.
// It measures the current frame's ink bounds, uses a compatible previous-frame
// relative bound for the current capture when available, and writes sampled
// 32-bit tile rows to the master-board export bank.  The C13 feature engine
// is deliberately absent; the EG4 board consumes these exact rows.
//
// The crop sampler uses the exact floor(t*W/32) partition.  For each source
// coordinate it emits q+carry output columns, where q=floor(32/W), r=32%W,
// and the error accumulator starts at W-1.  The vertical path uses the same
// recurrence.  Complete rows are written during the line drain (at most four
// destination rows per glyph for the guarded H>=8 crop, hence <=28 rows).
// Tile RAM addresses use bank*224 + slot*32 + row, matching the 14,336-bit
// double-bank store rather than a sparse 512-word concatenated address.
module cap_master_tile_capture #(
    parameter integer H_ACTIVE = 1280,
    parameter integer V_ACTIVE = 720,
    parameter integer P_USE_EXTERNAL_FRAME_INDEX = 0,
    parameter integer P_USE_TILE_READY = 0,
    parameter integer P_PIPELINE_GEOMETRY = 0
)(
    input  wire        I_clk_pix,
    input  wire        I_rst_pix_n,
    input  wire        I_ink,
    input  wire        I_ink_de,
    input  wire        I_vsync,
    input  wire        I_detected,
    input  wire [10:0] I_char_up,
    input  wire [10:0] I_char_down,
    input  wire [6:0]  I_char_valid,
    input  wire [10:0] I_c1_left,
    input  wire [10:0] I_c1_right,
    input  wire [10:0] I_c2_left,
    input  wire [10:0] I_c2_right,
    input  wire [10:0] I_c3_left,
    input  wire [10:0] I_c3_right,
    input  wire [10:0] I_c4_left,
    input  wire [10:0] I_c4_right,
    input  wire [10:0] I_c5_left,
    input  wire [10:0] I_c5_right,
    input  wire [10:0] I_c6_left,
    input  wire [10:0] I_c6_right,
    input  wire [10:0] I_c7_left,
    input  wire [10:0] I_c7_right,
    // High while the external packet adapter owns the published bank.
    // A completed frame during this interval is dropped as a whole.
    input  wire        I_export_busy,
    // Abort cancels the incomplete frame; complete crop history is retained.
    input  wire        I_frame_abort,
    input  wire [31:0] I_external_frame_index,
    // With P_USE_TILE_READY=1, O_tile_write_en is valid held until acceptance.
    input  wire        I_tile_ready,

    output wire        O_tile_write_en,
    output wire [8:0]  O_tile_write_addr,
    output wire [31:0] O_tile_write_data,
    output reg         O_snapshot_valid,
    output reg         O_snapshot_drop,
    output reg         O_snapshot_bank,
    output reg  [6:0]  O_snapshot_mask,
    output reg  [31:0] O_snapshot_frame_index,
    output wire        O_capture_active,
    output wire        O_commit_active,
    output wire        O_geometry_ready,
    output wire        O_frame_ready
);


    localparam [2:0] INVALID_SLOT = 3'd7;
    localparam [2:0] P_IDLE = 3'd0;
    localparam [2:0] P_WAIT_COMMIT = 3'd1;
    localparam [2:0] P_GLYPH_START = 3'd2;
    localparam [2:0] P_GLYPH_WAIT = 3'd3;
    localparam [10:0] H_LAST = H_ACTIVE[10:0] - 11'd1;
    localparam [10:0] V_LAST = V_ACTIVE[10:0] - 11'd1;

    wire [10:0] input_left [0:6];
    wire [10:0] input_right [0:6];
    assign input_left[0] = I_c1_left;
    assign input_left[1] = I_c2_left;
    assign input_left[2] = I_c3_left;
    assign input_left[3] = I_c4_left;
    assign input_left[4] = I_c5_left;
    assign input_left[5] = I_c6_left;
    assign input_left[6] = I_c7_left;
    assign input_right[0] = I_c1_right;
    assign input_right[1] = I_c2_right;
    assign input_right[2] = I_c3_right;
    assign input_right[3] = I_c4_right;
    assign input_right[4] = I_c5_right;
    assign input_right[5] = I_c6_right;
    assign input_right[6] = I_c7_right;

    // Frozen source geometry and the derived capture crop are deliberately
    // named like char_recognize so a root-level probe can verify alignment.
    reg [10:0] box_left [0:6];
    reg [10:0] box_right [0:6];
    reg [10:0] box_top;
    reg [10:0] box_bottom;
    reg [6:0]  frame_mask;
    reg [10:0] capture_left [0:6];
    reg [10:0] capture_right [0:6];
    reg [10:0] capture_top [0:6];
    reg [10:0] capture_bottom [0:6];
    reg [11:0] capture_width [0:6];
    reg [11:0] capture_height [0:6];
    reg [11:0] capture_x_q [0:6];
    reg [11:0] capture_x_r [0:6];
    reg [11:0] capture_y_q [0:6];
    reg [11:0] capture_y_r [0:6];
    reg [31:0] S_x_floor_mask [0:6];
    reg [31:0] S_x_ceil_mask [0:6];
    reg [6:0]  capture_valid;

    reg [6:0]  previous_ink_valid;
    reg [10:0] previous_box_left [0:6];
    reg [10:0] previous_box_top [0:6];
    reg [11:0] previous_box_width [0:6];
    reg [11:0] previous_box_height [0:6];
    reg [10:0] previous_ink_left_rel [0:6];
    reg [10:0] previous_ink_right_rel [0:6];
    reg [10:0] previous_ink_top_rel [0:6];
    reg [10:0] previous_ink_bottom_rel [0:6];

    reg [2:0] current_ink_count [0:6];
    reg [6:0] current_ink_valid;
    reg [10:0] current_ink_min_x [0:6];
    reg [10:0] current_ink_max_x [0:6];
    reg [10:0] current_ink_min_y [0:6];
    reg [10:0] current_ink_max_y [0:6];

    reg [10:0] pixel_x;
    reg [10:0] pixel_y;
    reg vsync_delay;
    wire vs_rise = I_vsync && !vsync_delay;
    wire vs_fall = !I_vsync && vsync_delay;

    reg capture_bank;
    reg commit_bank;
    reg [31:0] row_acc [0:6];
    reg [11:0] x_err [0:6];
    // Phase is error - width. Its sign replaces the carry comparison while
    // retaining one feedback update for every selected source pixel.
    reg signed [12:0] S_x_phase_pix [0:6];
    reg [6:0]  x_dst [0:6];
    reg [10:0] row_rep_start [0:6];
    reg [5:0]  row_rep_count [0:6];
    reg        y_started [0:6];
    reg [11:0] y_err [0:6];
    reg [6:0]  y_dst [0:6];
    reg        S_y_update_active;
    reg [2:0]  S_y_update_slot;

    reg [6:0]  pending_valid;
    reg [31:0] pending_data [0:6];
    reg [4:0]  pending_y_start [0:6];
    reg [5:0]  pending_y_count [0:6];
    reg        commit_active;
    reg [2:0]  commit_slot;
    reg [4:0]  commit_rep;
    reg        capture_active;
    reg        frame_capture_done;
    // Source frame numbering is assigned at the geometry latch edge.  The
    // active index is later copied into the processing metadata at vs_rise.
    reg [31:0] capture_frame_index;
    reg [31:0] active_frame_index;
    reg        capture_seen;

    reg tile_write_en;
    reg [8:0] tile_write_addr;
    reg [31:0] tile_write_data;
    reg snapshot_pending;
    reg snapshot_pending_bank;
    reg [6:0] snapshot_mask_q;
    reg [31:0] snapshot_frame_index_q;

    assign O_tile_write_en = tile_write_en;
    assign O_tile_write_addr = tile_write_addr;
    assign O_tile_write_data = tile_write_data;
    assign O_capture_active = capture_active;
    assign O_commit_active = commit_active;


    // Geometry selection crosses the first register boundary. Feedback state
    // is read only in the next stage, so consecutive pixels see fresh state.
    reg S_pixel_valid_r1;
    reg S_pixel_valid_r2;
    reg [6:0] S_capture_select_r1;
    reg [6:0] S_capture_select_r2;
    reg [6:0] S_capture_valid_r1;
    reg [6:0] S_capture_valid_r2;
    reg [6:0] S_first_pixel_r1;
    reg S_first_pixel_r2;
    reg S_ink_r1;
    reg S_ink_r2;
    reg S_row_start_r1;
    reg S_row_start_r2;
    reg S_row_last_r1;
    reg S_row_last_r2;
    reg S_capture_bank_r1;
    reg S_capture_bank_r2;
    reg [4:0] S_rep_start_r1 [0:6];
    reg [4:0] S_rep_start_r2 [0:6];
    reg [5:0] S_rep_count_r1 [0:6];
    reg [5:0] S_rep_count_r2 [0:6];
    reg [6:0] S_mask_start_r2;
    reg S_mask_extra_r2;
    reg [31:0] S_floor_mask_r2;
    reg [31:0] S_ceil_mask_r2;
    reg S_frame_bad_pix;
    reg S_finalize_pending_pix;
    wire W_tile_ready_pix = (P_USE_TILE_READY == 0) || (I_tile_ready === 1'b1);
    wire W_pipeline_empty_pix = !S_pixel_valid_r1 && !S_pixel_valid_r2;
    reg S_history_event_r1;
    wire W_history_empty_pix = !S_history_event_r1;
    wire W_frame_open_ready_pix = W_pipeline_empty_pix && W_history_empty_pix && !S_finalize_pending_pix;
    wire W_row_overrun_pix = S_pixel_valid_r2 && S_row_last_r2 &&
        !S_frame_bad_pix && (P_USE_TILE_READY != 0) &&
        (commit_active || (pending_valid != 7'd0));
    wire [31:0] W_pixel_mask_pix = (S_mask_start_r2 < 7'd32) ?
        ((S_mask_extra_r2 ? S_ceil_mask_r2 : S_floor_mask_r2) <<
        S_mask_start_r2[4:0]) : 32'd0;
    integer S_pipeline_slot;

    // Source selection and coordinate subtraction stop at a register before
    // the per-slot min/max feedback. Each update lane has a constant index.
    reg [6:0] S_history_select_r1;
    reg [10:0] S_history_relative_x_r1 [0:6];
    reg [10:0] S_history_relative_y_r1;
    integer S_history_slot;


    // Geometry preparation is optional; the compatibility path remains
    // combinational at the original VS edge when the parameter is zero.
    localparam [2:0] P_GEOMETRY_SPAN = 3'd1;
    localparam [2:0] P_GEOMETRY_SELECT = 3'd2;
    localparam [2:0] P_GEOMETRY_CROP = 3'd3;
    localparam [2:0] P_GEOMETRY_SCALE = 3'd4;
    reg [2:0] S_geometry_stage_pix;
    reg S_geometry_ready_pix;
    reg S_geometry_detected_pix;
    reg [6:0] S_geometry_mask_pix;
    reg [10:0] S_geometry_up_pix;
    reg [10:0] S_geometry_down_pix;
    reg [10:0] S_geometry_left_pix [0:6];
    reg [10:0] S_geometry_right_pix [0:6];
    reg [6:0] S_history_valid_pix;
    reg [10:0] S_history_box_left_pix [0:6];
    reg [10:0] S_history_box_top_pix [0:6];
    reg [11:0] S_history_box_width_pix [0:6];
    reg [11:0] S_history_box_height_pix [0:6];
    reg [10:0] S_history_ink_left_pix [0:6];
    reg [10:0] S_history_ink_right_pix [0:6];
    reg [10:0] S_history_ink_top_pix [0:6];
    reg [10:0] S_history_ink_bottom_pix [0:6];
    reg [11:0] S_source_width_pix [0:6];
    reg [11:0] S_source_height_pix;
    reg [11:0] S_history_width_pix [0:6];
    reg [11:0] S_history_height_pix [0:6];
    reg [6:0] S_geometry_reuse_pix;
    reg [6:0] S_geometry_valid_pix;
    wire [21:0] W_geometry_x_qr_pix [0:6];
    wire [21:0] W_geometry_y_qr_pix [0:6];
    wire [63:0] W_geometry_x_masks_pix [0:6];
    wire W_geometry_violation_pix = (P_PIPELINE_GEOMETRY != 0) &&
        ((capture_active && I_ink_de && !S_geometry_ready_pix) ||
         (vs_rise && S_geometry_stage_pix != 3'd0) ||
         (vs_fall && I_ink_de));
    assign O_geometry_ready = S_geometry_ready_pix;
    assign O_frame_ready = !capture_active && S_geometry_stage_pix == 3'd0 &&
        W_pipeline_empty_pix && W_history_empty_pix && !S_finalize_pending_pix && !snapshot_pending &&
        !commit_active && pending_valid == 7'd0 && !tile_write_en;


    // C21 bounded-width phase: fixed numerator 32 quotient/remainder lookup.
    // span==0 is the invalid guard; span>32 returns q=0,r=32.
    function [21:0] div32_qr_lookup;
        input [11:0] span;
        begin
            case (span)
                12'd0:  div32_qr_lookup = {11'd0,  11'd0};
                12'd1:  div32_qr_lookup = {11'd32, 11'd0};
                12'd2:  div32_qr_lookup = {11'd16, 11'd0};
                12'd3:  div32_qr_lookup = {11'd10, 11'd2};
                12'd4:  div32_qr_lookup = {11'd8,  11'd0};
                12'd5:  div32_qr_lookup = {11'd6,  11'd2};
                12'd6:  div32_qr_lookup = {11'd5,  11'd2};
                12'd7:  div32_qr_lookup = {11'd4,  11'd4};
                12'd8:  div32_qr_lookup = {11'd4,  11'd0};
                12'd9:  div32_qr_lookup = {11'd3,  11'd5};
                12'd10: div32_qr_lookup = {11'd3,  11'd2};
                12'd11: div32_qr_lookup = {11'd2,  11'd10};
                12'd12: div32_qr_lookup = {11'd2,  11'd8};
                12'd13: div32_qr_lookup = {11'd2,  11'd6};
                12'd14: div32_qr_lookup = {11'd2,  11'd4};
                12'd15: div32_qr_lookup = {11'd2,  11'd2};
                12'd16: div32_qr_lookup = {11'd2,  11'd0};
                12'd17: div32_qr_lookup = {11'd1,  11'd15};
                12'd18: div32_qr_lookup = {11'd1,  11'd14};
                12'd19: div32_qr_lookup = {11'd1,  11'd13};
                12'd20: div32_qr_lookup = {11'd1,  11'd12};
                12'd21: div32_qr_lookup = {11'd1,  11'd11};
                12'd22: div32_qr_lookup = {11'd1,  11'd10};
                12'd23: div32_qr_lookup = {11'd1,  11'd9};
                12'd24: div32_qr_lookup = {11'd1,  11'd8};
                12'd25: div32_qr_lookup = {11'd1,  11'd7};
                12'd26: div32_qr_lookup = {11'd1,  11'd6};
                12'd27: div32_qr_lookup = {11'd1,  11'd5};
                12'd28: div32_qr_lookup = {11'd1,  11'd4};
                12'd29: div32_qr_lookup = {11'd1,  11'd3};
                12'd30: div32_qr_lookup = {11'd1,  11'd2};
                12'd31: div32_qr_lookup = {11'd1,  11'd1};
                12'd32: div32_qr_lookup = {11'd1,  11'd0};
                default: div32_qr_lookup = {11'd0, 11'd32};
            endcase
        end
    endfunction

    // Decode both low masks directly from the crop span. This removes the
    // quotient -> plus-one -> variable-shift chain from VS-edge setup.
    // The packed result is {ceil_mask, floor_mask}; span zero retains the
    // original function result and invalid geometry is still cleared below.
    function [63:0] div32_span_masks_lookup;
        input [11:0] I_span;
        begin
            case (I_span)
                12'd1: div32_span_masks_lookup = {32'hffffffff, 32'hffffffff};
                12'd2: div32_span_masks_lookup = {32'h0001ffff, 32'h0000ffff};
                12'd3: div32_span_masks_lookup = {32'h000007ff, 32'h000003ff};
                12'd4: div32_span_masks_lookup = {32'h000001ff, 32'h000000ff};
                12'd5: div32_span_masks_lookup = {32'h0000007f, 32'h0000003f};
                12'd6: div32_span_masks_lookup = {32'h0000003f, 32'h0000001f};
                12'd7, 12'd8:
                    div32_span_masks_lookup = {32'h0000001f, 32'h0000000f};
                12'd9, 12'd10:
                    div32_span_masks_lookup = {32'h0000000f, 32'h00000007};
                12'd11, 12'd12, 12'd13, 12'd14, 12'd15, 12'd16:
                    div32_span_masks_lookup = {32'h00000007, 32'h00000003};
                12'd17, 12'd18, 12'd19, 12'd20, 12'd21, 12'd22,
                12'd23, 12'd24, 12'd25, 12'd26, 12'd27, 12'd28,
                12'd29, 12'd30, 12'd31, 12'd32:
                    div32_span_masks_lookup = {32'h00000003, 32'h00000001};
                default:
                    div32_span_masks_lookup = {32'h00000001, 32'h00000000};
            endcase
        end
    endfunction

    // Seven vertical states are consumed only at the last source pixel.
    // For rows of at least nine pixels, one lane updates them during pixels
    // 1..7; pixel 8 is the earliest possible consumer.  Short rows retain the
    // original parallel updates below.
    function [31:0] next_y_step;
        input [11:0] I_height;
        input [6:0] I_quotient;
        input [11:0] I_remainder;
        input [11:0] I_error;
        input [6:0] I_destination;
        input I_first;
        reg [11:0] W_sum;
        reg [5:0] W_count;
        reg [11:0] W_error;
        reg [6:0] W_destination;
        reg [6:0] W_start;
        begin
            W_sum = I_error + I_remainder;
            W_start = I_first ? 7'd0 : I_destination;
            if (I_first) begin
                if (I_remainder != 12'd0) begin
                    W_count = I_quotient[5:0] + 6'd1;
                    W_error = I_remainder - 12'd1;
                    W_destination = I_quotient + 7'd1;
                end else begin
                    W_count = I_quotient[5:0];
                    W_error = I_height - 12'd1;
                    W_destination = I_quotient;
                end
            end else if (W_sum >= I_height) begin
                W_count = I_quotient[5:0] + 6'd1;
                W_error = W_sum - I_height;
                W_destination = I_destination + {1'b0, I_quotient[5:0]} + 7'd1;
            end else begin
                W_count = I_quotient[5:0];
                W_error = W_sum;
                W_destination = I_destination + {1'b0, I_quotient[5:0]};
            end
            next_y_step = {W_count, W_error, W_destination, W_start};
        end
    endfunction

    wire [31:0] W_y_step;
    assign W_y_step = next_y_step(
        capture_height[S_y_update_slot], capture_y_q[S_y_update_slot][6:0],
        capture_y_r[S_y_update_slot], y_err[S_y_update_slot],
        y_dst[S_y_update_slot], pixel_y == capture_top[S_y_update_slot]);

    // Coordinate ports are 11 bits; inclusive endpoints therefore reach
    // span 2048.  Zero-extension prevents valid subtraction from wrapping.
    function [11:0] span12;
        input [10:0] high_value;
        input [10:0] low_value;
        begin
            if (high_value >= low_value)
                span12 = {1'b0, high_value} - {1'b0, low_value} + 12'd1;
            else
                span12 = 12'd0;
        end
    endfunction

    function [11:0] abs_delta12;
        input [11:0] left_value;
        input [11:0] right_value;
        begin
            if (left_value >= right_value)
                abs_delta12 = left_value - right_value;
            else
                abs_delta12 = right_value - left_value;
        end
    endfunction

    // The mask function is intentionally called once below after the
    // highest-priority capture_select mux.  Selected-slot arithmetic is
    // shared; per-slot state is retained below.
    function [31:0] repeat_mask;
        input [6:0] start_value;
        input [6:0] count_value;
        reg [31:0] W_low_mask;
        begin
            // Clip the interval to the 32-bit destination.  Explicit high-bit
            // guards retain the original behavior for every 7-bit argument;
            // two bounded shifts replace 32 separate range comparisons.
            if (count_value >= 7'd32)
                W_low_mask = 32'hffffffff;
            else
                W_low_mask = ~(32'hffffffff << count_value[4:0]);

            if (start_value >= 7'd32)
                repeat_mask = 32'd0;
            else
                repeat_mask = W_low_mask << start_value[4:0];
        end
    endfunction

    // Select the crop before decoding its scale.  Both crop candidates used
    // to have their own quotient/remainder lookup followed by an output mux.
    // Sharing each lookup preserves the frame-edge latch and all seven crops.
    wire [11:0] W_input_height;
    wire [11:0] W_input_width [0:6];
    wire [11:0] W_crop_width [0:6];
    wire [11:0] W_crop_height [0:6];
    wire [21:0] W_crop_x_qr [0:6];
    wire [21:0] W_crop_y_qr [0:6];
    wire [63:0] W_crop_x_masks [0:6];
    wire [6:0] W_geometry_valid;
    wire [6:0] W_crop_reuse;
    assign W_input_height = span12(I_char_down, I_char_up);

    genvar G_geometry_slot;
    generate
        for (G_geometry_slot = 0; G_geometry_slot < 7; G_geometry_slot = G_geometry_slot + 1) begin : gen_crop_scale
            assign W_input_width[G_geometry_slot] =
                span12(input_right[G_geometry_slot], input_left[G_geometry_slot]);
            assign W_geometry_valid[G_geometry_slot] =
                I_detected && I_char_valid[G_geometry_slot] &&
                input_right[G_geometry_slot] >= input_left[G_geometry_slot] &&
                I_char_down >= I_char_up &&
                W_input_width[G_geometry_slot] >= 12'd2 &&
                W_input_height >= 12'd8;
            assign W_crop_reuse[G_geometry_slot] =
                previous_ink_valid[G_geometry_slot] &&
                previous_box_width[G_geometry_slot] >= 12'd2 &&
                previous_box_height[G_geometry_slot] >= 12'd8 &&
                abs_delta12(W_input_width[G_geometry_slot], previous_box_width[G_geometry_slot]) <=
                    ((W_input_width[G_geometry_slot] >> 2) + 12'd2) &&
                abs_delta12(W_input_height, previous_box_height[G_geometry_slot]) <=
                    ((W_input_height >> 2) + 12'd2) &&
                abs_delta12({1'b0, input_left[G_geometry_slot]}, {1'b0, previous_box_left[G_geometry_slot]}) <= 12'd1 &&
                abs_delta12({1'b0, I_char_up}, {1'b0, previous_box_top[G_geometry_slot]}) <= 12'd1 &&
                {1'b0, previous_ink_right_rel[G_geometry_slot]} < W_input_width[G_geometry_slot] &&
                {1'b0, previous_ink_bottom_rel[G_geometry_slot]} < W_input_height;
            assign W_crop_width[G_geometry_slot] = W_crop_reuse[G_geometry_slot] ?
                span12(previous_ink_right_rel[G_geometry_slot], previous_ink_left_rel[G_geometry_slot]) :
                W_input_width[G_geometry_slot];
            assign W_crop_height[G_geometry_slot] = W_crop_reuse[G_geometry_slot] ?
                span12(previous_ink_bottom_rel[G_geometry_slot], previous_ink_top_rel[G_geometry_slot]) :
                W_input_height;
            assign W_crop_x_qr[G_geometry_slot] = div32_qr_lookup(W_crop_width[G_geometry_slot]);
            assign W_crop_x_masks[G_geometry_slot] =
                div32_span_masks_lookup(W_crop_width[G_geometry_slot]);
            assign W_crop_y_qr[G_geometry_slot] = div32_qr_lookup(W_crop_height[G_geometry_slot]);
            assign W_geometry_x_qr_pix[G_geometry_slot] = div32_qr_lookup(capture_width[G_geometry_slot]);
            assign W_geometry_y_qr_pix[G_geometry_slot] = div32_qr_lookup(capture_height[G_geometry_slot]);
            assign W_geometry_x_masks_pix[G_geometry_slot] = div32_span_masks_lookup(capture_width[G_geometry_slot]);
        end
    endgenerate

    // C21 local-slot phase.  Hit vectors are one-hot selected with the same
    // highest-index-wins rule as the C18 capture_slot/source_slot scan.
    // Each slot retains its independent x/y state.  The selected slot's
    // start/count and source-bound update share one arithmetic lane; the
    // vertical state and pending-row drain remain per-slot for equivalence.
    reg [6:0] capture_hit;
    reg [6:0] capture_select;
    reg [6:0] source_hit;
    reg [6:0] source_select;
    reg         shared_mask_valid;
    reg  [6:0]  shared_mask_start;
    reg  [6:0]  shared_mask_count;
    reg [31:0]  shared_x_mask;
    reg  [2:0]  W_capture_slot_index;
    reg  [6:0]  W_selected_x_start;
    reg  [6:0]  W_selected_x_count;
    reg [11:0]  W_selected_x_error_base;
    reg [11:0]  W_selected_x_error_sum;
    reg [11:0]  W_selected_x_error_next;
    reg [10:0]  W_selected_left;
    reg [11:0]  W_selected_width;
    reg [6:0]   W_selected_quotient;
    reg [11:0]  W_selected_remainder;
    reg [11:0]  W_selected_error;
    reg signed [12:0] W_selected_phase_pix;
    reg signed [12:0] W_x_phase_base_pix;
    reg signed [12:0] W_x_phase_sum_pix;
    reg signed [12:0] W_x_phase_next_pix;
    reg [6:0]   W_selected_destination;
    reg [31:0]  W_selected_floor_mask;
    reg [31:0]  W_selected_ceil_mask;
    reg         W_selected_extra;
    integer slot_scan;

    always @(*) begin
        capture_hit = 7'd0;
        source_hit = 7'd0;
        for (slot_scan = 0; slot_scan < 7; slot_scan = slot_scan + 1) begin
            if (capture_valid[slot_scan] &&
                pixel_x >= capture_left[slot_scan] &&
                pixel_x <= capture_right[slot_scan] &&
                pixel_y >= capture_top[slot_scan] &&
                pixel_y <= capture_bottom[slot_scan])
                capture_hit[slot_scan] = 1'b1;
            if (frame_mask[slot_scan] &&
                pixel_x >= box_left[slot_scan] &&
                pixel_x <= box_right[slot_scan] &&
                pixel_y >= box_top && pixel_y <= box_bottom)
                source_hit[slot_scan] = 1'b1;
        end

        // The original ascending loop overwrote the scalar on every hit.
        // Explicit masks preserve that last-match priority without a selected
        // array read in the update path.
        capture_select = 7'd0;
        capture_select[6] = capture_hit[6];
        capture_select[5] = capture_hit[5] && !capture_hit[6];
        capture_select[4] = capture_hit[4] && !(capture_hit[6] || capture_hit[5]);
        capture_select[3] = capture_hit[3] && !(capture_hit[6] || capture_hit[5] || capture_hit[4]);
        capture_select[2] = capture_hit[2] && !(capture_hit[6] || capture_hit[5] || capture_hit[4] || capture_hit[3]);
        capture_select[1] = capture_hit[1] && !(capture_hit[6] || capture_hit[5] || capture_hit[4] || capture_hit[3] || capture_hit[2]);
        capture_select[0] = capture_hit[0] && !(capture_hit[6] || capture_hit[5] || capture_hit[4] || capture_hit[3] || capture_hit[2] || capture_hit[1]);

        source_select = 7'd0;
        source_select[6] = source_hit[6];
        source_select[5] = source_hit[5] && !source_hit[6];
        source_select[4] = source_hit[4] && !(source_hit[6] || source_hit[5]);
        source_select[3] = source_hit[3] && !(source_hit[6] || source_hit[5] || source_hit[4]);
        source_select[2] = source_hit[2] && !(source_hit[6] || source_hit[5] || source_hit[4] || source_hit[3]);
        source_select[1] = source_hit[1] && !(source_hit[6] || source_hit[5] || source_hit[4] || source_hit[3] || source_hit[2]);
        source_select[0] = source_hit[0] && !(source_hit[6] || source_hit[5] || source_hit[4] || source_hit[3] || source_hit[2] || source_hit[1]);

    end

    always @(*) begin
        // The selection is already one-hot.  Read the state directly with
        // parallel masks instead of encoding an index then decoding a mux.
        W_selected_left = 11'd0;
        W_selected_width = 12'd0;
        W_selected_quotient = 7'd0;
        W_selected_remainder = 12'd0;
        W_selected_error = 12'd0;
        W_selected_phase_pix = 13'sd0;
        W_selected_destination = 7'd0;
        W_selected_floor_mask = 32'd0;
        W_selected_ceil_mask = 32'd0;
        for (slot_scan = 0; slot_scan < 7; slot_scan = slot_scan + 1) begin
            W_selected_left = W_selected_left |
                ({11{S_capture_select_r1[slot_scan]}} & capture_left[slot_scan]);
            W_selected_width = W_selected_width |
                ({12{S_capture_select_r1[slot_scan]}} & capture_width[slot_scan]);
            W_selected_quotient = W_selected_quotient |
                ({7{S_capture_select_r1[slot_scan]}} & capture_x_q[slot_scan][6:0]);
            W_selected_remainder = W_selected_remainder |
                ({12{S_capture_select_r1[slot_scan]}} & capture_x_r[slot_scan]);
            W_selected_error = W_selected_error |
                ({12{S_capture_select_r1[slot_scan]}} & x_err[slot_scan]);
            W_selected_phase_pix = W_selected_phase_pix |
                ({13{S_capture_select_r1[slot_scan]}} & S_x_phase_pix[slot_scan]);
            W_selected_destination = W_selected_destination |
                ({7{S_capture_select_r1[slot_scan]}} & x_dst[slot_scan]);
            W_selected_floor_mask = W_selected_floor_mask |
                ({32{S_capture_select_r1[slot_scan]}} & S_x_floor_mask[slot_scan]);
            W_selected_ceil_mask = W_selected_ceil_mask |
                ({32{S_capture_select_r1[slot_scan]}} & S_x_ceil_mask[slot_scan]);
        end

        // Only the selected crop can receive the current pixel.  The former
        // implementation calculated seven independent Bresenham lanes and
        // discarded six of them on overlap.  Select first, then calculate the
        // one lane that is observable at this pixel; the per-slot state is
        // still retained for the next pixel.
        W_capture_slot_index = 3'd0;
        if (S_capture_select_r1[1]) W_capture_slot_index = 3'd1;
        if (S_capture_select_r1[2]) W_capture_slot_index = 3'd2;
        if (S_capture_select_r1[3]) W_capture_slot_index = 3'd3;
        if (S_capture_select_r1[4]) W_capture_slot_index = 3'd4;
        if (S_capture_select_r1[5]) W_capture_slot_index = 3'd5;
        if (S_capture_select_r1[6]) W_capture_slot_index = 3'd6;


        shared_mask_valid = 1'b0;
        shared_mask_start = 7'd0;
        shared_mask_count = 7'd0;
        W_selected_x_start = 7'd0;
        W_selected_x_count = 7'd0;
        W_selected_x_error_base = 12'd0;
        W_selected_x_error_sum = 12'd0;
        W_selected_x_error_next = 12'd0;
        W_selected_extra = 1'b0;
        W_x_phase_base_pix = 13'sd0;
        W_x_phase_sum_pix = 13'sd0;
        W_x_phase_next_pix = 13'sd0;
        if (|S_capture_select_r1) begin
            shared_mask_valid = 1'b1;
            if (|(S_first_pixel_r1 & S_capture_select_r1)) begin
                W_selected_x_start = 7'd0;
                W_selected_x_error_base = W_selected_width - 12'd1;
                W_x_phase_base_pix = -13'sd1;
            end else begin
                W_selected_x_start = W_selected_destination;
                W_selected_x_error_base = W_selected_error;
                W_x_phase_base_pix = W_selected_phase_pix;
            end
            W_selected_x_error_sum = W_selected_x_error_base +
                                    W_selected_remainder;
            W_x_phase_sum_pix = W_x_phase_base_pix +
                               $signed({1'b0, W_selected_remainder});
            W_selected_extra = !W_x_phase_sum_pix[12];
            W_selected_x_count = W_selected_quotient +
                                 {6'd0, W_selected_extra};
            if (W_selected_extra)
                W_x_phase_next_pix = W_x_phase_sum_pix -
                                    $signed({1'b0, W_selected_width});
            else
                W_x_phase_next_pix = W_x_phase_sum_pix;
            W_selected_x_error_next = W_x_phase_next_pix +
                                      $signed({1'b0, W_selected_width});
            shared_mask_start = W_selected_x_start;
            shared_mask_count = W_selected_x_count;
        end
        shared_x_mask = 32'd0;
    end

    // Keep the packed lookup result scalar before ordinary array NBAs.  This
    // preserves the original edge and avoids a native tool concat-NBA issue.
    integer reset_index;
    integer capture_index;
    reg [6:0] process_mask_build;
    integer mask_index;
    always @(*) begin
        process_mask_build = 7'd0;
        for (mask_index = 0; mask_index < 7; mask_index = mask_index + 1)
            if (frame_mask[mask_index] && current_ink_valid[mask_index] &&
                current_ink_count[mask_index] >= 4 &&
                (current_ink_max_x[mask_index] - current_ink_min_x[mask_index] + 1) >= 2 &&
                (current_ink_max_y[mask_index] - current_ink_min_y[mask_index] + 1) >= 8)
                process_mask_build[mask_index] = 1'b1;
    end


    // Capture the winner and relative source coordinates independently.
    // DE-free cycles clear validity, so EOF can wait for exactly one drain.
    always @(posedge I_clk_pix or negedge I_rst_pix_n) begin
        if (!I_rst_pix_n) begin
            S_history_event_r1 <= 1'b0;
            S_history_select_r1 <= 7'd0;
            S_history_relative_y_r1 <= 11'd0;
            for (S_history_slot = 0; S_history_slot < 7; S_history_slot = S_history_slot + 1)
                S_history_relative_x_r1[S_history_slot] <= 11'd0;
        end else if ((I_frame_abort === 1'b1) || W_geometry_violation_pix) begin
            S_history_event_r1 <= 1'b0;
            S_history_select_r1 <= 7'd0;
        end else begin
            S_history_event_r1 <= capture_active && I_ink_de && I_ink && !vs_fall && !vs_rise &&
                ((P_PIPELINE_GEOMETRY == 0) || S_geometry_ready_pix);
            S_history_select_r1 <= source_select;
            S_history_relative_y_r1 <= pixel_y - box_top;
            for (S_history_slot = 0; S_history_slot < 7; S_history_slot = S_history_slot + 1)
                S_history_relative_x_r1[S_history_slot] <= pixel_x - box_left[S_history_slot];
        end
    end

    always @(posedge I_clk_pix or negedge I_rst_pix_n) begin
        if (!I_rst_pix_n) begin
            S_pixel_valid_r1 <= 1'b0;
            S_pixel_valid_r2 <= 1'b0;
            S_capture_select_r1 <= 7'd0;
            S_capture_select_r2 <= 7'd0;
            S_capture_valid_r1 <= 7'd0;
            S_capture_valid_r2 <= 7'd0;
            S_first_pixel_r1 <= 7'd0;
            S_first_pixel_r2 <= 1'b0;
            S_ink_r1 <= 1'b0;
            S_ink_r2 <= 1'b0;
            S_row_start_r1 <= 1'b0;
            S_row_start_r2 <= 1'b0;
            S_row_last_r1 <= 1'b0;
            S_row_last_r2 <= 1'b0;
            S_capture_bank_r1 <= 1'b0;
            S_capture_bank_r2 <= 1'b0;
            S_mask_start_r2 <= 7'd0;
            S_mask_extra_r2 <= 1'b0;
            S_floor_mask_r2 <= 32'd0;
            S_ceil_mask_r2 <= 32'd0;
            for (S_pipeline_slot = 0; S_pipeline_slot < 7; S_pipeline_slot = S_pipeline_slot + 1) begin
                S_rep_start_r1[S_pipeline_slot] <= 5'd0;
                S_rep_start_r2[S_pipeline_slot] <= 5'd0;
                S_rep_count_r1[S_pipeline_slot] <= 6'd0;
                S_rep_count_r2[S_pipeline_slot] <= 6'd0;
            end
        end else if ((I_frame_abort === 1'b1) || W_geometry_violation_pix) begin
            S_pixel_valid_r1 <= 1'b0;
            S_pixel_valid_r2 <= 1'b0;
        end else begin
            S_pixel_valid_r1 <= capture_active && I_ink_de && !vs_fall && !S_frame_bad_pix &&
                ((P_PIPELINE_GEOMETRY == 0) || S_geometry_ready_pix);
            S_pixel_valid_r2 <= S_pixel_valid_r1;
            S_capture_select_r1 <= capture_select;
            S_capture_select_r2 <= S_capture_select_r1;
            S_capture_valid_r1 <= capture_valid;
            S_capture_valid_r2 <= S_capture_valid_r1;
            S_ink_r1 <= I_ink;
            S_ink_r2 <= S_ink_r1;
            S_row_start_r1 <= pixel_x == 11'd0;
            S_row_start_r2 <= S_row_start_r1;
            S_row_last_r1 <= pixel_x == H_LAST;
            S_row_last_r2 <= S_row_last_r1;
            S_capture_bank_r1 <= capture_bank;
            S_capture_bank_r2 <= S_capture_bank_r1;
            S_first_pixel_r2 <= |(S_first_pixel_r1 & S_capture_select_r1);
            S_mask_start_r2 <= W_selected_x_start;
            S_mask_extra_r2 <= W_selected_extra;
            S_floor_mask_r2 <= W_selected_floor_mask;
            S_ceil_mask_r2 <= W_selected_ceil_mask;
            for (S_pipeline_slot = 0; S_pipeline_slot < 7; S_pipeline_slot = S_pipeline_slot + 1) begin
                S_first_pixel_r1[S_pipeline_slot] <= pixel_x == capture_left[S_pipeline_slot];
                S_rep_start_r1[S_pipeline_slot] <= row_rep_start[S_pipeline_slot][4:0];
                S_rep_count_r1[S_pipeline_slot] <= row_rep_count[S_pipeline_slot];
                S_rep_start_r2[S_pipeline_slot] <= S_rep_start_r1[S_pipeline_slot];
                S_rep_count_r2[S_pipeline_slot] <= S_rep_count_r1[S_pipeline_slot];
            end
        end
    end

    always @(posedge I_clk_pix or negedge I_rst_pix_n) begin
        if (!I_rst_pix_n) begin
            S_geometry_stage_pix <= 3'd0;
            S_geometry_ready_pix <= 1'b0;
            S_geometry_detected_pix <= 1'b0;
            S_geometry_mask_pix <= 7'd0;
            S_geometry_up_pix <= 11'd0;
            S_geometry_down_pix <= 11'd0;
            S_history_valid_pix <= 7'd0;
            S_source_height_pix <= 12'd0;
            S_geometry_reuse_pix <= 7'd0;
            S_geometry_valid_pix <= 7'd0;
            S_frame_bad_pix <= 1'b0;
            S_finalize_pending_pix <= 1'b0;
            vsync_delay <= 1'b0;
            pixel_x <= 11'd0;
            pixel_y <= 11'd0;
            capture_bank <= 1'b0;
            commit_bank <= 1'b0;
            capture_frame_index <= 32'd0;
            active_frame_index <= 32'd0;
            capture_seen <= 1'b0;
            capture_active <= 1'b0;
            frame_capture_done <= 1'b0;
            commit_active <= 1'b0;
            commit_slot <= 3'd0;
            commit_rep <= 5'd0;
            frame_mask <= 7'd0;
            capture_valid <= 7'd0;
            current_ink_valid <= 7'd0;
            previous_ink_valid <= 7'd0;
            pending_valid <= 7'd0;
            tile_write_en <= 1'b0;
            tile_write_addr <= 9'd0;
            tile_write_data <= 32'd0;
            snapshot_pending <= 1'b0;
            snapshot_pending_bank <= 1'b0;
            snapshot_mask_q <= 7'd0;
            snapshot_frame_index_q <= 32'd0;
            O_snapshot_valid <= 1'b0;
            O_snapshot_drop <= 1'b0;
            O_snapshot_bank <= 1'b0;
            O_snapshot_mask <= 7'd0;
            O_snapshot_frame_index <= 32'd0;
            for (reset_index = 0; reset_index < 7; reset_index = reset_index + 1) begin
                S_geometry_left_pix[reset_index] <= 11'd0;
                S_geometry_right_pix[reset_index] <= 11'd0;
                S_history_box_left_pix[reset_index] <= 11'd0;
                S_history_box_top_pix[reset_index] <= 11'd0;
                S_history_ink_left_pix[reset_index] <= 11'd0;
                S_history_ink_right_pix[reset_index] <= 11'd0;
                S_history_ink_top_pix[reset_index] <= 11'd0;
                S_history_ink_bottom_pix[reset_index] <= 11'd0;
                S_history_box_width_pix[reset_index] <= 12'd0;
                S_history_box_height_pix[reset_index] <= 12'd0;
                S_source_width_pix[reset_index] <= 12'd0;
                S_history_width_pix[reset_index] <= 12'd0;
                S_history_height_pix[reset_index] <= 12'd0;
                box_left[reset_index] <= 11'd0;
                box_right[reset_index] <= 11'd0;
                capture_left[reset_index] <= 11'd0;
                capture_right[reset_index] <= 11'd0;
                capture_top[reset_index] <= 11'd0;
                capture_bottom[reset_index] <= 11'd0;
                capture_width[reset_index] <= 12'd0;
                capture_height[reset_index] <= 12'd0;
                capture_x_q[reset_index] <= 12'd0;
                capture_x_r[reset_index] <= 12'd0;
                capture_y_q[reset_index] <= 12'd0;
                capture_y_r[reset_index] <= 12'd0;
                S_x_floor_mask[reset_index] <= 32'd0;
                S_x_ceil_mask[reset_index] <= 32'd0;
                previous_box_left[reset_index] <= 11'd0;
                previous_box_top[reset_index] <= 11'd0;
                previous_box_width[reset_index] <= 12'd0;
                previous_box_height[reset_index] <= 12'd0;
                previous_ink_left_rel[reset_index] <= 11'd0;
                previous_ink_right_rel[reset_index] <= 11'd0;
                previous_ink_top_rel[reset_index] <= 11'd0;
                previous_ink_bottom_rel[reset_index] <= 11'd0;
                current_ink_count[reset_index] <= 3'd0;
                current_ink_min_x[reset_index] <= 11'd0;
                current_ink_max_x[reset_index] <= 11'd0;
                current_ink_min_y[reset_index] <= 11'd0;
                current_ink_max_y[reset_index] <= 11'd0;
                row_acc[reset_index] <= 32'd0;
                x_err[reset_index] <= 12'd0;
                S_x_phase_pix[reset_index] <= 13'sd0;
                x_dst[reset_index] <= 7'd0;
                row_rep_start[reset_index] <= 11'd0;
                row_rep_count[reset_index] <= 6'd0;
                y_started[reset_index] <= 1'b0;
                y_err[reset_index] <= 12'd0;
                y_dst[reset_index] <= 7'd0;
                pending_data[reset_index] <= 32'd0;
                pending_y_start[reset_index] <= 5'd0;
                pending_y_count[reset_index] <= 6'd0;
            end
            box_top <= 11'd0;
            box_bottom <= 11'd0;
            S_y_update_active <= 1'b0;
            S_y_update_slot <= 3'd0;
        end else begin
            vsync_delay <= I_vsync;
            if (W_tile_ready_pix)
                tile_write_en <= 1'b0;
            O_snapshot_valid <= 1'b0;
            O_snapshot_drop <= 1'b0;

            if ((I_frame_abort === 1'b1) || W_geometry_violation_pix) begin
                S_geometry_stage_pix <= 3'd0;
                S_geometry_ready_pix <= 1'b0;
                if (W_geometry_violation_pix)
                    O_snapshot_drop <= 1'b1;
                capture_active <= 1'b0;
                capture_seen <= 1'b0;
                current_ink_valid <= 7'd0;
                pending_valid <= 7'd0;
                commit_active <= 1'b0;
                tile_write_en <= 1'b0;
                snapshot_pending <= 1'b0;
                S_y_update_active <= 1'b0;
                S_frame_bad_pix <= 1'b1;
                S_finalize_pending_pix <= 1'b0;
                for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1)
                    row_acc[capture_index] <= 32'd0;
            end else begin
            // Four registered preparation cycles use only the VS-edge
            // snapshots. Source metadata can change after the latch edge.
            if (P_PIPELINE_GEOMETRY != 0) begin
                case (S_geometry_stage_pix)
                    P_GEOMETRY_SPAN: begin
                        S_source_height_pix <= span12(S_geometry_down_pix, S_geometry_up_pix);
                        for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                            S_source_width_pix[capture_index] <= span12(S_geometry_right_pix[capture_index], S_geometry_left_pix[capture_index]);
                            S_history_width_pix[capture_index] <= span12(S_history_ink_right_pix[capture_index], S_history_ink_left_pix[capture_index]);
                            S_history_height_pix[capture_index] <= span12(S_history_ink_bottom_pix[capture_index], S_history_ink_top_pix[capture_index]);
                        end
                        S_geometry_stage_pix <= P_GEOMETRY_SELECT;
                    end
                    P_GEOMETRY_SELECT: begin
                        for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                            S_geometry_valid_pix[capture_index] <= S_geometry_detected_pix && S_geometry_mask_pix[capture_index] &&
                                S_geometry_right_pix[capture_index] >= S_geometry_left_pix[capture_index] &&
                                S_geometry_down_pix >= S_geometry_up_pix &&
                                S_source_width_pix[capture_index] >= 12'd2 && S_source_height_pix >= 12'd8;
                            S_geometry_reuse_pix[capture_index] <= S_history_valid_pix[capture_index] &&
                                S_history_box_width_pix[capture_index] >= 12'd2 && S_history_box_height_pix[capture_index] >= 12'd8 &&
                                abs_delta12(S_source_width_pix[capture_index], S_history_box_width_pix[capture_index]) <= ((S_source_width_pix[capture_index] >> 2) + 12'd2) &&
                                abs_delta12(S_source_height_pix, S_history_box_height_pix[capture_index]) <= ((S_source_height_pix >> 2) + 12'd2) &&
                                abs_delta12({1'b0, S_geometry_left_pix[capture_index]}, {1'b0, S_history_box_left_pix[capture_index]}) <= 12'd1 &&
                                abs_delta12({1'b0, S_geometry_up_pix}, {1'b0, S_history_box_top_pix[capture_index]}) <= 12'd1 &&
                                {1'b0, S_history_ink_right_pix[capture_index]} < S_source_width_pix[capture_index] &&
                                {1'b0, S_history_ink_bottom_pix[capture_index]} < S_source_height_pix;
                        end
                        S_geometry_stage_pix <= P_GEOMETRY_CROP;
                    end
                    P_GEOMETRY_CROP: begin
                        for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                            capture_valid[capture_index] <= S_geometry_valid_pix[capture_index];
                            if (S_geometry_valid_pix[capture_index]) begin
                                capture_left[capture_index] <= S_geometry_reuse_pix[capture_index] ? S_geometry_left_pix[capture_index] + S_history_ink_left_pix[capture_index] : S_geometry_left_pix[capture_index];
                                capture_right[capture_index] <= S_geometry_reuse_pix[capture_index] ? S_geometry_left_pix[capture_index] + S_history_ink_right_pix[capture_index] : S_geometry_right_pix[capture_index];
                                capture_top[capture_index] <= S_geometry_reuse_pix[capture_index] ? S_geometry_up_pix + S_history_ink_top_pix[capture_index] : S_geometry_up_pix;
                                capture_bottom[capture_index] <= S_geometry_reuse_pix[capture_index] ? S_geometry_up_pix + S_history_ink_bottom_pix[capture_index] : S_geometry_down_pix;
                                capture_width[capture_index] <= S_geometry_reuse_pix[capture_index] ? S_history_width_pix[capture_index] : S_source_width_pix[capture_index];
                                capture_height[capture_index] <= S_geometry_reuse_pix[capture_index] ? S_history_height_pix[capture_index] : S_source_height_pix;
                            end else begin
                                capture_left[capture_index] <= 11'd0;
                                capture_right[capture_index] <= 11'd0;
                                capture_top[capture_index] <= 11'd0;
                                capture_bottom[capture_index] <= 11'd0;
                                capture_width[capture_index] <= 12'd0;
                                capture_height[capture_index] <= 12'd0;
                            end
                        end
                        S_geometry_stage_pix <= P_GEOMETRY_SCALE;
                    end
                    P_GEOMETRY_SCALE: begin
                        for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                            if (S_geometry_valid_pix[capture_index]) begin
                                capture_x_q[capture_index] <= {1'b0, W_geometry_x_qr_pix[capture_index][21:11]};
                                capture_x_r[capture_index] <= {1'b0, W_geometry_x_qr_pix[capture_index][10:0]};
                                capture_y_q[capture_index] <= {1'b0, W_geometry_y_qr_pix[capture_index][21:11]};
                                capture_y_r[capture_index] <= {1'b0, W_geometry_y_qr_pix[capture_index][10:0]};
                                S_x_floor_mask[capture_index] <= W_geometry_x_masks_pix[capture_index][31:0];
                                S_x_ceil_mask[capture_index] <= W_geometry_x_masks_pix[capture_index][63:32];
                            end else begin
                                capture_x_q[capture_index] <= 12'd0;
                                capture_x_r[capture_index] <= 12'd0;
                                capture_y_q[capture_index] <= 12'd0;
                                capture_y_r[capture_index] <= 12'd0;
                                S_x_floor_mask[capture_index] <= 32'd0;
                                S_x_ceil_mask[capture_index] <= 32'd0;
                            end
                        end
                        S_geometry_stage_pix <= 3'd0;
                        S_geometry_ready_pix <= 1'b1;
                    end
                    default: begin
                        S_geometry_stage_pix <= 3'd0;
                    end
                endcase
            end
            if (vs_fall && (!W_frame_open_ready_pix ||
                ((P_PIPELINE_GEOMETRY != 0) && !O_frame_ready))) begin
                S_geometry_stage_pix <= 3'd0;
                S_geometry_ready_pix <= 1'b0;
                // A too-short blanking interval cannot replace live geometry.
                capture_active <= 1'b0;
                capture_seen <= 1'b0;
                O_snapshot_drop <= 1'b1;
            end else if (vs_fall) begin
                S_geometry_ready_pix <= (P_PIPELINE_GEOMETRY == 0);
                if (P_PIPELINE_GEOMETRY != 0) begin
                    S_geometry_stage_pix <= P_GEOMETRY_SPAN;
                    S_geometry_detected_pix <= I_detected;
                    S_geometry_mask_pix <= I_char_valid;
                    S_geometry_up_pix <= I_char_up;
                    S_geometry_down_pix <= I_char_down;
                    S_history_valid_pix <= previous_ink_valid;
                end
                S_frame_bad_pix <= 1'b0;
                S_y_update_active <= 1'b0;
                S_y_update_slot <= 3'd0;
                pixel_x <= 11'd0;
                pixel_y <= 11'd0;
                // Frame index zero is the first source frame whose geometry
                // is frozen here.  The corresponding processing metadata is
                // published later at the matching vs_rise/frame_start.
                active_frame_index <= (P_USE_EXTERNAL_FRAME_INDEX != 0) ?
                    I_external_frame_index : capture_frame_index;
                capture_frame_index <= capture_frame_index + 1'b1;
                capture_seen <= 1'b1;
                capture_active <= 1'b1;
                current_ink_valid <= 7'd0;
                for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                    box_left[capture_index] <= input_left[capture_index];
                    box_right[capture_index] <= input_right[capture_index];
                    current_ink_count[capture_index] <= 3'd0;
                    current_ink_min_x[capture_index] <= 11'd0;
                    current_ink_max_x[capture_index] <= 11'd0;
                    current_ink_min_y[capture_index] <= 11'd0;
                    current_ink_max_y[capture_index] <= 11'd0;
                    y_started[capture_index] <= 1'b0;
                    y_err[capture_index] <= 12'd0;
                    y_dst[capture_index] <= 7'd0;
                    if (P_PIPELINE_GEOMETRY != 0) begin
                        S_geometry_left_pix[capture_index] <= input_left[capture_index];
                        S_geometry_right_pix[capture_index] <= input_right[capture_index];
                        S_history_box_left_pix[capture_index] <= previous_box_left[capture_index];
                        S_history_box_top_pix[capture_index] <= previous_box_top[capture_index];
                        S_history_box_width_pix[capture_index] <= previous_box_width[capture_index];
                        S_history_box_height_pix[capture_index] <= previous_box_height[capture_index];
                        S_history_ink_left_pix[capture_index] <= previous_ink_left_rel[capture_index];
                        S_history_ink_right_pix[capture_index] <= previous_ink_right_rel[capture_index];
                        S_history_ink_top_pix[capture_index] <= previous_ink_top_rel[capture_index];
                        S_history_ink_bottom_pix[capture_index] <= previous_ink_bottom_rel[capture_index];
                    end else begin
                        if (W_geometry_valid[capture_index]) begin
                            capture_left[capture_index] <= W_crop_reuse[capture_index] ?
                                input_left[capture_index] + previous_ink_left_rel[capture_index] : input_left[capture_index];
                            capture_right[capture_index] <= W_crop_reuse[capture_index] ?
                                input_left[capture_index] + previous_ink_right_rel[capture_index] : input_right[capture_index];
                            capture_top[capture_index] <= W_crop_reuse[capture_index] ?
                                I_char_up + previous_ink_top_rel[capture_index] : I_char_up;
                            capture_bottom[capture_index] <= W_crop_reuse[capture_index] ?
                                I_char_up + previous_ink_bottom_rel[capture_index] : I_char_down;
                            capture_width[capture_index] <= W_crop_width[capture_index];
                            capture_height[capture_index] <= W_crop_height[capture_index];
                            capture_x_q[capture_index] <= {1'b0, W_crop_x_qr[capture_index][21:11]};
                            capture_x_r[capture_index] <= {1'b0, W_crop_x_qr[capture_index][10:0]};
                            capture_y_q[capture_index] <= {1'b0, W_crop_y_qr[capture_index][21:11]};
                            capture_y_r[capture_index] <= {1'b0, W_crop_y_qr[capture_index][10:0]};
                            S_x_floor_mask[capture_index] <= W_crop_x_masks[capture_index][31:0];
                            S_x_ceil_mask[capture_index] <= W_crop_x_masks[capture_index][63:32];
                            capture_valid[capture_index] <= 1'b1;
                        end else begin
                            capture_left[capture_index] <= 11'd0;
                            capture_right[capture_index] <= 11'd0;
                            capture_top[capture_index] <= 11'd0;
                            capture_bottom[capture_index] <= 11'd0;
                            capture_width[capture_index] <= 12'd0;
                            capture_height[capture_index] <= 12'd0;
                            capture_x_q[capture_index] <= 12'd0;
                            capture_x_r[capture_index] <= 12'd0;
                            capture_y_q[capture_index] <= 12'd0;
                            capture_y_r[capture_index] <= 12'd0;
                            S_x_floor_mask[capture_index] <= 32'd0;
                            S_x_ceil_mask[capture_index] <= 32'd0;
                            capture_valid[capture_index] <= 1'b0;
                        end
                    end
                end
                box_top <= I_char_up;
                box_bottom <= I_char_down;
                frame_mask <= I_detected ? I_char_valid : 7'd0;
            end else if (capture_active && I_ink_de &&
                ((P_PIPELINE_GEOMETRY == 0) || S_geometry_ready_pix)) begin
                if (H_ACTIVE >= 9 && S_y_update_active) begin
                    if (capture_valid[S_y_update_slot] &&
                        pixel_y >= capture_top[S_y_update_slot] &&
                        pixel_y <= capture_bottom[S_y_update_slot] &&
                        ((pixel_y == capture_top[S_y_update_slot]) || y_started[S_y_update_slot])) begin
                        row_rep_count[S_y_update_slot] <= W_y_step[31:26];
                        y_err[S_y_update_slot] <= W_y_step[25:14];
                        y_dst[S_y_update_slot] <= W_y_step[13:7];
                        row_rep_start[S_y_update_slot] <= {4'd0, W_y_step[6:0]};
                        y_started[S_y_update_slot] <= 1'b1;
                    end
                    if (S_y_update_slot == 3'd6)
                        S_y_update_active <= 1'b0;
                    else
                        S_y_update_slot <= S_y_update_slot + 3'd1;
                end
                if (pixel_x == 0) begin
                    if (H_ACTIVE >= 9) begin
                        S_y_update_active <= 1'b1;
                        S_y_update_slot <= 3'd0;
                    end
                    for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                        row_rep_start[capture_index] <= 11'd0;
                        row_rep_count[capture_index] <= 6'd0;
                        if (H_ACTIVE < 9 && capture_valid[capture_index] &&
                            pixel_y >= capture_top[capture_index] &&
                            pixel_y <= capture_bottom[capture_index]) begin
                            if (pixel_y == capture_top[capture_index]) begin
                                row_rep_start[capture_index] <= 11'd0;
                                // The initial error is height - 1.  A nonzero
                                // remainder always crosses height, leaving
                                // remainder - 1; avoid the add/compare/subtract
                                // chain in all seven first-row lanes.
                                if (capture_y_r[capture_index] != 12'd0) begin
                                    row_rep_count[capture_index] <= capture_y_q[capture_index][5:0] + 6'd1;
                                    y_err[capture_index] <= capture_y_r[capture_index] - 12'd1;
                                    y_dst[capture_index] <= capture_y_q[capture_index][6:0] + 7'd1;
                                end else begin
                                    row_rep_count[capture_index] <= capture_y_q[capture_index][5:0];
                                    y_err[capture_index] <= capture_height[capture_index] - 12'd1;
                                    y_dst[capture_index] <= capture_y_q[capture_index][6:0];
                                end
                                y_started[capture_index] <= 1'b1;
                            end else if (y_started[capture_index]) begin
                                row_rep_start[capture_index] <= {4'd0, y_dst[capture_index]};
                                if ((y_err[capture_index] + capture_y_r[capture_index]) >= capture_height[capture_index]) begin
                                    row_rep_count[capture_index] <= capture_y_q[capture_index][5:0] + 6'd1;
                                    y_err[capture_index] <= y_err[capture_index] + capture_y_r[capture_index] - capture_height[capture_index];
                                    y_dst[capture_index] <= y_dst[capture_index] +
                                        {1'b0, capture_y_q[capture_index][5:0]} + 7'd1;
                                end else begin
                                    row_rep_count[capture_index] <= capture_y_q[capture_index][5:0];
                                    y_err[capture_index] <= y_err[capture_index] + capture_y_r[capture_index];
                                    y_dst[capture_index] <= y_dst[capture_index] +
                                        {1'b0, capture_y_q[capture_index][5:0]};
                                end
                            end
                        end
                    end
                end

                if (pixel_x == H_LAST) begin
                    pixel_x <= 11'd0;
                    if (pixel_y < V_LAST)
                        pixel_y <= pixel_y + 1'b1;
                end else begin
                    pixel_x <= pixel_x + 1'b1;
                end
            end


            // Consume the registered winner with fresh constant-lane state.
            // This also consumes the final pixel while VS rises, before the
            // following drain edge can promote history or freeze the mask.
            if (S_history_event_r1) begin
                for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                    if (S_history_select_r1[capture_index]) begin
                        if (!current_ink_valid[capture_index]) begin
                            current_ink_valid[capture_index] <= 1'b1;
                            current_ink_count[capture_index] <= 3'd1;
                            current_ink_min_x[capture_index] <= S_history_relative_x_r1[capture_index];
                            current_ink_max_x[capture_index] <= S_history_relative_x_r1[capture_index];
                            current_ink_min_y[capture_index] <= S_history_relative_y_r1;
                            current_ink_max_y[capture_index] <= S_history_relative_y_r1;
                        end else begin
                            if (current_ink_count[capture_index] < 4)
                                current_ink_count[capture_index] <= current_ink_count[capture_index] + 1'b1;
                            if (S_history_relative_x_r1[capture_index] < current_ink_min_x[capture_index])
                                current_ink_min_x[capture_index] <= S_history_relative_x_r1[capture_index];
                            if (S_history_relative_x_r1[capture_index] > current_ink_max_x[capture_index])
                                current_ink_max_x[capture_index] <= S_history_relative_x_r1[capture_index];
                            if (S_history_relative_y_r1 < current_ink_min_y[capture_index])
                                current_ink_min_y[capture_index] <= S_history_relative_y_r1;
                            if (S_history_relative_y_r1 > current_ink_max_y[capture_index])
                                current_ink_max_y[capture_index] <= S_history_relative_y_r1;
                        end
                    end
                end
            end

            if (S_pixel_valid_r1) begin
                for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                    if (S_row_start_r1) begin
                        x_err[capture_index] <= 12'd0;
                        S_x_phase_pix[capture_index] <= -$signed({1'b0, capture_width[capture_index]});
                        x_dst[capture_index] <= 7'd0;
                    end
                    if (S_capture_select_r1[capture_index]) begin
                        x_err[capture_index] <= W_selected_x_error_next;
                        S_x_phase_pix[capture_index] <= W_x_phase_next_pix;
                        x_dst[capture_index] <= W_selected_x_start + W_selected_x_count;
                    end
                end
            end
            if (S_pixel_valid_r2 && !S_frame_bad_pix) begin
                for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                    if (S_row_start_r2)
                        row_acc[capture_index] <= 32'd0;
                    if (S_capture_select_r2[capture_index] && S_ink_r2) begin
                        if (S_first_pixel_r2)
                            row_acc[capture_index] <= W_pixel_mask_pix;
                        else
                            row_acc[capture_index] <= row_acc[capture_index] | W_pixel_mask_pix;
                    end
                    if (S_row_last_r2 && S_capture_valid_r2[capture_index] &&
                        S_rep_count_r2[capture_index] != 6'd0) begin
                        pending_valid[capture_index] <= 1'b1;
                        pending_data[capture_index] <= row_acc[capture_index] |
                            ((S_capture_select_r2[capture_index] && S_ink_r2) ? W_pixel_mask_pix : 32'd0);
                        pending_y_start[capture_index] <= S_rep_start_r2[capture_index];
                        pending_y_count[capture_index] <= S_rep_count_r2[capture_index];
                    end
                end
                if (S_row_last_r2) begin
                    if (W_row_overrun_pix) begin
                        // Row storage is bounded. A stalled consumer cannot
                        // silently overwrite an earlier row of this frame.
                        S_frame_bad_pix <= 1'b1;
                        snapshot_pending <= 1'b0;
                        capture_seen <= 1'b0;
                        pending_valid <= 7'd0;
                        commit_active <= 1'b0;
                        // Retain an offered word until ready, even when its
                        // frame is dropped. It belongs to the unpublished bank.
                        O_snapshot_drop <= 1'b1;
                    end else begin
                        commit_bank <= S_capture_bank_r2;
                        commit_active <= 1'b1;
                        commit_slot <= 3'd0;
                        commit_rep <= 5'd0;
                    end
                end
            end

            if (vs_rise) begin
                capture_active <= 1'b0;
                frame_capture_done <= 1'b1;
                commit_bank <= capture_bank;
                // Once a processing bank is active, keep capturing into the
                // other bank.  Additional frames are explicitly dropped at
                // the scheduler boundary instead of wrapping back and
                // overwriting the bank being read by the feature engine.
                // The initial high-level vs_rise after reset has no captured
                // source frame.  It must not toggle the bank or seed the
                // previous-frame geometry state.
                if (capture_seen && !S_frame_bad_pix && !W_row_overrun_pix) begin
                    S_finalize_pending_pix <= 1'b1;
                    snapshot_pending_bank <= capture_bank;
                    snapshot_frame_index_q <= active_frame_index;
                    if (I_export_busy) begin
                        // The adapter still owns the other bank: discard the
                        // just-completed capture and keep this bank for reuse.
                        snapshot_pending <= 1'b0;
                        O_snapshot_drop <= 1'b1;
                    end else begin
                        snapshot_pending <= 1'b1;
                        // The next capture uses the other bank.  The current
                        // bank remains immutable until snapshot_valid is seen.
                        capture_bank <= ~capture_bank;
                    end

                end
            end


            // Source statistics stop at EOF. Wait for late pipeline errors
            // before promoting them to reusable complete-frame history.
            if (S_finalize_pending_pix && W_pipeline_empty_pix && W_history_empty_pix) begin
                S_finalize_pending_pix <= 1'b0;
                if (!S_frame_bad_pix && !W_row_overrun_pix) begin
                    snapshot_mask_q <= process_mask_build;
                    for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                        if (frame_mask[capture_index] && current_ink_valid[capture_index] &&
                            current_ink_count[capture_index] >= 4 &&
                            (span12(current_ink_max_x[capture_index], current_ink_min_x[capture_index])) >= 12'd2 &&
                            (span12(current_ink_max_y[capture_index], current_ink_min_y[capture_index])) >= 12'd8) begin
                            previous_ink_valid[capture_index] <= 1'b1;
                            previous_box_left[capture_index] <= box_left[capture_index];
                            previous_box_top[capture_index] <= box_top;
                            previous_box_width[capture_index] <= span12(box_right[capture_index], box_left[capture_index]);
                            previous_box_height[capture_index] <= span12(box_bottom, box_top);
                            previous_ink_left_rel[capture_index] <= current_ink_min_x[capture_index];
                            previous_ink_right_rel[capture_index] <= current_ink_max_x[capture_index];
                            previous_ink_top_rel[capture_index] <= current_ink_min_y[capture_index];
                            previous_ink_bottom_rel[capture_index] <= current_ink_max_y[capture_index];
                        end else begin
                            previous_ink_valid[capture_index] <= 1'b0;
                            previous_box_left[capture_index] <= 11'd0;
                            previous_box_top[capture_index] <= 11'd0;
                            previous_box_width[capture_index] <= 12'd0;
                            previous_box_height[capture_index] <= 12'd0;
                            previous_ink_left_rel[capture_index] <= 11'd0;
                            previous_ink_right_rel[capture_index] <= 11'd0;
                            previous_ink_top_rel[capture_index] <= 11'd0;
                            previous_ink_bottom_rel[capture_index] <= 11'd0;
                        end
                    end
                end
            end

            // Drain one complete sampled row per cycle.  This write port is
            // independent of the feature engine's read bank.
            if (!W_row_overrun_pix && (!tile_write_en || W_tile_ready_pix) && commit_active && pending_valid[commit_slot]) begin
                tile_write_en <= 1'b1;
                tile_write_addr <= (commit_bank ? 9'd224 : 9'd0) +
                                   ({6'd0, commit_slot} << 5) +
                                   {4'd0, pending_y_start[commit_slot]} +
                                   {4'd0, commit_rep};
                tile_write_data <= pending_data[commit_slot];
                if (commit_rep + 1 >= pending_y_count[commit_slot]) begin
                    pending_valid[commit_slot] <= 1'b0;
                    commit_rep <= 5'd0;
                    if (commit_slot == 3'd6) begin
                        commit_active <= 1'b0;
                    end else begin
                        commit_slot <= commit_slot + 1'b1;
                    end
                end else begin
                    commit_rep <= commit_rep + 1'b1;
                end
            end else if (!W_row_overrun_pix && (!tile_write_en || W_tile_ready_pix) && commit_active) begin
                commit_rep <= 5'd0;
                if (commit_slot == 3'd6)
                    commit_active <= 1'b0;
                else
                    commit_slot <= commit_slot + 1'b1;
            end

            // Publication is delayed until all pending rows from the final
            // source line have drained.  This makes the bank/frame event
            // unambiguous to the byte-packet adapter.
            if (snapshot_pending && W_pipeline_empty_pix && W_history_empty_pix &&
                !S_finalize_pending_pix && !tile_write_en &&
                !commit_active && (pending_valid == 7'd0)) begin
                O_snapshot_valid <= 1'b1;
                O_snapshot_bank <= snapshot_pending_bank;
                O_snapshot_mask <= snapshot_mask_q;
                O_snapshot_frame_index <= snapshot_frame_index_q;
                snapshot_pending <= 1'b0;
            end
            end
        end
    end

endmodule

