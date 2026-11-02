//=============================================================================
//  cap_master_tile_capture  --  C32-compatible streamed character tile capture
//-----------------------------------------------------------------------------
//  所属域   : cap
//  时钟域   : I_clk_pix (HDMI/application)
//  复位     : I_rst_pix_n
//  参数     : H_ACTIVE, V_ACTIVE
//  验证状态 : Directed/equivalence capture fixture PASS; full board flow 未做
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
    parameter integer V_ACTIVE = 720
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

    output wire        O_tile_write_en,
    output wire [8:0]  O_tile_write_addr,
    output wire [31:0] O_tile_write_data,
    output reg         O_snapshot_valid,
    output reg         O_snapshot_drop,
    output reg         O_snapshot_bank,
    output reg  [6:0]  O_snapshot_mask,
    output reg  [31:0] O_snapshot_frame_index,
    output wire        O_capture_active,
    output wire        O_commit_active
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
            assign W_crop_y_qr[G_geometry_slot] = div32_qr_lookup(W_crop_height[G_geometry_slot]);
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
    reg  [2:0]  W_source_slot_index;
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
    reg [6:0]   W_selected_destination;
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

        // The selection is already one-hot.  Read the state directly with
        // parallel masks instead of encoding an index then decoding a mux.
        W_selected_left = 11'd0;
        W_selected_width = 12'd0;
        W_selected_quotient = 7'd0;
        W_selected_remainder = 12'd0;
        W_selected_error = 12'd0;
        W_selected_destination = 7'd0;
        for (slot_scan = 0; slot_scan < 7; slot_scan = slot_scan + 1) begin
            W_selected_left = W_selected_left |
                ({11{capture_select[slot_scan]}} & capture_left[slot_scan]);
            W_selected_width = W_selected_width |
                ({12{capture_select[slot_scan]}} & capture_width[slot_scan]);
            W_selected_quotient = W_selected_quotient |
                ({7{capture_select[slot_scan]}} & capture_x_q[slot_scan][6:0]);
            W_selected_remainder = W_selected_remainder |
                ({12{capture_select[slot_scan]}} & capture_x_r[slot_scan]);
            W_selected_error = W_selected_error |
                ({12{capture_select[slot_scan]}} & x_err[slot_scan]);
            W_selected_destination = W_selected_destination |
                ({7{capture_select[slot_scan]}} & x_dst[slot_scan]);
        end

        // Only the selected crop can receive the current pixel.  The former
        // implementation calculated seven independent Bresenham lanes and
        // discarded six of them on overlap.  Select first, then calculate the
        // one lane that is observable at this pixel; the per-slot state is
        // still retained for the next pixel.
        W_capture_slot_index = 3'd0;
        if (capture_select[1]) W_capture_slot_index = 3'd1;
        if (capture_select[2]) W_capture_slot_index = 3'd2;
        if (capture_select[3]) W_capture_slot_index = 3'd3;
        if (capture_select[4]) W_capture_slot_index = 3'd4;
        if (capture_select[5]) W_capture_slot_index = 3'd5;
        if (capture_select[6]) W_capture_slot_index = 3'd6;
        W_source_slot_index = 3'd0;
        if (source_select[1]) W_source_slot_index = 3'd1;
        if (source_select[2]) W_source_slot_index = 3'd2;
        if (source_select[3]) W_source_slot_index = 3'd3;
        if (source_select[4]) W_source_slot_index = 3'd4;
        if (source_select[5]) W_source_slot_index = 3'd5;
        if (source_select[6]) W_source_slot_index = 3'd6;

        shared_mask_valid = 1'b0;
        shared_mask_start = 7'd0;
        shared_mask_count = 7'd0;
        W_selected_x_start = 7'd0;
        W_selected_x_count = 7'd0;
        W_selected_x_error_base = 12'd0;
        W_selected_x_error_sum = 12'd0;
        W_selected_x_error_next = 12'd0;
        if (|capture_select) begin
            shared_mask_valid = 1'b1;
            if (pixel_x == W_selected_left) begin
                W_selected_x_start = 7'd0;
                // At the first source pixel, error starts at width - 1.
                // As in the vertical lane, a nonzero remainder emits one
                // extra destination pixel and leaves remainder - 1.
                if (W_selected_remainder != 12'd0) begin
                    W_selected_x_count = W_selected_quotient + 7'd1;
                    W_selected_x_error_next = W_selected_remainder - 12'd1;
                end else begin
                    W_selected_x_count = W_selected_quotient;
                    W_selected_x_error_next = W_selected_width - 12'd1;
                end
            end else begin
                W_selected_x_start = W_selected_destination;
                W_selected_x_error_base = W_selected_error;
                W_selected_x_error_sum = W_selected_x_error_base +
                                       W_selected_remainder;
                if (W_selected_x_error_sum >= W_selected_width) begin
                    W_selected_x_count = W_selected_quotient + 7'd1;
                    W_selected_x_error_next = W_selected_x_error_sum -
                                            W_selected_width;
                end else begin
                    W_selected_x_count = W_selected_quotient;
                    W_selected_x_error_next = W_selected_x_error_sum;
                end
            end
            shared_mask_start = W_selected_x_start;
            shared_mask_count = W_selected_x_count;
        end
        if (shared_mask_valid)
            shared_x_mask = repeat_mask(shared_mask_start, shared_mask_count);
        else
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

    always @(posedge I_clk_pix or negedge I_rst_pix_n) begin
        if (!I_rst_pix_n) begin
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
            tile_write_en <= 1'b0;
            O_snapshot_valid <= 1'b0;
            O_snapshot_drop <= 1'b0;

            if (vs_fall) begin
                S_y_update_active <= 1'b0;
                S_y_update_slot <= 3'd0;
                pixel_x <= 11'd0;
                pixel_y <= 11'd0;
                // Frame index zero is the first source frame whose geometry
                // is frozen here.  The corresponding processing metadata is
                // published later at the matching vs_rise/frame_start.
                active_frame_index <= capture_frame_index;
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
                        capture_valid[capture_index] <= 1'b0;
                    end
                end
                box_top <= I_char_up;
                box_bottom <= I_char_down;
                frame_mask <= I_detected ? I_char_valid : 7'd0;
            end else if (capture_active && I_ink_de) begin
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
                        row_acc[capture_index] <= 32'd0;
                        x_err[capture_index] <= 12'd0;
                        x_dst[capture_index] <= 7'd0;
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

                for (capture_index = 0; capture_index < 7; capture_index = capture_index + 1) begin
                    if (capture_select[capture_index]) begin
                        x_err[capture_index] <= W_selected_x_error_next;
                        x_dst[capture_index] <= W_selected_x_start + W_selected_x_count;
                        if (I_ink) begin
                            if (pixel_x == W_selected_left)
                                row_acc[capture_index] <= shared_x_mask;
                            else
                                row_acc[capture_index] <= row_acc[capture_index] | shared_x_mask;
                        end
                    end
                end

                if (|source_select && I_ink) begin
                    if (!current_ink_valid[W_source_slot_index]) begin
                        current_ink_valid[W_source_slot_index] <= 1'b1;
                        current_ink_count[W_source_slot_index] <= 3'd1;
                        current_ink_min_x[W_source_slot_index] <=
                            pixel_x - box_left[W_source_slot_index];
                        current_ink_max_x[W_source_slot_index] <=
                            pixel_x - box_left[W_source_slot_index];
                        current_ink_min_y[W_source_slot_index] <=
                            pixel_y - box_top;
                        current_ink_max_y[W_source_slot_index] <=
                            pixel_y - box_top;
                    end else begin
                        if (current_ink_count[W_source_slot_index] < 4)
                            current_ink_count[W_source_slot_index] <=
                                current_ink_count[W_source_slot_index] + 1'b1;
                        if (pixel_x - box_left[W_source_slot_index] <
                            current_ink_min_x[W_source_slot_index])
                            current_ink_min_x[W_source_slot_index] <=
                                pixel_x - box_left[W_source_slot_index];
                        if (pixel_x - box_left[W_source_slot_index] >
                            current_ink_max_x[W_source_slot_index])
                            current_ink_max_x[W_source_slot_index] <=
                                pixel_x - box_left[W_source_slot_index];
                        if (pixel_y - box_top <
                            current_ink_min_y[W_source_slot_index])
                            current_ink_min_y[W_source_slot_index] <=
                                pixel_y - box_top;
                        if (pixel_y - box_top >
                            current_ink_max_y[W_source_slot_index])
                            current_ink_max_y[W_source_slot_index] <=
                                pixel_y - box_top;
                    end
                end
                if (pixel_x == H_LAST) begin
                    if (capture_valid[0] && row_rep_count[0] != 0) begin
                        pending_valid[0] <= 1'b1;
                        pending_data[0] <= row_acc[0] |
                            ((capture_select[0] && I_ink) ? shared_x_mask : 32'd0);
                        pending_y_start[0] <= row_rep_start[0][4:0];
                        pending_y_count[0] <= row_rep_count[0];
                    end
                    if (capture_valid[1] && row_rep_count[1] != 0) begin
                        pending_valid[1] <= 1'b1;
                        pending_data[1] <= row_acc[1] |
                            ((capture_select[1] && I_ink) ? shared_x_mask : 32'd0);
                        pending_y_start[1] <= row_rep_start[1][4:0];
                        pending_y_count[1] <= row_rep_count[1];
                    end
                    if (capture_valid[2] && row_rep_count[2] != 0) begin
                        pending_valid[2] <= 1'b1;
                        pending_data[2] <= row_acc[2] |
                            ((capture_select[2] && I_ink) ? shared_x_mask : 32'd0);
                        pending_y_start[2] <= row_rep_start[2][4:0];
                        pending_y_count[2] <= row_rep_count[2];
                    end
                    if (capture_valid[3] && row_rep_count[3] != 0) begin
                        pending_valid[3] <= 1'b1;
                        pending_data[3] <= row_acc[3] |
                            ((capture_select[3] && I_ink) ? shared_x_mask : 32'd0);
                        pending_y_start[3] <= row_rep_start[3][4:0];
                        pending_y_count[3] <= row_rep_count[3];
                    end
                    if (capture_valid[4] && row_rep_count[4] != 0) begin
                        pending_valid[4] <= 1'b1;
                        pending_data[4] <= row_acc[4] |
                            ((capture_select[4] && I_ink) ? shared_x_mask : 32'd0);
                        pending_y_start[4] <= row_rep_start[4][4:0];
                        pending_y_count[4] <= row_rep_count[4];
                    end
                    if (capture_valid[5] && row_rep_count[5] != 0) begin
                        pending_valid[5] <= 1'b1;
                        pending_data[5] <= row_acc[5] |
                            ((capture_select[5] && I_ink) ? shared_x_mask : 32'd0);
                        pending_y_start[5] <= row_rep_start[5][4:0];
                        pending_y_count[5] <= row_rep_count[5];
                    end
                    if (capture_valid[6] && row_rep_count[6] != 0) begin
                        pending_valid[6] <= 1'b1;
                        pending_data[6] <= row_acc[6] |
                            ((capture_select[6] && I_ink) ? shared_x_mask : 32'd0);
                        pending_y_start[6] <= row_rep_start[6][4:0];
                        pending_y_count[6] <= row_rep_count[6];
                    end
                    // Pending rows belong to the bank being captured in this
                    // frame.  Latch that bank at every line boundary so a
                    // later frame can capture into the other bank while the
                    // previous line/frame's rows finish draining.
                    commit_bank <= capture_bank;
                    commit_active <= 1'b1;
                    commit_slot <= 3'd0;
                    commit_rep <= 5'd0;
                end

                if (pixel_x == H_LAST) begin
                    pixel_x <= 11'd0;
                    if (pixel_y < V_LAST)
                        pixel_y <= pixel_y + 1'b1;
                end else begin
                    pixel_x <= pixel_x + 1'b1;
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
                if (capture_seen) begin
                    snapshot_pending_bank <= capture_bank;
                    snapshot_mask_q <= process_mask_build;
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
            if (commit_active && pending_valid[commit_slot]) begin
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
            end else if (commit_active) begin
                commit_rep <= 5'd0;
                if (commit_slot == 3'd6)
                    commit_active <= 1'b0;
                else
                    commit_slot <= commit_slot + 1'b1;
            end

            // Publication is delayed until all pending rows from the final
            // source line have drained.  This makes the bank/frame event
            // unambiguous to the byte-packet adapter.
            if (snapshot_pending && !commit_active && (pending_valid == 7'd0)) begin
                O_snapshot_valid <= 1'b1;
                O_snapshot_bank <= snapshot_pending_bank;
                O_snapshot_mask <= snapshot_mask_q;
                O_snapshot_frame_index <= snapshot_frame_index_q;
                snapshot_pending <= 1'b0;
            end
        end
    end

endmodule

