`timescale 1ns / 1ps
// ============================================================================
// char_locate: 冻结车牌 ROI，按连续占用带分割七字符。
// Character segmentation with a frame-frozen ROI and inclusive character bounds.
// 全幅 R 阈值墨点只延迟一拍；投影 ROI 不裁剪识别输入，不再腐蚀细笔画。
// 固定 R 门限仍需真实曝光样本校准。搜索范围参数额外约束内部投影。
// ============================================================================
module char_locate #(
    parameter H_ACTIVE = 1280,
    parameter V_ACTIVE = 720,
    parameter BIN_THRESHOLD = 8'd160,
    parameter ADAPTIVE_THRESHOLD = 1,
    // The production row gate uses the ROI-width >> 6 base with current-frame
    // peak adaptation. A caller can set ADAPTIVE_ROW_GATE=0 for the
    // geometry-only gate and can select another base shift for comparison.
    parameter ADAPTIVE_ROW_GATE = 1,
    parameter ROW_THRESHOLD_SHIFT = 6,
    parameter MIN_CONTRAST = 8'd24,
    parameter THRESHOLD_MIN = 8'd32,
    parameter THRESHOLD_MAX = 8'd224,
    parameter BORDER_SHRINK = 11'd20, // 比例内缩的上限。
    parameter HORIZON_START = 11'd10,
    parameter HORIZON_END = 11'd1270,
    parameter VERTICAL_START = 10'd10,
    parameter VERTICAL_END = 10'd710
)(
    input wire clk,
    input wire rst_n,
    input wire [23:0] i_rgb,
    input wire i_de,
    input wire i_vsync,
    input wire [10:0] i_up,
    input wire [10:0] i_down,
    input wire [10:0] i_left,
    input wire [10:0] i_right,
    input wire i_detected,
    output reg [10:0] o_char_up,
    output reg [10:0] o_char_down,
    output reg [10:0] o_c1_left,
    output reg [10:0] o_c1_right,
    output reg [10:0] o_c2_left,
    output reg [10:0] o_c2_right,
    output reg [10:0] o_c3_left,
    output reg [10:0] o_c3_right,
    output reg [10:0] o_c4_left,
    output reg [10:0] o_c4_right,
    output reg [10:0] o_c5_left,
    output reg [10:0] o_c5_right,
    output reg [10:0] o_c6_left,
    output reg [10:0] o_c6_right,
    output reg [10:0] o_c7_left,
    output reg [10:0] o_c7_right,
    output reg o_char_detected,
    output reg [6:0] o_char_valid,
    output wire o_char_ink,
    output wire o_char_ink_de
);
    reg vsync_prev;
    wire vs_rise = i_vsync && !vsync_prev;
    wire vs_fall = !i_vsync && vsync_prev;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) vsync_prev <= 1'b0;
        else vsync_prev <= i_vsync;
    end

    // 先验证输入框，比例内缩在合法框内执行，不允许无符号下溢。
    wire [10:0] plate_width = i_right - i_left + 11'd1;
    wire [10:0] plate_height = i_down - i_up + 11'd1;
    wire [10:0] margin_x_ratio = plate_width >> 5;
    wire [10:0] margin_y_ratio = plate_height >> 4;
    wire [10:0] margin_x = (margin_x_ratio > BORDER_SHRINK) ? BORDER_SHRINK : margin_x_ratio;
    wire [10:0] margin_y = (margin_y_ratio > BORDER_SHRINK) ? BORDER_SHRINK : margin_y_ratio;
    wire plate_order_ok = (i_left < i_right) && (i_up < i_down);
    wire plate_range_ok = (i_right < H_ACTIVE) && (i_down < V_ACTIVE);
    reg roi_valid;
    reg [10:0] roi_left, roi_right, roi_up, roi_down;
    wire [10:0] roi_width = roi_right - roi_left + 11'd1;
    wire [10:0] roi_height = roi_down - roi_up + 11'd1;
    wire [10:0] central_row_quarter = roi_height >> 2;
    wire [10:0] central_row_up = roi_up + central_row_quarter;
    wire [10:0] central_row_down = roi_down - central_row_quarter;
    wire central_row_range_ok = (central_row_up <= central_row_down);
    wire [10:0] row_threshold_ratio = roi_width >> ROW_THRESHOLD_SHIFT;
    wire [10:0] row_threshold_base = (row_threshold_ratio < 11'd2) ?
                                      11'd2 : row_threshold_ratio;
    reg [10:0] current_row_peak;
    wire [11:0] row_peak_ceil_wide = {1'b0,current_row_peak} + 12'd7;
    wire [10:0] row_peak_threshold = row_peak_ceil_wide[11:3];
    wire [10:0] row_threshold = (ADAPTIVE_ROW_GATE && central_row_range_ok &&
                                 (current_row_peak != 11'd0) &&
                                 (row_peak_threshold > row_threshold_base)) ?
                                row_peak_threshold : row_threshold_base;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roi_valid <= 1'b0;
            roi_left <= 11'd0; roi_right <= 11'd0; roi_up <= 11'd0; roi_down <= 11'd0;
        end else if (vs_fall) begin
            roi_valid <= i_detected && plate_order_ok && plate_range_ok &&
                         (plate_width >= 11'd20) && (plate_height >= 11'd8);
            if (i_detected && plate_order_ok && plate_range_ok) begin
                roi_left <= i_left + margin_x; roi_right <= i_right - margin_x;
                roi_up <= i_up + margin_y; roi_down <= i_down - margin_y;
            end else begin
                roi_left <= 11'd0; roi_right <= 11'd0; roi_up <= 11'd0; roi_down <= 11'd0;
            end
        end
    end

    // 全幅墨点与坐标同拍；不按 detected 或 ROI 门控导出的墨点。
    reg [10:0] pixel_x, ink_x;
    reg [9:0] pixel_y, ink_y;
    reg ink_bit, ink_de;
    reg [7:0] frame_threshold;
    reg [7:0] roi_red_min, roi_red_max;
    reg [19:0] roi_samples;
    wire sample_in_roi = roi_valid && i_de &&
        pixel_x >= roi_left && pixel_x <= roi_right &&
        pixel_y >= roi_up && pixel_y <= roi_down;
    wire [7:0] roi_contrast = roi_red_max - roi_red_min;
    wire [8:0] roi_red_sum = {1'b0,roi_red_min} + {1'b0,roi_red_max};
    wire [7:0] midpoint_threshold = roi_red_sum[8:1];
    wire [7:0] limited_threshold = (midpoint_threshold < THRESHOLD_MIN) ? THRESHOLD_MIN :
        ((midpoint_threshold > THRESHOLD_MAX) ? THRESHOLD_MAX : midpoint_threshold);
    // Collect raw RGB independently of the current ink decision. A dark frame
    // can therefore recover even when its previous threshold found no glyphs.
    // The next frame uses one frozen threshold; min/max needs sample calibration.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_threshold <= BIN_THRESHOLD;
            roi_red_min <= 8'd255; roi_red_max <= 8'd0; roi_samples <= 20'd0;
        end else if (vs_fall) begin
            if (ADAPTIVE_THRESHOLD && i_detected && roi_valid &&
                roi_samples >= 32 && roi_contrast >= MIN_CONTRAST)
                frame_threshold <= limited_threshold;
            else frame_threshold <= BIN_THRESHOLD;
            roi_red_min <= 8'd255; roi_red_max <= 8'd0; roi_samples <= 20'd0;
        end else if (sample_in_roi) begin
            if (i_rgb[23:16] < roi_red_min) roi_red_min <= i_rgb[23:16];
            if (i_rgb[23:16] > roi_red_max) roi_red_max <= i_rgb[23:16];
            roi_samples <= roi_samples + 1'b1;
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_x <= 11'd0; pixel_y <= 10'd0; ink_x <= 11'd0; ink_y <= 10'd0;
            ink_bit <= 1'b0; ink_de <= 1'b0;
        end else begin
            ink_bit <= i_rgb[23:16] > frame_threshold;
            ink_de <= i_de; ink_x <= pixel_x; ink_y <= pixel_y;
            if (vs_fall) begin pixel_x <= 11'd0; pixel_y <= 10'd0; end
            else if (i_de) begin
                if (pixel_x == H_ACTIVE-1) begin pixel_x <= 11'd0; pixel_y <= pixel_y + 1'b1; end
                else pixel_x <= pixel_x + 1'b1;
            end
        end
    end
    assign o_char_ink = ink_bit;
    assign o_char_ink_de = ink_de;
    wire projection_x_ok = (ink_x >= roi_left) && (ink_x <= roi_right) &&
                           (ink_x >= HORIZON_START) && (ink_x <= HORIZON_END);
    wire projection_y_ok = (ink_y >= roi_up) && (ink_y <= roi_down) &&
                           (ink_y >= VERTICAL_START) && (ink_y <= VERTICAL_END);
    wire projected_ink = roi_valid && projection_x_ok && projection_y_ok && ink_bit;
    // 列投影首帧使用完整 ROI；同一目标的微小框抖动可平移复用上一帧主行带。
    // 目标框显著变化时 band_active 在新帧开始即失效，避免沿用旧目标上下带。
    reg band_valid;
    reg band_active;
    reg [10:0] band_up, band_down;
    reg [10:0] band_roi_left, band_roi_right, band_roi_up, band_roi_down;
    wire incoming_roi_ok = i_detected && plate_order_ok && plate_range_ok &&
                            (plate_width >= 11'd20) && (plate_height >= 11'd8);
    wire [10:0] incoming_roi_left = i_left + margin_x;
    wire [10:0] incoming_roi_right = i_right - margin_x;
    wire [10:0] incoming_roi_up = i_up + margin_y;
    wire [10:0] incoming_roi_down = i_down - margin_y;
    wire [10:0] band_left_delta = (incoming_roi_left >= band_roi_left) ?
                                   (incoming_roi_left - band_roi_left) :
                                   (band_roi_left - incoming_roi_left);
    wire [10:0] band_right_delta = (incoming_roi_right >= band_roi_right) ?
                                    (incoming_roi_right - band_roi_right) :
                                    (band_roi_right - incoming_roi_right);
    wire [10:0] band_up_delta = (incoming_roi_up >= band_roi_up) ?
                                 (incoming_roi_up - band_roi_up) :
                                 (band_roi_up - incoming_roi_up);
    wire [10:0] band_down_delta = (incoming_roi_down >= band_roi_down) ?
                                   (incoming_roi_down - band_roi_down) :
                                   (band_roi_down - incoming_roi_down);
    wire band_y_shift_positive = incoming_roi_up >= band_roi_up;
    wire [11:0] band_up_shifted_wide = band_y_shift_positive ?
        ({1'b0,band_up} + {1'b0,band_up_delta}) :
        ({1'b0,band_up} - {1'b0,band_up_delta});
    wire [11:0] band_down_shifted_wide = band_y_shift_positive ?
        ({1'b0,band_down} + {1'b0,band_up_delta}) :
        ({1'b0,band_down} - {1'b0,band_up_delta});
    wire [10:0] band_up_shifted = band_up_shifted_wide[10:0];
    wire [10:0] band_down_shifted = band_down_shifted_wide[10:0];
    // A few pixels of detector jitter can reuse the old band after translating
    // it vertically.  Independent boundary deltas and an in-ROI check keep a
    // real target switch from inheriting stale geometry.
    wire band_reuse_ok = incoming_roi_ok && band_valid &&
                         (band_left_delta <= 11'd2) &&
                         (band_right_delta <= 11'd2) &&
                         (band_up_delta <= 11'd2) &&
                         (band_down_delta <= 11'd2) &&
                         (band_up_shifted <= band_down_shifted) &&
                         (band_up_shifted >= incoming_roi_up) &&
                         (band_down_shifted <= incoming_roi_down);
    wire column_projected_ink = projected_ink &&
                                (!band_active ||
                                 ((ink_y >= band_up) && (ink_y <= band_down)));

    localparam [4:0] S_IDLE = 5'd0, S_H_READ = 5'd1, S_H_CHECK = 5'd2,
                     S_H_FLUSH = 5'd3, S_V_READ = 5'd4, S_V_CHECK = 5'd5,
                     S_V_FLUSH = 5'd6, S_MERGE_INIT = 5'd7, S_MERGE_EVAL = 5'd8,
                     S_MERGE_NEXT = 5'd9, S_SELECT = 5'd10, S_COMMIT = 5'd11,
                     S_EXP_INIT = 5'd12, S_EXP_ELIG = 5'd13,
                     S_EXP_ADDR = 5'd14, S_EXP_EVAL = 5'd15,
                     S_EXP_NEXT = 5'd16,
                     // Candidate10 serializes the merge score.  The old
                     // candidate9 path evaluated seven slot centers,
                     // variable-width arithmetic, and the global minimum in
                     // one combinational cone; these states make each part a
                     // registered one-pair/one-slot operation.
                     S_MERGE_PREP = 5'd17, S_MERGE_CHECK = 5'd18,
                     S_MERGE_SLOT_INIT = 5'd19, S_MERGE_SLOT_CALC = 5'd20,
                     S_MERGE_SLOT_NEXT = 5'd21, S_MERGE_WIDTH = 5'd22,
                     S_MERGE_SCORE = 5'd23;
    reg [4:0] state, next_state;
    reg [10:0] scan_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else if (vs_fall) state <= S_IDLE;
        else state <= next_state;
    end

    // 每帧所有行写满、所有列由第一行清零；RAM 不作大规模复位。
    // TD 6.2 inference attributes: force a block RAM, not distributed LUT RAM.
    (*force_ram*)
    (*ram_style = "bram"*)
    reg [10:0] h_proj [0:V_ACTIVE-1];
    reg [10:0] h_read, row_count, h_write_data;
    reg h_write;
    reg [9:0] h_write_addr;
    wire [10:0] h_row_value = row_count + {10'd0,projected_ink};
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            row_count <= 11'd0; h_write <= 1'b0; h_write_addr <= 10'd0;
            h_write_data <= 11'd0;
            current_row_peak <= 11'd0;
        end else if (vs_fall) begin
            // The peak belongs only to the next active frame.  In particular,
            // a blank/invalid frame never inherits an old row gate.
            current_row_peak <= 11'd0;
        end else begin
            h_write <= 1'b0;
            if (ink_de) begin
                if (ink_x == 0) row_count <= {10'd0, projected_ink};
                else row_count <= row_count + projected_ink;
                if (ink_x == H_ACTIVE-1) begin
                    h_write <= 1'b1; h_write_addr <= ink_y;
                    h_write_data <= h_row_value;
                    // Use the just-completed current-frame row value rather
                    // than the registered h_write_data from the prior row.
                    // The central half excludes the outer quarter of the
                    // frozen ROI, where screws/rails commonly sit.
                    if (central_row_range_ok &&
                        (ink_y >= central_row_up) && (ink_y <= central_row_down) &&
                        (h_row_value > current_row_peak))
                        current_row_peak <= h_row_value;
                end
            end
        end
    end

    // One unconditional synchronous read and one independent write.  The
    // address mux keeps a single array reference for TD inference while the
    // idle address is fixed inside the RAM bounds.  S_H_READ is followed by
    // the check state, so h_read retains the original one-cycle latency.
    wire [10:0] h_read_address = (state == S_H_READ) ? scan_index : 11'd0;
    wire [9:0] h_write_address = h_write_addr;
    wire [10:0] h_write_value = h_write_data;
    always @(posedge clk)
        h_read <= h_proj[h_read_address];
    always @(posedge clk)
        if (h_write)
            h_proj[h_write_address] <= h_write_value;

    // The vertical projection is read during scan/expansion and accumulated
    // during active video.  Keep the same TD 6.2 block-RAM inference pattern.
    (*force_ram*)
    (*ram_style = "bram"*)
    reg [9:0] v_proj [0:H_ACTIVE-1];
    reg [9:0] v_read;
    wire [9:0] v_scan_read = v_read;
    reg [10:0] selected_left [0:6];
    reg [10:0] selected_right [0:6];
    reg selected_valid;
    reg selected_layout_guard_active;
    wire [10:0] selected_target_width = roi_width >> 3;
    wire [11:0] selected_max_group_width =
        ((((selected_target_width << 1) + selected_target_width) >> 1) < 11'd8) ?
        12'd8 : {1'b0,(((selected_target_width << 1) + selected_target_width) >> 1)};
    wire [11:0] selected_max_gap =
        ((selected_target_width << 1) + 11'd8 < 11'd16) ?
        12'd16 : {1'b0,((selected_target_width << 1) + 11'd8)};
    wire [10:0] selected_width0 = selected_right[0] - selected_left[0] + 11'd1;
    wire [10:0] selected_width1 = selected_right[1] - selected_left[1] + 11'd1;
    wire [10:0] selected_width2 = selected_right[2] - selected_left[2] + 11'd1;
    wire [10:0] selected_width3 = selected_right[3] - selected_left[3] + 11'd1;
    wire [10:0] selected_width4 = selected_right[4] - selected_left[4] + 11'd1;
    wire [10:0] selected_width5 = selected_right[5] - selected_left[5] + 11'd1;
    wire [10:0] selected_width6 = selected_right[6] - selected_left[6] + 11'd1;
    wire selected_ordered = (selected_right[0] >= selected_left[0]) &&
                            (selected_right[1] >= selected_left[1]) &&
                            (selected_right[2] >= selected_left[2]) &&
                            (selected_right[3] >= selected_left[3]) &&
                            (selected_right[4] >= selected_left[4]) &&
                            (selected_right[5] >= selected_left[5]) &&
                            (selected_right[6] >= selected_left[6]);
    wire selected_layout_ok = selected_ordered &&
        (selected_width0 <= selected_max_group_width) &&
        (selected_width1 <= selected_max_group_width) &&
        (selected_width2 <= selected_max_group_width) &&
        (selected_width3 <= selected_max_group_width) &&
        (selected_width4 <= selected_max_group_width) &&
        (selected_width5 <= selected_max_group_width) &&
        (selected_width6 <= selected_max_group_width) &&
        ((selected_left[1] > selected_right[0]) &&
         ((selected_left[1] - selected_right[0] - 11'd1) <= selected_max_gap)) &&
        ((selected_left[2] > selected_right[1]) &&
         ((selected_left[2] - selected_right[1] - 11'd1) <= selected_max_gap)) &&
        ((selected_left[3] > selected_right[2]) &&
         ((selected_left[3] - selected_right[2] - 11'd1) <= selected_max_gap)) &&
        ((selected_left[4] > selected_right[3]) &&
         ((selected_left[4] - selected_right[3] - 11'd1) <= selected_max_gap)) &&
        ((selected_left[5] > selected_right[4]) &&
         ((selected_left[5] - selected_right[4] - 11'd1) <= selected_max_gap)) &&
        ((selected_left[6] > selected_right[5]) &&
         ((selected_left[6] - selected_right[5] - 11'd1) <= selected_max_gap));
    // A dense-core threshold can clip a narrow glyph's low-density stroke
    // even though the projection has a real (non-noise) run immediately
    // outside the selected core.  After the seven boxes are selected, inspect
    // one boundary column per clock through the same projection RAM.  Only a
    // box below half the nominal seven-slot width is eligible, each side is
    // capped at 32 columns, and a count <= 1 stops the walk.  This keeps the
    // weak-stroke recovery bounded and leaves normal-width 1/I boxes alone.
    reg [2:0] expand_cell;
    reg expand_side; // 0: left edge, 1: right edge
    reg [5:0] expand_steps;
    reg [6:0] expand_eligible;
    // Snapshot the seven selected cores before any side is moved.  The
    // snapshots make both eligibility and the neighbour midpoints invariant
    // while the serial expansion walk updates selected_left/right.
    reg [10:0] expand_core_left [0:6];
    reg [10:0] expand_core_right [0:6];
    reg [10:0] expand_left_limit_locked [0:6];
    reg [10:0] expand_right_limit_locked [0:6];
    wire [10:0] expand_target_width = roi_width >> 3;
    wire [10:0] expand_narrow_raw = expand_target_width >> 1;
    wire [10:0] expand_narrow_limit = (expand_narrow_raw < 11'd4) ?
                                       11'd4 : expand_narrow_raw;
    // Normal-width cells skip the serial walk entirely.  This preserves the
    // existing short commit path for ordinary plates; only a frame containing
    // at least one originally narrow core enters S_EXP_ELIG.
    wire expand_selected_any_narrow =
        ((selected_right[0] >= selected_left[0]) &&
         ((selected_right[0] - selected_left[0] + 11'd1) < expand_narrow_limit)) ||
        ((selected_right[1] >= selected_left[1]) &&
         ((selected_right[1] - selected_left[1] + 11'd1) < expand_narrow_limit)) ||
        ((selected_right[2] >= selected_left[2]) &&
         ((selected_right[2] - selected_left[2] + 11'd1) < expand_narrow_limit)) ||
        ((selected_right[3] >= selected_left[3]) &&
         ((selected_right[3] - selected_left[3] + 11'd1) < expand_narrow_limit)) ||
        ((selected_right[4] >= selected_left[4]) &&
         ((selected_right[4] - selected_left[4] + 11'd1) < expand_narrow_limit)) ||
        ((selected_right[5] >= selected_left[5]) &&
         ((selected_right[5] - selected_left[5] + 11'd1) < expand_narrow_limit)) ||
        ((selected_right[6] >= selected_left[6]) &&
         ((selected_right[6] - selected_left[6] + 11'd1) < expand_narrow_limit));
    wire expand_core_ordered = expand_core_right[expand_cell] >= expand_core_left[expand_cell];
    wire [10:0] expand_core_width = expand_core_ordered ?
        (expand_core_right[expand_cell] - expand_core_left[expand_cell] + 11'd1) : 11'd0;
    wire [10:0] expand_left_probe = (selected_left[expand_cell] > 0) ?
                                     (selected_left[expand_cell] - 11'd1) :
                                     selected_left[expand_cell];
    wire [10:0] expand_right_probe = (selected_right[expand_cell] < H_ACTIVE-1) ?
                                      (selected_right[expand_cell] + 11'd1) :
                                      selected_right[expand_cell];
    wire [2:0] expand_prev_cell = (expand_cell == 3'd0) ? 3'd0 :
                                   (expand_cell - 3'd1);
    wire [2:0] expand_next_cell = (expand_cell == 3'd6) ? 3'd6 :
                                   (expand_cell + 3'd1);
    wire [11:0] expand_core_left_mid_sum =
        {1'b0,expand_core_right[expand_prev_cell]} +
        {1'b0,expand_core_left[expand_cell]};
    wire [11:0] expand_core_right_mid_sum =
        {1'b0,expand_core_right[expand_cell]} +
        {1'b0,expand_core_left[expand_next_cell]};
    wire [10:0] expand_core_left_limit = (expand_cell == 3'd0) ? roi_left :
                                          expand_core_left_mid_sum[11:1];
    wire [10:0] expand_core_right_limit = (expand_cell == 3'd6) ? roi_right :
                                           expand_core_right_mid_sum[11:1];
    wire [10:0] expand_left_limit = expand_left_limit_locked[expand_cell];
    wire [10:0] expand_right_limit = expand_right_limit_locked[expand_cell];
    wire [10:0] expand_probe_addr = expand_side ? expand_right_probe : expand_left_probe;
    wire expand_probe_in_roi = expand_side ?
                                ((selected_right[expand_cell] < roi_right) &&
                                 (expand_right_probe <= roi_right) &&
                                 (expand_right_probe <= expand_right_limit)) :
                                ((selected_left[expand_cell] > roi_left) &&
                                 (expand_left_probe >= roi_left) &&
                                 (expand_left_probe >= expand_left_limit));
    // Eligibility is latched from the original seven core widths before any
    // side is expanded.  Otherwise a left walk can make the width reach the
    // narrow limit and incorrectly disable the right walk in the same cell.
    wire expand_step_ok = selected_valid && expand_eligible[expand_cell] &&
                          (expand_steps < 6'd32) && expand_probe_in_roi &&
                          (v_scan_read > 10'd1);
    reg v_write, v_write_bit;
    reg [10:0] v_write_x;
    reg [9:0] v_write_y;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            v_write <= 1'b0; v_write_x <= 11'd0; v_write_y <= 10'd0; v_write_bit <= 1'b0;
        end else begin
            v_write <= ink_de; v_write_x <= ink_x; v_write_y <= ink_y; v_write_bit <= column_projected_ink;
        end
    end

    // One unconditional synchronous read.  The mux presents exactly one
    // address for scan, expansion, or the pixel-stream read-modify-write;
    // idle cycles use address zero and add no read CE or reset to the RAM.
    wire [10:0] v_read_address = (state == S_V_READ) ? scan_index :
                                  (state == S_EXP_ADDR) ? expand_probe_addr :
                                  (ink_de ? ink_x : 11'd0);
    always @(posedge clk)
        v_read <= v_proj[v_read_address];

    // Preserve the existing registered write enable/address/value exactly;
    // v_write_bit already carries the band gate from the sampled pixel.
    wire v_write_enable = v_write;
    wire [10:0] v_write_address = v_write_x;
    wire [9:0] v_write_value = (v_write_y == 0) ?
                               {9'd0,v_write_bit} : (v_read + v_write_bit);
    always @(posedge clk)
        if (v_write_enable)
            v_proj[v_write_address] <= v_write_value;

    reg [10:0] run_up, run_height, best_up, best_down, best_height;
    // 保存上一帧主行带及其 ROI 身份；ROI 变化时只允许当前帧重新建立投影。
    // When a same-ROI frame reuses the old band, its vertical projection is
    // already restricted before the horizontal projection finishes.  Keep a
    // one-bit guard so a materially different current best band cannot
    // publish boxes made from the stale column band.  S_COMMIT still stores
    // the current best band, allowing the following frame to converge.
    reg band_mismatch;
    wire [10:0] finalized_best_up = (run_height > best_height) ? run_up : best_up;
    wire [10:0] finalized_best_down = (run_height > best_height) ? (V_ACTIVE - 1) : best_down;
    wire [10:0] finalized_best_height = (run_height > best_height) ? run_height : best_height;
    wire [10:0] finalized_band_up_delta = (finalized_best_up >= band_up) ?
                                           (finalized_best_up - band_up) :
                                           (band_up - finalized_best_up);
    wire [10:0] finalized_band_down_delta = (finalized_best_down >= band_down) ?
                                             (finalized_best_down - band_down) :
                                             (band_down - finalized_best_down);
    wire reused_band_matches_current = (finalized_best_height >= 11'd8) &&
                                       (finalized_band_up_delta <= 11'd2) &&
                                       (finalized_band_down_delta <= 11'd2);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) band_mismatch <= 1'b0;
        else if (vs_fall) band_mismatch <= 1'b0;
        else if ((state == S_H_FLUSH) && band_active && !reused_band_matches_current)
            band_mismatch <= 1'b1;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            band_valid <= 1'b0;
            band_active <= 1'b0;
            band_up <= 11'd0; band_down <= 11'd0;
            band_roi_left <= 11'd0; band_roi_right <= 11'd0;
            band_roi_up <= 11'd0; band_roi_down <= 11'd0;
        end else if (vs_fall) begin
            band_active <= band_reuse_ok;
            if (band_reuse_ok) begin
                band_up <= band_up_shifted;
                band_down <= band_down_shifted;
                band_roi_left <= incoming_roi_left; band_roi_right <= incoming_roi_right;
                band_roi_up <= incoming_roi_up; band_roi_down <= incoming_roi_down;
            end else begin
                band_valid <= 1'b0;
            end
        end else if (state == S_COMMIT) begin
            band_valid <= roi_valid && (best_height >= 11'd8);
            band_up <= best_up; band_down <= best_down;
            band_roi_left <= roi_left; band_roi_right <= roi_right;
            band_roi_up <= roi_up; band_roi_down <= roi_down;
        end
    end
    wire row_occupied = h_read >= row_threshold;
    wire [10:0] col_threshold_ratio = best_height >> 3;
    wire [10:0] col_threshold = (col_threshold_ratio < 11'd2) ? 11'd2 : col_threshold_ratio;
    wire col_occupied = {1'b0, v_scan_read} >= col_threshold;
    reg [10:0] segment_left, segment_width;
    reg [20:0] segment_area;
    // Candidate capacity is deliberately bounded. A thirteenth kept run is
    // an explicit overflow/reject condition; it is never silently truncated.
    reg [4:0] segment_count;
    reg [10:0] candidate_left [0:11];
    reg [10:0] candidate_right [0:11];
    reg [20:0] candidate_area [0:11];
    wire [10:0] dot_width = candidate_right[2] - candidate_left[2] + 11'd1;
    wire [10:0] dot_max_width = (best_height >> 2) + 11'd1;
    wire [22:0] dot_area_four = {candidate_area[2], 2'b00};
    wire dot_area_small = (dot_area_four < {2'b00, candidate_area[1]}) &&
                          (dot_area_four < {2'b00, candidate_area[3]});
    wire optional_dot = (segment_count == 5'd8) && (dot_width <= dot_max_width) && dot_area_small;
    // 内缩 ROI 边界的小面积连续带是 known ROI 的边框/亮边；接触边界且
    // 面积不超过两倍主行高才丢弃，避免把窄 1/I 等非边界字符删除。
    wire [20:0] edge_area_limit = {9'd0, best_height, 1'b0};
    wire [10:0] segment_right_at_close = (state == S_V_FLUSH) ? (H_ACTIVE - 1) :
                                         (scan_index - 11'd1);
    wire segment_touches_roi_edge = (segment_left == roi_left) ||
                                    (segment_right_at_close == roi_right);
    wire segment_edge_noise = segment_touches_roi_edge &&
                               (segment_area <= edge_area_limit);
    // Eight to twelve runs are merged one adjacent pair per blanking round.
    // Candidate10 keeps the candidate9 fixed-point score and tie order, but
    // computes it as a registered serial pipeline.  There is one pair in
    // flight, one slot-center comparison per cycle, then width/score/update.
    // A thirteenth run is retained as overflow and rejected.
    reg [3:0] merge_scan_index;
    reg merge_best_valid;
    reg [3:0] merge_best_index;
    reg [31:0] merge_best_score;
    reg [2:0] merge_round;
    wire [10:0] merge_target_width = roi_width >> 3;
    wire [11:0] merge_gap_limit =
        ((merge_target_width >> 1) > 11'd8) ?
        {1'b0,(merge_target_width >> 1)} : 12'd8;
    wire [11:0] merge_max_group_width =
        ((((merge_target_width << 1) + merge_target_width) >> 1) < 11'd8) ?
        12'd8 : {1'b0,(((merge_target_width << 1) + merge_target_width) >> 1)};

    // Pair fields are latched before any difference, shape, or score logic.
    // The case muxes are bounded to legal source entries (0..11), so the
    // old syntactic candidate[12] warning cannot recur.
    reg [10:0] merge_pair_left_reg, merge_pair_right_reg;
    reg [10:0] merge_pair_next_left_reg, merge_pair_next_right_reg;
    reg [11:0] merge_pair_gap_reg, merge_pair_width_reg;
    reg merge_pair_order_ok_reg, merge_pair_shape_ok_reg;
    reg [12:0] merge_pair_center_rel2_reg;
    reg [21:0] merge_pair_center_term_reg;
    reg [21:0] merge_w6_reg, merge_w7_reg, merge_w18_reg, merge_w26_reg;

    wire merge_pair_order_calc =
        merge_pair_next_left_reg > merge_pair_right_reg;
    wire [11:0] merge_pair_gap_calc = merge_pair_order_calc ?
        ({1'b0,merge_pair_next_left_reg} -
         {1'b0,merge_pair_right_reg} - 12'd1) : 12'hfff;
    wire [11:0] merge_pair_width_calc =
        {1'b0,merge_pair_next_right_reg} -
        {1'b0,merge_pair_left_reg} + 12'd1;
    wire [11:0] merge_pair_midpoint2_calc =
        {1'b0,merge_pair_left_reg} +
        {1'b0,merge_pair_next_right_reg};
    wire [12:0] merge_pair_center_rel2_calc =
        (merge_pair_midpoint2_calc >= {1'b0,roi_left,1'b0}) ?
        (merge_pair_midpoint2_calc - {1'b0,roi_left,1'b0}) : 13'd0;
    wire merge_pair_shape_calc =
        (segment_count >= 5'd8) && (segment_count <= 5'd12) && !optional_dot &&
        (merge_target_width >= 11'd4) && merge_pair_order_calc &&
        (merge_pair_right_reg >= merge_pair_left_reg) &&
        (merge_pair_next_right_reg >= merge_pair_next_left_reg) &&
        ((merge_pair_right_reg - merge_pair_left_reg + 11'd1) >= 11'd2) &&
        ((merge_pair_next_right_reg - merge_pair_next_left_reg + 11'd1) >= 11'd2) &&
        (merge_pair_gap_calc <= merge_gap_limit) &&
        (merge_pair_width_calc <= merge_max_group_width);

    function [21:0] abs_diff_22;
        input [21:0] a;
        input [21:0] b;
        begin
            abs_diff_22 = (a >= b) ? (a - b) : (b - a);
        end
    endfunction

    // Slot center targets are 6W, 24W, 50W, 68W, 86W, 104W, 122W.
    // They are generated from shift/add constants, avoiding seven inferred
    // multipliers in the critical merge path.
    reg [2:0] merge_slot_index;
    reg merge_slot_min_valid;
    reg [2:0] merge_pair_slot_reg;
    reg [21:0] merge_slot_center_target;
    reg [21:0] merge_pair_center_error;
    reg [11:0] merge_expected_width;
    reg [11:0] merge_width_error;
    reg [31:0] merge_pair_score;
    wire [21:0] merge_slot_error_calc =
        abs_diff_22(merge_pair_center_term_reg, merge_slot_center_target);
    wire [21:0] merge_expected_width_raw =
        ((merge_pair_slot_reg == 3'd0) || (merge_pair_slot_reg == 3'd6)) ?
        (merge_w6_reg >> 6) : (merge_w7_reg >> 6);
    wire [11:0] merge_expected_width_calc =
        (merge_expected_width_raw < 22'd2) ? 12'd2 :
        merge_expected_width_raw[11:0];
    wire [11:0] merge_width_error_calc =
        (merge_pair_width_reg >= {1'b0,merge_expected_width_calc}) ?
        (merge_pair_width_reg - {1'b0,merge_expected_width_calc}) :
        ({1'b0,merge_expected_width_calc} - merge_pair_width_reg);
    wire [31:0] merge_pair_score_calc =
        (merge_pair_center_error << 2) +
        (merge_width_error << 3) + merge_pair_gap_reg;
    wire flush_segment_kept = (segment_width >= 11'd2) && !segment_edge_noise;
    wire [4:0] flush_segment_count = flush_segment_kept ?
        ((segment_count < 5'd12) ? (segment_count + 1'b1) : 5'd13) : segment_count;

    always @(*) begin
        next_state = state;
        case (state)
            S_IDLE: if (vs_rise) next_state = S_H_READ;
            S_H_READ: next_state = S_H_CHECK;
            S_H_CHECK: next_state = (scan_index == V_ACTIVE-1) ? S_H_FLUSH : S_H_READ;
            S_H_FLUSH: next_state = S_V_READ;
            S_V_READ: next_state = S_V_CHECK;
            S_V_CHECK: next_state = (scan_index == H_ACTIVE-1) ? S_V_FLUSH : S_V_READ;
            S_V_FLUSH: next_state = ((flush_segment_count >= 5'd8) &&
                                     (flush_segment_count <= 5'd12) &&
                                     !optional_dot) ? S_MERGE_INIT : S_SELECT;
            S_MERGE_INIT: next_state = ((segment_count >= 5'd8) &&
                                        (segment_count <= 5'd12)) ?
                                       S_MERGE_EVAL : S_SELECT;
            S_MERGE_EVAL: next_state = S_MERGE_PREP;
            S_MERGE_PREP: next_state = S_MERGE_CHECK;
            S_MERGE_CHECK: next_state = merge_pair_shape_ok_reg ?
                                        S_MERGE_SLOT_CALC : S_MERGE_NEXT;
            S_MERGE_SLOT_CALC: next_state = S_MERGE_SLOT_NEXT;
            S_MERGE_SLOT_NEXT: next_state = (merge_slot_index < 3'd6) ?
                                            S_MERGE_SLOT_CALC : S_MERGE_WIDTH;
            S_MERGE_WIDTH: next_state = S_MERGE_SCORE;
            S_MERGE_SCORE: next_state = S_MERGE_NEXT;
            S_MERGE_NEXT: next_state =
                ((merge_scan_index + 4'd1) < (segment_count - 5'd1)) ?
                S_MERGE_EVAL : S_SELECT;
            S_SELECT: next_state = ((segment_count > 5'd8) &&
                                    merge_best_valid && (merge_round < 3'd5)) ?
                                   S_MERGE_INIT : S_EXP_INIT;
            S_EXP_INIT: next_state = selected_valid ?
                                     (expand_selected_any_narrow ? S_EXP_ELIG : S_COMMIT) :
                                     S_COMMIT;
            S_EXP_ELIG: next_state = (expand_cell == 3'd6) ? S_EXP_ADDR : S_EXP_ELIG;
            S_EXP_ADDR: next_state = S_EXP_EVAL;
            S_EXP_EVAL: next_state = expand_step_ok ? S_EXP_ADDR : S_EXP_NEXT;
            S_EXP_NEXT: next_state = expand_side ?
                                     ((expand_cell == 3'd6) ? S_COMMIT : S_EXP_ADDR) :
                                     S_EXP_ADDR;
            S_COMMIT: next_state = S_IDLE;
            default: next_state = S_IDLE;
        endcase
    end
    integer candidate_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_index <= 11'd0;
            run_up <= 11'd0; run_height <= 11'd0;
            best_up <= 11'd0; best_down <= 11'd0; best_height <= 11'd0;
            segment_left <= 11'd0; segment_width <= 11'd0; segment_area <= 21'd0;
            segment_count <= 5'd0; selected_valid <= 1'b0;
            selected_layout_guard_active <= 1'b0;
            merge_scan_index <= 4'd0;
            merge_best_valid <= 1'b0;
            merge_best_index <= 4'd0;
            merge_best_score <= 32'd0;
            merge_round <= 3'd0;
            merge_pair_left_reg <= 11'd0;
            merge_pair_right_reg <= 11'd0;
            merge_pair_next_left_reg <= 11'd0;
            merge_pair_next_right_reg <= 11'd0;
            merge_pair_gap_reg <= 12'd0;
            merge_pair_width_reg <= 12'd0;
            merge_pair_order_ok_reg <= 1'b0;
            merge_pair_shape_ok_reg <= 1'b0;
            merge_pair_center_rel2_reg <= 13'd0;
            merge_pair_center_term_reg <= 22'd0;
            merge_w6_reg <= 22'd0;
            merge_w7_reg <= 22'd0;
            merge_w18_reg <= 22'd0;
            merge_w26_reg <= 22'd0;
            merge_slot_index <= 3'd0;
            merge_slot_min_valid <= 1'b0;
            merge_pair_slot_reg <= 3'd0;
            merge_slot_center_target <= 22'd0;
            merge_pair_center_error <= 22'd0;
            merge_expected_width <= 12'd0;
            merge_width_error <= 12'd0;
            merge_pair_score <= 32'd0;
            expand_cell <= 3'd0;
            expand_side <= 1'b0;
            expand_steps <= 6'd0;
            expand_eligible <= 7'd0;
            for (candidate_index = 0; candidate_index < 12; candidate_index = candidate_index + 1) begin
                candidate_left[candidate_index] <= 11'd0;
                candidate_right[candidate_index] <= 11'd0;
                candidate_area[candidate_index] <= 21'd0;
            end
            for (candidate_index = 0; candidate_index < 7; candidate_index = candidate_index + 1) begin
                selected_left[candidate_index] <= 11'd0;
                selected_right[candidate_index] <= 11'd0;
                expand_core_left[candidate_index] <= 11'd0;
                expand_core_right[candidate_index] <= 11'd0;
                expand_left_limit_locked[candidate_index] <= 11'd0;
                expand_right_limit_locked[candidate_index] <= 11'd0;
            end
            o_char_up <= 11'd0; o_char_down <= 11'd0;
            o_c1_left <= 11'd0; o_c1_right <= 11'd0;
            o_c2_left <= 11'd0; o_c2_right <= 11'd0;
            o_c3_left <= 11'd0; o_c3_right <= 11'd0;
            o_c4_left <= 11'd0; o_c4_right <= 11'd0;
            o_c5_left <= 11'd0; o_c5_right <= 11'd0;
            o_c6_left <= 11'd0; o_c6_right <= 11'd0;
            o_c7_left <= 11'd0; o_c7_right <= 11'd0;
            o_char_valid <= 7'd0; o_char_detected <= 1'b0;
        end else if (vs_fall && (state != S_IDLE)) begin
            // 消隐不足时撤销未完成候选，不能在有效帧中途发布边界。
            o_char_valid <= 7'd0; o_char_detected <= 1'b0;
        end else begin
            case (state)
                S_IDLE: if (vs_rise) begin
                    scan_index <= 11'd0; run_height <= 11'd0;
                    best_up <= 11'd0; best_down <= 11'd0; best_height <= 11'd0;
                    segment_count <= 5'd0; segment_width <= 11'd0; segment_area <= 21'd0;
                    selected_valid <= 1'b0;
                    selected_layout_guard_active <= 1'b0;
                    merge_scan_index <= 4'd0;
                    merge_best_valid <= 1'b0;
                    merge_best_index <= 4'd0;
                    merge_best_score <= 32'd0;
                    merge_round <= 3'd0;
                    // Register the shift/add constants once per frame.  The
                    // merge states only consume these short registered terms.
                    // The original slot_center_term/slot_width_term calls
                    // use full roi_width.  merge_target_width remains the
                    // separate width/gap prior, exactly as in candidate9.
                    merge_w6_reg <= ({11'd0,roi_width} << 2) +
                                    ({11'd0,roi_width} << 1);
                    merge_w7_reg <= ({11'd0,roi_width} << 3) -
                                    {11'd0,roi_width};
                    merge_w18_reg <= ({11'd0,roi_width} << 4) +
                                     ({11'd0,roi_width} << 1);
                    merge_w26_reg <= (({11'd0,roi_width} << 4) +
                                      ({11'd0,roi_width} << 1)) +
                                     ({11'd0,roi_width} << 3);
                    merge_pair_shape_ok_reg <= 1'b0;
                    merge_slot_min_valid <= 1'b0;
                end
                S_H_CHECK: begin
                    if (row_occupied) begin
                        if (run_height == 0) run_up <= scan_index;
                        run_height <= run_height + 1'b1;
                    end else begin
                        if (run_height > best_height) begin
                            best_up <= run_up; best_down <= scan_index - 1'b1; best_height <= run_height;
                        end
                        run_height <= 11'd0;
                    end
                    if (scan_index != V_ACTIVE-1) scan_index <= scan_index + 1'b1;
                end
                S_H_FLUSH: begin
                    if (run_height > best_height) begin
                        best_up <= run_up; best_down <= V_ACTIVE-1; best_height <= run_height;
                    end
                    scan_index <= 11'd0;
                end
                S_V_CHECK: begin
                    if (col_occupied) begin
                        if (segment_width == 0) segment_left <= scan_index;
                        segment_width <= segment_width + 1'b1;
                        segment_area <= segment_area + v_scan_read;
                    end else begin
                        if (segment_width >= 11'd2) begin
                            if (!segment_edge_noise && (segment_count < 5'd12)) begin
                                candidate_left[segment_count] <= segment_left;
                                candidate_right[segment_count] <= scan_index - 1'b1;
                                candidate_area[segment_count] <= segment_area;
                                segment_count <= segment_count + 1'b1;
                            end else if (!segment_edge_noise) segment_count <= 5'd13;
                        end
                        segment_width <= 11'd0; segment_area <= 21'd0;
                    end
                    if (scan_index != H_ACTIVE-1) scan_index <= scan_index + 1'b1;
                end
                S_V_FLUSH: begin
                    if (segment_width >= 11'd2) begin
                        if (!segment_edge_noise && (segment_count < 5'd12)) begin
                            candidate_left[segment_count] <= segment_left;
                            candidate_right[segment_count] <= H_ACTIVE-1;
                            candidate_area[segment_count] <= segment_area;
                            segment_count <= segment_count + 1'b1;
                        end else if (!segment_edge_noise) segment_count <= 5'd13;
                    end
                end
                S_MERGE_INIT: begin
                    merge_scan_index <= 4'd0;
                    merge_best_valid <= 1'b0;
                    merge_best_index <= 4'd0;
                    merge_best_score <= 32'hffffffff;
                end
                S_MERGE_EVAL: begin
                    // A bounded case mux feeds pair registers only.  All
                    // subtraction, midpoint, shape and score work starts in
                    // later states from these stable fields.
                    case (merge_scan_index)
                        4'd0: begin
                            merge_pair_left_reg <= candidate_left[0];
                            merge_pair_right_reg <= candidate_right[0];
                            merge_pair_next_left_reg <= candidate_left[1];
                            merge_pair_next_right_reg <= candidate_right[1];
                        end
                        4'd1: begin
                            merge_pair_left_reg <= candidate_left[1];
                            merge_pair_right_reg <= candidate_right[1];
                            merge_pair_next_left_reg <= candidate_left[2];
                            merge_pair_next_right_reg <= candidate_right[2];
                        end
                        4'd2: begin
                            merge_pair_left_reg <= candidate_left[2];
                            merge_pair_right_reg <= candidate_right[2];
                            merge_pair_next_left_reg <= candidate_left[3];
                            merge_pair_next_right_reg <= candidate_right[3];
                        end
                        4'd3: begin
                            merge_pair_left_reg <= candidate_left[3];
                            merge_pair_right_reg <= candidate_right[3];
                            merge_pair_next_left_reg <= candidate_left[4];
                            merge_pair_next_right_reg <= candidate_right[4];
                        end
                        4'd4: begin
                            merge_pair_left_reg <= candidate_left[4];
                            merge_pair_right_reg <= candidate_right[4];
                            merge_pair_next_left_reg <= candidate_left[5];
                            merge_pair_next_right_reg <= candidate_right[5];
                        end
                        4'd5: begin
                            merge_pair_left_reg <= candidate_left[5];
                            merge_pair_right_reg <= candidate_right[5];
                            merge_pair_next_left_reg <= candidate_left[6];
                            merge_pair_next_right_reg <= candidate_right[6];
                        end
                        4'd6: begin
                            merge_pair_left_reg <= candidate_left[6];
                            merge_pair_right_reg <= candidate_right[6];
                            merge_pair_next_left_reg <= candidate_left[7];
                            merge_pair_next_right_reg <= candidate_right[7];
                        end
                        4'd7: begin
                            merge_pair_left_reg <= candidate_left[7];
                            merge_pair_right_reg <= candidate_right[7];
                            merge_pair_next_left_reg <= candidate_left[8];
                            merge_pair_next_right_reg <= candidate_right[8];
                        end
                        4'd8: begin
                            merge_pair_left_reg <= candidate_left[8];
                            merge_pair_right_reg <= candidate_right[8];
                            merge_pair_next_left_reg <= candidate_left[9];
                            merge_pair_next_right_reg <= candidate_right[9];
                        end
                        4'd9: begin
                            merge_pair_left_reg <= candidate_left[9];
                            merge_pair_right_reg <= candidate_right[9];
                            merge_pair_next_left_reg <= candidate_left[10];
                            merge_pair_next_right_reg <= candidate_right[10];
                        end
                        4'd10: begin
                            merge_pair_left_reg <= candidate_left[10];
                            merge_pair_right_reg <= candidate_right[10];
                            merge_pair_next_left_reg <= candidate_left[11];
                            merge_pair_next_right_reg <= candidate_right[11];
                        end
                        default: begin
                            merge_pair_left_reg <= 11'd0;
                            merge_pair_right_reg <= 11'd0;
                            merge_pair_next_left_reg <= 11'd0;
                            merge_pair_next_right_reg <= 11'd0;
                        end
                    endcase
                end
                S_MERGE_PREP: begin
                    merge_pair_order_ok_reg <= merge_pair_order_calc;
                    merge_pair_gap_reg <= merge_pair_gap_calc;
                    merge_pair_width_reg <= merge_pair_width_calc;
                    merge_pair_center_rel2_reg <= merge_pair_center_rel2_calc;
                    merge_pair_center_term_reg <= {9'd0,merge_pair_center_rel2_calc} << 6;
                    merge_pair_shape_ok_reg <= merge_pair_shape_calc;
                end
                S_MERGE_CHECK: begin
                    // Shape rejection advances directly to the next adjacent
                    // pair.  A valid pair initializes slot zero here and
                    // enters one center comparison per slot; this folds the
                    // former SLOT_INIT cycle into the registered check.
                    merge_slot_index <= 3'd0;
                    merge_slot_min_valid <= 1'b0;
                    merge_pair_slot_reg <= 3'd0;
                    merge_slot_center_target <= merge_w6_reg;
                end
                S_MERGE_SLOT_CALC: begin
                    // Strict less-than preserves candidate9's lower-slot tie
                    // order because slot zero is visited first.
                    if (!merge_slot_min_valid ||
                        (merge_slot_error_calc < merge_pair_center_error)) begin
                        merge_slot_min_valid <= 1'b1;
                        merge_pair_center_error <= merge_slot_error_calc;
                        merge_pair_slot_reg <= merge_slot_index;
                    end
                end
                S_MERGE_SLOT_NEXT: begin
                    if (merge_slot_index < 3'd6) begin
                        merge_slot_index <= merge_slot_index + 1'b1;
                        if (merge_slot_index == 3'd0)
                            merge_slot_center_target <= merge_slot_center_target + merge_w18_reg;
                        else if (merge_slot_index == 3'd1)
                            merge_slot_center_target <= merge_slot_center_target + merge_w26_reg;
                        else
                            merge_slot_center_target <= merge_slot_center_target + merge_w18_reg;
                    end
                end
                S_MERGE_WIDTH: begin
                    merge_expected_width <= merge_expected_width_calc;
                    merge_width_error <= merge_width_error_calc;
                end
                S_MERGE_SCORE: begin
                    merge_pair_score <= merge_pair_score_calc;
                    if (!merge_best_valid ||
                        (merge_pair_score_calc < merge_best_score)) begin
                        merge_best_valid <= 1'b1;
                        merge_best_index <= merge_scan_index;
                        merge_best_score <= merge_pair_score_calc;
                    end
                end
                S_MERGE_NEXT: begin
                    if ((merge_scan_index + 4'd1) < (segment_count - 5'd1))
                        merge_scan_index <= merge_scan_index + 4'd1;
                end
                S_SELECT: begin
                    selected_layout_guard_active <= 1'b0;
                    if ((segment_count > 5'd8) && merge_best_valid &&
                        (merge_round < 3'd5)) begin
                        // Compress the chosen pair in place. The next round
                        // starts from the updated count and scans one fewer pair.
                        // Only indices 0..10 may read the following entry.
                        // The last slot is cleared explicitly below, so no
                        // syntactic index 12 exists in the inferred mux.
                        for (candidate_index = 0; candidate_index < 11; candidate_index = candidate_index + 1) begin
                            if (candidate_index < (segment_count - 5'd1)) begin
                                if (candidate_index < merge_best_index) begin
                                    candidate_left[candidate_index] <= candidate_left[candidate_index];
                                    candidate_right[candidate_index] <= candidate_right[candidate_index];
                                    candidate_area[candidate_index] <= candidate_area[candidate_index];
                                end else if (candidate_index == merge_best_index) begin
                                    candidate_left[candidate_index] <= candidate_left[merge_best_index];
                                    candidate_right[candidate_index] <= candidate_right[merge_best_index + 4'd1];
                                    candidate_area[candidate_index] <=
                                        candidate_area[merge_best_index] +
                                        candidate_area[merge_best_index + 4'd1];
                                end else begin
                                    candidate_left[candidate_index] <= candidate_left[candidate_index + 4'd1];
                                    candidate_right[candidate_index] <= candidate_right[candidate_index + 4'd1];
                                    candidate_area[candidate_index] <= candidate_area[candidate_index + 4'd1];
                                end
                            end else begin
                                candidate_left[candidate_index] <= 11'd0;
                                candidate_right[candidate_index] <= 11'd0;
                                candidate_area[candidate_index] <= 21'd0;
                            end
                        end
                        candidate_left[11] <= 11'd0;
                        candidate_right[11] <= 11'd0;
                        candidate_area[11] <= 21'd0;
                        segment_count <= segment_count - 5'd1;
                        merge_round <= merge_round + 3'd1;
                        selected_valid <= 1'b0;
                    end else if (segment_count == 5'd7) begin
                        selected_valid <= roi_valid && (best_height >= 11'd8);
                        for (candidate_index = 0; candidate_index < 7; candidate_index = candidate_index + 1) begin
                            selected_left[candidate_index] <= candidate_left[candidate_index];
                            selected_right[candidate_index] <= candidate_right[candidate_index];
                        end
                    end else if (optional_dot) begin
                        // Preserve the existing fixed-position dot behavior.
                        selected_valid <= roi_valid && (best_height >= 11'd8);
                        for (candidate_index = 0; candidate_index < 7; candidate_index = candidate_index + 1) begin
                            if (candidate_index >= 2) begin
                                selected_left[candidate_index] <= candidate_left[candidate_index + 4'd1];
                                selected_right[candidate_index] <= candidate_right[candidate_index + 4'd1];
                            end else begin
                                selected_left[candidate_index] <= candidate_left[candidate_index];
                                selected_right[candidate_index] <= candidate_right[candidate_index];
                            end
                        end
                    end else if ((segment_count == 5'd8) && merge_best_valid) begin
                        // Final 8->7 merge: selected_* receives the merged
                        // view directly because candidate compression is nonblocking.
                        selected_valid <= roi_valid && (best_height >= 11'd8);
                        selected_layout_guard_active <= 1'b1;
                        for (candidate_index = 0; candidate_index < 7; candidate_index = candidate_index + 1) begin
                            if (candidate_index < merge_best_index) begin
                                selected_left[candidate_index] <= candidate_left[candidate_index];
                                selected_right[candidate_index] <= candidate_right[candidate_index];
                            end else if (candidate_index == merge_best_index) begin
                                selected_left[candidate_index] <= candidate_left[merge_best_index];
                                selected_right[candidate_index] <= candidate_right[merge_best_index + 4'd1];
                            end else begin
                                selected_left[candidate_index] <= candidate_left[candidate_index + 4'd1];
                                selected_right[candidate_index] <= candidate_right[candidate_index + 4'd1];
                            end
                        end
                        segment_count <= 5'd7;
                    end else begin
                        // Seven boxes are not forced from overflow, underflow,
                        // an unavailable pair, or a fifth-round failure.
                        selected_valid <= 1'b0;
                    end
                end
                S_EXP_INIT: begin
                    expand_cell <= 3'd0;
                    expand_side <= 1'b0;
                    expand_steps <= 6'd0;
                    expand_eligible <= 7'd0;
                    // This is the only point at which selected_* still holds
                    // the unexpanded seven core boxes.  Copy them before the
                    // serial walk is allowed to update either edge.
                    for (candidate_index = 0; candidate_index < 7; candidate_index = candidate_index + 1) begin
                        expand_core_left[candidate_index] <= selected_left[candidate_index];
                        expand_core_right[candidate_index] <= selected_right[candidate_index];
                    end
                end
                S_EXP_ELIG: begin
                    // Latch eligibility and both neighbour limits one cell at
                    // a time.  The source of every value is the immutable
                    // seven-core snapshot, so a left walk cannot move the
                    // midpoint used later by the right walk (or vice versa).
                    expand_eligible[expand_cell] <= expand_core_ordered &&
                                                   (expand_core_width < expand_narrow_limit);
                    expand_left_limit_locked[expand_cell] <= expand_core_left_limit;
                    expand_right_limit_locked[expand_cell] <= expand_core_right_limit;
                    if (expand_cell != 3'd6) expand_cell <= expand_cell + 1'b1;
                    else expand_cell <= 3'd0;
                end
                S_EXP_EVAL: begin
                    if (expand_step_ok) begin
                        if (expand_side) selected_right[expand_cell] <= expand_right_probe;
                        else selected_left[expand_cell] <= expand_left_probe;
                        expand_steps <= expand_steps + 1'b1;
                    end
                end
                S_EXP_NEXT: begin
                    expand_steps <= 6'd0;
                    if (expand_side) begin
                        expand_side <= 1'b0;
                        if (expand_cell != 3'd6) expand_cell <= expand_cell + 1'b1;
                    end else begin
                        expand_side <= 1'b1;
                    end
                end
                S_COMMIT: begin
                    o_char_up <= best_up; o_char_down <= best_down;
                    o_c1_left <= selected_left[0]; o_c1_right <= selected_right[0];
                    o_c2_left <= selected_left[1]; o_c2_right <= selected_right[1];
                    o_c3_left <= selected_left[2]; o_c3_right <= selected_right[2];
                    o_c4_left <= selected_left[3]; o_c4_right <= selected_right[3];
                    o_c5_left <= selected_left[4]; o_c5_right <= selected_right[4];
                    o_c6_left <= selected_left[5]; o_c6_right <= selected_right[5];
                    o_c7_left <= selected_left[6]; o_c7_right <= selected_right[6];
                    // A reused vertical band and a materially different
                    // horizontal best band are not a valid same-frame pair.
                    // Reject only this publication; the band manager above
                    // commits best_up/best_down for the next frame.
                    o_char_valid <= (!band_mismatch && selected_valid &&
                                     (!selected_layout_guard_active || selected_layout_ok)) ?
                                    7'h7f : 7'd0;
                    o_char_detected <= (!band_mismatch && selected_valid &&
                                        (!selected_layout_guard_active || selected_layout_ok));
                end
                default: begin end
            endcase
        end
    end
endmodule
