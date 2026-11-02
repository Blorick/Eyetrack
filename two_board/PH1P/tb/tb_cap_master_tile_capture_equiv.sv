//=============================================================================
//  tb_cap_master_tile_capture_equiv  --  capture arithmetic equivalence TB
//-----------------------------------------------------------------------------
//  所属域   : cap
//  时钟域   : I_clk_pix (arithmetic probes)
//  复位     : I_rst_pix_n
//  参数     : repeat-mask 7-bit domain, crop span 1..2048
//  验证状态 : Questa exhaustive arithmetic equivalence fixture
//=============================================================================
`timescale 1ns / 1ps

module tb_cap_master_tile_capture_equiv;
    reg W_clk = 1'b0;
    always #5 W_clk = ~W_clk;

    reg W_rst_n;
    reg W_ink;
    reg W_ink_de;
    reg W_vsync;
    reg W_detected;
    reg [10:0] W_char_up;
    reg [10:0] W_char_down;
    reg [6:0] W_char_valid;
    reg [10:0] W_left [0:6];
    reg [10:0] W_right [0:6];
    reg W_export_busy;

    wire W_tile_write_en;
    wire [8:0] W_tile_write_addr;
    wire [31:0] W_tile_write_data;
    wire W_snapshot_valid;
    wire W_snapshot_drop;
    wire W_snapshot_bank;
    wire [6:0] W_snapshot_mask;
    wire [31:0] W_snapshot_frame_index;
    wire W_capture_active;
    wire W_commit_active;

    reg [6:0] W_mask_start;
    reg [6:0] W_mask_count;
    reg [31:0] W_mask_got;
    reg [31:0] W_mask_expected;
    reg [11:0] W_span;
    reg [11:0] W_x_q;
    reg [11:0] W_x_r;
    reg [11:0] W_x_width;
    reg [11:0] W_y_q;
    reg [11:0] W_y_r;
    reg [11:0] W_y_height;
    reg [10:0] W_left_force;
    reg [10:0] W_right_force;
    reg [10:0] W_top_force;
    reg [10:0] W_bottom_force;
    integer W_start_index;
    integer W_count_index;
    integer W_width_index;
    integer W_height_index;
    integer W_expected_count;
    integer W_expected_error;
    integer W_expected_dst;
    integer W_errors;
    integer W_checks;
    integer W_k;
    integer W_geometry_checks;
    integer W_onehot_checks;
    integer W_onehot_mask;
    integer W_y_step_checks;
    integer W_y_step_errors;
    integer W_step_height_index;
    integer W_step_error_index;
    integer W_step_dst_index;
    integer W_step_dst_value;
    integer W_step_q_value;
    integer W_step_r_value;
    reg [31:0] W_step_got;
    reg [31:0] W_step_expected;
    reg [6:0] W_force_capture_valid;
    reg [10:0] W_force_pixel_x;
    reg [10:0] W_onehot_left [0:6];
    reg [10:0] W_onehot_right [0:6];
    reg [10:0] W_onehot_top [0:6];
    reg [10:0] W_onehot_bottom [0:6];
    reg [11:0] W_onehot_width [0:6];
    reg [11:0] W_onehot_q [0:6];
    reg [11:0] W_onehot_r [0:6];
    reg [11:0] W_onehot_error [0:6];
    reg [6:0] W_onehot_dst [0:6];

    task automatic check_geometry_case;
        input integer W_case_detected;
        input integer W_case_valid;
        input integer W_case_left;
        input integer W_case_right;
        input integer W_case_top;
        input integer W_case_bottom;
        input integer W_case_previous_valid;
        input integer W_case_previous_box_left;
        input integer W_case_previous_box_top;
        input integer W_case_previous_box_width;
        input integer W_case_previous_box_height;
        input integer W_case_ink_left;
        input integer W_case_ink_right;
        input integer W_case_ink_top;
        input integer W_case_ink_bottom;
        integer W_case_input_width;
        integer W_case_input_height;
        integer W_case_reuse;
        integer W_case_geometry_valid;
        integer W_case_crop_width;
        integer W_case_crop_height;
        integer W_case_x_q;
        integer W_case_x_r;
        integer W_case_y_q;
        integer W_case_y_r;
        integer W_case_abs_width;
        integer W_case_abs_height;
        integer W_case_abs_left;
        integer W_case_abs_top;
        begin
            W_rst_n = 1'b0;
            W_vsync = 1'b1;
            W_ink_de = 1'b0;
            W_detected = 1'b0;
            W_char_valid = 7'd0;
            repeat (3) @(posedge W_clk);
            W_rst_n = 1'b1;

            W_detected = (W_case_detected != 0);
            W_char_valid = W_case_valid ? 7'b0000001 : 7'd0;
            W_char_up = W_case_top;
            W_char_down = W_case_bottom;
            W_left[0] = W_case_left;
            W_right[0] = W_case_right;
            dut.previous_ink_valid[0] = (W_case_previous_valid != 0);
            dut.previous_box_left[0] = W_case_previous_box_left;
            dut.previous_box_top[0] = W_case_previous_box_top;
            dut.previous_box_width[0] = W_case_previous_box_width;
            dut.previous_box_height[0] = W_case_previous_box_height;
            dut.previous_ink_left_rel[0] = W_case_ink_left;
            dut.previous_ink_right_rel[0] = W_case_ink_right;
            dut.previous_ink_top_rel[0] = W_case_ink_top;
            dut.previous_ink_bottom_rel[0] = W_case_ink_bottom;
            @(posedge W_clk);

            W_vsync = 1'b0;
            @(posedge W_clk);
            #1;

            if (W_case_right >= W_case_left)
                W_case_input_width = W_case_right - W_case_left + 1;
            else
                W_case_input_width = 0;
            if (W_case_bottom >= W_case_top)
                W_case_input_height = W_case_bottom - W_case_top + 1;
            else
                W_case_input_height = 0;
            W_case_abs_width = W_case_input_width - W_case_previous_box_width;
            if (W_case_abs_width < 0) W_case_abs_width = -W_case_abs_width;
            W_case_abs_height = W_case_input_height - W_case_previous_box_height;
            if (W_case_abs_height < 0) W_case_abs_height = -W_case_abs_height;
            W_case_abs_left = W_case_left - W_case_previous_box_left;
            if (W_case_abs_left < 0) W_case_abs_left = -W_case_abs_left;
            W_case_abs_top = W_case_top - W_case_previous_box_top;
            if (W_case_abs_top < 0) W_case_abs_top = -W_case_abs_top;

            W_case_reuse = (W_case_previous_valid != 0) &&
                           (W_case_previous_box_width >= 2) &&
                           (W_case_previous_box_height >= 8) &&
                           (W_case_abs_width <= ((W_case_input_width >> 2) + 2)) &&
                           (W_case_abs_height <= ((W_case_input_height >> 2) + 2)) &&
                           (W_case_abs_left <= 1) &&
                           (W_case_abs_top <= 1) &&
                           (W_case_ink_right < W_case_input_width) &&
                           (W_case_ink_bottom < W_case_input_height);
            W_case_geometry_valid = (W_case_detected != 0) &&
                                    (W_case_valid != 0) &&
                                    (W_case_right >= W_case_left) &&
                                    (W_case_bottom >= W_case_top) &&
                                    (W_case_input_width >= 2) &&
                                    (W_case_input_height >= 8);
            if (W_case_reuse) begin
                W_case_crop_width = (W_case_ink_right >= W_case_ink_left) ?
                                    (W_case_ink_right - W_case_ink_left + 1) : 0;
                W_case_crop_height = (W_case_ink_bottom >= W_case_ink_top) ?
                                     (W_case_ink_bottom - W_case_ink_top + 1) : 0;
            end else begin
                W_case_crop_width = W_case_input_width;
                W_case_crop_height = W_case_input_height;
            end
            if (W_case_crop_width == 0) begin
                W_case_x_q = 0;
                W_case_x_r = 0;
            end else if (W_case_crop_width <= 32) begin
                W_case_x_q = 32 / W_case_crop_width;
                W_case_x_r = 32 % W_case_crop_width;
            end else begin
                W_case_x_q = 0;
                W_case_x_r = 32;
            end
            if (W_case_crop_height == 0) begin
                W_case_y_q = 0;
                W_case_y_r = 0;
            end else if (W_case_crop_height <= 32) begin
                W_case_y_q = 32 / W_case_crop_height;
                W_case_y_r = 32 % W_case_crop_height;
            end else begin
                W_case_y_q = 0;
                W_case_y_r = 32;
            end

            if (dut.W_geometry_valid[0] !== W_case_geometry_valid[0] ||
                dut.W_crop_reuse[0] !== W_case_reuse[0] ||
                dut.W_crop_width[0] !== W_case_crop_width[11:0] ||
                dut.W_crop_height[0] !== W_case_crop_height[11:0] ||
                dut.capture_valid[0] !== W_case_geometry_valid[0] ||
                dut.capture_width[0] !== (W_case_geometry_valid ? W_case_crop_width : 0) ||
                dut.capture_height[0] !== (W_case_geometry_valid ? W_case_crop_height : 0) ||
                dut.capture_x_q[0] !== (W_case_geometry_valid ? W_case_x_q : 0) ||
                dut.capture_x_r[0] !== (W_case_geometry_valid ? W_case_x_r : 0) ||
                dut.capture_y_q[0] !== (W_case_geometry_valid ? W_case_y_q : 0) ||
                dut.capture_y_r[0] !== (W_case_geometry_valid ? W_case_y_r : 0)) begin
                W_errors = W_errors + 1;
                $display("GEOMETRY_MISMATCH case=%0d valid=%0d reuse=%0d input=(%0d,%0d) crop=(%0d,%0d)",
                         W_geometry_checks + 1, W_case_geometry_valid, W_case_reuse,
                         W_case_input_width, W_case_input_height,
                         W_case_crop_width, W_case_crop_height);
            end
            W_geometry_checks = W_geometry_checks + 1;
        end
    endtask

    cap_master_tile_capture #(
        .H_ACTIVE(2048),
        .V_ACTIVE(2048)
    ) dut (
        .I_clk_pix(W_clk),
        .I_rst_pix_n(W_rst_n),
        .I_ink(W_ink),
        .I_ink_de(W_ink_de),
        .I_vsync(W_vsync),
        .I_detected(W_detected),
        .I_char_up(W_char_up),
        .I_char_down(W_char_down),
        .I_char_valid(W_char_valid),
        .I_c1_left(W_left[0]), .I_c1_right(W_right[0]),
        .I_c2_left(W_left[1]), .I_c2_right(W_right[1]),
        .I_c3_left(W_left[2]), .I_c3_right(W_right[2]),
        .I_c4_left(W_left[3]), .I_c4_right(W_right[3]),
        .I_c5_left(W_left[4]), .I_c5_right(W_right[4]),
        .I_c6_left(W_left[5]), .I_c6_right(W_right[5]),
        .I_c7_left(W_left[6]), .I_c7_right(W_right[6]),
        .I_export_busy(W_export_busy),
        .O_tile_write_en(W_tile_write_en),
        .O_tile_write_addr(W_tile_write_addr),
        .O_tile_write_data(W_tile_write_data),
        .O_snapshot_valid(W_snapshot_valid),
        .O_snapshot_drop(W_snapshot_drop),
        .O_snapshot_bank(W_snapshot_bank),
        .O_snapshot_mask(W_snapshot_mask),
        .O_snapshot_frame_index(W_snapshot_frame_index),
        .O_capture_active(W_capture_active),
        .O_commit_active(W_commit_active)
    );

    function automatic [31:0] golden_repeat_mask;
        input [6:0] W_start_value;
        input [6:0] W_count_value;
        integer W_repeat_index;
        begin
            golden_repeat_mask = 32'd0;
            for (W_repeat_index = 0;
                 W_repeat_index < W_count_value;
                 W_repeat_index = W_repeat_index + 1) begin
                if ((W_start_value + W_repeat_index) < 32)
                    golden_repeat_mask[W_start_value + W_repeat_index] = 1'b1;
            end
        end
    endfunction

    function automatic [31:0] golden_next_y_step;
        input integer W_height_value;
        input integer W_quotient_value;
        input integer W_remainder_value;
        input integer W_error_value;
        input integer W_destination_input;
        input integer W_first_value;
        integer W_sum_value;
        reg [5:0] W_count_result;
        reg [11:0] W_error_result;
        reg [6:0] W_destination_result;
        reg [6:0] W_start_result;
        begin
            W_sum_value = W_error_value + W_remainder_value;
            W_start_result = W_first_value ? 7'd0 : W_destination_input;
            if (W_first_value != 0) begin
                if (W_remainder_value != 0) begin
                    W_count_result = W_quotient_value + 1;
                    W_error_result = W_remainder_value - 1;
                    W_destination_result = W_quotient_value + 1;
                end else begin
                    W_count_result = W_quotient_value;
                    W_error_result = W_height_value - 1;
                    W_destination_result = W_quotient_value;
                end
            end else if (W_sum_value >= W_height_value) begin
                W_count_result = W_quotient_value + 1;
                W_error_result = W_sum_value - W_height_value;
                W_destination_result = W_destination_input + W_quotient_value + 1;
            end else begin
                W_count_result = W_quotient_value;
                W_error_result = W_sum_value;
                W_destination_result = W_destination_input + W_quotient_value;
            end
            golden_next_y_step = {W_count_result, W_error_result,
                                  W_destination_result, W_start_result};
        end
    endfunction

    task automatic check_first_x;
        begin
            if (dut.W_selected_x_start !== 7'd0 ||
                dut.W_selected_x_count !== W_expected_count[6:0] ||
                dut.W_selected_x_error_next !== W_expected_error[11:0]) begin
                W_errors = W_errors + 1;
                if (W_errors < 5)
                    $display("X_FIRST_MISMATCH width=%0d q=%0d r=%0d got=(%0d,%0d,%0d) exp=(0,%0d,%0d)",
                             W_width_index, W_x_q, W_x_r,
                             dut.W_selected_x_start, dut.W_selected_x_count,
                             dut.W_selected_x_error_next,
                             W_expected_count, W_expected_error);
            end
            W_checks = W_checks + 1;
        end
    endtask

    // Compare the one-hot selected lane with the former highest-slot dynamic
    // index for every nonempty hit vector.  Three passes cover the first
    // source pixel, a non-crossing error update, and an error crossing.
    task automatic check_onehot_selection;
        input integer W_case_mask;
        input integer W_case_mode;
        integer W_high_slot;
        integer W_slot_index;
        integer W_pixel_value;
        integer W_expected_start;
        integer W_expected_count;
        integer W_expected_error;
        integer W_error_sum;
        reg [6:0] W_expected_select;
        begin
            W_high_slot = 6;
            while (((W_case_mask >> W_high_slot) & 1) == 0)
                W_high_slot = W_high_slot - 1;
            W_expected_select = 7'd1 << W_high_slot;
            W_pixel_value = W_onehot_left[W_high_slot] +
                            ((W_case_mode == 0) ? 0 : 1);
            W_force_capture_valid = W_case_mask;
            W_force_pixel_x = W_pixel_value;
            force dut.capture_valid = W_force_capture_valid;
            force dut.pixel_x = W_force_pixel_x;
            force dut.pixel_y = 11'd0;
            #1;

            if (dut.capture_hit !== W_case_mask[6:0] ||
                dut.capture_select !== W_expected_select ||
                dut.W_capture_slot_index !== W_high_slot[2:0] ||
                dut.W_selected_left !== W_onehot_left[W_high_slot] ||
                dut.W_selected_width !== W_onehot_width[W_high_slot] ||
                dut.W_selected_quotient !== W_onehot_q[W_high_slot][6:0] ||
                dut.W_selected_remainder !== W_onehot_r[W_high_slot] ||
                dut.W_selected_error !== W_onehot_error[W_high_slot] ||
                dut.W_selected_destination !== W_onehot_dst[W_high_slot]) begin
                W_errors = W_errors + 1;
                if (W_errors < 5)
                    $display("ONEHOT_SELECT_MISMATCH mask=%02h mode=%0d high=%0d got_hit=%02h got_sel=%02h got_idx=%0d",
                             W_case_mask, W_case_mode, W_high_slot,
                             dut.capture_hit, dut.capture_select,
                             dut.W_capture_slot_index);
            end

            if (W_case_mode == 0) begin
                W_expected_start = 0;
                if (W_onehot_r[W_high_slot] != 0) begin
                    W_expected_count = W_onehot_q[W_high_slot] + 1;
                    W_expected_error = W_onehot_r[W_high_slot] - 1;
                end else begin
                    W_expected_count = W_onehot_q[W_high_slot];
                    W_expected_error = W_onehot_width[W_high_slot] - 1;
                end
            end else begin
                W_expected_start = W_onehot_dst[W_high_slot];
                W_error_sum = W_onehot_error[W_high_slot] +
                              W_onehot_r[W_high_slot];
                if (W_error_sum >= W_onehot_width[W_high_slot]) begin
                    W_expected_count = W_onehot_q[W_high_slot] + 1;
                    W_expected_error = W_error_sum - W_onehot_width[W_high_slot];
                end else begin
                    W_expected_count = W_onehot_q[W_high_slot];
                    W_expected_error = W_error_sum;
                end
            end
            if (dut.W_selected_x_start !== W_expected_start[6:0] ||
                dut.W_selected_x_count !== W_expected_count[6:0] ||
                dut.W_selected_x_error_next !== W_expected_error[11:0]) begin
                W_errors = W_errors + 1;
                if (W_errors < 5)
                    $display("ONEHOT_ARITH_MISMATCH mask=%02h mode=%0d high=%0d got=(%0d,%0d,%0d) exp=(%0d,%0d,%0d)",
                             W_case_mask, W_case_mode, W_high_slot,
                             dut.W_selected_x_start, dut.W_selected_x_count,
                             dut.W_selected_x_error_next,
                             W_expected_start, W_expected_count,
                             W_expected_error);
            end
            W_onehot_checks = W_onehot_checks + 1;
        end
    endtask

    initial begin
        W_rst_n = 1'b0;
        W_ink = 1'b0;
        W_ink_de = 1'b0;
        W_vsync = 1'b0;
        W_detected = 1'b0;
        W_char_up = 11'd0;
        W_char_down = 11'd0;
        W_char_valid = 7'd0;
        W_export_busy = 1'b0;
        for (W_k = 0; W_k < 7; W_k = W_k + 1) begin
            W_left[W_k] = 11'd0;
            W_right[W_k] = 11'd0;
        end
        W_errors = 0;
        W_checks = 0;
        W_geometry_checks = 0;
        W_onehot_checks = 0;
        W_y_step_checks = 0;
        W_y_step_errors = 0;

        repeat (3) @(posedge W_clk);
        W_rst_n = 1'b1;
        @(posedge W_clk);

        // Exercise the actual DUT function for every pair of 7-bit inputs.
        for (W_start_index = 0; W_start_index < 128; W_start_index = W_start_index + 1) begin
            for (W_count_index = 0; W_count_index < 128; W_count_index = W_count_index + 1) begin
                W_mask_start = W_start_index[6:0];
                W_mask_count = W_count_index[6:0];
                #1;
                W_mask_got = dut.repeat_mask(W_mask_start, W_mask_count);
                W_mask_expected = golden_repeat_mask(W_mask_start, W_mask_count);
                if (W_mask_got !== W_mask_expected) begin
                    W_errors = W_errors + 1;
                    if (W_errors < 5)
                        $display("MASK_MISMATCH start=%0d count=%0d got=%h exp=%h",
                                 W_start_index, W_count_index,
                                 W_mask_got, W_mask_expected);
                end
                W_checks = W_checks + 1;
            end
        end

        // Compare the shared crop-selection phase with the former per-branch
        // geometry contract across valid, fallback, invalid and boundary cases.
        check_geometry_case(1, 1, 10, 11, 20, 27,
                            0, 0, 0, 0, 0, 0, 0, 0, 0);
        check_geometry_case(1, 1, 0, 2047, 0, 2047,
                            0, 0, 0, 0, 0, 0, 0, 0, 0);
        check_geometry_case(1, 1, 100, 131, 100, 131,
                            0, 0, 0, 0, 0, 0, 0, 0, 0);
        check_geometry_case(1, 1, 50, 89, 60, 99,
                            1, 49, 59, 40, 40, 3, 34, 2, 33);
        check_geometry_case(1, 1, 50, 89, 60, 99,
                            1, 49, 59, 20, 40, 3, 34, 2, 33);
        check_geometry_case(0, 1, 50, 89, 60, 99,
                            0, 0, 0, 0, 0, 0, 0, 0, 0);
        check_geometry_case(1, 1, 100, 100, 20, 27,
                            0, 0, 0, 0, 0, 0, 0, 0, 0);
        check_geometry_case(1, 1, 100, 101, 20, 26,
                            0, 0, 0, 0, 0, 0, 0, 0, 0);
        check_geometry_case(1, 1, 50, 89, 60, 99,
                            1, 49, 59, 40, 40, 3, 40, 2, 33);
        check_geometry_case(1, 1, 90, 89, 60, 99,
                            0, 0, 0, 0, 0, 0, 0, 0, 0);

        // Exhaustively compare the shared vertical step against the former
        // recurrence for every legal height/error state and four destination
        // representatives.  The first-row branch is checked for every height.
        for (W_step_height_index = 1;
             W_step_height_index <= 2048;
             W_step_height_index = W_step_height_index + 1) begin
            if (W_step_height_index <= 32) begin
                W_step_q_value = 32 / W_step_height_index;
                W_step_r_value = 32 % W_step_height_index;
            end else begin
                W_step_q_value = 0;
                W_step_r_value = 32;
            end
            W_step_got = dut.next_y_step(
                W_step_height_index, W_step_q_value, W_step_r_value,
                0, 0, 1);
            W_step_expected = golden_next_y_step(
                W_step_height_index, W_step_q_value, W_step_r_value,
                0, 0, 1);
            if (W_step_got !== W_step_expected) begin
                W_y_step_errors = W_y_step_errors + 1;
                if (W_y_step_errors < 5)
                    $display("Y_STEP_FIRST_MISMATCH height=%0d got=%h exp=%h",
                             W_step_height_index, W_step_got, W_step_expected);
            end
            W_y_step_checks = W_y_step_checks + 1;
            for (W_step_error_index = 0;
                 W_step_error_index < W_step_height_index;
                 W_step_error_index = W_step_error_index + 1) begin
                for (W_step_dst_index = 0; W_step_dst_index < 4;
                     W_step_dst_index = W_step_dst_index + 1) begin
                    case (W_step_dst_index)
                        0: W_step_dst_value = 0;
                        1: W_step_dst_value = 31;
                        2: W_step_dst_value = 32;
                        default: W_step_dst_value = 127;
                    endcase
                    W_step_got = dut.next_y_step(
                        W_step_height_index, W_step_q_value, W_step_r_value,
                        W_step_error_index, W_step_dst_value, 0);
                    W_step_expected = golden_next_y_step(
                        W_step_height_index, W_step_q_value, W_step_r_value,
                        W_step_error_index, W_step_dst_value, 0);
                    if (W_step_got !== W_step_expected) begin
                        W_y_step_errors = W_y_step_errors + 1;
                        if (W_y_step_errors < 5)
                            $display("Y_STEP_MISMATCH height=%0d error=%0d dst=%0d got=%h exp=%h",
                                     W_step_height_index, W_step_error_index,
                                     W_step_dst_value, W_step_got,
                                     W_step_expected);
                    end
                    W_y_step_checks = W_y_step_checks + 1;
                end
            end
        end
        if (W_y_step_errors != 0)
            W_errors = W_errors + W_y_step_errors;

        // Force distinct per-slot state and exercise every nonempty hit
        // vector.  The ascending left edge makes all lower slots hit at the
        // selected highest slot's x coordinate, preserving the exact mask.
        for (W_k = 0; W_k < 7; W_k = W_k + 1) begin
            W_onehot_left[W_k] = 11'd100 + W_k * 11'd10;
            W_onehot_right[W_k] = 11'd2047;
            W_onehot_top[W_k] = 11'd0;
            W_onehot_bottom[W_k] = 11'd2047;
            W_onehot_width[W_k] = 12'd400 + W_k * 12'd10;
            W_onehot_q[W_k] = 12'd3 + W_k;
            W_onehot_r[W_k] = 12'd2 + W_k * 12'd2;
            W_onehot_error[W_k] = 12'd20 + W_k * 12'd3;
            W_onehot_dst[W_k] = 7'd30 + W_k * 7'd2;
        end
        force dut.capture_left[0] = 11'd100;
        force dut.capture_left[1] = 11'd110;
        force dut.capture_left[2] = 11'd120;
        force dut.capture_left[3] = 11'd130;
        force dut.capture_left[4] = 11'd140;
        force dut.capture_left[5] = 11'd150;
        force dut.capture_left[6] = 11'd160;
        force dut.capture_right[0] = 11'd2047;
        force dut.capture_right[1] = 11'd2047;
        force dut.capture_right[2] = 11'd2047;
        force dut.capture_right[3] = 11'd2047;
        force dut.capture_right[4] = 11'd2047;
        force dut.capture_right[5] = 11'd2047;
        force dut.capture_right[6] = 11'd2047;
        force dut.capture_top[0] = 11'd0;
        force dut.capture_top[1] = 11'd0;
        force dut.capture_top[2] = 11'd0;
        force dut.capture_top[3] = 11'd0;
        force dut.capture_top[4] = 11'd0;
        force dut.capture_top[5] = 11'd0;
        force dut.capture_top[6] = 11'd0;
        force dut.capture_bottom[0] = 11'd2047;
        force dut.capture_bottom[1] = 11'd2047;
        force dut.capture_bottom[2] = 11'd2047;
        force dut.capture_bottom[3] = 11'd2047;
        force dut.capture_bottom[4] = 11'd2047;
        force dut.capture_bottom[5] = 11'd2047;
        force dut.capture_bottom[6] = 11'd2047;
        force dut.capture_width[0] = 12'd400;
        force dut.capture_width[1] = 12'd410;
        force dut.capture_width[2] = 12'd420;
        force dut.capture_width[3] = 12'd430;
        force dut.capture_width[4] = 12'd440;
        force dut.capture_width[5] = 12'd450;
        force dut.capture_width[6] = 12'd460;
        force dut.capture_x_q[0] = 12'd3;
        force dut.capture_x_q[1] = 12'd4;
        force dut.capture_x_q[2] = 12'd5;
        force dut.capture_x_q[3] = 12'd6;
        force dut.capture_x_q[4] = 12'd7;
        force dut.capture_x_q[5] = 12'd8;
        force dut.capture_x_q[6] = 12'd9;
        force dut.capture_x_r[0] = 12'd2;
        force dut.capture_x_r[1] = 12'd4;
        force dut.capture_x_r[2] = 12'd6;
        force dut.capture_x_r[3] = 12'd8;
        force dut.capture_x_r[4] = 12'd10;
        force dut.capture_x_r[5] = 12'd12;
        force dut.capture_x_r[6] = 12'd14;
        force dut.x_err[0] = 12'd20;
        force dut.x_err[1] = 12'd23;
        force dut.x_err[2] = 12'd26;
        force dut.x_err[3] = 12'd29;
        force dut.x_err[4] = 12'd32;
        force dut.x_err[5] = 12'd35;
        force dut.x_err[6] = 12'd38;
        force dut.x_dst[0] = 7'd30;
        force dut.x_dst[1] = 7'd32;
        force dut.x_dst[2] = 7'd34;
        force dut.x_dst[3] = 7'd36;
        force dut.x_dst[4] = 7'd38;
        force dut.x_dst[5] = 7'd40;
        force dut.x_dst[6] = 7'd42;
        for (W_onehot_mask = 1; W_onehot_mask < 128; W_onehot_mask = W_onehot_mask + 1)
            check_onehot_selection(W_onehot_mask, 0);
        for (W_onehot_mask = 1; W_onehot_mask < 128; W_onehot_mask = W_onehot_mask + 1)
            check_onehot_selection(W_onehot_mask, 1);
        for (W_k = 0; W_k < 7; W_k = W_k + 1)
            W_onehot_error[W_k] = W_onehot_width[W_k] -
                                  W_onehot_r[W_k] + W_k;
        release dut.x_err[0]; release dut.x_err[1];
        release dut.x_err[2]; release dut.x_err[3];
        release dut.x_err[4]; release dut.x_err[5];
        release dut.x_err[6];
        force dut.x_err[0] = 12'd398;
        force dut.x_err[1] = 12'd407;
        force dut.x_err[2] = 12'd416;
        force dut.x_err[3] = 12'd425;
        force dut.x_err[4] = 12'd434;
        force dut.x_err[5] = 12'd443;
        force dut.x_err[6] = 12'd452;
        for (W_onehot_mask = 1; W_onehot_mask < 128; W_onehot_mask = W_onehot_mask + 1)
            check_onehot_selection(W_onehot_mask, 2);

        release dut.capture_valid;
        release dut.pixel_x;
        release dut.pixel_y;
        release dut.capture_left[0]; release dut.capture_left[1];
        release dut.capture_left[2]; release dut.capture_left[3];
        release dut.capture_left[4]; release dut.capture_left[5];
        release dut.capture_left[6];
        release dut.capture_right[0]; release dut.capture_right[1];
        release dut.capture_right[2]; release dut.capture_right[3];
        release dut.capture_right[4]; release dut.capture_right[5];
        release dut.capture_right[6];
        release dut.capture_top[0]; release dut.capture_top[1];
        release dut.capture_top[2]; release dut.capture_top[3];
        release dut.capture_top[4]; release dut.capture_top[5];
        release dut.capture_top[6];
        release dut.capture_bottom[0]; release dut.capture_bottom[1];
        release dut.capture_bottom[2]; release dut.capture_bottom[3];
        release dut.capture_bottom[4]; release dut.capture_bottom[5];
        release dut.capture_bottom[6];
        release dut.capture_width[0]; release dut.capture_width[1];
        release dut.capture_width[2]; release dut.capture_width[3];
        release dut.capture_width[4]; release dut.capture_width[5];
        release dut.capture_width[6];
        release dut.capture_x_q[0]; release dut.capture_x_q[1];
        release dut.capture_x_q[2]; release dut.capture_x_q[3];
        release dut.capture_x_q[4]; release dut.capture_x_q[5];
        release dut.capture_x_q[6];
        release dut.capture_x_r[0]; release dut.capture_x_r[1];
        release dut.capture_x_r[2]; release dut.capture_x_r[3];
        release dut.capture_x_r[4]; release dut.capture_x_r[5];
        release dut.capture_x_r[6];
        release dut.x_err[0]; release dut.x_err[1];
        release dut.x_err[2]; release dut.x_err[3];
        release dut.x_err[4]; release dut.x_err[5];
        release dut.x_err[6];
        release dut.x_dst[0]; release dut.x_dst[1];
        release dut.x_dst[2]; release dut.x_dst[3];
        release dut.x_dst[4]; release dut.x_dst[5];
        release dut.x_dst[6];

        // Force only the selected geometry state so the first-pixel x branch
        // is observed from the real combinational RTL, without a video frame.
        W_left_force = 11'd0;
        W_top_force = 11'd0;
        W_x_width = 12'd1;
        W_y_height = 12'd8;
        W_x_q = 12'd0;
        W_x_r = 12'd1;
        W_y_q = 12'd0;
        W_y_r = 12'd8;
        W_right_force = 11'd0;
        W_bottom_force = 11'd7;
        force dut.capture_active = 1'b1;
        force dut.pixel_x = 11'd0;
        force dut.pixel_y = 11'd0;
        force dut.capture_valid[0] = 1'b1;
        force dut.capture_left[0] = W_left_force;
        force dut.capture_right[0] = W_right_force;
        force dut.capture_top[0] = W_top_force;
        force dut.capture_bottom[0] = W_bottom_force;
        force dut.capture_width[0] = W_x_width;
        force dut.capture_x_q[0] = W_x_q;
        force dut.capture_x_r[0] = W_x_r;
        force dut.capture_height[0] = W_y_height;
        force dut.capture_y_q[0] = W_y_q;
        force dut.capture_y_r[0] = W_y_r;
        W_ink_de = 1'b1;

        for (W_width_index = 1; W_width_index <= 2048; W_width_index = W_width_index + 1) begin
            W_x_width = W_width_index;
            W_right_force = W_width_index - 1;
            if (W_width_index <= 32) begin
                W_x_q = 32 / W_width_index;
                W_x_r = 32 % W_width_index;
            end else begin
                W_x_q = 0;
                W_x_r = 32;
            end
            if (W_x_r != 0) begin
                W_expected_count = W_x_q + 1;
                W_expected_error = W_x_r - 1;
            end else begin
                W_expected_count = W_x_q;
                W_expected_error = W_x_width - 1;
            end
            #1;
            check_first_x();
        end

        // The sequential first-row y branch is checked at the earliest
        // consumer cycle.  H_ACTIVE=2048 uses the shared lane: x=0 starts
        // the lane, x=1..7 update slots 0..6, and x=8 consumes all rows.
        release dut.pixel_x;
        force dut.pixel_x = W_force_pixel_x;
        for (W_height_index = 1; W_height_index <= 2048; W_height_index = W_height_index + 1) begin
            W_y_height = W_height_index;
            W_bottom_force = W_height_index - 1;
            if (W_height_index <= 32) begin
                W_y_q = 32 / W_height_index;
                W_y_r = 32 % W_height_index;
            end else begin
                W_y_q = 0;
                W_y_r = 32;
            end
            if (W_y_r != 0) begin
                W_expected_count = (W_y_q + 1) & 63;
                W_expected_error = W_y_r - 1;
                W_expected_dst = W_y_q + 1;
            end else begin
                W_expected_count = W_y_q & 63;
                W_expected_error = W_height_index - 1;
                W_expected_dst = W_y_q;
            end
            W_force_pixel_x = 0;
            @(posedge W_clk);
            for (W_k = 1; W_k <= 8; W_k = W_k + 1) begin
                W_force_pixel_x = W_k;
                @(posedge W_clk);
            end
            #1;
            if (dut.row_rep_count[0] !== W_expected_count[5:0] ||
                dut.y_err[0] !== W_expected_error[11:0] ||
                dut.y_dst[0] !== W_expected_dst[6:0]) begin
                W_errors = W_errors + 1;
                if (W_errors < 5)
                    $display("Y_FIRST_MISMATCH height=%0d q=%0d r=%0d got=(%0d,%0d,%0d) exp=(%0d,%0d,%0d)",
                             W_height_index, W_y_q, W_y_r,
                             dut.row_rep_count[0], dut.y_err[0], dut.y_dst[0],
                             W_expected_count, W_expected_error, W_expected_dst);
            end
            W_checks = W_checks + 1;
        end

        release dut.capture_active;
        release dut.pixel_x;
        release dut.pixel_y;
        release dut.capture_valid[0];
        release dut.capture_left[0];
        release dut.capture_right[0];
        release dut.capture_top[0];
        release dut.capture_bottom[0];
        release dut.capture_width[0];
        release dut.capture_x_q[0];
        release dut.capture_x_r[0];
        release dut.capture_height[0];
        release dut.capture_y_q[0];
        release dut.capture_y_r[0];
        W_ink_de = 1'b0;

        if (W_errors == 0)
            $display("PH1P_CAPTURE_EQUIV_TB_PASS checks=%0d errors=0 mask_pairs=16384 x_spans=2048 y_spans=2048 geometry_cases=%0d onehot_cases=%0d y_step_checks=%0d", W_checks, W_geometry_checks, W_onehot_checks, W_y_step_checks);
        else
            $display("PH1P_CAPTURE_EQUIV_TB_FAIL checks=%0d errors=%0d", W_checks, W_errors);
        $finish;
    end
endmodule
