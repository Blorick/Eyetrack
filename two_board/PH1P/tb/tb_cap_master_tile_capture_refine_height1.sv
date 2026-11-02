//=============================================================================
//  tb_cap_master_tile_capture_refine_height1 -- two-frame thin-crop fixture
//-----------------------------------------------------------------------------
//  验证范围 : legal input box H=16, stable first-frame box, and the downstream
//             crop-reuse state with a one-row previous ink crop.  The first
//             frame is real video input; the metadata force after its snapshot
//             isolates the crop state that the current RTL qualification gate
//             cannot naturally produce from a horizontal-only ink line.
//  目标     : H_ACTIVE=640/1280, seven slots, second bank, 224 words.
//=============================================================================
`timescale 1ns / 1ps

module tb_cap_master_tile_capture_refine_height1_case #(
    parameter integer H_ACTIVE = 640,
    parameter integer CASE_ID = 640
) (
    output reg O_done,
    output reg [31:0] O_errors,
    output reg [31:0] O_first_writes,
    output reg [31:0] O_second_writes,
    output reg [31:0] O_natural_previous_valid
);
    localparam integer BOX_HEIGHT = 16;
    localparam integer CROP_TOP = 4;

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
    integer W_phase;

    wire W_tile_write_en;
    wire [8:0] W_tile_write_addr;
    wire [31:0] W_tile_write_data;
    wire W_snapshot_valid;
    wire W_snapshot_drop;
    wire W_snapshot_bank;
    wire [6:0] W_snapshot_mask;

    integer W_errors;
    integer W_first_write_count;
    integer W_second_write_count;
    integer W_snapshot_count;
    integer W_snapshot1_count;
    integer W_snapshot2_count;
    integer W_snapshot1_bank;
    integer W_snapshot2_bank;
    integer W_snapshot1_mask;
    integer W_snapshot2_mask;
    integer W_natural_previous_valid;
    integer W_k;
    integer W_wait;
    integer W_addr;
    reg [447:0] W_seen;
    reg [31:0] W_second_data_by_addr [0:447];

    cap_master_tile_capture #(
        .H_ACTIVE(H_ACTIVE),
        .V_ACTIVE(64)
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
        .O_snapshot_frame_index(),
        .O_capture_active(),
        .O_commit_active()
    );

    function automatic integer slot_left;
        input integer W_slot;
        begin
            slot_left = (W_slot * 16) + 2;
        end
    endfunction

    function automatic integer slot_right;
        input integer W_slot;
        begin
            slot_right = slot_left(W_slot) + 7;
        end
    endfunction

    function automatic integer ink_value;
        input integer W_x;
        input integer W_y;
        begin
            // Four points per row make the first frame a stable full-height
            // source box while retaining a same-y four-point witness.
            ink_value = (((W_x + (W_y * 2) + CASE_ID) % 8) < 4);
        end
    endfunction

    function automatic [31:0] golden_refined_word;
        input integer W_slot;
        integer W_source_x;
        integer W_global_x;
        integer W_x_start;
        integer W_x_end;
        integer W_output_x;
        begin
            golden_refined_word = 32'd0;
            for (W_source_x = 0; W_source_x < 8; W_source_x = W_source_x + 1) begin
                W_global_x = slot_left(W_slot) + W_source_x;
                W_x_start = (W_source_x * 32 + 7) / 8;
                W_x_end = ((W_source_x + 1) * 32 + 7) / 8;
                if (ink_value(W_global_x, CROP_TOP))
                    for (W_output_x = W_x_start; W_output_x < W_x_end;
                         W_output_x = W_output_x + 1)
                        golden_refined_word[W_output_x] = 1'b1;
            end
        end
    endfunction

    task automatic clear_log;
        integer W_index;
        begin
            W_first_write_count = 0;
            W_second_write_count = 0;
            W_snapshot_count = 0;
            W_snapshot1_count = 0;
            W_snapshot2_count = 0;
            W_snapshot1_bank = -1;
            W_snapshot2_bank = -1;
            W_snapshot1_mask = -1;
            W_snapshot2_mask = -1;
            W_seen = {448{1'b0}};
            for (W_index = 0; W_index < 448; W_index = W_index + 1)
                W_second_data_by_addr[W_index] = 32'd0;
        end
    endtask

    task automatic drive_first_frame;
        integer W_x;
        integer W_y;
        begin
            W_vsync = 1'b0;
            W_ink_de = 1'b0;
            W_ink = 1'b0;
            @(posedge W_clk);
            for (W_y = 0; W_y < 64; W_y = W_y + 1) begin
                for (W_x = 0; W_x < H_ACTIVE; W_x = W_x + 1) begin
                    W_ink_de = 1'b1;
                    W_ink = ink_value(W_x, W_y);
                    @(posedge W_clk);
                end
            end
            W_ink_de = 1'b0;
            W_ink = 1'b0;
            @(posedge W_clk);
            W_vsync = 1'b1;
            @(posedge W_clk);
            repeat (4) @(posedge W_clk);
        end
    endtask

    task automatic force_thin_previous_crop;
        begin
            // The box history is stable and legal.  Only the measured ink
            // crop is directed to a one-row state, which is allowed by the
            // W_crop_reuse expression but rejected by the current update
            // qualification when a horizontal-only source is used.
            force dut.previous_ink_valid = 7'h7f;
            force dut.previous_box_left[0] = slot_left(0);
            force dut.previous_box_left[1] = slot_left(1);
            force dut.previous_box_left[2] = slot_left(2);
            force dut.previous_box_left[3] = slot_left(3);
            force dut.previous_box_left[4] = slot_left(4);
            force dut.previous_box_left[5] = slot_left(5);
            force dut.previous_box_left[6] = slot_left(6);
            force dut.previous_box_top[0] = 11'd0;
            force dut.previous_box_top[1] = 11'd0;
            force dut.previous_box_top[2] = 11'd0;
            force dut.previous_box_top[3] = 11'd0;
            force dut.previous_box_top[4] = 11'd0;
            force dut.previous_box_top[5] = 11'd0;
            force dut.previous_box_top[6] = 11'd0;
            force dut.previous_box_width[0] = 12'd8;
            force dut.previous_box_width[1] = 12'd8;
            force dut.previous_box_width[2] = 12'd8;
            force dut.previous_box_width[3] = 12'd8;
            force dut.previous_box_width[4] = 12'd8;
            force dut.previous_box_width[5] = 12'd8;
            force dut.previous_box_width[6] = 12'd8;
            force dut.previous_box_height[0] = BOX_HEIGHT;
            force dut.previous_box_height[1] = BOX_HEIGHT;
            force dut.previous_box_height[2] = BOX_HEIGHT;
            force dut.previous_box_height[3] = BOX_HEIGHT;
            force dut.previous_box_height[4] = BOX_HEIGHT;
            force dut.previous_box_height[5] = BOX_HEIGHT;
            force dut.previous_box_height[6] = BOX_HEIGHT;
            force dut.previous_ink_left_rel[0] = 11'd0;
            force dut.previous_ink_left_rel[1] = 11'd0;
            force dut.previous_ink_left_rel[2] = 11'd0;
            force dut.previous_ink_left_rel[3] = 11'd0;
            force dut.previous_ink_left_rel[4] = 11'd0;
            force dut.previous_ink_left_rel[5] = 11'd0;
            force dut.previous_ink_left_rel[6] = 11'd0;
            force dut.previous_ink_right_rel[0] = 11'd7;
            force dut.previous_ink_right_rel[1] = 11'd7;
            force dut.previous_ink_right_rel[2] = 11'd7;
            force dut.previous_ink_right_rel[3] = 11'd7;
            force dut.previous_ink_right_rel[4] = 11'd7;
            force dut.previous_ink_right_rel[5] = 11'd7;
            force dut.previous_ink_right_rel[6] = 11'd7;
            force dut.previous_ink_top_rel[0] = CROP_TOP;
            force dut.previous_ink_top_rel[1] = CROP_TOP;
            force dut.previous_ink_top_rel[2] = CROP_TOP;
            force dut.previous_ink_top_rel[3] = CROP_TOP;
            force dut.previous_ink_top_rel[4] = CROP_TOP;
            force dut.previous_ink_top_rel[5] = CROP_TOP;
            force dut.previous_ink_top_rel[6] = CROP_TOP;
            force dut.previous_ink_bottom_rel[0] = CROP_TOP;
            force dut.previous_ink_bottom_rel[1] = CROP_TOP;
            force dut.previous_ink_bottom_rel[2] = CROP_TOP;
            force dut.previous_ink_bottom_rel[3] = CROP_TOP;
            force dut.previous_ink_bottom_rel[4] = CROP_TOP;
            force dut.previous_ink_bottom_rel[5] = CROP_TOP;
            force dut.previous_ink_bottom_rel[6] = CROP_TOP;
        end
    endtask

    task automatic release_thin_previous_crop;
        begin
            release dut.previous_ink_valid;
            release dut.previous_box_left[0]; release dut.previous_box_left[1];
            release dut.previous_box_left[2]; release dut.previous_box_left[3];
            release dut.previous_box_left[4]; release dut.previous_box_left[5];
            release dut.previous_box_left[6];
            release dut.previous_box_top[0]; release dut.previous_box_top[1];
            release dut.previous_box_top[2]; release dut.previous_box_top[3];
            release dut.previous_box_top[4]; release dut.previous_box_top[5];
            release dut.previous_box_top[6];
            release dut.previous_box_width[0]; release dut.previous_box_width[1];
            release dut.previous_box_width[2]; release dut.previous_box_width[3];
            release dut.previous_box_width[4]; release dut.previous_box_width[5];
            release dut.previous_box_width[6];
            release dut.previous_box_height[0]; release dut.previous_box_height[1];
            release dut.previous_box_height[2]; release dut.previous_box_height[3];
            release dut.previous_box_height[4]; release dut.previous_box_height[5];
            release dut.previous_box_height[6];
            release dut.previous_ink_left_rel[0]; release dut.previous_ink_left_rel[1];
            release dut.previous_ink_left_rel[2]; release dut.previous_ink_left_rel[3];
            release dut.previous_ink_left_rel[4]; release dut.previous_ink_left_rel[5];
            release dut.previous_ink_left_rel[6];
            release dut.previous_ink_right_rel[0]; release dut.previous_ink_right_rel[1];
            release dut.previous_ink_right_rel[2]; release dut.previous_ink_right_rel[3];
            release dut.previous_ink_right_rel[4]; release dut.previous_ink_right_rel[5];
            release dut.previous_ink_right_rel[6];
            release dut.previous_ink_top_rel[0]; release dut.previous_ink_top_rel[1];
            release dut.previous_ink_top_rel[2]; release dut.previous_ink_top_rel[3];
            release dut.previous_ink_top_rel[4]; release dut.previous_ink_top_rel[5];
            release dut.previous_ink_top_rel[6];
            release dut.previous_ink_bottom_rel[0]; release dut.previous_ink_bottom_rel[1];
            release dut.previous_ink_bottom_rel[2]; release dut.previous_ink_bottom_rel[3];
            release dut.previous_ink_bottom_rel[4]; release dut.previous_ink_bottom_rel[5];
            release dut.previous_ink_bottom_rel[6];
        end
    endtask

    task automatic drive_second_frame;
        integer W_x;
        integer W_y;
        begin
            W_vsync = 1'b0;
            W_ink_de = 1'b0;
            W_ink = 1'b0;
            @(posedge W_clk);
            #1;
            for (W_k = 0; W_k < 7; W_k = W_k + 1) begin
                // Keep this check compatible with the frozen legacy source.
                // The legacy module has no W_crop_reuse/W_crop_height wires;
                // the forced legal previous crop is proven by the common
                // capture_height/y/top/bottom result below.
                if (dut.capture_height[W_k] !== 12'd1 ||
                    dut.capture_y_q[W_k] !== 12'd32 ||
                    dut.capture_y_r[W_k] !== 12'd0 ||
                    dut.capture_top[W_k] !== CROP_TOP ||
                    dut.capture_bottom[W_k] !== CROP_TOP) begin
                    W_errors = W_errors + 1;
                    $display("REFINE_HEIGHT1_STATE_MISMATCH case=%0d slot=%0d cap_h=%0d y_q=%0d y_r=%0d top=%0d bottom=%0d",
                             CASE_ID, W_k, dut.capture_height[W_k],
                             dut.capture_y_q[W_k], dut.capture_y_r[W_k],
                             dut.capture_top[W_k], dut.capture_bottom[W_k]);
                end
            end
            for (W_y = 0; W_y < 64; W_y = W_y + 1) begin
                for (W_x = 0; W_x < H_ACTIVE; W_x = W_x + 1) begin
                    W_ink_de = 1'b1;
                    W_ink = ink_value(W_x, W_y);
                    @(posedge W_clk);
                end
            end
            W_ink_de = 1'b0;
            W_ink = 1'b0;
            @(posedge W_clk);
            W_vsync = 1'b1;
            @(posedge W_clk);
            repeat (4) @(posedge W_clk);
        end
    endtask

    always @(posedge W_clk) begin
        if (W_tile_write_en) begin
            $display("CAPTRACE_REF case=%0d H=%0d phase=%0d event=WRITE en=1 addr=%03h data=%08h snap=0 drop=0",
                     CASE_ID, H_ACTIVE, W_phase, W_tile_write_addr,
                     W_tile_write_data);
            if (W_tile_write_addr >= 448 || W_seen[W_tile_write_addr])
            begin
                $display("REFINE_HEIGHT1_WRITE_ERROR case=%0d phase=%0d addr=%0d seen=%0d",
                         CASE_ID, W_phase, W_tile_write_addr,
                         (W_tile_write_addr < 448) ? W_seen[W_tile_write_addr] : 1'b0);
                W_errors = W_errors + 1;
            end
            else
                W_seen[W_tile_write_addr] = 1'b1;
            if (W_phase == 1)
                W_first_write_count = W_first_write_count + 1;
            else begin
                W_second_write_count = W_second_write_count + 1;
                if (W_tile_write_addr < 224)
                    W_errors = W_errors + 1;
                else begin
                    W_second_data_by_addr[W_tile_write_addr] = W_tile_write_data;
                    W_k = (W_tile_write_addr - 224) >> 5;
                    W_addr = W_tile_write_addr - 224;
                    if (W_tile_write_data !== golden_refined_word(W_k)) begin
                        $display("REFINE_HEIGHT1_DATA_ERROR case=%0d phase=%0d addr=%0d slot=%0d got=%08h exp=%08h",
                                 CASE_ID, W_phase, W_tile_write_addr, W_k,
                                 W_tile_write_data, golden_refined_word(W_k));
                        W_errors = W_errors + 1;
                    end
                end
            end
        end
        if (W_snapshot_valid) begin
            $display("CAPTRACE_REF case=%0d H=%0d phase=%0d event=SNAPSHOT en=0 addr=xxx data=xxxxxxxx snap=1 drop=0 bank=%0d mask=%02h",
                     CASE_ID, H_ACTIVE, W_phase, W_snapshot_bank,
                     W_snapshot_mask);
            W_snapshot_count = W_snapshot_count + 1;
            if (W_phase == 1) begin
                W_snapshot1_count = W_snapshot1_count + 1;
                W_snapshot1_bank = W_snapshot_bank;
                W_snapshot1_mask = W_snapshot_mask;
            end else begin
                W_snapshot2_count = W_snapshot2_count + 1;
                W_snapshot2_bank = W_snapshot_bank;
                W_snapshot2_mask = W_snapshot_mask;
            end
        end
        if (W_snapshot_drop) begin
            $display("CAPTRACE_REF case=%0d H=%0d phase=%0d event=DROP en=0 addr=xxx data=xxxxxxxx snap=0 drop=1 bank=%0d mask=%02h",
                     CASE_ID, H_ACTIVE, W_phase, W_snapshot_bank,
                     W_snapshot_mask);
            W_errors = W_errors + 1;
        end
    end

    initial begin
        W_errors = 0;
        O_done = 1'b0;
        O_errors = 32'd0;
        O_first_writes = 32'd0;
        O_second_writes = 32'd0;
        O_natural_previous_valid = 32'd0;
        W_phase = 1;
        W_natural_previous_valid = 0;
        W_rst_n = 1'b0;
        W_ink = 1'b0;
        W_ink_de = 1'b0;
        W_vsync = 1'b1;
        W_detected = 1'b1;
        W_char_up = 11'd0;
        W_char_down = BOX_HEIGHT - 1;
        W_char_valid = 7'h7f;
        W_export_busy = 1'b0;
        for (W_k = 0; W_k < 7; W_k = W_k + 1) begin
            W_left[W_k] = slot_left(W_k);
            W_right[W_k] = slot_right(W_k);
        end
        clear_log;
        repeat (5) @(posedge W_clk);
        W_rst_n = 1'b1;
        repeat (3) @(posedge W_clk);
        drive_first_frame;

        W_wait = 0;
        while (W_snapshot1_count == 0 && W_wait < (H_ACTIVE * 64 + 5000)) begin
            @(posedge W_clk);
            W_wait = W_wait + 1;
        end
        #1;
        if (W_snapshot1_count == 0) begin
            $display("REFINE_HEIGHT1_NO_FIRST_SNAPSHOT case=%0d", CASE_ID);
            W_errors = W_errors + 1;
        end
        if (dut.previous_ink_valid !== 7'h7f) begin
            $display("REFINE_HEIGHT1_PREVIOUS_VALID_ERROR case=%0d got=%02h",
                     CASE_ID, dut.previous_ink_valid);
            W_errors = W_errors + 1;
        end
        else
            W_natural_previous_valid = 1;
        if (W_snapshot1_count != 1 || W_snapshot1_bank != 0 ||
            W_snapshot1_mask != 7'h7f || W_first_write_count != 224)
        begin
            $display("REFINE_HEIGHT1_FIRST_SUMMARY_ERROR case=%0d count=%0d bank=%0d mask=%02h writes=%0d",
                     CASE_ID, W_snapshot1_count, W_snapshot1_bank,
                     W_snapshot1_mask, W_first_write_count);
            W_errors = W_errors + 1;
        end

        force_thin_previous_crop;
        W_phase = 2;
        drive_second_frame;

        W_wait = 0;
        while (W_snapshot2_count == 0 && W_wait < (H_ACTIVE * 64 + 5000)) begin
            @(posedge W_clk);
            W_wait = W_wait + 1;
        end
        #1;
        if (W_snapshot2_count == 0) begin
            $display("REFINE_HEIGHT1_NO_SECOND_SNAPSHOT case=%0d", CASE_ID);
            W_errors = W_errors + 1;
        end
        repeat (20) @(posedge W_clk);
        if (W_snapshot2_count != 1 || W_snapshot2_bank != 1 ||
            W_snapshot2_mask != 7'h7f || W_second_write_count != 224)
        begin
            $display("REFINE_HEIGHT1_SECOND_SUMMARY_ERROR case=%0d count=%0d bank=%0d mask=%02h writes=%0d",
                     CASE_ID, W_snapshot2_count, W_snapshot2_bank,
                     W_snapshot2_mask, W_second_write_count);
            W_errors = W_errors + 1;
        end
        for (W_k = 0; W_k < 7; W_k = W_k + 1) begin
            for (W_addr = 0; W_addr < 32; W_addr = W_addr + 1) begin
                if (!W_seen[224 + W_k * 32 + W_addr] ||
                    W_second_data_by_addr[224 + W_k * 32 + W_addr] !==
                    golden_refined_word(W_k))
                begin
                    $display("REFINE_HEIGHT1_FINAL_DATA_ERROR case=%0d slot=%0d row=%0d seen=%0d got=%08h exp=%08h",
                             CASE_ID, W_k, W_addr,
                             W_seen[224 + W_k * 32 + W_addr],
                             W_second_data_by_addr[224 + W_k * 32 + W_addr],
                             golden_refined_word(W_k));
                    W_errors = W_errors + 1;
                end
            end
        end
        release_thin_previous_crop;
        $display("REFINE_HEIGHT1_CASE case=%0d H=%0d natural_previous_valid=%0d first_writes=%0d second_writes=%0d snapshots=%0d/%0d banks=%0d/%0d masks=%02h/%02h errors=%0d",
                 CASE_ID, H_ACTIVE, W_natural_previous_valid,
                 W_first_write_count, W_second_write_count,
                 W_snapshot1_count, W_snapshot2_count,
                 W_snapshot1_bank, W_snapshot2_bank,
                 W_snapshot1_mask, W_snapshot2_mask, W_errors);
        O_errors = W_errors;
        O_first_writes = W_first_write_count;
        O_second_writes = W_second_write_count;
        O_natural_previous_valid = W_natural_previous_valid;
        O_done = 1'b1;
    end
endmodule

module tb_cap_master_tile_capture_refine_height1;
    wire W_done_640;
    wire W_done_1280;
    wire [31:0] W_errors_640;
    wire [31:0] W_errors_1280;
    wire [31:0] W_first_640;
    wire [31:0] W_first_1280;
    wire [31:0] W_second_640;
    wire [31:0] W_second_1280;
    wire [31:0] W_previous_640;
    wire [31:0] W_previous_1280;

    tb_cap_master_tile_capture_refine_height1_case #(.H_ACTIVE(640), .CASE_ID(640))
        u_h640 (.O_done(W_done_640), .O_errors(W_errors_640),
                .O_first_writes(W_first_640), .O_second_writes(W_second_640),
                .O_natural_previous_valid(W_previous_640));
    tb_cap_master_tile_capture_refine_height1_case #(.H_ACTIVE(1280), .CASE_ID(1280))
        u_h1280 (.O_done(W_done_1280), .O_errors(W_errors_1280),
                 .O_first_writes(W_first_1280), .O_second_writes(W_second_1280),
                 .O_natural_previous_valid(W_previous_1280));

    initial begin
        wait (W_done_640 && W_done_1280);
        if ((W_errors_640 + W_errors_1280) == 0)
            $display("PH1P_CAPTURE_REFINE_HEIGHT1_TB_PASS cases=2 errors=0");
        else
            $display("PH1P_CAPTURE_REFINE_HEIGHT1_TB_FAIL h640=%0d h1280=%0d",
                     W_errors_640, W_errors_1280);
        $display("PH1P_CAPTURE_REFINE_HEIGHT1_NATURAL previous_valid_h640=%0d previous_valid_h1280=%0d",
                 W_previous_640, W_previous_1280);
        $finish;
    end
endmodule
