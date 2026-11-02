//=============================================================================
// tb_alg_vehicle_services -- service wrapper smoke and contract test
//-----------------------------------------------------------------------------
//  所属域   : alg
//  时钟域   : I_clk_pix, I_clk_24m
//  复位     : I_rst_n，异步低有效
//  参数     : disabled TF model for service/video integration test
//  验证状态 : 仿真 通过（Questa FSE 2025.2） | 综合 未做 | 布线 未做 | 上板 未做
//=============================================================================

module tb_alg_vehicle_services;

    reg         clk_pix;
    reg         clk_24m;
    reg         rst_n;
    reg [23:0]  rgb;
    reg         de;
    reg         user;
    reg         last;
    reg         bbox_frame;
    reg         bbox_valid;
    reg [11:0]  bbox_x0;
    reg [11:0]  bbox_y0;
    reg [11:0]  bbox_x1;
    reg [11:0]  bbox_y1;
    reg [55:0]  ocr_plate;
    reg [3:0]   ocr_res_n;
    reg         ocr_valid;
    reg         ocr_frame;
    reg         track_enable;
    reg         jog_enable;
    reg         jog_left;
    reg         jog_right;
    reg         jog_up;
    reg         jog_down;
    reg         home;
    reg         tf_key_n;
    reg         tf_miso;

    wire [23:0] rgb_out;
    wire        de_out;
    wire        user_out;
    wire        last_out;
    wire [15:0] pan_angle;
    wire [15:0] tilt_angle;
    wire        track_locked;
    wire [11:0] track_cx;
    wire [11:0] track_cy;
    wire [19:0] dist_mm;
    wire [19:0] speed_mmps;
    wire [11:0] plate_w_px;
    wire [7:0]  track_state;
    wire        metrics_valid;
    wire        tf_clk;
    wire        tf_cs_n;
    wire        tf_mosi;
    wire [55:0] tf_plate;
    wire        tf_plate_valid;
    wire        tf_card_ready;
    wire [1:0]  tf_status;
    wire        led_plate;
    wire [55:0] plate_osd;
    wire [3:0]  plate_osd_n;
    wire        plate_osd_valid;

    integer errors;
    integer px;
    integer video_mismatches;

    alg_vehicle_services #(
        .IMG_WIDTH            (1280),
        .IMG_HEIGHT           (720),
        .PIXEL_CLK_HZ         (1000000),
        .SERVO_PERIOD_US      (20),
        .SERVO_MIN_US         (1),
        .SERVO_MAX_US         (3),
        .GIMBAL_ACQ_FRAMES   (1),
        .GIMBAL_LOST_FRAMES  (2),
        .ENABLE_TF           (0)
    ) dut (
        .I_clk_pix          (clk_pix),
        .I_clk_24m          (clk_24m),
        .I_rst_n            (rst_n),
        .I_rgb              (rgb),
        .I_de               (de),
        .I_user             (user),
        .I_last             (last),
        .I_bbox_frame       (bbox_frame),
        .I_bbox_valid       (bbox_valid),
        .I_bbox_x0          (bbox_x0),
        .I_bbox_y0          (bbox_y0),
        .I_bbox_x1          (bbox_x1),
        .I_bbox_y1          (bbox_y1),
        .I_ocr_plate        (ocr_plate),
        .I_ocr_res_n        (ocr_res_n),
        .I_ocr_valid        (ocr_valid),
        .I_ocr_frame        (ocr_frame),
        .I_track_enable     (track_enable),
        .I_jog_enable       (jog_enable),
        .I_jog_left         (jog_left),
        .I_jog_right        (jog_right),
        .I_jog_up           (jog_up),
        .I_jog_down         (jog_down),
        .I_home             (home),
        .I_tf_key_n         (tf_key_n),
        .I_tf_miso          (tf_miso),
        .O_rgb              (rgb_out),
        .O_de               (de_out),
        .O_user             (user_out),
        .O_last             (last_out),
        .O_pan_angle        (pan_angle),
        .O_tilt_angle       (tilt_angle),
        .O_track_locked     (track_locked),
        .O_track_cx         (track_cx),
        .O_track_cy         (track_cy),
        .O_dist_mm          (dist_mm),
        .O_speed_mmps       (speed_mmps),
        .O_plate_w_px       (plate_w_px),
        .O_track_state       (track_state),
        .O_metrics_valid    (metrics_valid),
        .O_tf_clk           (tf_clk),
        .O_tf_cs_n          (tf_cs_n),
        .O_tf_mosi          (tf_mosi),
        .O_tf_plate         (tf_plate),
        .O_tf_plate_valid   (tf_plate_valid),
        .O_tf_card_ready    (tf_card_ready),
        .O_tf_status        (tf_status),
        .O_led_plate        (led_plate),
        .O_plate_osd        (plate_osd),
        .O_plate_osd_n      (plate_osd_n),
        .O_plate_osd_valid  (plate_osd_valid)
    );

    always #5 clk_pix = ~clk_pix;
    always #11 clk_24m = ~clk_24m;

    task pix_tick;
        begin
            @(posedge clk_pix);
            #1;
        end
    endtask

    task send_bbox;
        input        vld;
        input [11:0] x0;
        input [11:0] y0;
        input [11:0] x1;
        input [11:0] y1;
        begin
            bbox_valid = vld;
            bbox_x0 = x0;
            bbox_y0 = y0;
            bbox_x1 = x1;
            bbox_y1 = y1;
            bbox_frame = 1'b1;
            pix_tick;
            bbox_frame = 1'b0;
            pix_tick;
        end
    endtask

    initial begin
        clk_pix = 1'b0;
        clk_24m = 1'b0;
        rst_n = 1'b0;
        rgb = 24'h123456;
        de = 1'b0;
        user = 1'b0;
        last = 1'b1;
        bbox_frame = 1'b0;
        bbox_valid = 1'b0;
        bbox_x0 = 12'd0;
        bbox_y0 = 12'd0;
        bbox_x1 = 12'd0;
        bbox_y1 = 12'd0;
        ocr_plate = 56'h37363534333231;
        ocr_res_n = 4'd7;
        ocr_valid = 1'b0;
        ocr_frame = 1'b0;
        track_enable = 1'b0;
        jog_enable = 1'b0;
        jog_left = 1'b0;
        jog_right = 1'b0;
        jog_up = 1'b0;
        jog_down = 1'b0;
        home = 1'b0;
        tf_key_n = 1'b1;
        tf_miso = 1'b1;
        errors = 0;

        repeat (4) pix_tick;
        rst_n = 1'b1;
        repeat (3) pix_tick;

        // Contract check: video controls and last pass with no pixel delay.
        if ((rgb_out !== rgb) || (de_out !== de) ||
            (user_out !== user) || (last_out !== last)) begin
            $display("FAIL: zero-latency timing pass");
            errors = errors + 1;
        end
        if ((tf_cs_n !== 1'b1) || (tf_mosi !== 1'b1) || (tf_clk !== 1'b0)) begin
            $display("FAIL: disabled TF idle levels");
            errors = errors + 1;
        end

        // OCR result is latched at a result event, then made frame coherent at SOF.
        ocr_valid = 1'b1;
        ocr_frame = 1'b1;
        user = 1'b1;
        de = 1'b1;
        pix_tick;
        ocr_frame = 1'b0;
        user = 1'b0;
        de = 1'b0;
        pix_tick;
        if (!plate_osd_valid || (plate_osd !== ocr_plate) ||
            (plate_osd_n !== ocr_res_n)) begin
            $display("FAIL: frame-coherent OCR mailbox");
            errors = errors + 1;
        end

        // C14's first province class is 8'h80.  Exercise the production
        // province/alphanumeric code space and the {char6,...,char0}
        // packing used by both TF records and osd_char_lib; this is a
        // contract check, not an assertion that an arbitrary mock is OCR.
        ocr_plate = {8'h37, 8'h36, 8'h35, 8'h34, 8'h33, 8'h32, 8'h80};
        ocr_frame = 1'b1;
        user = 1'b1;
        pix_tick;
        ocr_frame = 1'b0;
        user = 1'b0;
        pix_tick;
        if (!plate_osd_valid || (plate_osd !== ocr_plate) ||
            (plate_osd[7:0] !== 8'h80)) begin
            $display("FAIL: C14 province-code packing");
            errors = errors + 1;
        end

        // Automatic bbox event drives both metrics and one-frame gimbal lock.
        track_enable = 1'b1;
        send_bbox(1'b1, 12'd700, 12'd300, 12'd800, 12'd360);
        repeat (24) pix_tick;
        // plate_metrics seeds its width IIR on the first valid frame and
        // reports the seeded width on the next valid frame.
        send_bbox(1'b1, 12'd700, 12'd300, 12'd800, 12'd360);
        repeat (32) begin
            pix_tick;
        end
        if (!track_locked) begin
            $display("FAIL: gimbal did not acquire a one-frame target");
            errors = errors + 1;
        end
        if (!metrics_valid || (plate_w_px == 12'd0) || (dist_mm == 20'd0)) begin
            $display("FAIL: metrics result valid=%0d width=%0d dist=%0d",
                     metrics_valid, plate_w_px, dist_mm);
            errors = errors + 1;
        end

        // A complete active raster must stay unchanged even with valid OCR
        // and a tracked bbox. This traverses the former crosshair, parking,
        // tracking-box and plate-text regions, so an accidental overlay fails.
        video_mismatches = 0;
        de = 1'b1;
        for (px = 0; px < 1280*720; px = px + 1) begin
            @(negedge clk_pix);
            user = (px == 0);
            last = ((px % 1280) == 1279);
            rgb = 24'h123456 ^ px;
            #1;
            if (rgb_out !== rgb || de_out !== de ||
                user_out !== user || last_out !== last)
                video_mismatches = video_mismatches + 1;
        end
        if (video_mismatches != 0) begin
            $display("FAIL: service overlay remained pixels=%0d", video_mismatches);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("TB_RESULT: PASS");
        else
            $display("TB_RESULT: FAIL errors=%0d", errors);
        $finish;
    end

endmodule

