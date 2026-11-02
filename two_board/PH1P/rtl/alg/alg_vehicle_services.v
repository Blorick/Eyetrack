//=============================================================================
// alg_vehicle_services -- vehicle algorithm and service integration wrapper
//-----------------------------------------------------------------------------
//  所属域   : alg
//  时钟域   : I_clk_pix, I_clk_24m
//  复位     : I_rst_n 异步断言；24M/像素域分别两拍释放
//  参数     : 1280x720 video, calibrated plate metrics, gimbal and TF services
//  验证状态 : 仿真 通过 | 综合 未做 | 布线 未做 | 上板 未做
//-----------------------------------------------------------------------------
//  The PH1P top supplies the mature v2.4 plate_locate result as a frame-tagged
//  bounding box.  This wrapper deliberately does not instantiate a second
//  blue-plate detector.  It owns the tracking, distance/speed,
//  gimbal and TF-card paths; displayed pixels pass through unchanged.
//
//  I_bbox_* and I_ocr_* are already synchronized to I_clk_pix by the caller.
//  I_clk_24m is used only by the TF-card path; I_tf_key_n is low active.
//  The pixel path is combinational: O_rgb corresponds to I_rgb in the same
//  cycle and O_de/O_user/O_last are direct timing passes.
//=============================================================================
module alg_vehicle_services #(
    parameter integer IMG_WIDTH            = 1280,
    parameter integer IMG_HEIGHT           = 720,
    parameter integer PIXEL_CLK_HZ         = 75000000,
    parameter integer PLATE_W_MM           = 68,
    parameter integer FOCAL_PX             = 1676,
    parameter integer SERVO_PERIOD_US     = 20000,
    parameter integer SERVO_MIN_US        = 500,
    parameter integer SERVO_MAX_US        = 2500,
    parameter integer SERVO_ANGLE_MAX     = 1000,
    parameter integer GIMBAL_ACQ_FRAMES   = 3,
    parameter integer GIMBAL_LOST_FRAMES  = 20,
    parameter integer ENABLE_TF           = 1
)(
    // ---- clocks and reset -------------------------------------------------
    input  wire        I_clk_pix,
    input  wire        I_clk_24m,
    input  wire        I_rst_n,

    // ---- raw displayed video; output timing is unchanged ------------------
    input  wire [23:0] I_rgb,
    input  wire        I_de,
    input  wire        I_user,
    input  wire        I_last,

    // ---- v2.4 plate_locate frame result, already in I_clk_pix -------------
    input  wire        I_bbox_frame,
    input  wire        I_bbox_valid,
    input  wire [11:0] I_bbox_x0,
    input  wire [11:0] I_bbox_y0,
    input  wire [11:0] I_bbox_x1,
    input  wire [11:0] I_bbox_y1,

    // ---- completed EG4D OCR result ----------------------------------------
    // C14 char_match, ctl_plate_card and osd_char_lib share the packed
    // {char6,...,char0} contract.  The trained C14 constants use the same
    // codes consumed by the font (province 80h..A7h, digits 30h..39h and
    // letters 41h..5Ah); no character-code conversion is inserted here.
    input  wire [55:0] I_ocr_plate,
    input  wire [3:0]  I_ocr_res_n,
    input  wire        I_ocr_valid,
    input  wire        I_ocr_frame,

    // ---- pixel-domain tracking controls, high active -----------------------
    input  wire        I_track_enable,
    input  wire        I_jog_enable,
    input  wire        I_jog_left,
    input  wire        I_jog_right,
    input  wire        I_jog_up,
    input  wire        I_jog_down,
    input  wire        I_home,

    // ---- 24M-domain TF-card control, low-active key ------------------------
    input  wire        I_tf_key_n,
    input  wire        I_tf_miso,

    // ---- service video and timing ------------------------------------------
    output wire [23:0] O_rgb,
    output wire        O_de,
    output wire        O_user,
    output wire        O_last,

    // ---- tracking / distance / speed ---------------------------------------
    output wire [15:0] O_pan_angle,
    output wire [15:0] O_tilt_angle,
    output wire        O_track_locked,
    output wire [11:0] O_track_cx,
    output wire [11:0] O_track_cy,
    output wire [19:0] O_dist_mm,
    output wire [19:0] O_speed_mmps,
    output wire [11:0] O_plate_w_px,
    output wire [7:0]  O_track_state,
    output wire        O_metrics_valid,

    // ---- TF-card SPI and status --------------------------------------------
    output wire        O_tf_clk,
    output wire        O_tf_cs_n,
    output wire        O_tf_mosi,
    output wire [55:0] O_tf_plate,
    output wire        O_tf_plate_valid,
    output wire        O_tf_card_ready,
    output wire [1:0]  O_tf_status,
    output wire        O_led_plate,

    // ---- frame-coherent OCR copy for service consumers ---------
    output wire [55:0] O_plate_osd,
    output wire [3:0]  O_plate_osd_n,
    output wire        O_plate_osd_valid
);

    // I_rst_n is shared by the independent pixel and TF clocks.  Keep the
    // asynchronous assertion, but release each clock domain locally so a
    // partially released display or TF state machine cannot observe an
    // unrelated clock edge.  The legacy TF protocol remains unchanged; its
    // controller simply receives the local 24M reset below.
    reg [1:0] S_release_pix;
    reg [1:0] S_release_24m;
    wire      W_rst_pix_n = S_release_pix[1];
    wire      W_rst_24m_n = S_release_24m[1];

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n)
            S_release_pix <= 2'b00;
        else
            S_release_pix <= {S_release_pix[0], 1'b1};
    end

    always @(posedge I_clk_24m or negedge I_rst_n) begin
        if (!I_rst_n)
            S_release_24m <= 2'b00;
        else
            S_release_24m <= {S_release_24m[0], 1'b1};
    end

    // ---- frame-coherent OCR mailbox ---------------------------------------
    reg [55:0] S_ocr_pending;
    reg [3:0]  S_ocr_pending_n;
    reg        S_ocr_pending_valid;
    reg [55:0] S_ocr_active;
    reg [3:0]  S_ocr_active_n;
    reg        S_ocr_active_valid;

    always @(posedge I_clk_pix or negedge W_rst_pix_n) begin
        if (!W_rst_pix_n) begin
            S_ocr_pending       <= 56'd0;
            S_ocr_pending_n     <= 4'd0;
            S_ocr_pending_valid <= 1'b0;
            S_ocr_active        <= 56'd0;
            S_ocr_active_n      <= 4'd0;
            S_ocr_active_valid  <= 1'b0;
        end else begin
            if (I_ocr_frame) begin
                S_ocr_pending       <= I_ocr_plate;
                S_ocr_pending_n     <= I_ocr_res_n;
                S_ocr_pending_valid <= I_ocr_valid;
            end
            if (I_user) begin
                if (I_ocr_frame) begin
                    S_ocr_active       <= I_ocr_plate;
                    S_ocr_active_n     <= I_ocr_res_n;
                    S_ocr_active_valid <= I_ocr_valid;
                end else begin
                    S_ocr_active       <= S_ocr_pending;
                    S_ocr_active_n     <= S_ocr_pending_n;
                    S_ocr_active_valid <= S_ocr_pending_valid;
                end
            end
        end
    end

    assign O_plate_osd       = S_ocr_active;
    assign O_plate_osd_n     = S_ocr_active_n;
    assign O_plate_osd_valid = S_ocr_active_valid;

    // ---- distance and signed image velocity -------------------------------
    wire [19:0] W_dist_mm;
    wire [19:0] W_speed_mmps;
    wire [11:0] W_plate_w_px;
    wire [11:0] W_metrics_cx;
    wire [11:0] W_metrics_cy;
    wire [7:0]  W_track_state;
    wire        W_metrics_valid;
    wire signed [15:0] W_vx_q3;
    wire signed [15:0] W_vy_q3;

    plate_metrics #(
        .IMG_W          ( IMG_WIDTH  ),
        .IMG_H          ( IMG_HEIGHT ),
        .PLATE_W_MM     ( PLATE_W_MM ),
        .FOCAL_PX       ( FOCAL_PX   )
    ) u_plate_metrics (
        .I_clk          ( I_clk_pix      ),
        .I_rst_n        ( W_rst_pix_n    ),
        .I_frame        ( I_bbox_frame   ),
        .I_valid        ( I_bbox_valid   ),
        .I_x0           ( I_bbox_x0      ),
        .I_y0           ( I_bbox_y0      ),
        .I_x1           ( I_bbox_x1      ),
        .I_y1           ( I_bbox_y1      ),
        .O_dist_mm      ( W_dist_mm      ),
        .O_speed_mmps   ( W_speed_mmps   ),
        .O_w_px         ( W_plate_w_px   ),
        .O_cx           ( W_metrics_cx   ),
        .O_cy           ( W_metrics_cy   ),
        .O_state        ( W_track_state  ),
        .O_valid        ( W_metrics_valid),
        .O_vx_q3        ( W_vx_q3        ),
        .O_vy_q3        ( W_vy_q3        )
    );

    assign O_dist_mm       = W_dist_mm;
    assign O_speed_mmps    = W_speed_mmps;
    assign O_plate_w_px    = W_plate_w_px;
    assign O_track_cx     = W_metrics_cx;
    assign O_track_cy     = W_metrics_cy;
    assign O_track_state  = W_track_state;
    assign O_metrics_valid= W_metrics_valid;

    // ---- one-frame-rate proportional gimbal control -----------------------
    gimbal_ctrl #(
        .IMG_W         ( IMG_WIDTH          ),
        .IMG_H         ( IMG_HEIGHT         ),
        .ACQ_FRAMES    ( GIMBAL_ACQ_FRAMES  ),
        .LOST_FRAMES   ( GIMBAL_LOST_FRAMES ),
        .LEAD_FRAMES   ( 0                  ),
        .COAST_ENABLE  ( 1                  ),
        .PAN_INVERT    ( 0                  ),  // 左右方向：1 实测仍反，改回 0
        .MAX_STEP      ( 2                  ),  // pan 越小越好（约 0.36°/帧）
        .KP_SHIFT      ( 6                  ),  // pan 增益更柔：err >>> 6
        .MAX_STEP_T    ( 2                  ),  // tilt 越慢越好（约 0.36°/帧）
        .KP_SHIFT_T    ( 6                  )   // tilt 增益更柔：err >>> 6
    ) u_gimbal_ctrl (
        .I_clk      ( I_clk_pix       ),
        .I_rst_n    ( W_rst_pix_n    ),
        .I_frame    ( I_bbox_frame    ),
        .I_valid    ( I_bbox_valid    ),
        .I_x0       ( I_bbox_x0       ),
        .I_y0       ( I_bbox_y0       ),
        .I_x1       ( I_bbox_x1       ),
        .I_y1       ( I_bbox_y1       ),
        .I_vx_q3    ( W_vx_q3         ),
        .I_vy_q3    ( W_vy_q3         ),
        .I_enable   ( I_track_enable  ),
        .I_jog_en   ( I_jog_enable    ),
        .I_jog_l    ( I_jog_left      ),
        .I_jog_r    ( I_jog_right     ),
        .I_jog_u    ( I_jog_up        ),
        .I_jog_d    ( I_jog_down      ),
        .I_home     ( I_home          ),
        .O_pan      ( O_pan_angle     ),
        .O_tilt     ( O_tilt_angle    ),
        .O_locked   ( O_track_locked  ),
        .O_cx       (                 ),
        .O_cy       (                 )
    );

    // ---- TF card chain; it is optional only for small service simulations --
    wire [55:0] W_tf_plate;
    wire        W_tf_plate_valid;
    wire        W_tf_card_ready;
    wire [1:0]  W_tf_status;

    generate
        if (ENABLE_TF != 0) begin : G_TF_ENABLED
            ctl_plate_card u_ctl_plate_card (
                .I_clk         ( I_clk_24m       ),
                .I_rst_n       ( W_rst_24m_n     ),
                .I_key         ( I_tf_key_n      ),
                .O_tf_clk      ( O_tf_clk        ),
                .O_tf_cs_n     ( O_tf_cs_n        ),
                .O_tf_mosi     ( O_tf_mosi       ),
                .I_tf_miso     ( I_tf_miso       ),
                .O_plate       ( W_tf_plate       ),
                .O_plate_valid ( W_tf_plate_valid ),
                .O_card_ready  ( W_tf_card_ready  ),
                .O_status      ( W_tf_status      )
            );
        end else begin : G_TF_DISABLED
            assign O_tf_clk       = 1'b0;
            assign O_tf_cs_n      = 1'b1;
            assign O_tf_mosi      = 1'b1;
            assign W_tf_plate     = 56'd0;
            assign W_tf_plate_valid=1'b0;
            assign W_tf_card_ready =1'b0;
            assign W_tf_status     = 2'd0;
        end
    endgenerate

    assign O_tf_plate       = W_tf_plate;
    assign O_tf_plate_valid = W_tf_plate_valid;
    assign O_tf_card_ready  = W_tf_card_ready;
    assign O_tf_status      = W_tf_status;
    assign O_led_plate      = (W_tf_status == 2'd1) && W_tf_plate_valid;

    // OSD composition is owned by hdmi_mixer and the final detection boxes.
    assign O_rgb  = I_rgb;
    assign O_de   = I_de;
    assign O_user = I_user;
    assign O_last = I_last;

endmodule

