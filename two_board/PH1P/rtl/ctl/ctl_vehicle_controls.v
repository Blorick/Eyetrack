//=============================================================================
// ctl_vehicle_controls -- camera and gimbal control-page arbiter
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk_24m input controls; I_clk_pix service outputs
//  复位     : I_rst_n 异步断言；24M/像素域分别两拍释放
//  参数     : debounce/long-press/TF pulse intervals are in I_clk_24m cycles
//  验证状态 : 仿真 通过（2026-10-26 Questa 2025.2） | 综合/布线/板测见工程记录
//-----------------------------------------------------------------------------
//  KEY2 is the page key.  A short action is emitted only on release when the
//  long-press threshold was not reached.  Pages are camera=0 and
//  gimbal=2; KEY2 long press toggles directly between them. Camera controls
//  are exported in the 24M domain so the existing
//  AE/AG and image-mode logic can be gated without changing board wiring.
//
//  Service button actions are event toggles in the 24M domain.  The pixel side
//  synchronizes each toggle through two registers and emits one-cycle pulses.
//  A same-channel event must be separated by at least the debouncer interval
//  (DEBOUNCE_CYCLES source clocks); the debounced key/switch edge therefore
//  cannot create two transitions inside the two-pixel-clock synchronizer
//  aperture.  I_frame_pulse remains a one-cycle pixel event and snapshots are
//  committed only at that boundary.
//  Debounced profile/switch/key levels are carried in a stable snapshot bus;
//  the pixel side commits that bus only at I_frame_pulse.  I_frame_pulse must
//  be the one-pulse rising-edge form of the display-frame vsync, while I_user
//  remains the video timing frame-start input to alg_vehicle_services.
//
//  SW1/SW3/SW4 retain the existing SW_ACTIVE_LEVEL convention.
//  SW2 is the plate-display switch: physical up=1, down=0.
//=============================================================================
module ctl_vehicle_controls #(
    parameter integer DEBOUNCE_CYCLES  = 500000,
    parameter integer LONG_HOLD_CYCLES = 12000000,
    parameter integer TF_PULSE_CYCLES  = 480000,
    parameter         SW_ACTIVE_LEVEL  = 1'b0
)(
    // ---- clocks and reset -------------------------------------------------
    input  wire       I_clk_24m,
    input  wire       I_clk_pix,
    input  wire       I_rst_n,
    input  wire       I_frame_pulse,

    // ---- board controls: keys active low; SW2 display up=1, other switches use parameter
    input  wire       I_key1_n,
    input  wire       I_key2_n,
    input  wire       I_key3_n,
    input  wire       I_key4_n,
    input  wire       I_sw1,
    input  wire       I_sw2,            // plate-number display enable, high when up
    input  wire       I_sw3,
    input  wire       I_sw4,

    // ---- 24M camera-page controls ----------------------------------------
    output wire [1:0] O_profile_24m,
    output wire       O_camera_key1_n,
    output reg        O_camera_key2_short,
    output wire       O_camera_ae_up,
    output wire       O_camera_ae_dn,
    output wire       O_camera_ag_up,
    output wire       O_camera_ag_dn,

    // ---- 24M TF-card control ---------------------------------------------
    output wire       O_tf_key_n,

    // ---- pixel-domain service levels and one-cycle events -----------------
    output reg  [1:0] O_profile_pix,
    output reg        O_plate_display_enable,
    output reg        O_track_enable,
    output reg        O_jog_enable,
    output reg        O_jog_left,
    output reg        O_jog_right,
    output reg        O_jog_up,
    output reg        O_jog_down,
    output reg        O_gimbal_home
);

    localparam SW_IDLE_LEVEL = (SW_ACTIVE_LEVEL == 1'b0) ? 1'b1 : 1'b0;
    localparam integer LONG_TRIGGER =
        (LONG_HOLD_CYCLES < 1) ? 0 : (LONG_HOLD_CYCLES - 1);

    // The top-level reset is shared by unrelated clocks.  Assert it
    // asynchronously, then release each domain through two local cycles.
    reg [1:0] S_release_24m;
    reg [1:0] S_release_pix;
    wire      W_rst_24m_n = S_release_24m[1];
    wire      W_rst_pix_n = S_release_pix[1];

    always @(posedge I_clk_24m or negedge I_rst_n) begin
        if (!I_rst_n)
            S_release_24m <= 2'b00;
        else
            S_release_24m <= {S_release_24m[0], 1'b1};
    end

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n)
            S_release_pix <= 2'b00;
        else
            S_release_pix <= {S_release_pix[0], 1'b1};
    end

    // ---- asynchronous input synchronizers --------------------------------
    reg [2:0] S_key1_sync;
    reg [2:0] S_key2_sync;
    reg [2:0] S_key3_sync;
    reg [2:0] S_key4_sync;
    reg [2:0] S_sw1_sync;
    reg [2:0] S_sw2_sync;
    reg [2:0] S_sw3_sync;
    reg [2:0] S_sw4_sync;

    always @(posedge I_clk_24m or negedge W_rst_24m_n) begin
        if (!W_rst_24m_n) begin
            S_key1_sync <= 3'b111;
            S_key2_sync <= 3'b111;
            S_key3_sync <= 3'b111;
            S_key4_sync <= 3'b111;
            S_sw1_sync  <= {3{SW_IDLE_LEVEL}};
            S_sw2_sync  <= 3'b000;
            S_sw3_sync  <= {3{SW_IDLE_LEVEL}};
            S_sw4_sync  <= {3{SW_IDLE_LEVEL}};
        end
        else begin
            S_key1_sync <= {S_key1_sync[1:0], I_key1_n};
            S_key2_sync <= {S_key2_sync[1:0], I_key2_n};
            S_key3_sync <= {S_key3_sync[1:0], I_key3_n};
            S_key4_sync <= {S_key4_sync[1:0], I_key4_n};
            S_sw1_sync  <= {S_sw1_sync[1:0], I_sw1};
            S_sw2_sync  <= {S_sw2_sync[1:0], I_sw2};
            S_sw3_sync  <= {S_sw3_sync[1:0], I_sw3};
            S_sw4_sync  <= {S_sw4_sync[1:0], I_sw4};
        end
    end

    // ---- debounced levels -------------------------------------------------
    reg        S_key1_clean;
    reg        S_key2_clean;
    reg        S_key3_clean;
    reg        S_key4_clean;
    reg        S_sw1_clean;
    reg        S_sw2_clean;
    reg        S_sw3_clean;
    reg        S_sw4_clean;
    reg [31:0] S_key1_db_cnt;
    reg [31:0] S_key2_db_cnt;
    reg [31:0] S_key3_db_cnt;
    reg [31:0] S_key4_db_cnt;
    reg [31:0] S_sw1_db_cnt;
    reg [31:0] S_sw2_db_cnt;
    reg [31:0] S_sw3_db_cnt;
    reg [31:0] S_sw4_db_cnt;

    always @(posedge I_clk_24m or negedge W_rst_24m_n) begin
        if (!W_rst_24m_n) begin
            S_key1_clean <= 1'b1;
            S_key2_clean <= 1'b1;
            S_key3_clean <= 1'b1;
            S_key4_clean <= 1'b1;
            S_sw1_clean  <= SW_IDLE_LEVEL;
            S_sw2_clean  <= 1'b0;
            S_sw3_clean  <= SW_IDLE_LEVEL;
            S_sw4_clean  <= SW_IDLE_LEVEL;
            S_key1_db_cnt <= 32'd0;
            S_key2_db_cnt <= 32'd0;
            S_key3_db_cnt <= 32'd0;
            S_key4_db_cnt <= 32'd0;
            S_sw1_db_cnt  <= 32'd0;
            S_sw2_db_cnt  <= 32'd0;
            S_sw3_db_cnt  <= 32'd0;
            S_sw4_db_cnt  <= 32'd0;
        end
        else begin
            if (S_key1_sync[2] == S_key1_clean) S_key1_db_cnt <= 32'd0;
            else if (S_key1_db_cnt >= DEBOUNCE_CYCLES) begin
                S_key1_clean <= S_key1_sync[2];
                S_key1_db_cnt <= 32'd0;
            end
            else S_key1_db_cnt <= S_key1_db_cnt + 32'd1;

            if (S_key2_sync[2] == S_key2_clean) S_key2_db_cnt <= 32'd0;
            else if (S_key2_db_cnt >= DEBOUNCE_CYCLES) begin
                S_key2_clean <= S_key2_sync[2];
                S_key2_db_cnt <= 32'd0;
            end
            else S_key2_db_cnt <= S_key2_db_cnt + 32'd1;

            if (S_key3_sync[2] == S_key3_clean) S_key3_db_cnt <= 32'd0;
            else if (S_key3_db_cnt >= DEBOUNCE_CYCLES) begin
                S_key3_clean <= S_key3_sync[2];
                S_key3_db_cnt <= 32'd0;
            end
            else S_key3_db_cnt <= S_key3_db_cnt + 32'd1;

            if (S_key4_sync[2] == S_key4_clean) S_key4_db_cnt <= 32'd0;
            else if (S_key4_db_cnt >= DEBOUNCE_CYCLES) begin
                S_key4_clean <= S_key4_sync[2];
                S_key4_db_cnt <= 32'd0;
            end
            else S_key4_db_cnt <= S_key4_db_cnt + 32'd1;

            if (S_sw1_sync[2] == S_sw1_clean) S_sw1_db_cnt <= 32'd0;
            else if (S_sw1_db_cnt >= DEBOUNCE_CYCLES) begin
                S_sw1_clean <= S_sw1_sync[2];
                S_sw1_db_cnt <= 32'd0;
            end
            else S_sw1_db_cnt <= S_sw1_db_cnt + 32'd1;

            if (S_sw2_sync[2] == S_sw2_clean) S_sw2_db_cnt <= 32'd0;
            else if (S_sw2_db_cnt >= DEBOUNCE_CYCLES) begin
                S_sw2_clean <= S_sw2_sync[2];
                S_sw2_db_cnt <= 32'd0;
            end
            else S_sw2_db_cnt <= S_sw2_db_cnt + 32'd1;

            if (S_sw3_sync[2] == S_sw3_clean) S_sw3_db_cnt <= 32'd0;
            else if (S_sw3_db_cnt >= DEBOUNCE_CYCLES) begin
                S_sw3_clean <= S_sw3_sync[2];
                S_sw3_db_cnt <= 32'd0;
            end
            else S_sw3_db_cnt <= S_sw3_db_cnt + 32'd1;

            if (S_sw4_sync[2] == S_sw4_clean) S_sw4_db_cnt <= 32'd0;
            else if (S_sw4_db_cnt >= DEBOUNCE_CYCLES) begin
                S_sw4_clean <= S_sw4_sync[2];
                S_sw4_db_cnt <= 32'd0;
            end
            else S_sw4_db_cnt <= S_sw4_db_cnt + 32'd1;
        end
    end

    // ---- accepted edge/level forms ---------------------------------------
    reg S_key2_clean_d;
    reg S_sw1_active_d;
    reg S_sw4_active_d;

    wire W_key1_down = ~S_key1_clean;
    wire W_key2_down = ~S_key2_clean;
    wire W_key3_down = ~S_key3_clean;
    wire W_key4_down = ~S_key4_clean;
    wire W_sw1_active = (S_sw1_clean == SW_ACTIVE_LEVEL);
    wire W_sw2_active = S_sw2_clean;
    wire W_sw3_active = (S_sw3_clean == SW_ACTIVE_LEVEL);
    wire W_sw4_active = (S_sw4_clean == SW_ACTIVE_LEVEL);
    wire W_key2_release = !S_key2_clean_d && S_key2_clean;
    wire W_sw1_rise = !S_sw1_active_d && W_sw1_active;
    wire W_sw4_rise = !S_sw4_active_d && W_sw4_active;

    always @(posedge I_clk_24m or negedge W_rst_24m_n) begin
        if (!W_rst_24m_n) begin
            S_key2_clean_d <= 1'b1;
            S_sw1_active_d <= 1'b0;
            S_sw4_active_d <= 1'b0;
        end
        else begin
            S_key2_clean_d <= S_key2_clean;
            S_sw1_active_d <= W_sw1_active;
            S_sw4_active_d <= W_sw4_active;
        end
    end

    // ---- camera page levels and KEY2 page/short handling ------------------
    reg [1:0]  S_profile_24m;
    reg [31:0] S_key2_hold_cnt;
    reg        S_key2_long_done;
    reg [31:0] S_tf_pulse_cnt;

    // Service events are toggles so a slow pixel clock cannot miss a short
    // 24M pulse.  They are synchronized independently below.
    reg S_home_toggle_24m;

    always @(posedge I_clk_24m or negedge W_rst_24m_n) begin
        if (!W_rst_24m_n) begin
            S_profile_24m     <= 2'd0;
            S_key2_hold_cnt   <= 32'd0;
            S_key2_long_done  <= 1'b0;
            S_tf_pulse_cnt    <= 32'd0;
            S_home_toggle_24m  <= 1'b0;
            O_camera_key2_short <= 1'b0;
        end
        else begin
            O_camera_key2_short <= 1'b0;

            if (S_tf_pulse_cnt != 0)
                S_tf_pulse_cnt <= S_tf_pulse_cnt - 32'd1;

            if (W_key2_down) begin
                if (S_key2_hold_cnt < LONG_HOLD_CYCLES)
                    S_key2_hold_cnt <= S_key2_hold_cnt + 32'd1;
                if (!S_key2_long_done &&
                    (S_key2_hold_cnt >= LONG_TRIGGER)) begin
                    S_key2_long_done <= 1'b1;
                    if (S_profile_24m == 2'd2)
                        S_profile_24m <= 2'd0;
                    else
                        S_profile_24m <= 2'd2;
                end
            end
            else begin
                S_key2_hold_cnt <= 32'd0;
                if (W_key2_release && !S_key2_long_done) begin
                    if (S_profile_24m == 2'd0)
                        O_camera_key2_short <= 1'b1;
                end
                S_key2_long_done <= 1'b0;
            end

            // The remaining service page uses SW1's activation edge to read TF.
            if ((S_profile_24m == 2'd2) && W_sw1_rise)
                S_tf_pulse_cnt <= TF_PULSE_CYCLES;

            if ((S_profile_24m == 2'd2) && W_sw4_rise)
                S_home_toggle_24m <= ~S_home_toggle_24m;
        end
    end

    assign O_profile_24m  = S_profile_24m;
    assign O_tf_key_n     = (S_tf_pulse_cnt == 0) ? 1'b1 : 1'b0;
    assign O_camera_key1_n = (S_profile_24m == 2'd0) ? S_key1_clean : 1'b1;
    assign O_camera_ae_up = (S_profile_24m == 2'd0) && W_key3_down && !W_key4_down;
    assign O_camera_ae_dn = (S_profile_24m == 2'd0) && W_key4_down;
    // SW2 is reserved for plate display and no longer changes or blocks AG.
    assign O_camera_ag_up = (S_profile_24m == 2'd0) && W_sw1_active;
    assign O_camera_ag_dn = 1'b0;

    // ---- stable profile/switch/key snapshot bus --------------------------
    // [9:8] profile; [7] SW4; [6] SW3; [5] SW2 plate display; [4] SW1;
    // [3] KEY4; [2] KEY3; [1] KEY2; [0] KEY1 (all key bits high while held).
    wire [9:0] W_snapshot_bus = {
        S_profile_24m, W_sw4_active, W_sw3_active, W_sw2_active, W_sw1_active,
        W_key4_down, W_key3_down, W_key2_down, W_key1_down
    };
    reg [9:0] S_snapshot_hold_24m;
    reg       S_snapshot_req_24m;
    reg       S_snapshot_ack_meta_24m;
    reg       S_snapshot_ack_sync_24m;
    reg       S_snapshot_ack_pix;
    wire      W_snapshot_source_idle =
        (S_snapshot_req_24m == S_snapshot_ack_sync_24m);
    wire      W_snapshot_source_changed =
        (W_snapshot_bus != S_snapshot_hold_24m);

    always @(posedge I_clk_24m or negedge W_rst_24m_n) begin
        if (!W_rst_24m_n) begin
            S_snapshot_hold_24m    <= 10'd0;
            S_snapshot_req_24m     <= 1'b0;
            S_snapshot_ack_meta_24m <= 1'b0;
            S_snapshot_ack_sync_24m <= 1'b0;
        end
        else begin
            S_snapshot_ack_meta_24m <= S_snapshot_ack_pix;
            S_snapshot_ack_sync_24m <= S_snapshot_ack_meta_24m;
            if (W_snapshot_source_idle && W_snapshot_source_changed) begin
                // The bundled bus is held unchanged until the pixel side
                // acknowledges it after a frame-boundary commit.
                S_snapshot_hold_24m <= W_snapshot_bus;
                S_snapshot_req_24m  <= ~S_snapshot_req_24m;
            end
        end
    end

    // ---- pixel-domain CDC and frame-boundary snapshot --------------------
    reg [9:0] S_snapshot_pending_pix;
    reg       S_snapshot_pending_valid_pix;
    reg       S_snapshot_req_pix_1;
    reg       S_snapshot_req_pix_2;
    reg       S_home_toggle_pix_1;
    reg       S_home_toggle_pix_2;
    reg       S_home_toggle_seen;

    always @(posedge I_clk_pix or negedge W_rst_pix_n) begin
        if (!W_rst_pix_n) begin
            S_snapshot_pending_pix <= 10'd0;
            S_snapshot_pending_valid_pix <= 1'b0;
            S_snapshot_req_pix_1 <= 1'b0;
            S_snapshot_req_pix_2 <= 1'b0;
            S_snapshot_ack_pix <= 1'b0;
            S_home_toggle_pix_1 <= 1'b0;
            S_home_toggle_pix_2 <= 1'b0;
            S_home_toggle_seen <= 1'b0;
            O_profile_pix <= 2'd0;
            O_plate_display_enable <= 1'b0;
            O_track_enable <= 1'b0;
            O_jog_enable <= 1'b0;
            O_jog_left <= 1'b0;
            O_jog_right <= 1'b0;
            O_jog_up <= 1'b0;
            O_jog_down <= 1'b0;
            O_gimbal_home <= 1'b0;
        end
        else begin
            S_snapshot_req_pix_1 <= S_snapshot_req_24m;
            S_snapshot_req_pix_2 <= S_snapshot_req_pix_1;
            S_home_toggle_pix_1 <= S_home_toggle_24m;
            S_home_toggle_pix_2 <= S_home_toggle_pix_1;

            // Capture the held bundled bus only after the request has crossed
            // two pixel clocks.  It remains pending until a frame boundary.
            if (S_snapshot_req_pix_2 != S_snapshot_ack_pix) begin
                S_snapshot_pending_pix <= S_snapshot_hold_24m;
                S_snapshot_pending_valid_pix <= 1'b1;
            end

            O_gimbal_home <= S_home_toggle_pix_2 ^ S_home_toggle_seen;
            S_home_toggle_seen <= S_home_toggle_pix_2;

            if (I_frame_pulse && S_snapshot_pending_valid_pix) begin
                O_profile_pix <= S_snapshot_pending_pix[9:8];
                O_plate_display_enable <= S_snapshot_pending_pix[5];
                O_track_enable <= S_snapshot_pending_pix[6];
                O_jog_enable <= (S_snapshot_pending_pix[9:8] == 2'd2);
                O_jog_left <= (S_snapshot_pending_pix[9:8] == 2'd2) &&
                              S_snapshot_pending_pix[0];
                O_jog_right <= (S_snapshot_pending_pix[9:8] == 2'd2) &&
                               S_snapshot_pending_pix[1];
                O_jog_up <= (S_snapshot_pending_pix[9:8] == 2'd2) &&
                            S_snapshot_pending_pix[2];
                O_jog_down <= (S_snapshot_pending_pix[9:8] == 2'd2) &&
                              S_snapshot_pending_pix[3];
                S_snapshot_pending_valid_pix <= 1'b0;
                S_snapshot_ack_pix <= S_snapshot_req_pix_2;
            end
        end
    end

endmodule
