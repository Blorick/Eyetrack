//=============================================================================
// tb_ctl_vehicle_controls -- control-page, debounce and CDC contract test
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk_24m, I_clk_pix
//  复位     : I_rst_n，异步低有效
//  参数     : reduced debounce/long-press/TF intervals for smoke simulation
//  验证状态 : 仿真 未做 | 综合 未做 | 布线 未做 | 上板 未做
//=============================================================================
module tb_ctl_vehicle_controls;

    reg clk_24m;
    reg clk_pix;
    reg rst_n;
    reg frame_pulse;
    reg key1_n;
    reg key2_n;
    reg key3_n;
    reg key4_n;
    reg sw1;
    reg sw2;
    reg sw3;
    reg sw4;

    wire [1:0] profile_24m;
    wire camera_key1_n;
    wire camera_key2_short;
    wire camera_ae_up;
    wire camera_ae_dn;
    wire camera_ag_up;
    wire camera_ag_dn;
    wire tf_key_n;
    wire [1:0] profile_pix;
    wire track_enable;
    wire jog_enable;
    wire jog_left;
    wire jog_right;
    wire jog_up;
    wire jog_down;
    wire gimbal_home;

    integer errors;
    integer camera_short_count;
    integer home_count;
    reg     tf_low_seen;
    integer tf_low_cycles;

    ctl_vehicle_controls #(
        .DEBOUNCE_CYCLES  (2),
        .LONG_HOLD_CYCLES (64),
        .TF_PULSE_CYCLES  (5),
        .SW_ACTIVE_LEVEL  (1'b0)
    ) dut (
        .I_clk_24m          (clk_24m),
        .I_clk_pix          (clk_pix),
        .I_rst_n            (rst_n),
        .I_frame_pulse      (frame_pulse),
        .I_key1_n          (key1_n),
        .I_key2_n          (key2_n),
        .I_key3_n          (key3_n),
        .I_key4_n          (key4_n),
        .I_sw1             (sw1),
        .I_sw2             (sw2),
        .I_sw3             (sw3),
        .I_sw4             (sw4),
        .O_profile_24m     (profile_24m),
        .O_camera_key1_n   (camera_key1_n),
        .O_camera_key2_short(camera_key2_short),
        .O_camera_ae_up    (camera_ae_up),
        .O_camera_ae_dn    (camera_ae_dn),
        .O_camera_ag_up    (camera_ag_up),
        .O_camera_ag_dn    (camera_ag_dn),
        .O_tf_key_n        (tf_key_n),
        .O_profile_pix     (profile_pix),
        .O_track_enable    (track_enable),
        .O_jog_enable      (jog_enable),
        .O_jog_left        (jog_left),
        .O_jog_right       (jog_right),
        .O_jog_up          (jog_up),
        .O_jog_down        (jog_down),
        .O_gimbal_home     (gimbal_home)
    );

    always #5 clk_24m = ~clk_24m;
    always #7 clk_pix = ~clk_pix;

    always @(posedge clk_24m) begin
        if (camera_key2_short)
            camera_short_count = camera_short_count + 1;
        if (!tf_key_n) begin
            tf_low_seen = 1'b1;
            tf_low_cycles = tf_low_cycles + 1;
        end
    end

    always @(posedge clk_pix) begin
        if (gimbal_home)
            home_count = home_count + 1;
    end

    task tick_24m;
        begin
            @(posedge clk_24m);
            #1;
        end
    endtask

    task tick_pix;
        begin
            @(posedge clk_pix);
            #1;
        end
    endtask

    task frame_commit;
        begin
            repeat (6) tick_pix;
            frame_pulse = 1'b1;
            tick_pix;
            frame_pulse = 1'b0;
            repeat (3) tick_pix;
        end
    endtask

    task key_press_short;
        input integer key_id;
        begin
            case (key_id)
                1: key1_n = 1'b0;
                2: key2_n = 1'b0;
                3: key3_n = 1'b0;
                default: key4_n = 1'b0;
            endcase
            repeat (8) tick_24m;
            case (key_id)
                1: key1_n = 1'b1;
                2: key2_n = 1'b1;
                3: key3_n = 1'b1;
                default: key4_n = 1'b1;
            endcase
            repeat (8) tick_24m;
        end
    endtask

    task key_hold_long;
        input integer key_id;
        begin
            case (key_id)
                1: key1_n = 1'b0;
                2: key2_n = 1'b0;
                3: key3_n = 1'b0;
                default: key4_n = 1'b0;
            endcase
            repeat (72) tick_24m;
            case (key_id)
                1: key1_n = 1'b1;
                2: key2_n = 1'b1;
                3: key3_n = 1'b1;
                default: key4_n = 1'b1;
            endcase
            repeat (8) tick_24m;
        end
    endtask

    initial begin
        clk_24m = 1'b0;
        clk_pix = 1'b0;
        rst_n = 1'b0;
        frame_pulse = 1'b0;
        key1_n = 1'b1;
        key2_n = 1'b1;
        key3_n = 1'b1;
        key4_n = 1'b1;
        sw1 = 1'b1;
        sw2 = 1'b1;
        sw3 = 1'b1;
        sw4 = 1'b1;
        errors = 0;
        camera_short_count = 0;
        home_count = 0;
        tf_low_seen = 1'b0;
        tf_low_cycles = 0;

        repeat (5) tick_24m;
        rst_n = 1'b1;
        repeat (4) tick_24m;

        // Camera page: KEY2 short is deferred until release; profile remains 0.
        key_press_short(2);
        if (profile_24m !== 2'd0 || camera_short_count != 1) begin
            $display("FAIL: camera KEY2 short profile=%0d count=%0d",
                     profile_24m, camera_short_count);
            errors = errors + 1;
        end

        key3_n = 1'b0;
        repeat (8) tick_24m;
        if (!camera_ae_up || camera_ae_dn) begin
            $display("FAIL: camera AE up mapping");
            errors = errors + 1;
        end
        key3_n = 1'b1;
        repeat (8) tick_24m;

        sw1 = 1'b0;
        repeat (8) tick_24m;
        if (!camera_ag_up || camera_ag_dn) begin
            $display("FAIL: camera AG up mapping");
            errors = errors + 1;
        end
        sw1 = 1'b1;
        repeat (8) tick_24m;

        // A long KEY2 skips the deleted parking page and selects gimbal.
        camera_short_count = 0;
        key_hold_long(2);
        if (profile_24m !== 2'd2 || camera_short_count != 0 ||
            profile_pix !== 2'd0) begin
            $display("FAIL: camera-to-gimbal long p24=%0d ppix=%0d short=%0d",
                     profile_24m, profile_pix, camera_short_count);
            errors = errors + 1;
        end
        if (tf_low_seen) begin
            $display("FAIL: camera SW1 triggered TF");
            errors = errors + 1;
        end

        // The gimbal page retains the existing frame-boundary jog snapshot.
        sw3 = 1'b0;
        frame_commit;
        repeat (8) tick_24m;
        frame_commit;
        frame_commit;
        if (profile_pix !== 2'd2 || !jog_enable || !track_enable) begin
            $display("FAIL: gimbal frame snapshot p=%0d track=%0d jog=%0d",
                     profile_pix, track_enable, jog_enable);
            errors = errors + 1;
        end

        key1_n = 1'b0;
        key2_n = 1'b0;
        key3_n = 1'b0;
        key4_n = 1'b0;
        sw4 = 1'b0;
        repeat (8) tick_24m;
        frame_commit;
        frame_commit;
        if (!jog_left || !jog_right || !jog_up || !jog_down ||
            !camera_key1_n || camera_ae_up || camera_ae_dn ||
            camera_ag_up || camera_ag_dn) begin
            $display("FAIL: gimbal jog or camera isolation");
            errors = errors + 1;
        end
        key1_n = 1'b1;
        key2_n = 1'b1;
        key3_n = 1'b1;
        key4_n = 1'b1;
        sw4 = 1'b1;
        repeat (16) tick_24m;
        repeat (4) tick_pix;
        if (home_count != 1) begin
            $display("FAIL: gimbal SW4 home count=%0d", home_count);
            errors = errors + 1;
        end

        // SW1 activation on the gimbal page now reads TF. Holding the
        // switch must generate exactly one pulse of TF_PULSE_CYCLES clocks.
        sw1 = 1'b0;
        repeat (24) tick_24m;
        if (!tf_low_seen || tf_low_cycles != 5 || tf_key_n !== 1'b1) begin
            $display("FAIL: gimbal TF pulse seen=%0d cycles=%0d level=%0d",
                     tf_low_seen, tf_low_cycles, tf_key_n);
            errors = errors + 1;
        end
        repeat (16) tick_24m;
        if (tf_low_cycles != 5) begin
            $display("FAIL: held SW1 retriggered TF cycles=%0d", tf_low_cycles);
            errors = errors + 1;
        end
        sw1 = 1'b1;
        repeat (8) tick_24m;

        key_hold_long(2);
        frame_commit;
        frame_commit;
        if (profile_24m !== 2'd0 || profile_pix !== 2'd0 || jog_enable ||
            jog_left || jog_right || jog_up || jog_down) begin
            $display("FAIL: gimbal-to-camera long p24=%0d ppix=%0d",
                     profile_24m, profile_pix);
            errors = errors + 1;
        end
        if (errors == 0)
            $display("TB_RESULT: PASS");
        else
            $display("TB_RESULT: FAIL errors=%0d", errors);
        $finish;
    end

endmodule
