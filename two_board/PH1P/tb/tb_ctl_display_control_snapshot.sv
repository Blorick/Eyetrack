//=============================================================================
//  tb_ctl_display_control_snapshot  --  CDC tuple/frame-boundary TB
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk_24m, I_clk_pix
//  复位     : I_rst_n
//  参数     : 无
//  验证状态 : Questa 通过时输出 TB_RESULT: PASS
//=============================================================================
`timescale 1ns / 1ps

module tb_ctl_display_control_snapshot;
    reg S_clk_24m = 1'b0;
    reg S_clk_pix = 1'b0;
    always #5 S_clk_24m = ~S_clk_24m;
    always #7 S_clk_pix = ~S_clk_pix;

    reg I_rst_n;
    reg [4:0] I_mode;
    reg signed [8:0] I_brightness;
    reg [7:0] I_sat_factor;
    reg I_frame_pix;

    wire [4:0] O_mode;
    wire signed [8:0] O_brightness;
    wire [7:0] O_sat_factor;

    integer S_checks;
    integer S_errors;
    integer S_monitor_enabled;
    integer S_monitor_count;

    ctl_display_control_snapshot dut (
        .I_clk_24m    ( S_clk_24m    ),
        .I_clk_pix    ( S_clk_pix    ),
        .I_rst_n      ( I_rst_n      ),
        .I_mode       ( I_mode       ),
        .I_brightness ( I_brightness ),
        .I_sat_factor ( I_sat_factor ),
        .I_frame_pix  ( I_frame_pix  ),
        .O_mode       ( O_mode       ),
        .O_brightness ( O_brightness ),
        .O_sat_factor ( O_sat_factor )
    );

    // Sample after NBA updates so a tuple is checked as one visible transfer.
    always @(posedge S_clk_pix) begin
        #1;
        if (S_monitor_enabled) begin
            S_monitor_count = S_monitor_count + 1;
            if ((O_mode !== 5'd0) && (O_mode !== 5'd3) && (O_mode !== 5'd7)) begin
                $display("FAIL torn mode=%0d bright=%0d sat=%0d", O_mode, O_brightness, O_sat_factor);
                S_errors = S_errors + 1;
            end
            if ((O_mode == 5'd3) &&
                ((O_brightness !== -9'sd11) || (O_sat_factor !== 8'd201))) begin
                $display("FAIL tuple A not atomic mode=%0d bright=%0d sat=%0d", O_mode, O_brightness, O_sat_factor);
                S_errors = S_errors + 1;
            end
            if ((O_mode == 5'd7) &&
                ((O_brightness !== 9'sd37) || (O_sat_factor !== 8'd77))) begin
                $display("FAIL tuple B not atomic mode=%0d bright=%0d sat=%0d", O_mode, O_brightness, O_sat_factor);
                S_errors = S_errors + 1;
            end
        end
    end

    task automatic set_source_tuple;
        input [4:0] V_mode;
        input signed [8:0] V_brightness;
        input [7:0] V_sat_factor;
        begin
            @(negedge S_clk_24m);
            I_mode = V_mode;
            I_brightness = V_brightness;
            I_sat_factor = V_sat_factor;
        end
    endtask

    task automatic pulse_frame;
        begin
            @(negedge S_clk_pix);
            I_frame_pix = 1'b1;
            @(negedge S_clk_pix);
            I_frame_pix = 1'b0;
        end
    endtask

    task automatic check_tuple;
        input [4:0] V_mode;
        input signed [8:0] V_brightness;
        input [7:0] V_sat_factor;
        input [255:0] V_label;
        begin
            S_checks = S_checks + 1;
            if ((O_mode !== V_mode) || (O_brightness !== V_brightness) ||
                (O_sat_factor !== V_sat_factor)) begin
                $display("FAIL %0s got mode=%0d bright=%0d sat=%0d expected mode=%0d bright=%0d sat=%0d",
                         V_label, O_mode, O_brightness, O_sat_factor,
                         V_mode, V_brightness, V_sat_factor);
                S_errors = S_errors + 1;
            end
        end
    endtask

    initial begin
        I_rst_n = 1'b0;
        I_mode = 5'd0;
        I_brightness = 9'sd64;
        I_sat_factor = 8'd128;
        I_frame_pix = 1'b0;
        S_checks = 0;
        S_errors = 0;
        S_monitor_enabled = 0;
        S_monitor_count = 0;

        repeat (5) @(posedge S_clk_pix);
        I_rst_n = 1'b1;
        // Allow the module's two-flop reset-release chains to settle in both
        // clock domains before checking the reset tuple.
        repeat (4) @(posedge S_clk_pix);
        #1;
        check_tuple(5'd0, 9'sd64, 8'd128, "reset tuple");

        // Tuple A must cross as one transfer and remain invisible until SOF.
        set_source_tuple(5'd3, -9'sd11, 8'd201);
        S_monitor_enabled = 1;
        repeat (9) @(posedge S_clk_pix);
        #1;
        S_checks = S_checks + 1;
        if ((O_mode !== 5'd0) || (O_brightness !== 9'sd64) || (O_sat_factor !== 8'd128)) begin
            $display("FAIL tuple A became visible before frame boundary mode=%0d bright=%0d sat=%0d",
                     O_mode, O_brightness, O_sat_factor);
            S_errors = S_errors + 1;
        end
        pulse_frame();
        #1;
        check_tuple(5'd3, -9'sd11, 8'd201, "tuple A at frame");

        // Change the source before the next frame.  A short race is allowed to
        // show either the old or new complete tuple, but never a mixed tuple.
        set_source_tuple(5'd7, 9'sd37, 8'd77);
        repeat (1) @(posedge S_clk_pix);
        pulse_frame();
        #1;
        S_checks = S_checks + 1;
        if (!(((O_mode === 5'd3) && (O_brightness === -9'sd11) && (O_sat_factor === 8'd201)) ||
              ((O_mode === 5'd7) && (O_brightness === 9'sd37) && (O_sat_factor === 8'd77)))) begin
            $display("FAIL race produced torn tuple mode=%0d bright=%0d sat=%0d", O_mode, O_brightness, O_sat_factor);
            S_errors = S_errors + 1;
        end
        repeat (16) @(posedge S_clk_pix);
        pulse_frame();
        #1;
        check_tuple(5'd7, 9'sd37, 8'd77, "tuple B at frame");

        if (S_errors == 0)
            $display("TB_RESULT: PASS tb_ctl_display_control_snapshot checks=%0d samples=%0d errors=0", S_checks, S_monitor_count);
        else
            $display("TB_RESULT: FAIL tb_ctl_display_control_snapshot checks=%0d samples=%0d errors=%0d", S_checks, S_monitor_count, S_errors);
        $finish;
    end
endmodule
