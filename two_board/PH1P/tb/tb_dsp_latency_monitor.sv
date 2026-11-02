//=============================================================================
//  tb_dsp_latency_monitor -- 延迟去抖/毫秒换算单元测试
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk_pix（被测），I_clk_ddr（模拟异步发布侧）
//  复位     : I_rst_n
//  参数     : 无
//  验证状态 : Questa 通过时输出 TB_RESULT: PASS
//=============================================================================
//  检查内容：
//    1. 复位后输出 0 帧 / 0 ms，valid=0。
//    2. v1.13 去抖门限：新值连续 9 个显示场才被采用；8 个不采用。
//    3. 中途出现旧值会清零“连续计数”（5 个新值 + 1 个旧值 + 8 个新值仍不采用）。
//    4. 16.5 ms/帧换算：2->33、3->49、1->16、4->66、15->247 ms。
//    5. 发布 toggle 异步于像素时钟，验证数据保持 + toggle 的两级同步采样。
//=============================================================================
`timescale 1ns / 1ps

module tb_dsp_latency_monitor;

    reg I_clk_pix = 1'b0;
    reg I_clk_ddr = 1'b0;

    always #6.667  I_clk_pix = ~I_clk_pix;   // 75 MHz
    always #7.5    I_clk_ddr = ~I_clk_ddr;   // 66.667 MHz（异步发布侧）

    reg I_rst_n;
    reg [4:0] I_lat_frames_async;
    reg I_lat_update_async;

    wire [4:0] O_lat_frames;
    wire [9:0] O_lat_ms;
    wire O_lat_valid;

    integer S_checks;
    integer S_errors;

    dsp_latency_monitor dut (
        .I_clk_pix            ( I_clk_pix            ),
        .I_rst_n              ( I_rst_n              ),
        .I_lat_frames_async   ( I_lat_frames_async   ),
        .I_lat_update_async   ( I_lat_update_async   ),
        .O_lat_frames         ( O_lat_frames         ),
        .O_lat_ms             ( O_lat_ms             ),
        .O_lat_valid          ( O_lat_valid          )
    );

    task automatic publish(input [4:0] frames);
        begin
            @(posedge I_clk_ddr);
            I_lat_frames_async <= frames;
            I_lat_update_async <= ~I_lat_update_async;
        end
    endtask

    task automatic publish_n(input [4:0] frames, input integer count);
        integer i;
        begin
            for (i = 0; i < count; i = i + 1) begin
                publish(frames);
                repeat (6) @(posedge I_clk_pix);
            end
        end
    endtask

    task automatic settle;
        begin
            repeat (20) @(posedge I_clk_pix);
        end
    endtask

    task automatic check(input [4:0] exp_frames, input [9:0] exp_ms,
                         input exp_valid, input [255:0] tag);
        begin
            S_checks = S_checks + 1;
            if (O_lat_frames !== exp_frames) begin
                $display("ERROR %0s: frames=%0d expected=%0d", tag, O_lat_frames, exp_frames);
                S_errors = S_errors + 1;
            end
            if (O_lat_ms !== exp_ms) begin
                $display("ERROR %0s: ms=%0d expected=%0d", tag, O_lat_ms, exp_ms);
                S_errors = S_errors + 1;
            end
            if (O_lat_valid !== exp_valid) begin
                $display("ERROR %0s: valid=%0d expected=%0d", tag, O_lat_valid, exp_valid);
                S_errors = S_errors + 1;
            end
        end
    endtask

    integer i;

    initial begin
        I_rst_n             = 1'b0;
        I_lat_frames_async  = 5'd0;
        I_lat_update_async  = 1'b0;
        S_checks            = 0;
        S_errors            = 0;

        repeat (10) @(posedge I_clk_pix);
        I_rst_n = 1'b1;
        settle;
        check(5'd0, 10'd0, 1'b0, "reset");

        // 12 个样本的 2 帧：第 9 个样本被采用。
        publish_n(5'd2, 12);
        settle;
        check(5'd2, 10'd33, 1'b1, "adopt-2");

        // 8 个 3 帧样本不够，读数保持 2 帧；第 9 个才变成 3 帧。
        publish_n(5'd3, 8);
        settle;
        check(5'd2, 10'd33, 1'b1, "hold-8");
        publish(5'd3);
        settle;
        check(5'd3, 10'd49, 1'b1, "adopt-3");

        // 5 个 2 + 1 个 3（等于当前值，清零计数）+ 8 个 2：仍不采用。
        publish_n(5'd2, 5);
        publish_n(5'd3, 1);
        publish_n(5'd2, 8);
        settle;
        check(5'd3, 10'd49, 1'b1, "counter-reset");

        // 再来一个 2 帧样本（连续第 9 个）才采用。
        publish(5'd2);
        settle;
        check(5'd2, 10'd33, 1'b1, "adopt-2-again");

        // 16.5 ms/帧换算边界。
        publish_n(5'd1, 9);
        settle;
        check(5'd1, 10'd16, 1'b1, "ms-1");
        publish_n(5'd15, 9);
        settle;
        check(5'd15, 10'd247, 1'b1, "ms-15");
        publish_n(5'd4, 9);
        settle;
        check(5'd4, 10'd66, 1'b1, "ms-4");

        if (S_checks < 9) begin
            $display("ERROR: only %0d checks were executed", S_checks);
            S_errors = S_errors + 1;
        end

        if (S_errors == 0) begin
            $display("TB_RESULT: PASS tb_dsp_latency_monitor checks=%0d", S_checks);
            $finish;
        end
        else begin
            $display("TB_RESULT: FAIL tb_dsp_latency_monitor checks=%0d errors=%0d",
                     S_checks, S_errors);
            $finish;
        end
    end

    initial begin
        #2000000;
        $display("TB_RESULT: FAIL tb_dsp_latency_monitor timeout");
        $finish;
    end

endmodule
