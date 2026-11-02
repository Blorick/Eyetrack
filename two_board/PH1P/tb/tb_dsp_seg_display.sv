//=============================================================================
//  tb_dsp_seg_display -- 8 位数码管显示驱动单元测试
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk_pix
//  复位     : I_rst_n
//  参数     : 无
//  验证状态 : Questa 通过时输出 TB_RESULT: PASS
//=============================================================================
//  检查内容：
//    1. 复位自检显示 0.1234567（第 1 位带小数点）。
//    2. 正常显示格式 "LAt." + 毫秒前导零消隐：33 -> L A t 空 空 空 3 3；
//       7 -> L A t 空 空 空 空 7；1023 -> L A t 空 1 0 2 3；0 -> L A t 空 空 空 空 0。
//    3. 动态扫描一次只有一位选中，段码/位选对齐。
//    4. 极性 I_pol=2'b00（段线低有效）时字模取反，字符序列不变。
//=============================================================================
`timescale 1ns / 1ps

module tb_dsp_seg_display;

    reg I_clk_pix = 1'b0;

    always #6.667 I_clk_pix = ~I_clk_pix;   // 75 MHz

    reg I_rst_n;
    reg [1:0] I_pol;
    reg [4:0] I_lat_frames;
    reg [9:0] I_lat_ms;

    wire [7:0] O_seg_dig;
    wire [7:0] O_seg_sel;
    wire [7:0] O_seg_dig_inv;
    wire [7:0] O_seg_sel_inv;

    integer S_checks;
    integer S_errors;

    localparam [3:0] P_CH_BLANK = 4'd13;
    localparam [3:0] P_CH_L     = 4'd10;
    localparam [3:0] P_CH_A     = 4'd11;
    localparam [3:0] P_CH_T     = 4'd12;

    //-------------------------------------------------------------------------
    // 段码反查（bit7=DP 在比较前屏蔽）
    //-------------------------------------------------------------------------
    function [3:0] seg_to_char;
        input [7:0] code;
        reg [7:0] font;
        begin
            font = code & 8'h7F;
            case (font)
                8'h00: seg_to_char = 4'd13;   // 空白
                8'h3f: seg_to_char = 4'd0;
                8'h06: seg_to_char = 4'd1;
                8'h5b: seg_to_char = 4'd2;
                8'h4f: seg_to_char = 4'd3;
                8'h66: seg_to_char = 4'd4;
                8'h6d: seg_to_char = 4'd5;
                8'h7d: seg_to_char = 4'd6;
                8'h07: seg_to_char = 4'd7;
                8'h7f: seg_to_char = 4'd8;
                8'h6f: seg_to_char = 4'd9;
                8'h38: seg_to_char = 4'd10;   // L
                8'h77: seg_to_char = 4'd11;   // A
                8'h78: seg_to_char = 4'd12;   // t
                8'h39: seg_to_char = 4'd14;   // C
                8'h54: seg_to_char = 4'd15;   // n
                default: seg_to_char = 4'hF;
            endcase
        end
    endfunction

    function [2:0] one_hot_index;
        input [7:0] value;
        integer i;
        reg found;
        begin
            one_hot_index = 3'd0;
            found = 1'b0;
            for (i = 0; i < 8; i = i + 1) begin
                if (value[i] && !found) begin
                    one_hot_index = i[2:0];
                    found = 1'b1;
                end
            end
            if (!found) one_hot_index = 3'd7;
        end
    endfunction

    //-------------------------------------------------------------------------
    // DUT：主实例为板上定案极性 2'b10；第二实例验证段线低有效（2'b00）。
    // P_SCAN_DIV=3 -> 每位 4 个时钟；P_BOOT_CYC=64 -> 自检约 64 个时钟。
    //-------------------------------------------------------------------------
    dsp_seg_display #(
        .P_SCAN_DIV ( 3   ),
        .P_BOOT_CYC ( 64  )
    ) dut_main (
        .I_clk_pix    ( I_clk_pix    ),
        .I_rst_n      ( I_rst_n      ),
        .I_pol        ( I_pol        ),
        .I_lat_frames ( I_lat_frames ),
        .I_lat_ms     ( I_lat_ms     ),
        .O_seg_dig    ( O_seg_dig    ),
        .O_seg_sel    ( O_seg_sel    )
    );

    dsp_seg_display #(
        .P_SCAN_DIV ( 3  ),
        .P_BOOT_CYC ( 64 )
    ) dut_inv (
        .I_clk_pix    ( I_clk_pix        ),
        .I_rst_n      ( I_rst_n          ),
        .I_pol        ( 2'b00            ),
        .I_lat_frames ( I_lat_frames     ),
        .I_lat_ms     ( I_lat_ms         ),
        .O_seg_dig    ( O_seg_dig_inv    ),
        .O_seg_sel    ( O_seg_sel_inv    )
    );

    //-------------------------------------------------------------------------
    // 扫描捕获：每次完整扫过 8 位后把最后一遍字模存到 S_scan_main / S_scan_inv。
    //-------------------------------------------------------------------------
    reg [3:0] S_seen_main [0:7];
    reg [3:0] S_seen_inv  [0:7];
    reg [7:0] S_seen_mask_main;
    reg [7:0] S_seen_mask_inv;
    reg [3:0] S_scan_main [0:7];
    reg [3:0] S_scan_inv  [0:7];
    reg S_dot_main;
    reg S_dot_inv;
    reg S_scan_dot_seen_main;
    reg S_scan_dot_seen_inv;
    integer S_scan_count;
    integer S_copy_idx;

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_seen_mask_main <= 8'h00;
            S_seen_mask_inv  <= 8'h00;
            S_scan_count     = 0;
            S_scan_dot_seen_main = 1'b0;
            S_scan_dot_seen_inv  = 1'b0;
            S_dot_main = 1'b0;
            S_dot_inv  = 1'b0;
        end
        else begin
            // 主 DUT 极性 2'b10：段线低有效、位选“一位低有效”。
            if (((~O_seg_sel) != 8'h00) &&
                (((~O_seg_sel) & ((~O_seg_sel) - 8'h01)) == 8'h00)) begin
                S_seen_main[one_hot_index(~O_seg_sel)] = seg_to_char(~O_seg_dig);
                S_seen_mask_main[one_hot_index(~O_seg_sel)] = 1'b1;
                if (O_seg_dig[7] == 1'b0) begin
                    S_scan_dot_seen_main = 1'b1;
                    S_dot_main = one_hot_index(~O_seg_sel) == 3'd0;
                end
            end

            // 对照 DUT 极性 2'b00：段线低有效、位选“一位高有效”。
            if ((O_seg_sel_inv != 8'h00) &&
                ((O_seg_sel_inv & (O_seg_sel_inv - 8'h01)) == 8'h00)) begin
                S_seen_inv[one_hot_index(O_seg_sel_inv)] = seg_to_char(~O_seg_dig_inv);
                S_seen_mask_inv[one_hot_index(O_seg_sel_inv)] = 1'b1;
                if (O_seg_dig_inv[7] == 1'b0) begin
                    S_scan_dot_seen_inv = 1'b1;
                    S_dot_inv = one_hot_index(O_seg_sel_inv) == 3'd0;
                end
            end

            if (S_seen_mask_main == 8'hFF) begin
                S_scan_count = S_scan_count + 1;
                for (S_copy_idx = 0; S_copy_idx < 8; S_copy_idx = S_copy_idx + 1)
                    S_scan_main[S_copy_idx] = S_seen_main[S_copy_idx];
                S_seen_mask_main <= 8'h00;
            end

            if (S_seen_mask_inv == 8'hFF) begin
                for (S_copy_idx = 0; S_copy_idx < 8; S_copy_idx = S_copy_idx + 1)
                    S_scan_inv[S_copy_idx] = S_seen_inv[S_copy_idx];
                S_seen_mask_inv <= 8'h00;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 检查任务
    //-------------------------------------------------------------------------
    task automatic wait_scans(input integer count);
        integer target;
        begin
            target = S_scan_count + count;
            while (S_scan_count < target) @(posedge I_clk_pix);
            repeat (2) @(posedge I_clk_pix);
        end
    endtask

    task automatic check_chars(input [3:0] c0, c1, c2, c3, c4, c5, c6, c7,
                               input [255:0] tag);
        begin
            S_checks = S_checks + 1;
            if ({S_scan_main[0], S_scan_main[1], S_scan_main[2], S_scan_main[3],
                 S_scan_main[4], S_scan_main[5], S_scan_main[6], S_scan_main[7]} !==
                {c0, c1, c2, c3, c4, c5, c6, c7}) begin
                $display("ERROR %0s: main scan = %0d %0d %0d %0d %0d %0d %0d %0d",
                         tag,
                         S_scan_main[0], S_scan_main[1], S_scan_main[2], S_scan_main[3],
                         S_scan_main[4], S_scan_main[5], S_scan_main[6], S_scan_main[7]);
                S_errors = S_errors + 1;
            end
            if ({S_scan_inv[0], S_scan_inv[1], S_scan_inv[2], S_scan_inv[3],
                 S_scan_inv[4], S_scan_inv[5], S_scan_inv[6], S_scan_inv[7]} !==
                {c0, c1, c2, c3, c4, c5, c6, c7}) begin
                $display("ERROR %0s: inverted-polarity scan = %0d %0d %0d %0d %0d %0d %0d %0d",
                         tag,
                         S_scan_inv[0], S_scan_inv[1], S_scan_inv[2], S_scan_inv[3],
                         S_scan_inv[4], S_scan_inv[5], S_scan_inv[6], S_scan_inv[7]);
                S_errors = S_errors + 1;
            end
        end
    endtask

    task automatic check_dot_position(input [255:0] tag);
        begin
            S_checks = S_checks + 1;
            if (!S_scan_dot_seen_main || S_dot_main) begin
                $display("ERROR %0s: main dot missing or still on digit 0", tag);
                S_errors = S_errors + 1;
            end
            if (!S_scan_dot_seen_inv || S_dot_inv) begin
                $display("ERROR %0s: inverted dot missing or still on digit 0", tag);
                S_errors = S_errors + 1;
            end
        end
    endtask

    initial begin
        I_rst_n       = 1'b0;
        I_pol         = 2'b10;
        I_lat_frames  = 5'd2;
        I_lat_ms      = 10'd33;
        S_checks      = 0;
        S_errors      = 0;
        S_scan_dot_seen_main = 1'b0;

        repeat (10) @(posedge I_clk_pix);
        I_rst_n = 1'b1;

        // 第 1 遍完整扫描仍在自检窗口内：0.1234567，DP 在第 1 位。
        wait_scans(1);
        check_chars(4'd0, 4'd1, 4'd2, 4'd3, 4'd4, 4'd5, 4'd6, 4'd7, "self-test");

        // 跳过自检尾部混合扫描，等正常显示稳定（BCD 也已转换完成）。
        wait_scans(4);
        check_chars(P_CH_L, P_CH_A, P_CH_T, P_CH_BLANK,
                    P_CH_BLANK, P_CH_BLANK, 4'd3, 4'd3, "33ms");
        check_dot_position("33ms");

        I_lat_ms = 10'd7;
        wait_scans(2);
        check_chars(P_CH_L, P_CH_A, P_CH_T, P_CH_BLANK,
                    P_CH_BLANK, P_CH_BLANK, P_CH_BLANK, 4'd7, "7ms");

        I_lat_ms = 10'd1023;
        wait_scans(2);
        check_chars(P_CH_L, P_CH_A, P_CH_T, P_CH_BLANK,
                    4'd1, 4'd0, 4'd2, 4'd3, "1023ms");

        I_lat_ms = 10'd0;
        wait_scans(2);
        check_chars(P_CH_L, P_CH_A, P_CH_T, P_CH_BLANK,
                    P_CH_BLANK, P_CH_BLANK, P_CH_BLANK, 4'd0, "0ms");

        if (S_checks < 6) begin
            $display("ERROR: only %0d checks were executed", S_checks);
            S_errors = S_errors + 1;
        end

        if (S_errors == 0) begin
            $display("TB_RESULT: PASS tb_dsp_seg_display checks=%0d scans=%0d",
                     S_checks, S_scan_count);
            $finish;
        end
        else begin
            $display("TB_RESULT: FAIL tb_dsp_seg_display checks=%0d errors=%0d scans=%0d",
                     S_checks, S_errors, S_scan_count);
            $finish;
        end
    end

    initial begin
        #2000000;
        $display("TB_RESULT: FAIL tb_dsp_seg_display timeout");
        $finish;
    end

endmodule
