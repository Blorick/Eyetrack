//=============================================================================
//  dsp_seg_display -- 8 位数码管动态扫描驱动（显示端到端延迟毫秒数）
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk_pix
//  复位     : I_rst_n，异步低有效
//  参数     : P_SCAN_DIV, P_BOOT_CYC
//  验证状态 : 仿真 通过 | 综合 通过 | 布线 通过 | 上板 未做
//=============================================================================
//  硬件依据（杭州康芯 HX1P35A/B 开发板，PH1P35MDG324）
//  ---------------------------------------------------------------------------
//    * 板上 2 组 4 位一体，共 8 位数码管。
//    * 段线 DIG[0..7] = a,b,c,d,e,f,g,h(DP)，8 位全部并联：
//          C9=a  C11=b  B11=c  E7=d  E6=e  B6=f  B7=g  E5=DP
//    * 位选 SEL[0..7]，SEL0 = 左起第 1 位：
//          G15=SEL0  G14=SEL1  A17=SEL2  A18=SEL3
//          B16=SEL4  A15=SEL5  B15=SEL6  A14=SEL7
//    * 手册要求整屏扫描频率 > 50 Hz（本设计 1 kHz/位 -> 整屏 125 Hz）。
//    * 这 16 个引脚与 8 组双色 LED 复用，板上 SW6 必须“拨上”才是数码管。
//
//  极性（v1.13 上板逐组试过三轮的结论，I_pol 直接选择下面两个输出级行为）
//  ---------------------------------------------------------------------------
//      I_pol[0]：0 = 段线低电平点亮 / 1 = 段线高电平点亮
//      I_pol[1]：0 = 位选一位高电平选中 / 1 = 位选一位低电平选中
//
//      2'b00（段线低 + 位选高）-> 全亮
//      2'b01（段线高 + 位选高）-> 全亮
//      2'b11（段线高 + 位选低）-> 乱码
//      2'b10（段线低 + 位选低）-> 自检 0.1234567 正常、随后读数正常  <= 默认
//
//  顶层固定 2'b10，即“段线低电平点亮、位选一位拉低选中”。
//
//  自检与显示格式
//  ---------------------------------------------------------------------------
//    复位后约 1.8 s（P_BOOT_CYC 个像素时钟）显示 **0.1234567**，一眼可判极性、
//    段映射和 8 位是否都活；随后转入延迟显示。
//
//    显示格式（8 位，左起 d0..d7）：
//        L   A   t .  (空)  千  百  十  个
//        └─── "LAt." = Latency ───┘   └── 毫秒，前导零消隐 ──┘
//        LAt.  33  ->  端到端延迟 33 ms
//=============================================================================
module dsp_seg_display #(
    parameter integer P_SCAN_DIV = 75000,        // 75 MHz / 75000 = 1 kHz/位
    parameter integer P_BOOT_CYC = 134217728     // 2^27 / 75 MHz ~= 1.79 s
) (
    input wire I_clk_pix,
    input wire I_rst_n,
    input wire [1:0] I_pol,                      // [0]=段线高有效 [1]=位选低有效
    input wire [4:0] I_lat_frames,               // 端到端延迟（帧）；保留备用，当前不上屏
    input wire [9:0] I_lat_ms,                   // 端到端延迟（ms）—— 上屏的量
    output reg [7:0] O_seg_dig,                  // 段线 a..h(DP)
    output reg [7:0] O_seg_sel                   // 位选 SEL0..7
);

    // 显式位宽，避免对 integer 型 parameter 做位选。
    localparam [16:0] P_SCAN_M1 = P_SCAN_DIV - 1;
    localparam [26:0] P_BOOT_MAX = P_BOOT_CYC - 1;

    //-------------------------------------------------------------------------
    // 段码：bit0=a ... bit6=g，bit7=DP，高有效（真正极性在输出级按 I_pol 定）
    //-------------------------------------------------------------------------
    localparam [3:0] P_CH_L  = 4'd10;
    localparam [3:0] P_CH_A  = 4'd11;
    localparam [3:0] P_CH_T  = 4'd12;
    localparam [3:0] P_CH_BL = 4'd13;
    localparam [3:0] P_CH_C  = 4'd14;
    localparam [3:0] P_CH_N  = 4'd15;

    function [7:0] seg_code;
        input [3:0] d;
        begin
            case (d)
                4'd0:    seg_code = 8'h3f;   // a b c d e f
                4'd1:    seg_code = 8'h06;   // b c
                4'd2:    seg_code = 8'h5b;   // a b d e g
                4'd3:    seg_code = 8'h4f;   // a b c d g
                4'd4:    seg_code = 8'h66;   // b c f g
                4'd5:    seg_code = 8'h6d;   // a c d f g
                4'd6:    seg_code = 8'h7d;   // a c d e f g
                4'd7:    seg_code = 8'h07;   // a b c
                4'd8:    seg_code = 8'h7f;   // 全段
                4'd9:    seg_code = 8'h6f;   // a b c d f g
                P_CH_L:  seg_code = 8'h38;   // L = f e d
                P_CH_A:  seg_code = 8'h77;   // A = a b c e f g
                P_CH_T:  seg_code = 8'h78;   // t = d e f g（小写 t）
                P_CH_C:  seg_code = 8'h39;   // C = a d e f
                P_CH_N:  seg_code = 8'h54;   // n = c e g（小写 n）
                default: seg_code = 8'h00;   // 空白
            endcase
        end
    endfunction

    //-------------------------------------------------------------------------
    // 自检：复位后 P_BOOT_CYC 个像素时钟；I_pol 一变也重新计时。
    //-------------------------------------------------------------------------
    reg [1:0] S_pol_d;

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) S_pol_d <= 2'b10;          // 与顶层上电默认保持一致
        else          S_pol_d <= I_pol;
    end

    wire W_pol_change = (I_pol != S_pol_d);

    reg [26:0] S_boot;

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n)                      S_boot <= 27'd0;
        else if (W_pol_change)             S_boot <= 27'd0;   // 切换极性 -> 重跑自检
        else if (S_boot != P_BOOT_MAX)     S_boot <= S_boot + 27'd1;
    end

    wire W_test = (S_boot != P_BOOT_MAX);

    //-------------------------------------------------------------------------
    // 串行 二进制 -> 4 位 BCD（双 dabble）
    //   只转毫秒（10 位，最大 1023，4 位够用）。每拍一步，10 拍转完。
    //-------------------------------------------------------------------------
    reg [9:0]  S_bin;
    reg [15:0] S_shift;
    reg [3:0]  S_bit_cnt;
    reg [15:0] S_bcd_ms;                          // {千,百,十,个}

    wire [15:0] W_shift_plus3 = {
        (S_shift[15:12] >= 4'd5) ? (S_shift[15:12] + 4'd3) : S_shift[15:12],
        (S_shift[11:8]  >= 4'd5) ? (S_shift[11:8]  + 4'd3) : S_shift[11:8],
        (S_shift[7:4]   >= 4'd5) ? (S_shift[7:4]   + 4'd3) : S_shift[7:4],
        (S_shift[3:0]   >= 4'd5) ? (S_shift[3:0]   + 4'd3) : S_shift[3:0]
    };

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_bin    <= 10'd0;
            S_shift  <= 16'd0;
            S_bit_cnt <= 4'd10;
            S_bcd_ms <= 16'd0;
        end
        else if (S_bit_cnt == 4'd0) begin
            S_bcd_ms  <= S_shift;
            S_bin     <= I_lat_ms;
            S_shift   <= 16'd0;
            S_bit_cnt <= 4'd10;
        end
        else begin
            S_shift   <= {W_shift_plus3[14:0], S_bin[9]};
            S_bin     <= {S_bin[8:0], 1'b0};
            S_bit_cnt <= S_bit_cnt - 4'd1;
        end
    end

    //-------------------------------------------------------------------------
    // 8 位从左到右的字码
    //   正常：L A t. (空) 千百十个（前导零消隐）
    //   自检：0 1 2 3 4 5 6 7
    //-------------------------------------------------------------------------
    wire [9:0] W_ms = I_lat_ms;

    wire [3:0] W_N0 = P_CH_L;
    wire [3:0] W_N1 = P_CH_A;
    wire [3:0] W_N2 = P_CH_T;
    wire [3:0] W_N3 = P_CH_BL;
    wire [3:0] W_N4 = (W_ms >= 10'd1000) ? S_bcd_ms[15:12] : P_CH_BL;
    wire [3:0] W_N5 = (W_ms >= 10'd100 ) ? S_bcd_ms[11:8]  : P_CH_BL;
    wire [3:0] W_N6 = (W_ms >= 10'd10  ) ? S_bcd_ms[7:4]   : P_CH_BL;
    wire [3:0] W_N7 = S_bcd_ms[3:0];

    wire [3:0] W_D0 = W_test ? 4'd0 : W_N0;
    wire [3:0] W_D1 = W_test ? 4'd1 : W_N1;
    wire [3:0] W_D2 = W_test ? 4'd2 : W_N2;
    wire [3:0] W_D3 = W_test ? 4'd3 : W_N3;
    wire [3:0] W_D4 = W_test ? 4'd4 : W_N4;
    wire [3:0] W_D5 = W_test ? 4'd5 : W_N5;
    wire [3:0] W_D6 = W_test ? 4'd6 : W_N6;
    wire [3:0] W_D7 = W_test ? 4'd7 : W_N7;

    //-------------------------------------------------------------------------
    // 扫描节拍
    //-------------------------------------------------------------------------
    reg [16:0] S_div;
    reg [2:0]  S_idx;

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_div <= 17'd0;
            S_idx <= 3'd0;
        end
        else if (S_div >= P_SCAN_M1) begin
            S_div <= 17'd0;
            S_idx <= S_idx + 3'd1;
        end
        else begin
            S_div <= S_div + 17'd1;
        end
    end

    reg [3:0] S_scan_char;

    always @(*) begin
        case (S_idx)
            3'd0:    S_scan_char = W_D0;
            3'd1:    S_scan_char = W_D1;
            3'd2:    S_scan_char = W_D2;
            3'd3:    S_scan_char = W_D3;
            3'd4:    S_scan_char = W_D4;
            3'd5:    S_scan_char = W_D5;
            3'd6:    S_scan_char = W_D6;
            default: S_scan_char = W_D7;
        endcase
    end

    // 小数点：自检时在第 1 位；正常时落在 "t" 上，把标签和数值分开
    wire W_dot = W_test ? (S_idx == 3'd0) : (S_idx == 3'd2);
    wire [7:0] W_font = seg_code(S_scan_char) | (W_dot ? 8'h80 : 8'h00);

    //-------------------------------------------------------------------------
    // 输出级：极性由 I_pol 决定
    //-------------------------------------------------------------------------
    wire [7:0] W_seg_pin = I_pol[0] ?  W_font          : ~W_font;
    wire [7:0] W_sel_pin = I_pol[1] ? ~(8'h01 << S_idx) : (8'h01 << S_idx);

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_seg_dig <= 8'h00;      // 复位期间不驱动，交给引脚上下拉
            O_seg_sel <= 8'h00;
        end
        else begin
            O_seg_dig <= W_seg_pin;
            O_seg_sel <= W_sel_pin;
        end
    end

endmodule
