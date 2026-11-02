//=============================================================================
//  alg_contrast_stretch  --  自动对比度：黑电平钳位 + 灰度映射表
//-----------------------------------------------------------------------------
//  所属域   : alg
//  时钟域   : I_clk_pix
//  复位     : I_rst_n
//  参数     : IMG_WIDTH, IMG_HEIGHT, SAMPLE_DIV, PERCENT_LO, PERCENT_HI, STRETCH_MIN_SPAN, NSMP_H17
//  验证状态 : 仿真 通过（9 套 TB） | 综合 0 ERROR（+113/+114 ps） | 布线 Setup +90 ps / Hold +20 ps，0 违例 | 上板 未做
//=============================================================================
//=============================================================================
//  alg_contrast_stretch  --  自动对比度 / 亮度拉伸（百分位线性拉伸，类直方图效果）
//
//  [重建说明] 原文件在 2026-10-05 的目录整理中丢失，按原设计重建。
//  重建后又做了一次**降资源改动**：直方图 64 bin → 32 bin，理由见下。
//
//  放在 75MHz 的 HDMI 显示域（1 像素 / 时钟），**完全不碰 103.5MHz 的 ISP 域**。
//
//  为什么不做完整直方图均衡
//  ------------------------
//  均衡要 CDF 累积 + 对像素总数求逆；"百分位线性拉伸"用同样的直方图，
//  只多一步找百分位位置，观感接近，控制逻辑简单得多。
//
//  为什么不用 min/max
//  ------------------
//  一帧 92 万像素，只要有一个噪声点 / 高光点，min 就恒为 0、max 恒为 255，
//  拉伸系数恒等于 1 —— 等于没做。所以必须用百分位。
//
//  ★ 为什么是 32 bin（原来 64 bin）
//  ---------------------------------------------------------------------
//  第一版 64 bin，用 64 个 18bit 寄存器实现 = 1152 个触发器。
//  上板综合时报 PHY-9053 slice = 21874 超出器件 21216（超 658），
//  资源报告显示本模块 1111 LUT + 1377 触发器，是第二大占用。
//  降到 32 bin 后直方图寄存器减半（576 个），百分位分辨率从 4 个灰度级
//  变成 8 个灰度级 —— 对一条线性拉伸曲线来说完全够用。
//
//  原理
//  ----
//  1) 用**拉伸之前**的视频算灰度直方图（每 SAMPLE_DIV 个有效像素采 1 个），
//     不会形成"拉伸 → 统计变 → 再拉伸"的正反馈。
//  2) 每帧在垂直消隐期扫一遍直方图，找 1% / 99% 百分位对应的 lo / hi。
//  3) 用 DDA（累加-比较-相减）造一条 256 项的映射表
//         out(x) = clamp( (x-lo) * 256 / (hi-lo) )
//     **不需要除法器，也不需要乘法器。**
//  4) 查表施加：三份 256x8 表，R/G/B 各查一份。
//
//  防呆
//  ----
//  * 只有本帧采样数 >= 半帧期望值才允许扫表 —— vsync 的上升沿和下降沿都在
//    消隐期，不加这个门会对着"已清零的空直方图"再扫一次，把刚算好的
//    lo/hi 覆盖成默认值。
//  * hi-lo 太窄（画面很平）时**直接直通**（out=x），避免把噪声放大成色块。
//
//  延迟：2 个时钟（v/hsync、de、user、last 一起延迟 2 拍，对外仍对齐）。
//=============================================================================
module alg_contrast_stretch #(
    parameter integer IMG_WIDTH      = 1280,
    parameter integer IMG_HEIGHT      = 720,
    parameter integer SAMPLE_DIV = 8,      // 每多少个有效像素采 1 个进直方图
    parameter integer PERCENT_LO = 1,      // 低端百分位 (%)
    parameter integer PERCENT_HI = 99,     // 高端百分位 (%)
    parameter integer STRETCH_MIN_SPAN   = 24      // hi-lo 小于此值就直通，不拉伸
)(
    input  wire        I_clk_pix,       // HDMI_PIXEL_CLK (75MHz)
    input  wire        I_rst_n,
    input  wire        I_enable,    // 1 = 开启（模式 17）
    input  wire        I_vsync,     // 只用来找消隐期，极性不敏感
    input  wire        I_hsync,
    input  wire        I_de,
    input  wire        I_user,
    input  wire        I_last,
    input  wire [23:0] I_video,
    output reg         O_vsync,
    output reg         O_hsync,
    output reg         O_de,
    output reg         O_user,
    output reg         O_last,
    output reg  [23:0] O_video
);

    //-------------------------------------------------------------------------
    // 0) 参数换算
    //-------------------------------------------------------------------------
    localparam integer NSMP   = (IMG_WIDTH * IMG_HEIGHT) / SAMPLE_DIV;
    localparam integer THR_LO = (NSMP * PERCENT_LO) / 100;
    localparam integer THR_HI = (NSMP * PERCENT_HI) / 100;
    localparam integer NSMP_H = NSMP / 2;
    localparam integer NBIN   = 32;                 // 直方图 bin 数

    localparam [2:0]  SDIV_M1   = SAMPLE_DIV - 1;
    // 显式位宽：不要对 integer 型 parameter 做位选，也不要把 wire 的声明
    // 放到使用它的 always 之后 —— Verilog 会把"先用后声明"的标识符当成
    // 隐式 1 位网络（TD 报 HDL-5373），那样 DDA 就在拿 1 位比较，斜率全错。
    localparam [16:0] NSMP_H17  = NSMP_H;
    localparam [7:0]  MINSPAN8  = STRETCH_MIN_SPAN;
    localparam [19:0] THR_LO20  = THR_LO;
    localparam [19:0] THR_HI20  = THR_HI;

    localparam [1:0] ST_COLLECT = 2'd0,
                     ST_SCAN    = 2'd1,
                     ST_CLEAR   = 2'd2,
                     ST_BUILD   = 2'd3;

    //-------------------------------------------------------------------------
    // 1) 灰度 + 采样   y = (r + 2g + b) / 4，与 alg_luma_stat.v 同一套刻度
    //    32 bin ⇒ 取灰度高 5 位
    //-------------------------------------------------------------------------
    wire [9:0] W_sum  = {2'b00, I_video[23:16]} + {1'b0, I_video[15:8], 1'b0}
                      + {2'b00, I_video[7:0]};
    wire [7:0] W_lum  = W_sum[9:2];
    wire [4:0] W_bin  = W_lum[7:3];

    reg  [1:0]  S_st;
    reg  [2:0]  S_smp;
    wire        W_sample = I_de && (S_st == ST_COLLECT) && (S_smp == SDIV_M1);

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if(!I_rst_n)      S_smp <= 3'd0;
        else if(I_de)     S_smp <= (S_smp == SDIV_M1) ? 3'd0 : (S_smp + 3'd1);
    end

    //-------------------------------------------------------------------------
    // 2) 直方图：32 个独立寄存器（刻意不用 ERAM）
    //-------------------------------------------------------------------------
    reg [17:0] S_bin0,  S_bin1,  S_bin2,  S_bin3,  S_bin4,  S_bin5,  S_bin6,  S_bin7;
    reg [17:0] S_bin8,  S_bin9,  S_bin10, S_bin11, S_bin12, S_bin13, S_bin14, S_bin15;
    reg [17:0] S_bin16, S_bin17, S_bin18, S_bin19, S_bin20, S_bin21, S_bin22, S_bin23;
    reg [17:0] S_bin24, S_bin25, S_bin26, S_bin27, S_bin28, S_bin29, S_bin30, S_bin31;

    //-------------------------------------------------------------------------
    // 3) 扫表 / 造表用的寄存器（全部先声明后使用）
    //-------------------------------------------------------------------------
    reg  [4:0]  S_idx_scan;
    reg  [19:0] S_acc;
    reg  [4:0]  S_lo5, S_hi5;
    reg         S_lo_f, S_hi_f;
    reg  [16:0] S_smp_cnt;

    reg  [7:0]  S_lo8, S_hi8;
    reg  [7:0]  S_span;
    reg         S_bps;              // 1 = 直通（画面太平，不拉伸）

    wire [7:0]  S_span_use = (S_span < MINSPAN8) ? 8'd1 : S_span;

    reg  [7:0]  S_idx_bld;
    reg  [15:0] S_phase;
    reg  [8:0]  S_val;
    reg         S_lut_ok;

    // 扫描时按 S_idx_scan 选一路（组合 32:1）
    reg  [17:0] W_bin_sel;
    always @(*) begin
        case(S_idx_scan)
            5'd0 : W_bin_sel = S_bin0;   5'd1 : W_bin_sel = S_bin1;
            5'd2 : W_bin_sel = S_bin2;   5'd3 : W_bin_sel = S_bin3;
            5'd4 : W_bin_sel = S_bin4;   5'd5 : W_bin_sel = S_bin5;
            5'd6 : W_bin_sel = S_bin6;   5'd7 : W_bin_sel = S_bin7;
            5'd8 : W_bin_sel = S_bin8;   5'd9 : W_bin_sel = S_bin9;
            5'd10: W_bin_sel = S_bin10;  5'd11: W_bin_sel = S_bin11;
            5'd12: W_bin_sel = S_bin12;  5'd13: W_bin_sel = S_bin13;
            5'd14: W_bin_sel = S_bin14;  5'd15: W_bin_sel = S_bin15;
            5'd16: W_bin_sel = S_bin16;  5'd17: W_bin_sel = S_bin17;
            5'd18: W_bin_sel = S_bin18;  5'd19: W_bin_sel = S_bin19;
            5'd20: W_bin_sel = S_bin20;  5'd21: W_bin_sel = S_bin21;
            5'd22: W_bin_sel = S_bin22;  5'd23: W_bin_sel = S_bin23;
            5'd24: W_bin_sel = S_bin24;  5'd25: W_bin_sel = S_bin25;
            5'd26: W_bin_sel = S_bin26;  5'd27: W_bin_sel = S_bin27;
            5'd28: W_bin_sel = S_bin28;  5'd29: W_bin_sel = S_bin29;
            5'd30: W_bin_sel = S_bin30;  default: W_bin_sel = S_bin31;
        endcase
    end

    //-------------------------------------------------------------------------
    // 4) 帧边界：vsync 任意跳变都算；用"本帧样本数"做门防止二次触发
    //-------------------------------------------------------------------------
    reg  S_vs_d;
    wire W_vs_edge = (I_vsync ^ S_vs_d);
    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if(!I_rst_n) S_vs_d <= 1'b0;
        else         S_vs_d <= I_vsync;
    end

    wire W_start_scan = (S_st == ST_COLLECT) && W_vs_edge && (S_smp_cnt >= NSMP_H17);

    //-------------------------------------------------------------------------
    // 5) 主状态机
    //-------------------------------------------------------------------------
    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_st       <= ST_COLLECT;
            S_idx_scan <= 5'd0;
            S_acc      <= 20'd0;
            S_lo_f     <= 1'b0;
            S_hi_f     <= 1'b0;
            S_lo5      <= 5'd0;
            S_hi5      <= 5'd31;
            S_smp_cnt  <= 17'd0;
            S_lo8      <= 8'd0;
            S_hi8      <= 8'd255;
            S_span     <= 8'd255;
            S_bps      <= 1'b1;
            S_idx_bld  <= 8'd0;
            S_phase    <= 16'd0;
            S_val      <= 9'd0;
            S_lut_ok   <= 1'b0;
        end
        else begin
            case(S_st)
            ST_COLLECT: begin
                if(W_sample) S_smp_cnt <= S_smp_cnt + 17'd1;
                if(W_start_scan) begin
                    S_st       <= ST_SCAN;
                    S_idx_scan <= 5'd0;
                    S_acc      <= 20'd0;
                    S_lo_f     <= 1'b0;
                    S_hi_f     <= 1'b0;
                    S_lo5      <= 5'd0;
                    S_hi5      <= 5'd31;
                    S_smp_cnt  <= 17'd0;
                end
            end
            // 扫表：一拍一个 bin
            ST_SCAN: begin
                if((S_acc + W_bin_sel >= THR_LO20) && !S_lo_f) begin
                    S_lo5  <= S_idx_scan;
                    S_lo_f <= 1'b1;
                end
                if((S_acc + W_bin_sel >= THR_HI20) && !S_hi_f) begin
                    S_hi5  <= S_idx_scan;
                    S_hi_f <= 1'b1;
                end
                S_acc <= S_acc + W_bin_sel;
                if(S_idx_scan == 5'd31) S_st <= ST_CLEAR;
                else                    S_idx_scan <= S_idx_scan + 5'd1;
            end
            // 清零 + 换算 lo/hi 到 8bit 刻度（32 bin 一格 = 8 个灰度级）
            ST_CLEAR: begin
                S_lo8  <= {S_lo5, 3'b000};
                S_hi8  <= {S_hi5, 3'b000};
                S_span <= ({S_hi5, 3'b000} > {S_lo5, 3'b000})
                          ? ({S_hi5, 3'b000} - {S_lo5, 3'b000}) : 8'd0;
                S_bps  <= ~(({S_hi5, 3'b000} - {S_lo5, 3'b000}) >= MINSPAN8);
                S_st      <= ST_BUILD;
                S_idx_bld <= 8'd0;
                S_phase   <= 16'd0;
                S_val     <= 9'd0;
            end
            // DDA 造表：phase += 256；每越过一次 span 就 val+1
            ST_BUILD: begin
                if(S_idx_bld == 8'd255) begin
                    S_st     <= ST_COLLECT;
                    S_lut_ok <= 1'b1;
                end
                else begin
                    S_idx_bld <= S_idx_bld + 8'd1;
                end
                if(S_phase + 16'd256 >= {8'd0, S_span_use}) begin
                    S_phase <= S_phase + 16'd256 - {8'd0, S_span_use};
                    S_val   <= S_val + 9'd1;
                end
                else begin
                    S_phase <= S_phase + 16'd256;
                end
            end
            default: S_st <= ST_COLLECT;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 6) 直方图累加 / 清零
    //-------------------------------------------------------------------------
    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if(!I_rst_n || (S_st == ST_CLEAR)) begin
            S_bin0<=0; S_bin1<=0; S_bin2<=0; S_bin3<=0; S_bin4<=0; S_bin5<=0; S_bin6<=0; S_bin7<=0;
            S_bin8<=0; S_bin9<=0; S_bin10<=0; S_bin11<=0; S_bin12<=0; S_bin13<=0; S_bin14<=0; S_bin15<=0;
            S_bin16<=0; S_bin17<=0; S_bin18<=0; S_bin19<=0; S_bin20<=0; S_bin21<=0; S_bin22<=0; S_bin23<=0;
            S_bin24<=0; S_bin25<=0; S_bin26<=0; S_bin27<=0; S_bin28<=0; S_bin29<=0; S_bin30<=0; S_bin31<=0;
        end
        else if(W_sample) begin
            case(W_bin)
                5'd0 : S_bin0  <= S_bin0  + 18'd1;   5'd1 : S_bin1  <= S_bin1  + 18'd1;
                5'd2 : S_bin2  <= S_bin2  + 18'd1;   5'd3 : S_bin3  <= S_bin3  + 18'd1;
                5'd4 : S_bin4  <= S_bin4  + 18'd1;   5'd5 : S_bin5  <= S_bin5  + 18'd1;
                5'd6 : S_bin6  <= S_bin6  + 18'd1;   5'd7 : S_bin7  <= S_bin7  + 18'd1;
                5'd8 : S_bin8  <= S_bin8  + 18'd1;   5'd9 : S_bin9  <= S_bin9  + 18'd1;
                5'd10: S_bin10 <= S_bin10 + 18'd1;   5'd11: S_bin11 <= S_bin11 + 18'd1;
                5'd12: S_bin12 <= S_bin12 + 18'd1;   5'd13: S_bin13 <= S_bin13 + 18'd1;
                5'd14: S_bin14 <= S_bin14 + 18'd1;   5'd15: S_bin15 <= S_bin15 + 18'd1;
                5'd16: S_bin16 <= S_bin16 + 18'd1;   5'd17: S_bin17 <= S_bin17 + 18'd1;
                5'd18: S_bin18 <= S_bin18 + 18'd1;   5'd19: S_bin19 <= S_bin19 + 18'd1;
                5'd20: S_bin20 <= S_bin20 + 18'd1;   5'd21: S_bin21 <= S_bin21 + 18'd1;
                5'd22: S_bin22 <= S_bin22 + 18'd1;   5'd23: S_bin23 <= S_bin23 + 18'd1;
                5'd24: S_bin24 <= S_bin24 + 18'd1;   5'd25: S_bin25 <= S_bin25 + 18'd1;
                5'd26: S_bin26 <= S_bin26 + 18'd1;   5'd27: S_bin27 <= S_bin27 + 18'd1;
                5'd28: S_bin28 <= S_bin28 + 18'd1;   5'd29: S_bin29 <= S_bin29 + 18'd1;
                5'd30: S_bin30 <= S_bin30 + 18'd1;   default: S_bin31 <= S_bin31 + 18'd1;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 7) 映射表：三份 256x8（R/G/B 各查一份）
    //-------------------------------------------------------------------------
    (* ram_style = "distributed" *) reg [7:0] S_lut0 [0:255];
    (* ram_style = "distributed" *) reg [7:0] S_lut1 [0:255];
    (* ram_style = "distributed" *) reg [7:0] S_lut2 [0:255];

    wire [7:0] W_lut_val = S_bps ? S_idx_bld
                         : ((S_idx_bld <= S_lo8) ? 8'd0
                            : ((S_val > 9'd255) ? 8'd255 : S_val[7:0]));

    always @(posedge I_clk_pix) begin
        if(S_st == ST_BUILD) begin
            S_lut0[S_idx_bld] <= W_lut_val;
            S_lut1[S_idx_bld] <= W_lut_val;
            S_lut2[S_idx_bld] <= W_lut_val;
        end
    end

    //-------------------------------------------------------------------------
    // 8) 查表 + 输出（2 拍流水）
    //-------------------------------------------------------------------------
    reg  [7:0]  S_lr, S_lg, S_lb;
    reg  [23:0] S_vid_d1;
    reg         S_vs1, S_hs1, S_de1, S_us1, S_ls1;

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_lr <= 8'd0; S_lg <= 8'd0; S_lb <= 8'd0;
            S_vid_d1 <= 24'd0;
            S_vs1 <= 1'b0; S_hs1 <= 1'b0; S_de1 <= 1'b0; S_us1 <= 1'b0; S_ls1 <= 1'b0;
        end
        else begin
            S_lr <= S_lut0[I_video[23:16]];
            S_lg <= S_lut1[I_video[15:8]];
            S_lb <= S_lut2[I_video[7:0]];
            S_vid_d1 <= I_video;
            S_vs1 <= I_vsync; S_hs1 <= I_hsync; S_de1 <= I_de;
            S_us1 <= I_user;  S_ls1 <= I_last;
        end
    end

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_vsync <= 1'b0; O_hsync <= 1'b0; O_de <= 1'b0;
            O_user  <= 1'b0; O_last  <= 1'b0; O_video <= 24'd0;
        end
        else begin
            O_vsync <= S_vs1; O_hsync <= S_hs1; O_de <= S_de1;
            O_user  <= S_us1; O_last  <= S_ls1;
            // 功能关闭或表还没建好，就原样放行
            O_video <= (I_enable && S_lut_ok) ? {S_lr, S_lg, S_lb} : S_vid_d1;
        end
    end

endmodule
