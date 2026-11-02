//=============================================================================
//  dsp_mode_filter -- 第一档本地点运算与共享邻域滤镜
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk，HDMI 像素域
//  复位     : I_rst_n，异步低有效；行缓存内容不复位
//  参数     : P_TIER1、P_WIDTH、BIN_TH、BRIGHTNESS、SAT_FACTOR、P_DSP_ARITHMETIC
//  验证状态 : 见同名 spec 与第一档交付验证报告，上板单独记录
//=============================================================================
// P_TIER1=1: 0/3/6/8/2/9/11/12/7/18，全部七级寄存；帧首锁存模式和参数。
// Sobel/浮雕/卡通共享 trailing 3x3 灰度窗口与两行同步块 RAM。
// P_TIER1=0: 保留历史三级逐像素分支，仅供非当前 profile 兼容。
// P_DSP_ARITHMETIC=1: 仅替换四个饱和重建常数乘法；不新增流水级。
// 灰度/YUV/棕褐/饱和度/热力图与卡通系数参考 v1.13 isp_mode_filter。
module dsp_mode_filter #(
    parameter [7:0]        BIN_TH     = 8'd128,   // 模式 2 二值化阈值
    parameter [7:0]        CARTOON_TH = 8'd80,    // 模式 18 卡通描边阈值（默认）
    parameter signed [8:0] BRIGHTNESS = 9'sd64,   // 模式 7 亮度（-255..+255）
    parameter [7:0]        SAT_FACTOR = 8'd192,   // 模式 8 饱和度（64/128/192）
    parameter integer P_RUNTIME_PARAMETERS = 0,
    parameter integer P_TIER1 = 0,
    parameter integer P_WIDTH = 1280,
    parameter integer P_DSP_ARITHMETIC = 0
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [4:0]  I_mode,       // 0..9（见文件头映射表）
    input  wire        I_vsync,
    input  wire        I_hsync,
    input  wire        I_de,
    input  wire        I_user,
    input  wire        I_last,
    input  wire [23:0] I_video,
    input  wire signed [8:0] I_brightness,
    input  wire [7:0] I_sat_factor,
    input  wire [7:0] I_bin_th,       // 模式 2/17 二值化阈值（运行时）
    input  wire [7:0] I_cartoon_th,   // 模式 18 卡通描边阈值（运行时）
    output wire        O_vsync,
    output wire        O_hsync,
    output wire        O_de,
    output wire        O_user,
    output wire        O_last,
    output wire [23:0] O_video,
    output wire        O_mask        // 模式 17 一次二值掩膜（对齐 O_video 时序）
);

    localparam integer P_STAGES = 3;   // 仅 legacy point 分支；P_TIER1 使用显式七级寄存

    //-------------------------------------------------------------------------
    // 模式编码（本模块编号）
    //-------------------------------------------------------------------------
    localparam [4:0] P_M_ORIG   = 5'd0,
                     P_M_GRAY   = 5'd1,
                     P_M_BIN    = 5'd2,
                     P_M_NEG    = 5'd3,
                     P_M_RED    = 5'd4,
                     P_M_POST   = 5'd5,
                     P_M_SEPIA  = 5'd6,
                     P_M_BRIGHT = 5'd7,
                     P_M_SAT    = 5'd8,
                     // ★ v1.13 numbering: 9 = Sobel (the EG4D slave owns it in the dual-board
    //   build), 12 = heat map.  This filter originally put heat at 9, which
    //   collided with the slave's mode 9 and made the user's "mode 9" show
    //   Sobel instead of heat.  Keep the master's numbering aligned with v1.13.
    P_M_HEAT   = 5'd12;

    reg [4:0] S_frame_mode;
    reg signed [8:0] S_frame_brightness;
    reg [7:0] S_frame_saturation;
    reg [7:0] S_frame_bin_th;
    reg [7:0] S_frame_cartoon_th;
    wire [4:0] W_effect_mode = P_TIER1 ? (I_user ? I_mode : S_frame_mode) : I_mode;
    wire [7:0] W_sat_factor = P_RUNTIME_PARAMETERS ?
        (P_TIER1 ? (I_user ? I_sat_factor : S_frame_saturation) : I_sat_factor) : SAT_FACTOR;
    wire signed [8:0] W_brightness = P_RUNTIME_PARAMETERS ?
        (P_TIER1 ? (I_user ? I_brightness : S_frame_brightness) : I_brightness) : BRIGHTNESS;
    wire [7:0] W_bin_th = P_RUNTIME_PARAMETERS ?
        (P_TIER1 ? (I_user ? I_bin_th : S_frame_bin_th) : I_bin_th) : BIN_TH;
    wire [7:0] W_cartoon_th = P_RUNTIME_PARAMETERS ?
        (P_TIER1 ? (I_user ? I_cartoon_th : S_frame_cartoon_th) : I_cartoon_th) : CARTOON_TH;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_frame_mode <= 5'd0;
            S_frame_brightness <= 9'sd64;
            S_frame_saturation <= 8'd128;
            S_frame_bin_th <= 8'd128;
            S_frame_cartoon_th <= 8'd80;
        end else if (I_de && I_user) begin
            S_frame_mode <= I_mode;
            S_frame_brightness <= I_brightness;
            S_frame_saturation <= I_sat_factor;
            S_frame_bin_th <= I_bin_th;
            S_frame_cartoon_th <= I_cartoon_th;
        end
    end

    //=========================================================================
    // 小工具函数
    //=========================================================================
    function [7:0] clamp8;
        input signed [23:0] v;
        begin
            if      (v > 24'sd255) clamp8 = 8'hFF;
            else if (v < 24'sd0)   clamp8 = 8'h00;
            else                   clamp8 = v[7:0];
        end
    endfunction

    // gray = (77R + 150G + 29B) >> 8
    //   77 = 64+8+4+1 ; 150 = 128+16+4+2 ; 29 = 16+8+4+1，全移位加法，无乘法器
    function [7:0] gray_of;
        input [7:0] r, g, b;
        reg [15:0] pr, pg, pb, s;
        begin
            pr = ({8'd0, r} << 6) + ({8'd0, r} << 3) +
                 ({8'd0, r} << 2) + {8'd0, r};
            pg = ({8'd0, g} << 7) + ({8'd0, g} << 4) +
                 ({8'd0, g} << 2) + ({8'd0, g} << 1);
            pb = ({8'd0, b} << 4) + ({8'd0, b} << 3) +
                 ({8'd0, b} << 2) + {8'd0, b};
            s  = (pr + pg) + pb;
            gray_of = s[15:8];
        end
    endfunction

    // YUV（系数逐字取自 v1.13）
    function [7:0] yuv_y;
        input [7:0] r, g, b;
        reg [17:0] er, eg, eb, yr, yg, yb, s;
        begin
            er = {10'd0, r}; eg = {10'd0, g}; eb = {10'd0, b};
            yr = (er << 6) + (er << 1);                       //  66*r
            yg = (eg << 7) + eg;                              // 129*g
            yb = (eb << 4) + (eb << 3) + eb;                  //  25*b
            s  = (yr + yg) + (yb + 18'd128);
            yuv_y = s[15:8] + 8'd16;
        end
    endfunction

    function signed [8:0] yuv_u;
        input [7:0] r, g, b;
        reg signed [17:0] er, eg, eb, ur, ug, ub, s;
        begin
            er = {10'd0, r}; eg = {10'd0, g}; eb = {10'd0, b};
            ur = (er <<< 5) + (er <<< 2) + (er <<< 1);        //  38*r
            ug = (eg <<< 6) + (eg <<< 3) + (eg <<< 1);        //  74*g
            ub = (eb <<< 6) + (eb <<< 5) + (eb <<< 4);        // 112*b
            s  = (ub - (ur + ug)) + 18'sd128;
            s  = s >>> 8;
            yuv_u = s[8:0] + 9'sd128;
        end
    endfunction

    function signed [8:0] yuv_v;
        input [7:0] r, g, b;
        reg signed [17:0] er, eg, eb, vr, vg, vb, s;
        begin
            er = {10'd0, r}; eg = {10'd0, g}; eb = {10'd0, b};
            vr = (er <<< 6) + (er <<< 5) + (er <<< 4);        // 112*r
            vg = (eg <<< 6) + (eg <<< 4) + (eg <<< 3)
               + (eg <<< 2) + (eg <<< 1);                     //  94*g
            vb = (eb <<< 4) + (eb <<< 1);                     //  18*b
            s  = (vr - (vg + vb)) + 18'sd128;
            s  = s >>> 8;
            yuv_v = s[8:0] + 9'sd128;
        end
    endfunction

    // 饱和度色度缩放：off * SAT_FACTOR >> 7
    //   SAT_FACTOR 只有 64/128/192 三档，写成等价移位，不用乘法器：
    //     64 -> off/2 ; 128 -> off ; 192 -> off + off/2
    //   ★ 这里是"除以 2 的幂"（v1.13 是先乘后 >>>7），不是左移；
    //     写成左移会把色度整体放大 128 倍，是本文件最容易写错的一处。
    function signed [8:0] sat_scale;
        input signed [8:0] off;
        input [7:0] factor;
        reg signed [17:0] m;
        begin
            case (factor)
                8'd64  : m = off >>> 1;
                8'd192 : m = off + (off >>> 1);
                default: m = off;                     // 128
            endcase
            if      (m > 18'sd127)  sat_scale = 9'sd127;
            else if (m < -18'sd128) sat_scale = -9'sd128;
            else                    sat_scale = m[8:0];
        end
    endfunction

    // 饱和度重建；默认全部移位加减，候选参数只替换 298/409/100/208。
    // 每个 helper 都先在更宽的 signed 暂存中相乘，再取与原 shift/add
    // 相同的低位；不增加寄存器或流水级。P_DSP_ARITHMETIC=0 是基线。
    function signed [17:0] scale_luma;
        input signed [17:0] I_value;
        reg signed [35:0] S_product;
        begin
            if (P_DSP_ARITHMETIC != 0) begin
                S_product = I_value * 36'sd298;
                scale_luma = S_product[17:0];
            end else begin
                scale_luma = (I_value <<< 8) + (I_value <<< 5) +
                             (I_value <<< 3) + (I_value <<< 1);
            end
        end
    endfunction

    function signed [23:0] scale_red_chroma;
        input signed [23:0] I_value;
        reg signed [47:0] S_product;
        begin
            if (P_DSP_ARITHMETIC != 0) begin
                S_product = I_value * 48'sd409;
                scale_red_chroma = S_product[23:0];
            end else begin
                scale_red_chroma = (I_value <<< 8) + (I_value <<< 7) +
                                   (I_value <<< 4) + (I_value <<< 3) + I_value;
            end
        end
    endfunction

    function signed [23:0] scale_green_u_chroma;
        input signed [23:0] I_value;
        reg signed [47:0] S_product;
        begin
            if (P_DSP_ARITHMETIC != 0) begin
                S_product = I_value * 48'sd100;
                scale_green_u_chroma = S_product[23:0];
            end else begin
                scale_green_u_chroma = (I_value <<< 6) + (I_value <<< 5) +
                                       (I_value <<< 2);
            end
        end
    endfunction

    function signed [23:0] scale_green_v_chroma;
        input signed [23:0] I_value;
        reg signed [47:0] S_product;
        begin
            if (P_DSP_ARITHMETIC != 0) begin
                S_product = I_value * 48'sd208;
                scale_green_v_chroma = S_product[23:0];
            end else begin
                scale_green_v_chroma = (I_value <<< 7) + (I_value <<< 6) +
                                       (I_value <<< 4);
            end
        end
    endfunction

    function [23:0] sat_recon;
        input signed [17:0] yterm;     // (Y-16)*298
        input signed [8:0]  us;
        input signed [8:0]  vs;
        reg signed [23:0] usw, vsw, uterm, vterm, bterm, rs, gs, bs, vt;
        begin
            usw = us; vsw = vs;
            uterm = scale_green_u_chroma(usw);                        // 100
            vterm = scale_green_v_chroma(vsw);                        // 208
            bterm = (usw <<< 9) + (usw <<< 2);                        // 516
            vt    = scale_red_chroma(vsw);                            // 409
            rs = yterm + vt + 24'sd128;
            gs = (yterm + 24'sd128) - (uterm + vterm);
            bs = (yterm + 24'sd128) + bterm;
            sat_recon = { clamp8(rs >>> 8), clamp8(gs >>> 8), clamp8(bs >>> 8) };
        end
    endfunction

    // (Y-16)*298，移位加法
    function signed [17:0] yterm_of;
        input [7:0] y;
        reg signed [17:0] yy;
        begin
            yy = $signed({10'd0, y}) - 18'sd16;
            yterm_of = scale_luma(yy);
        end
    endfunction

    // sepia 三通道（系数逐字取自 v1.13）
    function [16:0] sepia_r;
        input [7:0] r, g, b;
        reg [16:0] pr, pg, pb;
        begin
            pr = (({9'd0,r} << 6) + ({9'd0,r} << 5)) + (({9'd0,r} << 2) + {9'd0,r});
            pg = (({9'd0,g} << 7) + ({9'd0,g} << 6)) + (({9'd0,g} << 2) + {9'd0,g});
            pb = ({9'd0,b} << 5) + ({9'd0,b} << 4);
            sepia_r = (pr + pg) + pb;
        end
    endfunction

    function [16:0] sepia_g;
        input [7:0] r, g, b;
        reg [16:0] pr, pg, pb;
        begin
            pr = (({9'd0,r} << 6) + ({9'd0,r} << 4)) + (({9'd0,r} << 3) + {9'd0,r});
            pg = ({9'd0,g} << 7) + ({9'd0,g} << 5) + ({9'd0,g} << 4);
            pb = (({9'd0,b} << 5) + ({9'd0,b} << 3)) + (({9'd0,b} << 1) + {9'd0,b});
            sepia_g = (pr + pg) + pb;
        end
    endfunction

    function [16:0] sepia_b;
        input [7:0] r, g, b;
        reg [16:0] pr, pg, pb;
        begin
            pr = ({9'd0,r} << 6) + ({9'd0,r} << 2) + ({9'd0,r} << 1);
            pg = ({9'd0,g} << 7) + ({9'd0,g} << 3) + {9'd0,g};
            pb = ({9'd0,b} << 5) + ({9'd0,b} << 1);
            sepia_b = (pr + pg) + pb;
        end
    endfunction

    function [7:0] sepia_sat;
        input [16:0] v;
        begin sepia_sat = (v > 17'd65280) ? 8'hFF : v[15:8]; end
    endfunction

    // 热力图（参考 LUT，逐字取自 v1.13）
    function [23:0] px_heat;
        input [7:0] g;
        reg [7:0] rr, gg, bb;
        begin
            if (g < 8'd64) begin
                rr = 8'd0;              gg = g << 2;                        bb = 8'd255;
            end else if (g < 8'd128) begin
                rr = 8'd0;              gg = 8'd255; bb = 8'd255 - ((g - 8'd64) << 2);
            end else if (g < 8'd192) begin
                rr = (g - 8'd128) << 2; gg = 8'd255; bb = 8'd0;
            end else begin
                rr = 8'd255;            gg = 8'd255 - ((g - 8'd192) << 2); bb = 8'd0;
            end
            px_heat = {rr, gg, bb};
        end
    endfunction

    //=========================================================================
    // 输入拆通道
    //=========================================================================
    wire [7:0] W_r = I_video[23:16];
    wire [7:0] W_g = I_video[15: 8];
    wire [7:0] W_b = I_video[7:  0];

    //=========================================================================
    // S1：灰度 / YUV / 棕褐部分积
    //=========================================================================
    reg [7:0]  S1_gray;
    reg [7:0]  S1_yy;
    reg signed [8:0] S1_uu, S1_vv;
    reg [16:0] S1_sep_r, S1_sep_g, S1_sep_b;
    reg [23:0] S1_rgb;
    reg [4:0]  S1_mode;
    reg signed [8:0] S_first_brightness;
    reg [7:0] S_first_saturation;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S1_gray <= 8'd0;
            S1_yy   <= 8'd0;
            S1_uu   <= 9'sd0;
            S1_vv   <= 9'sd0;
            S1_sep_r <= 17'd0; S1_sep_g <= 17'd0; S1_sep_b <= 17'd0;
            S1_rgb  <= 24'd0;
            S1_mode <= 5'd0;
            S_first_brightness <= 9'sd64;
            S_first_saturation <= 8'd128;
        end
        else begin
            S1_gray  <= gray_of(W_r, W_g, W_b);
            S1_yy    <= yuv_y(W_r, W_g, W_b);
            S1_uu    <= yuv_u(W_r, W_g, W_b);
            S1_vv    <= yuv_v(W_r, W_g, W_b);
            S1_sep_r <= sepia_r(W_r, W_g, W_b);
            S1_sep_g <= sepia_g(W_r, W_g, W_b);
            S1_sep_b <= sepia_b(W_r, W_g, W_b);
            S1_rgb   <= I_video;
            S1_mode  <= W_effect_mode;
            S_first_brightness <= W_brightness;
            S_first_saturation <= W_sat_factor;
        end
    end

    //=========================================================================
    // S2：色度缩放 / 亮度项 / 各候选模式
    //=========================================================================
    wire [23:0] W_sepia2 = { sepia_sat(S1_sep_r),
                             sepia_sat(S1_sep_g),
                             sepia_sat(S1_sep_b) };

    wire signed [23:0] W_br_r = $signed({16'd0, S1_rgb[23:16]}) + S_first_brightness;
    wire signed [23:0] W_br_g = $signed({16'd0, S1_rgb[15: 8]}) + S_first_brightness;
    wire signed [23:0] W_br_b = $signed({16'd0, S1_rgb[7:  0]}) + S_first_brightness;
    wire [23:0] W_bright2 = { clamp8(W_br_r), clamp8(W_br_g), clamp8(W_br_b) };

    reg signed [8:0]  S2_us, S2_vs;
    reg signed [17:0] S2_yterm;
    reg [23:0] S2_rgb;
    reg [23:0] S2_sepia;
    reg [23:0] S2_bright;
    reg [23:0] S2_heat;
    reg [7:0]  S2_gray;
    reg [4:0]  S2_mode;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S2_us <= 9'sd0; S2_vs <= 9'sd0; S2_yterm <= 18'sd0;
            S2_rgb <= 24'd0; S2_sepia <= 24'd0; S2_bright <= 24'd0;
            S2_heat <= 24'd0; S2_gray <= 8'd0; S2_mode <= 5'd0;
        end
        else begin
            S2_us    <= sat_scale(S1_uu - 9'sd128, S_first_saturation);
            S2_vs    <= sat_scale(S1_vv - 9'sd128, S_first_saturation);
            S2_yterm <= yterm_of(S1_yy);
            S2_rgb   <= S1_rgb;
            S2_sepia <= W_sepia2;
            S2_bright<= W_bright2;
            S2_heat  <= px_heat(S1_gray);
            S2_gray  <= S1_gray;
            S2_mode  <= S1_mode;
        end
    end

    //=========================================================================
    // S3：饱和度重建 + 模式选择 + 输出寄存
    //=========================================================================
    wire [23:0] W_sat3 = sat_recon(S2_yterm, S2_us, S2_vs);
    wire [23:0] W_mono = {S2_gray, S2_gray, S2_gray};
    reg [23:0] S_point_video;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_point_video <= 24'd0;
        else begin
            case (S2_mode)
                P_M_ORIG   : S_point_video <= S2_rgb;
                P_M_GRAY   : S_point_video <= W_mono;
                P_M_BIN    : S_point_video <= (S2_gray >= W_bin_th) ? 24'hFFFFFF : 24'h000000;
                P_M_NEG    : S_point_video <= ~S2_rgb;
                P_M_RED    : S_point_video <= {S2_rgb[23:16], 8'h00, 8'h00};
                P_M_POST   : S_point_video <= S2_rgb & 24'hE0E0E0;
                P_M_SEPIA  : S_point_video <= S2_sepia;
                P_M_BRIGHT : S_point_video <= S2_bright;
                P_M_SAT    : S_point_video <= W_sat3;
                P_M_HEAT   : S_point_video <= S2_heat;
                default    : S_point_video <= S2_rgb;
            endcase
        end
    end

    //=========================================================================
    // 时序延迟：与 O_video 完全同延迟（正好 P_STAGES 拍）
    //
    //   ★ 这里刻意**不再加一级输出寄存器**：S_tsync[P_STAGES-1] 直接组合输出。
    //     像素路径是 S1 -> S2 -> S3(=O_video) 共 P_STAGES 级，
    //     若时序再额外打一拍就会变成"时序晚 1 拍、像素早 1 拍"，
    //     叠加层会整体错位一行（这个错位由 sim/tb_dsp_mode_filter.v 的
    //     de/pixel 配对检查抓到过）。
    //=========================================================================
    reg [4:0] S_tsync [0:P_STAGES-1];   // {vsync,hsync,de,user,last}
    integer   S_i;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (S_i = 0; S_i < P_STAGES; S_i = S_i + 1)
                S_tsync[S_i] <= 5'd0;
        end
        else begin
            S_tsync[0] <= {I_vsync, I_hsync, I_de, I_user, I_last};
            for (S_i = 1; S_i < P_STAGES; S_i = S_i + 1)
                S_tsync[S_i] <= S_tsync[S_i-1];
        end
    end


    // Shared trailing 3x3 aperture: (x-2..x, y-2..y). No line/frame delay
    // is added to sync; top two rows / left two columns are explicit borders.
    // RAM has a synchronous read and a one-cycle-later write at the prior
    // address. There is no read/write collision and no reset on storage.
    function [7:0] window_min;
        input [7:0] a, b;
        begin window_min = (a < b) ? a : b; end
    endfunction
    function [7:0] window_max;
        input [7:0] a, b;
        begin window_max = (a > b) ? a : b; end
    endfunction
    function [23:0] cartoon_colour;
        input [23:0] rgb;
        input [7:0] gray, level;
        reg signed [9:0] S_chroma_red, S_chroma_green, S_chroma_blue;
        reg signed [23:0] rr, gg, bb;
        begin
            S_chroma_red = $signed({1'b0,rgb[23:16]}) - $signed({1'b0,gray});
            S_chroma_green = $signed({1'b0,rgb[15:8]}) - $signed({1'b0,gray});
            S_chroma_blue = $signed({1'b0,rgb[7:0]}) - $signed({1'b0,gray});
            S_chroma_red = {S_chroma_red[9:4],4'b0000};
            S_chroma_green = {S_chroma_green[9:4],4'b0000};
            S_chroma_blue = {S_chroma_blue[9:4],4'b0000};
            rr = $signed({16'd0,level}) + S_chroma_red;
            gg = $signed({16'd0,level}) + S_chroma_green;
            bb = $signed({16'd0,level}) + S_chroma_blue;
            cartoon_colour = {clamp8(rr),clamp8(gg),clamp8(bb)};
        end
    endfunction

    generate
    if (P_TIER1 != 0) begin : g_priority_effects
        (* force_ram *) (* ram_style = "bram" *) reg [15:0] S_line_memory [0:2047];
        reg [15:0] S_line_word;
        reg [10:0] S_input_x;
        reg [9:0] S_input_y;
        reg [10:0] S_sample_x;
        reg [9:0] S_sample_y;
        reg [10:0] S_read_x;
        reg [7:0] S_read_gray;
        reg S_read_de;
        reg S_read_window_valid;
        reg [7:0] S_top_left, S_top_middle, S_top_right;
        reg [7:0] S_middle_left, S_middle_middle, S_middle_right;
        reg [7:0] S_bottom_left, S_bottom_middle, S_bottom_right;
        reg S_window_valid;
        reg [9:0] S_horizontal_positive, S_horizontal_negative;
        reg [9:0] S_vertical_positive, S_vertical_negative;
        reg [9:0] S_emboss_positive, S_emboss_negative;
        reg [9:0] S_top_sum, S_middle_sum, S_bottom_sum;
        reg [7:0] S_top_min, S_middle_min, S_bottom_min;
        reg [7:0] S_top_max, S_middle_max, S_bottom_max;
        reg S_partial_valid;
        reg signed [10:0] S_horizontal_gradient, S_vertical_gradient;
        reg signed [10:0] S_emboss_gradient;
        reg [11:0] S_neighbour_sum;
        reg [7:0] S_neighbour_min, S_neighbour_max;
        reg [7:0] S_neighbour_min_d, S_neighbour_max_d, S_binsobel;
        reg S_mask;
        reg S_gradient_valid;
        reg [7:0] S_sobel, S_emboss, S_quantized_luma;
        reg S_cartoon_edge, S_result_valid;
        reg [23:0] S_point_delay [0:2];
        reg [23:0] S_rgb_delay [0:5];
        reg [7:0] S_gray_delay [0:5];
        reg [4:0] S_mode_delay [0:5];
        reg [4:0] S_sync_delay [0:6];
        reg [23:0] S_output_video;
        reg        S_mask_out;
        integer S_delay_index;

        wire [10:0] W_current_x = I_user ? 11'd0 : S_input_x;
        wire [9:0] W_current_y = I_user ? 10'd0 : S_input_y;
        wire [10:0] W_horizontal_absolute = S_horizontal_gradient[10] ?
            -S_horizontal_gradient : S_horizontal_gradient;
        wire [10:0] W_vertical_absolute = S_vertical_gradient[10] ?
            -S_vertical_gradient : S_vertical_gradient;
        wire [11:0] W_magnitude = {1'b0,W_horizontal_absolute} + {1'b0,W_vertical_absolute};
        wire [11:0] W_sum_th = ({4'd0,W_bin_th} << 3) + {4'd0,W_bin_th};  // 9*bin_th
        wire signed [23:0] W_emboss_wide = S_emboss_gradient;
        wire signed [23:0] W_emboss_shifted = (W_emboss_wide <<< 1) + 24'sd128;
        wire [17:0] W_mean_product = ({6'd0,S_neighbour_sum} << 5) +
            ({6'd0,S_neighbour_sum} << 4) + ({6'd0,S_neighbour_sum} << 3) +
            {6'd0,S_neighbour_sum};
        wire [7:0] W_local_contrast = S_neighbour_max - S_neighbour_min;
        wire [7:0] W_morph_pix = S_neighbour_max_d - S_neighbour_min_d;
        wire [23:0] W_cartoon_colour = cartoon_colour(S_rgb_delay[5], S_gray_delay[5], S_quantized_luma);

        always @(posedge I_clk) begin
            if (S_tsync[0][2]) S_line_word <= S_line_memory[S_sample_x];
            if (S_read_de) S_line_memory[S_read_x] <= {S_line_word[7:0],S_read_gray};
        end

        always @(posedge I_clk or negedge I_rst_n) begin
            if (!I_rst_n) begin
                S_input_x <= 11'd0;
                S_input_y <= 10'd0;
                S_sample_x <= 11'd0;
                S_sample_y <= 10'd0;
                S_read_x <= 11'd0;
                S_read_gray <= 8'd0;
                S_read_de <= 1'b0;
                S_read_window_valid <= 1'b0;
                S_top_left <= 8'd0; S_top_middle <= 8'd0; S_top_right <= 8'd0;
                S_middle_left <= 8'd0; S_middle_middle <= 8'd0; S_middle_right <= 8'd0;
                S_bottom_left <= 8'd0; S_bottom_middle <= 8'd0; S_bottom_right <= 8'd0;
                S_window_valid <= 1'b0;
            end else begin
                if (I_de) begin
                    S_sample_x <= W_current_x;
                    S_sample_y <= W_current_y;
                    if (I_last || (W_current_x == P_WIDTH-1)) begin
                        S_input_x <= 11'd0;
                        S_input_y <= W_current_y + 10'd1;
                    end else begin
                        S_input_x <= W_current_x + 11'd1;
                        S_input_y <= W_current_y;
                    end
                end
                S_read_de <= S_tsync[0][2];
                S_read_x <= S_sample_x;
                S_read_gray <= S1_gray;
                S_read_window_valid <= (S_sample_x >= 11'd2) && (S_sample_y >= 10'd2);
                S_window_valid <= S_read_de && S_read_window_valid;
                if (S_read_de) begin
                    if (S_read_x == 11'd0) begin
                        S_top_left <= 8'd0; S_top_middle <= 8'd0;
                        S_middle_left <= 8'd0; S_middle_middle <= 8'd0;
                        S_bottom_left <= 8'd0; S_bottom_middle <= 8'd0;
                    end else begin
                        S_top_left <= S_top_middle; S_top_middle <= S_top_right;
                        S_middle_left <= S_middle_middle; S_middle_middle <= S_middle_right;
                        S_bottom_left <= S_bottom_middle; S_bottom_middle <= S_bottom_right;
                    end
                    S_top_right <= S_line_word[15:8];
                    S_middle_right <= S_line_word[7:0];
                    S_bottom_right <= S_read_gray;
                end
            end
        end

        always @(posedge I_clk or negedge I_rst_n) begin
            if (!I_rst_n) begin
                S_horizontal_positive <= 10'd0; S_horizontal_negative <= 10'd0;
                S_vertical_positive <= 10'd0; S_vertical_negative <= 10'd0;
                S_emboss_positive <= 10'd0; S_emboss_negative <= 10'd0;
                S_top_sum <= 10'd0; S_middle_sum <= 10'd0; S_bottom_sum <= 10'd0;
                S_top_min <= 8'd0; S_middle_min <= 8'd0; S_bottom_min <= 8'd0;
                S_top_max <= 8'd0; S_middle_max <= 8'd0; S_bottom_max <= 8'd0;
                S_partial_valid <= 1'b0;
                S_horizontal_gradient <= 11'sd0; S_vertical_gradient <= 11'sd0;
                S_emboss_gradient <= 11'sd0;
                S_neighbour_sum <= 12'd0; S_neighbour_min <= 8'd0; S_neighbour_max <= 8'd0;
                S_neighbour_min_d <= 8'd0; S_neighbour_max_d <= 8'd0; S_binsobel <= 8'd0;
                S_mask <= 1'b0;
                S_gradient_valid <= 1'b0;
                S_sobel <= 8'd0; S_emboss <= 8'd0; S_quantized_luma <= 8'd0;
                S_cartoon_edge <= 1'b0; S_result_valid <= 1'b0;
            end else begin
                S_horizontal_positive <= {2'd0,S_top_right} + {1'd0,S_middle_right,1'b0} + {2'd0,S_bottom_right};
                S_horizontal_negative <= {2'd0,S_top_left} + {1'd0,S_middle_left,1'b0} + {2'd0,S_bottom_left};
                S_vertical_positive <= {2'd0,S_bottom_left} + {1'd0,S_bottom_middle,1'b0} + {2'd0,S_bottom_right};
                S_vertical_negative <= {2'd0,S_top_left} + {1'd0,S_top_middle,1'b0} + {2'd0,S_top_right};
                S_emboss_positive <= {2'd0,S_middle_right} + {2'd0,S_bottom_middle} + {1'd0,S_bottom_right,1'b0};
                S_emboss_negative <= {1'd0,S_top_left,1'b0} + {2'd0,S_top_middle} + {2'd0,S_middle_left};
                S_top_sum <= {2'd0,S_top_left} + {2'd0,S_top_middle} + {2'd0,S_top_right};
                S_middle_sum <= {2'd0,S_middle_left} + {2'd0,S_middle_middle} + {2'd0,S_middle_right};
                S_bottom_sum <= {2'd0,S_bottom_left} + {2'd0,S_bottom_middle} + {2'd0,S_bottom_right};
                S_top_min <= window_min(S_top_left,window_min(S_top_middle,S_top_right));
                S_middle_min <= window_min(S_middle_left,window_min(S_middle_middle,S_middle_right));
                S_bottom_min <= window_min(S_bottom_left,window_min(S_bottom_middle,S_bottom_right));
                S_top_max <= window_max(S_top_left,window_max(S_top_middle,S_top_right));
                S_middle_max <= window_max(S_middle_left,window_max(S_middle_middle,S_middle_right));
                S_bottom_max <= window_max(S_bottom_left,window_max(S_bottom_middle,S_bottom_right));
                S_partial_valid <= S_window_valid;
                S_horizontal_gradient <= $signed({1'b0,S_horizontal_positive}) - $signed({1'b0,S_horizontal_negative});
                S_vertical_gradient <= $signed({1'b0,S_vertical_positive}) - $signed({1'b0,S_vertical_negative});
                S_emboss_gradient <= $signed({1'b0,S_emboss_positive}) - $signed({1'b0,S_emboss_negative});
                S_neighbour_sum <= {2'd0,S_top_sum} + {2'd0,S_middle_sum} + {2'd0,S_bottom_sum};
                S_neighbour_min <= window_min(S_top_min,window_min(S_middle_min,S_bottom_min));
                S_neighbour_max <= window_max(S_top_max,window_max(S_middle_max,S_bottom_max));
                S_gradient_valid <= S_partial_valid;
                S_sobel <= !S_gradient_valid ? 8'd0 : ((W_magnitude > 12'd255) ? 8'd255 : W_magnitude[7:0]);
                S_emboss <= S_gradient_valid ? clamp8(W_emboss_shifted) : 8'd0;
                S_quantized_luma <= {W_mean_product[16:14],5'd0};
                S_cartoon_edge <= (W_local_contrast >= W_cartoon_th);
                S_neighbour_min_d <= S_neighbour_min;
                S_neighbour_max_d <= S_neighbour_max;
                S_binsobel <= !S_gradient_valid ? 8'd0 :
                    ((W_magnitude >= {4'd0,W_bin_th}) ? 8'd255 : 8'd0);
                S_mask <= !S_gradient_valid ? 1'b0 : (S_neighbour_sum >= W_sum_th);
                S_result_valid <= S_gradient_valid;
            end
        end

        always @(posedge I_clk or negedge I_rst_n) begin
            if (!I_rst_n) begin
                for (S_delay_index=0; S_delay_index<3; S_delay_index=S_delay_index+1)
                    S_point_delay[S_delay_index] <= 24'd0;
                for (S_delay_index=0; S_delay_index<6; S_delay_index=S_delay_index+1) begin
                    S_rgb_delay[S_delay_index] <= 24'd0;
                    S_gray_delay[S_delay_index] <= 8'd0;
                    S_mode_delay[S_delay_index] <= 5'd0;
                end
                for (S_delay_index=0; S_delay_index<7; S_delay_index=S_delay_index+1)
                    S_sync_delay[S_delay_index] <= 5'd0;
                S_output_video <= 24'd0;
                S_mask_out <= 1'b0;
            end else begin
                S_point_delay[0] <= S_point_video;
                S_point_delay[1] <= S_point_delay[0];
                S_point_delay[2] <= S_point_delay[1];
                S_rgb_delay[0] <= I_video;
                S_gray_delay[0] <= gray_of(W_r,W_g,W_b);
                S_mode_delay[0] <= W_effect_mode;
                for (S_delay_index=1; S_delay_index<6; S_delay_index=S_delay_index+1) begin
                    S_rgb_delay[S_delay_index] <= S_rgb_delay[S_delay_index-1];
                    S_gray_delay[S_delay_index] <= S_gray_delay[S_delay_index-1];
                    S_mode_delay[S_delay_index] <= S_mode_delay[S_delay_index-1];
                end
                S_sync_delay[0] <= {I_vsync,I_hsync,I_de,I_user,I_last};
                for (S_delay_index=1; S_delay_index<7; S_delay_index=S_delay_index+1)
                    S_sync_delay[S_delay_index] <= S_sync_delay[S_delay_index-1];
                case (S_mode_delay[5])
                    5'd0,5'd1,5'd2,5'd3,5'd4,5'd5,5'd6,5'd7,5'd8,5'd12:
                        S_output_video <= S_point_delay[2];
                    5'd9:  S_output_video <= {S_sobel,S_sobel,S_sobel};
                    5'd10: S_output_video <= {~S_sobel,~S_sobel,~S_sobel};
                    5'd11: S_output_video <= {S_emboss,S_emboss,S_emboss};
                    5'd13: S_output_video <= {S_neighbour_min_d,S_neighbour_min_d,S_neighbour_min_d};
                    5'd14: S_output_video <= {S_neighbour_max_d,S_neighbour_max_d,S_neighbour_max_d};
                    5'd15: S_output_video <= {W_morph_pix,W_morph_pix,W_morph_pix};
                    5'd16: S_output_video <= {S_binsobel,S_binsobel,S_binsobel};
                    5'd18: S_output_video <= !S_result_valid ? S_rgb_delay[5] :
                        (S_cartoon_edge ? 24'd0 : W_cartoon_colour);
                    default: S_output_video <= S_rgb_delay[5];
                endcase
                S_mask_out <= S_mask;
            end
        end
        assign O_video = S_output_video;
        assign O_mask = S_mask_out;
        assign {O_vsync,O_hsync,O_de,O_user,O_last} = S_sync_delay[6];
    end else begin : g_legacy_points
        assign O_video = S_point_video;
        assign O_mask = 1'b0;
        assign O_vsync = S_tsync[P_STAGES-1][4];
        assign O_hsync = S_tsync[P_STAGES-1][3];
        assign O_de = S_tsync[P_STAGES-1][2];
        assign O_user = S_tsync[P_STAGES-1][1];
        assign O_last = S_tsync[P_STAGES-1][0];
    end
    endgenerate
endmodule
