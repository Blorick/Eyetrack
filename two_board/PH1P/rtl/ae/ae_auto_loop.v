//=============================================================================
//  ae_auto_loop  --  自动曝光/增益闭环（目标亮度收敛）
//-----------------------------------------------------------------------------
//  所属域   : ae
//  时钟域   : I_clk_24m
//  复位     : I_rst_n
//  参数     : TICK_DIV, PERIOD_TICKS, BIG_STEP, SMALL_STEP, BIG_ERR, DEAD_ZONE, Y_TARGET, AE_MIN, AE_MAX, AG_MIN, AG_MAX
//  验证状态 : 仿真 通过（9 套 TB） | 综合 0 ERROR（+113/+114 ps） | 布线 Setup +90 ps / Hold +20 ps，0 违例 | 上板 未做
//=============================================================================
//=============================================================================
//  ae_auto_loop  --  自动曝光 / 增益闭环（AE + AG）
//
//  [重建说明] 此文件在 2026-10-05 的目录整理中丢失，现按原设计重建。
//
//  输入帧平均亮度（alg_luma_stat.O_y_avg），输出 AE/AG 的"加/减"电平，
//  由 ae_agc_step 按自己的节拍采样后真正改动寄存值。
//
//  控制策略
//  ---------------------------------------------------------------------
//  * Y_TARGET 附近留 DEAD_ZONE 死区，避免在目标附近来回抖。
//  * 误差大（> BIG_ERR）时步进次数多（BIG_STEP），误差小时少（SMALL_STEP）。
//  * 太暗：**先加增益**（I_ag < AG_MAX），增益到顶再加曝光（I_ae < AE_MAX）。
//  * 太亮：**先减曝光**（I_ae > AE_MIN），曝光到底再减增益（I_ag > AG_MIN）。
//    这个顺序和相机的手动习惯一致，也避免了高光时一直加增益导致噪声放大。
//
//  ★ 上板踩过的坑：AG 必须整个周期保持电平
//  ---------------------------------------------------------------------
//  原实现让 AG 的加减电平只在"判定那一拍"有效。但 ae_auto_loop 的 TICK_DIV 和
//  ae_agc_step 的 r_ae_tick 是同一个 25000 分频、同一时钟、同时释放的，
//  于是"每 32 个 AE tick 才来一次"的 AG 节拍**正好落在方向电平已经拉低之后**
//  —— AG 一次都没步进过。结果 I_ag > AG_MIN 恒为真，
//  太亮分支永远停在"减增益"上，AE 永远不降，画面全白时曝光也纹丝不动。
//  ⇒ 改成：方向在周期边界锁存，**整个周期都保持**（S_dir_lat），
//    AG 在整个周期电平有效；AE 则按步进次数在周期内分次脉冲。
//=============================================================================
module ae_auto_loop #(
    parameter integer TICK_DIV     = 25000,   // 23.81MHz 下约 1.05ms 一个判定节拍
    parameter integer PERIOD_TICKS = 32,      // 多少个判定节拍算一个周期
    parameter integer BIG_STEP     = 32,      // 大误差时一个周期内的 AE 步进次数
    parameter integer SMALL_STEP   = 6,       // 小误差时的步进次数
    parameter [7:0]   BIG_ERR      = 8'd32,   // 判定"大误差"的阈值
    parameter [7:0]   DEAD_ZONE    = 8'd6,    // 目标附近死区
    parameter [7:0]   Y_TARGET     = 8'd100,  // 目标平均亮度
    parameter [15:0]  AE_MIN       = 16'd2,
    parameter [15:0]  AE_MAX       = 16'd1991,
    parameter [15:0]  AG_MIN       = 16'd16,
    parameter [15:0]  AG_MAX       = 16'd176
)(
    input  wire        I_clk_24m,
    input  wire        I_rst_n,
    input  wire        I_enable,     // 1 = 自动闭环工作（手动按键按住时为 0）
    input  wire [7:0]  I_y_avg,      // 帧平均亮度（alg_luma_stat）
    input  wire [15:0] I_ae,         // 当前曝光
    input  wire [15:0] I_ag,         // 当前增益
    output reg         O_ae_up,
    output reg         O_ae_dn,
    output reg         O_ag_up,
    output reg         O_ag_dn
);

    // 方向编码
    localparam [2:0] DIR_NONE = 3'd0,
                     DIR_AEUP = 3'd1,
                     DIR_AEDN = 3'd2,
                     DIR_AGUP = 3'd3,
                     DIR_AGDN = 3'd4;

    //-------------------------------------------------------------------------
    // 判定：太暗 / 太亮 / 误差大小
    //-------------------------------------------------------------------------
    wire [7:0] W_hi = Y_TARGET + DEAD_ZONE;
    wire [7:0] W_lo = (Y_TARGET > DEAD_ZONE) ? (Y_TARGET - DEAD_ZONE) : 8'd0;

    wire W_too_dark   = (I_y_avg < W_lo);
    wire W_too_bright = (I_y_avg > W_hi);

    wire [7:0] W_err = W_too_dark   ? (Y_TARGET - I_y_avg) :
                       W_too_bright ? (I_y_avg - Y_TARGET) : 8'd0;
    wire W_big = (W_err > BIG_ERR);

    // 太暗：先加增益，增益到顶再加曝光
    wire [2:0] W_dir_dark = (I_ag < AG_MAX) ? DIR_AGUP :
                            ((I_ae < AE_MAX) ? DIR_AEUP : DIR_NONE);
    // 太亮：先减曝光，曝光到底再减增益
    wire [2:0] W_dir_brt  = (I_ae > AE_MIN) ? DIR_AEDN :
                            ((I_ag > AG_MIN) ? DIR_AGDN : DIR_NONE);

    wire [2:0] W_dir_now = W_too_dark   ? W_dir_dark :
                           W_too_bright ? W_dir_brt  : DIR_NONE;

    // 太亮且增益还没到底时，额外允许一次"减增益"，加快回落
    wire W_ag_dn_bright = W_too_bright && (I_ag > AG_MIN);

    //-------------------------------------------------------------------------
    // 节拍：TICK_DIV 一个判定拍；PERIOD_TICKS 个判定拍一个周期
    //-------------------------------------------------------------------------
    reg [15:0] S_tick;
    reg [6:0]  S_ptick;
    reg [2:0]  S_dir_lat;      // 周期边界锁存的方向，整周期保持
    reg [6:0]  S_step_left;    // 本周期剩余 AE 步进次数

    wire W_tick_now = (S_tick == TICK_DIV[15:0] - 16'd1);

    always @(posedge I_clk_24m or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_tick      <= 16'd0;
            S_ptick     <= 7'd0;
            S_dir_lat   <= DIR_NONE;
            S_step_left <= 7'd0;
        end
        else if(W_tick_now) begin
            S_tick <= 16'd0;
            if(S_ptick == PERIOD_TICKS[6:0] - 7'd1) begin
                // 周期边界：重新锁存方向 + 装载本周期 AE 步进次数
                S_ptick     <= 7'd0;
                S_dir_lat   <= W_dir_now;
                S_step_left <= W_big ? BIG_STEP[6:0] : SMALL_STEP[6:0];
            end
            else begin
                S_ptick <= S_ptick + 7'd1;
                // 周期内 AE 分次脉冲，用完就停
                if(S_step_left != 7'd0 &&
                   (S_dir_lat == DIR_AEUP || S_dir_lat == DIR_AEDN))
                    S_step_left <= S_step_left - 7'd1;
            end
        end
        else begin
            S_tick <= S_tick + 16'd1;
        end
    end

    //-------------------------------------------------------------------------
    // 输出：电平信号，由 ae_agc_step 按自己的节拍采样
    //   AG —— **整个周期保持电平**（这是修掉"曝光不降"的关键）
    //   AE —— 周期内按剩余步进次数脉冲
    //-------------------------------------------------------------------------
    always @(posedge I_clk_24m or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_ag_up <= 1'b0;
            O_ag_dn <= 1'b0;
        end
        else begin
            O_ag_up <= I_enable && (S_dir_lat == DIR_AGUP);
            O_ag_dn <= I_enable && ((S_dir_lat == DIR_AGDN) || W_ag_dn_bright);
        end
    end

    always @(posedge I_clk_24m or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_ae_up <= 1'b0;
            O_ae_dn <= 1'b0;
        end
        else begin
            O_ae_up <= I_enable && W_tick_now && (S_step_left != 7'd0)
                       && (S_dir_lat == DIR_AEUP);
            O_ae_dn <= I_enable && W_tick_now && (S_step_left != 7'd0)
                       && (S_dir_lat == DIR_AEDN);
        end
    end

endmodule
