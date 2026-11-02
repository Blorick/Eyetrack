//=============================================================================
//  gimbal_ctrl -- v2.5 integrated source
//-----------------------------------------------------------------------------
//  所属域   : alg
//  时钟域   : I_clk
//  复位     : 按本模块接口定义
//  参数     : 见模块参数
//  验证状态 : 见 docs/validation.md
//=============================================================================
//=============================================================================
//  gimbal_ctrl  --  云台追踪控制器（v1.12 新增）
//
//  作用
//  ---------------------------------------------------------------------
//  把 plate_detect 给出的车牌包围盒（每帧一个）转换成两路舵机角度，使车牌
//  稳定停在画面中心。同时提供手动点动、回中与锁定状态指示。
//
//  控制律：增量式比例控制 + 死区 + 步幅限幅 + 角度硬限幅
//  ---------------------------------------------------------------------
//  参考工程 FPGA-Object-Recognition-and-Tracking 的 Moter_X.v / Moter_Y.v：
//    * 死区（X_error_threshold ≈ 29.5 像素）抑制中位抖动 —— 采纳；
//    * 误差分档切增益（>>8 / >>9 / >>10）—— **简化成单一 >>KP_SHIFT**，
//      因为原设计的 P_ratio_1 == P_ratio_2 使第一档形同虚设，实际只有
//      3 档，分档收益远小于它带来的参数整定负担；
//    * 100Hz 固定更新率与帧率解耦 —— **本设计不做**，见下面"为什么按帧更新"；
//    * 无 I 项、无 D 项 —— 本设计同样只用 P（纯 P + 死区 + 步幅限幅对
//      这一级"位置型舵机云台"足够，加 I 容易在画面延迟下振荡）。
//
//  为什么按帧（60Hz）更新，而不是像参考工程那样固定 100Hz
//  ---------------------------------------------------------------------
//  参考工程 100Hz 更新同一条 25/30fps 的测量值，等于在 10ms 内把同一份
//  误差重复积分 2~3 次 —— 增益被隐性放大了 2~3 倍，所以它必须靠很小的
//  增益档（>>8）来压住。本设计改成**每个新帧只更新一次控制量**，增益的
//  物理意义清晰（每帧最多走 MAX_STEP 个角度单位），不会因为帧率波动而
//  改变闭环增益。
//  同时保留一个独立的手动点动拍（JOG_DIV 分频），与追踪通路完全分离。
//
//  为什么必须限幅（两个参考工程都没做对）
//  ---------------------------------------------------------------------
//    * Camera-Tracking 的 servo.vhd：Angle 是裸 integer，超范围时
//      s_high_period 的 range 会截断，实测按到 200 以上脉宽就回绕；
//    * FPGA-Object-Recognition 的 Motor_position_detect.v：X=640 时算出
//      253125 > angle_max_num = 250000，**已经越界约 1.25%**，会把舵机顶到
//      机械限位、堵转发热。
//  ⇒ 这里 O_pan / O_tilt 用 sat_add() 做双向饱和，任何情况下都严格落在
//    [ANGLE_LO, ANGLE_HI] 内。ANGLE_LO/HI 默认留出 12% 机械余量。
//
//  方向约定（上板必须标定）
//  ---------------------------------------------------------------------
//  摄像头装在云台上时，"舵机角度增大"对应画面往哪个方向移动，取决于舵机
//  的安装朝向和摇臂方向，无法从代码推断。用 PAN_INVERT / TILT_INVERT 两个
//  参数在上板标定（见 docs/design/v1.12-plate-tracking-gimbal.md 的标定步骤）：
//      若目标是"车牌的 x 偏大（在画面右侧）时 pan 减小"，就把 PAN_INVERT=1。
//  注意 tilt 的直觉与 pan 相反：画面里物体偏下（cy 偏大）时，摄像头要往下
//  转（通常是角度减小或增大，取决于舵机装法），必须先标定再开自动追踪。
//
//  丢目标策略
//  ---------------------------------------------------------------------
//  连续 LOST_FRAMES 帧没有合法车牌 -> 解锁、**保持当前角度不动**（不扫掠）。
//  理由：扫掠会在重新捕获时引入不可预测的大角度跳变，机械冲击大；而"保持"
//  只要目标还在视野里就能立刻重新锁定。若需要丢失后自动回中，把
//  HOLD_ON_LOST 参数改成 0 即可（回中由手动回中键负责，见 I_home）。
//=============================================================================
module gimbal_ctrl #(
    parameter integer IMG_W       = 1280,
    parameter integer IMG_H       = 720,

    // ---- 控制律 ----
    parameter integer DEAD_PX     = 40,    // 中心死区（像素，单边）
    parameter integer KP_SHIFT    = 4,     // 比例增益(pan)：step = err >>> KP_SHIFT
    parameter integer MAX_STEP    = 24,    // 单帧最大角度增量(pan，角度单位)
    parameter integer KP_SHIFT_T  = 4,     // 比例增益(tilt)，默认同 pan
    parameter integer MAX_STEP_T  = 24,    // 单帧最大角度增量(tilt)，默认同 pan

    // ---- 机械行程限制（角度单位，0..1000 对应 0.5ms..2.5ms）----
    parameter integer ANGLE_MID   = 500,
    parameter integer ANGLE_LO    = 120,
    parameter integer ANGLE_HI    = 880,

    // ---- 锁定判据 ----
    parameter integer ACQ_FRAMES  = 3,     // 连续检测到这么多帧才报"已锁定"
    parameter integer LOST_FRAMES = 20,    // 连续丢失这么多帧才解锁
    parameter         HOLD_ON_LOST = 1,    // 1=丢失后保持角度；0=丢失后回中

    // ---- 手动点动 ----
    parameter integer JOG_STEP    = 6,     // 每次点动的角度增量（角度单位）
    parameter integer JOG_DIV     = 15000000,  // 点动节拍分频：75MHz/1.5e7 = 5Hz
    parameter integer HOME_STEP   = 12,    // 回中每拍的角度增量

    // ---- 安装方向（上板标定）----
    parameter         PAN_INVERT  = 0,
    parameter         TILT_INVERT = 0,

    // ---- v1.15 新增：速度超前补偿 + 丢目标滑行 ----
    //   LEAD_FRAMES：用多少帧的提前量做超前补偿。
    //     ★ 上板标定值 = 数码管上显示的端到端延迟(ms) / 16.7。
    //     **默认 0 = 关闭**，先验证"不劣化"，再逐帧加上去。
    //   LEAD_MAX_PX：超前量的单边限幅（像素）。速度估计一旦异常，
    //     没有限幅的超前量会把误差项直接拉爆。
    //   PX2UNIT_SHIFT：像素误差 -> 舵机角度单位 的换算移位。
    //     几何上：1 px = 1/FOCAL_PX rad；1676 px 焦距 + 舵机 180°/2000us
    //     ⇒ 需要转 e × 0.19 个角度单位，即 e >>> 2.4。
    //     >>>2 = 0.25（略激进）／ >>>3 = 0.125（保守）。这里默认 3。
    //     ★ 这个系数只在"舵机全程 2000us 刚好对应相机 180° 直驱"时成立；
    //       若中间有减速机构，必须按实测重新定。
    //   COAST_ENABLE：丢目标时按已知速度继续外推（滑行），
    //     而不是把角度冻住。
    parameter integer LEAD_FRAMES   = 0,
    parameter integer LEAD_MAX_PX   = 200,
    parameter integer PX2UNIT_SHIFT = 3,
    parameter         COAST_ENABLE  = 1
)(
    input  wire        I_clk,       // HDMI 像素时钟 75MHz
    input  wire        I_rst_n,

    // ---- 来自 plate_detect 的每帧结果 ----
    input  wire        I_frame,     // 每帧一个脉冲
    input  wire        I_valid,     // 本帧检测到合法蓝底车牌
    input  wire [11:0] I_x0,
    input  wire [11:0] I_y0,
    input  wire [11:0] I_x1,
    input  wire [11:0] I_y1,

    // ---- 来自 plate_metrics 的带符号速度（像素/帧，Q3 定点）----
    //   v1.15 新增。v1.16 的 O_speed_mmps 只有大小没有方向，用不了。
    //   这两根线的唯一用途就是超前补偿与丢目标滑行。
    input  wire signed [15:0] I_vx_q3,
    input  wire signed [15:0] I_vy_q3,

    // ---- 用户控制 ----
    input  wire        I_enable,    // 自动追踪总开关（SW3）
    input  wire        I_jog_en,    // 手动点动使能（SW4）
    input  wire        I_jog_l,     // 高有效电平（已消抖）
    input  wire        I_jog_r,
    input  wire        I_jog_u,
    input  wire        I_jog_d,
    input  wire        I_home,      // 回中

    // ---- 到 servo_pwm ----
    output reg  [15:0] O_pan,
    output reg  [15:0] O_tilt,
    output reg         O_locked,
    output reg  [11:0] O_cx,        // 最近一帧的车牌中心（OSD 用）
    output reg  [11:0] O_cy
);

    //-------------------------------------------------------------------------
    // 0) 常数（全部为整数常数，不产生硬件）
    //    这里把它们重新声明成**带位宽的 localparam**，而不是直接对 integer
    //    型 parameter 做位选 —— 后者在部分工具里会被当成 32 位无符号量，
    //    与 signed 运算混用时容易出隐性错误。
    //-------------------------------------------------------------------------
    localparam signed [12:0] CX0 = IMG_W / 2;        // 640
    localparam signed [12:0] CY0 = IMG_H / 2;        // 360

    localparam [15:0] MID_C  = ANGLE_MID;
    localparam [15:0] LO_C   = ANGLE_LO;
    localparam [15:0] HI_C   = ANGLE_HI;
    localparam [15:0] HOME_C = HOME_STEP;

    // sat_add 里做比较用的**带符号**门限（见 sat_add 的注释：无符号比较会退化）
    localparam signed [16:0] LO_S = ANGLE_LO;
    localparam signed [16:0] HI_S = ANGLE_HI;

    localparam [7:0]  ACQ_C  = ACQ_FRAMES;
    localparam [7:0]  LOST_C = LOST_FRAMES;

    localparam signed [12:0] DEAD_S = DEAD_PX;
    localparam signed [12:0] MAXS_S = MAX_STEP;
    localparam signed [12:0] MAXS_T = MAX_STEP_T;
    localparam [3:0]         KP_S   = KP_SHIFT;
    localparam [3:0]         KP_T   = KP_SHIFT_T;
    localparam signed [12:0] JOG_S  = JOG_STEP;

    //-------------------------------------------------------------------------
    // 1) 车牌中心（包围盒中心 = 一次加法 + 一次右移，零除法器）
    //-------------------------------------------------------------------------
    wire [12:0] W_sumx = {1'b0, I_x0} + {1'b0, I_x1};
    wire [12:0] W_sumy = {1'b0, I_y0} + {1'b0, I_y1};
    wire [11:0] W_cx   = W_sumx[12:1];
    wire [11:0] W_cy   = W_sumy[12:1];

    //-------------------------------------------------------------------------
    // 2) ★ v1.15 新增：超前补偿（lead）
    //
    //   纯 P 控制必须把增益压得很低，否则环路里那几帧延迟会让它振荡。
    //   用测出来的速度直接预测"延迟结束那一刻目标在哪"，就等于把延迟从
    //   环路里拿掉了：
    //        瞄准点 = cx + vx(像素/帧) × LEAD_FRAMES
    //        err    = 瞄准点 − 画面中心
    //   这是这一版最大的收益点。
    //
    //   ★ 乘法用**编译期展开的移位加**（mul_lead 里按 LEAD_S 的位逐位累加），
    //     不写 `v * LEAD_FRAMES`：后者会让综合器生成一个真正的乘法器，
    //     很可能落到 DSP 上。现在保证 0 DSP。
    //
    //   ★★ 这里有一个必须记住的 Verilog 陷阱（仿真抓出来的）：
    //      **不要用拼接常量去和有符号量比较**。
    //      写成 `if(p > {7'd0, LM_S})` 时，拼接表达式是**无符号**的，
    //      整个比较就退化成无符号比较；`-{7'd0, LM_S}` 更会变成
    //      2^21-200 这种巨大正数，于是 `p < -{...}` 对任何 p 都成立。
    //      现象：超前量被**恒定钳到 −LEAD_MAX_PX**，云台一路往负方向跑。
    //      正确做法是用**带符号的 localparam**，让比较保持 signed。
    //-------------------------------------------------------------------------
    localparam signed [5:0]  LEAD_S  = LEAD_FRAMES;      // 限制到 6 位，够用
    localparam signed [13:0] LM_S    = LEAD_MAX_PX;      // 带符号！
    localparam signed [21:0] LM_P    = LEAD_MAX_PX;      // 22 位带符号版本
    localparam signed [13:0] CX0_14  = IMG_W / 2;        // 带符号！
    localparam signed [13:0] CY0_14  = IMG_H / 2;

    wire signed [15:0] W_vxp = I_vx_q3 >>> 3;           // Q3 -> 整数像素/帧
    wire signed [15:0] W_vyp = I_vy_q3 >>> 3;

    function signed [21:0] mul_lead;
        input signed [15:0] v;
        integer i;
        reg signed [21:0] acc;
        begin
            acc = 22'sd0;
            for(i = 0; i < 6; i = i + 1) begin
                if(LEAD_S[i]) acc = acc + ({{6{v[15]}}, v} <<< i);
            end
            mul_lead = acc;
        end
    endfunction

    function signed [13:0] clamp_lead;
        input signed [21:0] p;
        begin
            if(p > LM_P)        clamp_lead = LM_S;
            else if(p < -LM_P)  clamp_lead = -LM_S;
            else                clamp_lead = p[13:0];
        end
    endfunction

    wire signed [13:0] W_lead_x = clamp_lead(mul_lead(W_vxp));
    wire signed [13:0] W_lead_y = clamp_lead(mul_lead(W_vyp));

    // 瞄准点 = 车牌中心 + 超前量（14 位带符号，范围 -200..1479 足够）
    wire signed [13:0] W_aim_x = $signed({1'b0, W_cx}) + W_lead_x;
    wire signed [13:0] W_aim_y = $signed({1'b0, W_cy}) + W_lead_y;

    //-------------------------------------------------------------------------
    // 2.6) 死区门限（★ 同样必须是**带符号**的 localparam，否则比较退化）
    //-------------------------------------------------------------------------
    localparam signed [13:0] DEAD_S14 = DEAD_PX;

    //-------------------------------------------------------------------------
    // 3) 误差 -> 角度增量（带死区、增益、步幅限幅）
    //    误差用瞄准点算，不再用原始中心 —— 这就是超前补偿的全部改动。
    //-------------------------------------------------------------------------
    wire signed [13:0] W_ex = W_aim_x - CX0_14;
    wire signed [13:0] W_ey = W_aim_y - CY0_14;

    function signed [12:0] pstep;
        input signed [13:0] e;
        input [3:0]         sh;
        input signed [12:0] mx;
        reg   signed [12:0] s;
        begin
            if((e > DEAD_S14) || (e < -DEAD_S14)) begin
                // ★ 有符号算术右移是**向下取整**：-260>>>4 = -17（不是 -16）。
                //   所以正负方向各差 1 个 LSB，属于可忽略的量化不对称
                //   （1 个角度单位 = 2us ≈ 0.18°），不额外加取绝对值逻辑。
                s = 13'sd0 + (e >>> sh);
                if(s > mx)       s = mx;
                else if(s < -mx) s = -mx;
            end
            else begin
                s = 13'sd0;                       // 死区内不动
            end
            pstep = s;
        end
    endfunction

    wire signed [12:0] W_step_raw_x = pstep(W_ex, KP_S, MAXS_S);
    wire signed [12:0] W_step_raw_y = pstep(W_ey, KP_T, MAXS_T);

    // 安装方向
    wire signed [12:0] W_step_x = PAN_INVERT  ? -W_step_raw_x : W_step_raw_x;
    wire signed [12:0] W_step_y = TILT_INVERT ? -W_step_raw_y : W_step_raw_y;

    //-------------------------------------------------------------------------
    // 2.5) ★ v1.15 新增：丢目标滑行用的每帧角度增量
    //   把"像素/帧"换算成"舵机角度单位/帧"：
    //     先 >>>3 去掉 Q3 定点 -> 整数像素/帧
    //     再 >>>PX2UNIT_SHIFT 做像素->角度单位换算
    //   I_vx_q3 已经被 plate_metrics 限幅在 ±JUMP_MAX，所以这里不会再爆。
    //-------------------------------------------------------------------------
    wire signed [12:0] W_coast_raw_x = I_vx_q3 >>> (3 + PX2UNIT_SHIFT);
    wire signed [12:0] W_coast_raw_y = I_vy_q3 >>> (3 + PX2UNIT_SHIFT);
    wire signed [12:0] W_coast_x = PAN_INVERT  ? -W_coast_raw_x : W_coast_raw_x;
    wire signed [12:0] W_coast_y = TILT_INVERT ? -W_coast_raw_y : W_coast_raw_y;

    //-------------------------------------------------------------------------
    // 3) 角度饱和加（任何路径都不可能越出 [ANGLE_LO, ANGLE_HI]）
    //-------------------------------------------------------------------------
    function [15:0] sat_add;
        input [15:0]        cur;
        input signed [12:0] d;
        reg   signed [16:0] s;
        begin
            // ★ 用**带符号**的门限做比较。写成 `s < LO_C`（LO_C 是
            //   localparam [15:0]，无符号）时，比较会退化成无符号；
            //   一旦 s 落到负数区就会误判到"高于上限"那一支。
            //   本设计里 cur 被钳在 >= ANGLE_LO=120、|d| <= 24，s 不会为负，
            //   所以原来没暴露问题；但这里还是按带符号写，免得以后改参数踩坑。
            s = $signed({1'b0, cur}) + d;
            if(s < LO_S)           sat_add = LO_C;
            else if(s > HI_S)      sat_add = HI_C;
            else                   sat_add = s[15:0];
        end
    endfunction

    // 回中：朝 ANGLE_MID 走一步，不越过中位
    function [15:0] step_home;
        input [15:0] cur;
        reg   [15:0] up;
        begin
            if(cur > MID_C) begin
                up = cur - HOME_C;
                step_home = (up < MID_C) ? MID_C : up;
            end
            else if(cur < MID_C) begin
                up = cur + HOME_C;
                step_home = (up > MID_C) ? MID_C : up;
            end
            else begin
                step_home = MID_C;
            end
        end
    endfunction

    //-------------------------------------------------------------------------
    // 4) 锁定计数
    //-------------------------------------------------------------------------
    reg [7:0] S_acq;    // 连续检测到的帧数
    reg [7:0] S_lost;   // 连续丢失的帧数

    //-------------------------------------------------------------------------
    // 5) 手动点动节拍（与追踪通路完全独立）
    //-------------------------------------------------------------------------
    reg [23:0] S_tick;
    wire W_manual_req = I_jog_en | I_home;      // 任一手动请求
    wire W_jog_pulse  = W_manual_req && (S_tick >= (JOG_DIV - 1));

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_tick <= 24'd0;
        else if(W_manual_req) begin
            if(S_tick >= (JOG_DIV - 1)) S_tick <= 24'd0;
            else                        S_tick <= S_tick + 24'd1;
        end
        else
            S_tick <= 24'd0;
    end

    //-------------------------------------------------------------------------
    // 6) 主时序
    //    优先级：复位 > 手动（仅在 !I_enable 时）> 每帧自动追踪
    //    两路互斥：I_enable=1 只跑自动；I_enable=0 只跑手动。
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_pan    <= MID_C;
            O_tilt   <= MID_C;
            O_locked <= 1'b0;
            O_cx     <= CX0[11:0];
            O_cy     <= CY0[11:0];
            S_acq    <= 8'd0;
            S_lost   <= 8'd0;
        end
        else begin
            //-------------------------------------------------------------
            // 6a) 手动点动 / 回中（只在自动追踪关闭时生效）
            //-------------------------------------------------------------
            if(W_jog_pulse && !I_enable) begin
                if(I_home) begin
                    O_pan  <= step_home(O_pan);
                    O_tilt <= step_home(O_tilt);
                end
                else if(I_jog_en) begin
                    if(I_jog_l) O_pan  <= sat_add(O_pan,  -JOG_S);
                    if(I_jog_r) O_pan  <= sat_add(O_pan,   JOG_S);
                    if(I_jog_u) O_tilt <= sat_add(O_tilt, -JOG_S);
                    if(I_jog_d) O_tilt <= sat_add(O_tilt,  JOG_S);
                end
            end

            //-------------------------------------------------------------
            // 6b) 每帧自动追踪
            //-------------------------------------------------------------
            if(I_frame) begin
                if(!I_enable) begin
                    // 关闭自动追踪：清锁定状态，角度保持（手动可改）
                    O_locked <= 1'b0;
                    S_acq    <= 8'd0;
                    S_lost   <= 8'd0;
                end
                else if(I_valid) begin
                    O_cx  <= W_cx;
                    O_cy  <= W_cy;
                    S_lost <= 8'd0;
                    if(S_acq < ACQ_C) S_acq <= S_acq + 8'd1;
                    if(S_acq >= (ACQ_C - 8'd1)) O_locked <= 1'b1;
                    O_pan  <= sat_add(O_pan,  W_step_x);
                    O_tilt <= sat_add(O_tilt, W_step_y);
                end
                else begin
                    // 本帧没检测到车牌
                    S_acq <= 8'd0;
                    if(S_lost < LOST_C) S_lost <= S_lost + 8'd1;
                    if(S_lost >= (LOST_C - 8'd1)) begin
                        O_locked <= 1'b0;
                        if(!HOLD_ON_LOST) begin
                            O_pan  <= step_home(O_pan);
                            O_tilt <= step_home(O_tilt);
                        end
                    end
                    else if(COAST_ENABLE) begin
                        //-------------------------------------------------
                        // ★ v1.15：丢目标滑行
                        //   原来这里什么都不做（角度冻住）。现在按 plate_metrics
                        //   给出的速度继续往前外推 —— 相当于把"保持"升级成
                        //   "按已知速度预测"，遮挡/漏检期间目标不会跑出视野。
                        //   速度本身在丢失期间会缓慢衰减（plate_metrics 里做的），
                        //   所以不会一路冲到底。
                        //-------------------------------------------------
                        O_pan  <= sat_add(O_pan,  W_coast_x);
                        O_tilt <= sat_add(O_tilt, W_coast_y);
                    end
                end
            end
        end
    end

endmodule
