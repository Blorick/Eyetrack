//======================================================================
//  key_ae_switch  --  曝光调节按键（KEY3 = B9 / KEY4 = C7）
//
//  背景
//  ----
//  原厂 v1.5 的 ae_set 里本来就有"AE 加 / AE 减"两个按钮输入，但顶层
//  design_top_wrapper.v 把它接成 .I_btn({I_button,2'b11})，把低两位
//  钉死为 2'b11（= 未按下），所以 AE 在板上**根本没有按键可用**，
//  只能靠改 ae_set.v 的常数换曝光。
//  （2026-10-02 起 ae_set 已改用显式的 I_ae_up/I_ae_dn/I_ag_up/I_ag_dn，
//    那条 {I_button,2'b11} 拼接连同 I_button 顶层端口一起删掉了。）
//
//  本模块把开发板独立按键 KEY3(B9) / KEY4(C7) 接到 AE 加 / AE 减上：
//
//      KEY3 -> O_ae_up   （增亮：曝光值增大）
//      KEY4 -> O_ae_dn   （减暗：曝光值减小）
//
//  两者方向可以只用参数 UP_IS_KEY3 互换，不必改线。
//
//  按键处理流程（与 key_mode_switch 同一套做法）
//  ------------------------------------------------
//     1) 3 级寄存器同步，消除异步按键的亚稳态
//     2) 可配置消抖计数，电平必须稳定超过 DEBOUNCE_CYCLES 才被接受
//     3) 对"已确认电平"做下降沿检测（低有效，下降沿即按下）
//     4) 输出两种形式：
//          O_ae_up / O_ae_dn  —— 按住期间**保持**的电平，送给 ae_set
//                                做连续增减（ae_set 内部有 1 ms 节拍）
//          O_up_pulse / O_dn_pulse —— 按下瞬间的单周期脉冲，留给
//                                以后做 OSD 显示用，本版未接
//
//  O_ae_up 与 O_ae_dn 互斥：同时按下时以"减"优先，保证 ae_set 的
//  if/else 不会出现一次增减两个方向。
//
//  时钟域
//  ------
//  与 ae_set 共用 S_24m_clk（23.809524 MHz）。DEBOUNCE_CYCLES 默认
//  500000 拍 ≈ 21 ms；换时钟必须按 §DEBOUNCE 重新换算。
//======================================================================
module key_ae_switch #(
    parameter integer DEBOUNCE_CYCLES = 500000,  // 约 21 ms @ 23.809524 MHz
    parameter integer UP_IS_KEY3      = 1        // 1: KEY3=增亮/KEY4=减暗；0: 互换
)(
    input  wire I_clk,
    input  wire I_rst_n,
    input  wire I_key3_n,       // KEY3 (B9)，低有效
    input  wire I_key4_n,       // KEY4 (C7)，低有效
    output wire O_ae_up,        // 高有效，按住期间保持
    output wire O_ae_dn,        // 高有效，按住期间保持
    output reg  O_up_pulse,     // 增亮按键按下瞬间的单周期脉冲
    output reg  O_dn_pulse      // 减暗按键按下瞬间的单周期脉冲
);

    //--------------------------------------------------------------
    // 1. 同步：每路按键打 3 拍，S_sync*[2] 作为稳定采样值
    //    上电/复位时为 1（按键松开，内部上拉）
    //--------------------------------------------------------------
    reg [2:0] S_sync3;
    reg [2:0] S_sync4;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_sync3 <= 3'b111;
            S_sync4 <= 3'b111;
        end
        else begin
            S_sync3 <= {S_sync3[1:0], I_key3_n};
            S_sync4 <= {S_sync4[1:0], I_key4_n};
        end
    end

    //--------------------------------------------------------------
    // 2. 消抖 + 3. 下降沿检测
    //    采样值与已确认电平不一致时开始计数，连续不一致超过消抖
    //    时间才更新已确认电平；S_state*_1d 与 S_state* 组合出按下沿。
    //--------------------------------------------------------------
    reg        S_state3, S_state4;
    reg [23:0] S_cnt3,   S_cnt4;
    reg        S_state3_1d, S_state4_1d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_state3    <= 1'b1;
            S_cnt3      <= 24'd0;
            S_state3_1d <= 1'b1;
        end
        else begin
            S_state3_1d <= S_state3;

            if(S_sync3[2] == S_state3)
                S_cnt3 <= 24'd0;
            else if(S_cnt3 >= DEBOUNCE_CYCLES) begin
                S_state3 <= S_sync3[2];
                S_cnt3   <= 24'd0;
            end
            else
                S_cnt3 <= S_cnt3 + 24'd1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_state4    <= 1'b1;
            S_cnt4      <= 24'd0;
            S_state4_1d <= 1'b1;
        end
        else begin
            S_state4_1d <= S_state4;

            if(S_sync4[2] == S_state4)
                S_cnt4 <= 24'd0;
            else if(S_cnt4 >= DEBOUNCE_CYCLES) begin
                S_state4 <= S_sync4[2];
                S_cnt4   <= 24'd0;
            end
            else
                S_cnt4 <= S_cnt4 + 24'd1;
        end
    end

    // 按下 = 1。S_state* 是"已确认的按键电平"，低有效，所以取反。
    wire S_down3 = ~S_state3;
    wire S_down4 = ~S_state4;

    wire S_press3 = S_state3_1d & ~S_state3;   // KEY3 按下沿
    wire S_press4 = S_state4_1d & ~S_state4;   // KEY4 按下沿

    //--------------------------------------------------------------
    // 4. 方向映射（UP_IS_KEY3 为 elaboration 期常数，综合后无额外逻辑）
    //--------------------------------------------------------------
    wire S_up_level  = UP_IS_KEY3 ? S_down3 : S_down4;
    wire S_dn_level  = UP_IS_KEY3 ? S_down4 : S_down3;
    wire S_up_press  = UP_IS_KEY3 ? S_press3 : S_press4;
    wire S_dn_press  = UP_IS_KEY3 ? S_press4 : S_press3;

    // 互斥：同时按下时以"减"优先
    assign O_ae_up = S_up_level & ~S_dn_level;
    assign O_ae_dn = S_dn_level;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_up_pulse <= 1'b0;
            O_dn_pulse <= 1'b0;
        end
        else begin
            // 与电平输出保持同一套互斥规则：同时按下时"减"优先。
            O_up_pulse <= S_up_press & ~S_dn_press;
            O_dn_pulse <= S_dn_press;
        end
    end

endmodule
