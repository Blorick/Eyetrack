//======================================================================
//  switch_ag_ctrl  --  AG 增益拨动开关（SW1 = A4 / SW2 = B4）
//
//  行为（按用户 2026-10-02 的需求）
//  --------------------------------
//      SW1 上拨 ->  AG 持续**增**加
//      SW2 上拨 ->  AG 持续**减**小
//      开关下拨 ->  停止该方向的增减
//      **同时上拨 -> 停止增减**（两个方向互相抵消）
//      同时下拨   -> 停止增减
//
//  与按键的区别
//  ------------
//  拨动开关是**电平**，不是脉冲：上拨期间必须一直输出有效电平，
//  由 ae_set 自己的节拍（rc_ag_tick）做持续增减。
//  因此本模块**不做边沿检测**，只做"同步 + 消抖 + 方向互斥"。
//
//  按键处理流程
//  ------------
//     1) 3 级寄存器同步，消除异步开关的亚稳态
//     2) 消抖：采样电平必须稳定超过 DEBOUNCE_CYCLES 才被接受
//        （开关拨动的机械抖动不会造成来回反复增减）
//     3) 方向互斥：上拨两路相与取反，保证 O_ag_up / O_ag_dn 不会同时为 1
//
//  极性
//  ----
//  UP_LEVEL = 开关"上拨"时引脚的电平。
//  默认 UP_LEVEL = 1'b0（上拨把引脚拉低）+ pin.adc 里的 PULLTYPE = PULLUP。
//  这样即使极性和板上实际相反，两个引脚也会同时读到"下拨"，
//  结果只是**不增减**（fail-safe），绝不会上电就跑飞。
//  若板上是"上拨为高"，把 UP_LEVEL 改成 1'b1，并把 pin.adc 的两个开关
//  改成 PULLTYPE = PULLDOWN 即可，不需要改电路以外的任何逻辑。
//
//  复位后的初始状态固定为"下拨"（不增减），即使某个开关物理上就是上拨，
//  也要等消抖窗口过后才开始增减。
//======================================================================
module switch_ag_ctrl #(
    parameter integer DEBOUNCE_CYCLES = 500000,  // 约 21 ms @ 23.809524 MHz
    parameter         UP_LEVEL        = 1'b0,    // 开关"上拨"时引脚的电平
    parameter integer SWAP_DIR        = 0        // 0: SW1=增/SW2=减；1: 互换
)(
    input  wire I_clk,
    input  wire I_rst_n,
    input  wire I_sw1,          // SW1 (A4)
    input  wire I_sw2,          // SW2 (B4)
    output wire O_ag_up,        // 高有效：AG 增加
    output wire O_ag_dn         // 高有效：AG 减小
);

    // 复位后默认"下拨"（无效）电平
    localparam SW_IDLE_LEVEL = ~UP_LEVEL;

    //--------------------------------------------------------------
    // 1. 同步：每路 3 拍
    //--------------------------------------------------------------
    reg [2:0] S_sync1;
    reg [2:0] S_sync2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_sync1 <= {3{SW_IDLE_LEVEL}};
            S_sync2 <= {3{SW_IDLE_LEVEL}};
        end
        else begin
            S_sync1 <= {S_sync1[1:0], I_sw1};
            S_sync2 <= {S_sync2[1:0], I_sw2};
        end
    end

    //--------------------------------------------------------------
    // 2. 消抖：采样电平与已确认电平不一致时计数，连续不一致
    //    超过消抖时间才更新。上电初始值 = 下拨。
    //--------------------------------------------------------------
    reg        S_state1, S_state2;
    reg [23:0] S_cnt1,   S_cnt2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_state1 <= SW_IDLE_LEVEL;
            S_cnt1   <= 24'd0;
        end
        else if(S_sync1[2] == S_state1)
            S_cnt1 <= 24'd0;
        else if(S_cnt1 >= DEBOUNCE_CYCLES) begin
            S_state1 <= S_sync1[2];
            S_cnt1   <= 24'd0;
        end
        else
            S_cnt1 <= S_cnt1 + 24'd1;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_state2 <= SW_IDLE_LEVEL;
            S_cnt2   <= 24'd0;
        end
        else if(S_sync2[2] == S_state2)
            S_cnt2 <= 24'd0;
        else if(S_cnt2 >= DEBOUNCE_CYCLES) begin
            S_state2 <= S_sync2[2];
            S_cnt2   <= 24'd0;
        end
        else
            S_cnt2 <= S_cnt2 + 24'd1;
    end

    //--------------------------------------------------------------
    // 3. 方向互斥
    //    上拨 = 已确认电平等于 UP_LEVEL
    //    两个都上拨时 up/dn 同时为 0 => 停止增减（用户要求）
    //--------------------------------------------------------------
    wire S_up1 = (S_state1 == UP_LEVEL);
    wire S_up2 = (S_state2 == UP_LEVEL);

    wire S_up_dir = SWAP_DIR ? S_up2 : S_up1;
    wire S_dn_dir = SWAP_DIR ? S_up1 : S_up2;

    assign O_ag_up = S_up_dir & ~S_dn_dir;
    assign O_ag_dn = S_dn_dir & ~S_up_dir;

endmodule
