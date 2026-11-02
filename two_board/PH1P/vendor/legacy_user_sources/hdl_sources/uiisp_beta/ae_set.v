//======================================================================
//  ae_set  --  曝光(AE) / 增益(AG) 手动设定
//
//  来源：米联客（MiLianKe）SC520CS 例程 uiisp_beta/ae_set.v。
//  本工程在 v1.5 上做过两次改动，都只改"谁来改变 r_ae_set / r_ag_set"：
//
//   2026-10-02 (a) 曝光按键：新增 I_ae_up / I_ae_dn，接开发板独立按键
//                   KEY3(B9)=加 / KEY4(C7)=减，经 key_ae_switch 消抖后送入；
//                   同时删掉原来那条被顶层 2'b11 钉死的 I_btn[1:0] 通道。
//   2026-10-02 (b) 增益开关：新增 I_ag_up / I_ag_dn，接拨动开关
//                   SW1(A4)=增 / SW2(B4)=减，经 switch_ag_ctrl 消抖后送入；
//                   原来的 I_btn（含顶层的 {I_button,2'b11} 拼接）整条删除。
//
//  AE / AG 的语义（沿用原厂 720p60 配置）
//  ------------------------------------
//   * AE 以"半行"为单位，VTS=1000 时最大安全值 = 2*VTS-9 = 1991，下限 2。
//   * AG 是 0..5 档对数增益，见 SC520GainTbl.v：
//         gain(AG) = 2^((AG-16)/32)，1 个 LSB = 1/32 档 ≈ 2.21%
//         AG: 16->1x, 48->2x, 80->4x, 112->8x, 144->16x, 176->32x
//
//  ★ 变化速率（重要，容易算错）
//  ---------------------------
//  AE_TICK_DIV 原本是按 100 MHz 写的常数，但本模块挂在 S_24m_clk = 23.809524 MHz 上：
//        25000 拍 / 23.809524 MHz = 1.05 ms 一次 AE 节拍
//     => AE : 约 238 LSB/s（全量程 2…1991 约 8.4 s）
//     => AG : 每 32 个 AE 节拍一步 => 134.4 ms/LSB => 约 7.4 LSB/s
//            （全量程 16…176 共 160 LSB，约 21.5 s；1 档 = 32 LSB 约 4.3 s）
//  要改手感就改 AE_TICK_DIV（例如 25000 会快 4 倍）。
//
//  数值只在变化时通过 I2C 写一次（O_ae_req 握手，AE 寄存器表见 uics520regAE.v）。
//======================================================================
module ae_set #(
    parameter integer AE_TICK_DIV = 25000   // AE 节拍分频：25000 拍 = 1.05 ms @23.809524 MHz
)(
    input  wire        I_clk,
    input  wire        I_rst,           // 高有效
    input  wire        I_ae_up,         // 曝光加（高有效，已消抖）
    input  wire        I_ae_dn,         // 曝光减（高有效，已消抖）
    input  wire        I_ag_up,         // 增益加（高有效，已消抖）
    input  wire        I_ag_dn,         // 增益减（高有效，已消抖）
    input  wire        I_ae_cfg_done,
    input  wire        I_cam_cfg_done,
    output wire        O_ae_req,
    output wire [15:0] O_ae,
    output wire [15:0] O_ag
);

    // 曝光上下限：AE 以半行计，VTS=1000 时最大 2*VTS-9 = 1991
    localparam [15:0] AE_MIN = 16'd2;
    localparam [15:0] AE_MAX = 16'd1991;
    // 增益上下限：SC520GainTbl.v 的有效输入区间
    localparam [15:0] AG_MIN = 16'd16;
    localparam [15:0] AG_MAX = 16'd176;

    //------------------------------------------------------------------
    // 节拍
    //   r_ae_tick : 每 AE_TICK_DIV 拍一个
    //   r_ag_tick : 每 32 个 r_ae_tick 一个（AG 比 AE 慢 32 倍）
    //------------------------------------------------------------------
    reg  [19:0] rc_ae_tick = 20'd0;
    reg         r_ae_tick  = 1'b0;
    reg  [4:0]  rc_ag_tick = 5'd0;
    reg         r_ag_tick  = 1'b0;

    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin
            rc_ae_tick <= 20'd0;
            r_ae_tick  <= 1'b0;
            rc_ag_tick <= 5'd0;
            r_ag_tick  <= 1'b0;
        end
        else begin
            rc_ae_tick <= rc_ae_tick + 20'd1;
            r_ae_tick  <= 1'b0;
            if(rc_ae_tick >= AE_TICK_DIV - 1) begin
                rc_ae_tick <= 20'd0;
                r_ae_tick  <= 1'b1;
            end
            rc_ag_tick <= rc_ag_tick + r_ae_tick;
            r_ag_tick  <= (&rc_ag_tick) && r_ae_tick;
        end
    end

    //------------------------------------------------------------------
    // 手动设定值。AE 按键与 AG 开关送进来的都是"按住/上拨期间保持"的
    // 高有效电平，配合上面的节拍做连续变化。
    //------------------------------------------------------------------
    reg [15:0] r_ae_set = 16'd1991;
    reg [15:0] r_ag_set = 16'd144;

    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin
            r_ae_set <= 16'd1991;
            r_ag_set <= 16'd144;
        end
        else begin
            if(I_ae_dn)
                r_ae_set <= (r_ae_set > AE_MIN) ? r_ae_set - r_ae_tick : r_ae_set;
            else if(I_ae_up)
                r_ae_set <= (r_ae_set < AE_MAX) ? r_ae_set + r_ae_tick : r_ae_set;

            if(I_ag_dn)
                r_ag_set <= (r_ag_set > AG_MIN) ? r_ag_set - r_ag_tick : r_ag_set;
            else if(I_ag_up)
                r_ag_set <= (r_ag_set < AG_MAX) ? r_ag_set + r_ag_tick : r_ag_set;
        end
    end

    //------------------------------------------------------------------
    // 变化时触发一次 AE/AG 写入（uicfgcs520 用 I_ae_req 选 AE 寄存器表）
    //------------------------------------------------------------------
    reg [15:0] r_ae;
    reg [15:0] r_ag;
    reg        r_ae_req = 1'b0;

    assign O_ae     = r_ae;
    assign O_ag     = r_ag;
    assign O_ae_req = r_ae_req;

    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst) begin
            r_ae_req <= 1'b0;
            r_ae     <= 16'd1990;
            // 与 r_ag 保持不同，使 Sensor 初始化完成后一定触发一次 AE/AG 写入
            r_ag     <= 16'd17;
        end
        else if(r_ae_req)
            r_ae_req <= 1'b0;
        else if(((r_ae != r_ae_set) || (r_ag != r_ag_set)) &&
                (I_cam_cfg_done == 1'b1) && (I_ae_cfg_done == 1'b1)) begin
            r_ae     <= r_ae_set;
            r_ag     <= r_ag_set;
            r_ae_req <= 1'b1;
        end
    end

endmodule
