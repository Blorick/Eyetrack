//=============================================================================
//  bus_servo_cdc  --  云台角度从 75MHz 追踪域 跨到 150MHz MCU/AHB 域
//
//  为什么需要它
//  ---------------------------------------------------------------------
//  v1.12/v1.15 里 gimbal_ctrl 的输出 S_pan_angle / S_tilt_angle 在
//  S_hdmi_pixel_clk（75.002MHz 显示域）里，servo_pwm 也跑同一个域，
//  所以"角度 -> PWM"是纯同步的，不需要任何握手。
//
//  现在改成总线舵机：角度要经 AHB 寄存器桥交给硬核 MCU。AHB 跑在 150MHz
//  （MCU DPLL 使能、AHB div=2，见 mcu_servo_subsystem.v），与 75MHz 显示域
//  是**两个时钟域**，16 位的角度直接跨域会被亚稳态污染。
//
//  为什么用"锁存 + 同步 + 边沿捕获"，而不是完整请求/应答握手
//  ---------------------------------------------------------------------
//  角度每帧（60Hz，约 16.7ms）才更新一次，而两个时钟都在 50MHz 以上。
//  数据在两次更新之间稳定了 16.7ms，远大于 2 级同步器的 40ns 传播延迟，
//  所以：在源域把数据锁存住 → 数据位各做 2 级同步 → 用一根"翻转"标志位的
//  边沿作为捕获使能，即可安全跨域。60Hz 的慢速数据用完整握手纯属浪费。
//
//  寄存器映射（对 MCU 侧，即 ahb_sram_regs 的 reg[8..11]）
//  ---------------------------------------------------------------------
//    O_f2m_0 = pan  角度   [15:0]
//    O_f2m_1 = tilt 角度   [15:0]
//    O_f2m_2 = 状态        bit0 = 车牌有效，bit1 = 云台锁定
//    O_f2m_3 = 帧计数器    [15:0]（MCU 用来感知数据是否在更新）
//=============================================================================
`timescale 1ns/1ps

module bus_servo_cdc (
    //---- 源域：75MHz 追踪/显示域 ----
    input  wire        I_hdmi_clk,
    input  wire        I_hdmi_rst_n,
    input  wire        I_frame,       // 每帧一个脉冲（S_plate_frame）
    input  wire        I_valid,       // 车牌检测有效（S_plate_valid）
    input  wire        I_locked,      // 云台锁定（S_locked）
    input  wire [15:0] I_pan,         // 水平角 0..1000
    input  wire [15:0] I_tilt,        // 垂直角 0..1000

    //---- 目的域：150MHz MCU/AHB 域 ----
    input  wire        I_ahb_clk,
    input  wire        I_ahb_rst_n,

    //---- 目的域输出（寄存，直接接 ahb_sram_regs 的 I_reg_f2m_*）----
    output reg  [31:0] O_f2m_0,
    output reg  [31:0] O_f2m_1,
    output reg  [31:0] O_f2m_2,
    output reg  [31:0] O_f2m_3
);

    //=========================================================================
    // 1) 源域：每帧锁存一次角度 / 状态 / 帧号，并翻转 S_new 作为"有新数据"标志
    //=========================================================================
    reg [15:0] S_pan_lat;
    reg [15:0] S_tilt_lat;
    reg        S_valid_lat;
    reg        S_locked_lat;
    reg [15:0] S_frame_cnt;
    reg        S_new;

    always @(posedge I_hdmi_clk or negedge I_hdmi_rst_n) begin
        if(!I_hdmi_rst_n) begin
            S_pan_lat    <= 16'd0;
            S_tilt_lat   <= 16'd0;
            S_valid_lat  <= 1'b0;
            S_locked_lat <= 1'b0;
            S_frame_cnt  <= 16'd0;
            S_new        <= 1'b0;
        end
        else if(I_frame) begin
            S_pan_lat    <= I_pan;
            S_tilt_lat   <= I_tilt;
            S_valid_lat  <= I_valid;
            S_locked_lat <= I_locked;
            S_frame_cnt  <= S_frame_cnt + 1'b1;
            S_new        <= ~S_new;       // 每帧翻转一次
        end
    end

    //=========================================================================
    // 2) 2 级同步器：把"翻转标志"和所有数据位同步进目的域
    //    （数据在两次帧更新间恒定 16.7ms，2 级同步足够安全）
    //=========================================================================
    reg        S_new_m1, S_new_m2, S_new_m3;
    reg [15:0] S_pan_m1,  S_pan_m2;
    reg [15:0] S_tilt_m1, S_tilt_m2;
    reg        S_valid_m1, S_valid_m2;
    reg        S_locked_m1, S_locked_m2;
    reg [15:0] S_frame_m1, S_frame_m2;

    always @(posedge I_ahb_clk or negedge I_ahb_rst_n) begin
        if(!I_ahb_rst_n) begin
            S_new_m1 <= 1'b0;  S_new_m2 <= 1'b0;  S_new_m3 <= 1'b0;
            S_pan_m1 <= 16'd0; S_pan_m2 <= 16'd0;
            S_tilt_m1<= 16'd0; S_tilt_m2<= 16'd0;
            S_valid_m1 <= 1'b0; S_valid_m2 <= 1'b0;
            S_locked_m1<= 1'b0; S_locked_m2<= 1'b0;
            S_frame_m1<= 16'd0; S_frame_m2<= 16'd0;
        end
        else begin
            S_new_m1 <= S_new;        S_new_m2 <= S_new_m1;  S_new_m3 <= S_new_m2;
            S_pan_m1 <= S_pan_lat;    S_pan_m2 <= S_pan_m1;
            S_tilt_m1<= S_tilt_lat;   S_tilt_m2<= S_tilt_m1;
            S_valid_m1 <= S_valid_lat;   S_valid_m2 <= S_valid_m1;
            S_locked_m1<= S_locked_lat;  S_locked_m2<= S_locked_m1;
            S_frame_m1<= S_frame_cnt;    S_frame_m2<= S_frame_m1;
        end
    end

    // 边沿检测：S_new 在目的域上产生一个单拍脉冲
    wire W_capture = S_new_m2 ^ S_new_m3;

    //=========================================================================
    // 3) 目的域：边沿出现时，把已同步的数据锁进输出寄存器
    //=========================================================================
    always @(posedge I_ahb_clk or negedge I_ahb_rst_n) begin
        if(!I_ahb_rst_n) begin
            O_f2m_0 <= 32'd0;
            O_f2m_1 <= 32'd0;
            O_f2m_2 <= 32'd0;
            O_f2m_3 <= 32'd0;
        end
        else if(W_capture) begin
            O_f2m_0 <= {16'd0, S_pan_m2};
            O_f2m_1 <= {16'd0, S_tilt_m2};
            O_f2m_2 <= {30'd0, S_locked_m2, S_valid_m2};
            O_f2m_3 <= {16'd0, S_frame_m2};
        end
    end

endmodule
