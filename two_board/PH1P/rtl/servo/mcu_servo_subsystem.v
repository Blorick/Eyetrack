//=============================================================================
//  mcu_servo_subsystem  --  总线舵机 MCU 子系统（v1.16）
//
//  把 v1.12/v1.15 的两路模拟 PWM 舵机（servo_pwm x2）换成"松甲总线舵机"
//  （单线半双工 UART，115200 8N1，帧 `#<ID3>P<脉宽4>T<时间4>!`）。
//
//  数据流
//  ---------------------------------------------------------------------
//    75MHz 追踪域                      150MHz AHB 域            固件
//   ┌─────────────┐  角度/状态/帧号   ┌──────────────┐  AHB   ┌────────┐
//   │ gimbal_ctrl │ ───────────────▶│ bus_servo_cdc│ ─────▶ │ahb_sram│ ◀─ MCU
//   └─────────────┘                 └──────────────┘  reg[8..11] │_regs  │
//                                                         UART1 ──▶ 总线舵机
//   追踪算法 gimbal_ctrl.v **原样留在 FPGA**，MCU 只做
//   「角度 → 脉宽(500+2*angle) → 总线帧 → UART1 发送」，行为与 v1.15 一致。
//
//  例化的 MCU 是用户在 TD 里生成的 IP：RISCV_F6F1D1P0_0（见
//  al_ip\RISCV_F6F1D1P0_0\）。它的实际配置：
//    * 勾选：UART1 + AHB；  未勾：UART0 / APB / GPIO / I2C / QSPI / JTAG
//    * core_clk=300MHz、timer_clk=100MHz（由内部 MCU_PLL 从 50MHz 产生）；
//      ahb_clk 是 MCU 的**输出**（=core/2=150MHz），从设备跑在它上面。
//    * MCU_JTAG_SEL="0"（JTAG 不勾！这样 RISC-V 调试模块才会被串进 FPGA 主
//      JTAG 链，烧写用的正是这条 2-TAP 链；勾了反而把调试 JTAG 引到 fabric
//      脚、脱离主链，OpenOCD 就找不到 RISC-V 核了）。
//      MCU_RESET_VECTOR='h20E00000、AHB_FREQ="150.0"、
//      MCU_DPLL_POST_DIV=4、MCU_DPLL_BYPASS="DISABLE"。
//    * JTAG 端口：JTAG 不勾时 IP 不再有 jtag_* 端口，例化里不接。
//    * 复位用移位寄存器 + PLL lock 逐时钟域释放（参考 G:\fdchinesse\MCUtest）。
//
//  依赖（TD 里添加/生成的两个 IP，模块名需与本文件一致）：
//    - RISCV_F6F1D1P0_0 : MCU IP（已生成）
//    - MCU_PLL          : PLL IP，50MHz -> 300MHz(core) + 100MHz(timer)，待添加
//=============================================================================
`timescale 1ns/1ps

module mcu_servo_subsystem (
    input  wire        I_sys_clk,      // 50MHz 板载时钟（C4）
    input  wire        I_rst_n,        // 系统复位（低有效，接顶层 S_rst_n）
    input  wire        I_hdmi_clk,     // 75MHz 追踪域时钟
    input  wire        I_hdmi_rst_n,   // 75MHz 域复位（低有效）
    input  wire        I_frame,        // 每帧一个脉冲（S_plate_frame）
    input  wire        I_valid,        // 车牌检测有效（S_plate_valid）
    input  wire        I_locked,       // 云台锁定（S_locked）
    input  wire [15:0] I_pan,          // 水平角 0..1000
    input  wire [15:0] I_tilt,         // 垂直角 0..1000
    output wire        O_servo_bus     // 总线舵机数据线（MCU UART1 TX）
);

    //=========================================================================
    // 0) PLL：50MHz -> 300MHz(core) + 100MHz(timer)
    //=========================================================================
    wire W_clk_core;    // 300MHz
    wire W_clk_timer;   // 100MHz
    wire W_pll_lock;

    MCU_PLL u_mcu_pll (
        .refclk   ( I_sys_clk   ),
        .reset    ( 1'b0        ),
        .clk0_out ( W_clk_core  ),
        .clk1_out ( W_clk_timer ),
        .lock     ( W_pll_lock  )
    );

    //=========================================================================
    // 1) core 域复位（300MHz，高有效；移位寄存器 + PLL lock）
    //=========================================================================
    reg [15:0] S_core_rst_sr = 16'h0000;
    always @(posedge W_clk_core)
        if (!W_pll_lock) S_core_rst_sr <= 16'h0000;
        else             S_core_rst_sr <= {S_core_rst_sr[14:0], 1'b1};
    wire W_core_sys_rst = ~S_core_rst_sr[15];

    //=========================================================================
    // 2) AHB 主 -> 从设备
    //=========================================================================
    wire [1:0]  W_htrans;
    wire        W_hwrite;
    wire [30:0] W_haddr;
    wire [2:0]  W_hsize;
    wire [2:0]  W_hburst;
    wire [3:0]  W_hprot;
    wire        W_hmastlock;
    wire [31:0] W_hwdata;
    wire [31:0] W_hrdata;
    wire [1:0]  W_hresp;
    wire        W_hready;

    wire        W_ahb_clk;      // MCU 输出的 AHB 时钟（150MHz）

    // AHB 域复位（在 ahb_clk 上产生；pll_lock 先同步进来）
    reg [1:0] S_lock_ahb = 2'b00;
    always @(posedge W_ahb_clk) S_lock_ahb <= {S_lock_ahb[0], W_pll_lock};
    reg [15:0] S_ahb_rst_sr = 16'h0000;
    always @(posedge W_ahb_clk)
        if (!S_lock_ahb[1]) S_ahb_rst_sr <= 16'h0000;
        else                S_ahb_rst_sr <= {S_ahb_rst_sr[14:0], 1'b1};
    wire W_ahb_rst_n = S_ahb_rst_sr[15];   // 低有效，给从设备/CDC

    //---- 角度跨时钟域 + AHB 寄存器桥 ----
    wire [31:0] W_f2m_0, W_f2m_1, W_f2m_2, W_f2m_3;

    bus_servo_cdc u_cdc (
        .I_hdmi_clk   ( I_hdmi_clk   ),
        .I_hdmi_rst_n ( I_hdmi_rst_n ),
        .I_frame      ( I_frame      ),
        .I_valid      ( I_valid      ),
        .I_locked     ( I_locked     ),
        .I_pan        ( I_pan        ),
        .I_tilt       ( I_tilt       ),
        .I_ahb_clk    ( W_ahb_clk    ),
        .I_ahb_rst_n  ( W_ahb_rst_n  ),
        .O_f2m_0      ( W_f2m_0      ),
        .O_f2m_1      ( W_f2m_1      ),
        .O_f2m_2      ( W_f2m_2      ),
        .O_f2m_3      ( W_f2m_3      )
    );

    wire [31:0] W_reg_m2f_0, W_reg_m2f_1, W_reg_m2f_2, W_reg_m2f_3;

    ahb_sram_regs #(
        .MEM_AW   ( 10            ),
        // 基地址已确认：fabric 窗口 = 0x8000_0000，MCU 的 haddr[30:0] 丢 bit31，
        //    从设备只译码"窗口内偏移"。REG_BASE=0x0001_0000 是窗口内偏移，对应
        //    固件绝对地址 0x8001_0000（固件 config.h 的 FABRIC_REG_BASE）。
        //    改这里时同步改固件：FABRIC_REG_BASE = 0x8000_0000 + REG_BASE。
        .MEM_BASE ( 32'h0000_0000 ),
        .REG_BASE ( 32'h0001_0000 ),
        .REG_N    ( 16            )
    ) u_sram (
        .I_clk        ( W_ahb_clk   ),
        .I_rst_n      ( W_ahb_rst_n ),
        .I_haddr      ( W_haddr     ),
        .I_htrans     ( W_htrans    ),
        .I_hwrite     ( W_hwrite    ),
        .I_hsize      ( W_hsize     ),
        .I_hwdata     ( W_hwdata    ),
        .O_hrdata     ( W_hrdata    ),
        .O_hready     ( W_hready    ),
        .O_hresp      ( W_hresp     ),
        .O_reg_m2f_0  ( W_reg_m2f_0 ),
        .O_reg_m2f_1  ( W_reg_m2f_1 ),
        .O_reg_m2f_2  ( W_reg_m2f_2 ),
        .O_reg_m2f_3  ( W_reg_m2f_3 ),
        .I_reg_f2m_0  ( W_f2m_0     ),
        .I_reg_f2m_1  ( W_f2m_1     ),
        .I_reg_f2m_2  ( W_f2m_2     ),
        .I_reg_f2m_3  ( W_f2m_3     )
    );

    //=========================================================================
    // 3) MCU（用户生成的 IP：RISCV_F6F1D1P0_0）
    //    JTAG 不勾（MCU_JTAG_SEL=0）→ 无 jtag_* 端口，RISC-V 自动进主 JTAG 链
    //=========================================================================
    wire        W_uart1_tx;

    RISCV_F6F1D1P0_0 u_mcu (
        .core_sysrst (              ),   // 未用
        .uart1_tx    ( W_uart1_tx   ),
        .uart1_rx    ( 1'b0         ),   // 只发位置指令，不读舵机
        .ahb_clk     ( W_ahb_clk    ),
        .htrans      ( W_htrans     ),
        .hwrite      ( W_hwrite     ),
        .haddr       ( W_haddr      ),
        .hsize       ( W_hsize      ),
        .hburst      ( W_hburst     ),
        .hprot       ( W_hprot      ),
        .hmastlock   ( W_hmastlock  ),
        .hwdata      ( W_hwdata     ),
        .hrdata      ( W_hrdata     ),
        .hresp       ( W_hresp      ),
        .hready      ( W_hready     ),
        .core_clk    ( W_clk_core   ),   // 300MHz
        .timer_clk   ( W_clk_timer  ),   // 100MHz
        .core_reset  ( W_core_sys_rst ),
        .por_reset   ( W_core_sys_rst ),
        .nmi         ( 1'b0         ),
        .clic_irq    ( 23'd0        )
    );

    //=========================================================================
    // 4) 输出：UART1 TX 直接接到总线舵机数据线（只发不读）
    //=========================================================================
    assign O_servo_bus = W_uart1_tx;

endmodule
