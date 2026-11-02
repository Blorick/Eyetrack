//=============================================================================
//  ahb_sram_regs  --  最小可用 AHB-Lite 从设备：4KB ERAM + 16 字寄存器块
//
//  这是 P0 要量的东西：硬核 RISC-V 本身占 0 fabric（已实测），
//  真正要花 fabric 的是**总线胶合 + 从设备**。本模块就是那个从设备。
//
//  地址映射
//  ---------------------------------------------------------------------
//    0x0000_0000 ~ 0x0000_0FFF   4KB ERAM（1024 x 32）
//    0x0001_0000 ~ 0x0001_003F   16 个 32 位寄存器
//                                  reg[0..7]   MCU 写 → FPGA 读
//                                  reg[8..11]  FPGA 写 → MCU 读（读时不落寄存器）
//
//  协议：AHB-Lite，1 个等待周期
//  ---------------------------------------------------------------------
//    地址相位 HREADY=1 → 锁存地址、译码、发起同步读
//    ST_D1    HREADY=0 → 同步读数据在本拍末尾回
//    ST_D2    HREADY=1 → HRDATA 有效，主机在本拍末采样
//  要 1 个等待周期是因为 **ERAM 是同步读**。改成组合读（分布式 RAM）
//  可以 0 等待，但那吃 LUT 而省不下 ERAM —— 这里按"省 fabric"的方向选。
//
//  ★ 上一版有一个会被"综合静默吃掉"的 bug，记在这里：
//    读地址寄存器 S_mem_a 声明了却从未赋值 ⇒ 永远只读写第 0 个字
//    ⇒ 综合把另外 1023 个字全部常量传播掉，面积报告里 #eram = 0。
//    教训：**存储器的读地址一定要显式寄存并在正确的拍上发起读**，
//    否则面积数是假的。
//=============================================================================
`timescale 1ns/1ps

module ahb_sram_regs #(
    parameter integer MEM_AW    = 10,            // 4KB = 1024 x 32
    parameter [31:0]  MEM_BASE  = 32'h0000_0000,
    parameter [31:0]  REG_BASE  = 32'h0001_0000,
    parameter integer REG_N     = 16
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    //---- AHB-Lite 从设备接口 ----
    input  wire [30:0] I_haddr,
    input  wire [1:0]  I_htrans,
    input  wire        I_hwrite,
    input  wire [2:0]  I_hsize,
    input  wire [31:0] I_hwdata,
    output      [31:0] O_hrdata,
    output reg         O_hready,
    output reg  [1:0]  O_hresp,

    //---- 寄存器块：MCU 写、FPGA 读 ----
    output reg  [31:0] O_reg_m2f_0,
    output reg  [31:0] O_reg_m2f_1,
    output reg  [31:0] O_reg_m2f_2,
    output reg  [31:0] O_reg_m2f_3,

    //---- 寄存器块：FPGA 写、MCU 读 ----
    input  wire [31:0] I_reg_f2m_0,
    input  wire [31:0] I_reg_f2m_1,
    input  wire [31:0] I_reg_f2m_2,
    input  wire [31:0] I_reg_f2m_3
);

    //-------------------------------------------------------------------------
    // 状态编码（必须在被引用之前声明）
    //-------------------------------------------------------------------------
    localparam [1:0] ST_A  = 2'd0,   // 地址相位
                     ST_D1 = 2'd1,   // 数据相位第 1 拍：HREADY=0
                     ST_D2 = 2'd2;   // 数据相位第 2 拍：HREADY=1，数据有效

    //-------------------------------------------------------------------------
    // 地址译码（地址相位，组合）
    //-------------------------------------------------------------------------
    wire [31:0] W_addr = {1'b0, I_haddr};

    wire W_act     = (I_htrans != 2'b00);                     // 一次有效传输
    wire W_hit_mem = (W_addr[31:12] == MEM_BASE[31:12]);
    wire W_hit_reg = (W_addr[31:8]  == REG_BASE[31:8]);

    wire [MEM_AW-1:0] W_mem_a = W_addr[MEM_AW+1:2];
    wire [3:0]        W_reg_a = W_addr[5:2];

    //-------------------------------------------------------------------------
    // ERAM：1024 x 32，同步写、同步读
    //-------------------------------------------------------------------------
    reg [31:0]       S_mem [0:(1<<MEM_AW)-1];
    reg [MEM_AW-1:0] S_mem_a;          // ★ 读地址寄存器（上一版漏了这句赋值）
    reg [31:0]       S_mem_q;

    //-------------------------------------------------------------------------
    // 寄存器块
    //-------------------------------------------------------------------------
    reg [31:0] S_reg [0:REG_N-1];
    reg [3:0]  S_reg_a;
    reg [31:0] S_reg_q;

    // 读 reg8..11 时穿插 FPGA 侧端口（不落寄存器，避免多驱动）
    function [31:0] reg_rdata;
        input [3:0] a;
        begin
            case(a)
                4'd8 :   reg_rdata = I_reg_f2m_0;
                4'd9 :   reg_rdata = I_reg_f2m_1;
                4'd10:   reg_rdata = I_reg_f2m_2;
                4'd11:   reg_rdata = I_reg_f2m_3;
                default: reg_rdata = S_reg[a];
            endcase
        end
    endfunction

    //-------------------------------------------------------------------------
    // 存储器 + 寄存器块：写、锁地址、同步读
    //-------------------------------------------------------------------------
    always @(posedge I_clk) begin
        //---- 写（地址相位）----
        if(W_act && I_hwrite) begin
            if(W_hit_mem)      S_mem[W_mem_a] <= I_hwdata;
            else if(W_hit_reg) S_reg[W_reg_a] <= I_hwdata;
        end

        //---- 锁存读地址与译码结果（地址相位）----
        if(W_act) begin
            S_mem_a <= W_mem_a;
            S_reg_a <= W_reg_a;
        end

        //---- 同步读：在数据相位第 1 拍发起，第 2 拍数据可用 ----
        if(S_st == ST_D1) begin
            S_mem_q <= S_mem[S_mem_a];
            S_reg_q <= reg_rdata(S_reg_a);
        end
    end

    //-------------------------------------------------------------------------
    // 给 FPGA 逻辑的四个寄存器
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_reg_m2f_0 <= 32'd0;
            O_reg_m2f_1 <= 32'd0;
            O_reg_m2f_2 <= 32'd0;
            O_reg_m2f_3 <= 32'd0;
        end
        else begin
            O_reg_m2f_0 <= S_reg[0];
            O_reg_m2f_1 <= S_reg[1];
            O_reg_m2f_2 <= S_reg[2];
            O_reg_m2f_3 <= S_reg[3];
        end
    end

    //-------------------------------------------------------------------------
    // AHB-Lite 从设备状态机（1 个等待周期）
    //-------------------------------------------------------------------------
    reg [1:0] S_st;
    reg       S_rd_mem, S_rd_reg;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_st     <= ST_A;
            O_hready <= 1'b1;
            O_hresp  <= 2'b00;
            S_rd_mem <= 1'b0;
            S_rd_reg <= 1'b0;
        end
        else begin
            case(S_st)
            //---------------------------------------------------------
            ST_A: begin
                O_hready <= 1'b0;          // 下一拍 ST_D1：HREADY=0（同步读等待）
                O_hresp  <= 2'b00;
                if(W_act) begin
                    S_rd_mem <= W_hit_mem;
                    S_rd_reg <= W_hit_reg;
                    S_st     <= ST_D1;
                end
            end
            //---------------------------------------------------------
            ST_D1: begin
                O_hready <= 1'b1;          // 下一拍 ST_D2：HREADY=1（数据有效）
                S_st     <= ST_D2;
            end
            //---------------------------------------------------------
            ST_D2: begin
                O_hready <= 1'b1;          // 下一拍 ST_A：HREADY=1（地址相位）
                S_st     <= ST_A;
            end
            default: S_st <= ST_A;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 读数据组合输出。S_mem_q / S_reg_q 在 ST_D1 末拍锁存、ST_D2 期间已稳定，
    // 因此 HRDATA 必须在 ST_D2 直接取它们（组合 mux），主机在 ST_D2 末拍采样。
    // 旧实现把 O_hrdata 做成寄存器、在 ST_D2 才赋值 → 数据晚一拍，主机永远
    // 采到上一笔传输的数据（寄存器错位，表现为 pan/tilt/status/frame 全错位）。
    //-------------------------------------------------------------------------
    assign O_hrdata = S_rd_mem ? S_mem_q :
                      S_rd_reg ? S_reg_q : 32'd0;

endmodule
