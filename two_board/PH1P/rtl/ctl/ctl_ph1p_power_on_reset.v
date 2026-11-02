//=============================================================================
//  ctl_ph1p_power_on_reset  --  PH1P 50 MHz FPGA power-on reset
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk_50m
//  复位     : FPGA initialization only
//  参数     : P_HOLD_CYCLES
//  验证状态 : 仿真 未做 | 综合 未做 | 布线 未做 | 上板 未做
//=============================================================================
`timescale 1ns / 1ps
// Power-on reset for the PH1P master board.
//
// The Ethernet PHY consumes the two former R10/R11 functions, so this
// project deliberately has no external reset/PWM pad.  PH1P FPGA registers
// power up cleared; the 50 MHz clock shifts in sixteen ones before releasing
// the board reset.  The initialized shift register is the FPGA POR primitive
// for this source-only project and gives every downstream domain a common
// asserted interval during PLL/PHY startup.
module ctl_ph1p_power_on_reset #(
    parameter integer P_HOLD_CYCLES = 16
) (
    input  wire I_clk_50m,
    output wire O_rst_n
);
    reg [P_HOLD_CYCLES-1:0] S_por_shift;

    // PH1P FPGA fabric supports an initialization value for this small POR
    // register; using an explicit Verilog initial block keeps the source
    // accepted when the .v file is compiled in Verilog mode.
    initial begin
        S_por_shift = {P_HOLD_CYCLES{1'b0}};
    end

    always @(posedge I_clk_50m) begin
        S_por_shift <= {S_por_shift[P_HOLD_CYCLES-2:0], 1'b1};
    end

    assign O_rst_n = S_por_shift[P_HOLD_CYCLES-1];
endmodule
