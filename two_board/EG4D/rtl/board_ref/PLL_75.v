//=============================================================================
//  PLL_75 -- v2.5 integrated source
//-----------------------------------------------------------------------------
//  所属域   : board_ref
//  时钟域   : refclk
//  复位     : 按本模块接口定义
//  参数     : 见模块参数
//  验证状态 : 见 docs/validation.md
//=============================================================================
/************************************************************\
 **  Copyright (c) 2012-2025 Anlogic Inc.
 **  All Right Reserved.\
************************************************************/
/************************************************************\
 ** Log : Generated from the EG4D 25 MHz -> 75 MHz clock
 **       configuration used by the v1.19 EG4D reference.
 **       This copy belongs to v2.5 and is independent of PLL_0.
************************************************************/

///////////////////////////////////////////////////////////////////////////////
// Input frequency:                25.000000 MHz
// Clock multiplication factor:   3
// Clock information:
//   C0 | 75.000000 MHz | 0.0000 DEG
//   C1 | 75.000000 MHz | 90.0000 DEG
///////////////////////////////////////////////////////////////////////////////
`timescale 1 ns / 100 fs

module PLL_75 (
    input  refclk,
    input  reset,
    output extlock,
    output clk0_out,
    output clk1_out
);

    wire clk0_buf;
    wire PLL_75_open0, PLL_75_open1, PLL_75_open2, PLL_75_open3;
    wire PLL_75_open4, PLL_75_open5, PLL_75_open6, PLL_75_open7;
    wire PLL_75_open8, PLL_75_open9, PLL_75_open10, PLL_75_open11;

    EG_LOGIC_BUFG bufg_feedback (
        .i(clk0_buf),
        .o(clk0_out)
    );

    EG_PHY_PLL #(
        .DPHASE_SOURCE("DISABLE"),
        .DYNCFG("DISABLE"),
        .FIN("25.000000"),
        .FEEDBK_MODE("NORMAL"),
        .FEEDBK_PATH("CLKC0_EXT"),
        .STDBY_ENABLE("DISABLE"),
        .PLLRST_ENA("ENABLE"),
        .SYNC_ENABLE("DISABLE"),
        .GMC_GAIN(2),
        .ICP_CURRENT(9),
        .KVCO(2),
        .LPF_CAPACITOR(1),
        .LPF_RESISTOR(8),
        .REFCLK_DIV(1),
        .FBCLK_DIV(3),
        .CLKC0_ENABLE("ENABLE"),
        .CLKC0_DIV(8),
        .CLKC0_CPHASE(7),
        .CLKC0_FPHASE(0),
        .CLKC1_ENABLE("ENABLE"),
        .CLKC1_DIV(8),
        .CLKC1_CPHASE(1),
        .CLKC1_FPHASE(0)
    ) pll_inst (
        .refclk(refclk),
        .reset(reset),
        .stdby(1'b0),
        .extlock(extlock),
        .load_reg(1'b0),
        .psclk(1'b0),
        .psdown(1'b0),
        .psstep(1'b0),
        .psclksel(3'b000),
        .psdone(PLL_75_open0),
        .dclk(1'b0),
        .dcs(1'b0),
        .dwe(1'b0),
        .di(8'b0),
        .daddr(6'b0),
        .\do ({PLL_75_open8, PLL_75_open7, PLL_75_open6, PLL_75_open5,
             PLL_75_open4, PLL_75_open3, PLL_75_open2, PLL_75_open1}),
        .fbclk(clk0_out),
        .clkc({PLL_75_open11, PLL_75_open10, PLL_75_open9, clk1_out,
               clk0_buf})
    );

endmodule
