/************************************************************\
**	Copyright (c) 2012-2025 Anlogic Inc.
**	All Right Reserved.
\************************************************************/
/************************************************************\
**	Build time: Oct 11 2026 16:11:50
**	TD version	:	6.2.168116
************************************************************/
`timescale 1ps/1ps 
module RISCV_F6F1D1P0_0
(
  output                        core_sysrst,
  output                        uart1_tx,
  input                         uart1_rx,
  output                        ahb_clk,
  output  [1:0]                 htrans,
  output                        hwrite,
  output  [30:0]                haddr,
  output  [2:0]                 hsize,
  output  [2:0]                 hburst,
  output  [3:0]                 hprot,
  output                        hmastlock,
  output  [31:0]                hwdata,
  input   [31:0]                hrdata,
  input   [1:0]                 hresp,
  input                         hready,
  input                         core_clk,
  input                         timer_clk,
  input                         core_reset,
  input                         por_reset,
  input                         nmi,
  input   [22:0]                clic_irq
);

  MCU_b04349ae1671
  #(
      .MCU_JTAG_SEL("0"),
      .MCU_RESET_VECTOR('h20E00000),
      .AHB_FREQ("150.0"),
      .MCU_DPLL_POST_DIV(4),
      .MCU_DPLL_BYPASS("DISABLE")
  )MCU_b04349ae1671_Inst
  (
      .core_sysrst(core_sysrst),
      .uart1_tx(uart1_tx),
      .uart1_rx(uart1_rx),
      .ahb_clk(ahb_clk),
      .htrans(htrans),
      .hwrite(hwrite),
      .haddr(haddr),
      .hsize(hsize),
      .hburst(hburst),
      .hprot(hprot),
      .hmastlock(hmastlock),
      .hwdata(hwdata),
      .hrdata(hrdata),
      .hresp(hresp),
      .hready(hready),
      .core_clk(core_clk),
      .timer_clk(timer_clk),
      .core_reset(core_reset),
      .por_reset(por_reset),
      .nmi(nmi),
      .clic_irq(clic_irq)
  );
endmodule
