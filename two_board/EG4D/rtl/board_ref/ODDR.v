`timescale 1ns / 1ps
//******************************************************************** 
// -------------------------------------------------------------------
// >>>>>>>>>>>>>>>>>>>>>>>Copyright Notice<<<<<<<<<<<<<<<<<<<<<<<<<<<< 
// ------------------------------------------------------------------- 
//             /\ --------------- 
//            /  \ ------------- 
//           / /\ \ -----------
//          / /  \ \ ---------
//         / /    \ \ ------- 
//        / /      \ \ ----- 
//       / /_ _ _   \ \ --- 
//      /_ _ _ _ _\  \_\ -
//********************************************************************
// Author: suluyang 
// Email:luyang.su@anlogic.com 
// Date:2020/10/16 
// Description: 
// 1.0:Date:2020/10/16 通用ODDR模块建立，支持PH1器件和EG4器件
// web：www.anlogic.com 
//-------------------------------------------------------------------- 
// 本工程改动（2026-10-06）：源码取自官方参考设计
//   2_Example/33_HX4S20_Etherent_test_01_02_final_1000/HX4S20_Ethernet_test/src/ODDR.v
//   原版只支持 "PH1" / "PH1A400" / "EG4" 三种 DEVICE。新增 "PH1P" 分支：
//   - PH1P 的原语名是 PH1P_LOGIC_ODDR（TD/arch/ph1p_macro.v:8343）；
//   - 但 RGMII 的 TXC/TXD/TX_CTL 落在**普通 IO（HDIO）**上，TD 会对非 HD 版
//     报 SYN-8228 "can't be mapped into HDIO, please use HD logic ddr"，
//     因此实际使用 **PH1P_LOGIC_HD_ODDR**（ph1p_macro.v:8432），
//     其端口与参数签名（q/clk/d1/d0/rst + ASYNCRST）与普通版完全一致。
//   除该分支外，逻辑与原版逐字一致。
//*******************************************************************/
module ODDR (
        q,
        clk,
        d1,
        d0,
        rst
);
output           q;
input            clk;
input            d1;
input            d0;
input            rst;
parameter ASYNCRST      = "ENABLE";     //ENABLE, DISABLE
parameter DEVICE 		= "EG4";//"PH1P" "PH1" "EG4" "PH1A400"

generate 
if(DEVICE == "EG4")
begin
	EG_LOGIC_ODDR#(
		.ASYNCRST(ASYNCRST)
	) 
	u_oddr
	(
		.q	(q	),
		.clk(clk),
		.d1	(d1	),
		.d0	(d0	),
		.rst(rst)
	);	
end
else if(DEVICE == "PH1P")
begin
	PH1P_LOGIC_HD_ODDR#(
		.ASYNCRST(ASYNCRST)
	) 
	u_oddr
	(
		.q	(q	),
		.clk(clk),
		.d1	(d1	),
		.d0	(d0	),
		.rst(rst)
	);		
end
else if(DEVICE == "PH1" || DEVICE == "PH1A400" )
begin
	PH1_LOGIC_ODDR#(
		.ASYNCRST(ASYNCRST)
	) 
	u_oddr
	(
		.q	(q	),
		.clk(clk),
		.d1	(d1	),
		.d0	(d0	),
		.rst(rst)
	);		
end
endgenerate

endmodule
