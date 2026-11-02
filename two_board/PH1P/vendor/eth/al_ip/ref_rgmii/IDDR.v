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
// 1.0:Date:2020/10/16 通用IDDR模块建立，支持PH1器件和EG4器件
// web：www.anlogic.com 
//-------------------------------------------------------------------- 
// 本工程改动（2026-10-06）：源码取自官方参考设计
//   2_Example/33_HX4S20_Etherent_test_01_02_final_1000/HX4S20_Ethernet_test/src/IDDR.v
//   原版只支持 "PH1" / "PH1A400" / "EG4" 三种 DEVICE。新增 "PH1P" 分支：
//   - PH1P 在普通 IO（HDIO）上做 DDR 必须用 HD 版原语，否则 TD 报
//     SYN-8228 "PH1P_LOGIC_IDDR ... can't be mapped into HDIO,
//     please use HD logic ddr"；故使用 PH1P_LOGIC_HD_IDDR
//     （TD/arch/ph1p_macro.v:8417）。
//   - HD 版用 MODE 参数而非 PIPEMODE，取值 SAME_EDGE_PIPE / SAME_EDGE /
//     OPPOSITE_EDGE。本模块 PIPEMODE="PIPED" 对应 SAME_EDGE_PIPE，
//     这也是 RGMII 接收（整周期对齐）所需的设置。
//   除该分支外，逻辑与原版逐字一致。
//*******************************************************************/
module IDDR (
        q1,
        q0,
        clk,
        d,
        rst
);
output           q1;
output           q0;
input            clk;
input            d;
input            rst;
parameter ASYNCRST      = "ENABLE";     //ENABLE, DISABLE
parameter PIPEMODE      = "PIPED";      //PIPED, NONE
parameter DEVICE 		= "EG4";//"PH1P" "PH1" "EG4" "PH1A400"

generate 
if(DEVICE == "EG4")
begin
	EG_LOGIC_IDDR #(
		.ASYNCRST(ASYNCRST),     //ENABLE, DISABLE
		.PIPEMODE(PIPEMODE)      //PIPED, NONE
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);		
end
else if(DEVICE == "PH1P")
begin
	PH1P_LOGIC_HD_IDDR #(
		.ASYNCRST(ASYNCRST),            //ENABLE, DISABLE
		.MODE("SAME_EDGE_PIPE")         //= PIPEMODE "PIPED"
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);		
end
else if(DEVICE == "PH1" || DEVICE == "PH1A400" )
begin
	PH1_LOGIC_IDDR #(
		.ASYNCRST(ASYNCRST),     //ENABLE, DISABLE
		.PIPEMODE(PIPEMODE)      //PIPED, NONE
	)
	u_iddr
	(
		.q1	(q1	),
		.q0	(q0	),
		.clk(clk),
		.d	(d	),
		.rst(rst)
	);		
end
endgenerate

endmodule
