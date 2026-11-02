module isp_top #(
    // ==========================================================================
    // 2026-10-05 P0-2 (v2.4)：img_process_96（17 种图像模式）可旁路。
    //
    // 动机：本板资源已到硬上限 —— DSP 39/40、slice 20,469/21,216 = 96.48%
    //   （v2.3_Runs\phy_1\route.qor:16-19），只剩 747 slice / 1 DSP。
    //   而 img_process_96 是这一层里最大的一块：28 个 DSP（v2.3_gate.area:108）
    //   + 约 1686 slice。
    //   题目方已确认 17 种图像模式为**可选**，车牌识别为最高优先级，
    //   故 ENABLE_IMG_PROC 默认 0。
    //
    // 这是**旁路，不是删除**：把参数改回 1，17 种模式完整恢复；
    //   img_process_96.v 与它的逐模式测试台都原样保留在工程里。
    //
    // 代价（如实记录）：
    //   1. 画面不再体现 17 种模式的效果，显示的是 AWB 校正后的图像
    //      —— 这正是车牌检测想要的下层画面；
    //   2. OSD 上由 hdmi_mixer 显示的 "ALG:<模式名>" 行会与实际画面不符
    //      （键仍会改 S_key_mode，但画面不再跟着变）。P0 暂不改 OSD，
    //      已在报告里记为已知的表面性不一致。
    // 收益（预期）：释放 28 DSP + 约 1686 slice，是加入识别级的前置条件。
    // ==========================================================================
    parameter ENABLE_IMG_PROC = 0
) (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    // v1.6: multi-mode image processing control (see img_process_96.v)
    input [4:0] 	I_mode			,   // 0..16 image-processing mode
    input signed [8:0] I_brightness	,   // mode 7 parameter, -255..+255
    input [7:0] 	I_sat_factor	,   // mode 8 parameter, 64/128/192
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready
);
 
wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 


  wire         awb_O_tlast;  //synthesis keep 
  wire         awb_O_tuser;  //synthesis keep 
  wire [ 95:0] awb_O_tdata;  //synthesis keep 
  wire         awb_O_tvalid;  //synthesis keep 
  wire         awb_O_tready;  //synthesis keep 

  wire         awb_O_tuser_128;  //synthesis keep 
  wire         awb_O_tvalid_128;  //synthesis keep 
  wire [127:0] awb_O_tdata_128;  //synthesis keep 

    wire         edge_O_tlast;  //synthesis keep
    wire         edge_O_tuser;  //synthesis keep
    wire [ 95:0] edge_O_tdata;  //synthesis keep
    wire         edge_O_tvalid;  //synthesis keep
    wire         edge_O_tready;  //synthesis keep

    // ---- 2026-10-05 P0-2：img_process_96 接入 / 旁路 选择 ----
    // 旁路时 img_process_96 整块不参与综合，AWB 输出直接进 96->128 变速级。
    // 注意 tuser 也要一起选：旁路时 edge_O_tuser 没有驱动源。
    wire [95:0] proc_tdata;
    wire        proc_tvalid;
    wire        proc_tlast;
    wire        proc_tuser;

    assign edge_O_tready = O_tready;

    generate
    if (ENABLE_IMG_PROC) begin : g_img_proc
        // 17 模式在环： awb -> img_process_96 -> 96->128
        assign awb_O_tready = edge_O_tready;
        assign proc_tdata    = edge_O_tdata;
        assign proc_tvalid   = edge_O_tvalid;
        assign proc_tlast    = edge_O_tlast;
        assign proc_tuser    = edge_O_tuser;
    end else begin : g_img_bypass
        // 旁路： awb -> 96->128
        assign awb_O_tready = O_tready;
        assign proc_tdata    = awb_O_tdata;
        assign proc_tvalid   = awb_O_tvalid;
        assign proc_tlast    = awb_O_tlast;
        assign proc_tuser    = awb_O_tuser;
    end
    endgenerate
  assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = awb_O_tlast    ;
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
demosaic #(
    .IMG_HEIGHT          (720),  // 图像高度
    .IMG_WIDTH           (1280), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .axi4s_video_tdata 	(I_tdata)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

awb #(
    .IMG_HEIGHT(720),
    .IMG_WIDTH (1280)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

// v1.6: the 2-mode edge_detect_96 is replaced by the 17-mode
// img_process_96 ported from the national-first-prize camera mode.
// edge_detect_96.v is kept in the project for traceability/regression
// but is no longer instantiated here.
// 2026-10-05 P0-2：整块包在 generate 里，ENABLE_IMG_PROC=0 时完全不参与综合。
generate
if (ENABLE_IMG_PROC) begin : g_proc_inst
img_process_96 #(
    .IMG_WIDTH  (1280),
    .IMG_HEIGHT (720),
    .EDGE_TH    (8'd80),
    .BIN_TH     (8'd128)
) u_img_process_96 (
    .I_clk        (axi4s_video_aclk),
    .I_rst_n      (I_rst_n),
    .I_mode       (I_mode),
    .I_brightness (I_brightness),
    .I_sat_factor (I_sat_factor),
    .I_tlast      (awb_O_tlast),
    .I_tuser      (awb_O_tuser),
    .I_tdata      (awb_O_tdata),
    .I_tvalid     (awb_O_tvalid),
    .I_tready     (awb_O_tready),
    .O_tlast      (edge_O_tlast),
    .O_tuser      (edge_O_tuser),
    .O_tdata      (edge_O_tdata),
    .O_tvalid     (edge_O_tvalid),
    .O_tready     (edge_O_tready)
);
end
endgenerate

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),
	
        .I_96b_frame_start  ( proc_tuser	    ),
        .I_96b_valid        ( proc_tvalid 		),
        .I_96b_data         ( proc_tdata		),
	
        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule
