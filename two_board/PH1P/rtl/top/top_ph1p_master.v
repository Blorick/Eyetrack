//=============================================================================
//  top_ph1p_master  --  PH1P C41 video/OSD master-board integration top
//-----------------------------------------------------------------------------
//  所属域   : top
//  时钟域   : I_sys_clk, MIPI CSI, HDMI pixel/serial, DDR, Ethernet GMII
//  复位     : internal 50 MHz POR and per-domain synchronized resets
//  参数     : inherited C41/IP parameters
//  验证状态 : 仿真 未做 | 综合 未做 | 布线 未做 | 上板 未做
//=============================================================================
module top_ph1p_master #(
    parameter integer P_CORE_BUSINESS = 1
) (
    input wire        I_sys_clk,
      
    output wire       O_cam_scl,
    inout  wire       IO_cam_sda,
    output wire       O_cam_24m,
    output wire       O_cam_rst,
      
    //--------------------------------------------------------------------------
    // v1.7 板载控制
    //   KEY1 (D5)：短按切换一个图像处理模式，长按持续循环切换（见 key_mode_switch）
    //   KEY2 (A9)：模式 7(亮度) / 8(饱和度) 参数键（v1.6 原本挂在 KEY3）
    //   KEY3 (B9)：曝光 AE 加（来自 v1.5）
    //   KEY4 (C7)：曝光 AE 减（来自 v1.5）
    //   SW1  (A4)：AG 增益持续增加（来自 v1.5）
    //   SW2  (B4)：车牌号显示开关（上拨显示，下拨隐藏）
    //--------------------------------------------------------------------------
    input wire        I_key1,       // KEY1 (D5)，低有效：切换/长按连续切换图像处理模式
    input wire        I_key2,       // KEY2 (A9)，低有效：调整亮度/饱和度参数
    input wire        I_key3,       // KEY3 (B9)，低有效：曝光 +
    input wire        I_key4,       // KEY4 (C7)，低有效：曝光 -
    input wire        I_sw1,        // 拨动开关 SW1 (A4)：上拨 = AG 增益持续增加
    input wire        I_sw2,        // SW2 (B4)：上拨高电平允许显示车牌号，默认隐藏
    input wire I_sw3,
    input wire I_sw4,              // SW4 (D11)：保留云台页面归中功能
    output wire O_servo_bus,
    output wire O_fill_din,       // Existing DIN connection: D15, 16 mA.
    output wire O_fill_din_ref,   // Reference-bit DIN: E12, identical waveform.
    output wire O_tf_clk,
    output wire O_tf_cs_n,
    output wire O_tf_mosi,
    input wire I_tf_miso,

    //--------------------------------------------------------------------------
    // 板载 8 位数码管：显示端到端延迟（v1.13 功能补齐）
    //   段线与位选这 16 个引脚与 8 组双色 LED 复用，板上 SW6 必须“拨上”
    //   O_seg_dig[0..7] = a, b, c, d, e, f, g, DP
    //   O_seg_sel[0..7] = 左起第 1..8 位
    //   极性固定 2'b10（段线低电平点亮 + 位选一位低电平选中，v1.13 上板定案）
    //--------------------------------------------------------------------------
    output wire[7:0] O_seg_dig,
    output wire[7:0] O_seg_sel,

    // PH1P Ethernet PHY.  GMII client traffic is owned by the common
    // ctl_lpr_packet_link adapter; this top owns only PHY/RGMII wiring.
    output wire [3:0] O_led,
    output wire       O_eth_txc,
    output wire [3:0] O_eth_txd,
    output wire       O_eth_tx_ctl,
    input  wire       I_eth_rxc,
    input  wire [3:0] I_eth_rxd,
    input  wire       I_eth_rx_ctl,
    output wire       O_eth_mdc,
    inout  wire       IO_eth_mdio,
    output wire       O_eth_rst_n,
    output wire       O_tmds_ch0_p,
    output wire       O_tmds_ch1_p,
    output wire       O_tmds_ch2_p,
    output wire       O_tmds_clk_p,

    inout wire        IO_rx_clk_pad_n, 
    inout wire        IO_rx_clk_pad_p, 
    inout wire[3:0]   IO_rx_data_pad_n,
    inout wire[3:0]   IO_rx_data_pad_p,

    output wire[12:0] ddr_addr,
    output wire[ 1:0] ddr_ba,
    output wire[ 0:0] ddr_cke,
    output wire[ 0:0] ddr_odt,
    output wire[ 0:0] ddr_cs_n,
    output wire       ddr_ras_n,
    output wire       ddr_cas_n,
    output wire       ddr_we_n,
    output wire[ 0:0] ddr_ck_p,
    output wire[ 0:0] ddr_ck_n,
    inout wire[1:0]   ddr_dm,
    inout wire[15:0]  ddr_dq,
    inout wire[1:0]   ddr_dqs_p,
    inout wire[1:0]   ddr_dqs_n   
);


    wire        	S_100m_clk;
    wire        	S_24m_clk;
    wire        	S_aux_50m_clk;
	wire        	S_10m_clk;
		
	wire        	S_pll_lock;
    wire        	S_rst_n;
    wire            S_board_rst_n;

    // 云台角度/锁定（总线舵机 MCU 子系统输入，来自 alg_vehicle_services 的追踪结果）
    wire [15:0] S_pan_angle, S_tilt_angle;
    wire        S_locked;

    // R10/R11 are Ethernet TX control/data pins in the two-board design;
    // release the video and Ethernet logic from an explicit 50 MHz FPGA POR.
    ctl_ph1p_power_on_reset u_ph1p_power_on_reset (
        .I_clk_50m ( I_sys_clk     ),
        .O_rst_n   ( S_board_rst_n )
    );
	reg  [17:0] 	S_cam_power_cnt;
	reg         	S_cam_pwdnb;
	reg         	S_cam_i2c_ready;
	(* keep = "true" *) wire [15:0] S_cam_sensor_id;
	(* keep = "true" *) wire        S_cam_sensor_id_ok;
	(* keep = "true" *) wire        S_cam_i2c_error;
	localparam integer CAM_PWDNB_LOW_CYCLES = 23810;  // >= 1 ms at 23.809524 MHz
	localparam integer CAM_I2C_START_CYCLES = 142857; // release + >= 5 ms
	
	wire [15:0] 	S_ae;
    wire [15:0] 	S_ag;
    wire        	S_ae_up;        // KEY3 -> 曝光加（已消抖的高有效电平）
    wire        	S_ae_dn;        // KEY4 -> 曝光减（已消抖的高有效电平）
    wire        	S_ag_up;        // SW1  -> 增益加（已消抖的高有效电平）
    wire        	S_ag_dn;        // 保留接口；SW2 改为显示开关，手动增益减固定关闭
    wire 			S_cam_cfg_done;
    wire 			S_ae_cfg_done;
    wire 			S_ae_req; 
	
	wire        	S_csi_rx_clk;  
    wire        	S_hs_rx_valid;  
    wire[15:0]  	S_hs_rx_data;   
	wire[1:0]   	S_lane_error;
	
	wire        	S_csi_frame_start;       
	wire        	S_csi_frame_end;         
	wire        	S_csi_valid;             
	wire[31:0]  	S_csi_data;  
		
	wire        	S_raw10_frame_start; 
	wire        	S_raw10_frame_end;   
	wire        	S_raw10_valid;       
	wire [39:0] 	S_raw10_data;
		
	wire 			S_axis_tlast;  
	wire 			S_axis_tuser;  
	wire [39:0]		S_axis_tdata;  
	wire 			S_axis_tvalid; 
	
	wire [39:0]		S_raw_tdata ;
	wire			S_raw_tlast ;
	wire 			S_raw_tvalid;
	wire 			S_raw_tuser ;
	
	wire			S_ISP_O_tready;
	(* keep = "true" *) wire		S_ISP_I_tready;
	wire [127:0]	S_ISP_O_tdata ;
	wire 			S_ISP_O_tlast ;
	wire 			S_ISP_O_tuser ;
	wire 			S_ISP_O_tvalid; 

	// video_in has no backpressure input; keep the ISP output stream consuming
	// continuously so the AWB elastic stage cannot stall after its first word.
	assign S_ISP_O_tready = 1'b1;
	

    wire        S_ddr_clk;
    wire        S_hdmi_pixel_clk;
    wire        S_hdmi_serial_clk;
    wire        S_hdmi_rst_n;
    wire        S_video_out_rst_n;


    wire        S_vi_128b_frame_start;
    wire        S_vi_128b_valid;      
    wire[127:0] S_vi_128b_data;    
    
    wire        S_video_out_rd_busy;
    wire        S_video_in_wr_busy; 
    wire[1:0]   S_video_out_rp;     

    wire        S_ddr_user_wr_en;       
    wire        S_ddr_user_rd_en;       
    wire[24:0]  S_ddr_user_addr;        
    wire[127:0] S_ddr_user_wr_data;     
    wire        S_ddr_user_ready;       
    wire        S_ddr_user_rd_valid;    
    wire[127:0] S_ddr_user_rd_data;     

    wire        S_vi_ddr_wr_en;   
    wire[24:0]  S_vi_ddr_wr_addr; 
    wire[127:0] S_vi_ddr_wr_data;
    wire        S_vi_ddr_wr_ready;

    wire        S_vo_ddr_rd_en;   
    wire[24:0]  S_vo_ddr_rd_addr; 
    wire        S_vo_ddr_rd_ready;
    wire        S_vo_dbg_fifo_rst;
    wire        S_vo_dbg_ddr_rd_valid;
    wire        S_vo_dbg_fifo_rd_en;
    wire        S_vo_dbg_fifo_empty;

    wire        S_init_calib_complete; 
    wire[24:0]  S_mc_app_addr;          
    wire[2:0]   S_mc_app_cmd;           
    wire        S_mc_app_en;            
    wire[127:0] S_mc_app_wdf_data;      
    wire        S_mc_app_wdf_end;       
    wire[15:0]  S_mc_app_wdf_mask;      
    wire        S_mc_app_wdf_wren;      
    wire[127:0] S_mc_app_rd_data;       
    wire        S_mc_app_rd_data_end;   
    wire        S_mc_app_rd_data_valid; 
    wire        S_mc_app_rdy;           
    wire        S_mc_app_wdf_rdy;       
    wire        S_mc_dbg_fifo_afull;
    wire        S_mc_dbg_app_rdy;
    wire        S_mc_dbg_wdf_rdy;
    wire        S_mc_dbg_cmd_pop;

    wire        S_hdmi_vsync;
    wire        S_hdmi_hsync;
    wire        S_hdmi_de;
    wire        S_hdmi_user;
    wire        S_hdmi_last;
    wire        S_video_out_vsync;
    wire        S_hdmi_window_rd_en;
    wire        S_hdmi_out_vsync;
    wire        S_hdmi_out_hsync;
    wire        S_hdmi_out_de;
    wire[23:0]  S_hdmi_out_data;

    wire[23:0]  S_video_rd_data;
    wire        S_video_data_valid;
    reg [1:0]   S_video_vs_pipe, S_video_hs_pipe;
    reg [1:0]   S_video_user_pipe, S_video_last_pipe;
    reg         S_recog_vsync;
    // FIFO read and RGB unpacking take two registers. Preserve every request
    // slot (including black underflow pixels) so image coordinates stay intact.
    assign S_hdmi_window_rd_en = S_hdmi_de;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_video_vs_pipe<=0; S_video_hs_pipe<=0;
            S_video_user_pipe<=0; S_video_last_pipe<=0; S_recog_vsync<=0;
        end else begin
            S_video_vs_pipe<={S_video_vs_pipe[0],S_hdmi_vsync};
            S_video_hs_pipe<={S_video_hs_pipe[0],S_hdmi_hsync};
            S_video_user_pipe<={S_video_user_pipe[0],S_hdmi_user};
            S_video_last_pipe<={S_video_last_pipe[0],S_hdmi_last};
            S_recog_vsync<=S_video_vs_pipe[1];
        end
    end

    // ---- 车牌定位 + 画框（v1.7 新增） ----
    wire [10:0] S_plate_up;
    wire [10:0] S_plate_down;
    wire [10:0] S_plate_left;
    wire [10:0] S_plate_right;
    wire        S_plate_detected;
    // plate_locate commits one result in S_DONE after its bounded scan.  The
    // result-update pulse is asserted for the following clock, when
    // S_plate_detected already contains that newly committed result.
    wire        W_plate_fresh_pix;
    wire        S_plate_result_update;
    wire [23:0] S_hdmi_out_data_ovl;
    wire        S_hdmi_out_de_ovl;
    wire        S_hdmi_out_vsync_ovl;

    // ---- 字符边界提取 + 画字符框（v1.7 功能二） ----
    wire [10:0] S_char_up;
    wire [10:0] S_char_down;
    wire [10:0] S_c1_left,  S_c1_right;
    wire [10:0] S_c2_left,  S_c2_right;
    wire [10:0] S_c3_left,  S_c3_right;
    wire [10:0] S_c4_left,  S_c4_right;
    wire [10:0] S_c5_left,  S_c5_right;
    wire [10:0] S_c6_left,  S_c6_right;
    wire [10:0] S_c7_left,  S_c7_right;
    wire        S_char_detected;
    wire [6:0]  S_char_valid;      // 2026-10-05 P0-1a：逐字符有效性（char_locate 新增）

    // Packed ink is exported from the original character-locator tap.
    // Sampling and matching run on EG; the camera/display paths stay local.
    (* keep = "true" *) wire S_char_ink;
    (* keep = "true" *) wire S_char_ink_de;
    wire W_ink_legacy_rx_valid_pix;
    wire W_ink_tx_grant_tx;
    wire W_ink_tx_request_tx;
    wire W_ink_tx_done_tx;
    wire [7:0] W_ink_gmii_txd_tx;
    wire W_ink_gmii_tx_en_tx;
    wire W_ink_gmii_tx_er_tx;
    wire [55:0] S_res;
    wire [3:0] S_res_n;
    wire S_res_valid;
    wire [6:0] S_raw_valid;
    wire S_frame_result;
    wire [55:0] S_raw_candidate_code;
    wire [6:0] S_raw_candidate_mask;
    wire [3:0] S_match_state_public;
    wire [2:0] W_ink_scheduler_grant_tx;
    reg [55:0] S_res_osd;
    reg [3:0]  S_res_n_osd;
    reg        S_res_valid_osd;
    // Classification can finish during active video. Snapshot the complete
    // result at SOF so the entire displayed frame uses one plate identity.
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) begin
            S_res_osd<=0; S_res_n_osd<=0; S_res_valid_osd<=0;
        end else if (S_video_user_pipe[1]) begin
            S_res_osd<=S_res; S_res_n_osd<=S_res_n; S_res_valid_osd<=S_res_valid;
        end
    end

    wire [23:0] S_hdmi_out_data_ovl2;
    wire        S_hdmi_out_de_ovl2;
    wire        S_hdmi_out_vsync_ovl2;

    reg         S_dbg_raw_seen;
    reg         S_dbg_isp_seen;
    reg         S_dbg_ddr_wr_seen;
    reg         S_dbg_ddr_rd_seen;
    reg         S_dbg_cam_cfg_seen;
    reg         S_dbg_hs_seen;
    reg         S_dbg_csi_seen;
    reg         S_dbg_raw10_seen;
    reg         S_dbg_isp_fs_seen;
    reg         S_dbg_isp_valid_seen;
    reg         S_dbg_ddr_req_seen;
    reg         S_dbg_ddr_ready_seen;
    reg [1:0]   S_dbg_raw_seen_sync;
    reg [1:0]   S_dbg_isp_seen_sync;
    reg [1:0]   S_dbg_ddr_wr_seen_sync;
    reg [1:0]   S_dbg_ddr_rd_seen_sync;
    reg [1:0]   S_dbg_cam_cfg_seen_sync;
    reg [1:0]   S_dbg_hs_seen_sync;
    reg [1:0]   S_dbg_csi_seen_sync;
    reg [1:0]   S_dbg_raw10_seen_sync;
    reg [1:0]   S_dbg_isp_fs_seen_sync;
    reg [1:0]   S_dbg_isp_valid_seen_sync;
    reg [1:0]   S_dbg_ddr_req_seen_sync;
    reg [1:0]   S_dbg_ddr_ready_seen_sync;
    reg         S_dbg_video_nonzero_seen;
    reg         S_dbg_rd_req_seen;
    reg         S_dbg_rd_valid_seen;
    reg         S_dbg_rd_nonzero_seen;
    reg [1:0]   S_dbg_rd_req_seen_sync;
    reg [1:0]   S_dbg_rd_valid_seen_sync;
    reg [1:0]   S_dbg_rd_nonzero_seen_sync;
    reg         S_dbg_mc_init_seen;
    reg         S_dbg_mc_app_rdy_seen;
    reg         S_dbg_mc_wdf_rdy_seen;
    reg         S_dbg_mc_cmd_pop_seen;
    reg [1:0]   S_dbg_mc_init_seen_sync;
    reg [1:0]   S_dbg_mc_app_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_wdf_rdy_seen_sync;
    reg [1:0]   S_dbg_mc_cmd_pop_seen_sync;
    wire        S_lane_error_any;

    // ---- 按键选择图像处理模式与参数（v1.6，17 种模式） ----
    wire [4:0]  S_key_mode;         // 摄像头算法编号，按 0..19 顺序循环
    wire [4:0]  W_camera_mode_index;
    reg  [4:0]  S_mode_meta;        // 跨时钟域同步第一级
    reg  [4:0]  S_mode_video_sync;  // 同步到 MIPI 视频时钟域
    wire [4:0]  S_mode_video;       // 历史 ISP 控制旁路；实际模式按第一档原编号
    wire        S_adj_pulse;        // KEY3 按下脉冲（24M 域）
    wire signed [8:0] S_brightness; // 24M 域
    wire [7:0]  S_sat_factor;       // 24M 域
    reg  signed [8:0] S_bright_meta, S_bright_sync;
    reg  [7:0]  S_sat_meta, S_sat_sync;

    wire [4:0] S_display_mode_pix;
    wire signed [8:0] S_display_brightness_pix;
    wire [7:0] S_display_sat_factor_pix;

    // ---- W4：自动曝光闭环信号 ----
    // 显示域帧平均亮度与 75M -> 24M CDC
    wire [7:0] S_y_avg_pix;
    reg  [7:0] S_y_avg_meta, S_y_avg_24m;
    // 自动曝光闭环输出 + 手动占用标志（24M 域）
    wire       S_auto_ae_up, S_auto_ae_dn, S_auto_ag_up, S_auto_ag_dn;
    wire       W_manual_busy;
    reg S_display_vsync_d_pix;
    wire W_display_frame_pix;
    wire [23:0] S_mode_rgb_pix;
    wire S_mode_vsync_pix;
    wire S_mode_hsync_pix;
    wire S_mode_de_pix;
    wire S_mode_user_pix;
    wire S_mode_last_pix;
    wire S_mode_link_ok_pix;
    wire S_mode_remote_used_pix;
    wire S_stream_tx_ready_pix;
    wire S_stream_tx_available_pix;
    wire S_stream_tx_write_pix;
    wire [10:0] S_stream_tx_address_pix;
    wire [7:0] S_stream_tx_data_pix;
    wire S_stream_tx_commit_pix;
    wire S_stream_rx_valid_pix;
    wire [15:0] S_stream_rx_length_pix;
    wire [10:0] S_stream_rx_address_pix;
    wire [7:0] S_stream_rx_data_pix;
    wire S_stream_rx_release_pix;
    wire [7:0] S_ocr_gmii_txd;
    wire S_ocr_gmii_tx_en;
    wire S_ocr_gmii_tx_er;
    wire S_ocr_tx_request;
    wire S_ocr_tx_grant;
    wire S_ocr_tx_done;
    // W3: 0x88B5 key-state RX path (pixel clock domain) + button plate OSD.
    wire [4:0]  W_key_code_pix;
    wire        W_key_valid_pix;
    wire        W_key_changed_pix;
    wire        W_key_rx_valid;
    wire [10:0] W_key_rx_length;
    wire [9:0]  W_key_rx_addr;
    wire [7:0]  W_key_rx_data;
    wire        W_key_rx_release;
    wire        W_key_auto_pix;
    wire        W_cycle_reset_request_pix;
    wire        W_plate_cycle_active_pix;
    wire        W_plate_cycle_reset_ack_pix;
    wire [55:0] S_key_res;
    wire [3:0]  S_key_res_n;
    wire        S_key_res_valid;
    wire [7:0] S_stream_gmii_txd;
    wire S_stream_gmii_tx_en;
    wire S_stream_gmii_tx_er;
    wire S_stream_tx_request;
    wire S_stream_tx_grant;
    wire S_stream_tx_done;
    wire S_camera_key_n_24m;
    wire [1:0] S_control_profile_24m;
    wire [1:0] S_control_profile_pix;
    wire S_plate_display_enable_pix;
    wire S_tf_key_n_24m;
    wire S_track_enable_pix;
    wire S_jog_enable_pix;
    wire S_jog_left_pix;
    wire S_jog_right_pix;
    wire S_jog_up_pix;
    wire S_jog_down_pix;
    wire S_gimbal_home_pix;
    wire [23:0] S_service_rgb_pix;
    wire S_service_de_pix;
    wire S_service_user_pix;
    wire S_service_last_pix;
    wire [55:0] S_service_ocr_pix;
    wire [3:0] S_service_ocr_count_pix;
    wire S_service_ocr_valid_pix;
    wire [15:0] W_ink_diag_status_pix;
    wire [15:0] W_ink_diag_counts_pix;
    wire [19:0] W_service_dist_mm_pix;
    wire [19:0] W_service_speed_mmps_pix;
    wire [7:0] W_service_metrics_state_pix;
    wire W_service_metrics_valid_pix;

    assign W_display_frame_pix = S_video_vs_pipe[1] && !S_display_vsync_d_pix;
    assign W_plate_fresh_pix = S_plate_result_update && S_plate_detected;
    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if (!S_hdmi_rst_n) S_display_vsync_d_pix <= 1'b0;
        else S_display_vsync_d_pix <= S_video_vs_pipe[1];
    end

    ctl_display_control_snapshot u_display_controls (
        .I_clk_24m(S_24m_clk), .I_clk_pix(S_hdmi_pixel_clk),
        .I_rst_n(S_rst_n && S_hdmi_rst_n),
        .I_mode(S_key_mode),
        .I_brightness(S_brightness), .I_sat_factor(S_sat_factor),
        .I_frame_pix(W_display_frame_pix), .O_mode(S_display_mode_pix),
        .O_brightness(S_display_brightness_pix), .O_sat_factor(S_display_sat_factor_pix)
    );

    ctl_vehicle_controls u_vehicle_controls (
        .I_clk_24m(S_24m_clk), .I_clk_pix(S_hdmi_pixel_clk),
        .I_rst_n(S_rst_n && S_hdmi_rst_n), .I_frame_pulse(W_display_frame_pix),
        .I_key1_n(I_key1), .I_key2_n(I_key2), .I_key3_n(I_key3), .I_key4_n(I_key4),
        .I_sw1(I_sw1), .I_sw2(I_sw2), .I_sw3(I_sw3), .I_sw4(I_sw4),
        .O_profile_24m(S_control_profile_24m), .O_profile_pix(S_control_profile_pix),
        .O_plate_display_enable(S_plate_display_enable_pix),
        .O_camera_key1_n(S_camera_key_n_24m), .O_camera_key2_short(S_adj_pulse),
        .O_camera_ae_up(S_ae_up), .O_camera_ae_dn(S_ae_dn),
        .O_camera_ag_up(S_ag_up), .O_camera_ag_dn(S_ag_dn), .O_tf_key_n(S_tf_key_n_24m),
        .O_track_enable(S_track_enable_pix), .O_jog_enable(S_jog_enable_pix),
        .O_jog_left(S_jog_left_pix), .O_jog_right(S_jog_right_pix),
        .O_jog_up(S_jog_up_pix), .O_jog_down(S_jog_down_pix),
        .O_gimbal_home(S_gimbal_home_pix)
    );


    // ---- Ethernet PHY/RGMII shell ---------------------------------------
    // The common ctl_lpr_packet_link below drives the GMII client wires;
    // this top owns only the PHY/RGMII shell and status observation.
    (* keep = "true" *) wire [7:0] S_eth_gmii_txd;
    (* keep = "true" *) wire       S_eth_gmii_tx_en;
    (* keep = "true" *) wire       S_eth_gmii_tx_er;
    (* keep = "true" *) wire [7:0] S_eth_gmii_rxd;
    (* keep = "true" *) wire       S_eth_gmii_rx_dv;
    (* keep = "true" *) wire       S_eth_gmii_rx_er;
    (* keep = "true" *) wire       S_eth_gmii_rx_clk;
    (* keep = "true" *) wire       S_eth_gmii_tx_clk;
    (* keep = "true" *) wire       S_eth_gmii_tx_clk_90;
    (* keep = "true" *) wire       S_eth_pll_lock;
    (* keep = "true" *) wire       S_eth_reset_ready;
    (* keep = "true" *) wire       S_eth_tx_reset_n;
    (* keep = "true" *) wire       S_eth_rx_reset_n;
    (* keep = "true" *) wire       S_eth_link_status;
    (* keep = "true" *) wire [1:0] S_eth_clock_speed_rx;
    (* keep = "true" *) wire       S_eth_duplex_status_rx;
    (* keep = "true" *) wire       S_eth_gigabit_ready_rx;
    (* keep = "true" *) wire       S_eth_gigabit_ready_app;
    (* keep = "true" *) wire       S_eth_app_reset_async_n;
    (* keep = "true" *) wire       S_eth_app_reset_n;
    // HDMI/video reset is also an asynchronous assertion source for the
    // common GMII TX/RX domains.  Release still comes from each shell domain
    // synchronizer; a video PLL loss cannot leave Ethernet state running.
    wire W_eth_tx_common_async_n;
    wire W_eth_rx_common_async_n;
    wire W_eth_tx_common_reset_n;
    wire W_eth_rx_common_reset_n;
    reg [1:0] S_eth_tx_common_rel_q;
    reg [1:0] S_eth_rx_common_rel_q;
    reg [1:0] S_eth_app_reset_sync;
    reg [1:0] S_eth_ready_app_sync;
    reg [1:0] S_telemetry_cfg_reset_sync;

    ctl_ph1p_rgmii_shell u_ph1p_eth_shell (
        .I_clk_50m       (I_sys_clk),
        .I_rst_n         (S_board_rst_n),
        .O_eth_txd       (O_eth_txd),
        .O_eth_txc       (O_eth_txc),
        .O_eth_tx_ctl    (O_eth_tx_ctl),
        .I_eth_rxd       (I_eth_rxd),
        .I_eth_rxc       (I_eth_rxc),
        .I_eth_rx_ctl    (I_eth_rx_ctl),
        .O_eth_mdc       (O_eth_mdc),
        .IO_eth_mdio     (IO_eth_mdio),
        .O_eth_rst_n     (O_eth_rst_n),
        .I_gmii_txd      (S_eth_gmii_txd),
        .I_gmii_tx_en    (S_eth_gmii_tx_en),
        .I_gmii_tx_er    (S_eth_gmii_tx_er),
        .O_gmii_rxd      (S_eth_gmii_rxd),
        .O_gmii_rx_dv    (S_eth_gmii_rx_dv),
        .O_gmii_rx_er    (S_eth_gmii_rx_er),
        .O_gmii_rx_clk   (S_eth_gmii_rx_clk),
        .O_gmii_tx_clk   (S_eth_gmii_tx_clk),
        .O_gmii_tx_clk_90(S_eth_gmii_tx_clk_90),
        .O_pll_lock      (S_eth_pll_lock),
        .O_reset_ready   (S_eth_reset_ready),
        .O_tx_reset_n    (S_eth_tx_reset_n),
        .O_rx_reset_n    (S_eth_rx_reset_n),
        .O_link_status   (S_eth_link_status),
        .O_clock_speed   (S_eth_clock_speed_rx),
        .O_duplex_status (S_eth_duplex_status_rx),
        .O_gigabit_ready (S_eth_gigabit_ready_rx)
    );

    // Common reset is asserted by any board/PLL/PHY-domain failure and is
    // released only by this HDMI-clock-domain two-flop synchronizer.  The
    // Ethernet shell independently synchronizes TX and RX release in their
    // own clocks; no asynchronous ETH-ready deassertion reaches app logic.
    assign S_eth_app_reset_async_n = S_hdmi_rst_n & S_eth_reset_ready;
    always @(posedge S_hdmi_pixel_clk or negedge S_eth_app_reset_async_n) begin
        if (!S_eth_app_reset_async_n)
            S_eth_app_reset_sync <= 2'b00;
        else
            S_eth_app_reset_sync <= {S_eth_app_reset_sync[0], 1'b1};
    end
    assign S_eth_app_reset_n = S_eth_app_reset_sync[1];
    // The telemetry configuration mailbox shares Ethernet reset assertion;
    // release is synchronized in the 24 MHz configuration domain.
    always @(posedge S_24m_clk or negedge S_eth_app_reset_async_n) begin
        if (!S_eth_app_reset_async_n)
            S_telemetry_cfg_reset_sync <= 2'b00;
        else
            S_telemetry_cfg_reset_sync <= {S_telemetry_cfg_reset_sync[0], 1'b1};
    end
    assign W_eth_tx_common_async_n = S_hdmi_rst_n & S_eth_tx_reset_n;
    assign W_eth_rx_common_async_n = S_hdmi_rst_n & S_eth_rx_reset_n;
    always @(posedge S_eth_gmii_tx_clk or negedge W_eth_tx_common_async_n) begin
        if (!W_eth_tx_common_async_n)
            S_eth_tx_common_rel_q <= 2'b00;
        else
            S_eth_tx_common_rel_q <= {S_eth_tx_common_rel_q[0], 1'b1};
    end
    always @(posedge S_eth_gmii_rx_clk or negedge W_eth_rx_common_async_n) begin
        if (!W_eth_rx_common_async_n)
            S_eth_rx_common_rel_q <= 2'b00;
        else
            S_eth_rx_common_rel_q <= {S_eth_rx_common_rel_q[0], 1'b1};
    end
    assign W_eth_tx_common_reset_n = S_eth_tx_common_rel_q[1];
    assign W_eth_rx_common_reset_n = S_eth_rx_common_rel_q[1];

    // Synchronize the RXC-domain 1G/full-duplex indication into the video
    // application clock.  The adapter does not publish a snapshot while
    // this indication is low, and its result-valid state is cleared.
    always @(posedge S_hdmi_pixel_clk or negedge S_eth_app_reset_n) begin
        if (!S_eth_app_reset_n)
            S_eth_ready_app_sync <= 2'b00;
        else
            S_eth_ready_app_sync <= {S_eth_ready_app_sync[0], S_eth_gigabit_ready_rx};
    end
    assign S_eth_gigabit_ready_app = S_eth_ready_app_sync[1];

    // Active-high LED1..4: RESULT seen, session ready, stable OSD, link ready.
    assign O_led = {S_eth_gigabit_ready_app, S_service_ocr_valid_pix,
                     W_ink_diag_status_pix[0], W_ink_diag_status_pix[2]};

    // Reader reset/prefetch uses the positive VS blanking pulse.
    assign S_video_out_vsync = S_hdmi_vsync;
	
	
    assign S_rst_n 		= S_board_rst_n & S_pll_lock;
    assign S_hdmi_rst_n = S_board_rst_n & S_pll_lock;

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw_seen <= 1'b0;
        else if(S_raw10_frame_start | S_axis_tvalid)
            S_dbg_raw_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_seen <= 1'b0;
        else if(S_ISP_O_tuser | S_ISP_O_tvalid)
            S_dbg_isp_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_wr_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_wr_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_rd_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_ddr_rd_seen <= 1'b1;
    end

    always @(posedge S_24m_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_cam_cfg_seen <= 1'b0;
        else if(S_cam_cfg_done)
            S_dbg_cam_cfg_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_hs_seen <= 1'b0;
        else if(S_hs_rx_valid)
            S_dbg_hs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_csi_seen <= 1'b0;
        else if(S_csi_frame_start | S_csi_valid)
            S_dbg_csi_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_raw10_seen <= 1'b0;
        else if(S_raw10_frame_start | S_raw10_valid)
            S_dbg_raw10_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_fs_seen <= 1'b0;
        else if(S_ISP_O_tuser)
            S_dbg_isp_fs_seen <= 1'b1;
    end

    always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_isp_valid_seen <= 1'b0;
        else if(S_ISP_O_tvalid)
            S_dbg_isp_valid_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_req_seen <= 1'b0;
        else if(S_vi_ddr_wr_en)
            S_dbg_ddr_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_ddr_ready_seen <= 1'b0;
        else if(S_ddr_user_ready)
            S_dbg_ddr_ready_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_req_seen <= 1'b0;
        else if(S_vo_ddr_rd_en)
            S_dbg_rd_req_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_valid_seen <= 1'b0;
        else if(S_ddr_user_rd_valid)
            S_dbg_rd_valid_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_video_out_rst_n) begin
        if(!S_video_out_rst_n)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_video_out_vsync)
            S_dbg_rd_nonzero_seen <= 1'b0;
        else if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
            S_dbg_rd_nonzero_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_init_seen <= 1'b0;
        else if(S_init_calib_complete)
            S_dbg_mc_init_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_app_rdy_seen <= 1'b0;
        else if(S_mc_dbg_app_rdy)
            S_dbg_mc_app_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_wdf_rdy_seen <= 1'b0;
        else if(S_mc_dbg_wdf_rdy)
            S_dbg_mc_wdf_rdy_seen <= 1'b1;
    end

    always @(posedge S_ddr_clk or negedge S_rst_n) begin
        if(!S_rst_n)
            S_dbg_mc_cmd_pop_seen <= 1'b0;
        else if(S_mc_dbg_cmd_pop)
            S_dbg_mc_cmd_pop_seen <= 1'b1;
    end

    always @(posedge S_hdmi_pixel_clk or negedge S_hdmi_rst_n) begin
        if(!S_hdmi_rst_n) begin
            S_dbg_cam_cfg_seen_sync <= 2'b00;
            S_dbg_hs_seen_sync      <= 2'b00;
            S_dbg_csi_seen_sync     <= 2'b00;
            S_dbg_raw10_seen_sync   <= 2'b00;
            S_dbg_raw_seen_sync    <= 2'b00;
            S_dbg_isp_seen_sync    <= 2'b00;
            S_dbg_ddr_wr_seen_sync <= 2'b00;
            S_dbg_ddr_rd_seen_sync <= 2'b00;
            S_dbg_isp_fs_seen_sync  <= 2'b00;
            S_dbg_isp_valid_seen_sync <= 2'b00;
            S_dbg_ddr_req_seen_sync <= 2'b00;
            S_dbg_ddr_ready_seen_sync <= 2'b00;
            S_dbg_video_nonzero_seen <= 1'b0;
            S_dbg_rd_req_seen_sync <= 2'b00;
            S_dbg_rd_valid_seen_sync <= 2'b00;
            S_dbg_rd_nonzero_seen_sync <= 2'b00;
            S_dbg_mc_init_seen_sync <= 2'b00;
            S_dbg_mc_app_rdy_seen_sync <= 2'b00;
            S_dbg_mc_wdf_rdy_seen_sync <= 2'b00;
            S_dbg_mc_cmd_pop_seen_sync <= 2'b00;
        end
        else begin
            S_dbg_cam_cfg_seen_sync <= {S_dbg_cam_cfg_seen_sync[0], S_dbg_cam_cfg_seen};
            S_dbg_hs_seen_sync      <= {S_dbg_hs_seen_sync[0],      S_dbg_hs_seen};
            S_dbg_csi_seen_sync     <= {S_dbg_csi_seen_sync[0],     S_dbg_csi_seen};
            S_dbg_raw10_seen_sync   <= {S_dbg_raw10_seen_sync[0],   S_dbg_raw10_seen};
            S_dbg_raw_seen_sync    <= {S_dbg_raw_seen_sync[0],    S_dbg_raw_seen};
            S_dbg_isp_seen_sync    <= {S_dbg_isp_seen_sync[0],    S_dbg_isp_seen};
            S_dbg_ddr_wr_seen_sync <= {S_dbg_ddr_wr_seen_sync[0], S_dbg_ddr_wr_seen};
            S_dbg_ddr_rd_seen_sync <= {S_dbg_ddr_rd_seen_sync[0], S_dbg_ddr_rd_seen};
            S_dbg_isp_fs_seen_sync  <= {S_dbg_isp_fs_seen_sync[0],  S_dbg_isp_fs_seen};
            S_dbg_isp_valid_seen_sync <= {S_dbg_isp_valid_seen_sync[0], S_dbg_isp_valid_seen};
            S_dbg_ddr_req_seen_sync <= {S_dbg_ddr_req_seen_sync[0], S_dbg_ddr_req_seen};
            S_dbg_ddr_ready_seen_sync <= {S_dbg_ddr_ready_seen_sync[0], S_dbg_ddr_ready_seen};
            S_dbg_rd_req_seen_sync <= {S_dbg_rd_req_seen_sync[0], S_dbg_rd_req_seen};
            S_dbg_rd_valid_seen_sync <= {S_dbg_rd_valid_seen_sync[0], S_dbg_rd_valid_seen};
            S_dbg_rd_nonzero_seen_sync <= {S_dbg_rd_nonzero_seen_sync[0], S_dbg_rd_nonzero_seen};
            S_dbg_mc_init_seen_sync <= {S_dbg_mc_init_seen_sync[0], S_dbg_mc_init_seen};
            S_dbg_mc_app_rdy_seen_sync <= {S_dbg_mc_app_rdy_seen_sync[0], S_dbg_mc_app_rdy_seen};
            S_dbg_mc_wdf_rdy_seen_sync <= {S_dbg_mc_wdf_rdy_seen_sync[0], S_dbg_mc_wdf_rdy_seen};
            S_dbg_mc_cmd_pop_seen_sync <= {S_dbg_mc_cmd_pop_seen_sync[0], S_dbg_mc_cmd_pop_seen};
            if(S_hdmi_window_rd_en && (S_video_rd_data != 24'd0))
                S_dbg_video_nonzero_seen <= 1'b1;
        end
    end

    assign S_lane_error_any = |S_lane_error;

    assign S_vo_dbg_fifo_rst = 1'b0;
    assign S_vo_dbg_ddr_rd_valid = 1'b0;
    assign S_vo_dbg_fifo_rd_en = 1'b0;
    assign S_vo_dbg_fifo_empty = 1'b0;
    assign S_mc_dbg_fifo_afull = 1'b0;
    assign S_mc_dbg_app_rdy = 1'b0;
    assign S_mc_dbg_wdf_rdy = 1'b0;
    assign S_mc_dbg_cmd_pop = 1'b0;

	
	// SC520CS上电后先保持PWDNB/XSHUTDN为低，再释放并等待内部上电完成，
	// 避免PLL刚锁定就开始I2C初始化而造成寄存器偶发写入失败。
	always @(posedge S_24m_clk or negedge S_pll_lock) begin
		if (!S_pll_lock) begin
			S_cam_power_cnt <= 18'd0;
			S_cam_pwdnb <= 1'b0;
			S_cam_i2c_ready <= 1'b0;
		end else begin
			if (S_cam_power_cnt < CAM_I2C_START_CYCLES)
				S_cam_power_cnt <= S_cam_power_cnt + 1'b1;
			if (S_cam_power_cnt >= CAM_PWDNB_LOW_CYCLES)
				S_cam_pwdnb <= 1'b1;
			if (S_cam_power_cnt >= CAM_I2C_START_CYCLES - 1)
				S_cam_i2c_ready <= 1'b1;
		end
	end

	assign O_cam_rst 	= S_cam_pwdnb;
    assign O_cam_24m 	= S_24m_clk;
	
	
	
	
    PLL u_PLL(
        .refclk      ( I_sys_clk         ),
        .reset    	 ( ~S_board_rst_n    ),
        
        .clk0_out    ( S_100m_clk        ),
        .clk1_out    ( S_24m_clk         ),

        .clk4_out    ( S_hdmi_pixel_clk  ),
        .clk5_out    ( S_hdmi_serial_clk ),

        .lock        ( S_pll_lock        )

    );


	

  
  // W4：手动优先的自动曝光闭环。手动按键/开关占用时冻结自动环；
  // 自动输出与手动电平做 OR 后送入 ae_set（ae_set 内部只认电平节拍）。
  assign W_manual_busy = S_ae_up | S_ae_dn | S_ag_up | S_ag_dn;

  ae_auto_loop u_ae_auto_loop (
      .I_clk_24m ( S_24m_clk       ),
      .I_rst_n   ( S_rst_n         ),
      .I_enable  ( ~W_manual_busy  ),
      .I_y_avg   ( S_y_avg_24m     ),
      .I_ae      ( S_ae            ),
      .I_ag      ( S_ag            ),
      .O_ae_up   ( S_auto_ae_up    ),
      .O_ae_dn   ( S_auto_ae_dn    ),
      .O_ag_up   ( S_auto_ag_up    ),
      .O_ag_dn   ( S_auto_ag_dn    )
  );

  // 亮度统计 75M -> 24M（准静态，每帧一次，两级同步）
  always @(posedge S_24m_clk or negedge S_rst_n) begin
      if (!S_rst_n) begin
          S_y_avg_meta <= 8'd100;
          S_y_avg_24m  <= 8'd100;
      end else begin
          S_y_avg_meta <= S_y_avg_pix;
          S_y_avg_24m  <= S_y_avg_meta;
      end
  end

  ae_set u_ae_set (
      .I_clk(S_24m_clk),
      .I_rst(~S_rst_n),
      .I_ae_up(S_ae_up | S_auto_ae_up),
      .I_ae_dn(S_ae_dn | S_auto_ae_dn),
      .I_ag_up(S_ag_up | S_auto_ag_up),
      .I_ag_dn(S_ag_dn | S_auto_ag_dn),
      .I_cam_cfg_done(S_cam_cfg_done),
      .I_ae_cfg_done(S_ae_cfg_done),
      .O_ae_req(S_ae_req),
      .O_ae(S_ae),
      .O_ag(S_ag)
  );

  // Teammate v1.13.2 fill light, driven by the same frame-luma statistic as AE.
  // S_y_avg_24m is already in this 23.809524 MHz (42 ns) control domain.
  reg [1:0] S_fill_reset_sync;
  always @(posedge S_24m_clk or negedge S_rst_n) begin
      if (!S_rst_n)
          S_fill_reset_sync <= 2'b00;
      else
          S_fill_reset_sync <= {S_fill_reset_sync[0], 1'b1};
  end
  wire W_fill_rst_n = S_fill_reset_sync[1];
  wire [3:0] W_fill_level;
  dsp_fill_light #(
      .N_LED       (64),
      .SERPENTINE  (1),
      .Y_HI        (8'd60),
      .Y_LO        (8'd30),
      .MAX_LV      (4'd8),
      .LED_WHITE   (8'd128),
      .GAP_MS      (10),
      .CYC_T       (30),
      .CYC_T0H     (7),
      .CYC_T1H     (18),
      .CLK_HZ      (23809524)
  ) u_fill_light (
      .I_clk       (S_24m_clk),
      .I_rst_n     (W_fill_rst_n),
      .I_y_avg     (S_y_avg_24m),
      .I_y_valid   (1'b0), // Compatibility input is unused in the proven module.
      .O_ws        (O_fill_din),
      .O_level     (W_fill_level)
  );
  // Preserve the user's existing cable while reproducing the reference PR DB
  // output pad. Both unused-by-other-peripherals pads carry the exact same DIN.
  assign O_fill_din_ref = O_fill_din;

  //--------------------------------------------------------------------------
  // v1.7 曝光按键 KEY3 (B9) / KEY4 (C7)（来自 v1.5）
  //     KEY3 -> AE 加（增亮），KEY4 -> AE 减（减暗）
  // 与 ae_set 同挂 S_24m_clk；模块内自带 3 级同步 + 21 ms 消抖，
  // 输出"按住期间保持"的高有效电平，交给 ae_set 的节拍连续增减。
  //--------------------------------------------------------------------------
  key_ae_switch #(
      .DEBOUNCE_CYCLES ( 500000 ),   // 约 21 ms @ 23.809524 MHz
      .UP_IS_KEY3      ( 1      )    // 1: KEY3=曝光加, KEY4=曝光减
  ) u_key_ae_switch (
      .I_clk      ( S_24m_clk   ),
      .I_rst_n    ( S_rst_n     ),
      .I_key3_n   ( I_key3      ),
      .I_key4_n   ( I_key4      ),
      .O_ae_up    (             ),
      .O_ae_dn    (             ),
      .O_up_pulse (             ),
      .O_dn_pulse (             )
  );

  //--------------------------------------------------------------------------
  // 历史 AG 开关模块的输出未使用；实际 SW1 增益控制来自 u_vehicle_controls。
  // SW2 现为车牌显示专用开关，此处将历史减益输入固定为空闲电平。
  //--------------------------------------------------------------------------
  switch_ag_ctrl #(
      .DEBOUNCE_CYCLES ( 500000 ),   // 约 21 ms @ 23.809524 MHz
      .UP_LEVEL        ( 1'b0   ),   // 历史模块有效电平；当前两个输出均未使用
      .SWAP_DIR        ( 0      )    // 0: SW1=增/SW2=减
  ) u_switch_ag_ctrl (
      .I_clk   ( S_24m_clk ),
      .I_rst_n ( S_rst_n   ),
      .I_sw1   ( I_sw1     ),
      .I_sw2   ( 1'b1     ),
      .O_ag_up (           ),
      .O_ag_dn (           )
  );

  //--------------------------------------------------------------------------
  // v1.7 图像处理模式与参数按键
  //   KEY1 (D5)：短按按算法编号 0..19 顺序切换二十种图像处理模式；
  //              长按超过 0.5 s 后每 0.1 s 自动切换一次（持续切换，松手停止）
  //   KEY2 (A9)：在当前模式为 7(亮度) / 8(饱和度) 时调整其参数
  // 按键用 24M 时钟采样（PLL 锁定后即可用，不依赖 MIPI 时钟），
  // 结果再同步到 MIPI 视频时钟域去控制 ISP 里的数据选择与参数。
  //--------------------------------------------------------------------------
  key_mode_switch #(
      .DEBOUNCE_CYCLES           ( 500000  ),   // 约 21 ms @ 23.809524 MHz
      .N_MODES                   ( 20      ),   // 二十种摄像头画面模式
      .INIT_MODE                 ( 0       ),   // 上电默认模式 0 = 原始 RGB
      .MODE_W                    ( 5       ),
      .HOLD_REPEAT_EN            ( 1       ),   // v1.7：长按持续切换
      .HOLD_REPEAT_DELAY_CYCLES  ( 11904762),   // 约 0.5 s @ 23.809524 MHz
      .HOLD_REPEAT_PERIOD_CYCLES ( 2380952 )    // 约 0.1 s @ 23.809524 MHz
  ) u_key_mode_switch (
      .I_clk        ( S_24m_clk      ),
      .I_rst_n      ( S_rst_n        ),
      .I_key_n      ( S_camera_key_n_24m ),
      .O_mode       ( W_camera_mode_index ),
      .O_mode_pulse (                )
  );

  // 模式号直接用于参数控制、显示滤镜和 OSD，顺序与历史摄像头工程一致。
  assign S_key_mode = W_camera_mode_index;

  key_press #(
      .DEBOUNCE_CYCLES ( 500000 )
  ) u_key_adj (
      .I_clk   ( S_24m_clk   ),
      .I_rst_n ( S_rst_n     ),
      .I_key_n ( I_key2      ),
      .O_press (             )
  );

  img_mode_ctrl u_img_mode_ctrl (
      .I_clk        ( S_24m_clk     ),
      .I_rst_n      ( S_rst_n       ),
      .I_mode       ( S_key_mode    ),
      .I_adj_pulse  ( S_adj_pulse   ),
      .O_brightness ( S_brightness  ),
      .O_sat_factor ( S_sat_factor  )
  );

  // 模式与参数跨时钟域：每比特打两拍后使用
  // （按键驱动的慢速控制信号，最坏情况只影响一帧，无需握手）
  always @(posedge S_csi_rx_clk or negedge S_rst_n) begin
      if(!S_rst_n) begin
          S_mode_meta       <= 5'd0;
          S_mode_video_sync <= 5'd0;
          S_bright_meta     <= 9'sd0;
          S_bright_sync     <= 9'sd0;
          S_sat_meta        <= 8'd128;
          S_sat_sync        <= 8'd128;
      end else begin
          S_mode_meta       <= S_key_mode;
          S_mode_video_sync <= S_mode_meta;
          S_bright_meta     <= S_brightness;
          S_bright_sync     <= S_bright_meta;
          S_sat_meta        <= S_sat_factor;
          S_sat_sync        <= S_sat_meta;
      end
  end

  assign S_mode_video = S_mode_video_sync;
  
    // SC520CS原厂真实HD模式：1280x720@60、RAW10、2-Lane、828 Mbps/Lane。
    // 当前寄存器表已关闭Slave/FSIN等待，Sensor以free-run方式连续输出。
    uicfgcs520 #(
      .CLK_DIV(23809524 / 100000 - 1)
  ) u_uicfgcs520 (
      .I_clk(S_24m_clk),  //系统时钟输入
      .I_rst_n(S_cam_i2c_ready), // SC520CS power-up delay completed
      .I_ae_req(S_ae_req),
      .I_ae(S_ae),
      .I_ag(S_ag),
      .O_cam_scl(O_cam_scl),  //I2C总线，SCL时钟
      .IO_cam_sda(IO_cam_sda),  //I2C总线，SDA数据
      .O_cfg_done(S_cam_cfg_done),  //摄像头寄存器初始化完成
      .O_ae_cfg_done(S_ae_cfg_done),  //AE配置完成
      .O_sensor_id(S_cam_sensor_id),
      .O_sensor_id_ok(S_cam_sensor_id_ok),
      .O_iic_error(S_cam_i2c_error)
  );
	
    mipi_dphy_rx_ph1p_mipiio_wrapper#(
        .DPHY_RX_LOCATION      ( "DPHY0" ),
        .HS_EQUALIZER          ( "3dB"   ),// 828 Mbps/Lane接收均衡
        .HS_VGA_GAIN           ( "8dB" ),//"-3dB","-1.5dB","0dB","1.5dB","3dB","4.5dB","6dB","7.5dB"
        .LANE_NUM              ( 2       ),
        .BYTE_NUM              ( 1       )
    )u_mipi_dphy_rx_ph1p_mipiio_wrapper(
        .I_lp_clk              ( S_100m_clk       ),
        .I_rst                 ( ~S_rst_n         ),

        .I_clk_lane_in_delay   ( 6'd0             ),
        .I_data_lane0_in_delay ( 6'd0             ),
        .I_data_lane1_in_delay ( 6'd0             ),
        .I_data_lane2_in_delay ( 6'd0             ),
        .I_data_lane3_in_delay ( 6'd0             ),

        .I_lane_invert         ( 4'b0000          ),
     
        .O_hs_rx_clk           ( S_csi_rx_clk     ),
        .O_hs_rx_valid         ( S_hs_rx_valid    ),
        .O_hs_rx_data          ( S_hs_rx_data     ),
      
        .O_lp_rx_lane0_p       (  ),
        .O_lp_rx_lane0_n       (  ),
      
        .I_lp_tx_en            ( 1'b0             ),
        .I_lp_tx_lane0_p       ( 1'b1             ),
        .I_lp_tx_lane0_n       ( 1'b1             ),
      
        .O_lane_match_error    (                  ),
        .O_lane_error          ( S_lane_error     ),
      
        .IO_rx_clk_pad_n       ( IO_rx_clk_pad_n  ),
        .IO_rx_clk_pad_p       ( IO_rx_clk_pad_p  ),
        .IO_rx_data_pad_n      ( IO_rx_data_pad_n ),
        .IO_rx_data_pad_p      ( IO_rx_data_pad_p )
    );
	
	
 //  cwc cwc_inst
 //(
 //    .probe0(S_hs_rx_valid),
 //    .probe1(S_hs_rx_data),
 //    .probe2(S_lane_error),
 //    .I_clk_pix(S_csi_rx_clk)
 //);

 //csi 解码为RAW数据
csi_unpacket_2lane u_csi_unpacket(
.I_clk                 ( S_csi_rx_clk       ),
.I_rst_n               ( S_rst_n        	),
.I_hs_valid            ( S_hs_rx_valid     ),
.I_hs_data             ( S_hs_rx_data      ),

.O_csi_frame_start     ( S_csi_frame_start ),
.O_csi_frame_end       ( S_csi_frame_end   ),
.O_csi_valid           ( S_csi_valid       ),
.O_csi_data            ( S_csi_data        )
);


//解码为RAW8
raw10_unpacket_2lane u_raw10_unpacket (
.I_clk  (S_csi_rx_clk),
.I_rst_n(S_rst_n),

.I_csi_frame_start(S_csi_frame_start),
.I_csi_frame_end  (S_csi_frame_end),
.I_csi_valid      (S_csi_valid),
.I_csi_data       (S_csi_data),

.O_raw10_frame_start(S_raw10_frame_start),
.O_raw10_frame_end  (S_raw10_frame_end),
.O_raw10_valid      (S_raw10_valid),
.O_raw10_data       (S_raw10_data)
  );


//将数据转为stream流
uial2axis #(
.IMG_WIDTH(1280),
.IMG_HEIGHT(720),
.INPUT_DATA_WIDTH(40)
) 
u_uial2axis (
.I_native_clk(S_csi_rx_clk),
.I_rst_n     (S_rst_n),
.I_data      (S_raw10_data       ),
.I_data_valid(S_raw10_valid      ),
.I_data_start(S_raw10_frame_start),
.I_data_end  (S_raw10_frame_end  ),
.axis_tvalid (S_axis_tvalid),
.axis_tdata  (S_axis_tdata ),
.axis_tuser  (S_axis_tuser ),
.axis_tlast  (S_axis_tlast )
);
	

// 原厂HD模式直接输出1280x720 RAW10，不再进行FPGA固定窗口裁剪。
assign S_raw_tdata  = S_axis_tdata;
assign S_raw_tlast  = S_axis_tlast;
assign S_raw_tvalid = S_axis_tvalid;
assign S_raw_tuser  = S_axis_tuser;


//cwc1 cwc1_Inst
//  (
//      .probe0(S_axis_tuser),
//      .probe1(S_axis_tvalid),
//      .probe2(S_axis_tlast),
//      .probe3(S_axis_tdata),
//      .probe4(S_ISP_O_tuser),
//      .probe5(S_ISP_O_tvalid),
//      .probe6(S_ISP_O_tlast),
//      .probe7(S_ISP_O_tdata),
//      .probe8 (S_raw_tdata ),
//      .probe9 (S_raw_tlast ),
//      .probe10(S_raw_tvalid),
//      .probe11(S_raw_tuser ),
//      .I_clk_pix(S_csi_rx_clk)
//  );


//ISP算法顶层模块
isp_top u_isp_top (
.axi4s_video_aclk(S_csi_rx_clk),
.I_rst_n         (S_rst_n),
.I_tlast         (S_raw_tlast	),
.I_tuser         (S_raw_tuser	),
.I_tdata         (S_raw_tdata	),
.I_tvalid        (S_raw_tvalid	),
.I_tdest         (),
.I_mode          (5'd0),
.I_brightness    (9'sd0),
.I_sat_factor    (8'd128),
.O_tready        (S_ISP_O_tready),
.O_tdata         (S_ISP_O_tdata ),
.O_tlast         (S_ISP_O_tlast ),
.O_tuser         (S_ISP_O_tuser ),
.O_tvalid        (S_ISP_O_tvalid),
.I_tready        (S_ISP_I_tready)
  );


    video_in u_video_in(
        .I_rst_n              (  S_rst_n             ),

        .I_camera_clk         ( S_csi_rx_clk          ),
        .I_camera_frame_start ( S_ISP_O_tuser  ),
        .I_camera_valid       ( S_ISP_O_tvalid ),
        .I_camera_data        ( S_ISP_O_tdata  ),
        .I_mipi_rx_error      ( 1'b0			      ),

        .I_ddr_clk            ( S_ddr_clk             ),
		.I_display_pause      ( 1'b0                  ),
        .I_video_out_rd_busy  ( S_video_out_rd_busy   ),
        .O_video_in_wr_busy   ( S_video_in_wr_busy    ),
        .O_video_out_rp       ( S_video_out_rp        ),

        .O_ddr_user_wr_en     ( S_vi_ddr_wr_en        ),
        .O_ddr_user_addr      ( S_vi_ddr_wr_addr      ),
        .O_ddr_user_wr_data   ( S_vi_ddr_wr_data      ),
        .I_ddr_user_ready     ( S_ddr_user_ready      )
    );



    assign S_ddr_user_wr_en    = S_vi_ddr_wr_en;

    assign S_ddr_user_rd_en    = S_vo_ddr_rd_en;

    assign S_ddr_user_addr     = S_vi_ddr_wr_en ? S_vi_ddr_wr_addr :
                                 S_vo_ddr_rd_en ? S_vo_ddr_rd_addr : 'd0;

    assign S_ddr_user_wr_data  = S_vi_ddr_wr_data;


    assign S_video_out_rst_n = S_hdmi_rst_n;

    video_out u_video_out(
        .I_rst_n             ( S_video_out_rst_n    ),
        .I_ddr_clk           ( S_ddr_clk            ),
 
        .O_video_out_rd_busy ( S_video_out_rd_busy  ),
        .I_video_in_wr_busy  ( S_video_in_wr_busy   ),
        .I_video_out_rp      ( S_video_out_rp       ),
 
        .O_ddr_user_rd_en    ( S_vo_ddr_rd_en       ),
        .O_ddr_user_addr     ( S_vo_ddr_rd_addr     ),
        .I_ddr_user_ready    ( S_ddr_user_ready     ),
        .I_ddr_user_rd_valid ( S_ddr_user_rd_valid  ),
        .I_ddr_user_rd_data  ( S_ddr_user_rd_data   ),

        .I_dsi_clk           ( S_hdmi_pixel_clk     ),
        .I_video_vsync       ( S_video_out_vsync    ),
        .I_video_rd_en       ( S_hdmi_window_rd_en  ),
        .O_vdieo_data        ( S_video_rd_data      ),
        .O_video_data_valid  ( S_video_data_valid   )
    );


    //==========================================================================
    // 端到端延迟检测（v1.13 功能补齐）
    //   采集侧由 video_in 在每个采集帧起始把帧写进 4 个 bank 之一；显示侧由
    //   video_out 按 (写指针+2)%4 选帧。buf_display_latency_probe 在 DDR 域
    //   给每个 bank 记采集帧票号，并在显示帧起始算出被显示帧的帧龄；
    //   dsp_latency_monitor 在 75 MHz 像素域做显示帧去抖和毫秒换算；
    //   dsp_seg_display 驱动板载 8 位数码管显示 "LAt.xxx" 毫秒。
    //   正常 720p60 稳态 = 2 帧 / 33 ms，与 v1.13 的四帧映射结论一致。
    //==========================================================================
    wire [4:0] W_lat_frames_ddr;    // DDR 域：显示帧起始发布的帧龄（帧）
    wire       W_lat_update_ddr;    // DDR 域：发布 toggle
    wire [4:0] W_lat_frames_pix;    // 像素域：去抖后的帧龄（帧）
    wire [9:0] W_lat_ms_pix;        // 像素域：去抖后的端到端延迟（ms）

    buf_display_latency_probe u_latency_probe (
        .I_clk_ddr        ( S_ddr_clk            ),
        .I_rst_n          ( S_rst_n              ),
        .I_wr_en          ( S_vi_ddr_wr_en       ),
        .I_wr_addr        ( S_vi_ddr_wr_addr     ),
        .I_display_rp     ( S_video_out_rp       ),
        .I_display_vsync  ( S_video_out_vsync    ),
        .O_lat_frames     ( W_lat_frames_ddr     ),
        .O_lat_update     ( W_lat_update_ddr     )
    );

    dsp_latency_monitor u_latency_monitor (
        .I_clk_pix             ( S_hdmi_pixel_clk     ),
        .I_rst_n               ( S_hdmi_rst_n         ),
        .I_lat_frames_async    ( W_lat_frames_ddr     ),
        .I_lat_update_async    ( W_lat_update_ddr     ),
        .O_lat_frames          ( W_lat_frames_pix     ),
        .O_lat_ms              ( W_lat_ms_pix         ),
        .O_lat_valid           (                      )
    );

    dsp_seg_display #(
        .P_SCAN_DIV ( 75000 ),
        .P_BOOT_CYC ( 134217728 )
    ) u_seg_display (
        .I_clk_pix     ( S_hdmi_pixel_clk  ),
        .I_rst_n       ( S_hdmi_rst_n      ),
        .I_pol         ( 2'b10             ),
        .I_lat_frames  ( W_lat_frames_pix  ),
        .I_lat_ms      ( W_lat_ms_pix      ),
        .O_seg_dig     ( O_seg_dig         ),
        .O_seg_sel     ( O_seg_sel         )
    );


    mc_to_user_interface u_mc_to_user_interface(
        .I_clk                   ( S_ddr_clk               ),
        .I_rst_n                 ( S_rst_n                 ),

        .I_ddr_user_wr_en        ( S_ddr_user_wr_en        ),
        .I_ddr_user_rd_en        ( S_ddr_user_rd_en        ),
        .I_ddr_user_addr         ( S_ddr_user_addr         ),
        .I_ddr_user_wr_data      ( S_ddr_user_wr_data      ),
        .O_ddr_user_ready        ( S_ddr_user_ready        ),
        .O_ddr_user_rd_valid     ( S_ddr_user_rd_valid     ),
        .O_ddr_user_rd_data      ( S_ddr_user_rd_data      ),
            
        .O_mc_app_en             ( S_mc_app_en             ),
        .O_mc_app_addr           ( S_mc_app_addr           ),
        .O_mc_app_cmd            ( S_mc_app_cmd            ),
        .I_mc_app_rdy            ( S_mc_app_rdy            ),
        .O_mc_app_wdf_wren       ( S_mc_app_wdf_wren       ),
        .O_mc_app_wdf_data       ( S_mc_app_wdf_data       ),
        .O_mc_app_wdf_end        ( S_mc_app_wdf_end        ),
        .O_mc_app_wdf_mask       ( S_mc_app_wdf_mask       ),
        .I_mc_app_wdf_rdy        ( S_mc_app_wdf_rdy        ),
        .I_mc_app_rd_data        ( S_mc_app_rd_data        ),
        .I_mc_app_rd_data_end    ( S_mc_app_rd_data_end    ),
        .I_mc_app_rd_data_valid  ( S_mc_app_rd_data_valid  )
    );



    ph1p35_324_ddr_wrapper u_ph1p35_324_ddr_wrapper(
        .I_sys_clk              ( I_sys_clk              ),
        .I_sys_rst_n            ( S_rst_n                ),

        .O_ddr_clk              ( S_ddr_clk              ),
        .O_init_calib_complete  ( S_init_calib_complete  ),
        .I_mc_app_addr          ( S_mc_app_addr          ),
        .I_mc_app_cmd           ( S_mc_app_cmd           ),
        .I_mc_app_en            ( S_mc_app_en            ),
        .I_mc_app_wdf_data      ( S_mc_app_wdf_data      ),
        .I_mc_app_wdf_end       ( S_mc_app_wdf_end       ),
        .I_mc_app_wdf_mask      ( S_mc_app_wdf_mask      ),
        .I_mc_app_wdf_wren      ( S_mc_app_wdf_wren      ),
        .O_mc_app_rd_data       ( S_mc_app_rd_data       ),
        .O_mc_app_rd_data_end   ( S_mc_app_rd_data_end   ),
        .O_mc_app_rd_data_valid ( S_mc_app_rd_data_valid ),
        .O_mc_app_rdy           ( S_mc_app_rdy           ),
        .O_mc_app_wdf_rdy       ( S_mc_app_wdf_rdy       ),

        .ddr_addr               ( ddr_addr               ),
        .ddr_ba                 ( ddr_ba                 ),
        .ddr_cke                ( ddr_cke                ),
        .ddr_odt                ( ddr_odt                ),
        .ddr_cs_n               ( ddr_cs_n               ),
        .ddr_ras_n              ( ddr_ras_n              ),
        .ddr_cas_n              ( ddr_cas_n              ),
        .ddr_we_n               ( ddr_we_n               ),
        .ddr_ck_p               ( ddr_ck_p               ),
        .ddr_ck_n               ( ddr_ck_n               ),
        .ddr_dm                 ( ddr_dm                 ),
        .ddr_dq                 ( ddr_dq                 ),
        .ddr_dqs_p              ( ddr_dqs_p              ),
        .ddr_dqs_n              ( ddr_dqs_n              )
    );



    uivtc #(
        .H_ActiveSize ( 1280 ),
        .H_FrameSize  ( 1650 ),
        .H_SyncStart  ( 1390 ),
        .H_SyncEnd    ( 1430 ),
        .V_ActiveSize ( 720  ),
        .V_FrameSize  ( 750  ),
        .V_SyncStart  ( 725  ),
        .V_SyncEnd    ( 730  )
    )u_hdmi_vtc(
        .I_vtc_rstn    ( S_hdmi_rst_n    ),
        .I_vtc_clk     ( S_hdmi_pixel_clk ),
        .O_vtc_vs      ( S_hdmi_vsync     ),
        .O_vtc_hs      ( S_hdmi_hsync     ),
        .O_vtc_de_valid( S_hdmi_de        ),
        .O_vtc_user    ( S_hdmi_user      ),
        .O_vtc_last    ( S_hdmi_last      )
    );

    // W4：显示域帧平均亮度统计（挂干净 RGB，纯观察者，供自动曝光闭环）
    dsp_luma_stat #(
        .P_WIDTH(1280), .P_HEIGHT(720)
    ) u_luma_stat (
        .I_clk_pix ( S_hdmi_pixel_clk  ),
        .I_rst_n   ( S_hdmi_rst_n      ),
        .I_video   ( S_video_rd_data   ),
        .I_de      ( S_video_data_valid),
        .I_vsync   ( S_video_vs_pipe[1]),
        .O_y_avg   ( S_y_avg_pix       ),
        .O_valid   (                   )
    );

    dsp_camera_modes #(
        .P_CORE_BUSINESS(P_CORE_BUSINESS)
    ) u_camera_modes (
        .I_clk_pix(S_hdmi_pixel_clk), .I_rst_n(S_hdmi_rst_n),
        .I_mode(S_display_mode_pix), .I_brightness(S_display_brightness_pix),
        .I_sat_factor(S_display_sat_factor_pix),
        .I_bin_th(8'd128), .I_cartoon_th(8'd140),
        .I_vsync(S_video_vs_pipe[1]), .I_hsync(S_video_hs_pipe[1]),
        .I_de(S_video_data_valid), .I_user(S_video_user_pipe[1]),
        .I_last(S_video_last_pipe[1]), .I_video(S_video_rd_data),
        .I_rx_valid(S_stream_rx_valid_pix), .I_rx_length(S_stream_rx_length_pix),
        .O_rx_address(S_stream_rx_address_pix), .I_rx_data(S_stream_rx_data_pix),
        .O_rx_release(S_stream_rx_release_pix), .I_tx_ready(S_stream_tx_ready_pix),
        .O_tx_write(S_stream_tx_write_pix), .O_tx_address(S_stream_tx_address_pix),
        .O_tx_data(S_stream_tx_data_pix), .O_tx_commit(S_stream_tx_commit_pix),
        .O_vsync(S_mode_vsync_pix), .O_hsync(S_mode_hsync_pix),
        .O_de(S_mode_de_pix), .O_user(S_mode_user_pix), .O_last(S_mode_last_pix),
        .O_video(S_mode_rgb_pix), .O_link_ok(S_mode_link_ok_pix),
        .O_remote_used(S_mode_remote_used_pix)
    );

    alg_vehicle_services u_vehicle_services (
        .I_clk_pix(S_hdmi_pixel_clk), .I_clk_24m(S_24m_clk), .I_rst_n(S_hdmi_rst_n),
        .I_rgb(S_mode_rgb_pix), .I_de(S_mode_de_pix),
        .I_user(S_mode_user_pix), .I_last(S_mode_last_pix),
        .I_bbox_frame(S_video_user_pipe[1]), .I_bbox_valid(S_plate_detected),
        .I_bbox_x0({1'b0, S_plate_left}), .I_bbox_y0({1'b0, S_plate_up}),
        .I_bbox_x1({1'b0, S_plate_right}), .I_bbox_y1({1'b0, S_plate_down}),
        .I_ocr_plate(S_res), .I_ocr_res_n(S_res_n),
        .I_ocr_valid(S_res_valid), .I_ocr_frame(S_frame_result),
        .I_track_enable(S_track_enable_pix), .I_jog_enable(S_jog_enable_pix),
        .I_jog_left(S_jog_left_pix), .I_jog_right(S_jog_right_pix),
        .I_jog_up(S_jog_up_pix), .I_jog_down(S_jog_down_pix), .I_home(S_gimbal_home_pix),
        .I_tf_key_n(S_tf_key_n_24m), .I_tf_miso(I_tf_miso),
        .O_rgb(S_service_rgb_pix), .O_de(S_service_de_pix),
        .O_user(S_service_user_pix), .O_last(S_service_last_pix),
        .O_pan_angle(S_pan_angle), .O_tilt_angle(S_tilt_angle), .O_track_locked(S_locked), .O_track_cx(), .O_track_cy(),
        .O_dist_mm(W_service_dist_mm_pix), .O_speed_mmps(W_service_speed_mmps_pix),
        .O_plate_w_px(), .O_track_state(W_service_metrics_state_pix),
        .O_metrics_valid(W_service_metrics_valid_pix),
        .O_tf_clk(O_tf_clk), .O_tf_cs_n(O_tf_cs_n), .O_tf_mosi(O_tf_mosi),
        .O_tf_plate(), .O_tf_plate_valid(), .O_tf_card_ready(), .O_tf_status(), .O_led_plate(),
        .O_plate_osd(S_service_ocr_pix), .O_plate_osd_n(S_service_ocr_count_pix),
        .O_plate_osd_valid(S_service_ocr_valid_pix)
    );

    //==========================================================================
    // 总线舵机 MCU 子系统（取代原先两路 servo_pwm 模拟 PWM）
    //   gimbal_ctrl 原样保留，算好的 S_pan_angle / S_tilt_angle 经 AHB 寄存器桥
    //   交给 PH1P 硬核 RISC-V MCU，MCU 用自带 UART1 组总线帧发给松甲总线舵机。
    //   固件 G:\fding\gimbal_servo 完全不变（FABRIC_REG_BASE=0x80010000）。
    //   详见 rtl/servo/mcu_servo_subsystem.v 文件头（时钟/复位/接线约定）。
    //==========================================================================
    mcu_servo_subsystem u_servo_mcu (
        .I_sys_clk    ( I_sys_clk           ),   // 50MHz 板载时钟（子系统内部 PLL 参考）
        .I_rst_n      ( S_rst_n             ),   // 系统复位（PLL 锁定）
        .I_hdmi_clk   ( S_hdmi_pixel_clk    ),   // 75MHz 追踪域时钟
        .I_hdmi_rst_n ( S_hdmi_rst_n        ),
        .I_frame      ( S_video_user_pipe[1]),   // 每帧单拍脉冲
        .I_valid      ( S_plate_detected    ),   // 车牌检测有效
        .I_locked     ( S_locked            ),   // 云台锁定
        .I_pan        ( S_pan_angle         ),
        .I_tilt       ( S_tilt_angle        ),
        .O_servo_bus  ( O_servo_bus         )
    );

    hdmi_mixer #(
        .H_OFFSET   ( 0    ),
        .V_OFFSET   ( 0    ),
        .IMG_WIDTH  ( 1280 ),
        .IMG_HEIGHT ( 720  )
    )u_hdmi_mixer(
        .I_clk           ( S_hdmi_pixel_clk   ),
        .I_rst_n         ( S_hdmi_rst_n       ),
        .I_video_vsync   ( S_mode_vsync_pix ),
        .I_video_hsync   ( S_mode_hsync_pix ),
        .I_video_de      ( S_service_de_pix ),
        .I_video_user    ( S_service_user_pix ),
        .I_video_last    ( S_service_last_pix ),
        .I_alg_mode      ( S_display_mode_pix ),
        // Seven 48x72 cells form a 336x72 plate at (472,144).
        .I_res           ( S_key_res ),
        .I_res_n         ( S_key_res_n ),
        .I_res_valid     ( S_key_res_valid ),
        .O_video_rd_en   (                   ),
        .I_video_rd_data ( S_service_rgb_pix    ),
        .O_hdmi_vsync    ( S_hdmi_out_vsync   ),
        .O_hdmi_hsync    ( S_hdmi_out_hsync   ),
        .O_hdmi_de       ( S_hdmi_out_de      ),
        .O_hdmi_data     ( S_hdmi_out_data    )
    );

    // ===================== 车牌定位（v1.7 新增） =====================
    // 在 DDR 回读的干净 RGB 流上检测蓝色车牌区域，输出其外接矩形（1 帧延迟）。
    plate_locate u_plate_locate(
        .clk             ( S_hdmi_pixel_clk    ),
        .rst_n           ( S_hdmi_rst_n        ),
        .i_rgb      ( S_video_rd_data     ),
        .i_de       ( S_video_data_valid  ),
        .i_vsync    ( S_video_vs_pipe[1]  ),
        .o_up       ( S_plate_up          ),
        .o_down     ( S_plate_down        ),
        .o_left     ( S_plate_left        ),
        .o_right    ( S_plate_right       ),
        .o_detected ( S_plate_detected    ),
        .o_result_update ( S_plate_result_update )
    );

    // ===================== 字符边界提取（v1.7 功能二） =====================
    // 在车牌框内做自适应二值化和行列投影，输出字符带及七个字符左右边界。
    char_locate u_char_locate(
        .clk             ( S_hdmi_pixel_clk    ),
        .rst_n           ( S_hdmi_rst_n        ),
        .i_rgb      ( S_video_rd_data     ),
        .i_de       ( S_video_data_valid  ),
        .i_vsync    ( S_video_vs_pipe[1]  ),
        .i_up       ( S_plate_up          ),
        .i_down     ( S_plate_down        ),
        .i_left     ( S_plate_left        ),
        .i_right    ( S_plate_right       ),
        .i_detected ( S_plate_detected    ),
        .o_char_up    ( S_char_up    ),
        .o_char_down  ( S_char_down  ),
        .o_c1_left    ( S_c1_left    ), .o_c1_right ( S_c1_right ),
        .o_c2_left    ( S_c2_left    ), .o_c2_right ( S_c2_right ),
        .o_c3_left    ( S_c3_left    ), .o_c3_right ( S_c3_right ),
        .o_c4_left    ( S_c4_left    ), .o_c4_right ( S_c4_right ),
        .o_c5_left    ( S_c5_left    ), .o_c5_right ( S_c5_right ),
        .o_c6_left    ( S_c6_left    ), .o_c6_right ( S_c6_right ),
        .o_c7_left    ( S_c7_left    ), .o_c7_right ( S_c7_right ),
        .o_char_detected ( S_char_detected ),
        .o_char_valid    ( S_char_valid    ),
        .o_char_ink      ( S_char_ink      ),
        .o_char_ink_de   ( S_char_ink_de   )
    );

    // W2: INK OCR recognition cascade removed.  plate_locate (框选) and
    // char_locate (字符分割) are kept; only the EG4D C13/C14 recognition and
    // the 0x88B5 legacy link are gone.  Tie off the former recognition result,
    // diagnostic, and TX-arbitration signals so the 3-client scheduler, the
    // vehicle-services OSD, and the status overlay keep their contracts with
    // no dangling drivers.  The OCR result was OSD text only; distance/speed/
    // tracking come from the plate bbox and are unaffected.
    assign S_res = 56'd0;
    assign S_res_n = 4'd0;
    assign S_res_valid = 1'b0;
    assign S_raw_valid = 7'd0;
    assign S_frame_result = 1'b0;
    assign S_raw_candidate_code = 56'd0;
    assign S_raw_candidate_mask = 7'd0;
    assign S_match_state_public = 4'd0;
    assign W_ink_diag_status_pix = 16'd0;
    assign W_ink_diag_counts_pix = 16'd0;
    assign W_ink_tx_request_tx = 1'b0;
    assign W_ink_tx_done_tx = 1'b1;
    assign W_ink_gmii_txd_tx = 8'd0;
    assign W_ink_gmii_tx_en_tx = 1'b0;
    assign W_ink_gmii_tx_er_tx = 1'b0;
    assign W_ink_legacy_rx_valid_pix = 1'b0;

    // ------------------------------------------------------------------
    // W3: 0x88B5 key-state receive path and button-driven plate OSD text.
    // The 0x88B5 link carries the EG4D key-state packet on RX and PH telemetry
    // on TX. RX feeds ctl_key_state_rx, whose key code drives
    // dsp_key_plate_osd -> S_key_res -> hdmi_mixer.I_res. The key-auto and
    // reset-toggle fields remain in the same legal frame as the key code.
    // ------------------------------------------------------------------
    ctl_key_state_rx u_key_rx (
        .clk            ( S_hdmi_pixel_clk ),
        .rst_n          ( S_hdmi_rst_n     ),
        .I_rx_valid     ( W_key_rx_valid   ),
        .I_rx_length    ( W_key_rx_length  ),
        .O_rx_addr      ( W_key_rx_addr    ),
        .I_rx_data      ( W_key_rx_data    ),
        .O_rx_release   ( W_key_rx_release ),
        .O_key_code     ( W_key_code_pix   ),
        .O_key_valid    ( W_key_valid_pix  ),
        .O_key_changed  ( W_key_changed_pix),
        .O_cycle_reset_request( W_cycle_reset_request_pix ),
        .O_key_auto      ( W_key_auto_pix   ),
        .O_rx_good_count( ),
        .O_rx_bad_count ( )
    );

    // The existing plate_locate recomputes o_detected at S_DONE for every
    // frame. Qualify the cycle with the result-commit pulse so the detector
    // samples the new result instead of the previous frame's held value.
    ctl_plate_cycle_detect u_plate_cycle_detect (
        .I_clk            ( S_hdmi_pixel_clk          ),
        .I_rst_n          ( S_eth_app_reset_n         ),
        .I_plate_sample   ( S_plate_result_update    ),
        .I_plate_detected ( W_plate_fresh_pix         ),
        .I_reset_request  ( W_cycle_reset_request_pix ),
        .O_cycle_active   ( W_plate_cycle_active_pix  ),
        .O_reset_ack      ( W_plate_cycle_reset_ack_pix )
    );

    // Match the gold qualification rule: automatic text is valid only while
    // a plate cycle is active; manual key text is valid outside that cycle.
    // SW2 uses the existing debounced, frame-committed control snapshot.
    // Gate text only; plate detection, telemetry, cycles and audio still run.
    wire W_key_osd_valid_pix = S_plate_display_enable_pix && W_key_valid_pix &&
        (W_key_auto_pix ? W_plate_cycle_active_pix : !W_plate_cycle_active_pix);

    dsp_key_plate_osd u_key_plate_osd (
        .I_clk       ( S_hdmi_pixel_clk ),
        .I_rst_n     ( S_hdmi_rst_n     ),
        .I_key_code  ( W_key_code_pix   ),
        .I_key_valid ( W_key_osd_valid_pix ),
        .I_vsync     ( S_hdmi_out_vsync ),
        .O_res       ( S_key_res        ),
        .O_res_n     ( S_key_res_n      ),
        .O_res_valid ( S_key_res_valid  )
    );

    wire [9:0]  W_telemetry_wr_addr;
    wire [7:0]  W_telemetry_wr_data;
    wire        W_telemetry_wr_en;
    wire        W_telemetry_commit;
    wire [10:0] W_telemetry_length;
    wire        W_telemetry_tx_busy;

    ctl_telemetry_tx u_telemetry_tx (
        .clk_app                 ( S_hdmi_pixel_clk             ),
        .rst_app_n               ( S_eth_app_reset_n            ),
        .clk_cfg                 ( S_24m_clk                    ),
        .rst_cfg_n               ( S_telemetry_cfg_reset_sync[1] ),
        .I_cfg_ag                ( S_ag                         ),
        .I_cfg_ae                ( S_ae                         ),
        .I_app_speed_mmps        ( W_service_speed_mmps_pix     ),
        .I_app_dist_mm           ( W_service_dist_mm_pix        ),
        .I_app_metrics_valid     ( W_service_metrics_valid_pix  ),
        .I_app_metrics_state     ( W_service_metrics_state_pix  ),
        .I_app_cycle_active      ( W_plate_cycle_active_pix     ),
        .I_app_cycle_reset_ack   ( W_plate_cycle_reset_ack_pix  ),
        .I_tx_busy               ( W_telemetry_tx_busy          ),
        .O_wr_addr               ( W_telemetry_wr_addr          ),
        .O_wr_data               ( W_telemetry_wr_data          ),
        .O_wr_en                 ( W_telemetry_wr_en            ),
        .O_commit                ( W_telemetry_commit           ),
        .O_length                ( W_telemetry_length           )
    );

    // Fixed raw-Ethernet link contract.  ctl_lpr_packet_link performs the
    // application/GMII clock-domain crossings, MAC/EtherType filtering,
    // preamble/FCS, bounded receive buffering, and 46-byte wire padding.
    ctl_lpr_packet_link #(
        .P_LOCAL_MAC (48'h02_00_00_12_34_56),
        .P_PEER_MAC  (48'h02_00_00_65_43_21),
        .P_ETHERTYPE (16'h88B5),
        .P_TX_ARB_EN (1)
    ) u_ctl_lpr_packet_link (
        .I_clk_app                ( S_hdmi_pixel_clk        ),
        .I_rst_app_n              ( S_eth_app_reset_n       ),
        .I_clk_tx                 ( S_eth_gmii_tx_clk       ),
        .I_rst_tx_n               ( W_eth_tx_common_reset_n ),
        .I_clk_rx                 ( S_eth_gmii_rx_clk       ),
        .I_rst_rx_n               ( W_eth_rx_common_reset_n ),
        .I_tx_write_en            ( W_telemetry_wr_en        ),
        .I_tx_write_addr          ( W_telemetry_wr_addr      ),
        .I_tx_write_data          ( W_telemetry_wr_data      ),
        .I_tx_commit              ( W_telemetry_commit        ),
        .I_tx_length              ( W_telemetry_length        ),
        .O_tx_busy                ( W_telemetry_tx_busy      ),
        .O_tx_done                (                         ),
        .I_tx_grant               ( S_ocr_tx_grant          ),
        .O_tx_request             ( S_ocr_tx_request        ),
        .O_tx_done_tx             ( S_ocr_tx_done           ),
        .O_rx_valid               ( W_key_rx_valid          ),
        .O_rx_length              ( W_key_rx_length         ),
        .I_rx_read_addr           ( W_key_rx_addr           ),
        .O_rx_read_data           ( W_key_rx_data           ),
        .I_rx_release             ( W_key_rx_release        ),
        .O_gmii_txd               ( S_ocr_gmii_txd          ),
        .O_gmii_tx_en             ( S_ocr_gmii_tx_en        ),
        .O_gmii_tx_er             ( S_ocr_gmii_tx_er        ),
        .I_gmii_rxd               ( S_eth_gmii_rxd          ),
        .I_gmii_rx_dv             ( S_eth_gmii_rx_dv        ),
        .I_gmii_rx_er             ( S_eth_gmii_rx_er        ),
        .O_tx_busy_drop_count     (                         ),
        .O_tx_invalid_drop_count  (                         ),
        .O_rx_bad_frame_count     (                         ),
        .O_rx_busy_drop_count     (                         ),
        .O_rx_oversize_drop_count (                         ),
        .O_bad_frame_count        (                         ),
        .O_busy_drop_count        (                         )
    );

    assign S_stream_tx_ready_pix = S_stream_tx_available_pix && S_eth_gigabit_ready_app;

    generate
    if (P_CORE_BUSINESS == 0) begin : g_mode_link_full
        ctl_stream_link #(
            .P_LOCAL_MAC(48'h02_00_00_12_34_56),
            .P_PEER_MAC(48'h02_00_00_65_43_21),
            .P_ETHERTYPE(16'h88B6), .P_TX_ARB_EN(1)
        ) u_mode_stream_link (
            .I_clk_tx(S_eth_gmii_tx_clk), .I_rst_tx_n(W_eth_tx_common_reset_n),
            .O_gmii_txd(S_stream_gmii_txd), .O_gmii_tx_en(S_stream_gmii_tx_en),
            .O_gmii_tx_er(S_stream_gmii_tx_er),
            .I_tx_grant(S_stream_tx_grant), .O_tx_request(S_stream_tx_request),
            .O_tx_done_tx(S_stream_tx_done),
            .I_clk_rx(S_eth_gmii_rx_clk), .I_rst_rx_n(W_eth_rx_common_reset_n),
            .I_gmii_rxd(S_eth_gmii_rxd), .I_gmii_rx_dv(S_eth_gmii_rx_dv),
            .I_gmii_rx_er(S_eth_gmii_rx_er),
            .I_clk_app(S_hdmi_pixel_clk), .I_rst_app_n(S_eth_app_reset_n),
            .O_tx_ready(S_stream_tx_available_pix), .O_tx_wbuf(),
            .I_tx_we(S_stream_tx_write_pix), .I_tx_addr(S_stream_tx_address_pix),
            .I_tx_wdata(S_stream_tx_data_pix), .I_tx_commit(S_stream_tx_commit_pix),
            .O_rx_valid(S_stream_rx_valid_pix), .O_rx_rbuf(), .O_rx_len(S_stream_rx_length_pix),
            .I_rx_addr(S_stream_rx_address_pix), .O_rx_rdata(S_stream_rx_data_pix),
            .I_rx_release(S_stream_rx_release_pix), .O_tx_frame_count(),
            .O_rx_frame_count(), .O_rx_bad_count(), .O_rx_busy_drop_count(),
            .O_rx_last_line(), .O_bad_frame_count(), .O_busy_drop_count()
        );

    end else begin : g_mode_link_disabled
        assign S_stream_gmii_txd = 8'd0;
        assign S_stream_gmii_tx_en = 1'b0;
        assign S_stream_gmii_tx_er = 1'b0;
        assign S_stream_tx_request = 1'b0;
        assign S_stream_tx_done = 1'b1;
        assign S_stream_tx_available_pix = 1'b0;
        assign S_stream_rx_valid_pix = 1'b0;
        assign S_stream_rx_length_pix = 16'd0;
        assign S_stream_rx_data_pix = 8'd0;
    end
    endgenerate

    ctl_gmii_tx_scheduler3 u_network_scheduler (
        .I_clk_tx(S_eth_gmii_tx_clk), .I_rst_tx_n(W_eth_tx_common_reset_n),
        .I_tx_request({W_ink_tx_request_tx, S_stream_tx_request, S_ocr_tx_request}),
        .I_tx_done({W_ink_tx_done_tx, S_stream_tx_done, S_ocr_tx_done}),
        .O_tx_grant(W_ink_scheduler_grant_tx),
        .I_tx0_txd(S_ocr_gmii_txd), .I_tx1_txd(S_stream_gmii_txd),
        .I_tx2_txd(W_ink_gmii_txd_tx),
        .I_tx_en({W_ink_gmii_tx_en_tx, S_stream_gmii_tx_en, S_ocr_gmii_tx_en}),
        .I_tx_er({W_ink_gmii_tx_er_tx, S_stream_gmii_tx_er, S_ocr_gmii_tx_er}),
        .O_txd(S_eth_gmii_txd), .O_tx_en(S_eth_gmii_tx_en),
        .O_tx_er(S_eth_gmii_tx_er), .O_owner_valid(), .O_owner()
    );
    assign S_ocr_tx_grant = W_ink_scheduler_grant_tx[0];
    assign S_stream_tx_grant = W_ink_scheduler_grant_tx[1];
    assign W_ink_tx_grant_tx = W_ink_scheduler_grant_tx[2];

    // ===================== 车牌画框叠加（v1.7 新增） =====================
    // 在 hdmi_mixer 之后、hdmi_tx 之前叠加绿色外接框（整体延迟 1 拍，de/vsync 同步）。
    plate_box_overlay u_plate_box_overlay(
        .clk             ( S_hdmi_pixel_clk      ),
        .rst_n           ( S_hdmi_rst_n          ),
        .i_data     ( S_hdmi_out_data       ),
        .i_de       ( S_hdmi_out_de         ),
        .i_vsync    ( S_hdmi_out_vsync      ),
        .i_up       ( S_plate_up            ),
        .i_down     ( S_plate_down          ),
        .i_left     ( S_plate_left          ),
        .i_right    ( S_plate_right         ),
        .i_detected ( S_plate_detected      ),
        .o_data     ( S_hdmi_out_data_ovl   ),
        .o_de       ( S_hdmi_out_de_ovl     ),
        .o_vsync    ( S_hdmi_out_vsync_ovl  )
    );

    // ===================== 字符框叠加（v1.7 功能二） =====================
    // 在车牌框之后叠加 7 个红色字符框（整体延迟 1 拍）。
    char_box_overlay u_char_box_overlay(
        .clk             ( S_hdmi_pixel_clk      ),
        .rst_n           ( S_hdmi_rst_n          ),
        .i_data     ( S_hdmi_out_data_ovl   ),
        .i_de       ( S_hdmi_out_de_ovl     ),
        .i_vsync    ( S_hdmi_out_vsync_ovl  ),
        .i_char_up    ( S_char_up    ),
        .i_char_down  ( S_char_down  ),
        .i_c1_left    ( S_c1_left    ), .i_c1_right ( S_c1_right ),
        .i_c2_left    ( S_c2_left    ), .i_c2_right ( S_c2_right ),
        .i_c3_left    ( S_c3_left    ), .i_c3_right ( S_c3_right ),
        .i_c4_left    ( S_c4_left    ), .i_c4_right ( S_c4_right ),
        .i_c5_left    ( S_c5_left    ), .i_c5_right ( S_c5_right ),
        .i_c6_left    ( S_c6_left    ), .i_c6_right ( S_c6_right ),
        .i_c7_left    ( S_c7_left    ), .i_c7_right ( S_c7_right ),
        .i_detected   ( S_char_detected     ),
        .i_char_valid ( S_char_valid        ),
        .o_data     ( S_hdmi_out_data_ovl2  ),
        .o_de       ( S_hdmi_out_de_ovl2    ),
        .o_vsync    ( S_hdmi_out_vsync_ovl2 )
    );

    hdmi_tx u_hdmi_tx(
        .I_pixel_clk        ( S_hdmi_pixel_clk  ),
        .I_serial_clk       ( S_hdmi_serial_clk ),
        .I_rst              ( ~S_hdmi_rst_n     ),
        .I_key_in           ( 1'b0              ),
        .I_edid_read_trig   ( 1'b0              ),
        .O_edid_read_valid  (                   ),
        .O_edid_read_data   (                   ),
        .I_video_rgb_enable ( 1'b1              ),
        .I_video_in_vs      ( S_hdmi_out_vsync_ovl2 ),
        .I_video_in_de      ( S_hdmi_out_de_ovl2    ),
        .I_video_in_user    ( 1'b0              ),
        .I_video_in_valid   ( 1'b0              ),
        .I_video_in_last    ( 1'b0              ),
        .O_video_in_ready   (                   ),
        .I_video_in_data    ( S_hdmi_out_data_ovl2  ),
        .I_audio_valid      ( 1'b0              ),
        .I_audio_left_data  ( 24'd0             ),
        .I_audio_right_data ( 24'd0             ),
        .I_i2s_BCLK         ( 1'b0              ),
        .I_i2s_LRCK         ( 1'b0              ),
        .I_i2s_DOUT         ( 1'b0              ),
        .O_ddc_scl          (                   ),
        .O_hdmi_clk_p       (                   ),
        .O_hdmi_tx_p        (                   ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p      ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p      ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p      ),
        .O_tmds_clk_p       ( O_tmds_clk_p      )
    );


    
endmodule

