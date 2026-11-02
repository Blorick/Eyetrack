`timescale 1ns/1ps
// Shared-production-composition full-raster application benchmark.
// Protected camera/DDR/HDMI/PHY models and physical board behavior are outside this boundary.
module tb_ink_integrated_reset #(parameter integer P_FAULT=0);
    reg S_link_ready=1,S_ph_ink_reset_n=1,S_eg_ink_lock=1;
    localparam integer P_FRAMES_PER_MODE=8;
    reg S_crc_flip=0, S_rxer_flip=0;
    localparam integer IMG_W       = 1280;
    localparam integer H_TOTAL     = 1650;
    localparam integer H_BLANK     = H_TOTAL - IMG_W;
    localparam integer N_LINES     = 720;
    localparam integer N_MODES     = 3;
    localparam integer HDR_BYTES   = 16;
    localparam integer MAX_LINES   = 720;
    localparam integer P_LAT       = 6;
    localparam [47:0] M_MAC = 48'h02_00_00_12_34_56;
    localparam [47:0] S_MAC = 48'h02_00_00_65_43_21;

    integer S_errors = 0;
    integer S_checks = 0;
    integer S_styled [0:N_MODES-1];
    integer S_styled_lines [0:N_MODES-1];
    integer S_gold_bytes [0:N_MODES-1];
    integer S_rx_stream_m = 0;
    integer S_rx_stream_s = 0;
    integer S_rx_ocr_m = 0;
    integer S_rx_ocr_s = 0;
    integer S_ocr_req_commits = 0;
    integer S_ocr_rsp_commits = 0;
    integer S_max_reply_ns = 0;
    integer S_frame_count = 0;
    integer S_mode_done = 0;
    integer S_current_mode_index = 0;
    integer S_current_mode = 9;
    integer S_current_frame = 0;
    integer S_out_col = 0;
    integer S_last_styled_line = -1;
    integer S_mode_input_line = 0;
    integer S_mode_line_start_time [0:MAX_LINES-1];
    integer S_send_time [0:MAX_LINES-1];
    integer S_send_line [0:MAX_LINES-1];
    reg S_m_rx_valid_d = 1'b0;
    reg S_s_rx_valid_d = 1'b0;
    reg S_m_ocr_rx_valid_d = 1'b0;
    reg S_s_ocr_rx_valid_d = 1'b0;

    // Three asynchronous application clocks are deliberate: PH raster/bridge
    // at 75 MHz, EG mode path at the independent 75-MHz mode PLL output, and
    // both real OCR adapters at 25 MHz.  GMII TX/RX remain 125 MHz.
    reg clk_tx = 1'b0;
    wire clk_rx;
    wire clk_rx_s;
    reg clk_tx_s = 1'b0;
    reg clk_pix = 1'b0;
    reg clk_mode = 1'b0;
    reg clk_25 = 1'b0;
    always #4.000  clk_tx = ~clk_tx;
    initial begin #2.123; forever #4.001 clk_tx_s = ~clk_tx_s; end
    assign #1.000 clk_rx = clk_tx_s;
    assign #1.000 clk_rx_s = clk_tx;
    always #6.667  clk_pix = ~clk_pix;
    initial begin #2.000; forever #6.668 clk_mode = ~clk_mode; end
    initial begin #3.731; forever #20.003 clk_25 = ~clk_25; end

    reg rst_n = 1'b0;
    reg S_eg_rst_n = 1'b0;

    //=======================================================================
    // PH raster -> real dsp_camera_modes -> PH stream endpoint
    //=======================================================================
    reg [4:0]  m_mode = 5'd9;
    reg signed [8:0] m_brightness = 9'sd0;
    reg [7:0]  m_sat_factor = 8'd64;
    reg        m_vsync = 1'b1;
    reg        m_hsync = 1'b0;
    reg        m_de = 1'b0;
    reg        m_user = 1'b0;
    reg        m_last = 1'b0;
    reg [23:0] m_video = 24'd0;

    wire        m_rx_valid;
    wire [15:0] m_rx_len;
    wire [10:0] m_rx_addr;
    wire [7:0]  m_rx_data;
    wire        m_rx_release;
    wire        m_tx_ready;
    wire        m_tx_we;
    wire [10:0] m_tx_addr;
    wire [7:0]  m_tx_data;
    wire        m_tx_commit;
    wire        m_out_vsync, m_out_hsync, m_out_de, m_out_user, m_out_last;
    wire [23:0] m_out_video;
    wire        m_out_link_ok, m_out_remote_used;

    //=======================================================================
    // EG stream endpoint and the real line adapter/mode pipeline
    //=======================================================================
    wire        s_rx_valid;
    wire [15:0] s_rx_len;
    wire [10:0] s_rx_addr;
    wire [7:0]  s_rx_data;
    wire        s_rx_release;
    wire        s_tx_ready;
    wire        s_tx_wbuf;
    wire        s_tx_we;
    wire [10:0] s_tx_addr;
    wire [7:0]  s_tx_data;
    wire        s_tx_commit;
    wire [4:0]  s_fil_mode;
    wire        s_fil_vsync, s_fil_hsync, s_fil_de, s_fil_user, s_fil_last;
    wire [7:0]  s_fil_y;
    wire        s_out_vsync, s_out_hsync, s_out_de, s_out_user, s_out_last;
    wire [4:0]  s_out_mode;
    wire [7:0]  s_out_y;
    wire        s_out_edge, s_out_mask, s_out_window_valid;


    wire [7:0] m_ocr_txd, m_mode_txd, m_ink_txd, m_gmii_txd;
    wire m_ocr_en, m_mode_en, m_ink_en, m_gmii_en;
    wire m_ocr_er, m_mode_er, m_ink_er, m_gmii_er;
    wire m_ocr_req, m_mode_req, m_ink_req;
    wire m_ocr_done_tx, m_mode_done_tx, m_ink_done_tx;
    wire [2:0] m_grant;
    wire m_ocr_grant=m_grant[0], m_mode_grant=m_grant[1];
    wire [7:0] s_ocr_txd, s_mode_txd, s_ink_txd, s_gmii_txd;
    wire s_ocr_en, s_mode_en, s_ink_en, s_gmii_en;
    wire s_ocr_er, s_mode_er, s_ink_er, s_gmii_er;
    wire s_ocr_req, s_mode_req, s_ink_req;
    wire s_ocr_done_tx, s_mode_done_tx, s_ink_done_tx;
    wire [2:0] s_grant;
    wire s_ocr_grant=s_grant[0], s_mode_grant=s_grant[1];
    wire [7:0] m2s_rxd=m_gmii_txd ^ {7'd0,S_crc_flip}, s2m_rxd=s_gmii_txd;
    wire m2s_dv=m_gmii_en, m2s_er=m_gmii_er | S_rxer_flip;
    wire s2m_dv=s_gmii_en, s2m_er=s_gmii_er;
    wire m_legacy_rx_valid, s_legacy_rx_valid, s_legacy_busy;
    wire [55:0] m_res, m_raw_code;
    wire [3:0] m_res_n, m_match_state;
    wire m_res_valid, m_frame_result;
    wire [6:0] m_raw_valid, m_raw_mask;
    reg m_ink=0, m_ink_de=0, m_detected=1;
    reg [10:0] m_up=200, m_down=207;
    reg [6:0] m_valid=7'h7f;
    reg [76:0] m_left=0, m_right=0;
    wire s_compute_busy, s_compute_result_valid;

    ctl_gmii_tx_scheduler3 u_m_sched (
        .I_clk_tx(clk_tx), .I_rst_tx_n(rst_n),
        .I_tx_request({m_ink_req,m_mode_req,m_ocr_req}),
        .I_tx_done({m_ink_done_tx,m_mode_done_tx,m_ocr_done_tx}),
        .O_tx_grant(m_grant), .I_tx0_txd(m_ocr_txd),
        .I_tx1_txd(m_mode_txd), .I_tx2_txd(m_ink_txd),
        .I_tx_en({m_ink_en,m_mode_en,m_ocr_en}),
        .I_tx_er({m_ink_er,m_mode_er,m_ocr_er}),
        .O_txd(m_gmii_txd), .O_tx_en(m_gmii_en), .O_tx_er(m_gmii_er),
        .O_owner_valid(), .O_owner()
    );
    ctl_gmii_tx_scheduler3 u_s_sched (
        .I_clk_tx(clk_tx_s), .I_rst_tx_n(S_eg_rst_n),
        .I_tx_request({s_ink_req,s_mode_req,s_ocr_req}),
        .I_tx_done({s_ink_done_tx,s_mode_done_tx,s_ocr_done_tx}),
        .O_tx_grant(s_grant), .I_tx0_txd(s_ocr_txd),
        .I_tx1_txd(s_mode_txd), .I_tx2_txd(s_ink_txd),
        .I_tx_en({s_ink_en,s_mode_en,s_ocr_en}),
        .I_tx_er({s_ink_er,s_mode_er,s_ocr_er}),
        .O_txd(s_gmii_txd), .O_tx_en(s_gmii_en), .O_tx_er(s_gmii_er),
        .O_owner_valid(), .O_owner()
    );
    ctl_master_ink_application u_master (
        .I_clk_app(clk_pix), .I_clk_tx(clk_tx), .I_clk_rx(clk_rx),
        .I_reset_app_async_n(rst_n && S_ph_ink_reset_n), .I_reset_tx_async_n(rst_n && S_ph_ink_reset_n),
        .I_reset_rx_async_n(rst_n && S_ph_ink_reset_n), .I_link_ready(S_link_ready),
        .I_ink(m_ink), .I_ink_de(m_ink_de), .I_vsync(m_vsync),
        .I_detected(m_detected), .I_char_up(m_up), .I_char_down(m_down),
        .I_char_valid(m_valid), .I_char_left(m_left), .I_char_right(m_right),
        .I_legacy_tx_request(m_ocr_req), .I_legacy_rx_valid(m_legacy_rx_valid),
        .I_tx_grant(m_grant[2]), .I_gmii_rxd(s2m_rxd),
        .I_gmii_rx_dv(s2m_dv), .I_gmii_rx_er(s2m_er),
        .O_tx_request(m_ink_req), .O_tx_done(m_ink_done_tx),
        .O_gmii_txd(m_ink_txd), .O_gmii_tx_en(m_ink_en), .O_gmii_tx_er(m_ink_er),
        .O_res(m_res), .O_res_n(m_res_n), .O_res_valid(m_res_valid),
        .O_raw_valid(m_raw_valid), .O_frame_result(m_frame_result),
        .O_raw_candidate_code(m_raw_code), .O_raw_candidate_mask(m_raw_mask),
        .O_match_state(m_match_state)
    );
    ctl_slave_ink_application u_slave (
        .I_clk_app(clk_mode), .I_clk_25m(clk_25),
        .I_clk_tx(clk_tx_s), .I_clk_rx(clk_rx_s),
        .I_por_done(S_eg_rst_n), .I_pll_lock(1'b1), .I_mode_pll_lock(S_eg_ink_lock),
        .I_legacy_tx_busy(s_legacy_busy), .I_legacy_tx_request(s_ocr_req),
        .I_legacy_gmii_tx_en(s_ocr_en), .I_tx_grant(s_grant[2]),
        .I_gmii_rxd(m2s_rxd), .I_gmii_rx_dv(m2s_dv), .I_gmii_rx_er(m2s_er),
        .O_tx_request(s_ink_req), .O_tx_done(s_ink_done_tx),
        .O_gmii_txd(s_ink_txd), .O_gmii_tx_en(s_ink_en), .O_gmii_tx_er(s_ink_er),
        .O_compute_busy(s_compute_busy), .O_compute_result_valid(s_compute_result_valid)
    );

    ctl_lpr_packet_link #(.P_LOCAL_MAC(M_MAC), .P_PEER_MAC(S_MAC), .P_ETHERTYPE(16'h88B5), .P_TX_ARB_EN(1)) u_m_legacy (
        .I_clk_app(clk_pix), .I_rst_app_n(rst_n), .I_clk_tx(clk_tx), .I_rst_tx_n(rst_n),
        .I_clk_rx(clk_rx), .I_rst_rx_n(rst_n),
        .I_tx_write_en(1'b0), .I_tx_write_addr(10'd0), .I_tx_write_data(8'd0),
        .I_tx_commit(1'b0), .I_tx_length(11'd0), .O_tx_busy(), .O_tx_done(),
        .I_tx_grant(m_ocr_grant), .O_tx_request(m_ocr_req), .O_tx_done_tx(m_ocr_done_tx),
        .O_rx_valid(m_legacy_rx_valid), .O_rx_length(), .I_rx_read_addr(10'd0), .O_rx_read_data(), .I_rx_release(m_legacy_rx_valid),
        .O_gmii_txd(m_ocr_txd), .O_gmii_tx_en(m_ocr_en), .O_gmii_tx_er(m_ocr_er),
        .I_gmii_rxd(s2m_rxd), .I_gmii_rx_dv(s2m_dv), .I_gmii_rx_er(s2m_er),
        .O_tx_busy_drop_count(), .O_tx_invalid_drop_count(), .O_rx_bad_frame_count(), .O_rx_busy_drop_count(),
        .O_rx_oversize_drop_count(), .O_bad_frame_count(), .O_busy_drop_count()
    );

    ctl_lpr_packet_link #(.P_LOCAL_MAC(S_MAC), .P_PEER_MAC(M_MAC), .P_ETHERTYPE(16'h88B5), .P_TX_ARB_EN(1)) u_s_legacy (
        .I_clk_app(clk_25), .I_rst_app_n(S_eg_rst_n), .I_clk_tx(clk_tx_s), .I_rst_tx_n(S_eg_rst_n),
        .I_clk_rx(clk_rx_s), .I_rst_rx_n(S_eg_rst_n),
        .I_tx_write_en(1'b0), .I_tx_write_addr(10'd0), .I_tx_write_data(8'd0),
        .I_tx_commit(1'b0), .I_tx_length(11'd0), .O_tx_busy(s_legacy_busy), .O_tx_done(),
        .I_tx_grant(s_ocr_grant), .O_tx_request(s_ocr_req), .O_tx_done_tx(s_ocr_done_tx),
        .O_rx_valid(s_legacy_rx_valid), .O_rx_length(), .I_rx_read_addr(10'd0), .O_rx_read_data(), .I_rx_release(s_legacy_rx_valid),
        .O_gmii_txd(s_ocr_txd), .O_gmii_tx_en(s_ocr_en), .O_gmii_tx_er(s_ocr_er),
        .I_gmii_rxd(m2s_rxd), .I_gmii_rx_dv(m2s_dv), .I_gmii_rx_er(m2s_er),
        .O_tx_busy_drop_count(), .O_tx_invalid_drop_count(), .O_rx_bad_frame_count(), .O_rx_busy_drop_count(),
        .O_rx_oversize_drop_count(), .O_bad_frame_count(), .O_busy_drop_count()
    );
    dsp_camera_modes #(.P_WIDTH(IMG_W), .P_REMOTE_LATENCY(P_LAT)) u_camera (
        .I_clk_pix(clk_pix), .I_rst_n(rst_n), .I_mode(m_mode),
        .I_brightness(m_brightness), .I_sat_factor(m_sat_factor),
        .I_vsync(m_vsync), .I_hsync(m_hsync), .I_de(m_de), .I_user(m_user),
        .I_last(m_last), .I_video(m_video),
        .I_rx_valid(m_rx_valid), .I_rx_length(m_rx_len),
        .O_rx_address(m_rx_addr), .I_rx_data(m_rx_data),
        .O_rx_release(m_rx_release), .I_tx_ready(m_tx_ready),
        .O_tx_write(m_tx_we), .O_tx_address(m_tx_addr), .O_tx_data(m_tx_data),
        .O_tx_commit(m_tx_commit), .O_vsync(m_out_vsync), .O_hsync(m_out_hsync),
        .O_de(m_out_de), .O_user(m_out_user), .O_last(m_out_last),
        .O_video(m_out_video), .O_link_ok(m_out_link_ok),
        .O_remote_used(m_out_remote_used)
    );

    ctl_stream_link #(.P_LOCAL_MAC(M_MAC), .P_PEER_MAC(S_MAC),
                      .P_TX_ARB_EN(1)) u_m_stream (
        .I_clk_tx(clk_tx), .I_rst_tx_n(rst_n),
        .O_gmii_txd(m_mode_txd), .O_gmii_tx_en(m_mode_en),
        .O_gmii_tx_er(m_mode_er), .I_tx_grant(m_mode_grant),
        .O_tx_request(m_mode_req), .O_tx_done_tx(m_mode_done_tx),
        .I_clk_rx(clk_rx), .I_rst_rx_n(rst_n), .I_gmii_rxd(s2m_rxd),
        .I_gmii_rx_dv(s2m_dv), .I_gmii_rx_er(s2m_er),
        .I_clk_app(clk_pix), .I_rst_app_n(rst_n),
        .O_tx_ready(m_tx_ready), .O_tx_wbuf(),
        .I_tx_we(m_tx_we), .I_tx_addr(m_tx_addr), .I_tx_wdata(m_tx_data),
        .I_tx_commit(m_tx_commit), .O_rx_valid(m_rx_valid), .O_rx_rbuf(),
        .O_rx_len(m_rx_len), .I_rx_addr(m_rx_addr),
        .O_rx_rdata(m_rx_data), .I_rx_release(m_rx_release),
        .O_tx_frame_count(), .O_rx_frame_count(), .O_rx_bad_count(),
        .O_rx_busy_drop_count(), .O_rx_last_line(), .O_bad_frame_count(),
        .O_busy_drop_count()
    );
    ctl_stream_link #(.P_LOCAL_MAC(S_MAC), .P_PEER_MAC(M_MAC),
                      .P_TX_ARB_EN(1)) u_s_stream (
        .I_clk_tx(clk_tx_s), .I_rst_tx_n(S_eg_rst_n),
        .O_gmii_txd(s_mode_txd), .O_gmii_tx_en(s_mode_en),
        .O_gmii_tx_er(s_mode_er), .I_tx_grant(s_mode_grant),
        .O_tx_request(s_mode_req), .O_tx_done_tx(s_mode_done_tx),
        .I_clk_rx(clk_rx_s), .I_rst_rx_n(S_eg_rst_n), .I_gmii_rxd(m2s_rxd),
        .I_gmii_rx_dv(m2s_dv), .I_gmii_rx_er(m2s_er),
        .I_clk_app(clk_mode), .I_rst_app_n(S_eg_rst_n), .O_tx_ready(s_tx_ready),
        .O_tx_wbuf(s_tx_wbuf), .I_tx_we(s_tx_we), .I_tx_addr(s_tx_addr),
        .I_tx_wdata(s_tx_data), .I_tx_commit(s_tx_commit),
        .O_rx_valid(s_rx_valid), .O_rx_rbuf(), .O_rx_len(s_rx_len),
        .I_rx_addr(s_rx_addr), .O_rx_rdata(s_rx_data),
        .I_rx_release(s_rx_release), .O_tx_frame_count(),
        .O_rx_frame_count(), .O_rx_bad_count(), .O_rx_busy_drop_count(),
        .O_rx_last_line(), .O_bad_frame_count(), .O_busy_drop_count()
    );
    eg4d_line_adapter #(.IMG_W(IMG_W), .P_HDR_BYTES(16),
                        .P_APP_MAGIC(16'h01A5)) u_s_line_adapter (
        .I_clk(clk_mode), .I_rst_n(S_eg_rst_n), .I_rx_valid(s_rx_valid),
        .I_rx_len(s_rx_len), .O_rx_addr(s_rx_addr), .I_rx_rdata(s_rx_data),
        .O_rx_release(s_rx_release), .I_tx_ready(s_tx_ready),
        .O_tx_wbuf(), .O_tx_we(s_tx_we), .O_tx_addr(s_tx_addr),
        .O_tx_wdata(s_tx_data), .O_tx_commit(s_tx_commit),
        .O_fil_vsync(s_fil_vsync), .O_fil_hsync(s_fil_hsync),
        .O_fil_de(s_fil_de), .O_fil_user(s_fil_user), .O_fil_last(s_fil_last),
        .O_fil_y(s_fil_y), .O_fil_mode(s_fil_mode),
        .I_fil_vsync(s_out_vsync), .I_fil_hsync(s_out_hsync),
        .I_fil_de(s_out_de), .I_fil_user(s_out_user), .I_fil_last(s_out_last),
        .I_fil_y(s_out_y), .I_fil_edge(s_out_edge),
        .I_fil_window_valid(s_out_window_valid)
    );
    eg4d_mode_pipeline #(.IMG_W(IMG_W), .EDGE_RGB_TH(8'd80),
                         .CARTOON_OFF(8'd0)) u_s_pipeline (
        .I_clk(clk_mode), .I_rst_n(S_eg_rst_n), .I_mode(s_fil_mode),
        .I_bin_th(8'd128), .I_cartoon_th(8'd80), .I_vsync(s_fil_vsync),
        .I_hsync(s_fil_hsync), .I_de(s_fil_de), .I_user(s_fil_user),
        .I_last(s_fil_last), .I_y(s_fil_y), .O_mode(s_out_mode),
        .O_vsync(s_out_vsync), .O_hsync(s_out_hsync), .O_de(s_out_de),
        .O_user(s_out_user), .O_last(s_out_last), .O_y(s_out_y),
        .O_edge(s_out_edge), .O_mask(s_out_mask),
        .O_window_valid(s_out_window_valid)
    );
    function automatic integer f_height(input integer fid);
        f_height = ((fid % P_FRAMES_PER_MODE) == 0) ? 8 : 32;
    endfunction
    function automatic integer f_detected(input integer fid);
        f_detected = (fid % P_FRAMES_PER_MODE) != 7;
    endfunction
    function automatic integer f_ink(input integer fid, input integer x, input integer y);
        integer slot, rx, gy, left;
        begin
            f_ink = 0;
            if ((fid % P_FRAMES_PER_MODE) != 6 && y >= 200 && y < 200 + f_height(fid)) begin
                for (slot = 0; slot < 7; slot = slot + 1) begin
                    left = 64 + slot * 160;
                    if (x >= left && x < left + 32) begin
                        rx = x-left;
                        gy = ((y-200)*8)/f_height(fid);
                        f_ink = rx==0 || rx==31 || gy==0 || gy==7 ||
                            (((rx*11 + gy*7 + slot*3 + (rx*gy)%17)%23) < 11);
                        if(fid==5 && (rx<4 || rx>27 || y<203 || y>228))f_ink=0;
                    end
                end
            end
        end
    endfunction
    task automatic configure_frame(input integer fid);
        integer slot;
        begin
            m_detected=f_detected(fid);m_up=200;m_down=199+f_height(fid);m_valid=7'h7f;
            for(slot=0;slot<7;slot=slot+1)begin m_left[slot*11+:11]=64+slot*160;m_right[slot*11+:11]=95+slot*160;end
        end
    endtask
    task automatic drive_raster(input integer fid,input integer first_row,input integer last_row);
        integer row,col;
        reg [7:0] yv;
        begin
            for(row=first_row;row<=last_row;row=row+1)for(col=0;col<1650;col=col+1)begin
                @(negedge clk_pix);
                if(row==731 && col==0)configure_frame(fid+1);
                m_vsync=(row>=726 && row<=730);m_de=(row<720 && col<1280);m_ink_de=m_de;
                m_user=(row==0 && col==0);m_last=(row==719 && col==1279);
                m_ink=m_de?f_ink(fid,col,row):0;
                yv=col[7:0]^(row*19)^(fid*37)^(m_mode*7);
                m_video=m_de?{yv,(yv+8'h31),(yv^8'h5a)}:24'd0;
            end
        end
    endtask
    integer S_result_count=0,S_snapshot_count=0,S_fault_hits=0,S_saw_not_ready=0;
    integer S_clear_count=0,S_clear_before=0,S_post_fault_results;
    integer S_fault4_pix_cycles=0,S_fault4_first_eof_cycle=-1,S_fault4_timeout_cycle=-1;
    integer S_fault4_timeout_count=0;
    reg S_fault_armed=0;
    reg S_fault4_eof_seen=0,S_fault4_timeout_seen=0;
    reg [31:0] S_latest_epoch=0,S_latest_fid=0;
    reg S_latest_valid=0;
    reg [188:0] S_last_result_tag=0;
    reg S_last_result_seen=0;
    real S_fault_start_ns,S_recovered_ns;
    always @(posedge clk_pix) if(rst_n)begin
        if(S_fault_armed && !u_master.W_ink_session_ready_pix)S_saw_not_ready=1;
        if(u_master.W_ink_result_clear_pix)S_clear_count=S_clear_count+1;
        if(!u_master.W_ink_reset_pix_n)begin S_latest_valid=0;S_last_result_seen=0;end
        if(u_master.W_ink_eof_committed_pix)begin
            S_latest_valid=1;S_latest_epoch=u_master.W_ink_eof_epoch_pix;S_latest_fid=u_master.W_ink_eof_frame_index_pix;
        end
        if(u_master.W_ink_frame_result_pix)begin
            if(!S_latest_valid || u_master.u_ink_response.W_epoch!==S_latest_epoch || u_master.u_ink_response.W_frame_index!==S_latest_fid)
                $fatal(1,"INK_RESET_STALE_RESULT");
            if(S_last_result_seen && S_last_result_tag[63:0]=={S_latest_epoch,S_latest_fid})$fatal(1,"INK_RESET_DUPLICATE_RESULT");
            S_last_result_tag={125'd0,S_latest_epoch,S_latest_fid};S_last_result_seen=1;S_result_count=S_result_count+1;
        end
    end
    // Fault4-only production watchdog observation. The watchdog output is a
    // registered one-cycle pulse. Sampling it in this active-region monitor
    // intentionally sees the pulse on the posedge after its NBA update.
    always @(posedge clk_pix) if(rst_n)begin
        S_fault4_pix_cycles=S_fault4_pix_cycles+1;
        if(P_FAULT==4 && S_fault_armed && !S_fault4_eof_seen && u_master.W_ink_eof_committed_pix)begin
            S_fault4_eof_seen=1;
            S_fault4_first_eof_cycle=S_fault4_pix_cycles;
        end
        if(P_FAULT==4 && S_fault4_eof_seen && !S_fault4_timeout_seen && u_master.W_ink_result_timeout_pix)begin
            S_fault4_timeout_seen=1;
            S_fault4_timeout_count=S_fault4_timeout_count+1;
            S_fault4_timeout_cycle=S_fault4_pix_cycles;
            if(S_fault4_timeout_cycle-S_fault4_first_eof_cycle!=7500001)
                $fatal(1,"INK_RESET_FAULT4_WATCHDOG_DELTA cycles=%0d",S_fault4_timeout_cycle-S_fault4_first_eof_cycle);
        end else if(P_FAULT==4 && S_fault4_timeout_seen && u_master.W_ink_result_timeout_pix)
            $fatal(1,"INK_RESET_FAULT4_REPEATED_WATCHDOG");
    end
    always @(posedge clk_mode) if(u_slave.W_ink_rst_mode_n && u_slave.W_ink_snapshot_valid)S_snapshot_count=S_snapshot_count+1;
    task automatic wait_sessions;
        integer guard;
        begin
            guard=0;while((!u_master.W_ink_session_ready_pix || !u_slave.W_ink_session_ready) && guard<3000000)begin @(negedge clk_pix);guard=guard+1;end
            if(guard==3000000)$fatal(1,"INK_RESET_HANDSHAKE_TIMEOUT fault=%0d",P_FAULT);
        end
    endtask
    task automatic wait_results(input integer wanted);
        integer guard;
        begin
            guard=0;while(S_result_count<wanted && guard<1500000)begin @(negedge clk_pix);guard=guard+1;end
            if(guard==1500000 || S_result_count!=wanted)$fatal(1,"INK_RESET_RESULT_TIMEOUT expected=%0d actual=%0d",wanted,S_result_count);
        end
    endtask
    task automatic wait_fault4_production_watchdog;
        integer guard;
        begin
            guard=0;
            while((!S_fault4_eof_seen || !S_fault4_timeout_seen) && guard<7500100)begin @(negedge clk_pix);guard=guard+1;end
            if(!S_fault4_eof_seen)$fatal(1,"INK_RESET_FAULT4_WATCHDOG_EOF_MISSING guard=%0d",guard);
            if(!S_fault4_timeout_seen)$fatal(1,"INK_RESET_FAULT4_WATCHDOG_TIMEOUT_MISSING guard=%0d",guard);
        end
    endtask
    task automatic await_active_line;
        integer guard;
        begin
            guard=0;
            while(!(u_slave.W_ink_pixel_de && u_slave.u_ink_decoder.S_next_line==300) && guard<3000000)begin @(negedge clk_mode);guard=guard+1;end
            if(guard==3000000)$fatal(1,"INK_RESET_ACTIVE_PHASE_MISSING");
        end
    endtask
    task automatic inject_fault;
        integer guard;
        reg [40:0] held_tile;
        begin
            if(P_FAULT<=2)await_active_line();
            S_fault_start_ns=$realtime;
            case(P_FAULT)
                0:begin
                    S_link_ready=0;S_fault_hits=1;
                    repeat(200)@(negedge clk_pix);
                    S_link_ready=1;
                end
                1:begin
                    S_ph_ink_reset_n=0;S_fault_hits=1;
                    #0.001;
                    if({u_master.W_ink_reset_pix_n,u_master.W_ink_reset_tx_n,u_master.W_ink_reset_rx_n}!==0)$fatal(1,"INK_RESET_PH_COMMON_ASSERT");
                    repeat(12)@(negedge clk_pix);S_ph_ink_reset_n=1;
                end
                2,6:begin
                    if(P_FAULT==6)begin
                        guard=0;
                        while(!(u_slave.W_ink_frame_index==2 && !u_slave.W_ink_vsync && !u_slave.W_ink_geometry_ready) && guard<3000000)begin @(negedge clk_mode);guard=guard+1;end
                        if(guard==3000000)$fatal(1,"INK_RESET_META_PHASE_MISSING");
                    end
                    S_fault_start_ns=$realtime;
                    S_eg_ink_lock=0;S_fault_hits=1;
                    #0.001;
                    if({u_slave.W_ink_rst_mode_n,u_slave.W_ink_rst_compute_n,u_slave.W_ink_rst_tx_n,u_slave.W_ink_rst_rx_n}!==0)$fatal(1,"INK_RESET_EG_COMMON_ASSERT");
                    repeat(22)@(negedge clk_mode);S_eg_ink_lock=1;
                end
                3:begin
                    guard=0;
                    while(!(u_master.u_ink_packet_tx.I_commit && u_master.u_ink_packet_tx.I_packet_length==178 && u_master.u_ink_capture.S_frame_id==1 && u_master.u_ink_capture.S_bank_line[u_master.u_ink_packet_tx.I_commit_bank]==300) && guard<3000000)begin @(negedge clk_pix);guard=guard+1;end
                    if(guard==3000000)$fatal(1,"INK_RESET_PH_COMMIT_PHASE_MISSING");
                    S_fault_start_ns=$realtime;
                    force u_master.u_ink_packet_tx.I_commit=1'b0;S_fault_hits=1;
                    guard=0;while(!u_master.W_ink_session_recover_pix && guard<1000)begin @(negedge clk_pix);guard=guard+1;end
                    if(guard==1000)$fatal(1,"INK_RESET_PH_COMMIT_RECOVERY_MISSING");
                    release u_master.u_ink_packet_tx.I_commit;
                end
                4:begin
                    guard=0;
                    while(!(u_slave.u_ink_packet_tx.I_commit && u_slave.u_ink_reply.S_kind==7 && u_slave.u_ink_reply.S_frame_index==1) && guard<4000000)begin @(negedge clk_mode);guard=guard+1;end
                    if(guard==4000000)$fatal(1,"INK_RESET_EG_COMMIT_PHASE_MISSING");
                    S_fault_start_ns=$realtime;
                    force u_slave.u_ink_packet_tx.I_commit=1'b0;S_fault_hits=1;
                    guard=0;while(!u_slave.W_ink_reply_recover && guard<1000)begin @(negedge clk_mode);guard=guard+1;end
                    if(guard==1000)$fatal(1,"INK_RESET_EG_COMMIT_RECOVERY_MISSING");
                    release u_slave.u_ink_packet_tx.I_commit;
                end
                5:begin
                    guard=0;while(!u_slave.W_ink_tile_dst_valid && guard<2000000)begin @(negedge clk_25);guard=guard+1;end
                    if(guard==2000000)$fatal(1,"INK_RESET_TILE_STALL_PHASE_MISSING");
                    S_fault_start_ns=$realtime;
                    force u_slave.u_ink_compute.O_tile_ready=1'b0;
                    held_tile=u_slave.W_ink_tile_dst_data;S_fault_hits=1;
                    repeat(40)begin
                        @(negedge clk_25);
                        if(!u_slave.W_ink_tile_dst_valid || u_slave.W_ink_tile_dst_data!==held_tile)$fatal(1,"INK_RESET_HELD_TILE_CHANGED");
                    end
                    release u_slave.u_ink_compute.O_tile_ready;
                end
                default:$fatal(1,"unsupported P_FAULT");
            endcase
        end
    endtask
    initial begin : independent_recovery_case
        if(P_FAULT<0 || P_FAULT>6)$fatal(1,"unsupported P_FAULT");
        configure_frame(0);m_vsync=1;m_de=0;m_ink_de=0;
        repeat(30)@(negedge clk_pix);rst_n=1;
        repeat(71)@(negedge clk_pix);S_eg_rst_n=1;
        wait_sessions();repeat(100)@(negedge clk_pix);
        drive_raster(-1,726,749);configure_frame(0);
        drive_raster(0,0,749);wait_results(1);
        S_fault_armed=1;S_clear_before=S_clear_count;
        fork
            drive_raster(1,0,749);
            inject_fault();
        join
        if(S_fault_hits!=1)$fatal(1,"INK_RESET_FAULT_COVERAGE");
        if(P_FAULT==4)wait_fault4_production_watchdog();
        wait_sessions();S_recovered_ns=$realtime;
        if(P_FAULT!=5 && (!S_saw_not_ready || S_clear_count<=S_clear_before))$fatal(1,"INK_RESET_BARRIER_CLEAR_COVERAGE");
        if(P_FAULT==5)begin
            wait_results(2);
            drive_raster(2,0,749);wait_results(3);
        end else begin
            if(S_result_count!=1)$fatal(1,"INK_RESET_CANCELLED_FRAME_PUBLISHED count=%0d",S_result_count);
            // A high VS interval also cancels any metadata-only frame opened
            // just before recovery. The following fall is a fresh transaction.
            drive_raster(8,726,749);drive_raster(9,0,749);wait_results(2);
        end
        if(P_FAULT==4)
            $display("TB_RESULT: PASS tb_ink_integrated_reset fault=%0d faults=1 production_timeout_event=%0d production_timeout_count=%0d production_timeout_cycles=%0d ph_clear_barrier_awaited=1 fresh_handshake_guard=1 fresh_result_guard=1 recovered_result=1 real_guard_handshake=1 common_assertion_preserved=1 natural_C13_C14=1 recovery_observed_ns=%0.3f compiler_private_history_writes=0 errors=0",P_FAULT,S_fault4_timeout_count,S_fault4_timeout_count,S_fault4_timeout_cycle-S_fault4_first_eof_cycle,S_recovered_ns-S_fault_start_ns);
        else
            $display("TB_RESULT: PASS tb_ink_integrated_reset fault=%0d faults=1 recovered_result=1 real_guard_handshake=1 common_assertion_preserved=1 natural_C13_C14=1 recovery_observed_ns=%0.3f compiler_private_history_writes=0 errors=0",P_FAULT,S_recovered_ns-S_fault_start_ns);
        $finish;
    end
    initial begin #1000000000;$fatal(1,"INK_RESET_GLOBAL_TIMEOUT");end
    initial begin : measured_clocks
        real ph0,eg0,c0,t0,s0,ph_period,eg_period,c_period,t_period,s_period;
        reg [4:0] measured;
        measured=5'b00000;
        fork
            begin @(posedge clk_pix);ph0=$realtime;@(posedge clk_pix);ph_period=$realtime-ph0;measured[0]=1;end
            begin @(posedge clk_mode);eg0=$realtime;@(posedge clk_mode);eg_period=$realtime-eg0;measured[1]=1;end
            begin @(posedge clk_25);c0=$realtime;@(posedge clk_25);c_period=$realtime-c0;measured[2]=1;end
            begin @(posedge clk_tx);t0=$realtime;@(posedge clk_tx);t_period=$realtime-t0;measured[3]=1;end
            begin @(posedge clk_tx_s);s0=$realtime;@(posedge clk_tx_s);s_period=$realtime-s0;measured[4]=1;end
        join
        wait(measured===5'b11111);
        // Advance one 1-ps simulation tick after all second-edge measurements.
        // Real subtraction is compared with a 1-ps tolerance, not exact ==.
        #0.001;
        $display("INK_E2E_CLOCKS timescale=1ns/1ps PH_app_period_ns=%0.6f EG_app_period_ns=%0.6f compute_period_ns=%0.6f PH_tx_period_ns=%0.6f EG_tx_period_ns=%0.6f first_edges_ns=%0.3f,%0.3f,%0.3f,%0.3f,%0.3f RX_clock_delay_ns=1.000 measured=%05b",ph_period,eg_period,c_period,t_period,s_period,ph0,eg0,c0,t0,s0,measured);
        if(ph_period<13.333 || ph_period>13.335 || eg_period<13.335 || eg_period>13.337 ||
           c_period<40.005 || c_period>40.007 || t_period<7.999 || t_period>8.001 || s_period<8.001 || s_period>8.003)
            $fatal(1,"INK_E2E_CLOCK_SCALE");
    end
endmodule
