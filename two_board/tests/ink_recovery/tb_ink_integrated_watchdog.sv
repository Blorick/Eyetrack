`timescale 1ns/1ps
// Shared-production-composition full-raster application benchmark.
// Protected camera/DDR/HDMI/PHY models and physical board behavior are outside this boundary.
module tb_ink_integrated_watchdog;
    localparam integer P_FRAMES_PER_MODE=8;
    reg S_crc_flip=0, S_rxer_flip=0;
    reg S_drop_reply=0;
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
    wire s2m_dv=s_gmii_en && !S_drop_reply, s2m_er=s_gmii_er;
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
        .I_reset_app_async_n(rst_n), .I_reset_tx_async_n(rst_n),
        .I_reset_rx_async_n(rst_n), .I_link_ready(1'b1),
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
        .I_por_done(S_eg_rst_n), .I_pll_lock(1'b1), .I_mode_pll_lock(1'b1),
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
                        if(fid==-999 && (rx<4 || rx>27 || y<203 || y>228))f_ink=0;
                    end
                end
            end
        end
    endfunction
    task automatic configure_frame(input integer fid);
        integer slot;
        begin
            m_detected=1;m_up=200;m_down=199+f_height(fid);m_valid=7'h7f;
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
    reg [7:0] S_reply_mirror[0:1][0:255];
    reg [7:0] S_reply_kind[0:1];
    reg S_reply_en_q=0;
    reg S_block_results=0;
    integer S_reply_bank, S_dropped_results=0;
    integer S_cycles=0, S_first_lost_eof_cycle=-1, S_timeout_cycle=-1;
    integer S_later_eofs=0, S_clears=0, S_clear_before=0, S_new_matcher_resets=0;
    integer S_result_counts[0:31], S_snapshot_counts[0:31];
    reg [31:0] S_latest_epoch=0, S_latest_fid=0, S_old_epoch=0;
    reg S_latest_valid=0;
    reg S_age_check;
    reg [23:0] S_age_before;
    real S_first_lost_eof_ns, S_timeout_ns;
    always @(posedge clk_mode) if(u_slave.W_ink_rst_mode_n)begin
        if(u_slave.u_ink_packet_tx.I_write_en)
            S_reply_mirror[u_slave.u_ink_packet_tx.I_write_bank][u_slave.u_ink_packet_tx.I_write_addr]=u_slave.u_ink_packet_tx.I_write_data;
        if(u_slave.u_ink_packet_tx.I_commit && u_slave.u_ink_packet_tx.W_commit_ok_app)
            S_reply_kind[u_slave.u_ink_packet_tx.I_commit_bank]=S_reply_mirror[u_slave.u_ink_packet_tx.I_commit_bank][3];
        if(u_slave.W_ink_snapshot_valid)S_snapshot_counts[u_slave.W_ink_snapshot_frame]=S_snapshot_counts[u_slave.W_ink_snapshot_frame]+1;
    end
    always @(posedge clk_tx_s)begin
        #0.001;
        if(s_ink_en && !S_reply_en_q)begin
            S_reply_bank=u_slave.u_ink_packet_tx.S_read_bin_tx[0];
            S_drop_reply=S_block_results && S_reply_kind[S_reply_bank]==7;
            if(S_drop_reply)S_dropped_results=S_dropped_results+1;
        end
        if(!s_ink_en)S_drop_reply=0;
        S_reply_en_q=s_ink_en;
    end
    always @(posedge clk_pix) if(rst_n)begin
        S_cycles=S_cycles+1;
        S_age_check=0;
        if(u_master.W_ink_eof_committed_pix)begin
            S_latest_valid=1;S_latest_epoch=u_master.W_ink_eof_epoch_pix;S_latest_fid=u_master.W_ink_eof_frame_index_pix;
            if(S_block_results)begin
                if(S_first_lost_eof_cycle<0)begin S_first_lost_eof_cycle=S_cycles;S_first_lost_eof_ns=$realtime;end
                else begin
                    S_later_eofs=S_later_eofs+1;
                    if(u_master.u_ink_watchdog.S_result_pending_app && !u_master.u_ink_watchdog.W_deadline_app)begin
                        S_age_check=1;S_age_before=u_master.u_ink_watchdog.S_result_age_app;
                    end
                end
            end
        end
        if(u_master.W_ink_frame_result_pix)begin
            if(S_block_results)$fatal(1,"INK_WATCHDOG_SUPPRESSED_RESULT_ESCAPED");
            if(!S_latest_valid || u_master.u_ink_response.W_epoch!==S_latest_epoch || u_master.u_ink_response.W_frame_index!==S_latest_fid || S_result_counts[S_latest_fid]!=0)
                $fatal(1,"INK_WATCHDOG_RESULT_OWNERSHIP");
            S_result_counts[S_latest_fid]=S_result_counts[S_latest_fid]+1;
        end
        if(u_master.W_ink_result_clear_pix)begin
            S_clears=S_clears+1;
            if(!m_frame_result || {m_res,m_res_n,m_res_valid,m_raw_code,m_raw_valid}!==0)
                $fatal(1,"INK_WATCHDOG_CLEAR_NOT_EXPORTED");
        end
        if(u_master.W_ink_result_timeout_pix)begin
            if(S_timeout_cycle>=0)$fatal(1,"INK_WATCHDOG_REPEATED_TIMEOUT");
            S_timeout_cycle=S_cycles;S_timeout_ns=$realtime;
            if(S_timeout_cycle-S_first_lost_eof_cycle<7500000 || S_timeout_cycle-S_first_lost_eof_cycle>7500002)
                $fatal(1,"INK_WATCHDOG_DEADLINE_EXTENDED cycles=%0d",S_timeout_cycle-S_first_lost_eof_cycle);
        end
        #0.001;
        if(S_age_check && u_master.u_ink_watchdog.S_result_age_app!==S_age_before+24'd1)
            $fatal(1,"INK_WATCHDOG_LATER_EOF_RESET_AGE");
    end
    always @(negedge u_slave.u_ink_compute.W_matcher_rst_n) if(S_eg_rst_n)begin
        #0.001;
        if({u_slave.u_ink_compute.U_match.u_plate_stability.pending_plate,
            u_slave.u_ink_compute.U_match.u_plate_stability.pending_count,
            u_slave.u_ink_compute.U_match.u_plate_stability.lost_count,
            u_slave.u_ink_compute.W_res,u_slave.u_ink_compute.W_res_n,
            u_slave.u_ink_compute.W_res_valid,u_slave.u_ink_compute.W_raw_valid}!==0)
            $fatal(1,"INK_WATCHDOG_NEW_SESSION_C14_HISTORY");
        if(S_timeout_cycle>=0)S_new_matcher_resets=S_new_matcher_resets+1;
    end
    task automatic wait_response(input integer fid);
        integer guard;
        begin
            guard=0;while(S_result_counts[fid]==0 && guard<1000000)begin @(negedge clk_pix);guard=guard+1;end
            if(guard==1000000 || S_snapshot_counts[fid]!=1)$fatal(1,"INK_WATCHDOG_RECOVERY_RESULT fid=%0d",fid);
        end
    endtask
    initial begin : production_watchdog_recovery
        integer guard;
        for(integer f=0;f<32;f=f+1)begin S_result_counts[f]=0;S_snapshot_counts[f]=0;end
        configure_frame(0);m_vsync=1;m_de=0;m_ink_de=0;
        repeat(30)@(negedge clk_pix);rst_n=1;
        repeat(71)@(negedge clk_pix);S_eg_rst_n=1;
        guard=0;while((!u_master.W_ink_session_ready_pix || !u_slave.W_ink_session_ready) && guard<3000000)begin @(negedge clk_pix);guard=guard+1;end
        if(guard==3000000)$fatal(1,"INK_WATCHDOG_INITIAL_SESSION_TIMEOUT");
        repeat(100)@(negedge clk_pix);drive_raster(-1,726,749);configure_frame(0);
        drive_raster(0,0,749);wait_response(0);
        repeat(5)@(negedge clk_pix);
        S_old_epoch=u_master.W_ink_epoch_pix;S_clear_before=S_clears;S_block_results=1;
        for(integer f=1;f<=6;f=f+1)drive_raster(f,0,749);
        guard=0;while(S_timeout_cycle<0 && guard<8000000)begin @(negedge clk_pix);guard=guard+1;end
        if(guard==8000000 || S_later_eofs<5 || S_dropped_results<6)$fatal(1,"INK_WATCHDOG_TIMEOUT_COVERAGE later_eofs=%0d dropped_results=%0d",S_later_eofs,S_dropped_results);
        S_block_results=0;
        guard=0;
        while((u_master.W_ink_epoch_pix==S_old_epoch || !u_master.W_ink_session_ready_pix || !u_slave.W_ink_session_ready) && guard<3000000)begin @(negedge clk_pix);guard=guard+1;end
        if(guard==3000000 || S_clears<=S_clear_before)$fatal(1,"INK_WATCHDOG_SESSION_RESTART_TIMEOUT");
        // Frame7's META existed before expiry and was cancelled by the barrier.
        // Re-enter through a real high VS interval and a fresh falling edge.
        drive_raster(7,726,749);
        drive_raster(8,0,749);wait_response(8);
        if(S_new_matcher_resets<1 || u_slave.W_ink_epoch!==u_master.W_ink_epoch_pix)$fatal(1,"INK_WATCHDOG_NEW_EPOCH_COVERAGE");
        $display("TB_RESULT: PASS tb_ink_integrated_watchdog production_timeout_cycles=7500000 actual_timeout_ns=%0.3f later_EOF_not_extend=%0d RESULT_packets_omitted=%0d result_clear_exported=1 fresh_session_recovered=1 natural_C13_C14=1 errors=0",S_timeout_ns-S_first_lost_eof_ns,S_later_eofs,S_dropped_results);
        $finish;
    end
    initial begin #1000000000;$fatal(1,"INK_WATCHDOG_GLOBAL_TIMEOUT");end
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
