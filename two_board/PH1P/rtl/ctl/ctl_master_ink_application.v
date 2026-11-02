`timescale 1ns/1ps
//=============================================================================
//  ctl_master_ink_application -- shared frozen-top INK application composition
//  所属域 : ctl
//  时钟域 : original application/compute and GMII clocks
//  复位   : original common assertion and per-domain release
//  参数   : fixed production endpoint constants
//  验证状态 : extraction equality only; external simulation pending
//=============================================================================
module ctl_master_ink_application (
    input wire I_clk_app,
    input wire I_clk_tx,
    input wire I_clk_rx,
    input wire I_reset_app_async_n,
    input wire I_reset_tx_async_n,
    input wire I_reset_rx_async_n,
    input wire I_link_ready,
    input wire I_ink,
    input wire I_ink_de,
    input wire I_vsync,
    input wire I_detected,
    input wire [10:0] I_char_up,
    input wire [10:0] I_char_down,
    input wire [6:0] I_char_valid,
    input wire [76:0] I_char_left,
    input wire [76:0] I_char_right,
    input wire I_legacy_tx_request,
    input wire I_legacy_rx_valid,
    input wire I_tx_grant,
    input wire [7:0] I_gmii_rxd,
    input wire I_gmii_rx_dv,
    input wire I_gmii_rx_er,
    output wire O_tx_request,
    output wire O_tx_done,
    output wire [7:0] O_gmii_txd,
    output wire O_gmii_tx_en,
    output wire O_gmii_tx_er,
    output wire [55:0] O_res,
    output wire [3:0] O_res_n,
    output wire O_res_valid,
    output wire [6:0] O_raw_valid,
    output wire O_frame_result,
    output wire [55:0] O_raw_candidate_code,
    output wire [6:0] O_raw_candidate_mask,
    output wire [3:0] O_match_state,
    output wire [15:0] O_diag_status,
    output wire [15:0] O_diag_counts
);

    wire W_ink_reset_async_n;
    wire W_ink_reset_pix_n;
    wire W_ink_reset_tx_n;
    wire W_ink_reset_rx_n;
    reg [1:0] S_ink_reset_release_pix;
    reg [1:0] S_ink_reset_release_tx;
    reg [1:0] S_ink_reset_release_rx;
    reg [1:0] S_ink_tx_request_sync_pix;
    reg [1:0] S_ink_legacy_request_sync_pix;
    wire W_ink_transport_quiet_pix;
    wire W_ink_source_quiet_pix;
    wire W_ink_rx_quiet_pix;
    wire W_ink_session_ready_pix;
    wire W_ink_barrier_pix;
    wire W_ink_result_clear_pix;
    wire W_ink_session_recover_pix;
    wire W_ink_result_timeout_pix;
    wire [31:0] W_ink_epoch_pix;
    wire W_ink_control_valid_pix;
    wire W_ink_control_ready_pix;
    wire [7:0] W_ink_control_kind_pix;
    wire [31:0] W_ink_control_epoch_pix;
    wire [31:0] W_ink_control_challenge_pix;
    wire W_ink_ack_seen_pix;
    wire [31:0] W_ink_ack_epoch_pix;
    wire [31:0] W_ink_ack_challenge_pix;
    wire W_ink_peer_hello_pix;
    wire [1:0] W_ink_free_bank_mask_pix;
    wire W_ink_next_commit_bank_pix;
    wire W_ink_write_en_pix;
    wire W_ink_write_bank_pix;
    wire [7:0] W_ink_write_addr_pix;
    wire [7:0] W_ink_write_data_pix;
    wire W_ink_commit_pix;
    wire W_ink_commit_bank_pix;
    wire [8:0] W_ink_packet_length_pix;
    wire W_ink_commit_accept_pix;
    wire W_ink_eof_committed_pix;
    wire [31:0] W_ink_eof_epoch_pix;
    wire [31:0] W_ink_eof_frame_index_pix;
    wire W_ink_packet_valid_pix;
    wire [7:0] W_ink_packet_read_addr_pix;
    wire [7:0] W_ink_packet_read_data_pix;
    wire W_ink_packet_release_pix;
    wire W_ink_tx_request_tx;
    wire W_ink_tx_done_tx;
    wire W_ink_tx_grant_tx;
    wire [7:0] W_ink_gmii_txd_tx;
    wire W_ink_gmii_tx_en_tx;
    wire W_ink_gmii_tx_er_tx;
    wire W_ink_legacy_rx_valid_pix;
    wire W_ink_frame_result_pix;
    wire [55:0] S_res;
    wire [3:0] S_res_n;
    wire S_res_valid;
    wire [6:0] S_raw_valid;
    wire S_frame_result;
    wire [55:0] S_raw_candidate_code;
    wire [6:0] S_raw_candidate_mask;
    wire [3:0] S_match_state_public;

    // Diagnostics observe transport events without participating in ready/valid.
    // Only counter low nibbles are consumed; upper carry chains can be removed.
    wire [31:0] W_diag_capture_drop_pix;
    wire [31:0] W_diag_rejected_pix;
    wire [31:0] W_diag_bad_rx;
    wire [31:0] W_diag_full_rx;
    reg S_diag_bad_rx;
    reg S_diag_full_rx;
    (* async_reg = "true" *) reg [1:0] S_diag_bad_sync_pix;
    (* async_reg = "true" *) reg [1:0] S_diag_full_sync_pix;
    reg [3:0] S_diag_drop_previous_pix;
    reg S_diag_ready_previous_pix;
    reg [3:0] S_diag_eof_count_pix;
    reg [3:0] S_diag_result_count_pix;
    reg [3:0] S_diag_timeout_count_pix;
    reg S_diag_eof_seen_pix;
    reg S_diag_result_seen_pix;
    reg S_diag_drop_seen_pix;
    reg S_diag_reject_seen_pix;
    reg S_diag_timeout_seen_pix;
    reg S_diag_recover_seen_pix;
    reg S_diag_peer_seen_pix;
    reg S_diag_ack_seen_pix;
    reg S_diag_stable_seen_pix;
    assign S_frame_result = W_ink_frame_result_pix || W_ink_result_clear_pix;
    assign S_raw_candidate_mask = S_raw_valid;
    assign S_match_state_public = 4'd0;


    wire S_hdmi_pixel_clk;
    assign S_hdmi_pixel_clk = I_clk_app;
    wire S_eth_gmii_tx_clk;
    assign S_eth_gmii_tx_clk = I_clk_tx;
    wire S_eth_gmii_rx_clk;
    assign S_eth_gmii_rx_clk = I_clk_rx;
    wire S_eth_app_reset_async_n;
    assign S_eth_app_reset_async_n = I_reset_app_async_n;
    wire W_eth_tx_common_async_n;
    assign W_eth_tx_common_async_n = I_reset_tx_async_n;
    wire W_eth_rx_common_async_n;
    assign W_eth_rx_common_async_n = I_reset_rx_async_n;
    wire S_eth_gigabit_ready_app;
    assign S_eth_gigabit_ready_app = I_link_ready;
    wire S_char_ink;
    assign S_char_ink = I_ink;
    wire S_char_ink_de;
    assign S_char_ink_de = I_ink_de;
    wire S_recog_vsync;
    assign S_recog_vsync = I_vsync;
    wire S_char_detected;
    assign S_char_detected = I_detected;
    wire [10:0] S_char_up;
    assign S_char_up = I_char_up;
    wire [10:0] S_char_down;
    assign S_char_down = I_char_down;
    wire [6:0] S_char_valid;
    assign S_char_valid = I_char_valid;
    wire S_ocr_tx_request;
    assign S_ocr_tx_request = I_legacy_tx_request;
    assign W_ink_legacy_rx_valid_pix = I_legacy_rx_valid;
    assign W_ink_tx_grant_tx = I_tx_grant;
    wire [7:0] S_eth_gmii_rxd;
    assign S_eth_gmii_rxd = I_gmii_rxd;
    wire S_eth_gmii_rx_dv;
    assign S_eth_gmii_rx_dv = I_gmii_rx_dv;
    wire S_eth_gmii_rx_er;
    assign S_eth_gmii_rx_er = I_gmii_rx_er;
    wire [10:0] S_c1_left = I_char_left[10:0];
    wire [10:0] S_c1_right = I_char_right[10:0];
    wire [10:0] S_c2_left = I_char_left[21:11];
    wire [10:0] S_c2_right = I_char_right[21:11];
    wire [10:0] S_c3_left = I_char_left[32:22];
    wire [10:0] S_c3_right = I_char_right[32:22];
    wire [10:0] S_c4_left = I_char_left[43:33];
    wire [10:0] S_c4_right = I_char_right[43:33];
    wire [10:0] S_c5_left = I_char_left[54:44];
    wire [10:0] S_c5_right = I_char_right[54:44];
    wire [10:0] S_c6_left = I_char_left[65:55];
    wire [10:0] S_c6_right = I_char_right[65:55];
    wire [10:0] S_c7_left = I_char_left[76:66];
    wire [10:0] S_c7_right = I_char_right[76:66];

    // The new transport shares assertion across app, RX and TX. Each domain
    // releases through its own synchronizer. Existing hardware resets remain.
    assign W_ink_reset_async_n = S_eth_app_reset_async_n &
        W_eth_tx_common_async_n & W_eth_rx_common_async_n;
    always @(posedge S_hdmi_pixel_clk or negedge W_ink_reset_async_n) begin
        if (!W_ink_reset_async_n) begin
            S_ink_reset_release_pix <= 2'b00;
            S_ink_tx_request_sync_pix <= 2'b00;
            S_ink_legacy_request_sync_pix <= 2'b00;
        end else begin
            S_ink_reset_release_pix <= {S_ink_reset_release_pix[0], 1'b1};
            S_ink_tx_request_sync_pix <= {S_ink_tx_request_sync_pix[0], W_ink_tx_request_tx};
            S_ink_legacy_request_sync_pix <= {S_ink_legacy_request_sync_pix[0], S_ocr_tx_request};
        end
    end
    always @(posedge S_eth_gmii_tx_clk or negedge W_ink_reset_async_n) begin
        if (!W_ink_reset_async_n) S_ink_reset_release_tx <= 2'b00;
        else S_ink_reset_release_tx <= {S_ink_reset_release_tx[0], 1'b1};
    end
    always @(posedge S_eth_gmii_rx_clk or negedge W_ink_reset_async_n) begin
        if (!W_ink_reset_async_n) S_ink_reset_release_rx <= 2'b00;
        else S_ink_reset_release_rx <= {S_ink_reset_release_rx[0], 1'b1};
    end
    assign W_ink_reset_pix_n = S_ink_reset_release_pix[1];
    assign W_ink_reset_tx_n = S_ink_reset_release_tx[1];
    assign W_ink_reset_rx_n = S_ink_reset_release_rx[1];
    // Recovery needs physical transport quiet even if a source reservation
    // is still held. The session additionally waits for source cancellation.
    assign W_ink_transport_quiet_pix = W_ink_free_bank_mask_pix == 2'b11 &&
        !S_ink_tx_request_sync_pix[1] && !S_ink_legacy_request_sync_pix[1];

    // Preserve the legacy request adapter's 100 ms result deadline.
    // Recover through the session barrier; no transport pointer is reset.
    ctl_master_ink_watchdog u_ink_watchdog (
        .I_clk_app(S_hdmi_pixel_clk), .I_rst_app_n(W_ink_reset_pix_n),
        .I_session_ready(W_ink_session_ready_pix),
        .I_eof_committed(W_ink_eof_committed_pix),
        .I_result_frame(W_ink_frame_result_pix),
        .O_session_recover(W_ink_result_timeout_pix)
    );

    ctl_master_ink_session u_ink_session (
        .I_clk_app(S_hdmi_pixel_clk), .I_rst_app_n(W_ink_reset_pix_n),
        .I_restart(W_ink_session_recover_pix || W_ink_result_timeout_pix || !S_eth_gigabit_ready_app),
        .I_peer_hello(W_ink_peer_hello_pix), .I_ack_seen(W_ink_ack_seen_pix),
        .I_ack_epoch(W_ink_ack_epoch_pix), .I_ack_challenge(W_ink_ack_challenge_pix),
        .I_transport_quiet(W_ink_transport_quiet_pix && W_ink_source_quiet_pix),
        .I_rx_quiet(W_ink_rx_quiet_pix && !W_ink_legacy_rx_valid_pix),
        .I_control_ready(W_ink_control_ready_pix),
        .O_control_valid(W_ink_control_valid_pix), .O_control_kind(W_ink_control_kind_pix),
        .O_control_epoch(W_ink_control_epoch_pix),
        .O_control_challenge(W_ink_control_challenge_pix),
        .O_session_ready(W_ink_session_ready_pix), .O_barrier(W_ink_barrier_pix),
        .O_result_clear(W_ink_result_clear_pix), .O_epoch(W_ink_epoch_pix)
    );
    ctl_master_ink_capture u_ink_capture (
        .I_clk_app(S_hdmi_pixel_clk), .I_rst_app_n(W_ink_reset_pix_n),
        .I_ink(S_char_ink), .I_ink_de(S_char_ink_de), .I_vsync(S_recog_vsync),
        .I_detected(S_char_detected), .I_char_up(S_char_up), .I_char_down(S_char_down),
        .I_char_valid(S_char_valid),
        .I_char_left({S_c7_left, S_c6_left, S_c5_left, S_c4_left, S_c3_left, S_c2_left, S_c1_left}),
        .I_char_right({S_c7_right, S_c6_right, S_c5_right, S_c4_right, S_c3_right, S_c2_right, S_c1_right}),
        .I_session_ready(W_ink_session_ready_pix), .I_epoch(W_ink_epoch_pix),
        .I_control_valid(W_ink_control_valid_pix), .I_control_kind(W_ink_control_kind_pix),
        .I_control_epoch(W_ink_control_epoch_pix),
        .I_control_challenge(W_ink_control_challenge_pix),
        .O_control_ready(W_ink_control_ready_pix),
        .I_free_bank_mask(W_ink_free_bank_mask_pix),
        .I_next_commit_bank(W_ink_next_commit_bank_pix),
        .I_transport_quiet(W_ink_transport_quiet_pix),
        .O_write_en(W_ink_write_en_pix), .O_write_bank(W_ink_write_bank_pix),
        .O_write_addr(W_ink_write_addr_pix), .O_write_data(W_ink_write_data_pix),
        .O_commit(W_ink_commit_pix), .O_commit_bank(W_ink_commit_bank_pix),
        .O_packet_length(W_ink_packet_length_pix), .I_commit_accept(W_ink_commit_accept_pix),
        .O_session_recover(W_ink_session_recover_pix),
        .O_eof_committed(W_ink_eof_committed_pix), .O_eof_epoch(W_ink_eof_epoch_pix),
        .O_eof_frame_index(W_ink_eof_frame_index_pix), .O_frame_drop_count(W_diag_capture_drop_pix),
        .O_source_quiet(W_ink_source_quiet_pix)
    );
    ctl_ink_packet_tx #(
        .P_LOCAL_MAC(48'h02_00_00_12_34_56), .P_PEER_MAC(48'h02_00_00_65_43_21)
    ) u_ink_packet_tx (
        .I_clk_app(S_hdmi_pixel_clk), .I_rst_app_n(W_ink_reset_pix_n),
        .O_free_bank_mask(W_ink_free_bank_mask_pix),
        .O_next_commit_bank(W_ink_next_commit_bank_pix),
        .I_write_en(W_ink_write_en_pix), .I_write_bank(W_ink_write_bank_pix),
        .I_write_addr(W_ink_write_addr_pix), .I_write_data(W_ink_write_data_pix),
        .I_commit(W_ink_commit_pix), .I_commit_bank(W_ink_commit_bank_pix),
        .I_packet_length(W_ink_packet_length_pix), .O_commit_accept(W_ink_commit_accept_pix),
        .O_commit_drop_count(), .I_clk_tx(S_eth_gmii_tx_clk), .I_rst_tx_n(W_ink_reset_tx_n),
        .O_tx_request(W_ink_tx_request_tx), .I_tx_grant(W_ink_tx_grant_tx),
        .O_tx_done(W_ink_tx_done_tx), .O_gmii_txd(W_ink_gmii_txd_tx),
        .O_gmii_tx_en(W_ink_gmii_tx_en_tx), .O_gmii_tx_er(W_ink_gmii_tx_er_tx)
    );
    ctl_ink_packet_rx #(
        .P_LOCAL_MAC(48'h02_00_00_12_34_56), .P_PEER_MAC(48'h02_00_00_65_43_21)
    ) u_ink_packet_rx (
        .I_clk_rx(S_eth_gmii_rx_clk), .I_rst_rx_n(W_ink_reset_rx_n),
        .I_gmii_rxd(S_eth_gmii_rxd), .I_gmii_rx_dv(S_eth_gmii_rx_dv),
        .I_gmii_rx_er(S_eth_gmii_rx_er), .I_clk_app(S_hdmi_pixel_clk),
        .I_rst_app_n(W_ink_reset_pix_n), .O_packet_valid(W_ink_packet_valid_pix),
        .I_packet_read_addr(W_ink_packet_read_addr_pix),
        .O_packet_read_data(W_ink_packet_read_data_pix),
        .I_packet_release(W_ink_packet_release_pix),
        .O_bad_frame_count(W_diag_bad_rx), .O_full_drop_count(W_diag_full_rx)
    );
    ctl_master_ink_response u_ink_response (
        .I_clk_app(S_hdmi_pixel_clk), .I_rst_app_n(W_ink_reset_pix_n),
        .I_barrier(W_ink_barrier_pix), .I_packet_valid(W_ink_packet_valid_pix),
        .O_packet_read_addr(W_ink_packet_read_addr_pix),
        .I_packet_read_data(W_ink_packet_read_data_pix),
        .O_packet_release(W_ink_packet_release_pix),
        .I_eof_committed(W_ink_eof_committed_pix), .I_eof_epoch(W_ink_eof_epoch_pix),
        .I_eof_frame_index(W_ink_eof_frame_index_pix),
        .O_ack_seen(W_ink_ack_seen_pix), .O_ack_epoch(W_ink_ack_epoch_pix),
        .O_ack_challenge(W_ink_ack_challenge_pix), .O_peer_hello(W_ink_peer_hello_pix),
        .O_res(S_res), .O_res_n(S_res_n), .O_res_valid(S_res_valid),
        .O_frame_result(W_ink_frame_result_pix), .O_raw_code(S_raw_candidate_code),
        .O_raw_valid(S_raw_valid), .O_rejected_result_count(W_diag_rejected_pix), .O_rx_quiet(W_ink_rx_quiet_pix)
    );

    // Latch RX errors in their owner domain, then synchronize held levels.
    always @(posedge I_clk_rx or negedge W_ink_reset_rx_n) begin
        if (!W_ink_reset_rx_n) begin
            S_diag_bad_rx <= 1'b0;
            S_diag_full_rx <= 1'b0;
        end else begin
            if (|W_diag_bad_rx[3:0]) S_diag_bad_rx <= 1'b1;
            if (|W_diag_full_rx[3:0]) S_diag_full_rx <= 1'b1;
        end
    end

    // History survives session barriers; a common endpoint reset clears it.
    always @(posedge I_clk_app or negedge W_ink_reset_pix_n) begin
        if (!W_ink_reset_pix_n) begin
            S_diag_bad_sync_pix <= 2'b00;
            S_diag_full_sync_pix <= 2'b00;
            S_diag_drop_previous_pix <= 4'd0;
            S_diag_ready_previous_pix <= 1'b0;
            S_diag_eof_count_pix <= 4'd0;
            S_diag_result_count_pix <= 4'd0;
            S_diag_timeout_count_pix <= 4'd0;
            S_diag_eof_seen_pix <= 1'b0;
            S_diag_result_seen_pix <= 1'b0;
            S_diag_drop_seen_pix <= 1'b0;
            S_diag_reject_seen_pix <= 1'b0;
            S_diag_timeout_seen_pix <= 1'b0;
            S_diag_recover_seen_pix <= 1'b0;
            S_diag_peer_seen_pix <= 1'b0;
            S_diag_ack_seen_pix <= 1'b0;
            S_diag_stable_seen_pix <= 1'b0;
        end else begin
            S_diag_bad_sync_pix <= {S_diag_bad_sync_pix[0], S_diag_bad_rx};
            S_diag_full_sync_pix <= {S_diag_full_sync_pix[0], S_diag_full_rx};
            S_diag_drop_previous_pix <= W_diag_capture_drop_pix[3:0];
            S_diag_ready_previous_pix <= W_ink_session_ready_pix;
            if (W_ink_eof_committed_pix) begin
                S_diag_eof_count_pix <= S_diag_eof_count_pix + 4'd1;
                S_diag_eof_seen_pix <= 1'b1;
            end
            if (W_ink_frame_result_pix) begin
                S_diag_result_count_pix <= S_diag_result_count_pix + 4'd1;
                S_diag_result_seen_pix <= 1'b1;
            end
            // Startup frames skipped before session-ready are not stream faults.
            if (W_ink_session_ready_pix && S_diag_ready_previous_pix &&
                W_diag_capture_drop_pix[3:0] != S_diag_drop_previous_pix)
                S_diag_drop_seen_pix <= 1'b1;
            if (|W_diag_rejected_pix[3:0]) S_diag_reject_seen_pix <= 1'b1;
            if (W_ink_result_timeout_pix) begin
                S_diag_timeout_count_pix <= S_diag_timeout_count_pix + 4'd1;
                S_diag_timeout_seen_pix <= 1'b1;
            end
            if (W_ink_session_recover_pix) S_diag_recover_seen_pix <= 1'b1;
            if (W_ink_peer_hello_pix) S_diag_peer_seen_pix <= 1'b1;
            if (W_ink_ack_seen_pix) S_diag_ack_seen_pix <= 1'b1;
            if (S_res_valid) S_diag_stable_seen_pix <= 1'b1;
        end
    end

    assign O_diag_status = {
        S_diag_stable_seen_pix, S_diag_ack_seen_pix, S_diag_peer_seen_pix,
        S_diag_recover_seen_pix, S_raw_valid == 7'h7f, I_char_valid == 7'h7f,
        I_detected, S_diag_timeout_seen_pix, S_diag_full_sync_pix[1],
        S_diag_bad_sync_pix[1], S_diag_reject_seen_pix, S_diag_drop_seen_pix,
        S_res_valid, S_diag_result_seen_pix, S_diag_eof_seen_pix, W_ink_session_ready_pix
    };
    assign O_diag_counts = {S_diag_timeout_count_pix, W_diag_rejected_pix[3:0],
        S_diag_result_count_pix, S_diag_eof_count_pix};

    assign O_tx_request = W_ink_tx_request_tx;
    assign O_tx_done = W_ink_tx_done_tx;
    assign O_gmii_txd = W_ink_gmii_txd_tx;
    assign O_gmii_tx_en = W_ink_gmii_tx_en_tx;
    assign O_gmii_tx_er = W_ink_gmii_tx_er_tx;
    assign O_res = S_res;
    assign O_res_n = S_res_n;
    assign O_res_valid = S_res_valid;
    assign O_raw_valid = S_raw_valid;
    assign O_frame_result = S_frame_result;
    assign O_raw_candidate_code = S_raw_candidate_code;
    assign O_raw_candidate_mask = S_raw_candidate_mask;
    assign O_match_state = S_match_state_public;
endmodule
