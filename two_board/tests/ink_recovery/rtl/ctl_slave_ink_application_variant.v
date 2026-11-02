`timescale 1ns/1ps
//=============================================================================
//  ctl_slave_ink_application -- shared frozen-top INK application composition
//  所属域 : ctl
//  时钟域 : original application/compute and GMII clocks
//  复位   : original common assertion and per-domain release
//  参数   : fixed production endpoint constants
//  验证状态 : extraction equality only; external simulation pending
//=============================================================================
module ctl_slave_ink_application (
    input wire I_clk_app,
    input wire I_clk_25m,
    input wire I_clk_tx,
    input wire I_clk_rx,
    input wire I_por_done,
    input wire I_pll_lock,
    input wire I_mode_pll_lock,
    input wire I_legacy_tx_busy,
    input wire I_legacy_tx_request,
    input wire I_legacy_gmii_tx_en,
    input wire I_tx_grant,
    input wire [7:0] I_gmii_rxd,
    input wire I_gmii_rx_dv,
    input wire I_gmii_rx_er,
    output wire O_tx_request,
    output wire O_tx_done,
    output wire [7:0] O_gmii_txd,
    output wire O_gmii_tx_en,
    output wire O_gmii_tx_er,
    output wire O_compute_busy,
    output wire O_compute_result_valid
);

    localparam [47:0] P_LOCAL_MAC = 48'h02_00_00_65_43_21;
    localparam [47:0] P_PEER_MAC = 48'h02_00_00_12_34_56;

    wire W_clk_mode;
    assign W_clk_mode = I_clk_app;
    wire W_clk_tx;
    assign W_clk_tx = I_clk_tx;
    wire W_clk_rx;
    assign W_clk_rx = I_clk_rx;
    wire W_por_done;
    assign W_por_done = I_por_done;
    wire W_pll_lock;
    assign W_pll_lock = I_pll_lock;
    wire W_mode_pll_lock;
    assign W_mode_pll_lock = I_mode_pll_lock;
    wire W_ocr_tx_busy;
    assign W_ocr_tx_busy = I_legacy_tx_busy;
    wire W_ocr_tx_request;
    assign W_ocr_tx_request = I_legacy_tx_request;
    wire W_ocr_gmii_tx_en;
    assign W_ocr_gmii_tx_en = I_legacy_gmii_tx_en;
    wire [7:0] W_gmii_rxd;
    assign W_gmii_rxd = I_gmii_rxd;
    wire W_gmii_rx_dv;
    assign W_gmii_rx_dv = I_gmii_rx_dv;
    wire W_gmii_rx_er;
    assign W_gmii_rx_er = I_gmii_rx_er;
    wire W_compute_busy;
    wire W_compute_response_valid;

    // All INK toggle/Gray-pointer endpoints share assertion. Mode-PLL loss
    // cancels the INK session while the qualified legacy hardware stays live.
    wire W_ink_reset_async_n = W_por_done & W_pll_lock & W_mode_pll_lock;
    reg [1:0] S_ink_reset_compute;
    reg [1:0] S_ink_reset_mode;
    reg [1:0] S_ink_reset_tx;
    reg [1:0] S_ink_reset_rx;
    initial begin
        S_ink_reset_compute = 2'b00;
        S_ink_reset_mode = 2'b00;
        S_ink_reset_tx = 2'b00;
        S_ink_reset_rx = 2'b00;
    end
    always @(posedge I_clk_25m or negedge W_ink_reset_async_n) begin
        if (!W_ink_reset_async_n) S_ink_reset_compute <= 2'b00;
        else S_ink_reset_compute <= {S_ink_reset_compute[0],1'b1};
    end
    always @(posedge W_clk_mode or negedge W_ink_reset_async_n) begin
        if (!W_ink_reset_async_n) S_ink_reset_mode <= 2'b00;
        else S_ink_reset_mode <= {S_ink_reset_mode[0],1'b1};
    end
    always @(posedge W_clk_tx or negedge W_ink_reset_async_n) begin
        if (!W_ink_reset_async_n) S_ink_reset_tx <= 2'b00;
        else S_ink_reset_tx <= {S_ink_reset_tx[0],1'b1};
    end
    always @(posedge W_clk_rx or negedge W_ink_reset_async_n) begin
        if (!W_ink_reset_async_n) S_ink_reset_rx <= 2'b00;
        else S_ink_reset_rx <= {S_ink_reset_rx[0],1'b1};
    end
    wire W_ink_rst_compute_n = S_ink_reset_compute[1];
    wire W_ink_rst_mode_n = S_ink_reset_mode[1];
    wire W_ink_rst_tx_n = S_ink_reset_tx[1];
    wire W_ink_rst_rx_n = S_ink_reset_rx[1];

    wire W_ink_packet_valid, W_ink_packet_release;
    wire [7:0] W_ink_packet_addr, W_ink_packet_data;
    wire W_ink_hello_seen;
    wire [31:0] W_ink_hello_epoch, W_ink_hello_challenge;
    wire W_ink_session_ready, W_ink_barrier, W_ink_session_abort;
    wire [31:0] W_ink_epoch;
    wire W_ink_pixel, W_ink_pixel_de, W_ink_vsync, W_ink_frame_abort;
    wire W_ink_detected, W_ink_complete_frame, W_ink_decoder_quiet;
    wire [6:0] W_ink_char_valid;
    wire [10:0] W_ink_char_up, W_ink_char_down;
    wire [76:0] W_ink_char_left, W_ink_char_right;
    wire [31:0] W_ink_frame_index;
    wire W_ink_frame_ready, W_ink_geometry_ready;
    wire W_ink_capture_active, W_ink_commit_active;
    wire W_ink_tile_valid, W_ink_tile_ready;
    wire [8:0] W_ink_tile_addr;
    wire [31:0] W_ink_tile_data;
    wire W_ink_snapshot_valid, W_ink_snapshot_drop, W_ink_snapshot_bank;
    wire [6:0] W_ink_snapshot_mask;
    wire [31:0] W_ink_snapshot_frame;
    wire W_ink_tile_src_ready, W_ink_tile_pending;
    wire W_ink_tile_dst_valid, W_ink_tile_dst_ready;
    wire [40:0] W_ink_tile_dst_data;
    wire W_ink_snapshot_src_ready, W_ink_snapshot_pending;
    wire W_ink_snapshot_dst_valid, W_ink_snapshot_dst_ready;
    wire [71:0] W_ink_snapshot_dst_data;
    reg S_ink_snapshot_offer_mode;
    reg [71:0] S_ink_snapshot_data_mode;
    reg S_ink_export_owned_mode, S_ink_compute_seen_mode;
    wire W_ink_compute_result_valid, W_ink_compute_result_ready;
    wire [188:0] W_ink_compute_result_data;
    wire W_ink_compute_owner_invalid, W_ink_compute_busy, W_ink_compute_quiet;
    wire W_ink_result_pending, W_ink_result_dst_valid, W_ink_result_dst_ready;
    wire [188:0] W_ink_result_dst_data;
    wire W_ink_control_valid, W_ink_control_ready;
    wire [7:0] W_ink_control_kind;
    wire [31:0] W_ink_control_epoch, W_ink_control_challenge;
    wire W_ink_reply_quiet, W_ink_reply_recover;
    wire [1:0] W_ink_free_bank_mask;
    wire W_ink_next_commit_bank;
    wire W_ink_tx_write_en, W_ink_tx_write_bank, W_ink_tx_commit, W_ink_tx_commit_bank;
    wire [7:0] W_ink_tx_write_addr, W_ink_tx_write_data;
    wire [8:0] W_ink_tx_length;
    wire W_ink_tx_commit_accept;
    wire W_ink_tx_request, W_ink_tx_grant, W_ink_tx_done;
    wire [7:0] W_ink_gmii_txd;
    wire W_ink_gmii_tx_en, W_ink_gmii_tx_er;
    reg [1:0] S_ink_barrier_compute;
    reg [1:0] S_ink_barrier_echo_mode;
    reg [1:0] S_ink_owner_invalid_mode;
    reg [1:0] S_ink_compute_busy_mode;
    reg [1:0] S_ink_result_pending_mode;
    reg [1:0] S_ink_legacy_busy_mode;
    reg [1:0] S_ink_legacy_tx_quiet_mode;
    reg [1:0] S_ink_tx_quiet_mode;
    reg [1:0] S_ink_rx_quiet_mode;
    reg [2:0] S_ink_rx_idle_count_rx;
    // Qualification outlasts transport EOF publication and its two-flop CDC.
    // It is a conservative level indication, never a sampled RX pulse.
    always @(posedge W_clk_rx or negedge W_ink_rst_rx_n) begin
        if (!W_ink_rst_rx_n) S_ink_rx_idle_count_rx <= 3'd0;
        else if (W_gmii_rx_dv) S_ink_rx_idle_count_rx <= 3'd0;
        else if (!(&S_ink_rx_idle_count_rx))
            S_ink_rx_idle_count_rx <= S_ink_rx_idle_count_rx + 3'd1;
    end
    always @(posedge I_clk_25m or negedge W_ink_rst_compute_n) begin
        if (!W_ink_rst_compute_n) S_ink_barrier_compute <= 2'b11;
        else S_ink_barrier_compute <= {S_ink_barrier_compute[0],W_ink_barrier};
    end
    always @(posedge W_clk_mode or negedge W_ink_rst_mode_n) begin
        if (!W_ink_rst_mode_n) begin
            S_ink_barrier_echo_mode <= 2'b00;
            S_ink_owner_invalid_mode <= 2'b00;
            S_ink_compute_busy_mode <= 2'b00;
            S_ink_result_pending_mode <= 2'b00;
            S_ink_legacy_busy_mode <= 2'b11;
            S_ink_legacy_tx_quiet_mode <= 2'b00;
            S_ink_tx_quiet_mode <= 2'b00;
            S_ink_rx_quiet_mode <= 2'b00;
        end else begin
            S_ink_barrier_echo_mode <= {S_ink_barrier_echo_mode[0],S_ink_barrier_compute[1]};
            S_ink_owner_invalid_mode <= {S_ink_owner_invalid_mode[0],W_ink_compute_owner_invalid};
            S_ink_compute_busy_mode <= {S_ink_compute_busy_mode[0],W_ink_compute_busy};
            S_ink_result_pending_mode <= {S_ink_result_pending_mode[0],W_ink_result_pending};
            S_ink_legacy_busy_mode <= {S_ink_legacy_busy_mode[0],W_ocr_tx_busy};
            S_ink_legacy_tx_quiet_mode <= {S_ink_legacy_tx_quiet_mode[0],!W_ocr_tx_request && !W_ocr_gmii_tx_en};
            S_ink_tx_quiet_mode <= {S_ink_tx_quiet_mode[0],!W_ink_tx_request && !W_ink_gmii_tx_en};
            S_ink_rx_quiet_mode <= {S_ink_rx_quiet_mode[0],&S_ink_rx_idle_count_rx};
        end
    end
    wire W_ink_sampler_abort = W_ink_frame_abort | W_ink_session_abort | W_ink_barrier;
    assign W_ink_tile_ready = W_ink_barrier | W_ink_tile_src_ready;
    // Snapshot is a pulse; hold it until the last tile mailbox ACK returns.
    // Its mask is already detected-gated by the sampler and is used as-is.
    wire W_ink_snapshot_offer = S_ink_snapshot_offer_mode && !W_ink_tile_pending && !W_ink_barrier;
    always @(posedge W_clk_mode or negedge W_ink_rst_mode_n) begin
        if (!W_ink_rst_mode_n) begin
            S_ink_snapshot_offer_mode <= 1'b0;
            S_ink_snapshot_data_mode <= 72'd0;
            S_ink_export_owned_mode <= 1'b0;
            S_ink_compute_seen_mode <= 1'b0;
        end else if (W_ink_barrier) begin
            S_ink_snapshot_offer_mode <= 1'b0;
            S_ink_export_owned_mode <= 1'b0;
            S_ink_compute_seen_mode <= 1'b0;
        end else begin
            if (W_ink_snapshot_offer && W_ink_snapshot_src_ready)
                S_ink_snapshot_offer_mode <= 1'b0;
            if (W_ink_snapshot_valid) begin
                S_ink_snapshot_data_mode <= {W_ink_epoch,W_ink_snapshot_frame,W_ink_snapshot_mask,W_ink_snapshot_bank};
                S_ink_snapshot_offer_mode <= 1'b1;
                S_ink_export_owned_mode <= 1'b1;
                S_ink_compute_seen_mode <= 1'b0;
            end
            if (S_ink_export_owned_mode && !W_ink_snapshot_valid &&
                !S_ink_snapshot_offer_mode && !W_ink_snapshot_pending && S_ink_compute_busy_mode[1])
                S_ink_compute_seen_mode <= 1'b1;
            if (S_ink_compute_seen_mode && !S_ink_compute_busy_mode[1]) begin
                S_ink_export_owned_mode <= 1'b0;
                S_ink_compute_seen_mode <= 1'b0;
            end
        end
    end
    wire W_ink_tile_quiet = !W_ink_tile_pending && !W_ink_tile_valid &&
        !W_ink_capture_active && !W_ink_commit_active;
    wire W_ink_snapshot_quiet = !W_ink_snapshot_pending && !S_ink_snapshot_offer_mode && !W_ink_snapshot_valid;
    // Read-pointer release occurs after the full 12-byte TX IFG.
    wire W_ink_transport_quiet = W_ink_free_bank_mask == 2'b11 && S_ink_tx_quiet_mode[1];
    wire W_ink_reply_all_quiet = W_ink_reply_quiet && W_ink_transport_quiet &&
        !S_ink_result_pending_mode[1] && !W_ink_result_dst_valid;
    // Rebuild a failed reply within this epoch; a real peer HELLO wins.
    // Neither transport pointers nor arithmetic ownership are reset here.
    wire W_ink_session_hello = W_ink_hello_seen | W_ink_reply_recover;
    wire [31:0] W_ink_session_hello_epoch = W_ink_hello_seen ? W_ink_hello_epoch : W_ink_epoch;
    wire [31:0] W_ink_session_hello_challenge = W_ink_hello_seen ? W_ink_hello_challenge : W_ink_control_challenge;

    ctl_ink_packet_rx #(.P_LOCAL_MAC(P_LOCAL_MAC), .P_PEER_MAC(P_PEER_MAC)) u_ink_packet_rx (
        .I_clk_rx(W_clk_rx), .I_rst_rx_n(W_ink_rst_rx_n),
        .I_gmii_rxd(W_gmii_rxd), .I_gmii_rx_dv(W_gmii_rx_dv), .I_gmii_rx_er(W_gmii_rx_er),
        .I_clk_app(W_clk_mode), .I_rst_app_n(W_ink_rst_mode_n),
        .O_packet_valid(W_ink_packet_valid), .I_packet_read_addr(W_ink_packet_addr),
        .O_packet_read_data(W_ink_packet_data), .I_packet_release(W_ink_packet_release),
        .O_bad_frame_count(), .O_full_drop_count()
    );
    ctl_slave_ink_decoder u_ink_decoder (
        .I_clk_app(W_clk_mode), .I_rst_app_n(W_ink_rst_mode_n),
        .I_packet_valid(W_ink_packet_valid), .O_packet_read_addr(W_ink_packet_addr),
        .I_packet_read_data(W_ink_packet_data), .O_packet_release(W_ink_packet_release),
        .I_session_ready(W_ink_session_ready), .I_epoch(W_ink_epoch), .I_barrier_abort(W_ink_session_abort),
        .I_sampler_frame_ready(W_ink_frame_ready), .I_sampler_geometry_ready(W_ink_geometry_ready),
        .O_hello_seen(W_ink_hello_seen), .O_hello_epoch(W_ink_hello_epoch), .O_hello_challenge(W_ink_hello_challenge),
        .O_ink(W_ink_pixel), .O_ink_de(W_ink_pixel_de), .O_vsync(W_ink_vsync), .O_frame_abort(W_ink_frame_abort),
        .O_detected(W_ink_detected), .O_char_valid(W_ink_char_valid), .O_char_up(W_ink_char_up),
        .O_char_down(W_ink_char_down), .O_char_left(W_ink_char_left), .O_char_right(W_ink_char_right),
        .O_external_frame_index(W_ink_frame_index), .O_complete_frame(W_ink_complete_frame),
        .O_bad_sequence_count(), .O_raw_quiet(W_ink_decoder_quiet)
    );
    ctl_slave_ink_session #(.P_BOOT_RETRY_CYCLES(75000)) u_ink_session (
        .I_clk_app(W_clk_mode), .I_rst_app_n(W_ink_rst_mode_n),
        .I_hello_seen(W_ink_session_hello), .I_hello_epoch(W_ink_session_hello_epoch), .I_hello_challenge(W_ink_session_hello_challenge),
        .I_raw_quiet(W_ink_decoder_quiet), .I_tile_quiet(W_ink_tile_quiet), .I_snapshot_quiet(W_ink_snapshot_quiet),
        .I_old_owner_invalid(S_ink_owner_invalid_mode[1] && S_ink_barrier_echo_mode[1]),
        .I_legacy_tx_quiet(!S_ink_legacy_busy_mode[1] && S_ink_legacy_tx_quiet_mode[1]),
        .I_reply_tx_quiet(W_ink_reply_all_quiet), .I_rx_quiet(S_ink_rx_quiet_mode[1] && !W_ink_packet_valid),
        .I_control_ready(W_ink_control_ready), .O_control_valid(W_ink_control_valid),
        .O_control_kind(W_ink_control_kind), .O_control_epoch(W_ink_control_epoch),
        .O_control_challenge(W_ink_control_challenge), .O_session_ready(W_ink_session_ready),
        .O_barrier(W_ink_barrier), .O_abort_frame(W_ink_session_abort), .O_epoch(W_ink_epoch)
    );
    cap_master_tile_capture #(.P_USE_EXTERNAL_FRAME_INDEX(1), .P_USE_TILE_READY(1), .P_PIPELINE_GEOMETRY(1)) u_ink_sampler (
        .I_clk_pix(W_clk_mode), .I_rst_pix_n(W_ink_rst_mode_n),
        .I_ink(W_ink_pixel), .I_ink_de(W_ink_pixel_de), .I_vsync(W_ink_vsync), .I_detected(W_ink_detected),
        .I_char_up(W_ink_char_up), .I_char_down(W_ink_char_down), .I_char_valid(W_ink_char_valid),
        .I_c1_left(W_ink_char_left[10:0]), .I_c1_right(W_ink_char_right[10:0]),
        .I_c2_left(W_ink_char_left[21:11]), .I_c2_right(W_ink_char_right[21:11]),
        .I_c3_left(W_ink_char_left[32:22]), .I_c3_right(W_ink_char_right[32:22]),
        .I_c4_left(W_ink_char_left[43:33]), .I_c4_right(W_ink_char_right[43:33]),
        .I_c5_left(W_ink_char_left[54:44]), .I_c5_right(W_ink_char_right[54:44]),
        .I_c6_left(W_ink_char_left[65:55]), .I_c6_right(W_ink_char_right[65:55]),
        .I_c7_left(W_ink_char_left[76:66]), .I_c7_right(W_ink_char_right[76:66]),
        .I_export_busy(S_ink_export_owned_mode || S_ink_compute_busy_mode[1] || W_ink_snapshot_pending),
        .I_frame_abort(W_ink_sampler_abort), .I_external_frame_index(W_ink_frame_index), .I_tile_ready(W_ink_tile_ready),
        .O_tile_write_en(W_ink_tile_valid), .O_tile_write_addr(W_ink_tile_addr), .O_tile_write_data(W_ink_tile_data),
        .O_snapshot_valid(W_ink_snapshot_valid), .O_snapshot_drop(W_ink_snapshot_drop),
        .O_snapshot_bank(W_ink_snapshot_bank), .O_snapshot_mask(W_ink_snapshot_mask), .O_snapshot_frame_index(W_ink_snapshot_frame),
        .O_capture_active(W_ink_capture_active), .O_commit_active(W_ink_commit_active),
        .O_geometry_ready(W_ink_geometry_ready), .O_frame_ready(W_ink_frame_ready)
    );
    ctl_mailbox_cdc #(.P_DATA_WIDTH(41)) u_ink_tile_cdc (
        .I_clk_src(W_clk_mode), .I_rst_src_n(W_ink_rst_mode_n),
        .I_src_valid(W_ink_tile_valid && !W_ink_barrier), .I_src_data({W_ink_tile_addr,W_ink_tile_data}),
        .O_src_ready(W_ink_tile_src_ready), .O_src_pending(W_ink_tile_pending),
        .I_clk_dst(I_clk_25m), .I_rst_dst_n(W_ink_rst_compute_n),
        .O_dst_valid(W_ink_tile_dst_valid), .O_dst_data(W_ink_tile_dst_data),
        .I_dst_ready(W_ink_tile_dst_ready || S_ink_barrier_compute[1])
    );
    ctl_mailbox_cdc #(.P_DATA_WIDTH(72)) u_ink_snapshot_cdc (
        .I_clk_src(W_clk_mode), .I_rst_src_n(W_ink_rst_mode_n),
        .I_src_valid(W_ink_snapshot_offer), .I_src_data(S_ink_snapshot_data_mode),
        .O_src_ready(W_ink_snapshot_src_ready), .O_src_pending(W_ink_snapshot_pending),
        .I_clk_dst(I_clk_25m), .I_rst_dst_n(W_ink_rst_compute_n),
        .O_dst_valid(W_ink_snapshot_dst_valid), .O_dst_data(W_ink_snapshot_dst_data),
        .I_dst_ready(W_ink_snapshot_dst_ready || S_ink_barrier_compute[1])
    );
    ctl_slave_ink_compute u_ink_compute (
        .I_clk_app(I_clk_25m), .I_rst_app_n(W_ink_rst_compute_n), .I_barrier(S_ink_barrier_compute[1]),
        .I_tile_valid(W_ink_tile_dst_valid), .O_tile_ready(W_ink_tile_dst_ready), .I_tile_data(W_ink_tile_dst_data),
        .I_snapshot_valid(W_ink_snapshot_dst_valid), .O_snapshot_ready(W_ink_snapshot_dst_ready), .I_snapshot_data(W_ink_snapshot_dst_data),
        .O_result_valid(W_ink_compute_result_valid), .I_result_ready(W_ink_compute_result_ready), .O_result_data(W_ink_compute_result_data),
        .O_old_owner_invalid(W_ink_compute_owner_invalid), .O_core_busy(W_ink_compute_busy), .O_quiet(W_ink_compute_quiet),
        .O_feature_valid(), .O_feature_data(), .O_feature_index()
    );
    ctl_mailbox_cdc #(.P_DATA_WIDTH(189)) u_ink_result_cdc (
        .I_clk_src(I_clk_25m), .I_rst_src_n(W_ink_rst_compute_n),
        .I_src_valid(W_ink_compute_result_valid), .I_src_data(W_ink_compute_result_data),
        .O_src_ready(W_ink_compute_result_ready), .O_src_pending(W_ink_result_pending),
        .I_clk_dst(W_clk_mode), .I_rst_dst_n(W_ink_rst_mode_n),
        .O_dst_valid(W_ink_result_dst_valid), .O_dst_data(W_ink_result_dst_data),
        .I_dst_ready(W_ink_result_dst_ready || W_ink_barrier)
    );
    ctl_slave_ink_reply u_ink_reply (
        .I_clk_app(W_clk_mode), .I_rst_app_n(W_ink_rst_mode_n),
        .I_barrier(W_ink_barrier), .I_cancel(W_ink_hello_seen),
        .I_control_valid(W_ink_control_valid), .O_control_ready(W_ink_control_ready),
        .I_control_kind(W_ink_control_kind), .I_control_epoch(W_ink_control_epoch), .I_control_challenge(W_ink_control_challenge),
        .I_result_valid(W_ink_result_dst_valid), .O_result_ready(W_ink_result_dst_ready), .I_result_data(W_ink_result_dst_data),
        .I_free_bank_mask(W_ink_free_bank_mask), .I_next_commit_bank(W_ink_next_commit_bank),
        .O_write_en(W_ink_tx_write_en), .O_write_bank(W_ink_tx_write_bank), .O_write_addr(W_ink_tx_write_addr), .O_write_data(W_ink_tx_write_data),
        .O_commit(W_ink_tx_commit), .O_commit_bank(W_ink_tx_commit_bank), .O_packet_length(W_ink_tx_length),
        .I_commit_accept(W_ink_tx_commit_accept), .I_transport_quiet(W_ink_transport_quiet),
        .O_session_recover(W_ink_reply_recover), .O_source_quiet(W_ink_reply_quiet)
    );
    ctl_ink_packet_tx #(.P_LOCAL_MAC(P_LOCAL_MAC), .P_PEER_MAC(P_PEER_MAC)) u_ink_packet_tx (
        .I_clk_app(W_clk_mode), .I_rst_app_n(W_ink_rst_mode_n), .O_free_bank_mask(W_ink_free_bank_mask), .O_next_commit_bank(W_ink_next_commit_bank),
        .I_write_en(W_ink_tx_write_en), .I_write_bank(W_ink_tx_write_bank), .I_write_addr(W_ink_tx_write_addr), .I_write_data(W_ink_tx_write_data),
        .I_commit(W_ink_tx_commit), .I_commit_bank(W_ink_tx_commit_bank), .I_packet_length(W_ink_tx_length),
        .O_commit_accept(W_ink_tx_commit_accept), .O_commit_drop_count(),
        .I_clk_tx(W_clk_tx), .I_rst_tx_n(W_ink_rst_tx_n),
        .O_tx_request(W_ink_tx_request), .I_tx_grant(W_ink_tx_grant), .O_tx_done(W_ink_tx_done),
        .O_gmii_txd(W_ink_gmii_txd), .O_gmii_tx_en(W_ink_gmii_tx_en), .O_gmii_tx_er(W_ink_gmii_tx_er)
    );
    assign W_compute_busy = W_ink_compute_busy;
    assign W_compute_response_valid = W_ink_compute_result_valid;


    assign W_ink_tx_grant = I_tx_grant;
    assign O_tx_request = W_ink_tx_request;
    assign O_tx_done = W_ink_tx_done;
    assign O_gmii_txd = W_ink_gmii_txd;
    assign O_gmii_tx_en = W_ink_gmii_tx_en;
    assign O_gmii_tx_er = W_ink_gmii_tx_er;
    assign O_compute_busy = W_compute_busy;
    assign O_compute_result_valid = W_compute_response_valid;
endmodule
