//=============================================================================
//  dsp_camera_modes -- 二十种摄像头模式与历史远端邻域接口
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk_pix
//  复位     : I_rst_n，异步低有效
//  参数     : P_WIDTH, P_REMOTE_LATENCY, P_CORE_BUSINESS
//  验证状态 : 见 docs/validation.md
//=============================================================================
module dsp_camera_modes #(
    parameter integer P_WIDTH = 1280,
    parameter integer P_REMOTE_LATENCY = 6,
    parameter integer P_CORE_BUSINESS = 0
) (
    input wire I_clk_pix,
    input wire I_rst_n,
    input wire [4:0] I_mode,
    input wire signed [8:0] I_brightness,
    input wire [7:0] I_sat_factor,
    input wire [7:0] I_bin_th,
    input wire [7:0] I_cartoon_th,
    input wire I_vsync,
    input wire I_hsync,
    input wire I_de,
    input wire I_user,
    input wire I_last,
    input wire [23:0] I_video,
    input wire I_rx_valid,
    input wire [15:0] I_rx_length,
    output wire [10:0] O_rx_address,
    input wire [7:0] I_rx_data,
    output wire O_rx_release,
    input wire I_tx_ready,
    output wire O_tx_write,
    output wire [10:0] O_tx_address,
    output wire [7:0] O_tx_data,
    output wire O_tx_commit,
    output wire O_vsync,
    output wire O_hsync,
    output wire O_de,
    output wire O_user,
    output wire O_last,
    output wire [23:0] O_video,
    output wire O_link_ok,
    output wire O_remote_used
);
    generate
    if (P_CORE_BUSINESS != 0) begin : g_core_business
        // Point effects retain one pixel per clock. Neighbourhood effects use
        // the historical four-pixel groups and pair-averaged line windows.
        // All paths and sync markers have ten register stages of latency.
        wire [23:0] W_point_video;
        wire [23:0] W_contrast_video;
        wire [95:0] W_group_video;
        wire W_group_valid;
        wire W_neighbourhood_mode;
        reg [1:0] S_pack_phase;
        reg [71:0] S_pack_pixels;
        reg [95:0] S_pack_data;
        reg S_pack_valid;
        reg S_pack_user;
        reg S_pack_start;
        reg S_pack_last;
        reg [71:0] S_unpack_pixels;
        reg [1:0] S_unpack_remaining;
        reg [23:0] S_neighbourhood_video;
        reg [23:0] S_point_pad [0:6];
        reg [23:0] S_contrast_pad [0:7];
        reg [4:0] S_sync_pad [0:9];
        integer S_pad_index;

        dsp_mode_filter #(
            .P_RUNTIME_PARAMETERS(1), .P_TIER1(0), .P_WIDTH(P_WIDTH)
        ) u_point_filter (
            .I_clk(I_clk_pix), .I_rst_n(I_rst_n), .I_mode(I_mode),
            .I_brightness(I_brightness), .I_sat_factor(I_sat_factor),
            .I_bin_th(I_bin_th), .I_cartoon_th(I_cartoon_th),
            .I_vsync(I_vsync), .I_hsync(I_hsync), .I_de(I_de),
            .I_user(I_user), .I_last(I_last), .I_video(I_video),
            .O_vsync(), .O_hsync(), .O_de(), .O_user(), .O_last(),
            .O_video(W_point_video), .O_mask()
        );

        // P_WIDTH is the production 1280 pixels, an integral number of groups.
        // Capture SOF on the first pixel and LAST on the fourth pixel of a group.
        always @(posedge I_clk_pix or negedge I_rst_n) begin
            if (!I_rst_n) begin
                S_pack_phase <= 2'd0;
                S_pack_pixels <= 72'd0;
                S_pack_data <= 96'd0;
                S_pack_valid <= 1'b0;
                S_pack_user <= 1'b0;
                S_pack_start <= 1'b0;
                S_pack_last <= 1'b0;
            end else begin
                S_pack_valid <= 1'b0;
                if (I_de) begin
                    if (I_user) begin
                        S_pack_phase <= 2'd1;
                        S_pack_pixels <= {48'd0, I_video};
                        S_pack_user <= 1'b1;
                    end else if (S_pack_phase == 2'd3) begin
                        S_pack_phase <= 2'd0;
                        S_pack_data <= {S_pack_pixels, I_video};
                        S_pack_valid <= 1'b1;
                        S_pack_start <= S_pack_user;
                        S_pack_last <= I_last;
                        S_pack_user <= 1'b0;
                    end else begin
                        S_pack_phase <= S_pack_phase + 2'd1;
                        S_pack_pixels <= {S_pack_pixels[47:0], I_video};
                    end
                end
            end
        end

        dsp_camera_neighbourhood #(
            .IMG_WIDTH(P_WIDTH), .IMG_HEIGHT(720), .EDGE_RGB_TH(8'd80)
        ) u_neighbourhood (
            .I_clk_csi(I_clk_pix), .I_rst_n(I_rst_n), .I_mode(I_mode),
            .I_brightness(I_brightness), .I_sat_factor(I_sat_factor),
            .I_bin_th(I_bin_th), .I_cartoon_th(I_cartoon_th),
            .I_tlast(S_pack_last), .I_tuser(S_pack_start),
            .I_tdata(S_pack_data), .I_tvalid(S_pack_valid), .I_tready(1'b1),
            .O_tlast(), .O_tuser(), .O_tdata(W_group_video),
            .O_tvalid(W_group_valid), .O_tready()
        );

        always @(posedge I_clk_pix or negedge I_rst_n) begin
            if (!I_rst_n) begin
                S_unpack_pixels <= 72'd0;
                S_unpack_remaining <= 2'd0;
                S_neighbourhood_video <= 24'd0;
            end else if (W_group_valid) begin
                S_neighbourhood_video <= W_group_video[95:72];
                S_unpack_pixels <= W_group_video[71:0];
                S_unpack_remaining <= 2'd3;
            end else if (S_unpack_remaining != 2'd0) begin
                S_neighbourhood_video <= S_unpack_pixels[71:48];
                S_unpack_pixels <= {S_unpack_pixels[47:0], 24'd0};
                S_unpack_remaining <= S_unpack_remaining - 2'd1;
            end
        end

        alg_contrast_stretch #(
            .IMG_WIDTH(P_WIDTH), .IMG_HEIGHT(720), .SAMPLE_DIV(8),
            .PERCENT_LO(1), .PERCENT_HI(99), .STRETCH_MIN_SPAN(24)
        ) u_contrast (
            .I_clk_pix(I_clk_pix), .I_rst_n(I_rst_n), .I_enable(I_mode == 5'd19),
            .I_vsync(I_vsync), .I_hsync(I_hsync), .I_de(I_de),
            .I_user(I_user), .I_last(I_last), .I_video(I_video),
            .O_vsync(), .O_hsync(), .O_de(), .O_user(), .O_last(),
            .O_video(W_contrast_video)
        );

        always @(posedge I_clk_pix or negedge I_rst_n) begin
            if (!I_rst_n) begin
                for (S_pad_index = 0; S_pad_index < 7; S_pad_index = S_pad_index + 1)
                    S_point_pad[S_pad_index] <= 24'd0;
                for (S_pad_index = 0; S_pad_index < 8; S_pad_index = S_pad_index + 1)
                    S_contrast_pad[S_pad_index] <= 24'd0;
                for (S_pad_index = 0; S_pad_index < 10; S_pad_index = S_pad_index + 1)
                    S_sync_pad[S_pad_index] <= 5'd0;
            end else begin
                S_point_pad[0] <= W_point_video;
                for (S_pad_index = 1; S_pad_index < 7; S_pad_index = S_pad_index + 1)
                    S_point_pad[S_pad_index] <= S_point_pad[S_pad_index-1];
                S_contrast_pad[0] <= W_contrast_video;
                for (S_pad_index = 1; S_pad_index < 8; S_pad_index = S_pad_index + 1)
                    S_contrast_pad[S_pad_index] <= S_contrast_pad[S_pad_index-1];
                S_sync_pad[0] <= {I_vsync, I_hsync, I_de, I_user, I_last};
                for (S_pad_index = 1; S_pad_index < 10; S_pad_index = S_pad_index + 1)
                    S_sync_pad[S_pad_index] <= S_sync_pad[S_pad_index-1];
            end
        end

        assign W_neighbourhood_mode = ((I_mode >= 5'd9) && (I_mode <= 5'd11)) ||
                                      ((I_mode >= 5'd13) && (I_mode <= 5'd18));
        assign O_video = (I_mode == 5'd19) ? S_contrast_pad[7] :
                         (W_neighbourhood_mode ? S_neighbourhood_video : S_point_pad[6]);
        assign {O_vsync, O_hsync, O_de, O_user, O_last} = S_sync_pad[9];
        assign O_rx_address = 11'd0;
        assign O_rx_release = 1'b0;
        assign O_tx_write = 1'b0;
        assign O_tx_address = 11'd0;
        assign O_tx_data = 8'd0;
        assign O_tx_commit = 1'b0;
        assign O_link_ok = 1'b0;
        assign O_remote_used = 1'b0;
    end else begin : g_full_effects

        wire W_remote_edges;
        wire W_remote_masks;
        wire W_remote_mode;
        wire [15:0] W_red_luma;
        wire [15:0] W_green_luma;
        wire [15:0] W_blue_luma;
        wire [15:0] W_luma_sum;
        wire [7:0] W_luma;
        wire [7:0] W_cartoon;
        wire [7:0] W_remote_pixel;
        wire [23:0] W_local_video;
        wire [7:0] W_return_pixel;
        wire W_return_styled;
        wire [23:0] W_return_rgb;
        wire [23:0] W_contrast_video;
        reg [23:0] S_remote_first_pix;
        reg [23:0] S_remote_second_pix;
        reg S_styled_first_pix;
        reg S_styled_second_pix;
        reg [23:0] S_contrast_delay_pix;
        wire W_contrast_mode;
        wire W_remote_select;
        wire [23:0] W_noncontrast_video;
        wire [23:0] W_mode_video;

        assign W_remote_edges = (I_mode >= 5'd9) && (I_mode <= 5'd11);
        assign W_remote_masks = (I_mode >= 5'd13) && (I_mode <= 5'd18);
        assign W_remote_mode = W_remote_edges || W_remote_masks;
        assign W_red_luma = I_video[23:16] * 16'd77;
        assign W_green_luma = I_video[15:8] * 16'd150;
        assign W_blue_luma = I_video[7:0] * 16'd29;
        assign W_luma_sum = W_red_luma + W_green_luma + W_blue_luma;
        assign W_luma = W_luma_sum[15:8];
        // Mode 18 transports RGB332 so colour and edge processing describe one row.
        assign W_cartoon = {I_video[23:21], I_video[15:13], I_video[7:6]};
        assign W_remote_pixel = (I_mode == 5'd18) ? W_cartoon : W_luma;
        assign W_return_rgb = (I_mode == 5'd18) ?
                             {W_return_pixel[7:5], 5'd0,
                              W_return_pixel[4:2], 5'd0,
                              W_return_pixel[1:0], 6'd0} :
                             {W_return_pixel, W_return_pixel, W_return_pixel};

        dsp_mode_filter #(.P_RUNTIME_PARAMETERS(1)) u_local_filter (
            .I_clk(I_clk_pix), .I_rst_n(I_rst_n), .I_mode(I_mode),
            .I_brightness(I_brightness), .I_sat_factor(I_sat_factor),
            .I_bin_th(I_bin_th), .I_cartoon_th(I_cartoon_th),
            .I_vsync(I_vsync), .I_hsync(I_hsync), .I_de(I_de),
            .I_user(I_user), .I_last(I_last), .I_video(I_video),
            .O_vsync(O_vsync), .O_hsync(O_hsync), .O_de(O_de),
            .O_user(O_user), .O_last(O_last), .O_video(W_local_video),
            .O_mask()
        );

        ph1p_line_bridge #(
            .IMG_W(P_WIDTH), .P_SLOT(8), .P_LAT(P_REMOTE_LATENCY),
            .P_VALIDATE_FRAME(1), .P_REMOTE_ONLY(1)
        ) u_remote_lines (
            .I_clk(I_clk_pix), .I_rst_n(I_rst_n),
            .I_rx_valid(I_rx_valid), .I_rx_len(I_rx_length),
            .O_rx_addr(O_rx_address), .I_rx_rdata(I_rx_data),
            .O_rx_release(O_rx_release), .I_tx_ready(I_tx_ready),
            .O_tx_we(O_tx_write), .O_tx_addr(O_tx_address),
            .O_tx_wdata(O_tx_data), .O_tx_commit(O_tx_commit),
            .I_vsync(I_vsync), .I_de(I_de), .I_user(I_user),
            .I_y(W_remote_pixel), .I_mode(I_mode), .I_enable(W_remote_mode),
            .O_vsync(), .O_de(), .O_user(), .O_y(W_return_pixel),
            .O_styled(W_return_styled), .O_link_ok(O_link_ok), .O_out_line()
        );

        alg_contrast_stretch #(
            .IMG_WIDTH(P_WIDTH), .IMG_HEIGHT(720), .SAMPLE_DIV(8),
            .PERCENT_LO(1), .PERCENT_HI(99), .STRETCH_MIN_SPAN(24)
        ) u_contrast (
            .I_clk_pix(I_clk_pix), .I_rst_n(I_rst_n), .I_enable(1'b1),
            .I_vsync(I_vsync), .I_hsync(I_hsync), .I_de(I_de),
            .I_user(I_user), .I_last(I_last), .I_video(I_video),
            .O_vsync(), .O_hsync(), .O_de(), .O_user(), .O_last(),
            .O_video(W_contrast_video)
        );

        // Local filter latency is three clocks. Remote read is one; contrast is two.
        always @(posedge I_clk_pix or negedge I_rst_n) begin
            if (!I_rst_n) begin
                S_remote_first_pix <= 24'd0;
                S_remote_second_pix <= 24'd0;
                S_styled_first_pix <= 1'b0;
                S_styled_second_pix <= 1'b0;
                S_contrast_delay_pix <= 24'd0;
            end else begin
                S_remote_first_pix <= W_return_rgb;
                S_remote_second_pix <= S_remote_first_pix;
                S_styled_first_pix <= W_return_styled;
                S_styled_second_pix <= S_styled_first_pix;
                S_contrast_delay_pix <= W_contrast_video;
            end
        end

        assign W_contrast_mode = (I_mode == 5'd19);
        assign W_remote_select = W_remote_mode && S_styled_second_pix;
        assign W_noncontrast_video = W_remote_select ? S_remote_second_pix : W_local_video;
        assign W_mode_video = W_contrast_mode ? S_contrast_delay_pix : W_noncontrast_video;
        assign O_video = W_mode_video;
        assign O_remote_used = W_remote_select && O_de;
    end
    endgenerate
endmodule
