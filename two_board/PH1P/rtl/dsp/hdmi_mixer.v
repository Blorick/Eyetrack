//======================================================================
// hdmi_mixer -- logo, mode label and native-resolution antialiased plate OSD
// Pixel domain: I_clk. Font/logo ROMs are synchronous, one-cycle reads.
// HDMI data and VS/HS/DE keep the existing four-register input pipeline.
//======================================================================
module hdmi_mixer #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1280,
    parameter IMG_HEIGHT = 720,
    parameter [11:0] MODE_OSD_X = IMG_WIDTH - 224 - 12,
    parameter [11:0] MODE_OSD_Y = 12'd8,
    parameter [11:0] RES_OSD_X = (IMG_WIDTH - 336) / 2,
    parameter [11:0] RES_OSD_Y = 12'd144
)(
    input wire        I_clk,
    input wire        I_rst_n,
    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,
    input wire [4:0]  I_alg_mode,
    input wire [55:0] I_res,
    input wire [3:0]  I_res_n,
    input wire        I_res_valid,
    output wire       O_video_rd_en,
    input wire [23:0] I_video_rd_data,
    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg [23:0] O_hdmi_data
);
    localparam [11:0] LOGO_W = 12'd160;
    localparam [11:0] LOGO_H = 12'd160;
    localparam [11:0] MODE_W = 12'd224;
    localparam [11:0] MODE_H = 12'd24;
    localparam [11:0] RES_W = 12'd336;
    localparam [11:0] RES_H = 12'd72;
    localparam [23:0] COLOR_MODE = 24'h00ff80;
    localparam [23:0] COLOR_RES = 24'hffd000;

    reg S_video_vsync_1d, S_video_vsync_2d;
    reg S_video_hsync_1d, S_video_hsync_2d;
    reg S_video_de_1d, S_video_de_2d;
    reg [23:0] S_video_data_1d, S_video_data_2d;
    reg [11:0] S_x, S_y;
    reg [11:0] S_x_1d, S_x_2d, S_y_1d, S_y_2d;
    reg [4:0] S_alg_s1, S_alg_s2;
    reg [14:0] S_logo_addr;
    wire [24:0] S_logo_pixel;
    wire [31:0] S_mode_row_alpha;
    wire [95:0] S_plate_row_alpha;

    reg [7:0] S_pf_char;
    reg [4:0] S_pf_row;
    reg [3:0] S_req_pf_col;
    reg S_req_logo_region;
    reg S_req_char_region;
    reg S_req_plate_region;
    reg [11:0] S_mode_dx, S_mode_dy;
    reg [8:0] S_res_dx;
    reg [6:0] S_res_dy;
    reg [2:0] S_plate_cell;
    reg [7:0] S_plate_char;
    reg [6:0] S_plate_row;
    reg [5:0] S_req_plate_col;

    reg S_meta_vsync_q, S_meta_hsync_q, S_meta_de_q;
    reg [23:0] S_meta_background_q;
    reg S_meta_logo_region_q, S_meta_char_region_q;
    reg S_meta_plate_region_q;
    reg [3:0] S_meta_pf_col_q;
    reg [5:0] S_meta_plate_col_q;

    wire W_logo_hit = S_meta_logo_region_q && S_logo_pixel[24];
    wire [1:0] W_mode_alpha =
        S_mode_row_alpha[(4'd15 - S_meta_pf_col_q) * 2 +: 2];
    wire [1:0] W_plate_alpha =
        S_plate_row_alpha[(6'd47 - S_meta_plate_col_q) * 2 +: 2];

    assign O_video_rd_en = I_video_de;

    anlogic_logo_rom_sync u_anlogic_logo_rom (
        .I_clk(I_clk), .I_addr(S_logo_addr), .O_pixel(S_logo_pixel)
    );
    mode_font_rom_sync u_mode_font (
        .I_clk(I_clk), .I_char(S_pf_char), .I_row(S_pf_row),
        .O_row_alpha(S_mode_row_alpha)
    );
    plate_font_rom_sync u_plate_font (
        .I_clk(I_clk), .I_char(S_plate_char), .I_row(S_plate_row),
        .O_row_alpha(S_plate_row_alpha)
    );

    // Four coverage levels: clear, 1/4, 3/4, solid. Shift/add blending
    // rounds to the nearest channel value without a multiplier or divider.
    function [7:0] blend_channel;
        input [7:0] background;
        input [7:0] foreground;
        input [1:0] alpha;
        reg [9:0] weighted;
        begin
            weighted = 10'd0;
            case (alpha)
                2'd0: blend_channel = background;
                2'd1: begin
                    weighted = {2'b00, background} +
                               {1'b0, background, 1'b0} +
                               {2'b00, foreground} + 10'd2;
                    blend_channel = weighted[9:2];
                end
                2'd2: begin
                    weighted = {2'b00, background} +
                               {2'b00, foreground} +
                               {1'b0, foreground, 1'b0} + 10'd2;
                    blend_channel = weighted[9:2];
                end
                default: blend_channel = foreground;
            endcase
        end
    endfunction

    // Fourteen cells include MODE: and the longest nine-letter mode names.
    function [7:0] mode_row_char;
        input [3:0] cidx;
        input [4:0] mode;
        reg [111:0] row;
        begin
            case (mode)
                5'd0: row = "MODE:ORIGINAL ";
                5'd1: row = "MODE:GRAY     ";
                5'd2: row = "MODE:BINARY   ";
                5'd3: row = "MODE:NEGATIVE ";
                5'd4: row = "MODE:RED      ";
                5'd5: row = "MODE:POSTER   ";
                5'd6: row = "MODE:SEPIA    ";
                5'd7: row = "MODE:BRIGHT   ";
                5'd8: row = "MODE:SATURATE ";
                5'd9: row = "MODE:SOBEL    ";
                5'd10: row = "MODE:SOBEL INV";
                5'd11: row = "MODE:EMBOSS   ";
                5'd12: row = "MODE:HEAT     ";
                5'd13: row = "MODE:ERODE    ";
                5'd14: row = "MODE:DILATE   ";
                5'd15: row = "MODE:MORPH    ";
                5'd16: row = "MODE:BINSOBEL ";
                5'd17: row = "MODE:FILLHOLE ";
                5'd18: row = "MODE:CARTOON  ";
                5'd19: row = "MODE:AUTO CONT";
                default: row = "              ";
            endcase
            if (cidx < 4'd14)
                mode_row_char = row[(7'd13 - {3'b000, cidx}) * 7'd8 +: 8];
            else
                mode_row_char = 8'h20;
        end
    endfunction

    wire [18:0] W_logo_row_off =
        ({7'b0, S_y_1d} << 7) + ({7'b0, S_y_1d} << 5);
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_logo_addr <= 15'd0;
        else if ((S_x_1d < LOGO_W) && (S_y_1d < LOGO_H))
            S_logo_addr <= W_logo_row_off[14:0] + S_x_1d;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_alg_s1 <= 5'd0;
            S_alg_s2 <= 5'd0;
            S_x <= 12'd0;
            S_y <= 12'd0;
            S_x_1d <= 12'd0;
            S_x_2d <= 12'd0;
            S_y_1d <= 12'd0;
            S_y_2d <= 12'd0;
        end else begin
            S_alg_s1 <= I_alg_mode;
            S_alg_s2 <= S_alg_s1;
            if (I_video_user) begin
                S_x <= 12'd0;
                S_y <= 12'd0;
            end else if (I_video_de) begin
                if (I_video_last) begin
                    S_x <= 12'd0;
                    S_y <= S_y + 12'd1;
                end else
                    S_x <= S_x + 12'd1;
            end
            S_x_1d <= S_x;
            S_x_2d <= S_x_1d;
            S_y_1d <= S_y;
            S_y_2d <= S_y_1d;
        end
    end

    // Requests and the next-stage metadata refer to the same source pixel.
    // The mode and plate regions do not overlap at their default positions.
    always @(*) begin
        S_pf_char = 8'd0;
        S_pf_row = 5'd0;
        S_req_pf_col = 4'd0;
        S_req_logo_region = 1'b0;
        S_req_char_region = 1'b0;
        S_req_plate_region = 1'b0;
        S_mode_dx = 12'd0;
        S_mode_dy = 12'd0;
        S_res_dx = 9'd0;
        S_res_dy = 7'd0;
        S_plate_cell = 3'd0;
        S_plate_char = 8'd0;
        S_plate_row = 7'd0;
        S_req_plate_col = 6'd0;
        if (S_video_de_2d) begin
            S_req_logo_region = (S_x_2d < LOGO_W) && (S_y_2d < LOGO_H);
            if ((S_x_2d >= MODE_OSD_X) && (S_x_2d < MODE_OSD_X + MODE_W) &&
                (S_y_2d >= MODE_OSD_Y) && (S_y_2d < MODE_OSD_Y + MODE_H)) begin
                S_mode_dx = S_x_2d - MODE_OSD_X;
                S_mode_dy = S_y_2d - MODE_OSD_Y;
                S_req_char_region = 1'b1;
                S_pf_char = mode_row_char(S_mode_dx[7:4], S_alg_s2);
                S_pf_row = S_mode_dy[4:0];
                S_req_pf_col = S_mode_dx[3:0];
            end else if (I_res_valid &&
                         (S_x_2d >= RES_OSD_X) && (S_x_2d < RES_OSD_X + RES_W) &&
                         (S_y_2d >= RES_OSD_Y) && (S_y_2d < RES_OSD_Y + RES_H)) begin
                S_res_dx = S_x_2d - RES_OSD_X;
                S_res_dy = S_y_2d - RES_OSD_Y;
                if (S_res_dx >= 9'd288) begin
                    S_plate_cell = 3'd6;
                    S_req_plate_col = S_res_dx - 9'd288;
                end else if (S_res_dx >= 9'd240) begin
                    S_plate_cell = 3'd5;
                    S_req_plate_col = S_res_dx - 9'd240;
                end else if (S_res_dx >= 9'd192) begin
                    S_plate_cell = 3'd4;
                    S_req_plate_col = S_res_dx - 9'd192;
                end else if (S_res_dx >= 9'd144) begin
                    S_plate_cell = 3'd3;
                    S_req_plate_col = S_res_dx - 9'd144;
                end else if (S_res_dx >= 9'd96) begin
                    S_plate_cell = 3'd2;
                    S_req_plate_col = S_res_dx - 9'd96;
                end else if (S_res_dx >= 9'd48) begin
                    S_plate_cell = 3'd1;
                    S_req_plate_col = S_res_dx - 9'd48;
                end else begin
                    S_req_plate_col = S_res_dx[5:0];
                end
                S_req_plate_region = 1'b1;
                S_plate_char = I_res[S_plate_cell * 8 +: 8];
                S_plate_row = S_res_dy;
            end
        end
    end

    // I_res_n is retained for the upstream seven-character tuple interface.
    // The display has seven fixed cells, matching the previous valid behavior.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d <= 1'b0;
            S_video_de_2d <= 1'b0;
            S_video_data_1d <= 24'd0;
            S_video_data_2d <= 24'd0;
            S_meta_vsync_q <= 1'b0;
            S_meta_hsync_q <= 1'b0;
            S_meta_de_q <= 1'b0;
            S_meta_background_q <= 24'd0;
            S_meta_logo_region_q <= 1'b0;
            S_meta_char_region_q <= 1'b0;
            S_meta_plate_region_q <= 1'b0;
            S_meta_pf_col_q <= 4'd0;
            S_meta_plate_col_q <= 6'd0;
            O_hdmi_vsync <= 1'b0;
            O_hdmi_hsync <= 1'b0;
            O_hdmi_de <= 1'b0;
            O_hdmi_data <= 24'd0;
        end else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            S_video_hsync_1d <= I_video_hsync;
            S_video_hsync_2d <= S_video_hsync_1d;
            S_video_de_1d <= I_video_de;
            S_video_de_2d <= S_video_de_1d;
            S_video_data_1d <= I_video_rd_data;
            S_video_data_2d <= S_video_data_1d;
            S_meta_vsync_q <= S_video_vsync_2d;
            S_meta_hsync_q <= S_video_hsync_2d;
            S_meta_de_q <= S_video_de_2d;
            S_meta_background_q <= S_video_data_2d;
            S_meta_logo_region_q <= S_req_logo_region;
            S_meta_char_region_q <= S_req_char_region;
            S_meta_plate_region_q <= S_req_plate_region;
            S_meta_pf_col_q <= S_req_pf_col;
            S_meta_plate_col_q <= S_req_plate_col;
            O_hdmi_vsync <= S_meta_vsync_q;
            O_hdmi_hsync <= S_meta_hsync_q;
            O_hdmi_de <= S_meta_de_q;
            if (W_logo_hit)
                O_hdmi_data <= S_logo_pixel[23:0];
            else if (S_meta_char_region_q)
                O_hdmi_data <= {
                    blend_channel(S_meta_background_q[23:16], COLOR_MODE[23:16], W_mode_alpha),
                    blend_channel(S_meta_background_q[15:8], COLOR_MODE[15:8], W_mode_alpha),
                    blend_channel(S_meta_background_q[7:0], COLOR_MODE[7:0], W_mode_alpha)
                };
            else if (S_meta_plate_region_q)
                O_hdmi_data <= {
                    blend_channel(S_meta_background_q[23:16], COLOR_RES[23:16], W_plate_alpha),
                    blend_channel(S_meta_background_q[15:8], COLOR_RES[15:8], W_plate_alpha),
                    blend_channel(S_meta_background_q[7:0], COLOR_RES[7:0], W_plate_alpha)
                };
            else
                O_hdmi_data <= S_meta_background_q;
        end
    end
endmodule
