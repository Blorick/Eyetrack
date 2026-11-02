`timescale 1ns/1ps
//=============================================================================
//  eg4d_mode_pipeline -- v2.5 integrated source
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk
//  复位     : 按本模块接口定义
//  参数     : 见模块参数
//  验证状态 : 见 docs/validation.md
//=============================================================================
//=============================================================================
// eg4d_mode_pipeline -- v2.5 mode engine endpoint
//
// The primary filter supplies the one-line 3x3 luma/RGB332 results.  Mode 17
// is intentionally a second pass: its first-pass binary mask is fed through
// eg4d_mode17_majority, which consumes the next mask row before returning the
// centre row.  The wrapper keeps that extra line latency local and exposes one
// raster stream to eg4d_line_adapter.
//=============================================================================
module eg4d_mode_pipeline #(
    parameter integer IMG_W        = 1280,
    parameter [7:0]   EDGE_RGB_TH  = 8'd80,
    parameter [7:0]   CARTOON_OFF  = 8'd0
) (
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire [4:0] I_mode,
    input  wire [7:0] I_bin_th,
    input  wire [7:0] I_cartoon_th,
    input  wire       I_vsync,
    input  wire       I_hsync,
    input  wire       I_de,
    input  wire       I_user,
    input  wire       I_last,
    input  wire [7:0] I_y,
    output wire [4:0] O_mode,
    output wire       O_vsync,
    output wire       O_hsync,
    output wire       O_de,
    output wire       O_user,
    output wire       O_last,
    output wire [7:0] O_y,
    output wire       O_edge,
    output wire       O_mask,
    output wire       O_window_valid
);

    wire [4:0] W_filter_mode;
    wire       W_filter_vsync;
    wire       W_filter_hsync;
    wire       W_filter_de;
    wire       W_filter_user;
    wire       W_filter_last;
    wire [7:0] W_filter_y;
    wire       W_filter_edge;
    wire       W_filter_mask;
    wire       W_filter_window_valid;

    eg4d_mode_filter #(
        .IMG_W       (IMG_W),
        .EDGE_RGB_TH (EDGE_RGB_TH),
        .CARTOON_OFF (CARTOON_OFF)
    ) u_primary_filter (
        .I_clk       (I_clk),
        .I_rst_n     (I_rst_n),
        .I_mode      (I_mode),
        .I_bin_th    (I_bin_th),
        .I_cartoon_th(I_cartoon_th),
        .I_vsync     (I_vsync),
        .I_hsync     (I_hsync),
        .I_de        (I_de),
        .I_user      (I_user),
        .I_last      (I_last),
        .I_y         (I_y),
        .O_mode      (W_filter_mode),
        .O_vsync     (W_filter_vsync),
        .O_hsync     (W_filter_hsync),
        .O_de        (W_filter_de),
        .O_user      (W_filter_user),
        .O_last      (W_filter_last),
        .O_y         (W_filter_y),
        .O_edge      (W_filter_edge),
        .O_mask      (W_filter_mask),
        .O_window_valid(W_filter_window_valid)
    );

    wire       W_majority_vsync;
    wire       W_majority_hsync;
    wire       W_majority_de;
    wire       W_majority_user;
    wire       W_majority_last;
    wire       W_majority_mask;
    wire       W_majority_valid;

    eg4d_mode17_majority #(
        .IMG_W (IMG_W)
    ) u_mode17_majority (
        .I_clk          (I_clk),
        .I_rst_n        (I_rst_n),
        .I_vsync        (W_filter_vsync),
        .I_hsync        (W_filter_hsync),
        .I_de           (W_filter_de),
        .I_user         (W_filter_user),
        .I_last         (W_filter_last),
        .I_mask         (W_filter_mask),
        .O_vsync        (W_majority_vsync),
        .O_hsync        (W_majority_hsync),
        .O_de           (W_majority_de),
        .O_user         (W_majority_user),
        .O_last         (W_majority_last),
        .O_mask         (W_majority_mask),
        .O_window_valid (W_majority_valid)
    );

    // Primary and second-pass streams share a three-clock endpoint delay.
    // Mode tags and all bypass controls belong to the same raster beat.
    reg [20:0] S_primary_pipe [0:2];
    integer S_pipe_index;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (S_pipe_index = 0; S_pipe_index < 3; S_pipe_index = S_pipe_index + 1)
                S_primary_pipe[S_pipe_index] <= 21'd0;
        end else begin
            S_primary_pipe[0] <= {W_filter_mode, W_filter_vsync, W_filter_hsync,
                W_filter_de, W_filter_user, W_filter_last, W_filter_y,
                W_filter_edge, W_filter_mask, W_filter_window_valid};
            S_primary_pipe[1] <= S_primary_pipe[0];
            S_primary_pipe[2] <= S_primary_pipe[1];
        end
    end
    wire [4:0] W_mode_aligned = S_primary_pipe[2][20:16];
    wire W_is_mode17 = (W_mode_aligned == 5'd17);
    assign O_mode = W_mode_aligned;
    assign O_vsync = W_is_mode17 ? W_majority_vsync : S_primary_pipe[2][15];
    assign O_hsync = W_is_mode17 ? W_majority_hsync : S_primary_pipe[2][14];
    assign O_de = W_is_mode17 ? W_majority_de : S_primary_pipe[2][13];
    assign O_user = W_is_mode17 ? W_majority_user : S_primary_pipe[2][12];
    assign O_last = W_is_mode17 ? W_majority_last : S_primary_pipe[2][11];
    assign O_y = W_is_mode17 ? {8{W_majority_mask}} : S_primary_pipe[2][10:3];
    assign O_edge = W_is_mode17 ? 1'b0 : S_primary_pipe[2][2];
    assign O_mask = W_is_mode17 ? W_majority_mask : S_primary_pipe[2][1];
    assign O_window_valid = W_is_mode17 ? W_majority_valid : S_primary_pipe[2][0];
endmodule
