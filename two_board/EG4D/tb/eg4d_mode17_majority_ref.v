`timescale 1ns/1ps
//=============================================================================
//  eg4d_mode17_majority -- v2.5 integrated source
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk
//  复位     : 按本模块接口定义
//  参数     : 见模块参数
//  验证状态 : 见 docs/validation.md
//=============================================================================
// -----------------------------------------------------------------------------
// Module: eg4d_mode17_majority
// Function: second 3x3 pass for mode 17's smoothed binary region fill
// Domain: EG4D application/pixel clock, one pixel per active clock
// Input: the primary eg4d_mode_filter O_mask stream and its timing
// Output: a one-line-later majority mask stream
// -----------------------------------------------------------------------------
module eg4d_mode17_majority_ref #(
    parameter integer IMG_W = 1280
) (
    input  wire       I_clk,
    input  wire       I_rst_n,
    input  wire       I_vsync,
    input  wire       I_hsync,
    input  wire       I_de,
    input  wire       I_user,
    input  wire       I_last,
    input  wire       I_mask,
    output wire       O_vsync,
    output wire       O_hsync,
    output wire       O_de,
    output wire       O_user,
    output wire       O_last,
    output wire       O_mask,
    output wire       O_window_valid
);

    reg [1:0] S_row_idx;
    reg [15:0] S_line_count;
    reg [10:0] S_x;
    reg S_de_q, S_de_d;
    reg S_user_q, S_user_d;
    reg S_vsync_q, S_vsync_d;
    reg S_hsync_q, S_hsync_d;
    reg S_last_q, S_last_d;
    reg S_valid_q;
    reg S_valid_d;

    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg S_mem [0:3*IMG_W-1];

    wire [10:0] W_x = I_user ? 11'd0 : S_x;
    wire [1:0] W_cur_row = I_user ?
        (I_vsync ? 2'd0 : ((S_row_idx == 2'd2) ? 2'd0 : S_row_idx + 2'd1)) :
        S_row_idx;
    wire [15:0] W_line_now = I_user ?
        (I_vsync ? 16'd0 : (S_line_count + 16'd1)) : S_line_count;
    wire [1:0] W_mid_row = (W_cur_row == 2'd0) ? 2'd2 : W_cur_row - 2'd1;
    wire [1:0] W_top_row = (W_cur_row >= 2'd2) ? W_cur_row - 2'd2 : W_cur_row + 2'd1;

    // The arrays are read synchronously.  After this edge the *_read values
    // are the top/middle/right samples for the centre at x-1; the delayed
    // timing bits below describe that same centre.
    reg S_top_read, S_mid_read, S_bot_read;
    reg S_top_left, S_top_mid;
    reg S_mid_left, S_mid_mid;
    reg S_bot_left, S_bot_mid;

    wire [3:0] W_count =
        {3'd0, S_top_left} + {3'd0, S_top_mid} + {3'd0, S_top_read} +
        {3'd0, S_mid_left} + {3'd0, S_mid_mid} + {3'd0, S_mid_read} +
        {3'd0, S_bot_left} + {3'd0, S_bot_mid} + {3'd0, S_bot_read};
    // W_count is formed from the taps after the current edge.  The horizontal
    // centre is therefore the input sample from two clocks earlier, while
    // S_valid_q has exactly the same delay.  Using S_valid_d here would move
    // the valid flag one column to the right of the mask result.
    wire W_majority = S_valid_q && (W_count >= 4'd5);

    assign O_vsync = S_vsync_d;
    assign O_hsync = S_hsync_d;
    assign O_de = S_de_d;
    assign O_user = S_user_d && S_de_d;
    assign O_last = S_last_d;
    assign O_mask = W_majority;
    assign O_window_valid = S_valid_q && S_de_d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_row_idx <= 2'd0;
            S_line_count <= 16'd0;
            S_x <= 11'd0;
            S_de_q <= 1'b0; S_de_d <= 1'b0;
            S_user_q <= 1'b0; S_user_d <= 1'b0;
            S_vsync_q <= 1'b0; S_vsync_d <= 1'b0;
            S_hsync_q <= 1'b0; S_hsync_d <= 1'b0;
            S_last_q <= 1'b0; S_last_d <= 1'b0;
            S_valid_q <= 1'b0; S_valid_d <= 1'b0;
            S_top_read <= 1'b0; S_mid_read <= 1'b0; S_bot_read <= 1'b0;
            S_top_left <= 1'b0; S_top_mid <= 1'b0;
            S_mid_left <= 1'b0; S_mid_mid <= 1'b0;
            S_bot_left <= 1'b0; S_bot_mid <= 1'b0;
        end
        else begin
            // Read the two previous rows and write the current mask row.
            S_top_read <= S_mem[W_top_row * IMG_W + W_x];
            S_mid_read <= S_mem[W_mid_row * IMG_W + W_x];
            S_bot_read <= I_mask;
            if (I_de)
                S_mem[W_cur_row * IMG_W + W_x] <= I_mask;

            // Horizontal taps: right is the newly read/current sample, so
            // the combinational count after this edge is centred at x-1.
            S_top_left <= S_top_mid;
            S_top_mid <= S_top_read;
            S_mid_left <= S_mid_mid;
            S_mid_mid <= S_mid_read;
            S_bot_left <= S_bot_mid;
            S_bot_mid <= S_bot_read;

            S_de_q <= I_de;
            S_de_d <= S_de_q;
            S_user_q <= I_user;
            S_user_d <= S_user_q;
            S_vsync_q <= I_vsync;
            S_vsync_d <= S_vsync_q;
            S_hsync_q <= I_hsync;
            S_hsync_d <= S_hsync_q;
            S_last_q <= I_last;
            S_last_d <= S_last_q;

            // A complete 3-row mask window exists for input row >=2 and the
            // centre column is x-1 (therefore x>=2 for a right neighbour).
            S_valid_q <= I_de && (W_line_now >= 16'd2) && (W_x >= 11'd2);
            S_valid_d <= S_valid_q;

            if (I_user)
                S_x <= 11'd1;
            else if (I_de)
                S_x <= S_x + 11'd1;
            if (I_user) begin
                if (I_vsync)
                    S_row_idx <= 2'd0;
                else if (S_row_idx == 2'd2)
                    S_row_idx <= 2'd0;
                else
                    S_row_idx <= S_row_idx + 2'd1;

                if (I_vsync)
                    S_line_count <= 16'd0;
                else
                    S_line_count <= S_line_count + 16'd1;
            end
        end
    end

endmodule
