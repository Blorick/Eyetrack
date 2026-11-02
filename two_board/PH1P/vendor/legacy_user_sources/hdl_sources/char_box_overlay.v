// ============================================================================
// char_box_overlay.v  --  字符框叠加（在视频流上画 7 个红色字符框）
//
// 位置：串接在 plate_box_overlay 之后、hdmi_tx 之前，整体只引入 1 拍流水延迟。
//   - 输入：plate_box_overlay 输出的 RGB888 视频流（de/vsync + 24bit 数据）
//   - 输入：char_locate 输出的 7 个字符左右边界 + 字符区上下边界（1 帧延迟，自动对齐）
//   - 输出：叠加了红色字符框的视频流（de/vsync/data 三者同步延迟 1 拍）
//
// 说明：对每个字符画一个红色矩形框（线宽 BORDER_W），i_detected=0 时直接透传。
// ============================================================================
`timescale 1ns / 1ps

module char_box_overlay #(
    parameter H_ACTIVE      = 1280,
    parameter V_ACTIVE      = 720,
    parameter BORDER_W      = 2,
    parameter [23:0] BOX_COLOR = 24'hFF0000     // 红色 {R,G,B}
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [23:0]  i_data,
    input  wire         i_de,
    input  wire         i_vsync,
    input  wire [10:0]  i_char_up,
    input  wire [10:0]  i_char_down,
    input  wire [10:0]  i_c1_left,  input wire [10:0] i_c1_right,
    input  wire [10:0]  i_c2_left,  input wire [10:0] i_c2_right,
    input  wire [10:0]  i_c3_left,  input wire [10:0] i_c3_right,
    input  wire [10:0]  i_c4_left,  input wire [10:0] i_c4_right,
    input  wire [10:0]  i_c5_left,  input wire [10:0] i_c5_right,
    input  wire [10:0]  i_c6_left,  input wire [10:0] i_c6_right,
    input  wire [10:0]  i_c7_left,  input wire [10:0] i_c7_right,
    input  wire         i_detected,
    // 2026-10-05 P0-1a：逐字符有效性（bit k = 第 k+1 个字符框有效）。
    // 由 char_locate.o_char_valid 驱动；无效字符不再画框，
    // 这样上板时"画了几个框"就等于"找到了几个字符"，指示不会被误导。
    input  wire [6:0]   i_char_valid,
    output wire [23:0]  o_data,
    output reg          o_de,
    output reg          o_vsync
);

    // ---- 输入像素坐标；输出数据、DE、VS 同拍寄存 ----
    reg [23:0] d_r;
    reg [10:0] x;
    reg [9:0]  y;
    wire on_any;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_r  <= 24'd0;
            o_de <= 1'b0;
            o_vsync <= 1'b0;
            x    <= 11'd0;
            y    <= 10'd0;
        end else begin
            // x/y 在本拍采样前指向 i_data，因此字符框与进入的像素同坐标。
            d_r <= (i_detected && i_de && on_any) ? BOX_COLOR : i_data;
            o_de <= i_de;
            o_vsync <= i_vsync;
            if (i_vsync) begin
                x <= 11'd0;
                y <= 10'd0;
            end else if (i_de) begin
                if (x == H_ACTIVE-1) begin
                    x <= 11'd0;
                    y <= y + 1'b1;
                end else
                    x <= x + 1'b1;
            end
        end
    end

    // ---- 字符框边框判定（组合逻辑） ----
    wire in_char_band = (y >= i_char_up) && (y <= i_char_down);
    wire near_top    = (y >= i_char_up)    && (y <  i_char_up    + BORDER_W);
    wire near_bottom = (y <= i_char_down)  && ((i_char_down - y) < BORDER_W);

    // 每个字符的左右边框
    wire on_c1 = in_char_band && i_char_valid[0] && (x >= i_c1_left) && (x <= i_c1_right) &&
                 (near_top || near_bottom ||
                   (x < i_c1_left  + BORDER_W) || ((i_c1_right - x) < BORDER_W));
    wire on_c2 = in_char_band && i_char_valid[1] && (x >= i_c2_left) && (x <= i_c2_right) &&
                 (near_top || near_bottom ||
                   (x < i_c2_left  + BORDER_W) || ((i_c2_right - x) < BORDER_W));
    wire on_c3 = in_char_band && i_char_valid[2] && (x >= i_c3_left) && (x <= i_c3_right) &&
                 (near_top || near_bottom ||
                   (x < i_c3_left  + BORDER_W) || ((i_c3_right - x) < BORDER_W));
    wire on_c4 = in_char_band && i_char_valid[3] && (x >= i_c4_left) && (x <= i_c4_right) &&
                 (near_top || near_bottom ||
                   (x < i_c4_left  + BORDER_W) || ((i_c4_right - x) < BORDER_W));
    wire on_c5 = in_char_band && i_char_valid[4] && (x >= i_c5_left) && (x <= i_c5_right) &&
                 (near_top || near_bottom ||
                   (x < i_c5_left  + BORDER_W) || ((i_c5_right - x) < BORDER_W));
    wire on_c6 = in_char_band && i_char_valid[5] && (x >= i_c6_left) && (x <= i_c6_right) &&
                 (near_top || near_bottom ||
                   (x < i_c6_left  + BORDER_W) || ((i_c6_right - x) < BORDER_W));
    wire on_c7 = in_char_band && i_char_valid[6] && (x >= i_c7_left) && (x <= i_c7_right) &&
                 (near_top || near_bottom ||
                   (x < i_c7_left  + BORDER_W) || ((i_c7_right - x) < BORDER_W));

    assign on_any = on_c1 || on_c2 || on_c3 || on_c4 || on_c5 || on_c6 || on_c7;

    assign o_data = d_r;

endmodule
