// ============================================================================
// plate_box_overlay.v  --  车牌外接框叠加（在视频流上画绿色矩形框）
//
// 位置：串接在 hdmi_mixer 之后、hdmi_tx 之前，整体只引入 1 拍流水延迟。
//   - 输入：hdmi_mixer 输出的 RGB888 视频流（de/vsync + 24bit 数据）
//   - 输入：plate_locate 输出的车牌外接矩形（1 帧延迟，故与视频流自动对齐）
//   - 输出：叠加了绿色边框的视频流（de/vsync/data 三者同步延迟 1 拍）
//
// 说明：框画在车牌外接矩形的边界上（线宽 BORDER_W）。i_detected=0 时直接透传。
// ============================================================================
`timescale 1ns / 1ps

module plate_box_overlay #(
    parameter H_ACTIVE      = 1280,
    parameter V_ACTIVE      = 720,
    parameter BORDER_W      = 2,                 // 边框线宽（像素）
    parameter [23:0] BOX_COLOR = 24'h00FF00     // 绿色 {R,G,B}
)(
    input  wire         clk,
    input  wire         rst_n,
    input  wire [23:0]  i_data,
    input  wire         i_de,
    input  wire         i_vsync,
    input  wire [10:0]  i_up,
    input  wire [10:0]  i_down,
    input  wire [10:0]  i_left,
    input  wire [10:0]  i_right,
    input  wire         i_detected,
    output wire [23:0]  o_data,
    output reg          o_de,
    output reg          o_vsync
);

    // ---- 输入像素坐标；输出数据、DE、VS 同拍寄存 ----
    reg [23:0] d_r;
    reg [10:0] x;
    reg [9:0]  y;
    wire on_border;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d_r  <= 24'd0;
            o_de <= 1'b0;
            o_vsync <= 1'b0;
            x    <= 11'd0;
            y    <= 10'd0;
        end else begin
            // x/y 在本拍采样前指向 i_data，因此边框替换不会错移一个像素。
            d_r <= (i_detected && i_de && on_border) ? BOX_COLOR : i_data;
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

    // ---- 边框判定（组合逻辑） ----
    wire in_box = (y >= i_up) && (y <= i_down) && (x >= i_left) && (x <= i_right);
    wire near_top    = (y >= i_up)    && (y <  i_up    + BORDER_W);
    wire near_bottom = (y <= i_down)  && ((i_down - y) < BORDER_W);
    wire near_left   = (x >= i_left)  && (x <  i_left  + BORDER_W);
    wire near_right  = (x <= i_right) && ((i_right - x) < BORDER_W);
    assign on_border = in_box && (near_top || near_bottom || near_left || near_right);

    assign o_data = d_r;

endmodule
