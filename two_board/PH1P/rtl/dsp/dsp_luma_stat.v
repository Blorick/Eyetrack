//=============================================================================
//  dsp_luma_stat  --  显示域帧亮度统计（1 像素/拍）
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk_pix（= S_hdmi_pixel_clk，75 MHz）
//  复位     : I_rst_n
//  参数     : P_WIDTH, P_HEIGHT
//  验证状态 : 仿真 待做 | 综合 待做 | 布线 待做 | 上板 未做
//=============================================================================
//=============================================================================
//  dsp_luma_stat  --  帧亮度统计（平均）
//
//  挂在 S_video_rd_data（24bit RGB888，1 像素/拍，video_out 已解包）上，
//  **纯观察者**：不接 ready、不影响原通路。给 AE 闭环（ae_auto_loop）用。
//
//  由 alg_luma_stat（v1.13，96bit = 4 像素/拍）适配为 1 像素/拍：
//  一帧总累加和不变（921600 像素 × lum），S_sum 仍 28 位，
//  O_y_avg = S_sum[27:20]（满帧全白 = 224，与 v1.13 同缩放）。
//
//  关键设计点（沿自 alg_luma_stat 上板踩过的坑）
//  ---------------------------------------------------------------------
//  1) 分帧按有效拍数（I_de），不依赖 I_tuser / I_vsync 的精确时序；
//     I_vsync 上升沿仅作**安全重同步**（DDR 读下溢黑帧填充时防止计数漂移，
//     误差限在一帧内）。
//  2) 灰度用 (r + 2g + b) >> 2 —— 移位加法，**不含乘法器**（0 DSP）。
//     早期用加权灰度 (77r+150g+29b)>>8 综合出 12 个 DSP，本器件 DSP 紧张。
//=============================================================================
module dsp_luma_stat #(
    parameter integer P_WIDTH  = 1280,
    parameter integer P_HEIGHT = 720
)(
    input  wire        I_clk_pix,
    input  wire        I_rst_n,
    input  wire [23:0] I_video,     // RGB888，1 像素/拍
    input  wire        I_de,        // 像素有效
    input  wire        I_vsync,     // 帧起始（仅重同步，不用于分帧）
    output reg  [7:0]  O_y_avg,     // 帧平均亮度（满帧全白 = 224）
    output reg         O_valid      // 帧边界输出一次
);

    // 一帧的有效像素数：1280 x 720 = 921600
    localparam [19:0] PIXELS_PER_FRAME = P_WIDTH * P_HEIGHT;

    //-------------------------------------------------------------------------
    // 单像素灰度：(r + 2g + b) >> 2     —— 无乘法器
    //-------------------------------------------------------------------------
    wire [9:0] W_s = {2'b00, I_video[23:16]} + {1'b0, I_video[15:8], 1'b0} + {2'b00, I_video[7:0]};
    wire [7:0] W_y = W_s[9:2];

    //-------------------------------------------------------------------------
    // 按有效拍数分帧
    //-------------------------------------------------------------------------
    reg [19:0] S_beat;
    reg [27:0] S_sum;               // 921600 拍 x 最多 255 = 235,008,000 < 2^28
    reg        S_vsync_d;

    wire W_frame_start = I_vsync && ~S_vsync_d;
    wire W_frame_end   = (S_beat == PIXELS_PER_FRAME - 20'd1);

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_beat    <= 20'd0;
            S_sum     <= 28'd0;
            S_vsync_d <= 1'b0;
            O_y_avg   <= 8'd0;
            O_valid   <= 1'b0;
        end
        else begin
            S_vsync_d <= I_vsync;
            O_valid   <= 1'b0;
            if (W_frame_start) begin
                // 帧起始：安全重同步（不依赖它分帧，防止下溢漂移）
                S_beat <= 20'd0;
                S_sum  <= 28'd0;
            end
            else if (I_de) begin
                S_sum <= S_sum + {20'd0, W_y};
                if (W_frame_end) begin
                    // 921600 像素 → 累加和 >> 20 就是平均亮度（满帧全白 = 224）
                    O_y_avg <= S_sum[27:20];
                    O_valid <= 1'b1;
                    S_beat  <= 20'd0;
                    S_sum   <= 28'd0;
                end
                else begin
                    S_beat <= S_beat + 20'd1;
                end
            end
        end
    end

endmodule
