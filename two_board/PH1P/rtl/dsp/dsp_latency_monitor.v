//=============================================================================
//  dsp_latency_monitor -- 显示域延迟锁存、去抖与毫秒换算
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk_pix
//  复位     : I_rst_n，异步低有效
//  参数     : P_STABLE_N
//  验证状态 : 仿真 通过 | 综合 通过 | 布线 通过 | 上板 未做
//=============================================================================
//  功能
//  ---------------------------------------------------------------------------
//  buf_display_latency_probe 在 DDR 域按显示场起始发布“显示帧龄（帧）”，用
//  数据保持 + toggle 的方式跨到 75 MHz 像素域。本模块：
//
//    1. 用两级同步器同步 toggle，在 toggle 边沿采样已经稳定整个显示帧的
//       5 bit 帧数（bundled-data CDC，见 probe 文件头说明）。
//    2. 复刻 v1.13 dsp_osd_mixer 的显示去抖：新值必须连续保持 P_STABLE_N+1
//       个显示帧才被采用，避免单帧异常让数码管跳数。
//    3. 把帧数换算成毫秒。720p60 的帧周期 = 1650 x 750 / 75 MHz = 16.5 ms，
//       因此 ms = frames * 16 + frames / 2（无乘法器/除法器，纯移位加法）。
//       2 帧 -> 33 ms，与 v1.13 的期望读数一致。
//
//  I_lat_frames_async 在探针侧只会发布 1..15，本模块不再重复做范围判断。
//=============================================================================
module dsp_latency_monitor #(
    parameter integer P_STABLE_N = 8
) (
    input wire I_clk_pix,
    input wire I_rst_n,

    // 来自 buf_display_latency_probe（DDR 域）
    input wire [4:0] I_lat_frames_async,
    input wire I_lat_update_async,

    output reg [4:0] O_lat_frames,
    output wire [9:0] O_lat_ms,
    output reg O_lat_valid
);

    localparam [3:0] P_STABLE_LIMIT = P_STABLE_N;

    //-------------------------------------------------------------------------
    // 两级同步：toggle 与数据同时出发，数据在两次发布之间保持一整帧，
    // 因此按 toggle 边沿采样数据是安全的。
    //-------------------------------------------------------------------------
    reg [1:0] S_update_sync;
    reg [4:0] S_frames_meta;
    reg [4:0] S_frames_sync;
    reg S_update_seen;

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_update_sync <= 2'b00;
            S_frames_meta <= 5'd0;
            S_frames_sync <= 5'd0;
        end
        else begin
            S_update_sync <= {S_update_sync[0], I_lat_update_async};
            S_frames_meta <= I_lat_frames_async;
            S_frames_sync <= S_frames_meta;
        end
    end

    wire W_new_sample = (S_update_sync[1] != S_update_seen);

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) S_update_seen <= 1'b0;
        else if (W_new_sample) S_update_seen <= S_update_sync[1];
    end

    //-------------------------------------------------------------------------
    // 显示去抖（v1.13：连续 STAB_N4+1 个显示帧同值才更新读数）
    //-------------------------------------------------------------------------
    reg [3:0] S_same_frames;

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_lat_frames <= 5'd0;
            O_lat_valid  <= 1'b0;
            S_same_frames <= 4'd0;
        end
        else if (W_new_sample) begin
            if (S_frames_sync == O_lat_frames) begin
                S_same_frames <= 4'd0;
            end
            else if (S_same_frames >= P_STABLE_LIMIT) begin
                O_lat_frames <= S_frames_sync;
                O_lat_valid  <= 1'b1;
                S_same_frames <= 4'd0;
            end
            else begin
                S_same_frames <= S_same_frames + 4'd1;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 帧数 -> 毫秒：ms = frames * 16 + frames / 2
    //-------------------------------------------------------------------------
    assign O_lat_ms = {1'b0, O_lat_frames, 4'b0} + {6'b0, O_lat_frames[4:1]};

endmodule
