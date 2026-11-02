// ============================================================================
// plate_locate.v  --  蓝底车牌定位（有界颜色填充投影）
//
// 算法链：RGB -> YCbCr -> Cb 二值化 -> 蓝色填充水平/垂直投影。
// 旧版“腐蚀 -> Sobel -> 膨胀”把填充牌变成两条边，再用固定 30 门限取
// 第一下降沿/最后上升沿；细牌和有缺口的牌会因此漏检或分裂。本版直接累加
// Cb 蓝色填充流，水平扫描取能量最大的有界连续带，垂直扫描只取该带内的
// 颜色投影。垂直投影复用上一帧的水平带，避免帧缓存和第二套大 RAM。
//
// 输入：RGB888 每像素视频流（de/vsync + 24bit 数据），时钟 = S_hdmi_pixel_clk (75MHz)
// 输出：o_up/o_down/o_left/o_right（车牌外接框，11bit）+ o_detected
// 延迟：边界在当前帧结束后的场消隐期计算并保持到下一帧（1 帧延迟）
//
// 颜色与尺寸参数：
//   Cb 二值化阈值 150（Cb > 150 判定为蓝）
//   水平带首选阈值 8、允许缺口 2 像素；扫描保留能量最大的有界连续带
//   垂直投影阈值 max(2, 水平带高>>4)，允许缺口 max(2, 水平带高>>2)
//   投影有效范围：列 10~1270（水平），行 10~710（垂直）
//
// 重要限制：
//   * 纯颜色定位 + 矩形形状过滤（长宽比 2:1~10:1、尺寸范围 MIN/MAX_W、MIN_H）。
//     形状过滤能滤掉大部分非车牌蓝色块，但无法保证 100% 正确；阈值仍建议上板微调。
//   * 水平带显著切换时本帧只输出 invalid；下一帧使用新带重新累加垂直投影。
// ============================================================================

`timescale 1ns / 1ps

module plate_locate #(
    parameter H_ACTIVE        = 1280,     // 水平有效像素
    parameter V_ACTIVE        = 720,      // 垂直有效行
    parameter BIN_THRESHOLD   = 8'd150,   // Cb 二值化阈值
    parameter PROJ_THRESHOLD  = 11'd30,   // 旧接口保留；颜色带扫描使用 H_RUN_THRESHOLD
    parameter HORIZON_START   = 11'd10,   // 水平投影起始列
    parameter HORIZON_END     = 11'd1270, // 水平投影结束列
    parameter VERTICAL_START  = 10'd10,   // 垂直投影起始行
    parameter VERTICAL_END    = 10'd710,  // 垂直投影结束行
    parameter MIN_W           = 11'd60,   // 车牌框最小宽（滤噪）
    parameter MAX_W           = 11'd1000, // 车牌框最大宽（排除整屏宽蓝区，如天空）
    parameter MIN_H           = 11'd16,   // 车牌框最小高（滤噪）
    parameter H_RUN_THRESHOLD = 11'd8,    // 水平填充投影门限（首个有界候选）
    parameter H_RUN_GAP       = 5'd2,    // 水平带允许的零值缺口
    parameter H_SWITCH_MARGIN = 11'd16,   // 水平带切换超过此值时本帧 invalid
    parameter RATIO_MIN_NUM   = 4'd2,     // box_w >= box_h * RATIO_MIN_NUM
    parameter RATIO_MAX_NUM   = 4'd10     // box_w <= box_h * RATIO_MAX_NUM
)(
    input  wire            clk,
    input  wire            rst_n,
    input  wire [23:0]     i_rgb,    // RGB888: {R[23:16], G[15:8], B[7:0]}
    input  wire            i_de,     // 像素有效
    input  wire            i_vsync,  // 帧同步（高有效，脉冲在垂直消隐期）
    output reg  [10:0]     o_up,
    output reg  [10:0]     o_down,
    output reg  [10:0]     o_left,
    output reg  [10:0]     o_right,
    output reg             o_detected,
    // One-clock pulse after the S_DONE result commit.  This is an
    // observation-only event for consumers that need the newly committed
    // result; bbox and o_detected behavior remain unchanged.
    output reg             o_result_update
);

    // ---- 场同步边沿 ----
    reg vsync_r;
    always @(posedge clk or negedge rst_n)
        if (!rst_n) vsync_r <= 1'b0;
        else        vsync_r <= i_vsync;
    wire vs_rise =  i_vsync & ~vsync_r;   // 场同步脉冲起点（消隐期，帧结束）
    wire vs_fall = ~i_vsync &  vsync_r;   // 场同步脉冲结束（有效帧即将开始）

    // ===================== 阶段1：RGB -> YCbCr（提取 Cb） =====================
    wire [7:0] cb;
    wire       cb_de;
    rgb2ycbcr u_rgb2ycbcr(
        .clk  (clk), .rst_n(rst_n),
        .i_r  (i_rgb[23:16]),
        .i_g  (i_rgb[15:8]),
        .i_b  (i_rgb[7:0]),
        .i_de (i_de),
        .o_cb (cb),
        .o_de (cb_de)
    );

    // ===================== 阶段2：Cb 二值化 =====================
    reg bin_bit;
    reg bin_de;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bin_bit <= 1'b0;
            bin_de  <= 1'b0;
        end else begin
            bin_bit <= (cb > BIN_THRESHOLD);
            bin_de  <= cb_de;
        end
    end

    // ===================== 阶段3：蓝色填充流 =====================
    // 直接复用二值颜色流，综合时不再保留车牌层的两套 3x3 行缓存。
    wire fill_bit = bin_bit;
    wire fill_de  = bin_de;

    // ===================== 阶段4：坐标计数与帧内投影 =====================
    reg [10:0] x_cnt;
    reg [9:0]  y_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_cnt <= 11'd0;
            y_cnt <= 10'd0;
        end else if (vs_fall) begin
            x_cnt <= 11'd0;
            y_cnt <= 10'd0;
        end else if (fill_de) begin
            if (x_cnt == H_ACTIVE - 1) begin
                x_cnt <= 11'd0;
                y_cnt <= y_cnt + 1'b1;
            end else begin
                x_cnt <= x_cnt + 1'b1;
            end
        end
    end

    // 每行写一次水平填充投影；当前像素也计入行末值。
    reg [10:0] h_line_cnt;
    reg        h_wr_en;
    reg [9:0]  h_wr_addr;
    reg [10:0] h_wr_data;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_line_cnt <= 11'd0;
            h_wr_en    <= 1'b0;
            h_wr_addr  <= 10'd0;
            h_wr_data  <= 11'd0;
        end else begin
            h_wr_en <= 1'b0;
            if (fill_de) begin
                if (x_cnt == 0)
                    h_line_cnt <= 11'd0;
                else if ((x_cnt > HORIZON_START) && (x_cnt < HORIZON_END))
                    h_line_cnt <= h_line_cnt + fill_bit;
                if (x_cnt == H_ACTIVE - 1) begin
                    h_wr_en   <= 1'b1;
                    h_wr_addr <= y_cnt;
                    if ((x_cnt > HORIZON_START) && (x_cnt < HORIZON_END))
                        h_wr_data <= h_line_cnt + fill_bit;
                    else
                        h_wr_data <= h_line_cnt;
                end
            end
        end
    end

    // h_proj/v_proj are the only frame-sized storage in this path.
    (*force_ram*)
    (*ram_style = "bram"*)
    reg [10:0] h_proj [0:V_ACTIVE-1];
    reg [10:0] h_rd;
    (*force_ram*)
    (*ram_style = "bram"*)
    reg [9:0]  v_proj [0:H_ACTIVE-1];

    // 上一帧选出的水平带约束当前帧的垂直累加；首帧没有带时使用全高。
    reg        h_prev_valid;
    reg [10:0] h_prev_lo;
    reg [10:0] h_prev_hi;
    reg [10:0] x_d1;
    reg [9:0]  y_d1;
    reg        bit_d1;
    reg        de_d1;
    reg [9:0]  v_rd;

    // ---- 有界扫描状态 ----
    localparam [3:0]
        S_IDLE    = 4'd0,
        S_H_REQ   = 4'd1,
        S_H_EVAL  = 4'd2,
        S_H_NEXT  = 4'd3,
        S_H_SELECT = 4'd4,
        S_V_REQ   = 4'd5,
        S_V_EVAL  = 4'd6,
        S_V_NEXT  = 4'd7,
        S_V_SELECT = 4'd8,
        S_DONE    = 4'd9;
    reg [3:0] st;
    reg [10:0] idx;

    // 水平扫描：一个候选带寄存器 + 一个能量最大值寄存器。
    reg        h_run_active;
    reg [4:0]  h_gap_count;
    reg [10:0] h_run_start;
    reg [10:0] h_last_hit;
    reg [10:0] h_run_length;
    reg [20:0] h_run_energy;
    reg        h_best_valid;
    reg [10:0] h_best_start;
    reg [10:0] h_best_end;
    reg [10:0] h_best_length;
    reg [20:0] h_best_energy;
    reg        h_band_valid;
    reg        h_switch_bad;

    // 垂直扫描：只在当前投影中保留一个合法候选。
    reg [10:0] v_scan_threshold;
    reg [10:0] v_gap_limit;
    reg        v_run_active;
    reg [10:0] v_gap_count;
    reg [10:0] v_run_start;
    reg [10:0] v_last_hit;
    reg [10:0] v_run_length;
    reg [20:0] v_run_energy;
    reg        v_best_valid;
    reg [10:0] v_best_start;
    reg [10:0] v_best_end;
    reg [10:0] v_best_length;
    reg [20:0] v_best_energy;
    reg [10:0] h_lo;
    reg [10:0] h_hi;
    reg [10:0] v_lo;
    reg [10:0] v_hi;

    wire h_scan = (st == S_H_REQ);
    wire v_scan = (st == S_V_REQ);

    // h_proj：水平投影 RAM。保持单一组合读地址，避免条件数组引用被
    // TD 前端拆成多个读口；非扫描周期读地址固定为零且读口无 CE/复位。
    wire [9:0] h_read_address = h_scan ? idx[9:0] : 10'd0;
    wire [9:0] h_write_address = h_wr_addr;
    wire [10:0] h_write_value = h_wr_data;
    always @(posedge clk)
        h_rd <= h_proj[h_read_address];
    always @(posedge clk)
        if (h_wr_en)
            h_proj[h_write_address] <= h_write_value;

    // 垂直投影：首行清零；后续只累加上一帧水平带（首帧全高）。
    // v_rd 在像素流阶段预读当前列，de_d1 对齐一次读改写。
    wire v_band_allow = !h_prev_valid ||
                         ((y_d1 >= h_prev_lo) && (y_d1 <= h_prev_hi));
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_d1   <= 11'd0;
            y_d1   <= 10'd0;
            bit_d1 <= 1'b0;
            de_d1  <= 1'b0;
        end else begin
            x_d1   <= x_cnt;
            y_d1   <= y_cnt;
            bit_d1 <= fill_bit;
            de_d1  <= fill_de;
        end
    end

    // Single unconditional synchronous read; the mux presents one address to
    // TD inference for scan or pixel-stream use, and zero while idle.
    wire [10:0] v_read_address = v_scan ? idx[10:0] :
                                  (fill_de ? {1'b0,x_cnt} : 11'd0);
    always @(posedge clk)
        v_rd <= v_proj[v_read_address];

    // Keep the original first-row clear and band-gated read-modify-write.
    wire v_write_enable = de_d1 &&
                          ((y_d1 == 0) ||
                           ((y_d1 > VERTICAL_START) &&
                            (y_d1 < VERTICAL_END) && v_band_allow));
    wire [10:0] v_write_address = x_d1;
    wire [9:0] v_write_data = (y_d1 == 0) ? 10'd0 : (v_rd + bit_d1);
    always @(posedge clk)
        if (v_write_enable)
            v_proj[v_write_address] <= v_write_data;

    // 形状判定：常数乘法展开成移位相加，避免再次推断 DSP。
    wire [10:0] box_w = v_hi - v_lo + 11'd1;
    wire [10:0] box_h = h_hi - h_lo + 11'd1;
    function [12:0] mul_small;
        input [10:0] v;
        input [3:0]  k;
        integer      i;
        reg [12:0]  acc;
        begin
            acc = 13'd0;
            for (i = 0; i < 4; i = i + 1)
                if (k[i])
                    acc = acc + (v << i);
            mul_small = acc;
        end
    endfunction
    wire [12:0] bw_x1  = {2'b00, box_w};
    wire [12:0] bh_min = mul_small(box_h, RATIO_MIN_NUM);
    wire [12:0] bh_max = mul_small(box_h, RATIO_MAX_NUM);
    wire ratio_ok = (bw_x1 >= bh_min) && (bw_x1 <= bh_max);
    wire size_ok  = (box_w >= MIN_W) && (box_w <= MAX_W) &&
                    (box_h >= MIN_H) && (box_h <= 11'd400);
    wire plate_ok = h_band_valid && !h_switch_bad && v_best_valid &&
                    (v_lo < v_hi) && (h_lo < h_hi) && ratio_ok && size_ok;

    wire [10:0] h_delta_lo = (h_best_start >= h_prev_lo) ?
                              (h_best_start - h_prev_lo) :
                              (h_prev_lo - h_best_start);
    wire [10:0] h_delta_hi = (h_best_end >= h_prev_hi) ?
                              (h_best_end - h_prev_hi) :
                              (h_prev_hi - h_best_end);

    // ---- 水平扫描：阈值 8、缺口 2，选择能量最大的合法带 ----
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st             <= S_IDLE;
            idx            <= 11'd0;
            h_run_active   <= 1'b0;
            h_gap_count    <= 5'd0;
            h_run_start    <= 11'd0;
            h_last_hit     <= 11'd0;
            h_run_length   <= 11'd0;
            h_run_energy   <= 21'd0;
            h_best_valid   <= 1'b0;
            h_best_start   <= 11'd0;
            h_best_end     <= 11'd0;
            h_best_length  <= 11'd0;
            h_best_energy  <= 21'd0;
            h_band_valid   <= 1'b0;
            h_switch_bad   <= 1'b0;
            h_prev_valid   <= 1'b0;
            h_prev_lo      <= 11'd0;
            h_prev_hi      <= 11'd0;
            v_scan_threshold <= 11'd2;
            v_gap_limit    <= 11'd2;
            v_run_active   <= 1'b0;
            v_gap_count    <= 11'd0;
            v_run_start    <= 11'd0;
            v_last_hit     <= 11'd0;
            v_run_length   <= 11'd0;
            v_run_energy   <= 21'd0;
            v_best_valid   <= 1'b0;
            v_best_start   <= 11'd0;
            v_best_end     <= 11'd0;
            v_best_length  <= 11'd0;
            v_best_energy  <= 21'd0;
            h_lo           <= 11'd0;
            h_hi           <= 11'd0;
            v_lo           <= 11'd0;
            v_hi           <= 11'd0;
            o_up           <= 11'd0;
            o_down         <= 11'd0;
            o_left         <= 11'd0;
            o_right        <= 11'd0;
            o_detected     <= 1'b0;
            o_result_update <= 1'b0;
        end else begin
            o_result_update <= 1'b0;
            case (st)
                S_IDLE: begin
                    if (vs_rise) begin
                        idx           <= 11'd0;
                        h_run_active  <= 1'b0;
                        h_gap_count   <= 5'd0;
                        h_best_valid  <= 1'b0;
                        h_best_start  <= 11'd0;
                        h_best_end    <= 11'd0;
                        h_best_length <= 11'd0;
                        h_best_energy <= 21'd0;
                        st            <= S_H_REQ;
                    end
                end

                S_H_REQ: begin
                    st <= S_H_EVAL;
                end

                S_H_EVAL: begin
                    if (h_rd > H_RUN_THRESHOLD) begin
                        if (!h_run_active) begin
                            h_run_active <= 1'b1;
                            h_run_start  <= idx;
                            h_last_hit   <= idx;
                            h_run_length <= 11'd1;
                            h_run_energy <= h_rd;
                            h_gap_count  <= 5'd0;
                        end else if (h_gap_count <= H_RUN_GAP) begin
                            h_last_hit   <= idx;
                            h_run_length <= idx - h_run_start + 1'b1;
                            h_run_energy <= h_run_energy + h_rd;
                            h_gap_count  <= 5'd0;
                        end else begin
                            if ((h_run_length >= MIN_H) &&
                                (h_run_length <= 11'd400) &&
                                ((!h_best_valid) ||
                                 (h_run_energy > h_best_energy) ||
                                 ((h_run_energy == h_best_energy) &&
                                  (h_run_length > h_best_length)))) begin
                                h_best_valid  <= 1'b1;
                                h_best_start  <= h_run_start;
                                h_best_end    <= h_last_hit;
                                h_best_length <= h_run_length;
                                h_best_energy <= h_run_energy;
                            end
                            h_run_start  <= idx;
                            h_last_hit   <= idx;
                            h_run_length <= 11'd1;
                            h_run_energy <= h_rd;
                            h_gap_count  <= 5'd0;
                            h_run_active  <= 1'b1;
                        end
                    end else if (h_run_active) begin
                        if (h_gap_count < H_RUN_GAP)
                            h_gap_count <= h_gap_count + 1'b1;
                        else begin
                            if ((h_run_length >= MIN_H) &&
                                (h_run_length <= 11'd400) &&
                                ((!h_best_valid) ||
                                 (h_run_energy > h_best_energy) ||
                                 ((h_run_energy == h_best_energy) &&
                                  (h_run_length > h_best_length)))) begin
                                h_best_valid  <= 1'b1;
                                h_best_start  <= h_run_start;
                                h_best_end    <= h_last_hit;
                                h_best_length <= h_run_length;
                                h_best_energy <= h_run_energy;
                            end
                            h_run_active <= 1'b0;
                            h_gap_count  <= 5'd0;
                        end
                    end

                    if (idx == V_ACTIVE - 1) begin
                        st <= S_H_NEXT;
                    end else begin
                        idx <= idx + 1'b1;
                        st  <= S_H_REQ;
                    end
                end

                S_H_NEXT: begin
                    // End-of-domain closes a run whose trailing gap is <= limit.
                    if (h_run_active &&
                        (h_run_length >= MIN_H) &&
                        (h_run_length <= 11'd400) &&
                        ((!h_best_valid) ||
                         (h_run_energy > h_best_energy) ||
                         ((h_run_energy == h_best_energy) &&
                          (h_run_length > h_best_length)))) begin
                        h_best_valid  <= 1'b1;
                        h_best_start  <= h_run_start;
                        h_best_end    <= h_last_hit;
                        h_best_length <= h_run_length;
                        h_best_energy <= h_run_energy;
                    end
                    st <= S_H_SELECT;
                end

                S_H_SELECT: begin
                    if (h_best_valid) begin
                        h_lo         <= h_best_start;
                        h_hi         <= h_best_end;
                        h_band_valid <= 1'b1;
                        if (h_prev_valid &&
                            ((h_delta_lo > H_SWITCH_MARGIN) ||
                             (h_delta_hi > H_SWITCH_MARGIN)))
                            h_switch_bad <= 1'b1;
                        else
                            h_switch_bad <= 1'b0;
                        h_prev_valid <= 1'b1;
                        h_prev_lo    <= h_best_start;
                        h_prev_hi    <= h_best_end;
                        v_scan_threshold <= ((h_best_length >> 4) < 2) ?
                                            11'd2 : (h_best_length >> 4);
                        v_gap_limit      <= ((h_best_length >> 2) < 2) ?
                                            11'd2 : (h_best_length >> 2);
                        idx           <= 11'd0;
                        v_run_active  <= 1'b0;
                        v_gap_count   <= 11'd0;
                        v_best_valid  <= 1'b0;
                        v_best_start  <= 11'd0;
                        v_best_end    <= 11'd0;
                        v_best_length <= 11'd0;
                        v_best_energy <= 21'd0;
                        st            <= S_V_REQ;
                    end else begin
                        h_band_valid <= 1'b0;
                        h_switch_bad <= 1'b1;
                        h_prev_valid <= 1'b0;
                        h_lo         <= 11'd0;
                        h_hi         <= 11'd0;
                        v_lo         <= 11'd0;
                        v_hi         <= 11'd0;
                        st            <= S_DONE;
                    end
                end

                S_V_REQ: begin
                    st <= S_V_EVAL;
                end

                S_V_EVAL: begin
                    if (v_rd > v_scan_threshold) begin
                        if (!v_run_active) begin
                            v_run_active <= 1'b1;
                            v_run_start  <= idx;
                            v_last_hit   <= idx;
                            v_run_length <= 11'd1;
                            v_run_energy <= v_rd;
                            v_gap_count  <= 11'd0;
                        end else if (v_gap_count <= v_gap_limit) begin
                            v_last_hit   <= idx;
                            v_run_length <= idx - v_run_start + 1'b1;
                            v_run_energy <= v_run_energy + v_rd;
                            v_gap_count  <= 11'd0;
                        end else begin
                            if ((v_run_length >= MIN_W) &&
                                (v_run_length <= MAX_W) &&
                                ((!v_best_valid) ||
                                 (v_run_energy > v_best_energy) ||
                                 ((v_run_energy == v_best_energy) &&
                                  (v_run_length > v_best_length)))) begin
                                v_best_valid  <= 1'b1;
                                v_best_start  <= v_run_start;
                                v_best_end    <= v_last_hit;
                                v_best_length <= v_run_length;
                                v_best_energy <= v_run_energy;
                            end
                            v_run_start  <= idx;
                            v_last_hit   <= idx;
                            v_run_length <= 11'd1;
                            v_run_energy <= v_rd;
                            v_gap_count  <= 11'd0;
                            v_run_active  <= 1'b1;
                        end
                    end else if (v_run_active) begin
                        if (v_gap_count < v_gap_limit)
                            v_gap_count <= v_gap_count + 1'b1;
                        else begin
                            if ((v_run_length >= MIN_W) &&
                                (v_run_length <= MAX_W) &&
                                ((!v_best_valid) ||
                                 (v_run_energy > v_best_energy) ||
                                 ((v_run_energy == v_best_energy) &&
                                  (v_run_length > v_best_length)))) begin
                                v_best_valid  <= 1'b1;
                                v_best_start  <= v_run_start;
                                v_best_end    <= v_last_hit;
                                v_best_length <= v_run_length;
                                v_best_energy <= v_run_energy;
                            end
                            v_run_active <= 1'b0;
                            v_gap_count  <= 11'd0;
                        end
                    end

                    if (idx == H_ACTIVE - 1) begin
                        st <= S_V_NEXT;
                    end else begin
                        idx <= idx + 1'b1;
                        st  <= S_V_REQ;
                    end
                end

                S_V_NEXT: begin
                    if (v_run_active &&
                        (v_run_length >= MIN_W) &&
                        (v_run_length <= MAX_W) &&
                        ((!v_best_valid) ||
                         (v_run_energy > v_best_energy) ||
                         ((v_run_energy == v_best_energy) &&
                          (v_run_length > v_best_length)))) begin
                        v_best_valid  <= 1'b1;
                        v_best_start  <= v_run_start;
                        v_best_end    <= v_last_hit;
                        v_best_length <= v_run_length;
                        v_best_energy <= v_run_energy;
                    end
                    st <= S_V_SELECT;
                end

                S_V_SELECT: begin
                    if (v_best_valid) begin
                        v_lo <= v_best_start;
                        v_hi <= v_best_end;
                    end else begin
                        v_lo <= 11'd0;
                        v_hi <= 11'd0;
                    end
                    st <= S_DONE;
                end

                S_DONE: begin
                    o_up       <= h_lo;
                    o_down     <= h_hi;
                    o_left     <= v_lo;
                    o_right    <= v_hi;
                    o_detected <= plate_ok;
                    o_result_update <= 1'b1;
                    st         <= S_IDLE;
                end

                default: begin
                    st <= S_IDLE;
                end
            endcase
        end
    end

endmodule


// ============================================================================
// rgb2ycbcr : RGB888 -> YCbCr 中的 Cb 分量（3 级流水）
//   Cb = (-43*R - 85*G + 128*B + 32768) >> 8
//   与原工程 rgb2ycbcr.v 公式一致，但直接接收 8bit RGB（原工程为 RGB565 先展开）。
// ============================================================================
module rgb2ycbcr(
    input  wire     clk,
    input  wire     rst_n,
    input  wire [7:0] i_r,
    input  wire [7:0] i_g,
    input  wire [7:0] i_b,
    input  wire     i_de,
    output wire [7:0] o_cb,
    output wire     o_de
);

    reg [15:0] r_m, g_m, b_m;   // 第 1 级：乘法
    reg [15:0] cb0;             // 第 2 级：累加
    reg [7:0]  cb1;             // 第 3 级：右移 8
    reg [2:0]  de_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_m <= 16'd0; g_m <= 16'd0; b_m <= 16'd0;
            cb0 <= 16'd0; cb1 <= 8'd0; de_d <= 3'd0;
        end else begin
            r_m <= i_r * 16'd43;                 // 43*R
            g_m <= i_g * 16'd85;                 // 85*G
            b_m <= {1'b0, i_b, 7'b0};            // 128*B（左移 7 位，明确 16bit）
            cb0 <= b_m - r_m - g_m + 16'd32768;  // -43R-85G+128B+32768
            cb1 <= cb0[15:8];                    // >>8
            de_d <= {de_d[1:0], i_de};           // de 延迟 3 拍
        end
    end

    assign o_cb = cb1;
    assign o_de = de_d[2];

endmodule


// ============================================================================
// sobel_bin : 二值图 Sobel 边缘检测（等价实现）
//   原工程把 1bit 腐蚀结果复制成 {8{bit}}（0 或 255）送入 8bit Sobel，
//   用 Xilinx Cordic 求 sqrt(gx^2+gy^2) 再与阈值 128 比较。
//   由于输入只有 0/255 两档，gx=255*bgx、gy=255*bgy（bgx,bgy 为 0/1 二值梯度），
//   sqrt(gx^2+gy^2)>=128  <==>  bgx^2+bgy^2 >= 1  <==>  bgx!=0 或 bgy!=0。
//   因此等价于：3x3 窗口内存在 0/1 跳变即判为边缘。省去 Cordic，便于 TD 综合。
// ============================================================================
module sobel_bin #(
    parameter H_ACTIVE = 1280
)(
    input  wire     clk,
    input  wire     rst_n,
    input  wire     de,
    input  wire     din,      // 1bit 输入
    output reg      de_o,
    output reg      dout
);

    // 两行移位缓存（深度 H_ACTIVE，仅 de 有效时移位）
    //
    // 2026-10-05 P0-1c 修复（斜切缺陷）：
    //   原实现声明 [H_ACTIVE:0]（=1281 位）却只把 [H_ACTIVE-1:0] 参与移位，
    //   而抽头取的是 lb1[H_ACTIVE]。移位链上 bit k 的延迟是 (1+k) 拍，所以
    //   lb1[H_ACTIVE] 的延迟是 H_ACTIVE+1 拍 —— 比一整行多 1 个像素。
    //   后果：3x3 窗口每向上一行就左偏 1 像素，窗口沿行方向被"斜切"，
    //   边缘检测在竖直方向上出现阶梯状伪边缘。参考工程用地址式 BRAM
    //   行缓存（line_shift_ram_8bit.v）没有这个问题。
    //   修法：寄存器改成正好 H_ACTIVE 位、抽最高位 H_ACTIVE-1，
    //   使行距精确等于 H_ACTIVE 个 de 拍。
    //   修复后延迟对照（H = H_ACTIVE）：
    //     p13/p12/p11 = din(t-H)  / din(t-H-1)  / din(t-H-2)
    //     p23/p22/p21 = din(t-2H) / din(t-2H-1) / din(t-2H-2)
    //     p33/p32/p31 = din(t)    / din(t-1)    / din(t-2)
    //   三行同列对齐，行距恒为 H。
    reg [H_ACTIVE-1:0] lb1;
    reg [H_ACTIVE-1:0] lb2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lb1 <= {H_ACTIVE{1'b0}};
            lb2 <= {H_ACTIVE{1'b0}};
        end else if (de) begin
            lb1 <= {lb1[H_ACTIVE-2:0], din};
            lb2 <= {lb2[H_ACTIVE-2:0], lb1[H_ACTIVE-1]};
        end
    end

    // 每行水平两拍延迟
    reg p12, p11;
    reg p22, p21;
    reg p32, p31;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p11 <= 1'b0; p12 <= 1'b0;
            p21 <= 1'b0; p22 <= 1'b0;
            p31 <= 1'b0; p32 <= 1'b0;
        end else begin
            p12 <= lb1[H_ACTIVE-1];  p11 <= p12;
            p22 <= lb2[H_ACTIVE-1];  p21 <= p22;
            p32 <= din;            p31 <= p32;
        end
    end

    wire p13 = lb1[H_ACTIVE-1];
    wire p23 = lb2[H_ACTIVE-1];
    wire p33 = din;

    // 二值 Sobel 梯度（各为 0..4）
    wire [2:0] gx_a = {2'b00, p13} + {1'b0, p23, 1'b0} + {2'b00, p33};  // 右列
    wire [2:0] gx_b = {2'b00, p11} + {1'b0, p21, 1'b0} + {2'b00, p31};  // 左列
    wire [2:0] gy_a = {2'b00, p11} + {1'b0, p12, 1'b0} + {2'b00, p13};  // 上行
    wire [2:0] gy_b = {2'b00, p31} + {1'b0, p32, 1'b0} + {2'b00, p33};  // 下行
    wire [2:0] gx = (gx_a >= gx_b) ? (gx_a - gx_b) : (gx_b - gx_a);
    wire [2:0] gy = (gy_a >= gy_b) ? (gy_a - gy_b) : (gy_b - gy_a);
    wire edge_bit = (gx != 3'd0) || (gy != 3'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_o <= 1'b0;
            dout <= 1'b0;
        end else begin
            de_o <= de;
            dout <= edge_bit;
        end
    end

endmodule


// ============================================================================
// morph3x3 : 1bit 3x3 形态学滤波
//   OP=0 腐蚀(9 点全与)；OP=1 膨胀(9 点全或)
//   行缓存用 H_ACTIVE 深度的移位寄存器实现（仅 de 有效时移位）。
//   输出相对输入延迟 1 拍，窗口中心对应 (x-1, y-1)（约 1 像素偏移，对定位无影响）。
// ============================================================================
module morph3x3 #(
    parameter H_ACTIVE = 1280,
    parameter OP       = 0        // 0 = 腐蚀(AND), 1 = 膨胀(OR)
)(
    input  wire     clk,
    input  wire     rst_n,
    input  wire     de,
    input  wire     din,
    output reg      de_o,
    output reg      dout
);

    // 2026-10-05 P0-1c 修复（与 sobel_bin 同一处斜切缺陷）：
    //   原 [H_ACTIVE:0]（1281 位）但只移位 [H_ACTIVE-1:0]、抽头 lb1[H_ACTIVE]
    //   ⇒ 抽头延迟为 H_ACTIVE+1 拍，每深一行多偏 1 像素，3x3 窗口被斜切。
    //   本模块同时被 plate_locate（车牌层腐蚀/膨胀）与 char_locate
    //   （字符层腐蚀/膨胀）使用，故一处修复受益两处。
    reg [H_ACTIVE-1:0] lb1;
    reg [H_ACTIVE-1:0] lb2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lb1 <= {H_ACTIVE{1'b0}};
            lb2 <= {H_ACTIVE{1'b0}};
        end else if (de) begin
            lb1 <= {lb1[H_ACTIVE-2:0], din};
            lb2 <= {lb2[H_ACTIVE-2:0], lb1[H_ACTIVE-1]};
        end
    end

    reg p12, p11;
    reg p22, p21;
    reg p32, p31;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p11 <= 1'b0; p12 <= 1'b0;
            p21 <= 1'b0; p22 <= 1'b0;
            p31 <= 1'b0; p32 <= 1'b0;
        end else begin
            p12 <= lb1[H_ACTIVE-1];  p11 <= p12;
            p22 <= lb2[H_ACTIVE-1];  p21 <= p22;
            p32 <= din;            p31 <= p32;
        end
    end

    wire p13 = lb1[H_ACTIVE-1];
    wire p23 = lb2[H_ACTIVE-1];
    wire p33 = din;

    wire w;
    generate
        if (OP == 0)
            assign w = p11 & p12 & p13 & p21 & p22 & p23 & p31 & p32 & p33;
        else
            assign w = p11 | p12 | p13 | p21 | p22 | p23 | p31 | p32 | p33;
    endgenerate

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_o <= 1'b0;
            dout <= 1'b0;
        end else begin
            de_o <= de;
            dout <= w;
        end
    end

endmodule
