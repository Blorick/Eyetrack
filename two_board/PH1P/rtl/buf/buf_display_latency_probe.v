//=============================================================================
//  buf_display_latency_probe -- 帧缓存侧的端到端延迟探测（采集帧票号 -> 显示帧龄）
//-----------------------------------------------------------------------------
//  所属域   : buf
//  时钟域   : I_clk_ddr
//  复位     : I_rst_n，异步低有效
//  参数     : 无
//  验证状态 : 仿真 通过 | 综合 通过 | 布线 通过 | 上板 未做
//=============================================================================
//  为什么这样测
//  ---------------------------------------------------------------------------
//  v1.13 在 8 位数码管上显示“端到端延迟”，做法是把采集侧帧票号打进像素流，
//  显示侧再从像素里把票号找回来。v2.5 的 PH1P 采集/DDR 交接由原厂
//  video_in/video_out 固定实现，128 bit 打包格式也不是自然像素对齐，直接往
//  像素里塞票号会改动 vendor 代码和打包语义；这里改为在帧缓存交接处打票号：
//
//    * video_in 在每个采集帧起始把该帧写进 4 个 bank 之一
//      （bank = 采集帧序号 mod 4，地址 IMAGE_BASE_ADDR_0..3）。
//    * video_out 在每次显示帧起始按 I_video_out_rp 选 bank；该 rp 在
//      采集帧起始被 video_in 设为 (写指针 + 2) mod 4，即显示的是 2 个
//      采集帧之前的完整帧（v1.13 的四帧地址映射结论同样是“稳态约滞后
//      两帧，60 fps 约 33.3 ms”）。
//
//  因此本模块在 DDR 域给每个 bank 记下“写入它的采集帧票号”，在显示帧起始
//  （与 video_out 内部完全相同的 3 级同步 + 上升沿检测）算：
//
//      延迟(帧) = 当前采集帧序号 - 被显示 bank 的采集帧序号
//
//  当前采集帧序号取 S_cap_count-1（S_cap_count 是已开始的帧数），被显示帧的
//  序号就是该 bank 的票号，稳态结果 = 2 帧 = 33 ms。票号 5 bit 回绕，减法在
//  5 bit 内完成；实测值只在 1..15 帧内才发布，超出（复位竞争、bank 未写、
//  显示严重落后）时保持上一次有效值。发布用“数据保持 + toggle”跨到像素域，
//  数据在两次发布之间至少稳定一整个显示帧，dsp_latency_monitor 侧按 toggle
//  边沿采样，属于 bundled-data CDC，不需要多 bit 位间偏斜约束。
//
//  这不是把常数写死：若显示取错/取旧 bank、或写指针与读指针关系被异常打断，
//  测到的值会随被显示帧的票号变化。正常 720p60 稳定显示 2 帧 / 33 ms。
//
//  依赖：bank 基地址必须与 video_in / video_out 内的 IMAGE_BASE_ADDR_* 一致。
//=============================================================================
module buf_display_latency_probe (
    input wire I_clk_ddr,
    input wire I_rst_n,

    // video_in -> DDR 写交接（DDR 域）
    input wire I_wr_en,
    input wire [24:0] I_wr_addr,

    // video_out 选帧结果与显示场起始（rp 为 DDR 域寄存器，vsync 来自像素域）
    input wire [1:0] I_display_rp,
    input wire I_display_vsync,

    output reg [4:0] O_lat_frames,
    output reg O_lat_update
);

    //-------------------------------------------------------------------------
    // 与 vendor video_in / video_out 固定实现一致的 4 个 bank 基地址。
    // 改 vendor 侧地址映射时这里必须同步，否则票号会记到错误的 bank。
    //-------------------------------------------------------------------------
    localparam [24:0] P_IMAGE_BASE_0 = 25'd0;
    localparam [24:0] P_IMAGE_BASE_1 = 25'd7000000;
    localparam [24:0] P_IMAGE_BASE_2 = 25'd14000000;
    localparam [24:0] P_IMAGE_BASE_3 = 25'd21000000;

    // v1.13 dsp_osd_mixer 的有效门限：物理上就该落在 1..15 帧。
    localparam [4:0] P_LAT_FRAMES_MIN = 5'd1;
    localparam [4:0] P_LAT_FRAMES_MAX = 5'd15;

    //-------------------------------------------------------------------------
    // 写帧起始检测：每个 bank 的第一次写地址 == 基地址，一帧只出现一次。
    //-------------------------------------------------------------------------
    wire W_wr_frame_start;
    wire [1:0] W_wr_frame_index;

    assign W_wr_frame_start = I_wr_en &&
        ((I_wr_addr == P_IMAGE_BASE_0) || (I_wr_addr == P_IMAGE_BASE_1) ||
         (I_wr_addr == P_IMAGE_BASE_2) || (I_wr_addr == P_IMAGE_BASE_3));

    assign W_wr_frame_index = (I_wr_addr == P_IMAGE_BASE_0) ? 2'd0 :
                              (I_wr_addr == P_IMAGE_BASE_1) ? 2'd1 :
                              (I_wr_addr == P_IMAGE_BASE_2) ? 2'd2 : 2'd3;

    //-------------------------------------------------------------------------
    // 采集帧票号 + 每个 bank 最近一次写入时的票号。
    //   S_cap_count 是“已开始的采集帧数”，写入 bank 的票号取自增之前的值，
    //   即 0 基帧序号；显示帧起始时用 (S_cap_count-1) 作当前帧序号。
    //-------------------------------------------------------------------------
    reg [4:0] S_cap_count;
    reg [4:0] S_buf_ticket_0;
    reg [4:0] S_buf_ticket_1;
    reg [4:0] S_buf_ticket_2;
    reg [4:0] S_buf_ticket_3;

    always @(posedge I_clk_ddr or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_cap_count    <= 5'd0;
            S_buf_ticket_0 <= 5'd0;
            S_buf_ticket_1 <= 5'd0;
            S_buf_ticket_2 <= 5'd0;
            S_buf_ticket_3 <= 5'd0;
        end
        else if (W_wr_frame_start) begin
            S_cap_count <= S_cap_count + 5'd1;
            case (W_wr_frame_index)
                2'd0: S_buf_ticket_0 <= S_cap_count;
                2'd1: S_buf_ticket_1 <= S_cap_count;
                2'd2: S_buf_ticket_2 <= S_cap_count;
                default: S_buf_ticket_3 <= S_cap_count;
            endcase
        end
    end

    //-------------------------------------------------------------------------
    // 显示场起始：复制 video_out 内部 S_video_vsync_1d/2d/3d + 上升沿检测，
    // 使本模块与 video_out 选 bank、发起 DDR 读在同一 DDR 时钟沿。
    //-------------------------------------------------------------------------
    reg [2:0] S_vsync_sync;

    always @(posedge I_clk_ddr or negedge I_rst_n) begin
        if (!I_rst_n) S_vsync_sync <= 3'b000;
        else          S_vsync_sync <= {S_vsync_sync[1:0], I_display_vsync};
    end

    wire W_display_sof = ~S_vsync_sync[2] & S_vsync_sync[1];

    wire [4:0] W_display_ticket;

    assign W_display_ticket = (I_display_rp == 2'd0) ? S_buf_ticket_0 :
                              (I_display_rp == 2'd1) ? S_buf_ticket_1 :
                              (I_display_rp == 2'd2) ? S_buf_ticket_2 :
                                                       S_buf_ticket_3;

    wire [4:0] W_lat_frames = (S_cap_count - 5'd1) - W_display_ticket;
    wire W_lat_valid = (W_lat_frames >= P_LAT_FRAMES_MIN) &&
                       (W_lat_frames <= P_LAT_FRAMES_MAX);

    //-------------------------------------------------------------------------
    // 发布：先在 DDR 域寄存数据，同一时钟沿翻转 toggle 通知像素域。
    //-------------------------------------------------------------------------
    always @(posedge I_clk_ddr or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_lat_frames <= 5'd0;
            O_lat_update <= 1'b0;
        end
        else if (W_display_sof && W_lat_valid) begin
            O_lat_frames <= W_lat_frames;
            O_lat_update <= ~O_lat_update;
        end
    end

endmodule
