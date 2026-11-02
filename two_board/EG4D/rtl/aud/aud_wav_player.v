//=============================================================================
//  aud_wav_player  --  TF 卡 FAT16 只读 + WAV 解析 + PCM 流式播放器
//
//  模块名   : aud_wav_player
//  功能     : 收到一次 I_play_req（某个 KEY 按下沿）后，按 I_track 选文件
//             （2026-10-24：曲目号 = KEY 号：
//                KEY1 -> P01.WAV，KEY2 -> P02.WAV（两条警告语音）；
//                KEY5..KEY16 -> P05.WAV .. P16.WAV（12 张车牌语音））：
//               1) 等 TF 卡识别完成（tf_card_ctrl 上电自动识别）；
//               2) CMD17 读 LBA0 引导扇区，从 BPB 解析 FAT16 布局
//                  （保留扇区数、FAT 个数、FAT 扇区数、根目录项数、每簇扇区数）；
//               3) 逐扇区扫描根目录，取第一个扩展名为 WAV 的文件
//                  （短名 8.3 目录项；跳过 LFN/删除项/目录项/卷标项）；
//               4) 沿 FAT16 簇链读取文件数据；
//               5) 扫前 512 字节找 "RIFF"/"WAVE"/"fmt "/"data" 四个标记，
//                  从 fmt 块取格式/声道/位深、从 data 块取长度
//                  （同时兼容 44 字节标准头与 68 字节 WAVE_FORMAT_EXTENSIBLE，
//                   ffmpeg 默认写后者）；
//               6) 从 PCM 起点开始按 16bit 小端把采样推进 FIFO。
//  所属域   : aud
//  时钟域   : I_clk = 25 MHz（与 TF 控制器、I2S 发送器同域，无跨域）
//  复位     : I_rst_n，异步低有效
//  参数     : AW（扇区缓冲地址位宽，与 tf_card_ctrl 的 AW 一致）
//
//  本模块的契约（明确不做什么）
//  ---------------------------------------------------------------------
//  * 只读；只支持 FAT16；只认短名 8.3 目录项，不解析长文件名（扩展名匹配即可）。
//  * 只支持 16bit 单声道 PCM；WAV 头必须全部落在文件第一个扇区内（<512 字节）。
//  * 不写卡、不做磨损均衡、不支持 exFAT/NTFS、不解码 MP3。
//
//  为什么 FAT16 而不是 FAT32
//  ---------------------------------------------------------------------
//  EG4D 上这张 1 GB TF 卡由 Windows 格式化为 FAT16（.NET DriveInfo 报 "FAT"）。
//  FAT16 根目录起点是固定的（保留区 + FAT 区之后），不必像 FAT32 那样先把
//  根目录当簇链去找，状态机少一大块。簇号->LBA 只需要移位+加法：
//      LBA = data_lba + (cluster - 2) << log2(sectors_per_cluster)
//  每簇扇区数永远是 2 的幂，所以**整个模块不需要 32 位乘法器**。
//
//  扇区缓冲的时序契约（来自 tf_card_ctrl / tf_sector_buf）
//  ---------------------------------------------------------------------
//  * O_busy=1 期间绝对不能读缓冲；本模块只在 I_tf_done 之后才开始扫扇区。
//  * 主机口读数据有 1 拍延迟，所以每个字节拆成"给地址 / 下一拍取数"两拍。
//  * 开始读下一个数据扇区前要求 FIFO 剩余空间 >= 256（一个扇区的采样数），
//    这样本轮推入 256 个采样绝不会溢出。
//
//  KEY 重触发
//  ---------------------------------------------------------------------
//  播放中再次按下：TF 控制器忙时不能换命令，所以记到 S_restart，等到下一个
//  可以发命令的点（DATA_REQ / FAT_REQ / ROOT_REQ 等）再从引导扇区重头播放。
//=============================================================================
module aud_wav_player #(
    parameter integer AW = 9,
    parameter integer P_WARNING_MAX_BYTES = 195312
)(
    input  wire              I_clk,
    input  wire              I_rst_n,

    input  wire              I_play_req,      // 1 拍脉冲：从头播放
    input  wire [4:0]        I_track,         // 曲目号：1/2 -> P01/P02.WAV，
                                              //         5..16 -> P05..P16.WAV

    // ---- 扇区缓冲主机口（tf_card_ctrl）----
    output wire [AW-1:0]     O_buf_addr,
    output wire              O_buf_we,
    output wire [7:0]        O_buf_wdata,
    input  wire [7:0]        I_buf_rdata,

    // ---- 命令口（tf_card_ctrl）----
    output reg  [1:0]        O_op,            // 1 = 读扇区
    output reg  [31:0]       O_sector,
    output reg               O_start,
    input  wire              I_tf_ready,
    input  wire              I_tf_busy,
    input  wire              I_tf_done,
    input  wire [3:0]        I_tf_err,

    // ---- PCM 输出 ----
    output reg               O_pcm_we,
    output reg  [15:0]       O_pcm_data,
    input  wire              I_pcm_space,     // FIFO 剩余空间 >= 256
    output reg               O_pcm_clr,       // 1 拍：清空 FIFO

    // ---- 状态 ----
    output reg               O_playing,
    output reg  [3:0]        O_status
);
    assign O_buf_we    = 1'b0;
    assign O_buf_wdata = 8'd0;

    // ------------------------------------------------------------------
    // 状态编码
    // ------------------------------------------------------------------
    localparam [4:0] P_IDLE      = 5'd0,
                     P_START     = 5'd1,
                     P_WAITCARD  = 5'd2,
                     P_BOOT_REQ  = 5'd3,
                     P_BOOT_WAIT = 5'd4,
                     P_BOOT_SCAN = 5'd5,
                     P_BOOT_CALC = 5'd6,
                     P_ROOT_REQ  = 5'd7,
                     P_ROOT_WAIT = 5'd8,
                     P_ROOT_SCAN = 5'd9,
                     P_FILE_REQ  = 5'd10,
                     P_FILE_WAIT = 5'd11,
                     P_HDR_SCAN  = 5'd12,
                     P_HDR_CHK   = 5'd13,
                     P_HDR_CAP   = 5'd14,
                     P_DATA_SPC  = 5'd15,
                     P_DATA_REQ  = 5'd16,
                     P_DATA_WAIT = 5'd17,
                     P_DATA_PROC = 5'd18,
                     P_FAT_REQ   = 5'd19,
                     P_FAT_WAIT  = 5'd20,
                     P_FAT_LO    = 5'd21,
                     P_FAT_HI    = 5'd22,
                     P_END       = 5'd23,
                     P_ERR       = 5'd24;

    // 状态码（对外，便于 OSD/调试口观察）
    localparam [3:0] ST_IDLE   = 4'd0,
                     ST_SEARCH = 4'd1,
                     ST_PLAY   = 4'd2,
                     ST_DONE   = 4'd3,
                     ST_NOCARD = 4'd4,
                     ST_NOFILE = 4'd5,
                     ST_BADWAV = 4'd6,
                     ST_TFERR  = 4'd7,
                     ST_BADFS  = 4'd8;

    // ------------------------------------------------------------------
    // 寄存器
    // ------------------------------------------------------------------
    reg [4:0]    S_st;
    reg          S_ph;              // 0 = 给地址，1 = 取数据（每字节两拍）
    reg [9:0]    S_idx;             // 扇区内字节索引 0..511
    reg          S_restart;

    // BPB 原始字节
    reg [7:0]    S_bps_l, S_bps_h, S_spc, S_rsvd_l, S_rsvd_h, S_nfats;
    reg [7:0]    S_re_l, S_re_h, S_fs_l, S_fs_h;
    // FAT16 布局
    reg [31:0]   S_root_lba, S_data_lba, S_fat_lba;
    reg [15:0]   S_root_secs;
    reg [3:0]    S_spc_log2;
    reg [15:0]   S_root_i;

    // 目录项 / 文件
    reg [7:0]    S_e_first, S_e_attr;
    reg [4:0]    S_track;             // 本次播放的曲目号（在 P_IDLE 锁存）
    reg          S_name_ok;             // 当前目录项 8.3 名字逐字节比对结果
    reg [7:0]    S_e_clu_l, S_e_clu_h;
    reg [31:0]   S_e_size;
    reg [15:0]   S_cluster;
    reg [31:0]   S_file_size;

    // 数据流位置
    reg [31:0]   S_cur_lba;
    reg [8:0]    S_sec_in_clus;
    reg [31:0]   S_pos;             // 文件内字节偏移
    reg [31:0]   S_data_start;      // PCM 起点（文件内偏移）
    reg [31:0]   S_play_end;        // PCM 结束（文件内偏移，不含）
    reg [7:0]    S_lo;
    reg [31:0]   S_shifter;

    // WAV 头扫描结果
    reg          S_riff_ok, S_wave_ok, S_fmt_ok, S_data_ok;
    reg [9:0]    S_fmt_pos, S_data_pos;
    reg [10:0]   S_pcm_off;
    reg [15:0]   S_format, S_channels, S_bits;
    reg [31:0]   S_data_len;
    reg [3:0]    S_cap_i;

    // FAT 读回
    reg [7:0]    S_next_l;

    // 看门狗（25 MHz 下 2^25 约 1.34 s）
    reg [24:0]   S_wd;
    reg [4:0]    S_st_d;

    // ------------------------------------------------------------------
    // 组合计算
    // ------------------------------------------------------------------
    wire [31:0]  W_fat_region = (S_nfats == 8'd2) ? {15'd0, S_fs_h, S_fs_l, 1'b0}
                                                  : {16'd0, S_fs_h, S_fs_l};
    wire [31:0]  W_root_lba   = {16'd0, S_rsvd_h, S_rsvd_l} + W_fat_region;
    wire [15:0]  W_root_ents  = {S_re_h, S_re_l};
    wire [15:0]  W_root_secs  = (W_root_ents + 16'd15) >> 4;
    wire [31:0]  W_data_lba   = W_root_lba + {16'd0, W_root_secs};

    // 簇 -> 首扇区 LBA（当前文件首簇）
    wire [31:0]  W_clu_lba    = S_data_lba + (({16'd0, S_cluster} - 32'd2) << S_spc_log2);

    // FAT16 表项地址：off = cluster * 2（18 bit）
    wire [17:0]  W_fat_off    = {2'b00, S_cluster, 1'b0};
    wire [31:0]  W_fat_sector = S_fat_lba + {14'd0, W_fat_off[17:9]};
    wire [8:0]   W_fat_sub    = W_fat_off[8:0];

    // 旧的“第一个 WAV”匹配逻辑已删除：现在按 S_track 逐字节比对 8.3 名字

    // 本拍从缓冲读到的字节
    wire [7:0]   W_rd         = I_buf_rdata;
    // 特殊键 KEY1/KEY2 最多输出 97,656 个单声道采样；车牌键不截短。
    wire W_warning_track = (S_track == 5'd1) || (S_track == 5'd2);
    wire [31:0] W_data_end = (S_data_len == 32'd0) ? S_file_size :
        ({21'd0, S_pcm_off} + S_data_len);
    wire [31:0] W_warning_end = {21'd0, S_pcm_off} + P_WARNING_MAX_BYTES;

    // ------------------------------------------------------------------
    // 目标 8.3 文件名（11 字节，空格补齐）
    //   I_track 1..2   -> "P01.WAV" / "P02.WAV"（警告语音）
    //   I_track 5..16  -> "P05.WAV" .. "P16.WAV"（车牌语音，与 KEY 号一一对应）
    // 名字匹配逐字节进行，不需要 8.3 名字 ROM。
    // ------------------------------------------------------------------
    // 允许的曲目号：1、2 与 5..16（3、4 在顶层走自检音，不读卡）。
    // 其余值（含 0）映射到 0，必然找不到文件 -> ST_NOFILE。
    function f_track_ok;
        input [4:0] track;
        begin
            f_track_ok = (track == 5'd1) || (track == 5'd2) ||
                         ((track >= 5'd5) && (track <= 5'd16));
        end
    endfunction

    function [7:0] f_name_byte;
        input [4:0] track;
        input [3:0] idx;              // 0..10
        begin
            case (idx)
                4'd0 : f_name_byte = 8'h50;   // 'P'
                4'd1 : f_name_byte = (track >= 5'd10) ? 8'h31 : 8'h30;
                4'd2 : f_name_byte = (track >= 5'd10) ? (8'h30 + (track - 5'd10))
                                                      : (8'h30 + track);
                4'd8 : f_name_byte = 8'h57;   // 'W'
                4'd9 : f_name_byte = 8'h41;   // 'A'
                4'd10: f_name_byte = 8'h56;   // 'V'
                default: f_name_byte = 8'h20;
            endcase
        end
    endfunction

    // FAT 表项拼装（当前拍取高字节）
    wire [15:0]  W_next_clu   = {W_rd, S_next_l};

    // ------------------------------------------------------------------
    // 扇区缓冲读地址（组合）
    //
    // ★ 关键时序（v1 仿真抓到的 bug）：
    //    tf_sector_buf 是"地址当拍给出、读数据在下一拍出现在 O_host_rdata"，
    //    而 O_buf_addr 本身是寄存器时，地址要到下一拍才进 RAM，数据再下一拍
    //    才有效 —— 两拍后才对。原来把 O_buf_addr 当寄存器驱动，结果每次读到
    //    的都是"上一个地址"的内容，引导扇区 BPB 全错（status=8 BADFS）。
    //    改成组合地址后：本拍地址就能被 RAM 采样，下一拍捕获即正确，
    //    现有的两相（给地址 / 取数据）扫描时序刚好对上。
    // ------------------------------------------------------------------
    reg [9:0] W_rd_addr;
    always @(*) begin
        W_rd_addr = S_idx;
        case (S_st)
            P_HDR_CAP: begin
                case (S_cap_i)
                    4'd0: W_rd_addr = S_fmt_pos  + 10'd8;
                    4'd1: W_rd_addr = S_fmt_pos  + 10'd9;
                    4'd2: W_rd_addr = S_fmt_pos  + 10'd10;
                    4'd3: W_rd_addr = S_fmt_pos  + 10'd11;
                    4'd4: W_rd_addr = S_fmt_pos  + 10'd22;
                    4'd5: W_rd_addr = S_fmt_pos  + 10'd23;
                    4'd6: W_rd_addr = S_data_pos + 10'd4;
                    4'd7: W_rd_addr = S_data_pos + 10'd5;
                    4'd8: W_rd_addr = S_data_pos + 10'd6;
                    default: W_rd_addr = S_data_pos + 10'd7;
                endcase
            end
            P_FAT_LO: W_rd_addr = {1'b0, W_fat_sub};
            P_FAT_HI: W_rd_addr = {1'b0, W_fat_sub} + 10'd1;
            default:  W_rd_addr = S_idx;
        endcase
    end
    assign O_buf_addr = W_rd_addr[AW-1:0];

    // ------------------------------------------------------------------
    // 主状态机
    // ------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_st          <= P_IDLE;
            S_ph          <= 1'b0;
            S_idx         <= 10'd0;
            S_restart     <= 1'b0;
            S_bps_l       <= 8'd0;
            S_bps_h       <= 8'd0;
            S_spc         <= 8'd0;
            S_rsvd_l      <= 8'd0;
            S_rsvd_h      <= 8'd0;
            S_nfats       <= 8'd0;
            S_re_l        <= 8'd0;
            S_re_h        <= 8'd0;
            S_fs_l        <= 8'd0;
            S_fs_h        <= 8'd0;
            S_root_lba    <= 32'd0;
            S_data_lba    <= 32'd0;
            S_fat_lba     <= 32'd0;
            S_root_secs   <= 16'd0;
            S_spc_log2    <= 4'd0;
            S_root_i      <= 16'd0;
            S_e_first     <= 8'd0;
            S_track       <= 5'd0;
            S_name_ok     <= 1'b0;
            S_e_attr      <= 8'd0;
            S_e_clu_l     <= 8'd0;
            S_e_clu_h     <= 8'd0;
            S_e_size      <= 32'd0;
            S_cluster     <= 16'd0;
            S_file_size   <= 32'd0;
            S_cur_lba     <= 32'd0;
            S_sec_in_clus <= 9'd0;
            S_pos         <= 32'd0;
            S_data_start  <= 32'd0;
            S_play_end    <= 32'd0;
            S_lo          <= 8'd0;
            S_shifter     <= 32'd0;
            S_riff_ok     <= 1'b0;
            S_wave_ok     <= 1'b0;
            S_fmt_ok      <= 1'b0;
            S_data_ok     <= 1'b0;
            S_fmt_pos     <= 10'd0;
            S_data_pos    <= 10'd0;
            S_pcm_off     <= 11'd0;
            S_format      <= 16'd0;
            S_channels    <= 16'd0;
            S_bits        <= 16'd0;
            S_data_len    <= 32'd0;
            S_cap_i       <= 4'd0;
            S_next_l      <= 8'd0;
            S_wd          <= 25'd0;
            S_st_d        <= P_IDLE;
            O_op          <= 2'd0;
            O_sector      <= 32'd0;
            O_start       <= 1'b0;
            O_pcm_we      <= 1'b0;
            O_pcm_data    <= 16'd0;
            O_pcm_clr     <= 1'b0;
            O_playing     <= 1'b0;
            O_status      <= ST_IDLE;
        end
        else begin
            // 默认单拍脉冲
            O_start   <= 1'b0;
            O_pcm_we  <= 1'b0;
            O_pcm_clr <= 1'b0;

            // 播放中再次按键：等能发命令时从头重播。
            // ★ 必须同时锁存新曲目号，否则重播的永远是 当前这一首。
            if (I_play_req && O_playing) begin
                S_restart <= 1'b1;
                S_track   <= f_track_ok(I_track) ? I_track : 5'd0;
            end

            // 看门狗：只在"等 TF 完成"类状态计数
            if (S_st != S_st_d) begin
                S_st_d <= S_st;
                S_wd   <= 25'd0;
            end
            else if (S_wd != 25'h1FFFFFF) begin
                S_wd <= S_wd + 25'd1;
            end
            else if (S_st != P_IDLE && S_st != P_WAITCARD &&
                     S_st != P_END  && S_st != P_ERR) begin
                O_status <= ST_TFERR;
                S_st     <= P_ERR;
            end

            case (S_st)
            // ==========================================================
            P_IDLE: begin
                O_playing <= 1'b0;
                if (I_play_req) begin
                    O_status  <= ST_SEARCH;
                    O_playing <= 1'b1;
                    S_restart <= 1'b0;
                    S_track   <= f_track_ok(I_track) ? I_track : 5'd0;
                    S_st      <= P_START;
                end
            end

            P_START: begin
                O_pcm_clr <= 1'b1;
                S_idx     <= 10'd0;
                S_ph      <= 1'b0;
                S_st      <= P_WAITCARD;
            end

            // ---- 等卡识别（tf_card_ctrl 上电自己跑识别序列）----
            P_WAITCARD: begin
                if (I_tf_ready) begin
                    S_st <= P_BOOT_REQ;
                end
                else if (I_tf_done && (I_tf_err != 4'd0)) begin
                    // 识别失败：发一次 I_op=0 让控制器重试（用户可换卡后再按）
                    if (!I_tf_busy) begin
                        O_op     <= 2'd0;
                        O_sector <= 32'd0;
                        O_start  <= 1'b1;
                    end
                end
            end

            // ==========================================================
            // 引导扇区
            // ==========================================================
            P_BOOT_REQ: begin
                if (!I_tf_busy) begin
                    O_op     <= 2'd1;
                    O_sector <= 32'd0;
                    O_start  <= 1'b1;
                    S_st     <= P_BOOT_WAIT;
                end
            end

            P_BOOT_WAIT: begin
                if (I_tf_done) begin
                    if (I_tf_err != 4'd0) begin
                        O_status <= ST_TFERR;
                        S_st     <= P_ERR;
                    end
                    else begin
                        S_idx <= 10'd0;
                        S_ph  <= 1'b0;
                        S_st  <= P_BOOT_SCAN;
                    end
                end
            end

            // 顺序读 32 字节，抓 BPB 关键字段
            P_BOOT_SCAN: begin
                if (!S_ph) begin
                    S_ph <= 1'b1;
                end
                else begin
                    S_ph <= 1'b0;
                    case (S_idx)
                        10'd11: S_bps_l  <= W_rd;
                        10'd12: S_bps_h  <= W_rd;
                        10'd13: S_spc    <= W_rd;
                        10'd14: S_rsvd_l <= W_rd;
                        10'd15: S_rsvd_h <= W_rd;
                        10'd16: S_nfats  <= W_rd;
                        10'd17: S_re_l   <= W_rd;
                        10'd18: S_re_h   <= W_rd;
                        10'd22: S_fs_l   <= W_rd;
                        10'd23: S_fs_h   <= W_rd;
                        default: ;
                    endcase
                    if (S_idx == 10'd31) S_st <= P_BOOT_CALC;
                    else                  S_idx <= S_idx + 10'd1;
                end
            end

            P_BOOT_CALC: begin
                if ((S_bps_l != 8'h00) || (S_bps_h != 8'h02)) begin
                    O_status <= ST_BADFS;              // 只支持 512 字节/扇区
                    S_st     <= P_ERR;
                end
                else if ((S_fs_h == 8'd0) && (S_fs_l == 8'd0)) begin
                    O_status <= ST_BADFS;              // 无 FAT16 尺寸 -> FAT32/exFAT
                    S_st     <= P_ERR;
                end
                else if ((S_re_h == 8'd0) && (S_re_l == 8'd0)) begin
                    O_status <= ST_BADFS;              // 根目录项数为 0 -> 非 FAT16
                    S_st     <= P_ERR;
                end
                else begin
                    case (S_spc)
                        8'd1  : S_spc_log2 <= 4'd0;
                        8'd2  : S_spc_log2 <= 4'd1;
                        8'd4  : S_spc_log2 <= 4'd2;
                        8'd8  : S_spc_log2 <= 4'd3;
                        8'd16 : S_spc_log2 <= 4'd4;
                        8'd32 : S_spc_log2 <= 4'd5;
                        8'd64 : S_spc_log2 <= 4'd6;
                        8'd128: S_spc_log2 <= 4'd7;
                        default: S_spc_log2 <= 4'd0;
                    endcase
                    S_root_lba  <= W_root_lba;
                    S_root_secs <= W_root_secs;
                    S_data_lba  <= W_data_lba;
                    S_fat_lba   <= {16'd0, S_rsvd_h, S_rsvd_l};
                    S_root_i    <= 16'd0;
                    S_st        <= P_ROOT_REQ;
                end
            end

            // ==========================================================
            // 根目录扫描（找扩展名 WAV 的文件）
            // ==========================================================
            P_ROOT_REQ: begin
                if (S_restart) begin
                    S_restart <= 1'b0;
                    S_st      <= P_START;
                end
                else if (!I_tf_busy) begin
                    O_op     <= 2'd1;
                    O_sector <= S_root_lba + {16'd0, S_root_i};
                    O_start  <= 1'b1;
                    S_st     <= P_ROOT_WAIT;
                end
            end

            P_ROOT_WAIT: begin
                if (I_tf_done) begin
                    if (I_tf_err != 4'd0) begin
                        O_status <= ST_TFERR;
                        S_st     <= P_ERR;
                    end
                    else begin
                        S_idx <= 10'd0;
                        S_ph  <= 1'b0;
                        S_st  <= P_ROOT_SCAN;
                    end
                end
            end

            P_ROOT_SCAN: begin
                if (!S_ph) begin
                    S_ph <= 1'b1;
                end
                else begin
                    S_ph <= 1'b0;
                    case (S_idx[4:0])
                        5'd0 : begin
                            S_e_first <= W_rd;
                            S_e_attr  <= 8'd0;
                            S_e_size  <= 32'd0;
                            // 每个目录项开头重新开始比对 8.3 名字（0..10 共 11 字节）
                            S_name_ok <= (W_rd == f_name_byte(S_track, 4'd0));
                        end
                        5'd1, 5'd2, 5'd3, 5'd4, 5'd5, 5'd6, 5'd7:
                            if (W_rd != f_name_byte(S_track, S_idx[3:0]))
                                S_name_ok <= 1'b0;
                        5'd8 : if (W_rd != f_name_byte(S_track, 4'd8))  S_name_ok <= 1'b0;
                        5'd9 : if (W_rd != f_name_byte(S_track, 4'd9))  S_name_ok <= 1'b0;
                        5'd10: if (W_rd != f_name_byte(S_track, 4'd10)) S_name_ok <= 1'b0;
                        5'd11: S_e_attr        <= W_rd;
                        5'd26: S_e_clu_l       <= W_rd;
                        5'd27: S_e_clu_h       <= W_rd;
                        5'd28: S_e_size[7:0]   <= W_rd;
                        5'd29: S_e_size[15:8]  <= W_rd;
                        5'd30: S_e_size[23:16] <= W_rd;
                        5'd31: S_e_size[31:24] <= W_rd;
                        default: ;
                    endcase

                    // 每个目录项的最后一个字节处做判定
                    if (S_idx[4:0] == 5'd31) begin
                        if (S_e_first == 8'h00) begin
                            O_status <= ST_NOFILE;                  // 目录结束
                            S_st     <= P_ERR;
                        end
                        else if (S_name_ok &&                       // 8.3 名字完全匹配
                                 ((S_e_attr & 8'h3F) != 8'h0F) &&   // 非 LFN
                                 ((S_e_attr & 8'h18) == 8'h00) &&   // 非目录/卷标
                                 ({S_e_clu_h, S_e_clu_l} != 16'd0)) begin
                            S_cluster   <= {S_e_clu_h, S_e_clu_l};
                            // 本拍 W_rd 才是大小的高字节（S_e_size[31:24] 此时尚未更新）
                            S_file_size <= {W_rd, S_e_size[23:0]};
                            S_st        <= P_FILE_REQ;
                        end
                        else if (S_idx == 10'd511) begin
                            if (({16'd0, S_root_i} + 16'd1) >= S_root_secs) begin
                                O_status <= ST_NOFILE;
                                S_st     <= P_ERR;
                            end
                            else begin
                                S_root_i <= S_root_i + 16'd1;
                                S_st     <= P_ROOT_REQ;
                            end
                        end
                        else begin
                            S_idx <= S_idx + 10'd1;
                        end
                    end
                    else if (S_idx == 10'd511) begin
                        if (({16'd0, S_root_i} + 16'd1) >= S_root_secs) begin
                            O_status <= ST_NOFILE;
                            S_st     <= P_ERR;
                        end
                        else begin
                            S_root_i <= S_root_i + 16'd1;
                            S_st     <= P_ROOT_REQ;
                        end
                    end
                    else begin
                        S_idx <= S_idx + 10'd1;
                    end
                end
            end

            // ==========================================================
            // 文件第一个扇区：扫 WAV 头
            // ==========================================================
            P_FILE_REQ: begin
                if (S_restart) begin
                    S_restart <= 1'b0;
                    S_st      <= P_START;
                end
                else if (!I_tf_busy) begin
                    O_op          <= 2'd1;
                    O_sector      <= W_clu_lba;
                    O_start       <= 1'b1;
                    S_cur_lba     <= W_clu_lba;
                    S_sec_in_clus <= 9'd0;
                    S_st          <= P_FILE_WAIT;
                end
            end

            P_FILE_WAIT: begin
                if (I_tf_done) begin
                    if (I_tf_err != 4'd0) begin
                        O_status <= ST_TFERR;
                        S_st     <= P_ERR;
                    end
                    else begin
                        S_idx     <= 10'd0;
                        S_ph      <= 1'b0;
                        S_shifter <= 32'd0;
                        S_riff_ok <= 1'b0;
                        S_wave_ok <= 1'b0;
                        S_fmt_ok  <= 1'b0;
                        S_data_ok <= 1'b0;
                        S_st      <= P_HDR_SCAN;
                    end
                end
            end

            // 扫 512 字节找标记（用"上一拍移位值 + 本拍字节"比较）
            P_HDR_SCAN: begin
                if (!S_ph) begin
                    S_ph <= 1'b1;
                end
                else begin
                    S_ph      <= 1'b0;
                    S_shifter <= {S_shifter[23:0], W_rd};
                    if ({S_shifter[23:0], W_rd} == 32'h52494646) S_riff_ok <= 1'b1;  // RIFF
                    if ({S_shifter[23:0], W_rd} == 32'h57415645) S_wave_ok <= 1'b1;  // WAVE
                    if (({S_shifter[23:0], W_rd} == 32'h666D7420) && !S_fmt_ok) begin
                        S_fmt_pos <= S_idx - 10'd3;                                   // "fmt "
                        S_fmt_ok  <= 1'b1;
                    end
                    if (({S_shifter[23:0], W_rd} == 32'h64617461) && !S_data_ok) begin
                        S_data_pos <= S_idx - 10'd3;                                  // "data"
                        S_data_ok  <= 1'b1;
                    end
                    if (S_idx == 10'd511) S_st <= P_HDR_CHK;
                    else                  S_idx <= S_idx + 10'd1;
                end
            end

            P_HDR_CHK: begin
                S_pcm_off <= {1'b0, S_data_pos} + 11'd8;
                if (!S_riff_ok || !S_wave_ok || !S_fmt_ok || !S_data_ok ||
                    (({1'b0, S_data_pos} + 11'd8) >= 11'd512)) begin
                    O_status <= ST_BADWAV;
                    S_st     <= P_ERR;
                end
                else begin
                    S_cap_i <= 4'd0;
                    S_ph    <= 1'b0;
                    S_st    <= P_HDR_CAP;
                end
            end

            // 取 fmt/data 字段：format(2) channels(2) bits(2) data_len(4)
            P_HDR_CAP: begin
                if (S_cap_i == 4'd10) begin
                    if (((S_format != 16'h0001) && (S_format != 16'hFFFE)) ||
                        (S_channels != 16'd1) || (S_bits != 16'd16)) begin
                        O_status <= ST_BADWAV;
                        S_st     <= P_ERR;
                    end
                    else begin
                        S_data_start <= {21'd0, S_pcm_off};
                        if (W_warning_track && (W_data_end > W_warning_end))
                            S_play_end <= W_warning_end;
                        else
                            S_play_end <= W_data_end;
                        S_pos     <= {21'd0, S_pcm_off};
                        S_idx     <= S_pcm_off[9:0];                          // 本扇区 PCM 起点
                        S_ph      <= 1'b0;
                        O_status  <= ST_PLAY;
                        S_st      <= P_DATA_PROC;
                    end
                end
                else if (!S_ph) begin
                    S_ph <= 1'b1;
                end
                else begin
                    S_ph <= 1'b0;
                    case (S_cap_i)
                        4'd0: S_format[7:0]     <= W_rd;
                        4'd1: S_format[15:8]    <= W_rd;
                        4'd2: S_channels[7:0]   <= W_rd;
                        4'd3: S_channels[15:8]  <= W_rd;
                        4'd4: S_bits[7:0]       <= W_rd;
                        4'd5: S_bits[15:8]      <= W_rd;
                        4'd6: S_data_len[7:0]   <= W_rd;
                        4'd7: S_data_len[15:8]  <= W_rd;
                        4'd8: S_data_len[23:16] <= W_rd;
                        default: S_data_len[31:24] <= W_rd;
                    endcase
                    S_cap_i <= S_cap_i + 4'd1;
                end
            end

            // ==========================================================
            // 数据扇区：512 字节按 16bit 小端推进 FIFO
            // ==========================================================
            P_DATA_SPC: begin
                if (I_pcm_space) S_st <= P_DATA_REQ;
            end

            P_DATA_REQ: begin
                if (S_restart) begin
                    S_restart <= 1'b0;
                    S_st      <= P_START;
                end
                else if (!I_tf_busy) begin
                    O_op     <= 2'd1;
                    O_sector <= S_cur_lba;
                    O_start  <= 1'b1;
                    S_idx    <= 10'd0;
                    S_ph     <= 1'b0;
                    S_st     <= P_DATA_WAIT;
                end
            end

            P_DATA_WAIT: begin
                if (I_tf_done) begin
                    if (I_tf_err != 4'd0) begin
                        O_status <= ST_TFERR;
                        S_st     <= P_ERR;
                    end
                    else begin
                        S_st <= P_DATA_PROC;
                    end
                end
            end

            P_DATA_PROC: begin
                if (!S_ph) begin
                    S_ph <= 1'b1;
                end
                else begin
                    S_ph <= 1'b0;
                    // 数据区之前（多扇区头）跳过；之后按小端拼采样
                    if (S_pos >= S_data_start) begin
                        if (((S_pos - S_data_start) & 32'd1) == 32'd0)
                            S_lo <= W_rd;
                        else begin
                            O_pcm_we   <= 1'b1;
                            O_pcm_data <= {W_rd, S_lo};
                        end
                    end
                    S_pos <= S_pos + 32'd1;

                    if ((S_pos + 32'd1) >= S_play_end) begin
                        O_status <= ST_DONE;
                        S_st     <= P_END;
                    end
                    else if (S_idx == 10'd511) begin
                        if (({1'b0, S_sec_in_clus} + 9'd1) >= {1'b0, S_spc}) begin
                            S_sec_in_clus <= 9'd0;
                            S_st          <= P_FAT_REQ;
                        end
                        else begin
                            S_sec_in_clus <= S_sec_in_clus + 9'd1;
                            S_cur_lba     <= S_cur_lba + 32'd1;
                            S_st          <= P_DATA_SPC;
                        end
                    end
                    else begin
                        S_idx <= S_idx + 10'd1;
                    end
                end
            end

            // ==========================================================
            // FAT16 簇链
            // ==========================================================
            P_FAT_REQ: begin
                if (S_restart) begin
                    S_restart <= 1'b0;
                    S_st      <= P_START;
                end
                else if (!I_tf_busy) begin
                    O_op     <= 2'd1;
                    O_sector <= W_fat_sector;
                    O_start  <= 1'b1;
                    S_st     <= P_FAT_WAIT;
                end
            end

            P_FAT_WAIT: begin
                if (I_tf_done) begin
                    if (I_tf_err != 4'd0) begin
                        O_status <= ST_TFERR;
                        S_st     <= P_ERR;
                    end
                    else begin
                        S_st <= P_FAT_LO;
                    end
                end
            end

            P_FAT_LO: begin
                if (!S_ph) begin
                    S_ph <= 1'b1;
                end
                else begin
                    S_next_l <= W_rd;
                    S_ph     <= 1'b0;
                    S_st     <= P_FAT_HI;
                end
            end

            P_FAT_HI: begin
                if (!S_ph) begin
                    S_ph <= 1'b1;
                end
                else begin
                    S_ph <= 1'b0;
                    // 链尾 / 坏簇 / 非法值 -> 播放结束
                    if ((W_next_clu >= 16'hFFF8) || (W_next_clu < 16'd2)) begin
                        O_status <= ST_DONE;
                        S_st     <= P_END;
                    end
                    else begin
                        S_cluster     <= W_next_clu;
                        S_cur_lba     <= S_data_lba +
                                         (({16'd0, W_next_clu} - 32'd2) << S_spc_log2);
                        S_sec_in_clus <= 9'd0;
                        S_st          <= P_DATA_SPC;
                    end
                end
            end

            // ==========================================================
            P_END: begin
                O_playing <= 1'b0;
                S_st      <= P_IDLE;
            end

            P_ERR: begin
                O_playing <= 1'b0;
                S_st      <= P_IDLE;
            end

            default: S_st <= P_IDLE;
            endcase
        end
    end

endmodule
