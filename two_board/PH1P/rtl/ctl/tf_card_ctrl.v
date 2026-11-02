//=============================================================================
//  tf_card_ctrl  --  SD/TF 卡 SPI 模式控制器（卡识别 + 单块读 + 单块写）
//
//  模块名   : tf_card_ctrl
//  功能     : 上电自动识别卡（CMD0/CMD8/CMD55+ACMD41/CMD58），之后按命令
//             对指定扇区做单块读（CMD17）/ 单块写（CMD24），512 字节数据
//             经片内扇区缓冲与主机交换
//  所属域   : tf（存储）
//  时钟域   : I_clk（100MHz；SPI 时钟由内部按阶段切换分频得到）
//  复位     : I_rst_n，异步低有效
//  参数     : SPI_W / DIV_INIT / DIV_RUN / TO_BYTES / ACMD41_RETRY /
//             CHECK_RD_CRC / AW
//  验证状态 : 仿真 通过（tb_tf_card_ctrl + tf_sd_card_model）
//             综合/布线/上板 见 docs/design/v1.17-tf-card.md
//
//  实现了协议文档的哪一页
//  ---------------------------------------------------------------------
//  只做 **SPI 模式 + 单块读写**：
//    CMD0  GO_IDLE_STATE      复位进 SPI 模式
//    CMD8  SEND_IF_COND       探测 SD v2（R7 回显 0x1AA）
//    CMD55 + ACMD41           初始化直到卡就绪（R1 从 0x01 变 0x00）
//    CMD58 READ_OCR           取 OCR，读 CCS 位判定**块寻址/字节寻址**
//    CMD17 READ_SINGLE_BLOCK  读 1 个扇区
//    CMD24 WRITE_SINGLE_BLOCK 写 1 个扇区
//  **没有实现**：多块读写（CMD18/CMD25）、高速模式、SD 4-bit 模式、
//  MMC 的 CMD1 初始化路径。对"存校内车牌"这种几百字节量级的用途，单块够用。
//
//  寻址方式（最容易错的一点）
//  ---------------------------------------------------------------------
//  CMD58 读回的 OCR bit30（CCS）：1 = SDHC/SDXC，命令参数是**块号**；
//                                 0 = SDSC，命令参数是**字节地址 = 块号 << 9**。
//  本模块按 O_block_mode 自动换算，主机只管给扇区号。
//
//  主机约定（必须遵守的两条）
//  ---------------------------------------------------------------------
//  1. **O_busy = 1 期间不要访问扇区缓冲**。缓冲是双口的，但忙期间控制器在
//     读/写它，此时从主机口写同一地址结果未定义（标准 TDP-RAM 行为）。
//  2. 一次完整流程：O_busy 上升 -> ... -> O_done 单拍脉冲 + O_busy 落下，
//     O_err 在 O_done 那一拍有效，0 = 成功。
//
//  上电自动识别
//  ---------------------------------------------------------------------
//  复位后控制器自己就跑完整个识别序列，**不需要主机发命令**，此时 O_busy=1。
//  识别成功 O_ready=1 并给一个 O_done；失败则 O_ready=0、O_err 给出原因。
//  失败后主机给一次 I_start（I_op 给 0）可以重试识别。
//
//  内部结构：两类状态，**不能混**
//  ---------------------------------------------------------------------
//  SD SPI 的每一步都是"发一个字节同时收一个字节"，但**并不是每个状态都需要
//  发字节**。这里把状态分成两类，这是本模块最容易写错的地方：
//
//   (A) 交换态（三相）：
//         PH_ISSUE  把本状态要发的字节交给 tf_spi_master
//         PH_WAIT   等 O_done，锁存收到的字节
//         PH_DECIDE 依据收到的字节决定下一个状态
//       共 13 个：PWRUP / CMD_SEND / CMD_RESP / RESP4 / RD_WAIT / RD_DATA /
//       RD_CRC / WR_WAIT / WR_TOKEN / WR_DATA / WR_CRC / WR_RESP / WR_BUSY
//
//   (B) 判断态（**单周期，绝对不发字节**）：只依据上一拍锁存好的
//       S_r1 / S_r4 决定跳转。共 9 个：C0_CHK / C8_CHK / C8_R7 /
//       C55_CHK / C41_CHK / C58_CHK / C58_R3 / RD_R1 / WR_R1
//
//  ★ 为什么必须分开（这是本模块第一个被仿真抓到的真 bug）：
//    最初让判断态也走三相，于是每进一次判断态就在总线上**多送一个 0xFF**。
//    后果不是"报错"，而是命令帧整体**错位一个字节**：CMD0 之后模型收到的
//    下一个命令被解成 `CMD=63 arg=48000001`（0xFF 被当成命令字节，0x48 被
//    当成 arg 最高字节）。卡会回非法命令，初始化直接失败。
//    字节错位这类问题**不会**被"状态机跑到了 ST_FINISH"发现，
//    只有对着协议模型逐字节比对才能抓到。
//
//  写扇区时缓冲字节的**预取**
//  ---------------------------------------------------------------------
//  扇区缓冲有 1 拍读延迟，而发送字节发生在 PH_ISSUE。所以在 ST_WR_WAIT 阶段
//  先把地址打到 0、在它的 PH_DECIDE 把第 0 字节装进 S_tx_data；
//  之后每个 ST_WR_DATA 的 PH_ISSUE 把地址 +1（一次字节交换有 80+ 个周期，
//  地址变化早就反映到读数据上），PH_DECIDE 再把下一字节装进 S_tx_data。
//  这样任何时刻都不需要组合读 RAM。
//=============================================================================
module tf_card_ctrl #(
    parameter integer        SPI_W        = 16,
    parameter [SPI_W-1:0]    DIV_INIT     = 16'd249,     // 100MHz/(2*250) = 200kHz
    parameter [SPI_W-1:0]    DIV_RUN      = 16'd4,       // 100MHz/(2*5)   = 10MHz
    parameter [15:0]         TO_BYTES     = 16'd60000,   // 等令牌/忙 的超时（字节数）
    parameter [15:0]         ACMD41_RETRY = 16'd2000,    // ACMD41 重试次数
    parameter                CHECK_RD_CRC = 1'b1,        // 1 = 校验读回数据的 CRC16
    parameter integer        AW           = 9            // 扇区缓冲地址位宽
)(
    input  wire              I_clk,
    input  wire              I_rst_n,

    // ---- 主机口：扇区缓冲 ----
    input  wire              I_buf_we,
    input  wire [AW-1:0]     I_buf_addr,
    input  wire [7:0]        I_buf_wdata,
    output wire [7:0]        O_buf_rdata,

    // ---- 主机口：命令 ----
    input  wire [1:0]        I_op,        // 0=无 1=读扇区 2=写扇区
    input  wire [31:0]       I_sector,    // 扇区号（不是字节地址）
    input  wire              I_start,     // 1 拍脉冲

    // ---- 状态 ----
    output reg               O_ready,
    output reg               O_busy,
    output reg               O_done,
    output reg  [3:0]        O_err,
    output reg  [1:0]        O_card_type, // 0=未知 1=SDv1/MMC 2=SDv2
    output reg               O_block_mode,
    output reg  [7:0]        O_state,     // 调试：状态编号
    output reg  [31:0]       O_ocr,       // 调试：CMD58 的 OCR

    // ---- 物理层（直接接 TF 卡座）----
    output wire              O_tf_clk,
    output wire              O_tf_cs_n,
    output wire              O_tf_mosi,
    input  wire              I_tf_miso
);

    //-------------------------------------------------------------------------
    // 命令索引与错误码
    //-------------------------------------------------------------------------
    localparam [5:0] P_CMD0  = 6'd0,
                     P_CMD8  = 6'd8,
                     P_CMD17 = 6'd17,
                     P_CMD24 = 6'd24,
                     P_CMD55 = 6'd55,
                     P_CMD58 = 6'd58,
                     P_ACMD41= 6'd41;

    localparam [3:0] P_ERR_NONE  = 4'd0,
                     P_ERR_CMD0  = 4'd1,
                     P_ERR_CMD8  = 4'd2,
                     P_ERR_INIT  = 4'd3,
                     P_ERR_R1    = 4'd4,
                     P_ERR_TOKEN = 4'd5,
                     P_ERR_RESP  = 4'd6,
                     P_ERR_BUSY  = 4'd7,
                     P_ERR_CMD58 = 4'd8,
                     P_ERR_CRC   = 4'd9;

    //-------------------------------------------------------------------------
    // 状态
    //-------------------------------------------------------------------------
    localparam [4:0] ST_PWRUP    = 5'd0,
                     ST_CMD_SEND = 5'd1,
                     ST_CMD_RESP = 5'd2,
                     ST_RESP4    = 5'd3,
                     ST_FINISH   = 5'd4,
                     ST_ERR      = 5'd5,
                     ST_RD_WAIT  = 5'd6,
                     ST_RD_DATA  = 5'd7,
                     ST_RD_CRC   = 5'd8,
                     ST_WR_WAIT  = 5'd9,
                     ST_WR_TOKEN = 5'd10,
                     ST_WR_DATA  = 5'd11,
                     ST_WR_CRC   = 5'd12,
                     ST_WR_RESP  = 5'd13,
                     ST_WR_BUSY  = 5'd14,
                     ST_IDLE     = 5'd15,
                     // ---- 判断态（不发字节）----
                     ST_C0_CHK   = 5'd16,
                     ST_C8_CHK   = 5'd17,
                     ST_C8_R7    = 5'd18,
                     ST_C55_CHK  = 5'd19,
                     ST_C41_CHK  = 5'd20,
                     ST_C58_CHK  = 5'd21,
                     ST_C58_R3   = 5'd22,
                     ST_RD_R1    = 5'd23,
                     ST_WR_R1    = 5'd24;

    localparam [1:0] PH_ISSUE = 2'd0, PH_WAIT = 2'd1, PH_DECIDE = 2'd2;

    reg [4:0]    S_st;
    reg [1:0]    S_ph;
    reg [4:0]    S_after;
    reg [5:0]    S_cmd;
    reg [31:0]   S_arg;
    reg [7:0]    S_r1;
    reg [31:0]   S_r4;
    reg [15:0]   S_n;
    reg [15:0]   S_to;
    reg [15:0]   S_retry;
    reg [15:0]   S_crc16;      // 数据块 CRC16（写=要发的 / 读=算出来的）
    reg [15:0]   S_rxcrc;      // 读回时卡发来的 CRC16
    reg [7:0]    S_tx_data;    // 写扇区时的预取字节
    reg [AW-1:0] S_buf_a;      // 控制器侧缓冲读地址（预取）
    reg          S_ctl_we;
    reg [AW-1:0] S_ctl_addr;
    reg [7:0]    S_ctl_wdata;
    reg [7:0]    S_rx;
    reg          S_inited;
    reg          S_fast;       // 1 = SPI 走 DIV_RUN
    reg          S_blk_mode;
    reg [1:0]    S_card;
    reg [3:0]    S_err;
    reg          S_txgo;
    reg [7:0]    W_issue;

    //-------------------------------------------------------------------------
    // CRC7（命令帧）+ CRC16（数据块）
    //-------------------------------------------------------------------------
    function [6:0] crc7_bit;
        input [6:0] c_in;
        input       d;
        begin
            if(c_in[6] ^ d) crc7_bit = {c_in[5:0], 1'b0} ^ 7'h09;
            else            crc7_bit = {c_in[5:0], 1'b0};
        end
    endfunction

    function [6:0] crc7_byte;
        input [6:0] c_in;
        input [7:0] d;
        integer i;
        reg [6:0] c;
        begin
            c = c_in;
            for(i = 7; i >= 0; i = i - 1)
                c = crc7_bit(c, d[i]);
            crc7_byte = c;
        end
    endfunction

    function [6:0] crc7_cmd;
        input [5:0]  idx;
        input [31:0] arg;
        reg [6:0] c;
        begin
            c = 7'd0;
            c = crc7_byte(c, {2'b01, idx});
            c = crc7_byte(c, arg[31:24]);
            c = crc7_byte(c, arg[23:16]);
            c = crc7_byte(c, arg[15:8]);
            c = crc7_byte(c, arg[7:0]);
            crc7_cmd = c;
        end
    endfunction

    function [15:0] crc16_byte;
        input [15:0] c_in;
        input [7:0]  d;
        integer i;
        reg [15:0] c;
        begin
            c = c_in;
            for(i = 7; i >= 0; i = i - 1) begin
                if(c[15] ^ d[i]) c = {c[14:0], 1'b0} ^ 16'h1021;
                else             c = {c[14:0], 1'b0};
            end
            crc16_byte = c;
        end
    endfunction

    //-------------------------------------------------------------------------
    // 判断态（不发字节）—— 见文件头"两类状态，不能混"
    //-------------------------------------------------------------------------
    wire W_is_check = (S_st == ST_C0_CHK)  || (S_st == ST_C8_CHK)  ||
                      (S_st == ST_C8_R7)   || (S_st == ST_C55_CHK) ||
                      (S_st == ST_C41_CHK) || (S_st == ST_C58_CHK) ||
                      (S_st == ST_C58_R3)  || (S_st == ST_RD_R1)  ||
                      (S_st == ST_WR_R1);

    //-------------------------------------------------------------------------
    // SPI 时钟分频：识别阶段慢、数据阶段快
    //-------------------------------------------------------------------------
    wire [SPI_W-1:0] W_div = S_fast ? DIV_RUN : DIV_INIT;

    //-------------------------------------------------------------------------
    // 物理层
    //-------------------------------------------------------------------------
    wire       W_spi_done;
    wire [7:0] W_spi_rx;

    tf_spi_master #(.DIV_W(SPI_W)) u_spi (
        .I_clk      ( I_clk      ),
        .I_rst_n    ( I_rst_n    ),
        .I_div      ( W_div      ),
        .I_tx_start ( S_txgo     ),
        .I_tx_byte  ( W_issue    ),
        .O_rx_byte  ( W_spi_rx   ),
        .O_done     ( W_spi_done ),
        .O_busy     (            ),
        .O_sclk     ( O_tf_clk   ),
        .O_mosi     ( O_tf_mosi  ),
        .I_miso     ( I_tf_miso  )
    );

    // CS：只在"空闲"和"上电 74 时钟"期间拉高，其余时间保持低
    assign O_tf_cs_n = (S_st == ST_IDLE) || (S_st == ST_PWRUP);

    //-------------------------------------------------------------------------
    // 扇区缓冲
    //   tf_sector_buf 的控制口只有**一根地址线**（读地址与写地址复用），所以这里
    //   按写使能分时复用：
    //     S_ctl_we=1（读扇区时把收到的字节写进缓冲）-> 用写地址 S_ctl_addr
    //     S_ctl_we=0（其余时间，包括写扇区时的预取）-> 用读地址 S_buf_a
    //   ★ 这一处曾经接错成"控制口地址恒接 S_ctl_addr"，后果是：写扇区时
    //     预取地址根本不生效，W_ctl_rdata 一直停在上一次读命令最后访问的那个
    //     地址的内容上，于是**整块 512 字节写出去是同一个常数**。
    //     症状在仿真里非常典型：回读 512 字节全部等于同一值，且该值等于
    //     上一操作残留地址处的数据。这种错不会让状态机报错，只能靠数据比对发现。
    //-------------------------------------------------------------------------
    wire [7:0]       W_ctl_rdata;
    wire [AW-1:0]    W_ctl_addr = S_ctl_we ? S_ctl_addr : S_buf_a;

    tf_sector_buf #(.AW(AW)) u_buf (
        .I_clk        ( I_clk       ),
        .I_rst_n      ( I_rst_n     ),
        .I_host_we    ( I_buf_we    ),
        .I_host_addr  ( I_buf_addr  ),
        .I_host_wdata ( I_buf_wdata ),
        .O_host_rdata ( O_buf_rdata ),
        .I_ctl_we     ( S_ctl_we    ),
        .I_ctl_addr   ( W_ctl_addr  ),
        .I_ctl_wdata  ( S_ctl_wdata ),
        .O_ctl_rdata  ( W_ctl_rdata )
    );

    //-------------------------------------------------------------------------
    // 本状态要发出的字节（只在交换态有意义）
    //-------------------------------------------------------------------------
    always @(*) begin
        case(S_st)
            ST_PWRUP    : W_issue = 8'hff;

            ST_CMD_SEND : begin
                case(S_n)
                    16'd0   : W_issue = {2'b01, S_cmd};
                    16'd1   : W_issue = S_arg[31:24];
                    16'd2   : W_issue = S_arg[23:16];
                    16'd3   : W_issue = S_arg[15:8];
                    16'd4   : W_issue = S_arg[7:0];
                    default : W_issue = {crc7_cmd(S_cmd, S_arg), 1'b1};
                endcase
            end

            ST_WR_DATA  : W_issue = S_tx_data;
            ST_WR_TOKEN : W_issue = 8'hfe;
            ST_WR_CRC   : W_issue = (S_n == 16'd0) ? S_crc16[15:8] : S_crc16[7:0];

            default     : W_issue = 8'hff;
        endcase
    end

    //-------------------------------------------------------------------------
    // 主状态机
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_st        <= ST_PWRUP;
            S_ph        <= PH_ISSUE;
            S_after     <= ST_FINISH;
            S_cmd       <= 6'd0;
            S_arg       <= 32'd0;
            S_r1        <= 8'hff;
            S_r4        <= 32'd0;
            S_n         <= 16'd0;
            S_to        <= 16'd0;
            S_retry     <= ACMD41_RETRY;
            S_crc16     <= 16'hffff;
            S_rxcrc     <= 16'h0000;
            S_tx_data   <= 8'd0;
            S_buf_a     <= {AW{1'b0}};
            S_ctl_we    <= 1'b0;
            S_ctl_addr  <= {AW{1'b0}};
            S_ctl_wdata <= 8'd0;
            S_rx        <= 8'hff;
            S_inited    <= 1'b0;
            S_fast      <= 1'b0;
            S_blk_mode  <= 1'b0;
            S_card      <= 2'd0;
            S_err       <= P_ERR_NONE;
            S_txgo      <= 1'b0;
            O_ready      <= 1'b0;
            O_busy       <= 1'b1;      // 上电自动识别期间为忙
            O_done       <= 1'b0;
            O_err        <= P_ERR_NONE;
            O_card_type  <= 2'd0;
            O_block_mode <= 1'b0;
            O_state      <= ST_PWRUP;
            O_ocr        <= 32'd0;
        end
        else begin
            O_done   <= 1'b0;
            S_txgo   <= 1'b0;
            S_ctl_we <= 1'b0;
            O_state  <= S_st;

            case(S_st)
            // ============================================================
            // 空闲
            // ============================================================
            ST_IDLE: begin
                S_ph <= PH_ISSUE;
                if(I_start) begin
                    O_err <= P_ERR_NONE;
                    S_err <= P_ERR_NONE;
                    if(!S_inited) begin
                        O_busy  <= 1'b1;
                        S_fast  <= 1'b0;
                        S_st    <= ST_PWRUP;
                        S_n     <= 16'd0;
                        S_retry <= ACMD41_RETRY;
                    end
                    else if(I_op == 2'd1) begin
                        O_busy  <= 1'b1;
                        S_cmd   <= P_CMD17;
                        S_arg   <= S_blk_mode ? I_sector : (I_sector << 9);
                        S_after <= ST_RD_R1;
                        S_st    <= ST_CMD_SEND;
                        S_n     <= 16'd0;
                    end
                    else if(I_op == 2'd2) begin
                        O_busy  <= 1'b1;
                        S_cmd   <= P_CMD24;
                        S_arg   <= S_blk_mode ? I_sector : (I_sector << 9);
                        S_after <= ST_WR_R1;
                        S_st    <= ST_CMD_SEND;
                        S_n     <= 16'd0;
                    end
                    else begin
                        O_busy <= 1'b0;    // I_op 非法：不产生任何动作
                    end
                end
            end

            // ============================================================
            // 收尾
            // ============================================================
            ST_FINISH: begin
                O_done <= 1'b1;
                O_busy <= 1'b0;
                S_st   <= ST_IDLE;
                S_ph   <= PH_ISSUE;
            end

            // ============================================================
            // 出错
            // ============================================================
            ST_ERR: begin
                O_err  <= S_err;
                O_done <= 1'b1;
                O_busy <= 1'b0;
                S_st   <= ST_IDLE;
                S_ph   <= PH_ISSUE;
            end

            // ============================================================
            // 判断态：单周期，**不发字节**
            // ============================================================
            default: begin
                if(W_is_check) begin
                    S_ph <= PH_ISSUE;
                    case(S_st)
                    //---- CMD0：期望 R1 = 0x01 ----
                    ST_C0_CHK: begin
                        if(S_r1 == 8'h01) begin
                            S_cmd   <= P_CMD8;
                            S_arg   <= 32'h0000_01AA;
                            S_after <= ST_C8_CHK;
                            S_st    <= ST_CMD_SEND;
                        end
                        else if(S_retry > 16'd0) begin
                            S_retry <= S_retry - 16'd1;
                            S_cmd   <= P_CMD0;
                            S_arg   <= 32'd0;
                            S_after <= ST_C0_CHK;
                            S_st    <= ST_CMD_SEND;
                        end
                        else begin
                            S_err <= P_ERR_CMD0;
                            S_st  <= ST_ERR;
                        end
                    end

                    //---- CMD8：0x01 -> SD v2 收 R7；0x05 -> 老卡/MMC ----
                    ST_C8_CHK: begin
                        if(S_r1 == 8'h01) begin
                            S_st    <= ST_RESP4;
                            S_after <= ST_C8_R7;
                        end
                        else if(S_r1 == 8'h05) begin
                            S_card  <= 2'd1;
                            S_cmd   <= P_CMD55;
                            S_arg   <= 32'd0;
                            S_after <= ST_C55_CHK;
                            S_st    <= ST_CMD_SEND;
                        end
                        else begin
                            S_err <= P_ERR_CMD8;
                            S_st  <= ST_ERR;
                        end
                    end

                    //---- R7 回显校验：低 12 位必须 0x1AA ----
                    ST_C8_R7: begin
                        if(S_r4[11:0] == 12'h1aa) begin
                            S_card  <= 2'd2;
                            S_cmd   <= P_CMD55;
                            S_arg   <= 32'd0;
                            S_after <= ST_C55_CHK;
                            S_st    <= ST_CMD_SEND;
                        end
                        else begin
                            S_err <= P_ERR_CMD8;    // 电压不被支持
                            S_st  <= ST_ERR;
                        end
                    end

                    //---- CMD55：接受 0x00/0x01 ----
                    ST_C55_CHK: begin
                        if((S_r1 == 8'h00) || (S_r1 == 8'h01)) begin
                            S_cmd   <= P_ACMD41;
                            S_arg   <= (S_card == 2'd2) ? 32'h4000_0000 : 32'd0;
                            S_after <= ST_C41_CHK;
                            S_st    <= ST_CMD_SEND;
                        end
                        else begin
                            S_err <= P_ERR_INIT;
                            S_st  <= ST_ERR;
                        end
                    end

                    //---- ACMD41：0x00 = 就绪；否则重发 CMD55+ACMD41 ----
                    ST_C41_CHK: begin
                        if(S_r1 == 8'h00) begin
                            S_cmd   <= P_CMD58;
                            S_arg   <= 32'd0;
                            S_after <= ST_C58_CHK;
                            S_st    <= ST_CMD_SEND;
                        end
                        else if(S_retry > 16'd0) begin
                            S_retry <= S_retry - 16'd1;
                            S_cmd   <= P_CMD55;
                            S_arg   <= 32'd0;
                            S_after <= ST_C55_CHK;
                            S_st    <= ST_CMD_SEND;
                        end
                        else begin
                            S_err <= P_ERR_INIT;
                            S_st  <= ST_ERR;
                        end
                    end

                    //---- CMD58：R1 = 0x00 后收 OCR ----
                    ST_C58_CHK: begin
                        if(S_r1 == 8'h00) begin
                            S_st    <= ST_RESP4;
                            S_after <= ST_C58_R3;
                        end
                        else begin
                            S_err <= P_ERR_CMD58;
                            S_st  <= ST_ERR;
                        end
                    end

                    ST_C58_R3: begin
                        O_ocr        <= S_r4;
                        S_blk_mode   <= S_r4[30];    // CCS：1 = 块寻址
                        O_block_mode <= S_r4[30];
                        O_card_type  <= S_card;
                        O_ready      <= 1'b1;
                        S_inited     <= 1'b1;
                        S_fast       <= 1'b1;        // 之后走 DIV_RUN
                        S_st         <= ST_FINISH;
                    end

                    //---- CMD17 的 R1 ----
                    ST_RD_R1: begin
                        if(S_r1 == 8'h00) begin
                            S_st <= ST_RD_WAIT;
                            S_n  <= 16'd0;
                            S_to <= 16'd0;
                        end
                        else begin
                            S_err <= P_ERR_R1;
                            S_st  <= ST_ERR;
                        end
                    end

                    //---- CMD24 的 R1，并开始预取第 0 个数据字节 ----
                    ST_WR_R1: begin
                        if(S_r1 == 8'h00) begin
                            S_st    <= ST_WR_WAIT;
                            S_n     <= 16'd0;
                            S_to    <= 16'd0;
                            S_buf_a <= {AW{1'b0}};
                        end
                        else begin
                            S_err <= P_ERR_R1;
                            S_st  <= ST_ERR;
                        end
                    end

                    default: S_st <= ST_FINISH;
                    endcase
                end
                // ========================================================
                // 交换态：三相
                // ========================================================
                else begin
                    case(S_ph)
                    PH_ISSUE: begin
                        S_txgo <= 1'b1;
                        // 写数据阶段：本拍把预取地址 +1（一次交换 80+ 个周期，
                        // 地址变化在 PH_DECIDE 之前早就反映到 W_ctl_rdata 上）
                        if(S_st == ST_WR_DATA)
                            S_buf_a <= S_buf_a + 1'b1;
                        S_ph <= PH_WAIT;
                    end

                    PH_WAIT: begin
                        if(W_spi_done) begin
                            S_rx <= W_spi_rx;
                            // 读数据：收到的字节直接写进缓冲（用当前 S_n）
                            if(S_st == ST_RD_DATA) begin
                                S_ctl_we    <= 1'b1;
                                S_ctl_addr  <= S_n[AW-1:0];
                                S_ctl_wdata <= W_spi_rx;
                            end
                            S_ph <= PH_DECIDE;
                        end
                    end

                    PH_DECIDE: begin
                        S_ph <= PH_ISSUE;

                        case(S_st)
                        //---- 上电：CS 高，发 10 个 0xFF = 80 个时钟 ----
                        ST_PWRUP: begin
                            if(S_n >= 16'd9) begin
                                S_cmd   <= P_CMD0;
                                S_arg   <= 32'd0;
                                S_after <= ST_C0_CHK;
                                S_st    <= ST_CMD_SEND;
                                S_n     <= 16'd0;
                            end
                            else begin
                                S_n <= S_n + 16'd1;
                            end
                        end

                        //---- 发命令：6 字节 ----
                        ST_CMD_SEND: begin
                            if(S_n >= 16'd5) begin
                                S_st <= ST_CMD_RESP;
                                S_n  <= 16'd0;
                                S_to <= 16'd0;
                            end
                            else begin
                                S_n <= S_n + 16'd1;
                            end
                        end

                        //---- 等 R1 ----
                        ST_CMD_RESP: begin
                            if(S_rx[7] == 1'b0) begin
                                S_r1 <= S_rx;
                                S_st <= S_after;
                                S_n  <= 16'd0;
                                S_to <= 16'd0;
                            end
                            else begin
                                S_to <= S_to + 16'd1;
                                if(S_to >= TO_BYTES) begin
                                    S_err <= (S_cmd == P_CMD0) ? P_ERR_CMD0 : P_ERR_R1;
                                    S_st  <= ST_ERR;
                                end
                            end
                        end

                        //---- 收 R3 / R7 的 4 个附加字节 ----
                        ST_RESP4: begin
                            S_r4 <= {S_r4[23:0], S_rx};
                            if(S_n >= 16'd3) begin
                                S_st <= S_after;
                                S_n  <= 16'd0;
                            end
                            else begin
                                S_n <= S_n + 16'd1;
                            end
                        end

                        //---- 读：等数据令牌 ----
                        ST_RD_WAIT: begin
                            if(S_rx == 8'hfe) begin
                                S_st    <= ST_RD_DATA;
                                S_n     <= 16'd0;
                                S_crc16 <= 16'h0000;
                            end
                            else if(S_rx != 8'hff) begin
                                S_err <= P_ERR_TOKEN;
                                S_st  <= ST_ERR;
                            end
                            else begin
                                S_to <= S_to + 16'd1;
                                if(S_to >= TO_BYTES) begin
                                    S_err <= P_ERR_TOKEN;
                                    S_st  <= ST_ERR;
                                end
                            end
                        end

                        //---- 读：512 字节 ----
                        ST_RD_DATA: begin
                            S_crc16 <= crc16_byte(S_crc16, S_rx);
                            if(S_n >= 16'd511) begin
                                S_st    <= ST_RD_CRC;
                                S_n     <= 16'd0;
                                S_rxcrc <= 16'h0000;
                            end
                            else begin
                                S_n <= S_n + 16'd1;
                            end
                        end

                        //---- 读：2 字节 CRC16 ----
                        ST_RD_CRC: begin
                            if(S_n == 16'd0) begin
                                S_rxcrc <= {S_rxcrc[7:0], S_rx};
                                S_n     <= 16'd1;
                            end
                            else begin
                                if(CHECK_RD_CRC && (S_crc16 != {S_rxcrc[7:0], S_rx})) begin
                                    S_err <= P_ERR_CRC;
                                    S_st  <= ST_ERR;
                                end
                                else begin
                                    S_st <= ST_FINISH;
                                end
                                S_n <= 16'd0;
                            end
                        end

                        //---- 写：Nwr，同时把第 0 字节装进 S_tx_data ----
                        ST_WR_WAIT: begin
                            S_tx_data <= W_ctl_rdata;   // = 缓冲[0]
                            S_st      <= ST_WR_TOKEN;
                        end

                        //---- 写：数据令牌 0xFE ----
                        ST_WR_TOKEN: begin
                            S_st    <= ST_WR_DATA;
                            S_n     <= 16'd0;
                            S_crc16 <= 16'h0000;
                        end

                        //---- 写：512 字节 + 累计 CRC16 + 预取下一字节 ----
                        ST_WR_DATA: begin
                            S_crc16   <= crc16_byte(S_crc16, S_tx_data);
                            S_tx_data <= W_ctl_rdata;   // 地址已在 PH_ISSUE 里 +1
                            if(S_n >= 16'd511) begin
                                S_st <= ST_WR_CRC;
                                S_n  <= 16'd0;
                            end
                            else begin
                                S_n <= S_n + 16'd1;
                            end
                        end

                        //---- 写：2 字节 CRC16 ----
                        ST_WR_CRC: begin
                            if(S_n >= 16'd1) begin
                                S_st <= ST_WR_RESP;
                                S_n  <= 16'd0;
                                S_to <= 16'd0;
                            end
                            else begin
                                S_n <= S_n + 16'd1;
                            end
                        end

                        //---- 写：数据响应，xxx0_0101 = 接受 ----
                        ST_WR_RESP: begin
                            if(S_rx[4:0] == 5'b00101) begin
                                S_st <= ST_WR_BUSY;
                                S_to <= 16'd0;
                            end
                            else begin
                                S_err <= P_ERR_RESP;
                                S_st  <= ST_ERR;
                            end
                        end

                        //---- 写：等卡把 MISO 放高 ----
                        ST_WR_BUSY: begin
                            if(S_rx == 8'hff) begin
                                S_st <= ST_FINISH;
                            end
                            else begin
                                S_to <= S_to + 16'd1;
                                if(S_to >= TO_BYTES) begin
                                    S_err <= P_ERR_BUSY;
                                    S_st  <= ST_ERR;
                                end
                            end
                        end

                        default: S_st <= ST_FINISH;
                        endcase
                    end
                    endcase
                end
            end
            endcase
        end
    end

endmodule
