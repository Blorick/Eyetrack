//=============================================================================
//  aud_pcm_fifo  --  PCM 采样同步 FIFO（播放器写 / I2S 发送器读）
//
//  模块名   : aud_pcm_fifo
//  功能     : 单时钟域（EG4D 25 MHz）512x16 同步 FIFO，跨"读卡节奏"与
//             "播放节奏"做缓冲。数据段来自 TF 卡（一次 512 字节 = 256 个
//             16bit 采样），I2S 侧按 48 828.125 Hz 每帧取 1 个采样。
//  所属域   : aud
//  时钟域   : I_clk（25 MHz，与 TF 控制器、I2S 发送器同域）
//  复位     : I_rst_n，异步低有效；I_clr 单拍同步清空（播放重新开始时用）
//  参数     : AW（地址位宽，DEPTH = 2^AW = 512）
//
//  为什么需要它
//  ---------------------------------------------------------------------
//  一次 CMD17 读 512 字节需要约 0.66 ms（6.25 MHz SPI），而 512 字节只是
//  256 个采样 = 5.24 ms 的播放量。读卡是"突发"的、播放是"匀速"的，中间
//  必须有缓冲，否则播放会随读卡节奏断续。
//
//  深度选择
//  ---------------------------------------------------------------------
//  512 个采样 = 512/48828.125 = 10.49 ms。最坏情况（一次扇区读 + FAT 扇区
//  读 + 命令开销 ≈ 1.5 ms）也远小于 10.49 ms，所以不会下溢；而一轮读卡
//  只产生 256 个采样，所以"开始读下一扇区前至少空 256 格"即可保证不上溢。
//
//  读数据时序
//  ---------------------------------------------------------------------
//  读端口是**寄存器输出**（BRAM 友好）：I_re 有效后下一拍 O_rdata 有效。
//  I2S 发送器在帧尾（b=63）发起 I_re，到帧头（b=0）才把数据装进当前采样，
//  中间隔 8 个时钟，时序充裕。
//=============================================================================
module aud_pcm_fifo #(
    parameter integer AW = 9                      // 512 深
)(
    input  wire               I_clk,
    input  wire               I_rst_n,
    input  wire               I_clr,              // 1 拍脉冲：清空

    input  wire               I_we,
    input  wire [15:0]        I_wdata,

    input  wire               I_re,
    output reg  [15:0]        O_rdata,

    output wire [AW:0]        O_count,
    output wire               O_empty,
    output wire               O_full,
    output wire               O_space256          // 剩余空间 >= 256（可安全读一扇区）
);
    localparam integer DEPTH = 1 << AW;

    reg [15:0]    S_mem [0:DEPTH-1];
    reg [AW-1:0]  S_wr;
    reg [AW-1:0]  S_rd;
    reg [AW:0]    S_cnt;

    wire W_we_ok = I_we && (S_cnt != DEPTH);
    wire W_re_ok = I_re && (S_cnt != 0);

    assign O_count    = S_cnt;
    assign O_empty    = (S_cnt == 0);
    assign O_full     = (S_cnt == DEPTH);
    assign O_space256 = (S_cnt <= (DEPTH - 256));

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_wr    <= {AW{1'b0}};
            S_rd    <= {AW{1'b0}};
            S_cnt   <= {(AW+1){1'b0}};
            O_rdata <= 16'd0;
        end
        else if (I_clr) begin
            S_wr    <= {AW{1'b0}};
            S_rd    <= {AW{1'b0}};
            S_cnt   <= {(AW+1){1'b0}};
            O_rdata <= 16'd0;
        end
        else begin
            if (W_we_ok) begin
                S_mem[S_wr] <= I_wdata;
                S_wr        <= S_wr + 1'b1;
            end
            if (W_re_ok) begin
                S_rd <= S_rd + 1'b1;
                // 寄存器输出读：I_re 有效后下一拍数据有效
                O_rdata <= S_mem[S_rd];
            end
            // 计数：读写同一拍时保持不变
            if (W_we_ok && !W_re_ok)      S_cnt <= S_cnt + 1'b1;
            else if (!W_we_ok && W_re_ok) S_cnt <= S_cnt - 1'b1;
        end
    end

endmodule
