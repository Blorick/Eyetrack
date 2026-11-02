//=============================================================================
//  aud_i2s_tx  --  I2S 主模式发送器（16bit 数据放在 32bit 时隙里的标准 I2S）
//
//  模块名   : aud_i2s_tx
//  功能     : 从 PCM FIFO 取采样，按 Philips I2S 标准时序输出 BCLK/LRCLK/DIN
//  所属域   : aud
//  时钟域   : I_clk = 25 MHz（EG4D 板载晶振；不新增 PLL、不新增时钟域）
//  复位     : I_rst_n，异步低有效
//
//  时钟关系（全部由 25 MHz 整数分频得到，零抖动）
//  ---------------------------------------------------------------------
//      BCLK  = 25 MHz / 8  = 3.125 MHz      （S_cnt 3 bit，O_bclk = S_cnt[2]）
//      LRCLK = BCLK / 64   = 48 828.125 Hz  （S_bit 6 bit，O_lrclk = S_bit[5]）
//      每帧 64 个 BCLK：左时隙 32、右时隙 32
//      -> 25 MHz 同时被 8 与 512 整除，两级分频都是精确值，没有累加误差。
//
//  为什么用 32bit 时隙
//  ---------------------------------------------------------------------
//  I2S 规定 LRCLK 在 MSB 前 1 个 BCLK 翻转。若直接用 16bit 时隙（每帧 32
//  BCLK），LSB 会落进下一个 LRCLK 半帧，边界容易做错。用 32bit 时隙后：
//      时隙 1..16   = 左声道 data[15:0]（MSB 先出）
//      时隙 17..32  = 0
//      时隙 33..48  = 右声道 data[15:0]
//      时隙 49..64(=0) = 0
//  16bit 采样左对齐放进 32bit 时隙，等价于"采样放大 2^16"，MAX98357A 支持
//  16/24/32bit 字长，听感完全一致。
//
//  单声道处理
//  ---------------------------------------------------------------------
//  同一个采样同时写进左、右两个时隙。这样无论功放 SD_MODE 被配成
//  "左声道 / 右声道 / (L+R)/2"，输出都是同一个信号，不依赖模块上的电阻接法。
//
//  空 FIFO 行为
//  ---------------------------------------------------------------------
//  FIFO 空时 DIN=0（静音），但 BCLK/LRCLK 继续运行 —— MAX98357A 靠 BCLK
//  工作，停时钟会进入关断并可能爆音。
//
//  FIFO 读时序
//  ---------------------------------------------------------------------
//  aud_pcm_fifo 是寄存器输出读：I_re 有效后下一拍 O_rdata 有效。这里在帧尾
//  （时隙 63 的下降沿）发起 I_re，在 S_cnt==1 时装入 S_cur，到下一个下降沿
//  （时隙 1）才用 MSB，中间隔 7 个时钟，余量充足。
//=============================================================================
module aud_i2s_tx (
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_fifo_empty,
    input  wire [15:0] I_fifo_dout,
    output reg         O_fifo_re,

    output reg         O_bclk,
    output reg         O_lrclk,
    output reg         O_din,
    output wire        O_active
);
    reg [2:0]  S_cnt;          // 0..7：一个 BCLK 周期
    reg [5:0]  S_bit;          // 0..63：一帧内时隙号
    reg [15:0] S_shift;
    reg [15:0] S_cur;          // 本帧采样
    reg        S_have;         // 本帧有有效采样
    assign O_active = S_have;

    wire [5:0] W_bit_next    = S_bit + 6'd1;        // 本下降沿之后进入的时隙
    wire       W_frame_start = (S_bit == 6'd63);    // 本下降沿 = 帧边界
    wire       W_bit_load_l  = (W_bit_next == 6'd1);
    wire       W_bit_load_r  = (W_bit_next == 6'd33);
    wire       W_bit_shift_l = (W_bit_next >= 6'd2)  && (W_bit_next <= 6'd16);
    wire       W_bit_shift_r = (W_bit_next >= 6'd34) && (W_bit_next <= 6'd48);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_cnt     <= 3'd0;
            S_bit     <= 6'd0;
            S_shift   <= 16'd0;
            S_cur     <= 16'd0;
            S_have    <= 1'b0;
            O_fifo_re <= 1'b0;
            O_bclk    <= 1'b0;
            O_lrclk   <= 1'b0;
            O_din     <= 1'b0;
        end
        else begin
            O_fifo_re <= 1'b0;
            S_cnt     <= S_cnt + 3'd1;
            O_bclk    <= S_cnt[2];          // 25/8 = 3.125 MHz，占空比 50%

            // ------------------------------------------------------
            // 帧边界后第 1 拍：FIFO 寄存器输出已经有效，装入本帧采样
            //   ★ 必须限定 S_bit==0（帧内第一个 BCLK 周期），否则每个 BCLK
            //     周期都会把 S_cur 覆盖成 FIFO 的旧输出，整帧数据错乱。
            // ------------------------------------------------------
            if ((S_cnt == 3'd1) && (S_bit == 6'd0)) begin
                S_cur <= I_fifo_dout;
            end

            // ------------------------------------------------------
            // BCLK 下降沿（S_cnt 7 -> 0）：推进时隙、更新 DIN/LRCLK
            // ------------------------------------------------------
            if (S_cnt == 3'd7) begin
                S_bit   <= W_bit_next;
                O_lrclk <= W_bit_next[5];

                // 帧边界：请求下一个采样
                if (W_frame_start) begin
                    if (!I_fifo_empty) begin
                        O_fifo_re <= 1'b1;
                        S_have    <= 1'b1;
                    end
                    else begin
                        S_have    <= 1'b0;
                    end
                end

                // 数据位
                if (W_bit_load_l || W_bit_load_r) begin
                    S_shift <= {S_cur[14:0], 1'b0};
                    O_din   <= S_have ? S_cur[15] : 1'b0;
                end
                else if (W_bit_shift_l || W_bit_shift_r) begin
                    S_shift <= {S_shift[14:0], 1'b0};
                    O_din   <= S_have ? S_shift[15] : 1'b0;
                end
                else begin
                    O_din   <= 1'b0;
                end
            end
        end
    end

endmodule
