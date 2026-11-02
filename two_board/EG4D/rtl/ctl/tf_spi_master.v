//=============================================================================
//  tf_spi_master  --  SPI 字节级主机（SD/TF 卡 SPI 模式物理层）
//
//  模块名   : tf_spi_master
//  功能     : 把"发一个字节 / 同时收一个字节"做成一次握手；SPI 模式 0
//             （CPOL=0, CPHA=0：上升沿采样 MISO、下降沿换 MOSI），MSB 先出
//  所属域   : tf（存储）
//  时钟域   : I_clk（本工程用 100MHz 系统时钟，SPI 时钟由 I_div 分频得到）
//  复位     : I_rst_n，异步低有效
//  参数     : DIV_W
//  验证状态 : 仿真 通过（tb_tf_card_ctrl + tf_sd_card_model）
//             综合/布线/上板 见 docs/design/v1.17-tf-card.md
//
//  为什么要有这一层
//  ---------------------------------------------------------------------
//  SD 卡协议里"发命令"和"收响应"是同一个 8 拍移位（全双工时发 0xFF 收
//  响应）。把它抽成一次字节交换，上层的协议状态机就只需要关心
//  "这一拍该换哪个字节、回来的字节是什么"，不用碰任何位时序。
//
//  时钟速率
//  ---------------------------------------------------------------------
//      SCLK 频率 = I_clk / (2 * (I_div + 1))
//      I_clk = 100MHz 时：I_div=124 -> 400kHz（卡识别的上限频率）
//                        I_div=4   -> 10MHz （数据传输）
//  两个值由上层 tf_card_ctrl 在识别阶段/数据阶段之间切换。
//
//  时序约定（重要）
//  ---------------------------------------------------------------------
//  * **所有输出都过一级寄存器**（O_sclk / O_mosi）。SCLK 是由计数器翻转
//    出来的，如果直接送到引脚，中间任何组合毛刺都会变成卡看到的时钟沿；
//    寄存一拍后引脚上是干净的方法波。
//  * SCLK 空闲**保持低**（模式 0）。I_tx_start 到来后先摆好 MOSI 的 MSB，
//    再开始产生 8 个时钟。
//  * MISO 在上升沿采样进 S_miso_r，到下降沿才并进接收移位寄存器 —— 因为
//    从机是在下降沿换 MISO 的，下降沿当场采会采到正在变化的位。
//=============================================================================
module tf_spi_master #(
    parameter integer DIV_W = 16
)(
    input  wire              I_clk,
    input  wire              I_rst_n,

    input  wire [DIV_W-1:0]  I_div,        // 半周期计数，见文件头公式
    input  wire              I_tx_start,   // 1 拍脉冲，启动一次字节交换
    input  wire [7:0]        I_tx_byte,

    output reg  [7:0]        O_rx_byte,
    output reg               O_done,       // 1 拍脉冲，此时 O_rx_byte 有效
    output reg               O_busy,

    output reg               O_sclk,
    output reg               O_mosi,
    input  wire              I_miso
);

    reg [2:0]             S_bit;      // 本字节已完成的位数 0..7
    reg                   S_phase;    // 0 = 下一步是上升沿，1 = 下一步是下降沿
    reg [DIV_W-1:0]       S_div;
    reg [7:0]             S_sh_tx;
    reg [7:0]             S_sh_rx;
    reg                   S_miso_r;   // 上升沿采到的 MISO

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_sclk    <= 1'b0;
            O_mosi    <= 1'b0;
            O_done    <= 1'b0;
            O_busy    <= 1'b0;
            O_rx_byte <= 8'd0;
            S_bit     <= 3'd0;
            S_phase   <= 1'b0;
            S_div     <= {DIV_W{1'b0}};
            S_sh_tx   <= 8'd0;
            S_sh_rx   <= 8'd0;
            S_miso_r  <= 1'b0;
        end
        else begin
            O_done <= 1'b0;

            if(!O_busy) begin
                O_sclk <= 1'b0;
                if(I_tx_start) begin
                    O_busy  <= 1'b1;
                    S_sh_tx <= I_tx_byte;
                    O_mosi  <= I_tx_byte[7];   // 先摆好 MSB，再出时钟
                    S_bit   <= 3'd0;
                    S_phase <= 1'b0;
                    S_div   <= {DIV_W{1'b0}};
                    S_sh_rx <= 8'd0;
                end
            end
            else begin
                if(S_div >= I_div) begin
                    S_div <= {DIV_W{1'b0}};

                    if(S_phase == 1'b0) begin
                        // ---- 上升沿：从机数据在这时最稳，采样 ----
                        S_miso_r <= I_miso;
                        O_sclk   <= 1'b1;
                        S_phase  <= 1'b1;
                    end
                    else begin
                        // ---- 下降沿：并进接收位，换出下一位 ----
                        O_sclk  <= 1'b0;
                        S_phase <= 1'b0;
                        S_sh_rx <= {S_sh_rx[6:0], S_miso_r};

                        if(S_bit == 3'd7) begin
                            O_busy    <= 1'b0;
                            O_done    <= 1'b1;
                            O_rx_byte <= {S_sh_rx[6:0], S_miso_r};
                            O_mosi    <= 1'b0;
                        end
                        else begin
                            S_bit   <= S_bit + 1'b1;
                            S_sh_tx <= {S_sh_tx[6:0], 1'b0};
                            O_mosi  <= S_sh_tx[6];
                        end
                    end
                end
                else begin
                    S_div <= S_div + 1'b1;
                end
            end
        end
    end

endmodule
