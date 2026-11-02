//=============================================================================
//  aud_tone  --  自检音 / 失败提示音发生器（不依赖 TF 卡）
//
//  模块名   : aud_tone
//  功能     : I_nbeeps == 0  -> 连续 1.5 s 的 1 kHz 方波（KEY16 自检音）
//             I_nbeeps == N  -> 响 N 声 "嘀"（每声 0.12 s，间隔 0.12 s）
//                              用来把播放失败的状态码变成可听的声音：
//                              4=无卡 5=找不到文件 6=WAV 非法 7=TF 错误 8=文件系统不支持
//  所属域   : aud
//  时钟域   : I_clk = 25 MHz（与 FIFO/I2S 同域）
//  复位     : I_rst_n，异步低有效
//
//  频率计算
//  ---------------------------------------------------------------------
//  采样率 fs = 25 MHz / 512 = 48 828.125 Hz；1 kHz 半周期取 24 个采样
//  -> 实际 1017 Hz，幅度 ±7000（约 -13 dBFS），不刺耳。
//
//  流控
//  ---------------------------------------------------------------------
//  只有 I_space（FIFO 剩余 >= 256）时才写一个采样；FIFO 填到 257 后自动停住，
//  之后每被 I2S 取走一个才补一个 —— 频率严格等于 I2S 采样率。
//  I_abort 立即停止（KEY1..15 播语音时把 FIFO 让给播放器）。
//=============================================================================
module aud_tone (
    input  wire        I_clk,
    input  wire        I_rst_n,

    input  wire        I_start,        // 1 拍脉冲：开始
    input  wire [3:0]  I_nbeeps,       // 0 = 连续 1.5 s；N = 响 N 声
    input  wire        I_abort,        // 1 = 立即停止（播放器接管 FIFO）
    input  wire        I_space,        // FIFO 剩余空间 >= 256

    output reg         O_we,
    output reg  [15:0] O_data,
    output reg         O_busy
);
    // 采样数：48 828.125 Hz
    localparam [16:0] P_LONG   = 17'd73242;   // 1.5 s（连续模式）
    localparam [15:0] P_BEEP   = 16'd5860;    // 0.12 s（单声长度 / 间隔长度）
    localparam [4:0]  P_HALF   = 5'd24;       // 半周期采样数

    localparam [1:0]  T_IDLE = 2'd0,
                      T_ON   = 2'd1,
                      T_GAP  = 2'd2;

    reg [1:0]  S_st;
    reg [4:0]  S_ph;          // 半周期内计数
    reg        S_pol;         // 0 = +AMP, 1 = -AMP
    reg [16:0] S_left;        // 当前 ON 段剩余采样
    reg [15:0] S_gap;         // 当前 GAP 段剩余采样
    reg [3:0]  S_beeps;       // 还剩几声
    reg        S_cont;        // 1 = 连续模式

    wire [16:0] W_on_len = S_cont ? P_LONG : {1'b0, P_BEEP};

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_st    <= T_IDLE;
            S_ph    <= 5'd0;
            S_pol   <= 1'b0;
            S_left  <= 17'd0;
            S_gap   <= 16'd0;
            S_beeps <= 4'd0;
            S_cont  <= 1'b1;
            O_we    <= 1'b0;
            O_data  <= 16'd0;
            O_busy  <= 1'b0;
        end
        else begin
            O_we <= 1'b0;

            if (I_abort) begin
                S_st    <= T_IDLE;
                S_beeps <= 4'd0;
                O_busy  <= 1'b0;
            end
            else if (I_start) begin
                S_cont  <= (I_nbeeps == 4'd0);
                S_beeps <= (I_nbeeps == 4'd0) ? 4'd1 : I_nbeeps;
                S_left  <= (I_nbeeps == 4'd0) ? P_LONG : {1'b0, P_BEEP};
                S_ph    <= 5'd0;
                S_pol   <= 1'b0;
                S_st    <= T_ON;
                O_busy  <= 1'b1;
            end
            else begin
                case (S_st)
                T_ON: begin
                    if (I_space) begin
                        O_we   <= 1'b1;
                        O_data <= S_pol ? (-16'sd7000) : 16'sd7000;
                        S_ph   <= (S_ph == (P_HALF - 5'd1)) ? 5'd0 : (S_ph + 5'd1);
                        S_pol  <= (S_ph == (P_HALF - 5'd1)) ? ~S_pol : S_pol;
                        if (S_left <= 17'd1) begin
                            S_left <= 17'd0;
                            if (S_beeps <= 4'd1) begin
                                S_beeps <= 4'd0;
                                S_st    <= T_IDLE;
                                O_busy  <= 1'b0;
                            end
                            else begin
                                S_beeps <= S_beeps - 4'd1;
                                S_gap   <= P_BEEP;
                                S_st    <= T_GAP;
                            end
                        end
                        else begin
                            S_left <= S_left - 17'd1;
                        end
                    end
                end
                T_GAP: begin
                    S_gap <= S_gap - 16'd1;
                    if (S_gap <= 16'd1) begin
                        S_left <= W_on_len;
                        S_st   <= T_ON;
                    end
                end
                default: begin
                    O_busy <= 1'b0;
                end
                endcase
            end
        end
    end

endmodule
