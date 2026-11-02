//=============================================================================
//  tf_sector_buf  --  512 字节扇区缓冲（TF 卡读写的中转区）
//
//  模块名   : tf_sector_buf
//  功能     : 512 x 8 双口 RAM；主机口与控制器口各自独立读写
//  所属域   : tf（存储）
//  时钟域   : I_clk（与 tf_card_ctrl 同域）
//  复位     : I_rst_n，异步低有效（只复位读数据寄存器，不复位 RAM 内容）
//  参数     : AW（地址位宽；512 字节 -> 9）
//  验证状态 : 仿真 通过（tb_tf_card_ctrl）
//
//  为什么是"真双口"而不是单口 + 仲裁
//  ---------------------------------------------------------------------
//  一个扇区 512x8 = 4096 bit，映射成 LUTRAM 约 130 个 LUT6，或占 1 块 ERAM
//  （本工程 v1.17 用 39/108）。两种都便宜，所以没必要为省资源去做仲裁。
//  真双口的好处是**接口语义简单**：主机填/取缓冲不需要关心控制器此刻在
//  干什么，少一类"忘了等 busy 就写"的隐性错误。
//
//  两条使用约定
//  ---------------------------------------------------------------------
//  1. 两个口在**同一拍写同一地址**时结果未定义（标准双口 RAM 行为）。
//     本设计不会出现：主机口只在 tf_card_ctrl 的 O_busy=0 期间使用。
//  2. 读是**1 拍延迟**：地址在本拍给出，读数据下一拍出现在 O_*_rdata 上。
//     读口**不需要读使能**（一直跟着地址走）。控制器侧的调用方按这个延迟
//     做了一字节预取，见 tf_card_ctrl 文件头。
//=============================================================================
module tf_sector_buf #(
    parameter integer AW = 9
)(
    input  wire            I_clk,
    input  wire            I_rst_n,

    // ---- 主机口 ----
    input  wire            I_host_we,
    input  wire [AW-1:0]   I_host_addr,
    input  wire [7:0]      I_host_wdata,
    output reg  [7:0]      O_host_rdata,

    // ---- 控制器口 ----
    input  wire            I_ctl_we,
    input  wire [AW-1:0]   I_ctl_addr,
    input  wire [7:0]      I_ctl_wdata,
    output reg  [7:0]      O_ctl_rdata
);

    reg [7:0] S_mem [0:(1 << AW) - 1];

    always @(posedge I_clk) begin
        if(I_host_we)
            S_mem[I_host_addr] <= I_host_wdata;
        if(I_ctl_we)
            S_mem[I_ctl_addr]  <= I_ctl_wdata;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_host_rdata <= 8'd0;
            O_ctl_rdata  <= 8'd0;
        end
        else begin
            O_host_rdata <= S_mem[I_host_addr];
            O_ctl_rdata  <= S_mem[I_ctl_addr];
        end
    end

endmodule
