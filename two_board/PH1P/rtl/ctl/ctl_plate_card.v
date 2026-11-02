//=============================================================================
//  ctl_plate_card  --  TF 卡车牌表读取 / KEY2 循环取牌控制器
//
//  模块名   : ctl_plate_card
//  功能     : 例化已验证的 tf_card_ctrl，上电等它自动完成卡识别，然后读一次
//             扇区 0 解析车牌表头（magic = "PLT1" + count 1..16）；此后按板载
//             KEY2（低有效，内部去抖）循环取牌：index 自增 -> CMD17 读扇区 0
//             -> 从 0x10 + index*0x20 取 name[0..6] 并校验该记录的 rec_xor
//             -> 7 字节车牌整体更新到 O_plate（准静态，供跨域打拍）
//  所属域   : ctl
//  时钟域   : I_clk（24MHz 相机时钟域 S_cam_24m_clk）
//  复位     : I_rst_n，异步低有效
//  参数     : DIV_INIT / DIV_RUN / KEY_MS / CLK_HZ
//  验证状态 : 仿真 通过（sim/tb_ctl_plate_card.v + sim/tf_sd_card_model.v，
//             日志 sim/tb_ctl_plate_card.log 的 TB_RESULT: PASS）
//             综合/布局布线/上板 未做（顶层集成由他人负责）
//
//  依据文档
//  ---------------------------------------------------------------------
//  docs/design/v1.18-plate-card-contract.md §1（扇区 0 布局、记录结构、
//  rec_xor 定义）、§2（车牌字符串编码与省份码）、§3.2（端口契约与行为）。
//
//  设计取舍（为什么这么写）
//  ---------------------------------------------------------------------
//  1. **不校验扇区级 CRC32**。契约 §3.3 明确划界：整扇区 CRC32 由写入工具
//     负责，FPGA 侧只查 magic + count + 记录级 rec_xor。这样省掉一个 32 位
//     LFSR，而逐记录 8 位异或树几乎不占资源。强弱分工是刻意的。
//  2. **不写卡、不做文件系统**。tf_card_ctrl 只有 CMD17/CMD24 单块读写，
//     本期只用 CMD17；16 条记录一个扇区装得下，不需要多块读。
//  3. **识别失败不重试**。tf_card_ctrl 提供了一次 I_start（I_op=0）重试识别
//     的机会，但契约 §3.2 行为 1 要求"不再尝试"。理由：如果卡没插/没格式化，
//     自动反复重试只会让 SPI 总线一直忙，屏幕上什么都不会变，反而更难诊断。
//     真需要重试时给整模块复位即可（顶层可用按键或上电复位）。
//  4. **表无效后彻底停止读卡**。magic 错或 count 越界时置 O_status=3 并且
//     之后永不再发命令：一次坏表读 16 遍没有任何信息增益，只白白占总线。
//  5. **O_status 编码按需更新的时机**：识别期间保持 0（等待），表头解析完
//     才写 1/3，记录级校验失败时从 1 降到 3。也就是说"曾经好过、后来坏了"
//     会体现在 O_status 上，而 O_plate_valid 同步降 0，屏幕立刻不再显示。
//
//  ★ 踩过的时序坑（写这片代码时最容易错的地方）
//  ---------------------------------------------------------------------
//  A. **主机口读缓冲有 1 拍延迟**。tf_sector_buf 的 O_host_rdata 是寄存器
//     输出，本拍给的 I_buf_addr 要下一拍才出现在数据上。所以取一个字节必须
//     拆两拍：P_S_ADDR 摆地址，P_S_CAPT 采数据。若在同一拍里既给地址又采
//     数据，取到的永远是**上一个地址**的内容 —— 表现为"整个车牌串错位一字节"
//     或者"始终读到上一次残留"，而状态机不会报任何错。
//     （v1.17 的 tf_probe_top.v 也是这么拆两拍的，这里照抄它的习惯。）
//  B. **O_busy=1 期间绝对不碰扇区缓冲**。tf_sector_buf 是真双口，但控制器
//     在忙时正往同一片 RAM 里写，主机口此时给地址只会得到未定义数据。
//     本模块的做法是：只在 O_busy=0 时才进 P_S_ADDR/P_S_CAPT，一旦 O_busy
//     升起就退回 P_S_WAIT，等 O_done 落下再开始读。
//  C. **treated O_done 是单拍脉冲**。它和 O_busy 落下同时发生，所以等待条件
//     写成 `O_done || !O_busy`：只等 !O_busy 会漏掉 done 那一拍上有效的
//     O_err（从而把读失败当成读成功，拿一缓冲垃圾去解析）。
//  D. **rec_xor 的累加不能在采样的同一拍里读**。`S_xor_v <= S_xor_v ^ rd` 是
//     非阻塞赋值，比较时读到的是**上一拍的旧值**。所以比较用的是组合线
//     W_xor_new = S_xor_v ^ W_rd_data，避免差一个字节的经典错误。
//  E. **按键不排队**。忙的时候按下的键直接丢弃（契约 §3.2 行为 4）。做排队
//     需要 FIFO，而"连按两下只取一张牌"对用户是可接受的；排队反而会让用户
//     连按后看到屏幕连续跳好几条，更难理解。
//=============================================================================
module ctl_plate_card #(
    parameter integer DIV_INIT = 31,        // 24MHz: 24e6/(2*32) = 375kHz < 400kHz
    parameter integer DIV_RUN  = 1,         // 24MHz: 24e6/(2*2)  = 6MHz
    parameter integer KEY_MS   = 20,        // KEY2 去抖时长(ms)
    parameter integer CLK_HZ   = 24000000   // I_clk 频率，用于算去抖计数
)(
    input  wire        I_clk,         // 24MHz 相机时钟域
    input  wire        I_rst_n,       // 异步低有效
    input  wire        I_key,         // 板载 KEY2，低有效，内部去抖取"按下"脉冲
    output wire        O_tf_clk,      // SPI 时钟
    output wire        O_tf_cs_n,     // SPI 片选
    output wire        O_tf_mosi,     // SPI MOSI
    input  wire        I_tf_miso,     // SPI MISO
    output reg  [55:0] O_plate,       // {char6..char0}，char0 = O_plate[7:0]
    output reg         O_plate_valid, // 1 = 可显示
    output reg         O_card_ready,  // 卡识别完成
    output reg  [1:0]  O_status       // 0=等待 1=正常 2=卡识别失败 3=表格式无效
);

    //-------------------------------------------------------------------------
    // 卡上格式常量（契约 §1）
    //   扇区布局的魔数一律走带名字的 localparam，不散落在代码里
    //-------------------------------------------------------------------------
    localparam [8:0]  P_OFF_MAGIC     = 9'h000;   // 4 字节 magic
    localparam [8:0]  P_OFF_COUNT     = 9'h004;   // 2 字节 count（小端）
    localparam [8:0]  P_OFF_REC_BASE  = 9'h010;   // 记录 0 起始
    localparam [7:0]  P_REC_STRIDE    = 8'h20;    // 每条记录 32 字节

    localparam [7:0]  P_MAGIC_B0      = 8'h50;    // 'P'
    localparam [7:0]  P_MAGIC_B1      = 8'h4C;    // 'L'
    localparam [7:0]  P_MAGIC_B2      = 8'h54;    // 'T'
    localparam [7:0]  P_MAGIC_B3      = 8'h31;    // '1'

    // 记录内偏移（契约 §1.1）
    localparam [4:0]  P_REC_OFF_FLAGS = 5'h10;    // flags：bit0 = 校内车
    localparam [4:0]  P_REC_OFF_LEN   = 5'h11;    // len：name 有效字节数（本期 7）
    localparam [4:0]  P_REC_OFF_XOR   = 5'h14;    // rec_xor，小端 32 位（高 3 字节 0）

    // 一次取牌要读的记录内字节序号 -> 记录内偏移
    //   0..6  = name[0..6]
    //   7     = flags      8 = len
    //   9..12 = rec_xor[7:0] / [15:8] / [23:16] / [31:24]
    localparam [3:0]  P_RIDX_FLAGS   = 4'd7;
    localparam [3:0]  P_RIDX_LEN     = 4'd8;
    localparam [3:0]  P_RIDX_XOR0    = 4'd9;
    localparam [3:0]  P_RIDX_XOR3    = 4'd12;
    localparam [3:0]  P_RIDX_LAST    = 4'd12;

    localparam integer P_COUNT_MIN   = 1;
    localparam integer P_COUNT_MAX   = 16;
    // "上一次取过的记录号"初值：取 MAX 使第一次自增回绕到 0（见 P_S_READY 注释）
    localparam [3:0]   P_MAX_RECORDS = 4'd15;

    // O_status 编码（契约 §3.2）
    localparam [1:0]  P_ST_WAIT      = 2'd0;
    localparam [1:0]  P_ST_OK        = 2'd1;
    localparam [1:0]  P_ST_CARD_ERR  = 2'd2;
    localparam [1:0]  P_ST_TABLE_ERR = 2'd3;

    // 去抖计数：按下后要求连续这么多拍 I_key 读到低才认"按下"
    localparam integer P_KEY_MAX     = (CLK_HZ / 1000) * KEY_MS;
    localparam integer P_CNT_W       = (P_KEY_MAX < 16) ? 4 : $clog2(P_KEY_MAX + 1);

    // 主机口缓冲地址（AW=9，AW 随 tf_card_ctrl 的 AW 参数）
    localparam integer P_BUF_AW      = 9;

    //-------------------------------------------------------------------------
    // 状态机（全部是本模块自己的前缀 P_S_，与 tf_card_ctrl 无关）
    //   ★ S_st 用 **5 位**：4 位只有 0..15，而本模块需要 17 个状态。
    //     硬塞进 4 位会把 4'd16 截断（vlog-13562），或者被迫让两个状态共用
    //     一个编码（case 里后者被遮住，症状是"某条路径永远等不到预期信号"）。
    //     tf_card_ctrl 里的状态寄存器同样是 5 位，这里与它保持一致。
    //-------------------------------------------------------------------------
    localparam [4:0]  P_S_IDLE      = 5'd0,    // 等卡识别
                      P_S_HDR_CMD   = 5'd1,    // 发 CMD17 读扇区 0（表头）
                      P_S_HDR_BUSY  = 5'd13,   // ★ 先等 O_busy 起来（见下）
                      P_S_HDR_WAIT  = 5'd2,    // 再等 O_done
                      P_S_HDR_ADDR  = 5'd3,    // 摆缓冲地址（第 1 步）
                      P_S_HDR_SETTLE= 5'd14,   // ★ 等满读延迟（见下）
                      P_S_HDR_CAPT  = 5'd4,    // 采缓冲数据（第 3 步）
                      P_S_HDR_EVAL  = 5'd5,    // 表头判定（magic + count）
                      P_S_READY     = 5'd6,    // 表有效，等 KEY2
                      P_S_REC_CMD   = 5'd7,    // 取牌：发 CMD17
                      P_S_REC_BUSY  = 5'd15,   // ★ 同上，取牌路径也要
                      P_S_REC_WAIT  = 5'd8,    // 再等 O_done
                      P_S_REC_ADDR  = 5'd9,
                      P_S_REC_SETTLE= 5'd16,   // ★ 同 P_S_HDR_SETTLE
                      P_S_REC_CAPT  = 5'd10,
                      P_S_REC_EVAL  = 5'd11,   // 校验 rec_xor 并输出
                      P_S_DEAD      = 5'd12;   // 卡识别失败 / 表无效，不再读
    // 编码占用自检：0..16 全部唯一，无一重复（改状态表时请顺手数一遍）。

    // ★★ 为什么必须单独有 *_BUSY 这两个状态（这是仿真抓到的真 bug，别删）★★
    //   tf_card_ctrl 的握手是"O_busy 上升 -> ... -> O_done 单拍 + O_busy 落下"。
    //   但 **O_busy 是在 I_start 那一拍的**下一个**时钟沿才置 1 的**：
    //     * 主机在 P_S_xxx_CMD 里把 S_start 置 1（当拍）
    //     * 控制器在接下来那一拍才 O_busy <= 1
    //   如果直接从 CMD 跳到"等 O_done 或 !O_busy"，那么进入等待态的当拍
    //   O_busy 还**是 0**（起始值），条件 `!W_busy` 立刻成立 → 状态机以为
    //   一次读已经做完，其实一个字节都没读 → 扇区缓冲里全是 x →
    //   status=3、车牌全 0（首版就是栽在这里）。
    //   ★ 顺带一个同源坑：扇区缓冲主机口的读延迟是 **2 拍**，不是 1 拍 ——
    //     tf_sector_buf 内部是 `O_host_rdata <= S_mem[I_host_addr]`（1 拍），
    //     而这一拍又是从"地址寄存器在某个时钟沿变新"开始算的，于是从"摆地址"
    //     到"数据在组合/寄存输出上可见"实际跨 2 个时钟沿。
    //     只等 1 拍就开始采，采到的是**上一次地址**的数据：表头会读成
    //     `50 50 4c 54` / count 读成 `0331`（首版实测就是这个值）。
    //     所以两个读路径都要 地址 -> SETTLE -> CAPT 三步。

    reg [4:0]         S_st;

    //-------------------------------------------------------------------------
    // 子模块握手
    //-------------------------------------------------------------------------
    reg               S_op;
    reg  [31:0]       S_sector;
    reg               S_start;
    reg               S_buf_we;
    reg  [P_BUF_AW-1:0] S_buf_addr;
    reg  [7:0]        S_buf_wdata;

    wire              W_ready;
    wire              W_busy;
    wire              W_done;
    wire [3:0]        W_err;
    wire [7:0]        W_buf_rdata;

    tf_card_ctrl #(
        .SPI_W        ( 16        ),
        .DIV_INIT     ( DIV_INIT  ),
        .DIV_RUN      ( DIV_RUN   ),
        .TO_BYTES     ( 16'd60000 ),
        .ACMD41_RETRY ( 16'd4000  ),
        .CHECK_RD_CRC ( 1'b1      ),
        .AW           ( 9         )
    ) u_ctrl (
        .I_clk        ( I_clk       ),
        .I_rst_n      ( I_rst_n     ),
        .I_buf_we     ( S_buf_we    ),
        .I_buf_addr   ( S_buf_addr  ),
        .I_buf_wdata  ( S_buf_wdata ),
        .O_buf_rdata  ( W_buf_rdata ),
        .I_op         ( {1'b0, S_op} ),
        .I_sector     ( S_sector    ),
        .I_start      ( S_start     ),
        .O_ready      ( W_ready     ),
        .O_busy       ( W_busy      ),
        .O_done       ( W_done      ),
        .O_err        ( W_err       ),
        .O_card_type  (             ),
        .O_block_mode (             ),
        .O_state      (             ),
        .O_ocr        (             ),
        .O_tf_clk     ( O_tf_clk    ),
        .O_tf_cs_n    ( O_tf_cs_n   ),
        .O_tf_mosi    ( O_tf_mosi   ),
        .I_tf_miso    ( I_tf_miso   )
    );

    //-------------------------------------------------------------------------
    // KEY2 去抖：低有效，按下产生 **1 拍高脉冲**（S_key_new）
    //
    //   分三级，每级只做一件事，便于单独验证：
    //     ① 同步 2 拍   ：按键是异步输入，先进触发器去亚稳态。
    //     ② 规整（去抖）：连续 P_KEY_MAX 拍读到低才把 S_key_clean 拉低，
    //                     读到高就立刻拉高。慢包络，毛刺穿不过去。
    //     ③ 边沿 + 一次性：S_key_clean 的下降沿产生 1 拍脉冲；S_key_pressed
    //                     锁存"这一按还没被消费"，等 S_key_clean 回到高才解锁。
    //                     所以按住不放只算一次，弹跳不会算成第二次。
    //
    //   ★ 踩过的坑：第一版把"一次性"和"计数填满"写在同一个条件里
    //     （cnt>=MAX 且 !armed 才发脉冲），结果 armed 和 cnt 饱和发生在同一拍，
    //     脉冲永远不发，表现成"按键完全没反应"而状态机一切正常。改成
    //     "干净电平 -> 边沿 -> 一次性"三级串起来之后，脉冲的产生条件是
    //     单一、可观察的（就是 S_key_clean 的下降沿）。
    //-------------------------------------------------------------------------
    reg               S_key_s0;
    reg               S_key_s1;
    reg  [P_CNT_W-1:0] S_key_cnt;
    reg               S_key_clean;     // 去抖后的低有效电平（1 = 未按下）
    reg               S_key_pressed;   // 一次按下尚未被消费
    reg               S_key_new;       // 1 拍脉冲：按下了

    wire W_key_low = ~S_key_s1;

    // 计数比较用的常量（截到计数器位宽，避免超宽比较引起仿真/综合告警）
    localparam [P_CNT_W-1:0] P_KEY_MAX_CNT = P_KEY_MAX[P_CNT_W-1:0];

    // "本拍已经认定为按住"：连续够久读到低 且 计数器已填满
    wire W_key_down = W_key_low && (S_key_cnt >= P_KEY_MAX_CNT);

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_key_s0      <= 1'b1;
            S_key_s1      <= 1'b1;
            S_key_cnt     <= {P_CNT_W{1'b0}};
            S_key_clean   <= 1'b1;      // 复位后当作"没按"
            S_key_pressed <= 1'b0;
            S_key_new     <= 1'b0;
        end
        else begin
            // ① 同步
            S_key_s0 <= I_key;
            S_key_s1 <= S_key_s0;

            // ② 规整
            if(W_key_low) begin
                if(S_key_cnt != P_KEY_MAX_CNT) S_key_cnt <= S_key_cnt + 1'b1;
                if(S_key_cnt >= P_KEY_MAX_CNT) S_key_clean <= 1'b0;   // 连续够久才算按下
            end
            else begin
                S_key_cnt   <= {P_CNT_W{1'b0}};
                S_key_clean <= 1'b1;                                   // 一松手立刻释放
            end

            // ③ 边沿 -> 1 拍脉冲；锁存到松开（clean 回高）为止。
            //   ★ 这里必须用组合条件 W_key_down 判断"现在是否认为按着"，
            //     不能用 S_key_clean 自己：它在下面才是非阻塞更新，本拍读到的
            //     是**旧值**，会形成"下了又上"的自激振荡，一次按住就产生
            //     出一串脉冲（实测按住 25ms 会连续取 16 张牌）。
            S_key_new <= 1'b0;
            if(W_key_down) begin
                if(!S_key_pressed) begin
                    S_key_pressed <= 1'b1;
                    S_key_new     <= 1'b1;
                end
            end
            else begin
                S_key_pressed <= 1'b0;
            end
        end
    end

    //-------------------------------------------------------------------------
    // 内部寄存器
    //-------------------------------------------------------------------------
    reg [4:0]         S_count;        // 表里的记录条数（1..16）
    reg [3:0]         S_index;        // 下一条要取的记录号
    reg [3:0]         S_hdr_b;        // 表头字节序号 0..5
    reg [3:0]         S_rec_idx;      // 记录内字节序号 0..12
    reg [2:0]         S_xor_cnt;      // rec_xor 收到的字节数
    reg [8:0]         S_rec_base;     // 记录基址 = 0x10 + index*0x20
    reg [7:0]         S_xor_v;        // rec_xor 累加值（组合算，寄存用）
    reg [7:0]         S_rx_x0;
    reg [7:0]         S_rx_x1;
    reg [7:0]         S_rx_x2;
    reg [7:0]         S_rx_x3;
    reg [7:0]         S_rec_flags;
    reg [7:0]         S_rec_len;
    reg [7:0]         S_name0;
    reg [7:0]         S_name1;
    reg [7:0]         S_name2;
    reg [7:0]         S_name3;
    reg [7:0]         S_name4;
    reg [7:0]         S_name5;
    reg [7:0]         S_name6;

    //-------------------------------------------------------------------------
    // 取牌用的记录内字节地址（组合）
    //   主机口读延迟 1 拍，所以只有摆地址那一拍它有意义；这里按"当前记录 +
    //   当前字节序号"算出来，不用再维护一个自增地址寄存器。
    //-------------------------------------------------------------------------
    wire [4:0] W_rec_off_b = (S_rec_idx < 4'd7)  ? {1'b0, S_rec_idx}              :  // name[0..6]
                             (S_rec_idx == P_RIDX_FLAGS) ? P_REC_OFF_FLAGS :
                             (S_rec_idx == P_RIDX_LEN)   ? P_REC_OFF_LEN   :
                                                           (P_REC_OFF_XOR + (S_rec_idx - P_RIDX_XOR0));
    wire [P_BUF_AW-1:0] W_rec_addr = S_rec_base[P_BUF_AW-1:0] +
                                     {{(P_BUF_AW-5){1'b0}}, W_rec_off_b};

    wire [7:0] W_rd_data  = W_buf_rdata;
    // ★ 见文件头坑 D：比较用组合值，绝不能用 S_xor_v 自己（非阻塞赋值的旧值）
    wire [7:0] W_xor_new  = S_xor_v ^ W_rd_data;

    //-------------------------------------------------------------------------
    // 下一个要取的记录号（组合次态）
    //   为什么单独抽成一根 wire：P_S_READY 里既要更新 S_index，又要用它算
    //   S_rec_base。用非阻塞的 S_index 算地址会拿到**旧值**（地址整体滞后一次），
    //   见 P_S_READY 里的注释。抽成组合次态后两个赋值用的是同一个新值。
    //-------------------------------------------------------------------------
    wire [3:0] W_idx_nxt = (S_index >= (S_count - 5'd1)) ? 4'd0 : (S_index + 4'd1);

    // 表头字节（magic 4 + count 2），MSB 先到
    reg [31:0] S_magic;
    reg [15:0] S_cnt_raw;

    // 命令发起打包成"单拍"寄存器集合，避免和状态机同拍竞争
    reg        S_go;

    //-------------------------------------------------------------------------
    // 主状态机
    //-------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_st         <= P_S_IDLE;
            S_op         <= 1'b0;
            S_sector     <= 32'd0;
            S_start      <= 1'b0;
            S_buf_we     <= 1'b0;
            S_buf_addr   <= {P_BUF_AW{1'b0}};
            S_buf_wdata  <= 8'd0;
            S_count      <= 5'd0;
            S_index      <= P_MAX_RECORDS;
            S_hdr_b      <= 4'd0;
            S_rec_idx    <= 4'd0;
            S_xor_cnt    <= 3'd0;
            S_rec_base   <= 9'd0;
            S_xor_v      <= 8'd0;
            S_rx_x0      <= 8'd0;
            S_rx_x1      <= 8'd0;
            S_rx_x2      <= 8'd0;
            S_rx_x3      <= 8'd0;
            S_rec_flags  <= 8'd0;
            S_rec_len    <= 8'd0;
            S_name0      <= 8'd0;
            S_name1      <= 8'd0;
            S_name2      <= 8'd0;
            S_name3      <= 8'd0;
            S_name4      <= 8'd0;
            S_name5      <= 8'd0;
            S_name6      <= 8'd0;
            S_magic      <= 32'd0;
            S_cnt_raw    <= 16'd0;
            S_go         <= 1'b0;
            O_plate        <= 56'd0;
            O_plate_valid  <= 1'b0;
            O_card_ready   <= 1'b0;
            O_status       <= P_ST_WAIT;
        end
        else begin
            // 默认值
            S_start <= 1'b0;
            S_buf_we <= 1'b0;

            // 卡识别完成即对外可见（只置位，不复位：契约 §3.2 行为 6）
            if(W_ready) O_card_ready <= 1'b1;

            case(S_st)
            //=============================================================
            // 等 tf_card_ctrl 自己跑完上电识别
            //=============================================================
            P_S_IDLE: begin
                if(W_ready && !W_busy) begin
                    // 识别成功：先读一次扇区 0 取表头
                    S_op     <= 1'b1;
                    S_sector <= 32'd0;
                    S_go     <= 1'b1;
                    S_hdr_b  <= 4'd0;
                    S_st     <= P_S_HDR_CMD;
                end
                else if(!W_ready && !W_busy) begin
                    // 识别结束且没就绪 = 卡识别失败，按契约不再重试
                    O_status <= P_ST_CARD_ERR;
                    S_st     <= P_S_DEAD;
                end
            end

            //=============================================================
            // 表头：发 CMD17 读扇区 0
            //=============================================================
            P_S_HDR_CMD: begin
                if(S_go) begin
                    S_start <= 1'b1;
                    S_go    <= 1'b0;
                    S_st    <= P_S_HDR_BUSY;
                end
            end

            // 先等控制器把 O_busy 拉起来（I_start 之后一拍才起来，
            // 直接跳到下一个状态会误判"已完成" —— 见 *_BUSY 状态上方的说明）
            P_S_HDR_BUSY: begin
                if(W_busy) S_st <= P_S_HDR_WAIT;
            end

            P_S_HDR_WAIT: begin
                // 只认 O_done（一次完整流程的结束标志）。
                // ★ 这里**故意不看 !W_busy**：busy 在前一态已经确认为 1，
                //   此处再拿 !busy 当完成条件只会在两者时序错开时引入误判。
                if(W_done) begin
                    if(W_err != 4'd0) begin
                        // 扇区 0 读不回来：连表头都没有，按"表格式无效"处理
                        O_status      <= P_ST_TABLE_ERR;
                        O_plate_valid <= 1'b0;
                        S_st          <= P_S_DEAD;
                    end
                    else begin
                        S_hdr_b <= 4'd0;
                        S_st    <= P_S_HDR_ADDR;
                    end
                end
            end

            P_S_HDR_ADDR: begin
                // 第 1 步：只摆地址
                S_buf_addr <= P_OFF_MAGIC + {5'd0, S_hdr_b};
                S_st       <= P_S_HDR_SETTLE;
            end

            P_S_HDR_SETTLE: begin
                // 第 2 步：地址保持不动，等满 2 拍读延迟（坑见状态表上方注释）
                S_st <= P_S_HDR_CAPT;
            end

            P_S_HDR_CAPT: begin
                // 第 3 步：数据有效
                if(S_hdr_b < 4'd4)
                    S_magic <= {S_magic[23:0], W_rd_data};
                else if(S_hdr_b == 4'd4)
                    S_cnt_raw[7:0]  <= W_rd_data;
                else
                    S_cnt_raw[15:8] <= W_rd_data;

                if(S_hdr_b == 4'd5) begin
                    // 表头读完：这里只能判定，实际落 status 放到下一状态，
                    // 因为 S_magic / S_cnt_raw 是这一拍才写进去的
                    S_st <= P_S_HDR_EVAL;
                end
                else begin
                    S_hdr_b <= S_hdr_b + 4'd1;
                    S_st    <= P_S_HDR_ADDR;
                end
            end

            P_S_HDR_EVAL: begin
                if( (S_magic == {P_MAGIC_B0, P_MAGIC_B1, P_MAGIC_B2, P_MAGIC_B3}) &&
                    (S_cnt_raw >= P_COUNT_MIN) && (S_cnt_raw <= P_COUNT_MAX) ) begin
                    S_count  <= S_cnt_raw[4:0];
                    // ★ 必须是 P_MAX_RECORDS 而不是 0：见 P_S_READY 的注释，
                    //   "先自增再取牌"决定了第一次按键要取记录 0。
                    S_index  <= P_MAX_RECORDS;
                    O_status <= P_ST_OK;
                    S_st     <= P_S_READY;
                end
                else begin
                    // magic 错 / count 越界：置 3 且之后不再读（契约 §3.2 行为 2）
                    O_status      <= P_ST_TABLE_ERR;
                    O_plate_valid <= 1'b0;
                    S_st          <= P_S_DEAD;
                end
            end

            //=============================================================
            // 表有效：等 KEY2
            //=============================================================
            P_S_READY: begin
                if(S_key_new) begin
                    // ★ 先自增再取，所以"第 1 次按键要取记录 0"必须让自增从
                    //   P_MAX_RECORDS(15) 起步（15+1 按上面判据回绕到 0）。
                    //   首版直接把 S_index 复位/表头解析后置成 0，于是第一次按键
                    //   取到的是**记录 1** —— 整条序列整体偏移一格，
                    //   而且"回绕到 0"永远在最后一次之前发生。
                    //   S_index 的复位值与表头解析后的赋值都必须是 P_MAX_RECORDS。
                    //
                    // ★★ 这里必须用组合次态 W_idx_nxt，不能再用 S_index ★★
                    //   S_index 在本状态里是**非阻塞**更新的，本拍读到的仍是旧值。
                    //   第一版写成
                    //       if(S_index >= count-1) S_index <= 0; else S_index <= S_index+1;
                    //       S_rec_base <= P_OFF_REC_BASE + {S_index, 5'd0};   // ← 旧值！
                    //   于是地址永远比序号晚一次：第一次按键（S_index 初值 15）
                    //   算出的基址是 0x1F0 而不是 0x10，读到的是空白保留区，
                    //   表现为 valid=0、names 全 0，而状态机一路正常跑完 ——
                    //   又是一个"不报错但数据错"的坑。
                    S_index  <= W_idx_nxt;

                    S_rec_base <= P_OFF_REC_BASE + {W_idx_nxt, 5'd0};
                    S_rec_idx  <= 4'd0;
                    S_xor_cnt  <= 3'd0;
                    S_xor_v    <= 8'd0;
                    S_op       <= 1'b1;
                    S_sector   <= 32'd0;
                    S_go       <= 1'b1;
                    S_st       <= P_S_REC_CMD;
                end
            end

            //=============================================================
            // 取牌：发 CMD17
            //=============================================================
            P_S_REC_CMD: begin
                if(S_go) begin
                    S_start <= 1'b1;
                    S_go    <= 1'b0;
                    S_st    <= P_S_REC_BUSY;
                end
            end

            // 同 P_S_HDR_BUSY：先等 O_busy 起来再等 O_done
            P_S_REC_BUSY: begin
                if(W_busy) S_st <= P_S_REC_WAIT;
            end

            P_S_REC_WAIT: begin
                // 同 P_S_HDR_WAIT：只认 O_done
                if(W_done) begin
                    if(W_err != 4'd0) begin
                        // 读失败：不显示，并降到"表格式无效"提示上层
                        O_plate       <= {7{8'hFF}};
                        O_plate_valid <= 1'b0;
                        O_status      <= P_ST_TABLE_ERR;
                        S_st          <= P_S_READY;
                    end
                    else begin
                        S_rec_idx <= 4'd0;
                        S_st      <= P_S_REC_ADDR;
                    end
                end
            end

            P_S_REC_ADDR: begin
                // 第 1 步：摆地址
                S_buf_addr <= W_rec_addr;
                S_st       <= P_S_REC_SETTLE;
            end

            P_S_REC_SETTLE: begin
                // 第 2 步：等满 2 拍读延迟
                S_st <= P_S_REC_CAPT;
            end

            P_S_REC_CAPT: begin
                // 第 3 步：数据有效
                if(S_rec_idx < 4'd7) begin
                    // name[0..6]：同时做 rec_xor 的逐字节异或
                    case(S_rec_idx)
                        4'd0: S_name0 <= W_rd_data;
                        4'd1: S_name1 <= W_rd_data;
                        4'd2: S_name2 <= W_rd_data;
                        4'd3: S_name3 <= W_rd_data;
                        4'd4: S_name4 <= W_rd_data;
                        4'd5: S_name5 <= W_rd_data;
                        default: S_name6 <= W_rd_data;
                    endcase
                    S_xor_v <= W_xor_new;
                end
                else if(S_rec_idx == P_RIDX_FLAGS) begin
                    S_rec_flags <= W_rd_data;
                    S_xor_v     <= W_xor_new;
                end
                else if(S_rec_idx == P_RIDX_LEN) begin
                    S_rec_len <= W_rd_data;
                    S_xor_v   <= W_xor_new;
                end
                else begin
                    // rec_xor：小端 32 位，收 4 字节
                    case(S_xor_cnt)
                        3'd0: S_rx_x0 <= W_rd_data;
                        3'd1: S_rx_x1 <= W_rd_data;
                        3'd2: S_rx_x2 <= W_rd_data;
                        default: S_rx_x3 <= W_rd_data;
                    endcase
                    if(S_xor_cnt != 3'd3) S_xor_cnt <= S_xor_cnt + 3'd1;
                end

                if(S_rec_idx == P_RIDX_LAST) S_st <= P_S_REC_EVAL;
                else begin
                    S_rec_idx <= S_rec_idx + 4'd1;
                    S_st      <= P_S_REC_ADDR;
                end
            end

            P_S_REC_EVAL: begin
                if({S_rx_x3, S_rx_x2, S_rx_x1, S_rx_x0} == {24'd0, S_xor_v}) begin
                    // 校验通过：整串 7 字节一次性更新（准静态，供跨域打拍）
                    O_plate       <= {S_name6, S_name5, S_name4, S_name3,
                                      S_name2, S_name1, S_name0};
                    O_plate_valid <= 1'b1;
                    O_status      <= P_ST_OK;
                end
                else begin
                    // 校验失败：不显示，并标记表格式无效（契约 §3.2 行为 3）
                    O_plate       <= {7{8'hFF}};
                    O_plate_valid <= 1'b0;
                    O_status      <= P_ST_TABLE_ERR;
                end
                S_st <= P_S_READY;
            end

            //=============================================================
            // 终态：卡识别失败 / 表无效 —— 永不再发命令，按键一律忽略
            //=============================================================
            P_S_DEAD: begin
                S_st <= P_S_DEAD;
            end

            default: S_st <= P_S_IDLE;
            endcase
        end
    end

endmodule
