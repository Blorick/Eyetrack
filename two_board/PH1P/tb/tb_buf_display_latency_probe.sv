//=============================================================================
//  tb_buf_display_latency_probe -- DDR 帧票号/显示帧龄探测单元测试
//-----------------------------------------------------------------------------
//  所属域   : buf
//  时钟域   : I_clk_ddr, I_clk_pix
//  复位     : I_rst_n
//  参数     : 无
//  验证状态 : Questa 通过时输出 TB_RESULT: PASS
//=============================================================================
//  检查内容：
//    1. 复位后、尚未开始采集时，显示场起始不发布非法值（帧龄 0/31 被门限挡住）。
//    2. 采集帧按 video_in 的 bank 轮转写入、显示帧按 (写指针+2)%4 选 bank 时，
//       稳态帧龄恒为 2 帧（v1.13 四帧映射的 33 ms 结论）。
//    3. 跨 5 bit 票号回绕（40+ 帧）后仍为 2 帧，验证模 32 减法。
//    4. 人为把显示 bank 指到更老一帧时，帧龄变为 3 帧，证明是本模块实测而不是
//       常数；恢复后回到 2 帧。
//=============================================================================
`timescale 1ns / 1ps

module tb_buf_display_latency_probe;

    localparam [24:0] P_BASE_0 = 25'd0;
    localparam [24:0] P_BASE_1 = 25'd7000000;
    localparam [24:0] P_BASE_2 = 25'd14000000;
    localparam [24:0] P_BASE_3 = 25'd21000000;

    reg I_clk_ddr = 1'b0;
    reg I_clk_pix = 1'b0;

    always #7.5    I_clk_ddr = ~I_clk_ddr;   // 66.667 MHz
    always #6.667  I_clk_pix = ~I_clk_pix;   // 75 MHz

    reg I_rst_n;
    reg I_wr_en;
    reg [24:0] I_wr_addr;
    reg [1:0] I_display_rp;
    reg I_display_vsync;

    wire [4:0] O_lat_frames;
    wire O_lat_update;

    integer S_checks;
    integer S_errors;
    integer S_frames_started;

    //-------------------------------------------------------------------------
    // 参考模型：与 DUT 看同一组输入，独立保存票号/计数，用于对齐“哪个显示场”
    // 该出什么值；期望值本身由下面的阶段常量（2/3 帧）给出，不是把 DUT 抄一遍。
    //-------------------------------------------------------------------------
    reg [4:0] M_cap_count;
    reg [4:0] M_ticket_0;
    reg [4:0] M_ticket_1;
    reg [4:0] M_ticket_2;
    reg [4:0] M_ticket_3;
    reg [1:0] M_rp;
    reg [2:0] M_vsync_sync;
    reg [1:0] M_phase;              // 0=未开始，1=稳态(2)，2=注入(3)
    reg [1:0] M_phase_d;
    integer M_phase_settle;

    wire M_wr_frame = I_wr_en &&
        ((I_wr_addr == P_BASE_0) || (I_wr_addr == P_BASE_1) ||
         (I_wr_addr == P_BASE_2) || (I_wr_addr == P_BASE_3));

    wire [1:0] M_wr_index = (I_wr_addr == P_BASE_0) ? 2'd0 :
                            (I_wr_addr == P_BASE_1) ? 2'd1 :
                            (I_wr_addr == P_BASE_2) ? 2'd2 : 2'd3;

    always @(posedge I_clk_ddr or negedge I_rst_n) begin
        if (!I_rst_n) begin
            M_cap_count <= 5'd0;
            M_ticket_0  <= 5'd0;
            M_ticket_1  <= 5'd0;
            M_ticket_2  <= 5'd0;
            M_ticket_3  <= 5'd0;
        end
        else if (M_wr_frame) begin
            M_cap_count <= M_cap_count + 5'd1;
            case (M_wr_index)
                2'd0: M_ticket_0 <= M_cap_count;
                2'd1: M_ticket_1 <= M_cap_count;
                2'd2: M_ticket_2 <= M_cap_count;
                default: M_ticket_3 <= M_cap_count;
            endcase
        end
    end

    always @(posedge I_clk_ddr or negedge I_rst_n) begin
        if (!I_rst_n) M_vsync_sync <= 3'b000;
        else          M_vsync_sync <= {M_vsync_sync[1:0], I_display_vsync};
    end

    wire M_display_sof = ~M_vsync_sync[2] & M_vsync_sync[1];

    wire [4:0] M_display_ticket = (M_rp == 2'd0) ? M_ticket_0 :
                                  (M_rp == 2'd1) ? M_ticket_1 :
                                  (M_rp == 2'd2) ? M_ticket_2 : M_ticket_3;

    // SOF 处的期望帧龄（模 32 减法）。M_cap_count 是“已开始帧数”，
    // 当前采集帧序号 = M_cap_count-1。
    wire [4:0] M_exp_raw = (M_cap_count - 5'd1) - M_display_ticket;

    reg M_check;
    reg [4:0] M_exp_frames;
    reg M_exp_valid;
    reg M_last_update;

    always @(posedge I_clk_ddr or negedge I_rst_n) begin
        if (!I_rst_n) begin
            M_check       <= 1'b0;
            M_exp_frames  <= 5'd0;
            M_exp_valid   <= 1'b0;
            M_last_update <= 1'b0;
            M_phase_d     <= 2'd0;
            M_phase_settle = 0;
        end
        else begin
            M_check <= M_display_sof;

            // 阶段切换后重新计显示场数，只在阶段稳定两个显示场后才检查
            // “稳态=2 帧 / 注入=3 帧”这两个独立结论。
            if (M_phase != M_phase_d) begin
                M_phase_d      <= M_phase;
                M_phase_settle = 0;
            end
            else begin
                M_phase_d <= M_phase;
                if (M_display_sof && (M_phase_settle < 4))
                    M_phase_settle = M_phase_settle + 1;
            end

            if (M_display_sof) begin
                M_exp_frames <= M_exp_raw;
                M_exp_valid  <= (M_exp_raw >= 5'd1) && (M_exp_raw <= 5'd15);
            end

            if (M_check) begin
                S_checks = S_checks + 1;
                // 阶段常量是独立于 DUT 的地址策略结论：写指针+2 选帧 -> 2 帧；
                // 显示 bank 再老一帧 -> 3 帧。只在阶段稳定两个显示场后检查。
                if (M_exp_valid && (M_phase_settle >= 2)) begin
                    if (M_phase == 2'd1 && M_exp_frames !== 5'd2) begin
                        $display("ERROR: steady phase expected 2 frames, model=%0d", M_exp_frames);
                        S_errors = S_errors + 1;
                    end
                    if (M_phase == 2'd2 && M_exp_frames !== 5'd3) begin
                        $display("ERROR: inject phase expected 3 frames, model=%0d", M_exp_frames);
                        S_errors = S_errors + 1;
                    end
                end
                if (M_exp_valid) begin
                    if (M_exp_frames !== O_lat_frames) begin
                        $display("ERROR: t=%0t O_lat_frames=%0d expected=%0d",
                                 $time, O_lat_frames, M_exp_frames);
                        S_errors = S_errors + 1;
                    end
                    if (O_lat_update === M_last_update) begin
                        $display("ERROR: t=%0t valid sample did not toggle O_lat_update", $time);
                        S_errors = S_errors + 1;
                    end
                end
                else begin
                    if (O_lat_update !== M_last_update) begin
                        $display("ERROR: t=%0t invalid frame age %0d was published",
                                 $time, M_exp_frames);
                        S_errors = S_errors + 1;
                    end
                end
                M_last_update = O_lat_update;
            end
        end
    end

    //-------------------------------------------------------------------------
    // DUT
    //-------------------------------------------------------------------------
    buf_display_latency_probe dut (
        .I_clk_ddr        ( I_clk_ddr        ),
        .I_rst_n          ( I_rst_n          ),
        .I_wr_en          ( I_wr_en          ),
        .I_wr_addr        ( I_wr_addr        ),
        .I_display_rp     ( I_display_rp     ),
        .I_display_vsync  ( I_display_vsync  ),
        .O_lat_frames     ( O_lat_frames     ),
        .O_lat_update     ( O_lat_update     )
    );

    task automatic send_frame(input integer frame_index);
        reg [1:0] bank;
        reg [1:0] next_rp;
        begin
            bank    = frame_index % 4;
            next_rp = (bank + 2'd2) % 4;
            @(posedge I_clk_ddr);
            I_wr_en      <= 1'b1;
            case (bank)
                2'd0:    I_wr_addr <= P_BASE_0;
                2'd1:    I_wr_addr <= P_BASE_1;
                2'd2:    I_wr_addr <= P_BASE_2;
                default: I_wr_addr <= P_BASE_3;
            endcase
            I_display_rp <= next_rp;
            M_rp         <= next_rp;
            @(posedge I_clk_ddr);
            I_wr_en <= 1'b0;
        end
    endtask

    task automatic send_older_frame(input integer frame_index);
        reg [1:0] bank;
        reg [1:0] next_rp;
        begin
            bank    = frame_index % 4;
            // 正常是 (bank+2)%4（帧 n-2）；更老一帧是 (bank+1)%4（帧 n-3）。
            next_rp = (bank + 2'd1) % 4;
            @(posedge I_clk_ddr);
            I_wr_en      <= 1'b1;
            case (bank)
                2'd0:    I_wr_addr <= P_BASE_0;
                2'd1:    I_wr_addr <= P_BASE_1;
                2'd2:    I_wr_addr <= P_BASE_2;
                default: I_wr_addr <= P_BASE_3;
            endcase
            I_display_rp <= next_rp;
            M_rp         <= next_rp;
            @(posedge I_clk_ddr);
            I_wr_en <= 1'b0;
        end
    endtask

    integer n;
    integer gap;

    initial begin
        I_rst_n          = 1'b0;
        I_wr_en          = 1'b0;
        I_wr_addr        = 25'd0;
        I_display_rp     = 2'd0;
        I_display_vsync  = 1'b0;
        M_rp             = 2'd0;
        M_phase          = 2'd0;
        S_checks         = 0;
        S_errors         = 0;
        S_frames_started = 0;

        repeat (10) @(posedge I_clk_ddr);
        I_rst_n = 1'b1;

        // 空载阶段：只有显示场起始，没有采集帧，DUT 不得发布。
        repeat (150) @(posedge I_clk_ddr);

        // 稳态：40 帧覆盖 5 bit 票号回绕，帧间隔伪随机以错开显示场相位。
        for (n = 0; n < 40; n = n + 1) begin
            if (n >= 8) M_phase = 2'd1;
            send_frame(n);
            S_frames_started = n + 1;
            gap = 20 + ((n * 7) % 40);
            repeat (gap) @(posedge I_clk_ddr);
        end

        // 注入：显示 bank 指到更老一帧，帧龄应为 3 帧；保持 8 帧。
        for (n = 40; n < 48; n = n + 1) begin
            M_phase = 2'd2;
            send_older_frame(n);
            S_frames_started = n + 1;
            repeat (40) @(posedge I_clk_ddr);
        end

        // 恢复：回到 (写指针+2)%4，帧龄应回到 2 帧。
        M_phase = 2'd1;
        for (n = 48; n < 56; n = n + 1) begin
            send_frame(n);
            S_frames_started = n + 1;
            repeat (40) @(posedge I_clk_ddr);
        end

        repeat (200) @(posedge I_clk_ddr);
        if (S_checks < 40) begin
            $display("ERROR: only %0d display SOF checks were observed", S_checks);
            S_errors = S_errors + 1;
        end
        if (S_errors == 0) begin
            $display("TB_RESULT: PASS tb_buf_display_latency_probe checks=%0d frames=%0d",
                     S_checks, S_frames_started);
            $finish;
        end
        else begin
            $display("TB_RESULT: FAIL tb_buf_display_latency_probe checks=%0d errors=%0d",
                     S_checks, S_errors);
            $finish;
        end
    end

    initial begin
        // 显示场起始：60 个像素时钟周期，与采集帧周期不同步，覆盖双相位。
        I_display_vsync = 1'b0;
        repeat (20) @(posedge I_clk_pix);
        forever begin
            I_display_vsync <= 1'b1;
            repeat (4) @(posedge I_clk_pix);
            I_display_vsync <= 1'b0;
            repeat (56) @(posedge I_clk_pix);
        end
    end

    initial begin
        #2000000;
        $display("TB_RESULT: FAIL tb_buf_display_latency_probe timeout");
        $finish;
    end

endmodule
