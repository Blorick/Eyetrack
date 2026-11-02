//=============================================================================
//  tb_ph1p_line_bridge  --  line packet, validation and backpressure TB
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk_50m model
//  复位     : I_rst_n
//  参数     : IMG_W=4, P_SLOT=4, P_LAT=1, P_VALIDATE_FRAME=1
//  验证状态 : Questa 通过时输出 TB_RESULT: PASS
//=============================================================================
`timescale 1ns / 1ps

module tb_ph1p_line_bridge;
    localparam integer P_IMG_W = 4;
    localparam integer P_HDR = 16;
    localparam integer P_FRAME_LEN = P_HDR + P_IMG_W;

    reg S_clk = 1'b0;
    always #5 S_clk = ~S_clk;

    reg I_rst_n;
    reg I_rx_valid;
    reg [15:0] I_rx_len;
    wire [10:0] O_rx_addr;
    reg [7:0] I_rx_rdata;
    wire O_rx_release;
    reg I_tx_ready;
    wire O_tx_we;
    wire [10:0] O_tx_addr;
    wire [7:0] O_tx_wdata;
    wire O_tx_commit;
    reg I_vsync;
    reg I_de;
    reg I_user;
    reg [7:0] I_y;
    reg [4:0] I_mode;
    reg I_enable;
    wire O_vsync;
    wire O_de;
    wire O_user;
    wire [7:0] O_y;
    wire O_styled;
    wire O_link_ok;
    wire [15:0] O_out_line;

    reg [7:0] S_tx_mem [0:63];
    reg [7:0] S_rx_mem [0:63];
    reg S_rx_start;
    reg [15:0] S_rx_start_len;
    integer S_commit_count;
    integer S_release_count;
    integer S_style_hits;
    integer S_checks;
    integer S_errors;
    reg S_watch_style;
    reg [7:0] S_expected_style;
    integer S_i;

    ph1p_line_bridge #(
        .IMG_W             ( P_IMG_W ),
        .P_SLOT            ( 4 ),
        .P_LAT             ( 1 ),
        .P_VALIDATE_FRAME  ( 1 ),
        .P_REMOTE_ONLY     ( 0 )
    ) dut (
        .I_clk         ( S_clk         ),
        .I_rst_n       ( I_rst_n       ),
        .I_rx_valid    ( I_rx_valid    ),
        .I_rx_len      ( I_rx_len      ),
        .O_rx_addr     ( O_rx_addr     ),
        .I_rx_rdata    ( I_rx_rdata    ),
        .O_rx_release  ( O_rx_release  ),
        .I_tx_ready    ( I_tx_ready    ),
        .O_tx_we       ( O_tx_we       ),
        .O_tx_addr     ( O_tx_addr     ),
        .O_tx_wdata    ( O_tx_wdata    ),
        .O_tx_commit   ( O_tx_commit   ),
        .I_vsync       ( I_vsync       ),
        .I_de          ( I_de          ),
        .I_user        ( I_user        ),
        .I_y          ( I_y           ),
        .I_mode        ( I_mode        ),
        .I_enable      ( I_enable      ),
        .O_vsync       ( O_vsync       ),
        .O_de          ( O_de          ),
        .O_user        ( O_user        ),
        .O_y           ( O_y           ),
        .O_styled      ( O_styled      ),
        .O_link_ok     ( O_link_ok     ),
        .O_out_line    ( O_out_line    )
    );

    // This models the link's registered RX memory output and counts only the
    // public handshakes.  No DUT hierarchy is forced or deposited.
    always @(posedge S_clk) begin
        I_rx_rdata <= S_rx_mem[O_rx_addr];
        if (S_rx_start) begin
            I_rx_valid <= 1'b1;
            I_rx_len <= S_rx_start_len;
            S_rx_start <= 1'b0;
        end else begin
            I_rx_valid <= 1'b0;
            if (O_rx_release)
                I_rx_len <= 16'd0;
        end
        if (O_tx_we)
            S_tx_mem[O_tx_addr] <= O_tx_wdata;
        if (O_tx_commit)
            S_commit_count = S_commit_count + 1;
        if (O_rx_release)
            S_release_count = S_release_count + 1;
        #1;
        // The bridge registers the raster position and memory read.  The
        // first visible styled byte is therefore one column later; count the
        // returned payload values rather than requiring the zero-column byte.
        if (S_watch_style && O_styled && O_de &&
            ((O_y === 8'hD0) || (O_y === 8'hD1) ||
             (O_y === 8'hD2) || (O_y === 8'hD3)))
            S_style_hits = S_style_hits + 1;
    end

    task automatic pulse_frame;
        begin
            @(negedge S_clk);
            I_vsync = 1'b1;
            I_de = 1'b0;
            I_user = 1'b0;
            @(posedge S_clk);
            @(negedge S_clk);
            I_vsync = 1'b0;
        end
    endtask

    // V_drop_after >= 0 makes the link unavailable from that pixel onward.
    task automatic drive_line;
        input [4:0] V_mode;
        input integer V_seed;
        input integer V_drop_after;
        input V_user;
        integer S_col_i;
        begin
            I_mode = V_mode;
            for (S_col_i = 0; S_col_i < P_IMG_W; S_col_i = S_col_i + 1) begin
                @(negedge S_clk);
                I_de = 1'b1;
                // Production uivtc marks the frame's first active pixel only;
                // line boundaries are represented by DE rising edges.
                I_user = (S_col_i == 0) && V_user;
                I_y = V_seed + S_col_i;
                I_tx_ready = (V_drop_after < 0) || (S_col_i < V_drop_after);
                @(posedge S_clk);
            end
            @(negedge S_clk);
            I_de = 1'b0;
            I_user = 1'b0;
            I_y = 8'd0;
            I_tx_ready = 1'b1;
            @(posedge S_clk);
        end
    endtask

    task automatic wait_for_commit;
        input integer V_target;
        integer S_wait_i;
        begin
            S_wait_i = 0;
            while ((S_commit_count < V_target) && (S_wait_i < 80)) begin
                @(posedge S_clk);
                S_wait_i = S_wait_i + 1;
            end
            #1;
            if (S_commit_count < V_target) begin
                $display("FAIL wait commit target=%0d got=%0d", V_target, S_commit_count);
                S_errors = S_errors + 1;
            end
        end
    endtask

    task automatic wait_for_release;
        input integer V_target;
        integer S_wait_i;
        begin
            S_wait_i = 0;
            while ((S_release_count < V_target) && (S_wait_i < 120)) begin
                @(posedge S_clk);
                S_wait_i = S_wait_i + 1;
            end
            #1;
            if (S_release_count < V_target) begin
                $display("FAIL wait RX release target=%0d got=%0d", V_target, S_release_count);
                S_errors = S_errors + 1;
            end
        end
    endtask

    task automatic arm_rx_packet;
        input [15:0] V_line;
        input [31:0] V_frame;
        input [4:0] V_mode;
        input V_valid;
        input [15:0] V_length;
        integer S_payload_i;
        begin
            for (S_payload_i = 0; S_payload_i < 64; S_payload_i = S_payload_i + 1)
                S_rx_mem[S_payload_i] = 8'd0;
            S_rx_mem[0] = 8'hA5;
            S_rx_mem[1] = 8'h01;
            S_rx_mem[2] = V_line[7:0];
            S_rx_mem[3] = V_line[15:8];
            S_rx_mem[4] = {3'd0, V_mode};
            S_rx_mem[6] = P_IMG_W[7:0];
            S_rx_mem[7] = 8'd0;
            S_rx_mem[8] = V_frame[7:0];
            S_rx_mem[9] = V_frame[15:8];
            S_rx_mem[10] = V_frame[23:16];
            S_rx_mem[11] = V_frame[31:24];
            S_rx_mem[12] = V_valid ? 8'h01 : 8'h00;
            S_rx_mem[16] = 8'hD0;
            S_rx_mem[17] = 8'hD1;
            S_rx_mem[18] = 8'hD2;
            S_rx_mem[19] = 8'hD3;
            S_rx_start_len = V_length;
            S_rx_start = 1'b1;
        end
    endtask

    task automatic check_tx_header;
        input [15:0] V_line;
        input [31:0] V_frame;
        input [4:0] V_mode;
        begin
            S_checks = S_checks + 1;
            if ((S_tx_mem[0] !== 8'hA5) || (S_tx_mem[1] !== 8'h01) ||
                ({S_tx_mem[3], S_tx_mem[2]} !== V_line) ||
                (S_tx_mem[4] !== {3'd0, V_mode}) ||
                ({S_tx_mem[7], S_tx_mem[6]} !== P_IMG_W) ||
                ({S_tx_mem[11], S_tx_mem[10], S_tx_mem[9], S_tx_mem[8]} !== V_frame)) begin
                $display("FAIL TX header magic=%02h%02h line=%02h%02h mode=%02h len=%02h%02h frame=%02h%02h%02h%02h",
                         S_tx_mem[0], S_tx_mem[1], S_tx_mem[3], S_tx_mem[2], S_tx_mem[4],
                         S_tx_mem[7], S_tx_mem[6], S_tx_mem[11], S_tx_mem[10], S_tx_mem[9], S_tx_mem[8]);
                S_errors = S_errors + 1;
            end
            if ((S_tx_mem[16] !== 8'h10) || (S_tx_mem[17] !== 8'h11) ||
                (S_tx_mem[18] !== 8'h12) || (S_tx_mem[19] !== 8'h13)) begin
                $display("FAIL TX payload bytes=%02h%02h%02h%02h", S_tx_mem[16], S_tx_mem[17], S_tx_mem[18], S_tx_mem[19]);
                S_errors = S_errors + 1;
            end
        end
    endtask

    initial begin
        I_rst_n = 1'b0;
        I_rx_valid = 1'b0;
        I_rx_len = 16'd0;
        I_rx_rdata = 8'd0;
        I_tx_ready = 1'b1;
        I_vsync = 1'b0;
        I_de = 1'b0;
        I_user = 1'b0;
        I_y = 8'd0;
        I_mode = 5'd2;
        I_enable = 1'b1;
        S_rx_start = 1'b0;
        S_rx_start_len = 16'd0;
        S_commit_count = 0;
        S_release_count = 0;
        S_style_hits = 0;
        S_checks = 0;
        S_errors = 0;
        S_watch_style = 1'b0;
        S_expected_style = 8'd0;
        for (S_i = 0; S_i < 64; S_i = S_i + 1) begin
            S_tx_mem[S_i] = 8'd0;
            S_rx_mem[S_i] = 8'd0;
        end

        repeat (4) @(posedge S_clk);
        I_rst_n = 1'b1;
        repeat (2) @(posedge S_clk);

        // Frame 1: deliberate half-line backpressure must produce no commit.
        pulse_frame();
        drive_line(5'd2, 8'h10, 2, 1'b1);
        repeat (25) @(posedge S_clk);
        S_checks = S_checks + 1;
        if (S_commit_count !== 0) begin
            $display("FAIL half-line drop committed unexpectedly count=%0d", S_commit_count);
            S_errors = S_errors + 1;
        end

        // Frame 2: complete line 0 is the good baseline for all RX checks.
        pulse_frame();
        drive_line(5'd2, 8'h10, -1, 1'b1);
        wait_for_commit(1);
        check_tx_header(16'd0, 32'd2, 5'd2);

        // A valid frame-2/mode-2 response is displayed one line later.
        arm_rx_packet(16'd0, 32'd2, 5'd2, 1'b1, P_FRAME_LEN);
        wait_for_release(1);
        repeat (3) @(posedge S_clk);
        S_style_hits = 0;
        S_expected_style = 8'hD0;
        S_watch_style = 1'b1;
        drive_line(5'd2, 8'h20, -1, 1'b0);
        S_watch_style = 1'b0;
        S_checks = S_checks + 1;
        if (S_style_hits == 0) begin
            $display("FAIL valid line response was never styled line=%0d frame=%0d slot0_ok=%b slot0_line=%0d slot0_frame=%0d slot0_mode=%0d rd_ok=%b rd_data=%02h out_line=%0d releases=%0d",
                     dut.S_line, dut.S_frame, dut.S_slot_ok[0], dut.S_slot_line[0],
                     dut.S_slot_frame[0], dut.S_slot_mode[0], dut.S_rd_ok, dut.S_rd_data,
                     O_out_line, S_release_count);
            S_errors = S_errors + 1;
        end

        // Frame 3: late previous-frame, wrong-mode and bad-length packets all
        // release cleanly and must not make a pixel look valid.
        pulse_frame();
        I_mode = 5'd2;
        arm_rx_packet(16'd0, 32'd2, 5'd2, 1'b1, P_FRAME_LEN);
        wait_for_release(2);
        arm_rx_packet(16'd0, 32'd3, 5'd3, 1'b1, P_FRAME_LEN);
        wait_for_release(3);
        arm_rx_packet(16'd0, 32'd3, 5'd2, 1'b1, P_FRAME_LEN - 1);
        wait_for_release(4);
        S_style_hits = 0;
        S_watch_style = 1'b1;
        drive_line(5'd2, 8'h30, -1, 1'b1);
        drive_line(5'd2, 8'h40, -1, 1'b0);
        S_watch_style = 1'b0;
        S_checks = S_checks + 1;
        if (S_style_hits != 0) begin
            $display("FAIL invalid frame/mode/length response styled %0d pixels", S_style_hits);
            S_errors = S_errors + 1;
        end

        // A matching frame/mode/length packet restores styling on the next line.
        arm_rx_packet(16'd1, 32'd3, 5'd2, 1'b1, P_FRAME_LEN);
        wait_for_release(5);
        repeat (3) @(posedge S_clk);
        S_style_hits = 0;
        S_watch_style = 1'b1;
        drive_line(5'd2, 8'h50, -1, 1'b0);
        S_watch_style = 1'b0;
        S_checks = S_checks + 1;
        if (S_style_hits == 0) begin
            $display("FAIL matching frame response was not restored line=%0d frame=%0d slot1_ok=%b slot1_line=%0d slot1_frame=%0d slot1_mode=%0d rd_ok=%b rd_data=%02h out_line=%0d releases=%0d",
                     dut.S_line, dut.S_frame, dut.S_slot_ok[1], dut.S_slot_line[1],
                     dut.S_slot_frame[1], dut.S_slot_mode[1], dut.S_rd_ok, dut.S_rd_data,
                     O_out_line, S_release_count);
            S_errors = S_errors + 1;
        end

        if (S_errors == 0)
            $display("TB_RESULT: PASS tb_ph1p_line_bridge checks=%0d commits=%0d releases=%0d errors=0", S_checks, S_commit_count, S_release_count);
        else
            $display("TB_RESULT: FAIL tb_ph1p_line_bridge checks=%0d commits=%0d releases=%0d errors=%0d", S_checks, S_commit_count, S_release_count, S_errors);
        $finish;
    end
endmodule
