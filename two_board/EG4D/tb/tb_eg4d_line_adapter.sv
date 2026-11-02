`timescale 1ns/1ps

// Application-boundary line bridge test.  The Ethernet link is represented by
// its validated ping-pong memory API; the filter is a one-cycle delayed raster
// pass-through so header, line-number, frame-sequence, valid-bit and gap
// handling are exercised without hiding the registered-read timing.
module tb_eg4d_line_adapter;
    localparam integer W = 8;
    localparam integer H = 32;

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    always #5 clk = ~clk;

    reg rx_valid;
    reg [15:0] rx_len;
    wire [10:0] rx_addr;
    reg [7:0] rx_rdata;
    wire rx_release;
    reg tx_ready;
    wire tx_we;
    wire [10:0] tx_addr;
    wire [7:0] tx_wdata;
    wire tx_commit;
    wire fil_vsync, fil_hsync, fil_de, fil_user, fil_last;
    wire [7:0] fil_y;
    wire [4:0] fil_mode;
    reg out_vsync, out_hsync, out_de, out_user, out_last;
    reg [7:0] out_y;
    reg out_edge, out_window_valid;

    reg [7:0] rx_mem [0:H-1];
    reg [7:0] tx_mem [0:H-1];
    reg [7:0] filter_cur [0:W-1];
    reg [7:0] filter_prev [0:W-1];
    reg [7:0] filter_prev2 [0:W-1];
    integer filter_col;
    reg filter_prev_valid;
    reg filter_prev2_valid;
    integer commit_count;
    integer release_count;
    integer errors;
    integer checks;
    integer n;

    eg4d_line_adapter #(.IMG_W(W), .P_HDR_BYTES(16), .P_APP_MAGIC(16'h01A5)) dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_rx_valid(rx_valid), .I_rx_len(rx_len), .O_rx_addr(rx_addr),
        .I_rx_rdata(rx_rdata), .O_rx_release(rx_release),
        .I_tx_ready(tx_ready), .O_tx_wbuf(), .O_tx_we(tx_we),
        .O_tx_addr(tx_addr), .O_tx_wdata(tx_wdata), .O_tx_commit(tx_commit),
        .O_fil_vsync(fil_vsync), .O_fil_hsync(fil_hsync), .O_fil_de(fil_de),
        .O_fil_user(fil_user), .O_fil_last(fil_last), .O_fil_y(fil_y),
        .O_fil_mode(fil_mode), .I_fil_vsync(out_vsync),
        .I_fil_hsync(out_hsync), .I_fil_de(out_de), .I_fil_user(out_user),
        .I_fil_last(out_last), .I_fil_y(out_y), .I_fil_edge(out_edge),
        .I_fil_window_valid(out_window_valid)
    );

    // Registered read response from the link RX mailbox and a one-line
    // delayed filter output.  The real 3x3 filter emits line L-1 while the
    // adapter streams line L; modelling that latency is essential because a
    // one-cycle pass-through would pair line L's payload with line L-1's
    // header.
    always @(posedge clk) begin
        rx_rdata <= rx_mem[rx_addr];
        out_vsync <= 1'b0;
        out_hsync <= 1'b0;
        out_de <= 1'b0;
        out_user <= 1'b0;
        out_last <= 1'b0;
        out_y <= 8'd0;
        out_edge <= 1'b0;
        out_window_valid <= 1'b0;
        if (fil_de) begin
            if (fil_user) begin
                filter_col <= 1;
                if ((fil_mode == 5'd17) ? filter_prev2_valid : filter_prev_valid) begin
                    out_de <= 1'b1;
                    out_user <= 1'b1;
                    out_hsync <= 1'b1;
                    out_y <= (fil_mode == 5'd17) ? filter_prev2[0] : filter_prev[0];
                    out_window_valid <= 1'b1;
                end
                filter_cur[0] <= fil_y;
            end
            else begin
                if ((fil_mode == 5'd17) ? filter_prev2_valid : filter_prev_valid) begin
                    out_de <= 1'b1;
                    out_hsync <= 1'b1;
                    out_last <= fil_last;
                    out_y <= (fil_mode == 5'd17) ? filter_prev2[filter_col] :
                             filter_prev[filter_col];
                    out_window_valid <= 1'b1;
                end
                filter_cur[filter_col] <= fil_y;
                if (fil_last) begin
                    for (n = 0; n < W-1; n = n + 1) begin
                        filter_prev2[n] <= filter_prev[n];
                        filter_prev[n] <= filter_cur[n];
                    end
                    filter_prev2[W-1] <= filter_prev[W-1];
                    filter_prev[W-1] <= fil_y;
                    if (fil_vsync) begin
                        filter_prev_valid <= 1'b1;
                        filter_prev2_valid <= 1'b0;
                    end
                    else begin
                        filter_prev2_valid <= filter_prev_valid;
                        filter_prev_valid <= 1'b1;
                    end
                    filter_col <= 0;
                end
                else
                    filter_col <= filter_col + 1;
            end
        end
        if (tx_we)
            tx_mem[tx_addr] <= tx_wdata;
        if (tx_commit)
            commit_count = commit_count + 1;
        if (rx_release)
            release_count = release_count + 1;
    end

    task automatic make_line;
        input [15:0] line_no;
        input [4:0] mode;
        input [31:0] seq;
        input sof;
        input [7:0] base;
        begin
            rx_mem[0]  = 8'hA5; rx_mem[1]  = 8'h01;
            rx_mem[2]  = line_no[7:0]; rx_mem[3] = line_no[15:8];
            rx_mem[4]  = {3'b000, mode};
            rx_mem[5]  = {7'd0, sof};
            rx_mem[6]  = W[7:0]; rx_mem[7] = 8'h00;
            rx_mem[8]  = seq[7:0]; rx_mem[9]  = seq[15:8];
            rx_mem[10] = seq[23:16]; rx_mem[11] = seq[31:24];
            rx_mem[12] = 8'h00; rx_mem[13] = 8'h00;
            rx_mem[14] = 8'h00; rx_mem[15] = 8'h00;
            for (n = 0; n < W; n = n + 1)
                rx_mem[16+n] = base + n[7:0];
        end
    endtask

    task automatic send_line;
        input [15:0] line_no;
        input [4:0] mode;
        input [31:0] seq;
        input sof;
        input [7:0] base;
        integer t;
        begin
            make_line(line_no, mode, seq, sof, base);
            @(negedge clk);
            rx_valid = 1'b1;
            t = 0;
            while (!rx_release && t < 2000) begin
                @(posedge clk);
                t = t + 1;
            end
            if (!rx_release) begin
                $display("FAIL line=%0d release timeout state=%0d", line_no, dut.S_state);
                errors = errors + 1;
            end
            @(negedge clk);
            rx_valid = 1'b0;
            repeat (3) @(posedge clk);
        end
    endtask

    // A real ping-pong stream endpoint may publish the next buffer before the
    // consumer's release toggle has made O_rx_valid fall.  Keep the level high
    // across two frames to prove that the adapter's one-cycle S_WAIT neither
    // re-reads the first buffer nor deadlocks waiting for a low level.
    task automatic send_two_lines_contiguous;
        input [15:0] line0;
        input [15:0] line1;
        input [4:0] mode;
        input [31:0] seq;
        input [7:0] base0;
        input [7:0] base1;
        integer t;
        begin
            make_line(line0, mode, seq, 1'b1, base0);
            @(negedge clk);
            rx_valid = 1'b1;
            t = 0;
            while (!rx_release && t < 2000) begin
                @(posedge clk);
                t = t + 1;
            end
            if (!rx_release) begin
                $display("FAIL contiguous first line=%0d release timeout state=%0d",
                         line0, dut.S_state);
                errors = errors + 1;
            end

            // Consume the release pulse, then replace the mailbox contents
            // while rx_valid remains asserted.  S_WAIT must absorb the old
            // level for one cycle and S_IDLE must accept this new line.
            @(negedge clk);
            make_line(line1, mode, seq, 1'b0, base1);
            t = 0;
            while (!rx_release && t < 2000) begin
                @(posedge clk);
                t = t + 1;
            end
            if (!rx_release) begin
                $display("FAIL contiguous second line=%0d release timeout state=%0d",
                         line1, dut.S_state);
                errors = errors + 1;
            end
            @(negedge clk);
            rx_valid = 1'b0;
            repeat (3) @(posedge clk);
        end
    endtask

    task automatic wait_commit_target;
        input integer target;
        integer t;
        begin
            t = 0;
            while (commit_count < target && t < 2000) begin
                @(posedge clk);
                t = t + 1;
            end
            if (commit_count < target) begin
                $display("FAIL commit timeout target=%0d got=%0d state=%0d", target,
                         commit_count, dut.S_state);
                errors = errors + 1;
            end
            @(posedge clk);
        end
    endtask

    task automatic check_header;
        input [15:0] expected_line;
        input [4:0] expected_mode;
        input [31:0] expected_seq;
        input expected_valid;
        input [7:0] expected_payload;
        begin
            checks = checks + 1;
            if ({tx_mem[1], tx_mem[0]} != 16'h01A5 ||
                {tx_mem[3], tx_mem[2]} != expected_line ||
                tx_mem[4] != {3'b000, expected_mode} ||
                {tx_mem[7], tx_mem[6]} != 16'd8 ||
                {tx_mem[11], tx_mem[10], tx_mem[9], tx_mem[8]} != expected_seq ||
                tx_mem[12] != {7'd0, expected_valid} ||
                tx_mem[13] != 8'h00 || tx_mem[14] != 8'h00 || tx_mem[15] != 8'h00 ||
                tx_mem[16] != expected_payload) begin
                $display("FAIL header line=%0d mode=%0d seq=%08h valid=%b payload=%02h",
                         {tx_mem[3], tx_mem[2]}, tx_mem[4][4:0],
                         {tx_mem[11], tx_mem[10], tx_mem[9], tx_mem[8]},
                         tx_mem[12][0], tx_mem[16]);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        rx_valid = 1'b0; rx_len = 16'd24; rx_rdata = 8'd0; tx_ready = 1'b1;
        out_vsync = 1'b0; out_hsync = 1'b0; out_de = 1'b0; out_user = 1'b0;
        out_last = 1'b0; out_y = 8'd0; out_edge = 1'b0;
        out_window_valid = 1'b0; commit_count = 0; release_count = 0;
        filter_col = 0; filter_prev_valid = 1'b0; filter_prev2_valid = 1'b0;
        errors = 0; checks = 0;
        for (n = 0; n < H; n = n + 1) begin
            rx_mem[n] = 8'd0;
            tx_mem[n] = 8'd0;
        end
        for (n = 0; n < W; n = n + 1) begin
            filter_cur[n] = 8'd0;
            filter_prev[n] = 8'd0;
            filter_prev2[n] = 8'd0;
        end

        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // Mode 18: first line only primes the filter; line 1 returns line 0
        // with valid=0; line 2 returns line 1 with valid=1.
        send_line(16'd0, 5'd18, 32'h1122_3344, 1'b1, 8'h20);
        if (commit_count != 0) begin
            $display("FAIL SOF line unexpectedly committed count=%0d", commit_count);
            errors = errors + 1;
        end
        send_line(16'd1, 5'd18, 32'h1122_3344, 1'b0, 8'h40);
        wait_commit_target(1);
        check_header(16'd0, 5'd18, 32'h1122_3344, 1'b0, 8'h20);
        send_line(16'd2, 5'd18, 32'h1122_3344, 1'b0, 8'h60);
        wait_commit_target(2);
        check_header(16'd1, 5'd18, 32'h1122_3344, 1'b1, 8'h40);

        // A missing line clears the window; the first contiguous line after
        // the gap is explicitly invalid and starts a fresh segment.
        send_line(16'd4, 5'd18, 32'h1122_3344, 1'b0, 8'h80);
        if (commit_count != 2) begin
            $display("FAIL gap line committed count=%0d", commit_count);
            errors = errors + 1;
        end
        send_line(16'd5, 5'd18, 32'h1122_3344, 1'b0, 8'hA0);
        wait_commit_target(3);
        check_header(16'd4, 5'd18, 32'h1122_3344, 1'b0, 8'h80);

        // Mode 17 has the extra majority pass: line 3 is the first returned
        // line whose two-row history is complete, and its header names line 1.
        send_line(16'd0, 5'd17, 32'h5566_7788, 1'b1, 8'h10);
        send_line(16'd1, 5'd17, 32'h5566_7788, 1'b0, 8'h20);
        send_line(16'd2, 5'd17, 32'h5566_7788, 1'b0, 8'h30);
        send_line(16'd3, 5'd17, 32'h5566_7788, 1'b0, 8'h40);
        wait_commit_target(4);
        check_header(16'd1, 5'd17, 32'h5566_7788, 1'b1, 8'h20);

        // The link-level RX_VALID may stay high when a second ping-pong buffer
        // is already published.  It must still be consumed exactly once.
        begin : contiguous_check
            integer commit_before;
            integer release_before;
            commit_before = commit_count;
            release_before = release_count;
            send_two_lines_contiguous(16'd10, 16'd11, 5'd18,
                                      32'h99AA_BBCC, 8'hC0, 8'hD0);
            checks = checks + 2;
            if (release_count != release_before + 2) begin
                $display("FAIL contiguous releases before=%0d after=%0d",
                         release_before, release_count);
                errors = errors + 1;
            end
            if (commit_count != commit_before + 1) begin
                $display("FAIL contiguous commits before=%0d after=%0d",
                         commit_before, commit_count);
                errors = errors + 1;
            end
        end

        if (release_count < 9) begin
            $display("FAIL release_count=%0d expected>=9", release_count);
            errors = errors + 1;
        end
        if (errors == 0)
            $display("TB_RESULT: PASS line adapter checks=%0d commits=%0d releases=%0d",
                     checks, commit_count, release_count);
        else
            $display("TB_RESULT: FAIL line adapter checks=%0d errors=%0d",
                     checks, errors);
        $finish;
    end
endmodule
