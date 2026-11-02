`timescale 1ns/1ps
// Root-owned native TB seed.  This file is intentionally not run by this
// worker.  It instantiates the real C13 engine and C14 matcher through the
// adapter; no ideal matcher or eval-truth image is used.
module tb_ctl_eg4d_compute_adapter;
    localparam [4:0] TB_S_RX_HEADER_REQ = 5'd1;
    localparam [4:0] TB_S_RX_HEADER_WAIT = 5'd2;
    localparam [4:0] TB_S_RX_TILE_REQ = 5'd3;
    localparam [4:0] TB_S_RX_TILE_WAIT = 5'd4;
    localparam [4:0] TB_S_HEADER_ID_CHECK = 5'd18;
    localparam [4:0] TB_S_DROP = 5'd17;

    reg clk;
    reg rst_n;

    reg        tx_busy;
    reg        tx_done;
    wire       tx_write_en;
    wire [9:0] tx_write_addr;
    wire [7:0] tx_write_data;
    wire       tx_commit;
    wire [10:0] tx_length;

    reg        rx_valid;
    reg [10:0] rx_length;
    wire [9:0] rx_read_addr;
    reg  [7:0] rx_read_data;
    wire       rx_release;

    reg [7:0] rx_mem [0:911];
    reg [7:0] tx_mem [0:31];
    reg [31:0] expected_rows [0:223];

    integer tx_commit_count;
    integer tx_done_count;
    integer tile_write_count;
    integer frame_start_count;
    integer glyph_start_count;
    integer frame_done_count;
    integer i;
    integer sl;
    integer rw;
    integer before_commits;
    integer before_done;
    reg [7:0] observed_header [0:15];
    reg       header_check_seen;

    ctl_eg4d_compute_adapter dut (
        .I_clk_app        (clk),
        .I_rst_app_n      (rst_n),
        .O_tx_write_en    (tx_write_en),
        .O_tx_write_addr  (tx_write_addr),
        .O_tx_write_data  (tx_write_data),
        .O_tx_commit      (tx_commit),
        .O_tx_length      (tx_length),
        .I_tx_busy        (tx_busy),
        .I_tx_done        (tx_done),
        .I_rx_valid       (rx_valid),
        .I_rx_length      (rx_length),
        .O_rx_read_addr   (rx_read_addr),
        .I_rx_read_data   (rx_read_data),
        .O_rx_release     (rx_release),
        .O_busy           (),
        .O_response_valid (),
        .O_frame_result_pulse(),
        .O_debug_state    ()
    );

    initial clk = 1'b0;
    always #20 clk = ~clk;

    // Synchronous one-clock RX RAM and a small TX completion model.  The
    // address must stay stable across the request/wait pair in the adapter.
    integer tx_countdown;
    always @(posedge clk) begin
        if (!rst_n) begin
            rx_read_data <= 8'd0;
            tx_busy <= 1'b0;
            tx_done <= 1'b0;
            tx_countdown <= 0;
            tx_commit_count <= 0;
            tx_done_count <= 0;
            if (rx_valid)
                rx_valid <= 1'b0;
        end else begin
            if (rx_valid)
                rx_read_data <= rx_mem[rx_read_addr];
            else
                rx_read_data <= 8'd0;

            tx_done <= 1'b0;
            if (tx_write_en && !tx_busy)
                tx_mem[tx_write_addr[4:0]] <= tx_write_data;

            if (tx_commit && !tx_busy) begin
                tx_busy <= 1'b1;
                tx_countdown <= 3;
                tx_commit_count <= tx_commit_count + 1;
            end else if (tx_busy) begin
                if (tx_countdown == 0) begin
                    tx_busy <= 1'b0;
                    tx_done <= 1'b1;
                    tx_done_count <= tx_done_count + 1;
                end else begin
                    tx_countdown <= tx_countdown - 1;
                end
            end

            if (rx_release)
                rx_valid <= 1'b0;
        end
    end

    // Address and row-word assertions are independent of any classifier
    // result.  Changed RX addresses must be contiguous; C13 writes must see
    // the expected little-byte-first word for each slot/row.
    reg        addr_seen;
    reg [9:0]  previous_addr;
    always @(posedge clk) begin
        if (!rst_n) begin
            addr_seen <= 1'b0;
            previous_addr <= 10'd0;
            header_check_seen <= 1'b0;
            tile_write_count <= 0;
            frame_start_count <= 0;
            glyph_start_count <= 0;
            frame_done_count <= 0;
        end else begin
            // The adapter deliberately drives address zero in non-read
            // transition states (header check, row write, session check,
            // release).  Start a new contiguous-read run there; treating that
            // idle address as payload would falsely report 15 -> 0 skipped.
            if (!rx_valid) begin
                addr_seen <= 1'b0;
                header_check_seen <= 1'b0;
            end else if ((dut.state == TB_S_RX_HEADER_REQ) ||
                         (dut.state == TB_S_RX_HEADER_WAIT) ||
                         (dut.state == TB_S_RX_TILE_REQ) ||
                         (dut.state == TB_S_RX_TILE_WAIT)) begin
                if (!addr_seen) begin
                    addr_seen <= 1'b1;
                    previous_addr <= rx_read_addr;
                end else if (rx_read_addr != previous_addr) begin
                    if (rx_read_addr != previous_addr + 10'd1)
                        $fatal(1, "RX address skipped: previous=%0d current=%0d state=%0d",
                               previous_addr, rx_read_addr, dut.state);
                    previous_addr <= rx_read_addr;
                end
                if ((dut.state == TB_S_RX_HEADER_WAIT) &&
                    (dut.rx_index_q <= 10'd15)) begin
                    observed_header[dut.rx_index_q] <= rx_read_data;
                    $display("HEADER_BYTE t=%0t index=%0d addr=%0d data=%02x state=%0d",
                             $time, dut.rx_index_q, rx_read_addr,
                             rx_read_data, dut.state);
                end
            end else begin
                addr_seen <= 1'b0;
            end

            if ((dut.state == TB_S_HEADER_ID_CHECK) && !header_check_seen) begin
                header_check_seen <= 1'b1;
                $display("HEADER_CHECK t=%0t state=%0d bytes=%02x%02x%02x%02x session=%08x frame=%08x payload=%0d mask=%02x plate=%0d len=%0d",
                         $time, dut.state, observed_header[0], observed_header[1],
                         observed_header[2], observed_header[3],
                         dut.request_session_q, dut.request_frame_q,
                         dut.request_payload_len_q, dut.request_mask_q,
                         dut.request_plate_valid_q, rx_length);
                if (observed_header[0] !== 8'h4c ||
                    observed_header[1] !== 8'h50 ||
                    observed_header[2] !== 8'h01 ||
                    observed_header[3] !== 8'h01 ||
                    dut.request_payload_len_q !== 16'd896 ||
                    dut.request_mask_reserved_q !== 1'b0 ||
                    dut.request_plate_valid_q !== 1'b1 ||
                    rx_length !== 11'd912)
                    $fatal(1, "first legal header was rejected or captured incorrectly");
            end

            if (dut.state == TB_S_DROP) begin
                $display("DROP t=%0t state=%0d len=%0d h0=%02x h1=%02x ver=%02x kind=%02x payload=%0d reserved=%0d plate=%0d session=%08x frame=%08x",
                         $time, dut.state, rx_length, dut.header_magic0_q,
                         dut.header_magic1_q, dut.header_version_q,
                         dut.header_kind_q, dut.request_payload_len_q,
                         dut.request_mask_reserved_q, dut.request_plate_valid_q,
                         dut.request_session_q, dut.request_frame_q);
            end

            if (dut.engine_tile_write_en) begin
                if (dut.engine_tile_write_addr !==
                    (tile_write_count % 224))
                    $fatal(1, "tile row address mismatch at write %0d",
                           tile_write_count);
                if (dut.tile_word_q !==
                    expected_rows[tile_write_count % 224])
                    $fatal(1, "little-byte-first row mismatch at write %0d",
                           tile_write_count);
                tile_write_count <= tile_write_count + 1;
            end
            if (dut.matcher_frame_start)
                frame_start_count <= frame_start_count + 1;
            if (dut.matcher_glyph_start)
                glyph_start_count <= glyph_start_count + 1;
            if (dut.matcher_frame_done)
                frame_done_count <= frame_done_count + 1;
        end
    end

    task load_fixture;
        input [31:0] session_value;
        input [31:0] frame_value;
        input [10:0] length_value;
        input [7:0] magic0_value;
        input [7:0] mask_value;
        input [7:0] reserved_value;
        integer n;
        reg [7:0] b0;
        reg [7:0] b1;
        reg [7:0] b2;
        reg [7:0] b3;
        begin
            for (n = 0; n < 912; n = n + 1)
                rx_mem[n] = 8'h00;
            rx_mem[0]  = magic0_value;
            rx_mem[1]  = 8'h50;
            rx_mem[2]  = 8'h01;
            rx_mem[3]  = 8'h01;
            rx_mem[4]  = session_value[31:24];
            rx_mem[5]  = session_value[23:16];
            rx_mem[6]  = session_value[15:8];
            rx_mem[7]  = session_value[7:0];
            rx_mem[8]  = frame_value[31:24];
            rx_mem[9]  = frame_value[23:16];
            rx_mem[10] = frame_value[15:8];
            rx_mem[11] = frame_value[7:0];
            rx_mem[12] = 8'h03;
            rx_mem[13] = 8'h80;
            rx_mem[14] = mask_value;
            rx_mem[15] = reserved_value;
            for (sl = 0; sl < 7; sl = sl + 1) begin
                for (rw = 0; rw < 32; rw = rw + 1) begin
                    b0 = 8'h10 ^ sl[7:0] ^ rw[7:0];
                    b1 = 8'h80 | sl[7:0];
                    b2 = 8'h40 | rw[7:0];
                    b3 = 8'h20 + sl[7:0] + rw[7:0];
                    rx_mem[16 + sl*128 + rw*4 + 0] = b0;
                    rx_mem[16 + sl*128 + rw*4 + 1] = b1;
                    rx_mem[16 + sl*128 + rw*4 + 2] = b2;
                    rx_mem[16 + sl*128 + rw*4 + 3] = b3;
                    expected_rows[sl*32 + rw] = {b3,b2,b1,b0};
                end
            end
            rx_length = length_value;
        end
    endtask

    task start_fixture;
        begin
            @(negedge clk);
            rx_valid = 1'b1;
        end
    endtask

    task wait_release;
        begin
            while (rx_valid)
                @(posedge clk);
            @(negedge clk);
        end
    endtask

    task check_response;
        input [31:0] session_value;
        input [31:0] frame_value;
        begin
            if (tx_mem[0] !== 8'h4c || tx_mem[1] !== 8'h50 ||
                tx_mem[2] !== 8'h01 || tx_mem[3] !== 8'h02)
                $fatal(1, "response fixed header mismatch");
            if ({tx_mem[4],tx_mem[5],tx_mem[6],tx_mem[7]} !== session_value)
                $fatal(1, "response session echo mismatch");
            if ({tx_mem[8],tx_mem[9],tx_mem[10],tx_mem[11]} !== frame_value)
                $fatal(1, "response frame echo mismatch");
            if (tx_mem[12] !== 8'h00 || tx_mem[13] !== 8'h10)
                $fatal(1, "response payload length mismatch");
            if (!((tx_mem[14] === 8'h00 && tx_mem[15] === 8'h00) ||
                  (tx_mem[14] === 8'h7f && tx_mem[15] === 8'h01)))
                $fatal(1, "stable valid header is not atomic");
            if (tx_mem[16] !== dut.response_res_q[7:0] ||
                tx_mem[25] !== dut.response_raw_code_q[7:0])
                $fatal(1, "response slot0 low-byte packing mismatch");
            if (tx_mem[23][7:4] !== 4'h0 || tx_mem[24][7] !== 1'b0)
                $fatal(1, "response reserved/result mask bits set");
        end
    endtask

    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        rx_valid = 1'b0;
        rx_length = 11'd0;
        rx_read_data = 8'd0;
        tx_busy = 1'b0;
        tx_done = 1'b0;
        tx_commit_count = 0;
        tx_done_count = 0;
        tile_write_count = 0;
        frame_start_count = 0;
        glyph_start_count = 0;
        frame_done_count = 0;
        for (i = 0; i < 16; i = i + 1)
            observed_header[i] = 8'h00;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;

        // Five complete requests in the second session use the real C13/C14
        // path.  HOLD_K=5 is frozen in the adapter, so Root can inspect the
        // fifth response for the normal confirmation behavior without ever
        // replacing the matcher with an ideal model.
        load_fixture(32'h01020304, 32'd1, 11'd912, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        before_commits = tx_commit_count;
        before_done = tx_done_count;
        if (before_commits != 1 || before_done != 1)
            $fatal(1, "first response did not complete");
        check_response(32'h01020304, 32'd1);

        load_fixture(32'h01020304, 32'd2, 11'd912, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 2 || tx_done_count != 2)
            $fatal(1, "second response did not complete");
        check_response(32'h01020304, 32'd2);

        // A new session resets C14 stability and retires the first session.
        load_fixture(32'h05060708, 32'd1, 11'd912, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 3 || tx_done_count != 3)
            $fatal(1, "third response did not complete");
        check_response(32'h05060708, 32'd1);

        load_fixture(32'h05060708, 32'd2, 11'd912, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 4 || tx_done_count != 4)
            $fatal(1, "fourth response did not complete");
        check_response(32'h05060708, 32'd2);

        load_fixture(32'h05060708, 32'd3, 11'd912, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 5 || tx_done_count != 5)
            $fatal(1, "fifth response did not complete");
        check_response(32'h05060708, 32'd3);

        load_fixture(32'h05060708, 32'd4, 11'd912, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 6 || tx_done_count != 6)
            $fatal(1, "sixth response did not complete");
        check_response(32'h05060708, 32'd4);

        load_fixture(32'h05060708, 32'd5, 11'd912, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 7 || tx_done_count != 7)
            $fatal(1, "seventh response did not complete");
        check_response(32'h05060708, 32'd5);

        // The immediately retired session and a same-session replay must both
        // be released before a fourth tile write or TX commit.
        load_fixture(32'h01020304, 32'd3, 11'd912, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 7 || tile_write_count != 7*224)
            $fatal(1, "old session was not dropped before compute");

        load_fixture(32'h05060708, 32'd1, 11'd912, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 7 || tile_write_count != 7*224)
            $fatal(1, "old frame was not dropped before compute");

        // Structural bad length/magic/reserved packets are also dropped.
        load_fixture(32'h05060708, 32'd2, 11'd911, 8'h4c, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 7)
            $fatal(1, "bad length unexpectedly produced response");

        load_fixture(32'h05060708, 32'd2, 11'd912, 8'h00, 8'h7f, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 7)
            $fatal(1, "bad magic unexpectedly produced response");

        load_fixture(32'h05060708, 32'd2, 11'd912, 8'h4c, 8'hff, 8'h01);
        start_fixture;
        wait_release;
        if (tx_commit_count != 7)
            $fatal(1, "reserved request bit unexpectedly produced response");

        if (frame_start_count != 7 || glyph_start_count != 49 ||
            frame_done_count != 7)
            $fatal(1, "compute scheduling count mismatch: frames=%0d glyphs=%0d done=%0d",
                   frame_start_count, glyph_start_count, frame_done_count);
        $display("EG4D adapter TB seed passed protocol/row/scheduling checks");
        $finish;
    end
endmodule
