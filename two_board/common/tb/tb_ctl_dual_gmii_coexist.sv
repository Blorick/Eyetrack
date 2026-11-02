`timescale 1ns/1ps

// Exercise the two real packet endpoints behind one GMII arbiter.  Both
// applications commit before the first grant is available; OCR (client 0)
// must therefore own the first complete frame, followed by the 0x88B6 stream
// frame.  The monitor samples the shared GMII pins after each TX clock edge,
// so a frame cannot be counted twice when the scheduler changes owner.
module tb_ctl_dual_gmii_coexist;
    localparam [47:0] LOCAL_MAC = 48'h02_00_00_65_43_21;
    localparam [47:0] PEER_MAC  = 48'h02_00_00_12_34_56;

    reg clk_tx = 1'b0;
    reg clk_25 = 1'b0;
    reg clk_75 = 1'b0;
    reg rst_n  = 1'b0;
    always #4.000 clk_tx = ~clk_tx;       // 125 MHz GMII
    always #20.000 clk_25 = ~clk_25;      // OCR application clock
    always #6.666 clk_75 = ~clk_75;       // mode application clock

    reg        ocr_we, ocr_commit;
    reg [9:0]  ocr_addr;
    reg [7:0]  ocr_wdata;
    reg [10:0] ocr_len;
    wire       ocr_busy, ocr_done;
    wire       ocr_req, ocr_grant, ocr_done_tx;
    wire [7:0] ocr_txd;
    wire       ocr_tx_en, ocr_tx_er;

    reg        mode_we, mode_commit;
    reg [10:0] mode_addr;
    reg [7:0]  mode_wdata;
    wire       mode_ready, mode_wbuf;
    wire       mode_req, mode_grant, mode_done_tx;
    wire [7:0] mode_txd;
    wire       mode_tx_en, mode_tx_er;

    wire [7:0] gmii_txd;
    wire       gmii_tx_en, gmii_tx_er;
    wire       owner_valid, owner;

    ctl_lpr_packet_link #(
        .P_LOCAL_MAC(LOCAL_MAC), .P_PEER_MAC(PEER_MAC),
        .P_ETHERTYPE(16'h88B5), .P_TX_ARB_EN(1)
    ) u_ocr (
        .I_clk_app(clk_25), .I_rst_app_n(rst_n),
        .I_clk_tx(clk_tx), .I_rst_tx_n(rst_n),
        .I_clk_rx(clk_tx), .I_rst_rx_n(rst_n),
        .I_tx_write_en(ocr_we), .I_tx_write_addr(ocr_addr),
        .I_tx_write_data(ocr_wdata), .I_tx_commit(ocr_commit),
        .I_tx_length(ocr_len), .O_tx_busy(ocr_busy), .O_tx_done(ocr_done),
        .I_tx_grant(ocr_grant), .O_tx_request(ocr_req),
        .O_tx_done_tx(ocr_done_tx),
        .O_rx_valid(), .O_rx_length(), .I_rx_read_addr(10'd0),
        .O_rx_read_data(), .I_rx_release(1'b0),
        .O_gmii_txd(ocr_txd), .O_gmii_tx_en(ocr_tx_en),
        .O_gmii_tx_er(ocr_tx_er), .I_gmii_rxd(8'd0),
        .I_gmii_rx_dv(1'b0), .I_gmii_rx_er(1'b0),
        .O_tx_busy_drop_count(), .O_tx_invalid_drop_count(),
        .O_rx_bad_frame_count(), .O_rx_busy_drop_count(),
        .O_rx_oversize_drop_count(), .O_bad_frame_count(),
        .O_busy_drop_count()
    );

    ctl_stream_link #(
        .P_LOCAL_MAC(LOCAL_MAC), .P_PEER_MAC(PEER_MAC),
        .P_ETHERTYPE(16'h88B6), .P_TX_ARB_EN(1)
    ) u_mode (
        .I_clk_tx(clk_tx), .I_rst_tx_n(rst_n),
        .O_gmii_txd(mode_txd), .O_gmii_tx_en(mode_tx_en),
        .O_gmii_tx_er(mode_tx_er), .I_tx_grant(mode_grant),
        .O_tx_request(mode_req), .O_tx_done_tx(mode_done_tx),
        .I_clk_rx(clk_tx), .I_rst_rx_n(rst_n), .I_gmii_rxd(8'd0),
        .I_gmii_rx_dv(1'b0), .I_gmii_rx_er(1'b0),
        .I_clk_app(clk_75), .I_rst_app_n(rst_n),
        .O_tx_ready(mode_ready), .O_tx_wbuf(mode_wbuf),
        .I_tx_we(mode_we), .I_tx_addr(mode_addr),
        .I_tx_wdata(mode_wdata), .I_tx_commit(mode_commit),
        .O_rx_valid(), .O_rx_rbuf(), .O_rx_len(), .I_rx_addr(11'd0),
        .O_rx_rdata(), .I_rx_release(1'b0),
        .O_tx_frame_count(), .O_rx_frame_count(), .O_rx_bad_count(),
        .O_rx_busy_drop_count(), .O_rx_last_line(),
        .O_bad_frame_count(), .O_busy_drop_count()
    );

    ctl_gmii_tx_scheduler #(.P_TX_SCHEDULE(0)) u_arb (
        .I_clk_tx(clk_tx), .I_rst_tx_n(rst_n),
        .I_tx0_request(ocr_req), .I_tx1_request(mode_req),
        .I_tx0_done(ocr_done_tx), .I_tx1_done(mode_done_tx),
        .O_tx0_grant(ocr_grant), .O_tx1_grant(mode_grant),
        .I_tx0_txd(ocr_txd), .I_tx0_tx_en(ocr_tx_en),
        .I_tx0_tx_er(ocr_tx_er), .I_tx1_txd(mode_txd),
        .I_tx1_tx_en(mode_tx_en), .I_tx1_tx_er(mode_tx_er),
        .O_txd(gmii_txd), .O_tx_en(gmii_tx_en), .O_tx_er(gmii_tx_er),
        .O_owner_valid(owner_valid), .O_owner(owner)
    );

    integer i;
    integer frame_count;
    integer frame_len;
    integer errors;
    reg frame_active;
    time frame_start [0:1];
    reg [7:0] frame_bytes [0:2047];
    // Use the production application sizes: the OCR request is 912 bytes and
    // one stream frame carries its 16-byte line header plus 1280 payload bytes.
    reg [7:0] ocr_payload [0:911];
    reg [7:0] mode_payload [0:1279];

    // GMII data is stable for the whole half-cycle after the source edge.
    // Parse only at the end of that half-cycle, after all endpoint NBA updates.
    always @(negedge clk_tx) begin
        if (rst_n) begin
            if (gmii_tx_en) begin
                if (!frame_active) begin
                    frame_active = 1'b1;
                    frame_len = 0;
                    if (frame_count < 2)
                        frame_start[frame_count] = $time;
                end
                if (frame_len < 2048) begin
                    frame_bytes[frame_len] = gmii_txd;
                    frame_len = frame_len + 1;
                end
            end
            else if (frame_active) begin
                if (frame_len < 22 || frame_bytes[0] !== 8'h55 ||
                    frame_bytes[7] !== 8'hD5) begin
                    $display("FAIL coexist malformed frame len=%0d", frame_len);
                    errors = errors + 1;
                end
                else if (frame_count == 0) begin
                    if (frame_len != 938 ||
                        {frame_bytes[20], frame_bytes[21]} !== 16'h88B5 ||
                        frame_bytes[22] !== ocr_payload[0]) begin
                        $display("FAIL coexist frame0 len/type/payload=%0d/%02h%02h/%02h",
                                 frame_len,
                                 frame_bytes[20], frame_bytes[21], frame_bytes[22]);
                        errors = errors + 1;
                    end
                end
                else if (frame_count == 1) begin
                    if (frame_len != 1322 ||
                        {frame_bytes[20], frame_bytes[21]} !== 16'h88B6 ||
                        frame_bytes[22] !== 8'hA5 || frame_bytes[23] !== 8'h01 ||
                        {frame_bytes[29], frame_bytes[28]} !== 16'd1280 ||
                        frame_bytes[38] !== mode_payload[0]) begin
                        $display("FAIL coexist frame1 len/type/header/payload=%0d/%02h%02h/%02h%02h/%02h%02h/%02h",
                                 frame_len,
                                 frame_bytes[20], frame_bytes[21], frame_bytes[22],
                                 frame_bytes[23], frame_bytes[28], frame_bytes[29],
                                 frame_bytes[38]);
                        errors = errors + 1;
                    end
                end
                if (frame_count < 2)
                    $display("TB_NOTE coexist frame%0d bytes=%0d wire_ns=%0t",
                             frame_count, frame_len, $time - frame_start[frame_count]);
                frame_count = frame_count + 1;
                frame_active = 1'b0;
            end
        end
    end

    task write_ocr_mailbox;
        begin
            for (i = 0; i < 912; i = i + 1) begin
                @(negedge clk_25);
                ocr_we = 1'b1;
                ocr_addr = i[9:0];
                ocr_wdata = ocr_payload[i];
            end
            @(negedge clk_25);
            ocr_we = 1'b0;
        end
    endtask

    task write_mode_mailbox;
        begin
            for (i = 0; i < 16; i = i + 1) begin
                @(negedge clk_75);
                mode_we = 1'b1;
                mode_addr = i[10:0];
                case (i)
                    0: mode_wdata = 8'hA5;
                    1: mode_wdata = 8'h01;
                    2: mode_wdata = 8'h02; // centre line number
                    3: mode_wdata = 8'h00;
                    4: mode_wdata = 8'h00;
                    5: mode_wdata = 8'h00;
                    6: mode_wdata = 8'h00; // 1280-byte payload, little endian
                    7: mode_wdata = 8'h05;
                    8: mode_wdata = 8'h78; // frame sequence 0x12345678
                    9: mode_wdata = 8'h56;
                    10: mode_wdata = 8'h34;
                    11: mode_wdata = 8'h12;
                    default: mode_wdata = 8'h00;
                endcase
            end
            for (i = 0; i < 1280; i = i + 1) begin
                @(negedge clk_75);
                mode_we = 1'b1;
                mode_addr = 11'd16 + i;
                mode_wdata = mode_payload[i];
            end
            @(negedge clk_75);
            mode_we = 1'b0;
        end
    endtask

    initial begin
        ocr_we = 1'b0; ocr_commit = 1'b0; ocr_addr = 10'd0;
        ocr_wdata = 8'd0; ocr_len = 11'd912;
        mode_we = 1'b0; mode_commit = 1'b0; mode_addr = 11'd0;
        mode_wdata = 8'd0;
        frame_count = 0; frame_len = 0; frame_active = 1'b0; errors = 0;
        for (i = 0; i < 912; i = i + 1) ocr_payload[i] = 8'hC0 + i;
        for (i = 0; i < 1280; i = i + 1) mode_payload[i] = 8'hA0 + i;

        repeat (8) @(posedge clk_tx);
        rst_n = 1'b1;
        // Prefill both mailboxes, then commit OCR first.  Both requests are
        // pending before the scheduler can observe the synchronized toggles.
        write_ocr_mailbox;
        write_mode_mailbox;
        @(negedge clk_25); ocr_commit = 1'b1;
        @(negedge clk_25); ocr_commit = 1'b0;
        @(negedge clk_75); mode_commit = 1'b1;
        @(negedge clk_75); mode_commit = 1'b0;

        repeat (4000) @(negedge clk_tx);
        if (frame_count != 2) begin
            $display("FAIL coexist frame_count=%0d (expected 2)", frame_count);
            errors = errors + 1;
        end
        if (errors != 0)
            $fatal(1, "TB_RESULT: FAIL dual GMII coexist errors=%0d", errors);
        $display("TB_RESULT: PASS dual GMII coexist frames=%0d OCR-first", frame_count);
        $finish;
    end
endmodule
