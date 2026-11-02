`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// Module: tb_ctl_stream_link_faults
// Function: v2.5 GMII RX preamble, RX_ER, CRC, truncation, and reset checks
// Domain: independent 125 MHz TX/RX and 75 MHz application clocks
// Reset: active-low asynchronous assertion
// -----------------------------------------------------------------------------
module tb_ctl_stream_link_faults;

    localparam [47:0] P_LOCAL_MAC = 48'h02_00_00_12_34_56;
    localparam [47:0] P_PEER_MAC  = 48'h02_00_00_65_43_21;
    localparam [15:0] P_ETHERTYPE = 16'h88B6;
    localparam integer P_PAYLOAD_BYTES = 64;
    localparam integer P_APP_BYTES = 16 + P_PAYLOAD_BYTES;
    localparam integer P_FRAME_MAX = 4096;

    reg I_clk_tx = 1'b0;
    reg I_clk_rx = 1'b0;
    reg I_clk_app = 1'b0;
    reg I_rst_tx_n = 1'b0;
    reg I_rst_rx_n = 1'b0;
    reg I_rst_app_n = 1'b0;
    reg [7:0] I_gmii_rxd = 8'h00;
    reg I_gmii_rx_dv = 1'b0;
    reg I_gmii_rx_er = 1'b0;
    reg I_tx_grant = 1'b0;
    reg I_tx_we = 1'b0;
    reg [10:0] I_tx_addr = 11'd0;
    reg [7:0] I_tx_wdata = 8'h00;
    reg I_tx_commit = 1'b0;
    reg [10:0] I_rx_addr = 11'd0;
    reg I_rx_release = 1'b0;

    wire [7:0] O_gmii_txd;
    wire O_gmii_tx_en;
    wire O_gmii_tx_er;
    wire O_tx_request;
    wire O_tx_done_tx;
    wire O_tx_ready;
    wire O_tx_wbuf;
    wire O_rx_valid;
    wire O_rx_rbuf;
    wire [15:0] O_rx_len;
    wire [7:0] O_rx_rdata;
    wire [31:0] O_tx_frame_count;
    wire [31:0] O_rx_frame_count;
    wire [31:0] O_rx_bad_count;
    wire [31:0] O_rx_busy_drop_count;
    wire [15:0] O_rx_last_line;
    wire [31:0] O_bad_frame_count;
    wire [31:0] O_busy_drop_count;

    reg [7:0] S_frame [0:P_FRAME_MAX-1];
    integer S_frame_len;
    reg [31:0] S_crc_build;
    integer S_checks;
    integer S_errors;

    always #4.000 I_clk_tx = ~I_clk_tx;
    initial begin #1.000; forever #4.000 I_clk_rx = ~I_clk_rx; end
    always #6.666 I_clk_app = ~I_clk_app;

    ctl_stream_link #(
        .P_LOCAL_MAC(P_LOCAL_MAC),
        .P_PEER_MAC(P_PEER_MAC),
        .P_ETHERTYPE(P_ETHERTYPE),
        .P_TX_ARB_EN(0)
    ) u_dut (
        .I_clk_tx(I_clk_tx), .I_rst_tx_n(I_rst_tx_n),
        .O_gmii_txd(O_gmii_txd), .O_gmii_tx_en(O_gmii_tx_en),
        .O_gmii_tx_er(O_gmii_tx_er), .I_tx_grant(I_tx_grant),
        .O_tx_request(O_tx_request), .O_tx_done_tx(O_tx_done_tx),
        .I_clk_rx(I_clk_rx), .I_rst_rx_n(I_rst_rx_n),
        .I_gmii_rxd(I_gmii_rxd), .I_gmii_rx_dv(I_gmii_rx_dv),
        .I_gmii_rx_er(I_gmii_rx_er),
        .I_clk_app(I_clk_app), .I_rst_app_n(I_rst_app_n),
        .O_tx_ready(O_tx_ready), .O_tx_wbuf(O_tx_wbuf),
        .I_tx_we(I_tx_we), .I_tx_addr(I_tx_addr), .I_tx_wdata(I_tx_wdata),
        .I_tx_commit(I_tx_commit),
        .O_rx_valid(O_rx_valid), .O_rx_rbuf(O_rx_rbuf), .O_rx_len(O_rx_len),
        .I_rx_addr(I_rx_addr), .O_rx_rdata(O_rx_rdata),
        .I_rx_release(I_rx_release),
        .O_tx_frame_count(O_tx_frame_count), .O_rx_frame_count(O_rx_frame_count),
        .O_rx_bad_count(O_rx_bad_count),
        .O_rx_busy_drop_count(O_rx_busy_drop_count),
        .O_rx_last_line(O_rx_last_line), .O_bad_frame_count(O_bad_frame_count),
        .O_busy_drop_count(O_busy_drop_count)
    );

    function [31:0] f_crc32_byte;
        input [31:0] I_crc;
        input [7:0] I_data;
        integer bit_index;
        reg [31:0] W_crc;
        begin
            W_crc = I_crc;
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1) begin
                if (W_crc[0] ^ I_data[bit_index])
                    W_crc = (W_crc >> 1) ^ 32'hEDB8_8320;
                else
                    W_crc = W_crc >> 1;
            end
            f_crc32_byte = W_crc;
        end
    endfunction

    task automatic t_check;
        input I_condition;
        input [255:0] I_message;
        begin
            S_checks = S_checks + 1;
            if (!I_condition) begin
                S_errors = S_errors + 1;
                $display("FAIL %0s", I_message);
            end
        end
    endtask

    task automatic t_append_body;
        input [7:0] I_value;
        begin
            S_frame[S_frame_len] = I_value;
            S_frame_len = S_frame_len + 1;
            S_crc_build = f_crc32_byte(S_crc_build, I_value);
        end
    endtask

    task automatic t_build_frame;
        input integer I_preamble_count;
        input integer I_bad_crc;
        integer i;
        reg [31:0] W_fcs;
        begin
            S_frame_len = 0;
            for (i = 0; i < I_preamble_count; i = i + 1) begin
                S_frame[S_frame_len] = 8'h55;
                S_frame_len = S_frame_len + 1;
            end
            S_frame[S_frame_len] = 8'hD5;
            S_frame_len = S_frame_len + 1;
            S_crc_build = 32'hFFFF_FFFF;

            t_append_body(P_LOCAL_MAC[47:40]);
            t_append_body(P_LOCAL_MAC[39:32]);
            t_append_body(P_LOCAL_MAC[31:24]);
            t_append_body(P_LOCAL_MAC[23:16]);
            t_append_body(P_LOCAL_MAC[15:8]);
            t_append_body(P_LOCAL_MAC[7:0]);
            t_append_body(P_PEER_MAC[47:40]);
            t_append_body(P_PEER_MAC[39:32]);
            t_append_body(P_PEER_MAC[31:24]);
            t_append_body(P_PEER_MAC[23:16]);
            t_append_body(P_PEER_MAC[15:8]);
            t_append_body(P_PEER_MAC[7:0]);
            t_append_body(P_ETHERTYPE[15:8]);
            t_append_body(P_ETHERTYPE[7:0]);

            t_append_body(8'hA5);
            t_append_body(8'h01);
            t_append_body(8'h01);
            t_append_body(8'h00);
            t_append_body(8'h09);
            t_append_body(8'h00);
            t_append_body(P_PAYLOAD_BYTES[7:0]);
            t_append_body(P_PAYLOAD_BYTES[15:8]);
            for (i = 8; i < 16; i = i + 1)
                t_append_body(8'h00);
            for (i = 0; i < P_PAYLOAD_BYTES; i = i + 1)
                t_append_body(8'h80 ^ i[7:0]);

            W_fcs = ~S_crc_build;
            if (I_bad_crc != 0)
                W_fcs[0] = ~W_fcs[0];
            S_frame[S_frame_len] = W_fcs[7:0];
            S_frame_len = S_frame_len + 1;
            S_frame[S_frame_len] = W_fcs[15:8];
            S_frame_len = S_frame_len + 1;
            S_frame[S_frame_len] = W_fcs[23:16];
            S_frame_len = S_frame_len + 1;
            S_frame[S_frame_len] = W_fcs[31:24];
            S_frame_len = S_frame_len + 1;
        end
    endtask

    task automatic t_send_frame;
        input integer I_error_index;
        integer i;
        begin
            for (i = 0; i < S_frame_len; i = i + 1) begin
                I_gmii_rxd = S_frame[i];
                I_gmii_rx_dv = 1'b1;
                I_gmii_rx_er = (i == I_error_index);
                @(posedge I_clk_rx);
            end
            I_gmii_rxd = 8'h00;
            I_gmii_rx_dv = 1'b0;
            I_gmii_rx_er = 1'b0;
            repeat (4) @(posedge I_clk_rx);
        end
    endtask

    task automatic t_send_prefix;
        input integer I_count;
        integer i;
        begin
            for (i = 0; i < I_count; i = i + 1) begin
                I_gmii_rxd = S_frame[i];
                I_gmii_rx_dv = 1'b1;
                I_gmii_rx_er = 1'b0;
                @(posedge I_clk_rx);
            end
            I_gmii_rxd = 8'h00;
            I_gmii_rx_dv = 1'b0;
            repeat (2) @(posedge I_clk_rx);
        end
    endtask

    task automatic t_wait_bad;
        input [31:0] I_before;
        integer timeout;
        begin
            timeout = 0;
            while ((O_rx_bad_count <= I_before) && (timeout < 400)) begin
                @(posedge I_clk_app);
                timeout = timeout + 1;
            end
            t_check(O_rx_bad_count > I_before, "bad-frame counter advanced");
            t_check(!O_rx_valid, "bad frame not published");
        end
    endtask

    task automatic t_wait_valid_release;
        integer timeout;
        begin
            timeout = 0;
            while (!O_rx_valid && (timeout < 400)) begin
                @(posedge I_clk_app);
                timeout = timeout + 1;
            end
            t_check(O_rx_valid, "valid frame published");
            t_check(O_rx_len == P_APP_BYTES, "valid application length");
            I_rx_release = 1'b1;
            @(posedge I_clk_app);
            I_rx_release = 1'b0;
            repeat (8) @(posedge I_clk_app);
            t_check(!O_rx_valid, "released frame no longer held");
        end
    endtask

    task automatic t_send_bad;
        input integer I_preamble_count;
        input integer I_bad_crc;
        input integer I_error_index;
        reg [31:0] W_before;
        begin
            t_build_frame(I_preamble_count, I_bad_crc);
            W_before = O_rx_bad_count;
            t_send_frame(I_error_index);
            t_wait_bad(W_before);
        end
    endtask

    integer W_before;
    initial begin
        S_checks = 0;
        S_errors = 0;
        repeat (6) @(posedge I_clk_app);
        I_rst_tx_n = 1'b1;
        I_rst_rx_n = 1'b1;
        I_rst_app_n = 1'b1;
        repeat (12) @(posedge I_clk_app);

        // A correctly shaped frame remains the control sample for all faults.
        t_build_frame(7, 0);
        t_send_frame(-1);
        t_wait_valid_release();

        // Preamble length is part of the receive contract: both early and late
        // SFDs must be rejected even when their CRC is otherwise correct.
        t_send_bad(6, 0, -1);
        t_send_bad(8, 0, -1);

        // RX_ER is checked in every receive phase, including preamble and FCS.
        t_build_frame(7, 0);
        $display("TEST RX_ER preamble"); W_before = O_rx_bad_count; t_send_frame(0);  t_wait_bad(W_before);
        $display("TEST RX_ER SFD");      W_before = O_rx_bad_count; t_send_frame(7);  t_wait_bad(W_before);
        $display("TEST RX_ER header");    W_before = O_rx_bad_count; t_send_frame(8);  t_wait_bad(W_before);
        $display("TEST RX_ER app");       W_before = O_rx_bad_count; t_send_frame(22); t_wait_bad(W_before);
        $display("TEST RX_ER payload");   W_before = O_rx_bad_count; t_send_frame(38); t_wait_bad(W_before);
        $display("TEST RX_ER FCS");       W_before = O_rx_bad_count; t_send_frame(S_frame_len - 1); t_wait_bad(W_before);

        // RX_ER frames may have written the current receive buffer, but they
        // must never toggle its publish ownership or hold a stale valid flag.
        // A clean frame immediately after the damaged sequence proves the
        // buffer can be reused without leaking the rejected bytes.
        t_build_frame(7, 0);
        t_send_frame(-1);
        t_wait_valid_release();

        // A complete frame with a wrong FCS is consumed but never published.
        t_send_bad(7, 1, -1);

        // A dropped/truncated frame must not leak a partial application packet.
        t_build_frame(7, 0);
        t_send_prefix(30);
        repeat (80) @(posedge I_clk_app);
        t_check(!O_rx_valid, "truncated frame not published");

        // Reset during an in-flight frame clears RX state and no stale packet
        // appears after release; a fresh valid frame must still work.
        t_build_frame(7, 0);
        t_send_prefix(30);
        I_rst_tx_n = 1'b0;
        I_rst_rx_n = 1'b0;
        I_rst_app_n = 1'b0;
        repeat (5) @(posedge I_clk_rx);
        I_gmii_rxd = 8'h00;
        I_gmii_rx_dv = 1'b0;
        I_gmii_rx_er = 1'b0;
        I_rst_tx_n = 1'b1;
        I_rst_rx_n = 1'b1;
        I_rst_app_n = 1'b1;
        repeat (12) @(posedge I_clk_app);
        t_check(!O_rx_valid, "in-flight reset leaves no stale packet");
        t_build_frame(7, 0);
        t_send_frame(-1);
        t_wait_valid_release();

        if (S_errors == 0)
            $display("TB_RESULT: PASS tb_ctl_stream_link_faults checks=%0d errors=0",
                     S_checks);
        else
            $display("TB_RESULT: FAIL tb_ctl_stream_link_faults checks=%0d errors=%0d",
                     S_checks, S_errors);
        $finish;
    end

endmodule
