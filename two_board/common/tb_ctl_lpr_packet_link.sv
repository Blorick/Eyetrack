`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// Module: tb_ctl_lpr_packet_link
// Function: fixed-vector raw Ethernet TX/RX, CRC, busy-drop, and CDC checks
// Domain: simulation testbench for application, GMII TX, and GMII RX clocks
// Clock: independent 100MHz application and phase-shifted 125MHz GMII clocks
// Reset: one common board reset drives all DUT domains; release is per-domain synchronous
// Parameters: golden-vector lengths are fixed by protocol-contract-v1.md
// Verification: byte-for-byte request/response GMII vectors; no DUT CRC oracle
// -----------------------------------------------------------------------------
module tb_ctl_lpr_packet_link;

    localparam integer P_REQUEST_APP_LENGTH = 912;
    localparam integer P_REQUEST_GMII_LENGTH = 938;
    localparam integer P_RESPONSE_APP_LENGTH = 32;
    localparam integer P_RESPONSE_GMII_LENGTH = 72;
    localparam integer P_MAX_VECTOR_LENGTH = 2048;

    reg S_clk_app;
    reg S_clk_tx;
    reg S_clk_rx;
    reg S_rst_app_n;
    reg S_rst_tx_n;
    reg S_rst_rx_n;
    reg S_slave_rst_app_n;
    reg S_slave_rst_tx_n;
    reg S_slave_rst_rx_n;

    reg S_master_tx_write_en;
    reg [9:0] S_master_tx_write_addr;
    reg [7:0] S_master_tx_write_data;
    reg S_master_tx_commit;
    reg [10:0] S_master_tx_length;
    wire W_master_tx_busy;
    wire W_master_tx_done;
    wire W_master_tx_request;
    wire W_master_tx_done_tx;
    wire W_master_rx_valid;
    wire [10:0] W_master_rx_length;
    reg [9:0] S_master_rx_read_addr;
    wire [7:0] W_master_rx_read_data;
    reg S_master_rx_release;
    wire [7:0] W_master_gmii_txd;
    wire W_master_gmii_tx_en;
    wire W_master_gmii_tx_er;
    reg [7:0] S_master_gmii_rxd;
    reg S_master_gmii_rx_dv;
    reg S_master_gmii_rx_er;
    wire [31:0] W_master_tx_busy_drops;
    wire [31:0] W_master_tx_invalid_drops;
    wire [31:0] W_master_rx_bad_drops;
    wire [31:0] W_master_rx_busy_drops;
    wire [31:0] W_master_rx_oversize_drops;
    wire [31:0] W_master_bad_frame_count;
    wire [31:0] W_master_busy_drop_count;

    reg S_slave_tx_write_en;
    reg [9:0] S_slave_tx_write_addr;
    reg [7:0] S_slave_tx_write_data;
    reg S_slave_tx_commit;
    reg [10:0] S_slave_tx_length;
    wire W_slave_tx_busy;
    wire W_slave_tx_done;
    wire W_slave_tx_request;
    wire W_slave_tx_done_tx;
    wire W_slave_rx_valid;
    wire [10:0] W_slave_rx_length;
    reg [9:0] S_slave_rx_read_addr;
    wire [7:0] W_slave_rx_read_data;
    reg S_slave_rx_release;
    wire [7:0] W_slave_gmii_txd;
    wire W_slave_gmii_tx_en;
    wire W_slave_gmii_tx_er;
    reg [7:0] S_slave_gmii_rxd;
    reg S_slave_gmii_rx_dv;
    reg S_slave_gmii_rx_er;
    wire [31:0] W_slave_tx_busy_drops;
    wire [31:0] W_slave_tx_invalid_drops;
    wire [31:0] W_slave_rx_bad_drops;
    wire [31:0] W_slave_rx_busy_drops;
    wire [31:0] W_slave_rx_oversize_drops;
    wire [31:0] W_slave_bad_frame_count;
    wire [31:0] W_slave_busy_drop_count;

    reg [7:0] S_request_app [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_response_app [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_request_golden [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_response_golden [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_response_to_slave [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_request_capture [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_response_capture [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_request_bad_crc [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_request_wrong_destination [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_request_wrong_type [0:P_MAX_VECTOR_LENGTH - 1];
    reg [7:0] S_response_bad_padding [0:P_MAX_VECTOR_LENGTH - 1];
    integer S_request_capture_count;
    integer S_response_capture_count;
    integer S_failures;
    integer S_index;
    integer S_partial_capture_count;
    integer S_wait_count;
    reg S_wait_ok;
    reg [7:0] S_crc_known_data [0:8];

    localparam integer P_VALID_TIMEOUT_CYCLES = 2000;

    ctl_lpr_packet_link #(
        .P_LOCAL_MAC(48'h02_00_00_12_34_56),
        .P_PEER_MAC(48'h02_00_00_65_43_21),
        .P_ETHERTYPE(16'h88B5)
    ) u_master (
        .I_clk_app(S_clk_app),
        .I_rst_app_n(S_rst_app_n),
        .I_clk_tx(S_clk_tx),
        .I_rst_tx_n(S_rst_tx_n),
        .I_clk_rx(S_clk_rx),
        .I_rst_rx_n(S_rst_rx_n),
        .I_tx_write_en(S_master_tx_write_en),
        .I_tx_write_addr(S_master_tx_write_addr),
        .I_tx_write_data(S_master_tx_write_data),
        .I_tx_commit(S_master_tx_commit),
        .I_tx_length(S_master_tx_length),
        .O_tx_busy(W_master_tx_busy),
        .O_tx_done(W_master_tx_done),
        .I_tx_grant(1'b1),
        .O_tx_request(W_master_tx_request),
        .O_tx_done_tx(W_master_tx_done_tx),
        .O_rx_valid(W_master_rx_valid),
        .O_rx_length(W_master_rx_length),
        .I_rx_read_addr(S_master_rx_read_addr),
        .O_rx_read_data(W_master_rx_read_data),
        .I_rx_release(S_master_rx_release),
        .O_gmii_txd(W_master_gmii_txd),
        .O_gmii_tx_en(W_master_gmii_tx_en),
        .O_gmii_tx_er(W_master_gmii_tx_er),
        .I_gmii_rxd(S_master_gmii_rxd),
        .I_gmii_rx_dv(S_master_gmii_rx_dv),
        .I_gmii_rx_er(S_master_gmii_rx_er),
        .O_tx_busy_drop_count(W_master_tx_busy_drops),
        .O_tx_invalid_drop_count(W_master_tx_invalid_drops),
        .O_rx_bad_frame_count(W_master_rx_bad_drops),
        .O_rx_busy_drop_count(W_master_rx_busy_drops),
        .O_rx_oversize_drop_count(W_master_rx_oversize_drops),
        .O_bad_frame_count(W_master_bad_frame_count),
        .O_busy_drop_count(W_master_busy_drop_count)
    );

    ctl_lpr_packet_link #(
        .P_LOCAL_MAC(48'h02_00_00_65_43_21),
        .P_PEER_MAC(48'h02_00_00_12_34_56),
        .P_ETHERTYPE(16'h88B5)
    ) u_slave (
        .I_clk_app(S_clk_app),
        .I_rst_app_n(S_slave_rst_app_n),
        .I_clk_tx(S_clk_tx),
        .I_rst_tx_n(S_slave_rst_tx_n),
        .I_clk_rx(S_clk_rx),
        .I_rst_rx_n(S_slave_rst_rx_n),
        .I_tx_write_en(S_slave_tx_write_en),
        .I_tx_write_addr(S_slave_tx_write_addr),
        .I_tx_write_data(S_slave_tx_write_data),
        .I_tx_commit(S_slave_tx_commit),
        .I_tx_length(S_slave_tx_length),
        .O_tx_busy(W_slave_tx_busy),
        .O_tx_done(W_slave_tx_done),
        .I_tx_grant(1'b1),
        .O_tx_request(W_slave_tx_request),
        .O_tx_done_tx(W_slave_tx_done_tx),
        .O_rx_valid(W_slave_rx_valid),
        .O_rx_length(W_slave_rx_length),
        .I_rx_read_addr(S_slave_rx_read_addr),
        .O_rx_read_data(W_slave_rx_read_data),
        .I_rx_release(S_slave_rx_release),
        .O_gmii_txd(W_slave_gmii_txd),
        .O_gmii_tx_en(W_slave_gmii_tx_en),
        .O_gmii_tx_er(W_slave_gmii_tx_er),
        .I_gmii_rxd(S_slave_gmii_rxd),
        .I_gmii_rx_dv(S_slave_gmii_rx_dv),
        .I_gmii_rx_er(S_slave_gmii_rx_er),
        .O_tx_busy_drop_count(W_slave_tx_busy_drops),
        .O_tx_invalid_drop_count(W_slave_tx_invalid_drops),
        .O_rx_bad_frame_count(W_slave_rx_bad_drops),
        .O_rx_busy_drop_count(W_slave_rx_busy_drops),
        .O_rx_oversize_drop_count(W_slave_rx_oversize_drops),
        .O_bad_frame_count(W_slave_bad_frame_count),
        .O_busy_drop_count(W_slave_busy_drop_count)
    );

    initial begin
        S_clk_app = 1'b0;
        forever #5 S_clk_app = ~S_clk_app;
    end

    initial begin
        S_clk_tx = 1'b0;
        forever #4 S_clk_tx = ~S_clk_tx;
    end

    initial begin
        #1;
        S_clk_rx = 1'b0;
        forever #4 S_clk_rx = ~S_clk_rx;
    end

    always @(posedge S_clk_tx) begin
        if (W_master_gmii_tx_en) begin
            S_request_capture[S_request_capture_count] <= W_master_gmii_txd;
            S_request_capture_count <= S_request_capture_count + 1;
        end
        if (W_slave_gmii_tx_en) begin
            S_response_capture[S_response_capture_count] <= W_slave_gmii_txd;
            S_response_capture_count <= S_response_capture_count + 1;
        end
    end

    task automatic t_check_equal;
        input [7:0] I_actual;
        input [7:0] I_expected;
        input integer I_number;
        input [255:0] I_label;
        begin
            if (I_actual !== I_expected) begin
                $display("FAIL %0s index=%0d actual=%02x expected=%02x", I_label, I_number, I_actual, I_expected);
                S_failures = S_failures + 1;
            end
        end
    endtask

    task automatic t_write_master_request;
        begin
            for (S_index = 0; S_index < P_REQUEST_APP_LENGTH; S_index = S_index + 1) begin
                @(negedge S_clk_app);
                S_master_tx_write_en = 1'b1;
                S_master_tx_write_addr = S_index[9:0];
                S_master_tx_write_data = S_request_app[S_index];
                @(posedge S_clk_app);
            end
            @(negedge S_clk_app);
            S_master_tx_write_en = 1'b0;
            S_master_tx_commit = 1'b1;
            S_master_tx_length = P_REQUEST_APP_LENGTH[10:0];
            @(posedge S_clk_app);
            @(negedge S_clk_app);
            S_master_tx_commit = 1'b0;
        end
    endtask

    // Hold the already synchronized TX length at its old value until the
    // request has reached the TX domain.  The old DUT consumes that stale
    // length on the request edge; the repaired DUT waits one TX settling
    // cycle and then uses the committed 912-byte length.  The forced LHS is
    // a fixed packed scalar so this directed CDC injection has no variable
    // array select.
    task automatic t_write_master_request_with_length_lag;
        begin
            force u_master.S_tx_length_s2_tx = 11'd0;
            for (S_index = 0; S_index < P_REQUEST_APP_LENGTH; S_index = S_index + 1) begin
                @(negedge S_clk_app);
                S_master_tx_write_en = 1'b1;
                S_master_tx_write_addr = S_index[9:0];
                S_master_tx_write_data = S_request_app[S_index];
                @(posedge S_clk_app);
            end
            @(negedge S_clk_app);
            S_master_tx_write_en = 1'b0;
            S_master_tx_commit = 1'b1;
            S_master_tx_length = P_REQUEST_APP_LENGTH[10:0];
            @(posedge S_clk_app);
            @(negedge S_clk_app);
            S_master_tx_commit = 1'b0;

            S_wait_count = 0;
            while ((u_master.S_tx_request_s2_tx !== 1'b1) &&
                   (S_wait_count < P_VALID_TIMEOUT_CYCLES)) begin
                @(posedge S_clk_tx);
                #1;
                S_wait_count = S_wait_count + 1;
            end
            if (u_master.S_tx_request_s2_tx !== 1'b1) begin
                $display("FAIL TX length-lag injection request timeout");
                S_failures = S_failures + 1;
            end
            @(negedge S_clk_tx);
            release u_master.S_tx_length_s2_tx;
        end
    endtask

    task automatic t_write_slave_response;
        begin
            for (S_index = 0; S_index < P_RESPONSE_APP_LENGTH; S_index = S_index + 1) begin
                @(negedge S_clk_app);
                S_slave_tx_write_en = 1'b1;
                S_slave_tx_write_addr = S_index[9:0];
                S_slave_tx_write_data = S_response_app[S_index];
                @(posedge S_clk_app);
            end
            @(negedge S_clk_app);
            S_slave_tx_write_en = 1'b0;
            S_slave_tx_commit = 1'b1;
            S_slave_tx_length = P_RESPONSE_APP_LENGTH[10:0];
            @(posedge S_clk_app);
            @(negedge S_clk_app);
            S_slave_tx_commit = 1'b0;
        end
    endtask

    task automatic t_compare_request_tx;
        begin
            if (S_request_capture_count != P_REQUEST_GMII_LENGTH) begin
                $display("FAIL request GMII count=%0d expected=%0d", S_request_capture_count, P_REQUEST_GMII_LENGTH);
                S_failures = S_failures + 1;
            end
            for (S_index = 0; S_index < P_REQUEST_GMII_LENGTH; S_index = S_index + 1) begin
                t_check_equal(S_request_capture[S_index], S_request_golden[S_index], S_index, "request-tx");
            end
        end
    endtask

    task automatic t_compare_response_tx;
        begin
            if (S_response_capture_count != P_RESPONSE_GMII_LENGTH) begin
                $display("FAIL response GMII count=%0d expected=%0d", S_response_capture_count, P_RESPONSE_GMII_LENGTH);
                S_failures = S_failures + 1;
            end
            for (S_index = 0; S_index < P_RESPONSE_GMII_LENGTH; S_index = S_index + 1) begin
                t_check_equal(S_response_capture[S_index], S_response_golden[S_index], S_index, "response-tx");
            end
        end
    endtask

    function automatic [31:0] f_tb_crc32_byte;
        input [31:0] I_crc;
        input [7:0] I_data;
        reg [31:0] W_crc;
        integer I_crc_bit;
        begin
            W_crc = I_crc;
            for (I_crc_bit = 0; I_crc_bit < 8; I_crc_bit = I_crc_bit + 1) begin
                if (W_crc[0] ^ I_data[I_crc_bit]) begin
                    W_crc = (W_crc >> 1) ^ 32'hEDB8_8320;
                end else begin
                    W_crc = W_crc >> 1;
                end
            end
            f_tb_crc32_byte = W_crc;
        end
    endfunction

    task automatic t_check_known_crc;
        reg [31:0] W_crc;
        begin
            W_crc = 32'hFFFF_FFFF;
            for (S_index = 0; S_index < 9; S_index = S_index + 1) begin
                W_crc = f_tb_crc32_byte(W_crc, S_crc_known_data[S_index]);
            end
            if ((~W_crc) !== 32'hCBF4_3926) begin
                $display("FAIL CRC32 known vector actual=%08x expected=CBF43926", ~W_crc);
                S_failures = S_failures + 1;
            end
        end
    endtask

    task automatic t_build_response_to_slave;
        reg [31:0] W_crc;
        begin
            // response-gmii.hex is master-directed.  The slave-directed copy
            // swaps only DA/SA and independently rebuilds its frame FCS.
            for (S_index = 0; S_index < P_RESPONSE_GMII_LENGTH; S_index = S_index + 1) begin
                S_response_to_slave[S_index] = S_response_golden[S_index];
            end
            for (S_index = 0; S_index < 6; S_index = S_index + 1) begin
                S_response_to_slave[8 + S_index] = S_response_golden[14 + S_index];
                S_response_to_slave[14 + S_index] = S_response_golden[8 + S_index];
            end
            W_crc = 32'hFFFF_FFFF;
            for (S_index = 8; S_index < 68; S_index = S_index + 1) begin
                W_crc = f_tb_crc32_byte(W_crc, S_response_to_slave[S_index]);
            end
            W_crc = ~W_crc;
            S_response_to_slave[68] = W_crc[7:0];
            S_response_to_slave[69] = W_crc[15:8];
            S_response_to_slave[70] = W_crc[23:16];
            S_response_to_slave[71] = W_crc[31:24];
            if ({S_response_to_slave[71], S_response_to_slave[70],
                 S_response_to_slave[69], S_response_to_slave[68]} !== 32'h674D_F417) begin
                $display("FAIL rebuilt slave response FCS=%08x expected=674DF417",
                    {S_response_to_slave[71], S_response_to_slave[70],
                     S_response_to_slave[69], S_response_to_slave[68]});
                S_failures = S_failures + 1;
            end
        end
    endtask

    task automatic t_wait_slave_valid;
        input [10:0] I_expected_length;
        begin
            S_wait_count = 0;
            S_wait_ok = 0;
            while ((W_slave_rx_valid !== 1'b1) &&
                   (S_wait_count < P_VALID_TIMEOUT_CYCLES)) begin
                @(posedge S_clk_app);
                #1;
                S_wait_count = S_wait_count + 1;
            end
            if (W_slave_rx_valid !== 1'b1) begin
                $display("FAIL slave RX valid timeout cycles=%0d length=%0d",
                    S_wait_count, W_slave_rx_length);
                S_failures = S_failures + 1;
            end else if (W_slave_rx_length !== I_expected_length) begin
                $display("FAIL slave RX first-valid length=%0d expected=%0d",
                    W_slave_rx_length, I_expected_length);
                S_failures = S_failures + 1;
            end else begin
                S_wait_ok = 1;
            end
        end
    endtask

    task automatic t_wait_master_valid;
        input [10:0] I_expected_length;
        begin
            S_wait_count = 0;
            S_wait_ok = 0;
            while ((W_master_rx_valid !== 1'b1) &&
                   (S_wait_count < P_VALID_TIMEOUT_CYCLES)) begin
                @(posedge S_clk_app);
                #1;
                S_wait_count = S_wait_count + 1;
            end
            if (W_master_rx_valid !== 1'b1) begin
                $display("FAIL master RX valid timeout cycles=%0d length=%0d",
                    S_wait_count, W_master_rx_length);
                S_failures = S_failures + 1;
            end else if (W_master_rx_length !== I_expected_length) begin
                $display("FAIL master RX first-valid length=%0d expected=%0d",
                    W_master_rx_length, I_expected_length);
                S_failures = S_failures + 1;
            end else begin
                S_wait_ok = 1;
            end
        end
    endtask

    task automatic t_start_slave_frame_then_board_reset;
        begin
            for (S_index = 0; S_index < 120; S_index = S_index + 1) begin
                @(negedge S_clk_rx);
                S_slave_gmii_rx_dv = 1'b1;
                S_slave_gmii_rx_er = 1'b0;
                S_slave_gmii_rxd = S_request_golden[S_index];
            end
            @(negedge S_clk_rx);
            S_slave_rst_app_n = 1'b0;
            S_slave_rst_tx_n = 1'b0;
            S_slave_rst_rx_n = 1'b0;
            S_slave_gmii_rx_dv = 1'b0;
            S_slave_gmii_rx_er = 1'b0;
            S_slave_gmii_rxd = 8'h00;
            repeat (4) @(posedge S_clk_app);
            @(negedge S_clk_app);
            #1;
            S_slave_rst_app_n = 1'b1;
            S_slave_rst_tx_n = 1'b1;
            S_slave_rst_rx_n = 1'b1;
            repeat (8) @(posedge S_clk_app);
        end
    endtask

    task automatic t_inject_request_to_slave;
        input integer I_length;
        input integer I_variant;
        begin
            for (S_index = 0; S_index < I_length; S_index = S_index + 1) begin
                @(negedge S_clk_rx);
                S_slave_gmii_rx_dv = 1'b1;
                S_slave_gmii_rx_er = (I_variant == 4) && (S_index == 80);
                if (I_variant == 1) begin
                    S_slave_gmii_rxd = S_request_bad_crc[S_index];
                end else if (I_variant == 2) begin
                    S_slave_gmii_rxd = S_request_wrong_destination[S_index];
                end else if (I_variant == 3) begin
                    S_slave_gmii_rxd = S_request_wrong_type[S_index];
                end else begin
                    S_slave_gmii_rxd = S_request_golden[S_index];
                end
            end
            @(negedge S_clk_rx);
            S_slave_gmii_rx_dv = 1'b0;
            S_slave_gmii_rx_er = 1'b0;
            S_slave_gmii_rxd = 8'h00;
            repeat (4) @(posedge S_clk_rx);
        end
    endtask

    task automatic t_inject_response_to_master;
        input integer I_length;
        begin
            for (S_index = 0; S_index < I_length; S_index = S_index + 1) begin
                @(negedge S_clk_rx);
                S_master_gmii_rx_dv = 1'b1;
                S_master_gmii_rx_er = 1'b0;
                S_master_gmii_rxd = S_response_golden[S_index];
            end
            @(negedge S_clk_rx);
            S_master_gmii_rx_dv = 1'b0;
            S_master_gmii_rx_er = 1'b0;
            S_master_gmii_rxd = 8'h00;
            repeat (4) @(posedge S_clk_rx);
        end
    endtask

    task automatic t_inject_response_to_slave;
        begin
            for (S_index = 0; S_index < P_RESPONSE_GMII_LENGTH; S_index = S_index + 1) begin
                @(negedge S_clk_rx);
                S_slave_gmii_rx_dv = 1'b1;
                S_slave_gmii_rx_er = 1'b0;
                S_slave_gmii_rxd = S_response_to_slave[S_index];
            end
            @(negedge S_clk_rx);
            S_slave_gmii_rx_dv = 1'b0;
            S_slave_gmii_rx_er = 1'b0;
            S_slave_gmii_rxd = 8'h00;
            repeat (4) @(posedge S_clk_rx);
        end
    endtask

    // Keep the RX length synchronizer one app cycle behind the hold level.
    // The old DUT publishes O_rx_valid while still latching the stale
    // length; the repaired DUT gives the bundled length one extra hold stage.
    task automatic t_inject_response_to_slave_with_length_lag;
        begin
            force u_slave.S_rx_length_s2_app = 11'd0;
            for (S_index = 0; S_index < P_RESPONSE_GMII_LENGTH; S_index = S_index + 1) begin
                @(negedge S_clk_rx);
                S_slave_gmii_rx_dv = 1'b1;
                S_slave_gmii_rx_er = 1'b0;
                S_slave_gmii_rxd = S_response_to_slave[S_index];
            end
            @(negedge S_clk_rx);
            S_slave_gmii_rx_dv = 1'b0;
            S_slave_gmii_rx_er = 1'b0;
            S_slave_gmii_rxd = 8'h00;
            repeat (4) @(posedge S_clk_rx);

            S_wait_count = 0;
            while ((u_slave.S_rx_hold_s2_app !== 1'b1) &&
                   (S_wait_count < P_VALID_TIMEOUT_CYCLES)) begin
                @(negedge S_clk_app);
                #1;
                S_wait_count = S_wait_count + 1;
            end
            if (u_slave.S_rx_hold_s2_app !== 1'b1) begin
                $display("FAIL RX length-lag injection hold timeout");
                S_failures = S_failures + 1;
            end
            release u_slave.S_rx_length_s2_app;
        end
    endtask

    task automatic t_inject_bad_response_padding_to_master;
        begin
            for (S_index = 0; S_index < P_RESPONSE_GMII_LENGTH; S_index = S_index + 1) begin
                @(negedge S_clk_rx);
                S_master_gmii_rx_dv = 1'b1;
                S_master_gmii_rx_er = 1'b0;
                S_master_gmii_rxd = S_response_bad_padding[S_index];
            end
            @(negedge S_clk_rx);
            S_master_gmii_rx_dv = 1'b0;
            S_master_gmii_rx_er = 1'b0;
            S_master_gmii_rxd = 8'h00;
            repeat (4) @(posedge S_clk_rx);
        end
    endtask

    task automatic t_check_slave_request;
        begin
            t_wait_slave_valid(11'd912);
            if (S_wait_ok) begin
                for (S_index = 0; S_index < P_REQUEST_APP_LENGTH; S_index = S_index + 1) begin
                    @(negedge S_clk_app);
                    S_slave_rx_read_addr = S_index[9:0];
                    @(posedge S_clk_app);
                    #1 t_check_equal(W_slave_rx_read_data, S_request_app[S_index], S_index, "request-rx");
                end
            end
        end
    endtask

    task automatic t_check_master_response;
        begin
            t_wait_master_valid(11'd46);
            if (S_wait_ok) begin
                for (S_index = 0; S_index < 46; S_index = S_index + 1) begin
                    @(negedge S_clk_app);
                    S_master_rx_read_addr = S_index[9:0];
                    @(posedge S_clk_app);
                    #1 if (S_index < P_RESPONSE_APP_LENGTH) begin
                        t_check_equal(W_master_rx_read_data, S_response_app[S_index], S_index, "response-rx");
                    end else begin
                        t_check_equal(W_master_rx_read_data, 8'h00, S_index, "response-padding");
                    end
                end
            end
        end
    endtask

    initial begin
        $readmemh("golden-vectors/request-app.hex", S_request_app);
        $readmemh("golden-vectors/response-app.hex", S_response_app);
        $readmemh("golden-vectors/request-gmii.hex", S_request_golden);
        $readmemh("golden-vectors/response-gmii.hex", S_response_golden);
        for (S_index = 0; S_index < P_REQUEST_GMII_LENGTH; S_index = S_index + 1) begin
            S_request_bad_crc[S_index] = S_request_golden[S_index];
            S_request_wrong_destination[S_index] = S_request_golden[S_index];
            S_request_wrong_type[S_index] = S_request_golden[S_index];
        end
        for (S_index = 0; S_index < P_RESPONSE_GMII_LENGTH; S_index = S_index + 1) begin
            S_response_bad_padding[S_index] = S_response_golden[S_index];
        end
        S_request_bad_crc[P_REQUEST_GMII_LENGTH - 1] =
            S_request_bad_crc[P_REQUEST_GMII_LENGTH - 1] ^ 8'h01;
        S_request_wrong_destination[8] = 8'h03;
        S_request_wrong_type[20] = 8'h08;
        S_request_wrong_type[21] = 8'h00;
        S_response_bad_padding[54] = 8'hA5;
        begin : B_REBUILD_RESPONSE_BAD_PADDING_FCS
            reg [31:0] W_crc;
            W_crc = 32'hFFFF_FFFF;
            for (S_index = 8; S_index < 68; S_index = S_index + 1) begin
                W_crc = f_tb_crc32_byte(W_crc, S_response_bad_padding[S_index]);
            end
            W_crc = ~W_crc;
            S_response_bad_padding[68] = W_crc[7:0];
            S_response_bad_padding[69] = W_crc[15:8];
            S_response_bad_padding[70] = W_crc[23:16];
            S_response_bad_padding[71] = W_crc[31:24];
        end

        S_rst_app_n = 1'b0;
        S_rst_tx_n = 1'b0;
        S_rst_rx_n = 1'b0;
        S_slave_rst_app_n = 1'b0;
        S_slave_rst_tx_n = 1'b0;
        S_slave_rst_rx_n = 1'b0;
        S_master_tx_write_en = 1'b0;
        S_master_tx_write_addr = 10'd0;
        S_master_tx_write_data = 8'h00;
        S_master_tx_commit = 1'b0;
        S_master_tx_length = 11'd0;
        S_master_rx_read_addr = 10'd0;
        S_master_rx_release = 1'b0;
        S_master_gmii_rxd = 8'h00;
        S_master_gmii_rx_dv = 1'b0;
        S_master_gmii_rx_er = 1'b0;
        S_slave_tx_write_en = 1'b0;
        S_slave_tx_write_addr = 10'd0;
        S_slave_tx_write_data = 8'h00;
        S_slave_tx_commit = 1'b0;
        S_slave_tx_length = 11'd0;
        S_slave_rx_read_addr = 10'd0;
        S_slave_rx_release = 1'b0;
        S_slave_gmii_rxd = 8'h00;
        S_slave_gmii_rx_dv = 1'b0;
        S_slave_gmii_rx_er = 1'b0;
        S_request_capture_count = 0;
        S_response_capture_count = 0;
        S_failures = 0;
        t_build_response_to_slave();
        S_wait_count = 0;
        S_wait_ok = 0;
        S_crc_known_data[0] = "1";
        S_crc_known_data[1] = "2";
        S_crc_known_data[2] = "3";
        S_crc_known_data[3] = "4";
        S_crc_known_data[4] = "5";
        S_crc_known_data[5] = "6";
        S_crc_known_data[6] = "7";
        S_crc_known_data[7] = "8";
        S_crc_known_data[8] = "9";

        t_check_known_crc();

        repeat (6) @(posedge S_clk_app);
        S_rst_app_n = 1'b1;
        S_rst_tx_n = 1'b1;
        S_rst_rx_n = 1'b1;
        S_slave_rst_app_n = 1'b1;
        S_slave_rst_tx_n = 1'b1;
        S_slave_rst_rx_n = 1'b1;
        repeat (6) @(posedge S_clk_app);

        @(negedge S_clk_app);
        S_master_tx_length = 11'd1025;
        S_master_tx_commit = 1'b1;
        @(posedge S_clk_app);
        @(negedge S_clk_app);
        S_master_tx_commit = 1'b0;
        if (W_master_tx_invalid_drops != 32'd1) begin
            $display("FAIL invalid TX length count=%0d expected=1", W_master_tx_invalid_drops);
            S_failures = S_failures + 1;
        end
        S_master_tx_length = 11'd0;

        t_write_master_request_with_length_lag();
        repeat (1500) @(posedge S_clk_tx);
        if (!W_master_tx_done && W_master_tx_busy) begin
            $display("FAIL master TX did not complete");
            S_failures = S_failures + 1;
        end
        t_compare_request_tx();

        // A common board reset must discard an in-flight TX request without
        // allowing the synchronized request level to restart after release.
        @(negedge S_clk_tx);
        S_request_capture_count = 0;
        t_write_master_request();
        repeat (20) @(posedge S_clk_tx);
        #1;
        if (!W_master_tx_busy) begin
            $display("FAIL master TX was not busy before board reset");
            S_failures = S_failures + 1;
        end
        @(negedge S_clk_app);
        S_master_tx_commit = 1'b1;
        @(posedge S_clk_app);
        @(negedge S_clk_app);
        S_master_tx_commit = 1'b0;
        if (W_master_tx_busy_drops != 32'd1) begin
            $display("FAIL busy TX commit count=%0d expected=1", W_master_tx_busy_drops);
            S_failures = S_failures + 1;
        end
        @(negedge S_clk_tx);
        #1;
        S_partial_capture_count = S_request_capture_count;
        if ((S_partial_capture_count <= 0) ||
            (S_partial_capture_count >= P_REQUEST_GMII_LENGTH)) begin
            $display("FAIL master partial TX count=%0d", S_partial_capture_count);
            S_failures = S_failures + 1;
        end
        S_rst_app_n = 1'b0;
        S_rst_tx_n = 1'b0;
        S_rst_rx_n = 1'b0;
        repeat (4) @(posedge S_clk_app);
        @(negedge S_clk_app);
        #1;
        S_rst_app_n = 1'b1;
        S_rst_tx_n = 1'b1;
        S_rst_rx_n = 1'b1;
        repeat (8) @(posedge S_clk_tx);
        if (W_master_tx_busy || (S_request_capture_count != S_partial_capture_count)) begin
            $display("FAIL master TX restarted after board reset busy=%b count=%0d before=%0d",
                W_master_tx_busy, S_request_capture_count, S_partial_capture_count);
            S_failures = S_failures + 1;
        end

        t_inject_request_to_slave(P_REQUEST_GMII_LENGTH, 0);
        t_check_slave_request();

        // The first complete packet remains held while a second packet arrives;
        // the second packet must be dropped as one unit.
        t_inject_request_to_slave(P_REQUEST_GMII_LENGTH, 0);
        if ((W_slave_rx_busy_drops != 32'd1) ||
            (W_slave_busy_drop_count != W_slave_rx_busy_drops)) begin
            $display("FAIL slave busy-drop count=%0d alias=%0d expected=1",
                W_slave_rx_busy_drops, W_slave_busy_drop_count);
            S_failures = S_failures + 1;
        end
        @(negedge S_clk_app);
        S_slave_rx_release = 1'b1;
        @(posedge S_clk_app);
        @(negedge S_clk_app);
        S_slave_rx_release = 1'b0;
        repeat (8) @(posedge S_clk_app);

        // A second valid packet with a different wire length must expose its
        // new length on the very first cycle of O_rx_valid.
        t_inject_response_to_slave_with_length_lag();
        t_wait_slave_valid(11'd46);
        @(negedge S_clk_app);
        S_slave_rx_release = 1'b1;
        @(posedge S_clk_app);
        @(negedge S_clk_app);
        S_slave_rx_release = 1'b0;
        repeat (8) @(posedge S_clk_app);

        t_inject_request_to_slave(P_REQUEST_GMII_LENGTH, 1);
        t_inject_request_to_slave(P_REQUEST_GMII_LENGTH, 2);
        t_inject_request_to_slave(P_REQUEST_GMII_LENGTH, 3);
        t_inject_request_to_slave(P_REQUEST_GMII_LENGTH, 4);
        t_inject_request_to_slave(80, 0);
        if ((W_slave_rx_bad_drops != 5) ||
            (W_slave_bad_frame_count != W_slave_rx_bad_drops)) begin
            $display("FAIL slave bad-frame count=%0d alias=%0d expected=5",
                W_slave_rx_bad_drops, W_slave_bad_frame_count);
            S_failures = S_failures + 1;
        end

        t_write_slave_response();
        repeat (500) @(posedge S_clk_tx);
        t_compare_response_tx();
        t_inject_response_to_master(P_RESPONSE_GMII_LENGTH);
        t_check_master_response();
        @(negedge S_clk_app);
        S_master_rx_release = 1'b1;
        @(posedge S_clk_app);
        @(negedge S_clk_app);
        S_master_rx_release = 1'b0;
        repeat (8) @(posedge S_clk_app);
        t_inject_bad_response_padding_to_master();
        if (W_master_rx_bad_drops != 32'd1) begin
            $display("FAIL master nonzero response padding accepted bad=%0d", W_master_rx_bad_drops);
            S_failures = S_failures + 1;
        end

        // Reset the three slave domains together while RX is inside a frame.
        t_start_slave_frame_then_board_reset();
        if (W_slave_rx_valid || (W_slave_rx_busy_drops != 32'd0) ||
            (W_slave_bad_frame_count != 32'd0)) begin
            $display("FAIL slave board reset left stale RX state valid=%b busy=%0d bad=%0d",
                W_slave_rx_valid, W_slave_rx_busy_drops, W_slave_bad_frame_count);
            S_failures = S_failures + 1;
        end
        t_inject_request_to_slave(P_REQUEST_GMII_LENGTH, 0);
        if (W_slave_rx_busy_drops != 0) begin
            $display("FAIL slave busy-drop count=%0d expected=0 after board reset", W_slave_rx_busy_drops);
            S_failures = S_failures + 1;
        end
        @(negedge S_clk_app);
        S_slave_rx_release = 1'b1;
        @(posedge S_clk_app);
        @(negedge S_clk_app);
        S_slave_rx_release = 1'b0;

        if (W_master_gmii_tx_er || W_slave_gmii_tx_er) begin
            $display("FAIL GMII TX error asserted during valid operation");
            S_failures = S_failures + 1;
        end
        if ((W_master_rx_busy_drops != 32'd0) ||
            (W_master_rx_oversize_drops != 32'd0) ||
            (W_master_bad_frame_count != W_master_rx_bad_drops) ||
            (W_master_busy_drop_count != W_master_rx_busy_drops) ||
            W_slave_tx_busy ||
            W_slave_tx_done ||
            (W_slave_tx_busy_drops != 32'd0) ||
            (W_slave_tx_invalid_drops != 32'd0) ||
            (W_slave_rx_oversize_drops != 32'd0)) begin
            $display("FAIL diagnostic counter aliases or unexpected counters");
            S_failures = S_failures + 1;
        end

        if (S_failures == 0) begin
            $display("TB_CTL_LPR_PACKET_LINK_PASS failures=0");
        end else begin
            $display("TB_CTL_LPR_PACKET_LINK_FAIL failures=%0d", S_failures);
        end
        $finish;
    end

endmodule
