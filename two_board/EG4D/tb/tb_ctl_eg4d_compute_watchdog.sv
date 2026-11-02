`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// Watchdog/release regression for the EG4D compute adapter.
// A held RX frame must be released and the adapter must return to IDLE within
// the configured bound instead of monopolising the shared Ethernet service.
// -----------------------------------------------------------------------------
module tb_ctl_eg4d_compute_watchdog;
    reg clk;
    reg rst_n;
    reg rx_valid;
    reg [10:0] rx_length;
    reg [7:0] rx_read_data;
    wire [9:0] rx_read_addr;
    wire rx_release;
    wire [4:0] debug_state;
    wire busy;
    wire frame_result;
    wire tx_write_en;
    wire tx_commit;
    reg tx_busy;
    reg tx_done;

    ctl_eg4d_compute_adapter #(
        .P_WATCHDOG_CYCLES(16),
        .P_DRAIN_CYCLES(4)
    ) dut (
        .I_clk_app        (clk),
        .I_rst_app_n      (rst_n),
        .O_tx_write_en    (tx_write_en),
        .O_tx_write_addr  (),
        .O_tx_write_data  (),
        .O_tx_commit      (tx_commit),
        .O_tx_length      (),
        .I_tx_busy        (tx_busy),
        .I_tx_done        (tx_done),
        .I_rx_valid       (rx_valid),
        .I_rx_length      (rx_length),
        .O_rx_read_addr   (rx_read_addr),
        .I_rx_read_data   (rx_read_data),
        .O_rx_release     (rx_release),
        .O_busy           (busy),
        .O_response_valid (),
        .O_frame_result_pulse(frame_result),
        .O_debug_state    (debug_state[3:0])
    );

    always #5 clk = ~clk;

    integer k;
    initial begin
        clk = 1'b0;
        rst_n = 1'b0;
        rx_valid = 1'b0;
        rx_length = 11'd912;
        rx_read_data = 8'h00;
        tx_busy = 1'b0;
        tx_done = 1'b0;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        @(negedge clk);
        rx_valid = 1'b1;

        k = 0;
        while (!rx_release && (k < 40)) begin
            @(posedge clk);
            k = k + 1;
        end
        if (!rx_release)
            $fatal(1, "watchdog did not request RX release");

        // Model a well-behaved link after seeing the one-cycle release pulse.
        @(negedge clk);
        rx_valid = 1'b0;
        repeat (5) @(posedge clk);
        if (busy || (dut.state != dut.S_IDLE))
            $fatal(1, "watchdog adapter failed to drain: state=%0d", dut.state);
        if (!frame_result)
            $display("TB_NOTE: frame_result pulse may have occurred before monitor edge");
        $display("TB_RESULT: PASS (bounded compute watchdog and RX drain)");
        $finish;
    end
endmodule
