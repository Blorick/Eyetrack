`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// Directed checks for the shared GMII TX arbiter.
// The test deliberately keeps both requests asserted while the selected
// endpoint is active; a second client must not reach GMII until done+IFG.
// -----------------------------------------------------------------------------
module tb_ctl_gmii_tx_scheduler;

    reg I_clk_tx;
    reg I_rst_tx_n;
    reg I_tx0_request;
    reg I_tx1_request;
    reg I_tx0_done;
    reg I_tx1_done;
    wire O_tx0_grant;
    wire O_tx1_grant;
    reg [7:0] I_tx0_txd;
    reg I_tx0_tx_en;
    reg I_tx0_tx_er;
    reg [7:0] I_tx1_txd;
    reg I_tx1_tx_en;
    reg I_tx1_tx_er;
    wire [7:0] O_txd;
    wire O_tx_en;
    wire O_tx_er;
    wire O_owner_valid;
    wire O_owner;
    integer quiet_cycles;
    integer wait_cycles;

    ctl_gmii_tx_scheduler #(
        .P_TX_SCHEDULE(0)
    ) u_dut (
        .I_clk_tx      (I_clk_tx),
        .I_rst_tx_n    (I_rst_tx_n),
        .I_tx0_request (I_tx0_request),
        .I_tx1_request (I_tx1_request),
        .I_tx0_done    (I_tx0_done),
        .I_tx1_done    (I_tx1_done),
        .O_tx0_grant   (O_tx0_grant),
        .O_tx1_grant   (O_tx1_grant),
        .I_tx0_txd     (I_tx0_txd),
        .I_tx0_tx_en   (I_tx0_tx_en),
        .I_tx0_tx_er   (I_tx0_tx_er),
        .I_tx1_txd     (I_tx1_txd),
        .I_tx1_tx_en   (I_tx1_tx_en),
        .I_tx1_tx_er   (I_tx1_tx_er),
        .O_txd        (O_txd),
        .O_tx_en      (O_tx_en),
        .O_tx_er      (O_tx_er),
        .O_owner_valid(O_owner_valid),
        .O_owner      (O_owner)
    );

    always #4 I_clk_tx = ~I_clk_tx;

    task check_condition;
        input I_condition;
        input [255:0] I_message;
        begin
            if (!I_condition) begin
                $display("TB_RESULT: FAIL: %0s", I_message);
                $fatal(1);
            end
        end
    endtask

    initial begin
        I_clk_tx = 1'b0;
        I_rst_tx_n = 1'b0;
        I_tx0_request = 1'b0;
        I_tx1_request = 1'b0;
        I_tx0_done = 1'b0;
        I_tx1_done = 1'b0;
        I_tx0_txd = 8'hB5;
        I_tx0_tx_en = 1'b1;
        I_tx0_tx_er = 1'b0;
        I_tx1_txd = 8'hB6;
        I_tx1_tx_en = 1'b1;
        I_tx1_tx_er = 1'b0;

        repeat (3) @(posedge I_clk_tx);
        I_rst_tx_n = 1'b1;
        @(posedge I_clk_tx);
        I_tx0_request <= 1'b1;
        I_tx1_request <= 1'b1;

        // Fixed policy grants OCR/control client 0 first.
        @(posedge I_clk_tx);
        #1 check_condition(O_tx0_grant && !O_tx1_grant, "fixed priority grant");
        #1 check_condition(O_owner_valid && (O_owner == 1'b0), "owner locked to client 0");
        repeat (3) begin
            @(posedge I_clk_tx);
            #1 check_condition(!O_tx0_grant && !O_tx1_grant && O_owner_valid &&
                      (O_owner == 1'b0) && O_tx_en && (O_txd == 8'hB5),
                      "client 1 must not collide with client 0");
        end

        I_tx0_done <= 1'b1;
        @(posedge I_clk_tx);
        I_tx0_done <= 1'b0;
        I_tx0_request <= 1'b0;
        // The scheduler observes done, releases owner, then grants client 1.
        @(posedge I_clk_tx);
        #1 check_condition(O_tx1_grant && !O_tx0_grant, "second client grant after done");
        #1 check_condition(O_owner_valid && (O_owner == 1'b1), "owner switched to client 1");

        I_tx1_done <= 1'b1;
        @(posedge I_clk_tx);
        I_tx1_done <= 1'b0;
        I_tx1_request <= 1'b0;
        @(posedge I_clk_tx);
        #1 check_condition(!O_owner_valid && !O_tx_en, "owner released after client 1");

        // Simulate a mode-only reset while client 1 owns the shared GMII.
        // Its request and done disappear together; client 0 must not be
        // starved behind the abandoned owner, and the scheduler must insert at
        // least the Ethernet 12-clock IFG before granting OCR.
        I_tx1_request <= 1'b1;
        I_tx0_request <= 1'b0;
        @(posedge I_clk_tx);
        #1 check_condition(O_tx1_grant && O_owner_valid && (O_owner == 1'b1),
                           "mode owner granted for abort test");
        I_tx1_request <= 1'b0;
        // The top-level mode-lock guard presents a synthetic done pulse while
        // the reset has already withdrawn the mode request.  This combination
        // must take the scheduler's abort path, not the normal completion path.
        I_tx1_done    <= 1'b1;
        I_tx0_request <= 1'b1;
        quiet_cycles = 0;
        wait_cycles = 0;
        while (!O_tx0_grant && wait_cycles < 40) begin
            @(posedge I_clk_tx);
            #1;
            wait_cycles = wait_cycles + 1;
            if (!O_owner_valid && !O_tx_en)
                quiet_cycles = quiet_cycles + 1;
        end
        #1 check_condition(O_tx0_grant && O_owner_valid && (O_owner == 1'b0),
                           "OCR resumes after mode abort");
        #1 check_condition(quiet_cycles >= 12,
                           "mode abort keeps at least 12 clocks IFG");
        I_tx1_done <= 1'b0;

        $display("TB_RESULT: PASS (scheduler fixed-priority ownership)");
        $finish;
    end

endmodule
