`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// Module: ctl_master_ink_watchdog
// Function: recover a committed-frame transaction that receives no owned result
// Domain: ctl; the PH pixel/application clock, nominally 75 MHz
// Reset: asynchronous active-low assertion; synchronized release by the caller
// Parameter: P_TIMEOUT_CYCLES counts application clocks after the first EOF
// Contract: later EOFs do not extend a pending deadline; any owned result does
// Verification: timeout, boundary, barrier and repeated-EOF tests are pending
// -----------------------------------------------------------------------------
module ctl_master_ink_watchdog #(
    parameter [23:0] P_TIMEOUT_CYCLES = 24'd7500000
) (
    input wire I_clk_app,
    input wire I_rst_app_n,
    input wire I_session_ready,
    input wire I_eof_committed,
    input wire I_result_frame,
    output reg O_session_recover
);
    reg S_result_pending_app;
    reg [23:0] S_result_age_app;
    wire W_deadline_app = S_result_age_app >= P_TIMEOUT_CYCLES - 1'b1;

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            S_result_pending_app <= 1'b0;
            S_result_age_app <= 24'd0;
            O_session_recover <= 1'b0;
        end else begin
            O_session_recover <= 1'b0;
            if (!I_session_ready) begin
                S_result_pending_app <= 1'b0;
                S_result_age_app <= 24'd0;
            end else if (I_result_frame) begin
                // A coincident new EOF starts a new bounded transaction.
                S_result_pending_app <= I_eof_committed;
                S_result_age_app <= 24'd0;
            end else if (S_result_pending_app) begin
                if (W_deadline_app) begin
                    S_result_pending_app <= 1'b0;
                    S_result_age_app <= 24'd0;
                    O_session_recover <= 1'b1;
                end else begin
                    S_result_age_app <= S_result_age_app + 1'b1;
                end
            end else if (I_eof_committed) begin
                S_result_pending_app <= 1'b1;
                S_result_age_app <= 24'd0;
            end
        end
    end
endmodule
