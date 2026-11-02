`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : declared module clock domain(s)
//  复位     : declared asynchronous active-low reset(s)
//  参数     : frozen module parameters; no interface change
//  Contract: comment-only style clone; ports and expressions are unchanged
// Module: ctl_gmii_tx_scheduler3
// Function: hold one GMII owner across OCR, display-line, and ink-line packets
// Domain: ctl; all inputs and outputs use I_clk_tx
// Reset: asynchronous active-low assertion; board logic synchronizes release
// Parameters: P_TX_SCHEDULE=0 fixed priority, otherwise packet round robin
// Contract: each endpoint includes its Ethernet IFG before a normal done pulse
// Verification: isolated ownership, fairness, and endpoint-reset fault tests
// -----------------------------------------------------------------------------
module ctl_gmii_tx_scheduler3 #(
    parameter integer P_TX_SCHEDULE = 1
) (
    input wire I_clk_tx,
    input wire I_rst_tx_n,
    input wire [2:0] I_tx_request,
    input wire [2:0] I_tx_done,
    output reg [2:0] O_tx_grant,
    input wire [7:0] I_tx0_txd,
    input wire [7:0] I_tx1_txd,
    input wire [7:0] I_tx2_txd,
    input wire [2:0] I_tx_en,
    input wire [2:0] I_tx_er,
    output reg [7:0] O_txd,
    output reg O_tx_en,
    output reg O_tx_er,
    output wire O_owner_valid,
    output wire [1:0] O_owner
);

    reg S_owner_valid_tx;
    reg [1:0] S_owner_tx;
    reg [1:0] S_rr_next_tx;
    reg [3:0] S_abort_ifg_tx;
    reg S_done_wait_tx;
    reg [1:0] W_pick_tx;
    reg W_pick_valid_tx;
    reg W_owner_request_tx;
    reg W_owner_done_tx;

    function [1:0] f_next_owner;
        input [1:0] I_owner;
        begin
            f_next_owner = (I_owner == 2'd2) ? 2'd0 : I_owner + 2'd1;
        end
    endfunction

    assign O_owner_valid = S_owner_valid_tx;
    assign O_owner = S_owner_tx;

    always @(*) begin
        W_pick_tx = 2'd0;
        W_pick_valid_tx = |I_tx_request;
        if (P_TX_SCHEDULE == 0 || S_rr_next_tx == 2'd0) begin
            if (I_tx_request[0]) W_pick_tx = 2'd0;
            else if (I_tx_request[1]) W_pick_tx = 2'd1;
            else W_pick_tx = 2'd2;
        end else if (S_rr_next_tx == 2'd1) begin
            if (I_tx_request[1]) W_pick_tx = 2'd1;
            else if (I_tx_request[2]) W_pick_tx = 2'd2;
            else W_pick_tx = 2'd0;
        end else begin
            if (I_tx_request[2]) W_pick_tx = 2'd2;
            else if (I_tx_request[0]) W_pick_tx = 2'd0;
            else W_pick_tx = 2'd1;
        end

        W_owner_request_tx = 1'b0;
        W_owner_done_tx = 1'b0;
        O_txd = 8'd0;
        O_tx_en = 1'b0;
        O_tx_er = 1'b0;
        case (S_owner_tx)
            2'd0: begin
                W_owner_request_tx = I_tx_request[0];
                W_owner_done_tx = I_tx_done[0];
                if (S_owner_valid_tx) begin
                    O_txd = I_tx0_txd;
                    O_tx_en = I_tx_en[0];
                    O_tx_er = I_tx_er[0];
                end
            end
            2'd1: begin
                W_owner_request_tx = I_tx_request[1];
                W_owner_done_tx = I_tx_done[1];
                if (S_owner_valid_tx) begin
                    O_txd = I_tx1_txd;
                    O_tx_en = I_tx_en[1];
                    O_tx_er = I_tx_er[1];
                end
            end
            2'd2: begin
                W_owner_request_tx = I_tx_request[2];
                W_owner_done_tx = I_tx_done[2];
                if (S_owner_valid_tx) begin
                    O_txd = I_tx2_txd;
                    O_tx_en = I_tx_en[2];
                    O_tx_er = I_tx_er[2];
                end
            end
            default: begin
            end
        endcase
    end

    always @(posedge I_clk_tx or negedge I_rst_tx_n) begin
        if (!I_rst_tx_n) begin
            S_owner_valid_tx <= 1'b0;
            S_owner_tx <= 2'd0;
            S_rr_next_tx <= 2'd0;
            S_abort_ifg_tx <= 4'd0;
            S_done_wait_tx <= 1'b0;
            O_tx_grant <= 3'd0;
        end else begin
            O_tx_grant <= 3'd0;
            if (S_owner_valid_tx) begin
                // A one-cycle done pulse and simultaneous request withdrawal
                // are a normal completion. A persistent done level during
                // withdrawal denotes the existing mode-clock reset abort.
                if (S_done_wait_tx) begin
                    S_done_wait_tx <= 1'b0;
                    if (W_owner_done_tx && !W_owner_request_tx) begin
                        S_owner_valid_tx <= 1'b0;
                        S_rr_next_tx <= f_next_owner(S_owner_tx);
                        S_abort_ifg_tx <= 4'd12;
                    end else if (!W_owner_request_tx || W_owner_done_tx) begin
                        S_owner_valid_tx <= 1'b0;
                        S_rr_next_tx <= f_next_owner(S_owner_tx);
                        S_abort_ifg_tx <= 4'd0;
                    end
                end else if (W_owner_done_tx && !W_owner_request_tx) begin
                    S_done_wait_tx <= 1'b1;
                end else if (!W_owner_request_tx) begin
                    S_owner_valid_tx <= 1'b0;
                    S_rr_next_tx <= f_next_owner(S_owner_tx);
                    S_abort_ifg_tx <= 4'd12;
                end else if (W_owner_done_tx) begin
                    S_owner_valid_tx <= 1'b0;
                    S_rr_next_tx <= f_next_owner(S_owner_tx);
                    S_abort_ifg_tx <= 4'd0;
                end
            end else if (S_abort_ifg_tx != 4'd0) begin
                S_abort_ifg_tx <= S_abort_ifg_tx - 4'd1;
            end else if (W_pick_valid_tx) begin
                S_owner_valid_tx <= 1'b1;
                S_owner_tx <= W_pick_tx;
                S_done_wait_tx <= 1'b0;
                case (W_pick_tx)
                    2'd0: O_tx_grant <= 3'b001;
                    2'd1: O_tx_grant <= 3'b010;
                    2'd2: O_tx_grant <= 3'b100;
                    default: O_tx_grant <= 3'd0;
                endcase
            end
        end
    end
endmodule
