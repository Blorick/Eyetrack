`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : declared module clock domain(s)
//  复位     : declared asynchronous active-low reset(s)
//  参数     : frozen module parameters; no interface change
//  Contract: comment-only style clone; ports and expressions are unchanged
// Module: ctl_mailbox_cdc
// Function: transfer one held data word with destination acceptance feedback
// Domain: ctl; independent source and destination clocks
// Reset: common asynchronous assertion; each domain synchronizes release
// Parameters: P_DATA_WIDTH selects a tile or snapshot/control tuple
// Contract: valid/ready acceptance; source data held until destination consumes
// Verification: arbitrary phase, backpressure, ordered data, and common reset
// -----------------------------------------------------------------------------
module ctl_mailbox_cdc #(
    parameter integer P_DATA_WIDTH = 41
) (
    input wire I_clk_src,
    input wire I_rst_src_n,
    input wire I_src_valid,
    input wire [P_DATA_WIDTH-1:0] I_src_data,
    output wire O_src_ready,
    output wire O_src_pending,
    input wire I_clk_dst,
    input wire I_rst_dst_n,
    output reg O_dst_valid,
    output reg [P_DATA_WIDTH-1:0] O_dst_data,
    input wire I_dst_ready
);

    reg [P_DATA_WIDTH-1:0] S_data_src;
    reg S_request_src;
    reg S_ack_s1_src;
    reg S_ack_s2_src;
    reg S_request_s1_dst;
    reg S_request_s2_dst;
    reg S_tag_dst;
    reg S_ack_dst;

    assign O_src_pending = S_request_src != S_ack_s2_src;
    assign O_src_ready = !O_src_pending;

    always @(posedge I_clk_src or negedge I_rst_src_n) begin
        if (!I_rst_src_n) begin
            S_data_src <= {P_DATA_WIDTH{1'b0}};
            S_request_src <= 1'b0;
            S_ack_s1_src <= 1'b0;
            S_ack_s2_src <= 1'b0;
        end else begin
            S_ack_s1_src <= S_ack_dst;
            S_ack_s2_src <= S_ack_s1_src;
            if (I_src_valid && O_src_ready) begin
                S_data_src <= I_src_data;
                S_request_src <= ~S_request_src;
            end
        end
    end

    // Only the toggle is synchronized. The bundled word is already stable
    // before the synchronized request is observed, and cannot change until
    // this domain has consumed its registered copy and the ACK returns.
    // ACK represents the valid/ready consumption edge, including a tile RAM
    // write on that edge; a snapshot sender must also wait for source idle.
    always @(posedge I_clk_dst or negedge I_rst_dst_n) begin
        if (!I_rst_dst_n) begin
            S_request_s1_dst <= 1'b0;
            S_request_s2_dst <= 1'b0;
            S_tag_dst <= 1'b0;
            S_ack_dst <= 1'b0;
            O_dst_valid <= 1'b0;
            O_dst_data <= {P_DATA_WIDTH{1'b0}};
        end else begin
            S_request_s1_dst <= S_request_src;
            S_request_s2_dst <= S_request_s1_dst;
            if (O_dst_valid) begin
                if (I_dst_ready) begin
                    O_dst_valid <= 1'b0;
                    S_ack_dst <= S_tag_dst;
                end
            end else if (S_request_s2_dst != S_ack_dst) begin
                O_dst_data <= S_data_src;
                S_tag_dst <= S_request_s2_dst;
                O_dst_valid <= 1'b1;
            end
        end
    end
endmodule
