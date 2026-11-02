`timescale 1ns/1ps
//=============================================================================
//  ctl_master_ink_session -- guarded direct-link epoch establishment
//  所属域 : ctl
//  时钟域 : I_clk_app (PH pixel/application clock)
//  复位   : I_rst_app_n, asynchronous active low
//  Contract: finite residence bound; drain both RX protocols during barrier
//  Verification: isolated testbench authored; EDA execution pending
//=============================================================================
module ctl_master_ink_session #(
    parameter integer P_DRAIN_GUARD_CYCLES = 75000,
    parameter integer P_RX_QUIET_CYCLES = 64,
    parameter integer P_ACK_TIMEOUT_CYCLES = 750000
)(
    input wire I_clk_app,
    input wire I_rst_app_n,
    input wire I_restart,
    input wire I_peer_hello,
    input wire I_ack_seen,
    input wire [31:0] I_ack_epoch,
    input wire [31:0] I_ack_challenge,
    input wire I_transport_quiet,
    input wire I_rx_quiet,
    input wire I_control_ready,
    output wire O_control_valid,
    output wire [7:0] O_control_kind,
    output wire [31:0] O_control_epoch,
    output wire [31:0] O_control_challenge,
    output wire O_session_ready,
    output wire O_barrier,
    output reg O_result_clear,
    output wire [31:0] O_epoch
);
    localparam [2:0] P_FIRST = 0, P_GUARD = 1, P_FINAL = 2,
        P_ACK = 3, P_RX_DRAIN = 4, P_ACTIVE = 5;
    reg [2:0] S_state;
    reg [31:0] S_epoch, S_challenge;
    reg [31:0] S_timer, S_quiet_timer;
    wire W_restart = I_restart || (I_peer_hello && S_state == P_ACTIVE);
    assign O_control_valid = (S_state == P_FIRST || S_state == P_FINAL) && !W_restart;
    assign O_control_kind = 8'd5;
    assign O_control_epoch = S_epoch;
    assign O_control_challenge = S_challenge;
    assign O_session_ready = S_state == P_ACTIVE && !W_restart;
    assign O_barrier = !O_session_ready;
    assign O_epoch = S_epoch;
    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if(!I_rst_app_n) begin
            S_state <= P_FIRST; S_epoch <= 0; S_challenge <= 0;
            S_timer <= 0; S_quiet_timer <= 0; O_result_clear <= 1;
        end else begin
            O_result_clear <= 0;
            if(W_restart) begin
                S_state <= P_FIRST; S_epoch <= S_epoch + 1;
                S_challenge <= S_challenge + 1;
                S_timer <= 0; S_quiet_timer <= 0; O_result_clear <= 1;
            end else case(S_state)
                P_FIRST: if(I_control_ready) begin
                    S_state <= P_GUARD; S_timer <= 0;
                end
                P_GUARD: begin
                    if(S_timer < P_DRAIN_GUARD_CYCLES) S_timer <= S_timer + 1;
                    if(S_timer >= P_DRAIN_GUARD_CYCLES && I_transport_quiet && I_rx_quiet) begin
                        S_challenge <= S_challenge + 1; S_state <= P_FINAL;
                    end
                end
                P_FINAL: if(I_control_ready) begin
                    S_state <= P_ACK; S_timer <= 0;
                end
                P_ACK: begin
                    if(I_ack_seen && I_ack_epoch == S_epoch && I_ack_challenge == S_challenge) begin
                        S_state <= P_RX_DRAIN; S_quiet_timer <= 0;
                    end else if(S_timer >= P_ACK_TIMEOUT_CYCLES) begin
                        S_state <= P_FIRST; S_epoch <= S_epoch + 1;
                        S_challenge <= S_challenge + 1; S_timer <= 0; O_result_clear <= 1;
                    end else S_timer <= S_timer + 1;
                end
                P_RX_DRAIN: begin
                    if(!I_rx_quiet || !I_transport_quiet) S_quiet_timer <= 0;
                    else if(S_quiet_timer >= P_RX_QUIET_CYCLES) S_state <= P_ACTIVE;
                    else S_quiet_timer <= S_quiet_timer + 1;
                end
                P_ACTIVE: begin end
                default: begin S_state <= P_FIRST; O_result_clear <= 1; end
            endcase
        end
    end
endmodule
