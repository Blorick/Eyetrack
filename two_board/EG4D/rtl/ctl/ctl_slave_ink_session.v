`timescale 1ns/1ps
//=============================================================================
//  ctl_slave_ink_session -- cancel owners and drain before acknowledging epoch
//  所属域 : ctl
//  时钟域 : I_clk_app (EG 75 MHz application domain)
//  复位   : I_rst_app_n, asynchronous active low
//  Contract: barrier consumers force ready; old compute busy never delays ACK
//  Verification: standalone reset/stale ACK tests authored; EDA pending
//=============================================================================
module ctl_slave_ink_session #(
    parameter integer P_BOOT_RETRY_CYCLES = 75000
)(
    input wire I_clk_app,
    input wire I_rst_app_n,
    input wire I_hello_seen,
    input wire [31:0] I_hello_epoch,
    input wire [31:0] I_hello_challenge,
    input wire I_raw_quiet,
    input wire I_tile_quiet,
    input wire I_snapshot_quiet,
    input wire I_old_owner_invalid,
    input wire I_legacy_tx_quiet,
    input wire I_reply_tx_quiet,
    input wire I_rx_quiet,
    input wire I_control_ready,
    output wire O_control_valid,
    output wire [7:0] O_control_kind,
    output wire [31:0] O_control_epoch,
    output wire [31:0] O_control_challenge,
    output wire O_session_ready,
    output wire O_barrier,
    output reg O_abort_frame,
    output wire [31:0] O_epoch
);
    localparam [2:0] P_BOOT_HELLO = 0, P_WAIT_HELLO = 1,
        P_DRAIN = 2, P_SEND_ACK = 3, P_ACTIVE = 4;
    reg [2:0] S_state;
    reg [31:0] S_epoch, S_challenge, S_timer;
    wire W_quiet = I_raw_quiet && I_tile_quiet && I_snapshot_quiet &&
        I_old_owner_invalid && I_legacy_tx_quiet && I_reply_tx_quiet && I_rx_quiet;
    assign O_control_valid = (S_state == P_BOOT_HELLO || S_state == P_SEND_ACK) && !I_hello_seen;
    assign O_control_kind = S_state == P_BOOT_HELLO ? 8'd5 : 8'd6;
    assign O_control_epoch = S_epoch;
    assign O_control_challenge = S_challenge;
    assign O_session_ready = S_state == P_ACTIVE && !I_hello_seen;
    assign O_barrier = !O_session_ready;
    assign O_epoch = S_epoch;
    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if(!I_rst_app_n) begin
            S_state <= P_BOOT_HELLO; S_epoch <= 0; S_challenge <= 0;
            S_timer <= 0; O_abort_frame <= 1;
        end else begin
            O_abort_frame <= 0;
            if(I_hello_seen) begin
                // Override even an uncommitted old ACK. A committed old ACK
                // is drained by I_reply_tx_quiet before this ACK is offered.
                S_epoch <= I_hello_epoch; S_challenge <= I_hello_challenge;
                S_state <= P_DRAIN; S_timer <= 0; O_abort_frame <= 1;
            end else case(S_state)
                P_BOOT_HELLO: if(I_control_ready) begin S_state <= P_WAIT_HELLO; S_timer <= 0; end
                P_WAIT_HELLO: if(S_timer >= P_BOOT_RETRY_CYCLES) begin
                    S_state <= P_BOOT_HELLO; S_timer <= 0;
                end else S_timer <= S_timer + 1;
                P_DRAIN: if(W_quiet) S_state <= P_SEND_ACK;
                P_SEND_ACK: if(I_control_ready) S_state <= P_ACTIVE;
                P_ACTIVE: begin end
                default: begin S_state <= P_BOOT_HELLO; O_abort_frame <= 1; end
            endcase
        end
    end
endmodule
