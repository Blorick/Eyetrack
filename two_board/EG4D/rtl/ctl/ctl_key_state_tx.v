// ============================================================================
// ctl_key_state_tx.v -- EG4D key-state packet producer for the 0x88B5 link.
//
// The shared ctl_lpr_packet_link RX contract accepts two application kinds:
//   kind 1: 16-byte header + 896-byte payload (legacy plate result)
//   kind 2: 16-byte header +  16-byte payload
// This module uses kind 2 (the previous legacy "OCR request" shape), so the
// existing PH1P receive endpoint accepts the frame without any protocol change:
//
//   app byte 0..1  : 8'h4C, 8'h50 ("LP" magic)
//   app byte 2     : version 8'h01
//   app byte 3     : kind    8'h02
//   app byte 12..13: payload length = 16
//   app byte 14    : mask MSB = 0
//   app byte 15    : flags = 0
//   app byte 16..31: payload (key packet)
//
// Payload: A5 4B key ~key flags, remaining bytes zero; key 0 is release.
// flags[0] is the cycle-reset request toggle and flags[1] is automatic OSD.
// A packet is sent whenever the
// debounced key changes and every P_REFRESH_CYCLES (100 ms) as a recovery
// beacon so the PH1P side can rebuild state after a link glitch.
// ============================================================================
`timescale 1ns / 1ps

module ctl_key_state_tx #(
    parameter integer P_REFRESH_CYCLES = 2500000   // 100 ms at 25 MHz
)(
    input  wire        clk,            // 25 MHz application clock
    input  wire        rst_n,
    input  wire [4:0]  I_key_code,     // 1..16
    input  wire        I_key_valid,
    input  wire        I_key_auto,
    input  wire        I_cycle_reset_request,
    input  wire        I_tx_busy,      // link TX mailbox busy
    output wire        O_tx_done,      // one-cycle pulse after a frame is sent
    output reg  [9:0]  O_wr_addr,
    output reg  [7:0]  O_wr_data,
    output reg         O_wr_en,
    output reg         O_commit,
    output reg  [10:0] O_length
);

    localparam integer P_TIMER_W = 22;   // 2,500,000 < 2^22

    localparam [2:0] S_IDLE    = 3'd0;
    localparam [2:0] S_WRITE   = 3'd1;
    localparam [2:0] S_COMMIT  = 3'd2;
    localparam [2:0] S_WAIT_UP = 3'd3;
    localparam [2:0] S_WAIT_DN = 3'd4;

    reg [2:0]           S_state;
    reg [9:0]           S_byte_idx;
    reg [4:0]           S_sent_code;
    reg [4:0]           S_want_code;
    reg [P_TIMER_W-1:0] S_refresh_cnt;
    reg                 S_refresh_pulse;
    reg                 S_done_q;
    reg  [9:0]          S_wait_cnt;
    reg                 S_sent_reset_request;
    reg                 S_want_reset_request;
    reg                 S_sent_key_auto;
    reg                 S_want_key_auto;

    assign O_tx_done = S_done_q;
    wire [4:0] W_key_code = (I_key_valid && I_key_code >= 5'd1 &&
                            I_key_code <= 5'd16) ? I_key_code : 5'd0;

    // Byte value for application byte index S_byte_idx.
    function [7:0] f_app_byte;
        input [9:0] index;
        input [4:0] key;
        begin
            case (index)
                10'd0:  f_app_byte = 8'h4C;
                10'd1:  f_app_byte = 8'h50;
                10'd2:  f_app_byte = 8'h01;
                10'd3:  f_app_byte = 8'h02;
                10'd12: f_app_byte = 8'h00;
                10'd13: f_app_byte = 8'h10;
                10'd14: f_app_byte = 8'h00;
                10'd15: f_app_byte = 8'h00;
                10'd16: f_app_byte = 8'hA5;
                10'd17: f_app_byte = 8'h4B;
                10'd18: f_app_byte = {3'b000, key};
                10'd19: f_app_byte = {3'b000, ~key};
                10'd20: f_app_byte = {6'd0, S_want_key_auto, S_want_reset_request};
                default: f_app_byte = 8'h00;
            endcase
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_state         <= S_IDLE;
            S_byte_idx      <= 10'd0;
            S_sent_code     <= 5'd0;
            S_want_code     <= 5'd0;
            S_refresh_cnt   <= P_REFRESH_CYCLES - 1;
            S_refresh_pulse <= 1'b0;
            S_done_q        <= 1'b0;
            S_wait_cnt      <= 10'd0;
            S_sent_reset_request <= 1'b0;
            S_want_reset_request <= 1'b0;
            S_sent_key_auto <= 1'b0;
            S_want_key_auto <= 1'b0;
            O_wr_addr       <= 10'd0;
            O_wr_data       <= 8'd0;
            O_wr_en         <= 1'b0;
            O_commit        <= 1'b0;
            O_length        <= 11'd32;
        end else begin
            S_done_q  <= 1'b0;
            O_wr_en   <= 1'b0;
            O_commit  <= 1'b0;

            // 100 ms recovery beacon.
            if (S_refresh_cnt == {P_TIMER_W{1'b0}}) begin
                S_refresh_cnt   <= P_REFRESH_CYCLES - 1;
                S_refresh_pulse <= 1'b1;
            end else begin
                S_refresh_cnt   <= S_refresh_cnt - 1'b1;
                S_refresh_pulse <= 1'b0;
            end

            case (S_state)
                S_IDLE: begin
                    if (!I_tx_busy &&
                        (S_refresh_pulse || (W_key_code != S_sent_code) ||
                         (I_cycle_reset_request != S_sent_reset_request) ||
                         (I_key_auto != S_sent_key_auto))) begin
                        S_byte_idx  <= 10'd0;
                        S_want_code <= W_key_code;
                        S_want_reset_request <= I_cycle_reset_request;
                        S_want_key_auto <= I_key_auto;
                        S_state     <= S_WRITE;
                    end
                end
                S_WRITE: begin
                    O_wr_en   <= 1'b1;
                    O_wr_addr <= S_byte_idx;
                    O_wr_data <= f_app_byte(S_byte_idx, S_want_code);
                    if (S_byte_idx == 10'd31) begin
                        S_state <= S_COMMIT;
                    end else begin
                        S_byte_idx <= S_byte_idx + 1'b1;
                    end
                end
                S_COMMIT: begin
                    O_length   <= 11'd32;
                    O_commit   <= 1'b1;
                    S_wait_cnt <= 10'd0;
                    S_state    <= S_WAIT_UP;
                end
                S_WAIT_UP: begin
                    // Wait for the link to take ownership; bounded so a busy
                    // link cannot wedge the scanner.
                    if (I_tx_busy) begin
                        S_wait_cnt <= 10'd0;
                        S_state    <= S_WAIT_DN;
                    end else if (S_wait_cnt == 10'd1023) begin
                        S_state <= S_IDLE;
                    end else begin
                        S_wait_cnt <= S_wait_cnt + 1'b1;
                    end
                end
                S_WAIT_DN: begin
                    if (!I_tx_busy) begin
                        S_sent_code <= S_want_code;
                        S_sent_reset_request <= S_want_reset_request;
                        S_sent_key_auto <= S_want_key_auto;
                        S_done_q    <= 1'b1;
                        S_state     <= S_IDLE;
                    end else if (S_wait_cnt == 10'd1023) begin
                        // Retry the newest state once the link becomes available.
                        S_state <= S_IDLE;
                    end else begin
                        S_wait_cnt <= S_wait_cnt + 1'b1;
                    end
                end
                default: S_state <= S_IDLE;
            endcase
        end
    end

endmodule
