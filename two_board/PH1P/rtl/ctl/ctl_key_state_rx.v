// ============================================================================
// ctl_key_state_rx.v -- PH1P key-state packet consumer for the 0x88B5 link.
//
// The EG4D sends a kind-2 legacy frame (see ctl_key_state_tx.v):
//   app bytes 0..15 : 16-byte header ("LP", version 1, kind 2, length 16)
//   app bytes 16..31: payload A5 4B key ~key {reserved[7:2],key_auto,reset} .. 00
//
// This block reads the link mailbox, validates the header/payload on the fly
// (no 32-byte buffer), latches the key code, and releases the mailbox.  The
// code is cleared by release packets or after 350 ms without a valid packet.
//
// ctl_lpr_packet_link exposes a registered read port: the address presented in
// cycle n returns data in cycle n+1.  The reader therefore uses two cycles per
// byte (address phase, sample phase), which tolerates that register stage.
// ============================================================================
`timescale 1ns / 1ps

module ctl_key_state_rx #(
    parameter integer P_TIMEOUT_CYCLES = 26250000
) (
    input  wire        clk,            // application clock (75 MHz pixel clock)
    input  wire        rst_n,
    input  wire        I_rx_valid,     // link mailbox holds a complete frame
    input  wire [10:0] I_rx_length,    // application byte count
    output reg  [9:0]  O_rx_addr,      // mailbox read address
    input  wire [7:0]  I_rx_data,      // mailbox read data, one cycle after address
    output reg         O_rx_release,   // release the mailbox after reading
    output reg  [4:0]  O_key_code,     // 1..16, 0 = none received yet
    output reg         O_key_valid,
    output reg         O_key_changed,
    output reg         O_cycle_reset_request, // legal byte20 bit0, held until next legal frame
    output reg         O_key_auto,             // legal byte20 bit1, held until next legal frame
    output reg  [15:0] O_rx_good_count,
    output reg  [15:0] O_rx_bad_count
);

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_READ = 2'd1;
    localparam [1:0] S_FIN  = 2'd2;
    localparam [1:0] S_DONE = 2'd3;

    reg [1:0]  S_state;
    reg [1:0]  S_phase;      // 0 = address, 1 = settle, 2 = sample
    reg [4:0]  S_idx;
    reg [7:0]  S_byte0, S_byte1, S_byte2, S_byte3;
    reg [7:0]  S_len_hi, S_len_lo, S_mask, S_flags;
    reg [7:0]  S_magic0, S_magic1, S_key_b, S_notkey_b, S_cycle_reset_b;
    reg [9:0]  S_timeout;
    reg [31:0] S_stale_count;

    wire W_hdr_fields_ok =
        (S_byte0 == 8'h4C) && (S_byte1 == 8'h50) &&
        (S_byte2 == 8'h01) && (S_byte3 == 8'h02) &&
        (S_len_hi == 8'h00) && (S_len_lo == 8'h10) &&
        (S_mask[7] == 1'b0) && ((S_flags & 8'hFE) == 8'h00);
    wire W_payload_ok =
        (S_magic0 == 8'hA5) && (S_magic1 == 8'h4B) &&
        (S_key_b[7:5] == 3'b000) && (S_notkey_b[7:5] == 3'b000) &&
        (S_notkey_b[4:0] == ~S_key_b[4:0]) &&
        (S_key_b[4:0] <= 5'd16) &&
        (S_cycle_reset_b[7:2] == 6'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_state         <= S_IDLE;
            S_phase         <= 1'b0;
            S_idx           <= 5'd0;
            S_byte0         <= 8'd0; S_byte1 <= 8'd0; S_byte2 <= 8'd0; S_byte3 <= 8'd0;
            S_len_hi        <= 8'd0; S_len_lo <= 8'd0; S_mask <= 8'd0; S_flags <= 8'd0;
            S_magic0        <= 8'd0; S_magic1 <= 8'd0;
            S_key_b         <= 8'd0; S_notkey_b <= 8'd0; S_cycle_reset_b <= 8'd0;
            S_timeout       <= 10'd0;
            S_stale_count   <= 32'd0;
            O_rx_addr       <= 10'd0;
            O_rx_release    <= 1'b0;
            O_key_code      <= 5'd0;
            O_key_valid     <= 1'b0;
            O_key_changed   <= 1'b0;
            O_cycle_reset_request <= 1'b0;
            O_key_auto      <= 1'b0;
            O_rx_good_count <= 16'd0;
            O_rx_bad_count  <= 16'd0;
        end else begin
            O_key_changed <= 1'b0;
            O_rx_release  <= 1'b0;
            // Validated heartbeats renew this lease; corrupt traffic cannot.
            if (S_stale_count < P_TIMEOUT_CYCLES - 1) begin
                S_stale_count <= S_stale_count + 1'b1;
            end else if (O_key_valid) begin
                O_key_code <= 5'd0;
                O_key_valid <= 1'b0;
                O_key_changed <= 1'b1;
            end

            case (S_state)
                S_IDLE: begin
                    S_timeout <= 10'd0;
                    if (I_rx_valid && (I_rx_length >= 11'd32) &&
                        (I_rx_length <= 11'd46)) begin
                        O_rx_addr <= 10'd0;
                        S_idx     <= 5'd0;
                        S_phase   <= 1'b0;
                        S_state   <= S_READ;
                    end else if (I_rx_valid) begin
                        O_rx_release <= 1'b1;
                        O_rx_bad_count <= O_rx_bad_count + 1'b1;
                        S_state <= S_DONE;
                    end
                end
                S_READ: begin
                    case (S_phase)
                        2'd0: begin
                            // Address phase.
                            O_rx_addr <= {5'd0, S_idx};
                            S_phase   <= 2'd1;
                            S_timeout <= 10'd0;
                        end
                        2'd1: begin
                            // Settle phase: the link register samples S_idx now,
                            // so the byte is valid in the next cycle.
                            S_phase <= 2'd2;
                        end
                        default: begin
                            // Sample phase.
                            case (S_idx)
                                5'd0:  S_byte0    <= I_rx_data;
                                5'd1:  S_byte1    <= I_rx_data;
                                5'd2:  S_byte2    <= I_rx_data;
                                5'd3:  S_byte3    <= I_rx_data;
                                5'd12: S_len_hi   <= I_rx_data;
                                5'd13: S_len_lo   <= I_rx_data;
                                5'd14: S_mask     <= I_rx_data;
                                5'd15: S_flags    <= I_rx_data;
                                5'd16: S_magic0   <= I_rx_data;
                                5'd17: S_magic1   <= I_rx_data;
                                5'd18: S_key_b    <= I_rx_data;
                                5'd19: S_notkey_b <= I_rx_data;
                                5'd20: S_cycle_reset_b <= I_rx_data;
                                default: ;
                            endcase
                            S_phase <= 2'd0;
                            if (S_idx == 5'd31) begin
                                S_state <= S_FIN;
                            end else begin
                                S_idx <= S_idx + 1'b1;
                            end
                        end
                    endcase
                    if (!I_rx_valid) begin
                        O_rx_release <= 1'b1;
                        O_rx_bad_count <= O_rx_bad_count + 1'b1;
                        S_state <= S_DONE;
                    end
                end
                S_FIN: begin
                    O_rx_release <= 1'b1;
                    if (I_rx_valid && W_hdr_fields_ok && W_payload_ok) begin
                        S_stale_count <= 32'd0;
                        if (O_key_code != S_key_b[4:0])
                            O_key_changed <= 1'b1;
                        O_key_code      <= S_key_b[4:0];
                        O_key_valid     <= (S_key_b[4:0] != 5'd0);
                        O_cycle_reset_request <= S_cycle_reset_b[0];
                        O_key_auto      <= S_cycle_reset_b[1];
                        O_rx_good_count <= O_rx_good_count + 1'b1;
                    end else begin
                        O_rx_bad_count <= O_rx_bad_count + 1'b1;
                    end
                    S_state <= S_DONE;
                end
                S_DONE: begin
                    O_rx_release <= I_rx_valid;
                    if (!I_rx_valid) S_state <= S_IDLE;
                end
                default: S_state <= S_IDLE;
            endcase
        end
    end

endmodule
