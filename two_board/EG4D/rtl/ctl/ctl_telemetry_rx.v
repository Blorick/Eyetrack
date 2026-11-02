// ============================================================================
// ctl_telemetry_rx.v -- EG4D telemetry snapshot consumer for the 0x88B5 link.
//
// The LPR application mailbox exposes a registered one-cycle RAM read.  The
// reader therefore uses address, settle, and sample phases for each of the 32
// application bytes.  A real kind-2 frame is 32 application bytes plus 14
// Ethernet padding bytes, so only I_rx_length == 46 is accepted.
//
// Reset is expected to be asynchronously asserted and synchronously released
// in the caller's 25 MHz clock domain.  ctl_lpr_packet_link already performs
// Ethernet header/FCS checks; this module validates the telemetry application
// contract and always releases a held mailbox.
// ============================================================================
`timescale 1ns / 1ps

module ctl_telemetry_rx (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        I_rx_valid,
    input  wire [10:0] I_rx_length,
    output reg  [9:0]  O_rx_addr,
    input  wire [7:0]  I_rx_data,
    output reg         O_rx_release,
    output wire [15:0] O_ag /*synthesis keep*/,
    output wire [15:0] O_ae /*synthesis keep*/,
    output wire [19:0] O_speed_mmps /*synthesis keep*/,
    output wire [19:0] O_dist_mm /*synthesis keep*/,
    output wire        O_metrics_valid /*synthesis keep*/,
    output wire [7:0]  O_metrics_state /*synthesis keep*/,
    output wire [31:0] O_sample_seq /*synthesis keep*/,
    output wire        O_sample_valid /*synthesis keep*/,
    output wire        O_update /*synthesis keep*/,
    output wire        O_cycle_active,
    output wire        O_cycle_reset_ack
);

    localparam [1:0] S_IDLE = 2'd0;
    localparam [1:0] S_READ = 2'd1;
    localparam [1:0] S_FIN  = 2'd2;
    localparam [1:0] S_DONE = 2'd3;

    reg [1:0] S_state;
    reg [1:0] S_phase;
    reg [5:0] S_idx;

    reg [7:0] S_hdr0;
    reg [7:0] S_hdr1;
    reg [7:0] S_hdr2;
    reg [7:0] S_hdr3;
    reg [7:0] S_seq0;
    reg [7:0] S_seq1;
    reg [7:0] S_seq2;
    reg [7:0] S_seq3;
    reg [7:0] S_reserved0;
    reg [7:0] S_reserved1;
    reg [7:0] S_reserved2;
    reg [7:0] S_reserved3;
    reg [7:0] S_len_hi;
    reg [7:0] S_len_lo;
    reg [7:0] S_header14;
    reg [7:0] S_header15;
    reg [7:0] S_magic0;
    reg [7:0] S_magic1;
    reg [7:0] S_ag_hi;
    reg [7:0] S_ag_lo;
    reg [7:0] S_ae_hi;
    reg [7:0] S_ae_lo;
    reg [7:0] S_speed0;
    reg [7:0] S_speed1;
    reg [7:0] S_speed2;
    reg [7:0] S_dist0;
    reg [7:0] S_dist1;
    reg [7:0] S_dist2;
    reg [7:0] S_valid_byte;
    reg [7:0] S_state_byte;
    reg [7:0] S_tail0;
    reg [7:0] S_tail1;

    (*preserve*) reg [15:0] S_ag_q; //synthesis keep
    (*preserve*) reg [15:0] S_ae_q; //synthesis keep
    (*preserve*) reg [19:0] S_speed_mmps_q; //synthesis keep
    (*preserve*) reg [19:0] S_dist_mm_q; //synthesis keep
    (*preserve*) reg        S_metrics_valid_q; //synthesis keep
    (*preserve*) reg [7:0]  S_metrics_state_q; //synthesis keep
    (*preserve*) reg [31:0] S_sample_seq_q; //synthesis keep
    (*preserve*) reg        S_sample_valid_q; //synthesis keep
    (*preserve*) reg S_update_q; //synthesis keep
    reg S_cycle_active_q;
    reg S_cycle_reset_ack_q;

    assign O_ag             = S_ag_q;
    assign O_ae             = S_ae_q;
    assign O_speed_mmps     = S_speed_mmps_q;
    assign O_dist_mm        = S_dist_mm_q;
    assign O_metrics_valid  = S_metrics_valid_q;
    assign O_metrics_state  = S_metrics_state_q;
    assign O_sample_seq     = S_sample_seq_q;
    assign O_sample_valid   = S_sample_valid_q;
    assign O_update         = S_update_q;
    assign O_cycle_active   = S_cycle_active_q;
    assign O_cycle_reset_ack = S_cycle_reset_ack_q;

    wire W_packet_ok =
        (S_hdr0       == 8'h4C) &&
        (S_hdr1       == 8'h50) &&
        (S_hdr2       == 8'h01) &&
        (S_hdr3       == 8'h02) &&
        (S_reserved0  == 8'h00) &&
        (S_reserved1  == 8'h00) &&
        (S_reserved2  == 8'h00) &&
        (S_reserved3  == 8'h00) &&
        (S_len_hi     == 8'h00) &&
        (S_len_lo     == 8'h10) &&
        (S_header14   == 8'h00) &&
        (S_header15   == 8'h00) &&
        (S_magic0     == 8'hA5) &&
        (S_magic1     == 8'h54) &&
        (S_speed0[7:4] == 4'h0) &&
        (S_dist0[7:4]  == 4'h0) &&
        (S_valid_byte[7:1] == 7'h00) &&
        (S_tail0[7:2] == 6'h00) &&
        (S_tail1      == 8'h00);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_state          <= S_IDLE;
            S_phase          <= 2'd0;
            S_idx            <= 6'd0;
            S_hdr0           <= 8'd0;
            S_hdr1           <= 8'd0;
            S_hdr2           <= 8'd0;
            S_hdr3           <= 8'd0;
            S_seq0           <= 8'd0;
            S_seq1           <= 8'd0;
            S_seq2           <= 8'd0;
            S_seq3           <= 8'd0;
            S_reserved0      <= 8'd0;
            S_reserved1      <= 8'd0;
            S_reserved2      <= 8'd0;
            S_reserved3      <= 8'd0;
            S_len_hi         <= 8'd0;
            S_len_lo         <= 8'd0;
            S_header14       <= 8'd0;
            S_header15       <= 8'd0;
            S_magic0         <= 8'd0;
            S_magic1         <= 8'd0;
            S_ag_hi          <= 8'd0;
            S_ag_lo          <= 8'd0;
            S_ae_hi          <= 8'd0;
            S_ae_lo          <= 8'd0;
            S_speed0         <= 8'd0;
            S_speed1         <= 8'd0;
            S_speed2         <= 8'd0;
            S_dist0          <= 8'd0;
            S_dist1          <= 8'd0;
            S_dist2          <= 8'd0;
            S_valid_byte     <= 8'd0;
            S_state_byte     <= 8'd0;
            S_tail0          <= 8'd0;
            S_tail1          <= 8'd0;
            S_ag_q           <= 16'd0;
            S_ae_q           <= 16'd0;
            S_speed_mmps_q   <= 20'd0;
            S_dist_mm_q      <= 20'd0;
            S_metrics_valid_q<= 1'b0;
            S_metrics_state_q<= 8'd0;
            S_sample_seq_q   <= 32'd0;
            S_sample_valid_q <= 1'b0;
            S_update_q       <= 1'b0;
            S_cycle_active_q <= 1'b0;
            S_cycle_reset_ack_q <= 1'b0;
            O_rx_addr        <= 10'd0;
            O_rx_release     <= 1'b0;
        end else begin
            O_rx_release <= 1'b0;
            S_update_q   <= 1'b0;

            case (S_state)
                S_IDLE: begin
                    if (I_rx_valid) begin
                        if (I_rx_length == 11'd46) begin
                            S_idx   <= 6'd0;
                            S_phase <= 2'd0;
                            S_state <= S_READ;
                        end else begin
                            O_rx_release <= 1'b1;
                            S_state      <= S_DONE;
                        end
                    end
                end

                S_READ: begin
                    if (!I_rx_valid) begin
                        O_rx_release <= 1'b1;
                        S_state      <= S_DONE;
                    end else begin
                        case (S_phase)
                            2'd0: begin
                                O_rx_addr <= {4'd0, S_idx};
                                S_phase   <= 2'd1;
                            end
                            2'd1: begin
                                S_phase <= 2'd2;
                            end
                            default: begin
                                case (S_idx)
                                    6'd0:  S_hdr0       <= I_rx_data;
                                    6'd1:  S_hdr1       <= I_rx_data;
                                    6'd2:  S_hdr2       <= I_rx_data;
                                    6'd3:  S_hdr3       <= I_rx_data;
                                    6'd4:  S_seq0       <= I_rx_data;
                                    6'd5:  S_seq1       <= I_rx_data;
                                    6'd6:  S_seq2       <= I_rx_data;
                                    6'd7:  S_seq3       <= I_rx_data;
                                    6'd8:  S_reserved0  <= I_rx_data;
                                    6'd9:  S_reserved1  <= I_rx_data;
                                    6'd10: S_reserved2  <= I_rx_data;
                                    6'd11: S_reserved3  <= I_rx_data;
                                    6'd12: S_len_hi     <= I_rx_data;
                                    6'd13: S_len_lo     <= I_rx_data;
                                    6'd14: S_header14   <= I_rx_data;
                                    6'd15: S_header15   <= I_rx_data;
                                    6'd16: S_magic0     <= I_rx_data;
                                    6'd17: S_magic1     <= I_rx_data;
                                    6'd18: S_ag_hi      <= I_rx_data;
                                    6'd19: S_ag_lo      <= I_rx_data;
                                    6'd20: S_ae_hi      <= I_rx_data;
                                    6'd21: S_ae_lo      <= I_rx_data;
                                    6'd22: S_speed0     <= I_rx_data;
                                    6'd23: S_speed1     <= I_rx_data;
                                    6'd24: S_speed2     <= I_rx_data;
                                    6'd25: S_dist0      <= I_rx_data;
                                    6'd26: S_dist1      <= I_rx_data;
                                    6'd27: S_dist2      <= I_rx_data;
                                    6'd28: S_valid_byte <= I_rx_data;
                                    6'd29: S_state_byte <= I_rx_data;
                                    6'd30: S_tail0      <= I_rx_data;
                                    6'd31: S_tail1      <= I_rx_data;
                                    default: begin
                                    end
                                endcase
                                S_phase <= 2'd0;
                                if (S_idx == 6'd31) begin
                                    S_state <= S_FIN;
                                end else begin
                                    S_idx <= S_idx + 1'b1;
                                end
                            end
                        endcase
                    end
                end

                S_FIN: begin
                    // Invalid application data is consumed just like a valid
                    // frame, so it cannot hold the LPR mailbox indefinitely.
                    O_rx_release <= 1'b1;
                    if (I_rx_valid && W_packet_ok) begin
                        S_ag_q            <= {S_ag_hi, S_ag_lo};
                        S_ae_q            <= {S_ae_hi, S_ae_lo};
                        S_speed_mmps_q   <= {S_speed0[3:0], S_speed1, S_speed2};
                        S_dist_mm_q      <= {S_dist0[3:0], S_dist1, S_dist2};
                        S_metrics_valid_q<= S_valid_byte[0];
                        S_metrics_state_q<= S_state_byte;
                        S_sample_seq_q   <= {S_seq0, S_seq1, S_seq2, S_seq3};
                        S_sample_valid_q <= 1'b1;
                        S_update_q       <= 1'b1;
                        S_cycle_active_q <= S_tail0[0];
                        S_cycle_reset_ack_q <= S_tail0[1];
                    end
                    S_state <= S_DONE;
                end

                S_DONE: begin
                    if (!I_rx_valid)
                        S_state <= S_IDLE;
                end

                default: begin
                    S_state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
