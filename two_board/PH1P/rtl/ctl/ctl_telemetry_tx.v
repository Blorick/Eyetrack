// ============================================================================
// ctl_telemetry_tx.v -- PH1P telemetry snapshot producer for the 0x88B5 link.
//
// The application packet is the existing LPR kind-2, 32-byte application
// shape.  AG/AE are sampled in the 23.809524 MHz configuration domain through
// a request/acknowledge toggle handshake.  The acknowledged configuration hold
// bus and the application-domain metrics are then captured together before any
// mailbox byte is written.
//
// Reset inputs are expected to be asynchronously asserted and synchronously
// released in their respective clock domains by the caller.
// ============================================================================
`timescale 1ns / 1ps

module ctl_telemetry_tx #(
    parameter integer P_REFRESH_CYCLES = 7500000
) (
    input  wire        clk_app,
    input  wire        rst_app_n,
    input  wire        clk_cfg,
    input  wire        rst_cfg_n,
    input  wire [15:0] I_cfg_ag,
    input  wire [15:0] I_cfg_ae,
    input  wire [19:0] I_app_speed_mmps,
    input  wire [19:0] I_app_dist_mm,
    input  wire        I_app_metrics_valid,
    input  wire [7:0]  I_app_metrics_state,
    input  wire        I_app_cycle_active,
    input  wire        I_app_cycle_reset_ack,
    input  wire        I_tx_busy,
    output reg  [9:0]  O_wr_addr,
    output reg  [7:0]  O_wr_data,
    output reg         O_wr_en,
    output reg         O_commit,
    output reg  [10:0] O_length
);

    localparam [2:0] S_IDLE          = 3'd0;
    localparam [2:0] S_WAIT_CFG      = 3'd1;
    localparam [2:0] S_WAIT_WRITE    = 3'd2;
    localparam [2:0] S_WRITE         = 3'd3;
    localparam [2:0] S_COMMIT        = 3'd4;
    localparam [2:0] S_WAIT_BUSY_UP  = 3'd5;
    localparam [2:0] S_WAIT_BUSY_DN  = 3'd6;

    // The request is an event, while S_cfg_hold is a bundled stable data bus.
    // It is not overwritten until the next request, after the previous ack.
    reg S_request_toggle_app;
    reg S_request_meta_cfg;
    reg S_request_sync_cfg;
    reg S_request_seen_cfg;
    reg S_ack_toggle_cfg;
    reg S_ack_meta_app;
    reg S_ack_sync_app;
    reg S_ack_seen_app;

    reg [31:0] S_cfg_hold; // synthesis keep

    reg [31:0] S_refresh_cnt;
    reg        S_refresh_due;
    reg [2:0]  S_state;
    reg [5:0]  S_byte_idx;

    reg [31:0] S_sample_seq;
    reg [31:0] S_snapshot_seq; // synthesis keep
    reg [15:0] S_snapshot_ag; // synthesis keep
    reg [15:0] S_snapshot_ae; // synthesis keep
    reg [19:0] S_snapshot_speed; // synthesis keep
    reg [19:0] S_snapshot_dist; // synthesis keep
    reg        S_snapshot_metrics_valid; // synthesis keep
    reg [7:0]  S_snapshot_metrics_state; // synthesis keep
    reg        S_snapshot_cycle_active; // synthesis keep
    reg        S_snapshot_cycle_reset_ack; // synthesis keep
    reg        S_cycle_sent_active;
    reg        S_cycle_sent_reset_ack;

    wire W_cycle_state_changed =
        (I_app_cycle_active != S_cycle_sent_active) ||
        (I_app_cycle_reset_ack != S_cycle_sent_reset_ack);

    // Application-byte encoder.  Multi-byte fields are sent network-order.
    function [7:0] f_app_byte;
        input [5:0] index;
        begin
            case (index)
                6'd0:  f_app_byte = 8'h4C;
                6'd1:  f_app_byte = 8'h50;
                6'd2:  f_app_byte = 8'h01;
                6'd3:  f_app_byte = 8'h02;
                6'd4:  f_app_byte = S_snapshot_seq[31:24];
                6'd5:  f_app_byte = S_snapshot_seq[23:16];
                6'd6:  f_app_byte = S_snapshot_seq[15:8];
                6'd7:  f_app_byte = S_snapshot_seq[7:0];
                6'd12: f_app_byte = 8'h00;
                6'd13: f_app_byte = 8'h10;
                6'd16: f_app_byte = 8'hA5;
                6'd17: f_app_byte = 8'h54;
                6'd18: f_app_byte = S_snapshot_ag[15:8];
                6'd19: f_app_byte = S_snapshot_ag[7:0];
                6'd20: f_app_byte = S_snapshot_ae[15:8];
                6'd21: f_app_byte = S_snapshot_ae[7:0];
                6'd22: f_app_byte = {4'h0, S_snapshot_speed[19:16]};
                6'd23: f_app_byte = S_snapshot_speed[15:8];
                6'd24: f_app_byte = S_snapshot_speed[7:0];
                6'd25: f_app_byte = {4'h0, S_snapshot_dist[19:16]};
                6'd26: f_app_byte = S_snapshot_dist[15:8];
                6'd27: f_app_byte = S_snapshot_dist[7:0];
                6'd28: f_app_byte = {7'd0, S_snapshot_metrics_valid};
                6'd29: f_app_byte = S_snapshot_metrics_state;
                // A554 byte30 carries reset acknowledgement in bit1 and active in bit0.
                6'd30: f_app_byte = {6'd0, S_snapshot_cycle_reset_ack,
                                      S_snapshot_cycle_active};
                6'd31: f_app_byte = 8'h00;
                default: f_app_byte = 8'h00;
            endcase
        end
    endfunction

    // Configuration-domain side of the bundled request/acknowledge transfer.
    // The request toggle is only changed after the previous transaction has
    // completed, so the held bus is stable for both synchronizer delays.
    always @(posedge clk_cfg or negedge rst_cfg_n) begin
        if (!rst_cfg_n) begin
            S_request_meta_cfg <= 1'b0;
            S_request_sync_cfg <= 1'b0;
            S_request_seen_cfg <= 1'b0;
            S_cfg_hold         <= 32'd0;
            S_ack_toggle_cfg   <= 1'b0;
        end else begin
            S_request_meta_cfg <= S_request_toggle_app;
            S_request_sync_cfg <= S_request_meta_cfg;
            if (S_request_sync_cfg != S_request_seen_cfg) begin
                S_request_seen_cfg <= S_request_sync_cfg;
                S_cfg_hold         <= {I_cfg_ag, I_cfg_ae};
                S_ack_toggle_cfg   <= S_request_sync_cfg;
            end
        end
    end

    // Application-domain producer and the return acknowledge synchronizer.
    // There is deliberately no timeout or alternate transaction path: a
    // missing acknowledge or a stuck busy signal remains visible as a stall.
    always @(posedge clk_app or negedge rst_app_n) begin
        if (!rst_app_n) begin
            S_request_toggle_app    <= 1'b0;
            S_ack_meta_app          <= 1'b0;
            S_ack_sync_app          <= 1'b0;
            S_ack_seen_app          <= 1'b0;
            S_refresh_cnt           <= P_REFRESH_CYCLES - 1;
            S_refresh_due           <= 1'b0;
            S_state                 <= S_IDLE;
            S_byte_idx              <= 6'd0;
            S_sample_seq            <= 32'd0;
            S_snapshot_seq          <= 32'd0;
            S_snapshot_ag           <= 16'd0;
            S_snapshot_ae           <= 16'd0;
            S_snapshot_speed        <= 20'd0;
            S_snapshot_dist         <= 20'd0;
            S_snapshot_metrics_valid<= 1'b0;
            S_snapshot_metrics_state<= 8'd0;
            S_snapshot_cycle_active  <= 1'b0;
            S_snapshot_cycle_reset_ack <= 1'b0;
            S_cycle_sent_active       <= 1'b0;
            S_cycle_sent_reset_ack    <= 1'b0;
            O_wr_addr               <= 10'd0;
            O_wr_data               <= 8'd0;
            O_wr_en                 <= 1'b0;
            O_commit                <= 1'b0;
            O_length                <= 11'd32;
        end else begin
            S_ack_meta_app <= S_ack_toggle_cfg;
            S_ack_sync_app <= S_ack_meta_app;

            O_wr_en  <= 1'b0;
            O_commit <= 1'b0;

            if (S_refresh_cnt == 32'd0) begin
                S_refresh_cnt <= P_REFRESH_CYCLES - 1;
                S_refresh_due <= 1'b1;
            end else begin
                S_refresh_cnt <= S_refresh_cnt - 1'b1;
            end

            if (S_ack_sync_app != S_ack_seen_app) begin
                S_ack_seen_app <= S_ack_sync_app;
                if (S_state == S_WAIT_CFG) begin
                    // The cfg hold bus has been stable since the cfg-domain
                    // capture, and the returning ack has two app-domain
                    // synchronizer stages.  Capture it with app metrics as
                    // one coherent packet snapshot.
                    S_snapshot_seq           <= S_sample_seq;
                    S_snapshot_ag            <= S_cfg_hold[31:16];
                    S_snapshot_ae            <= S_cfg_hold[15:0];
                    S_snapshot_speed         <= I_app_speed_mmps;
                    S_snapshot_dist          <= I_app_dist_mm;
                    S_snapshot_metrics_valid <= I_app_metrics_valid;
                    S_snapshot_metrics_state <= I_app_metrics_state;
                    S_snapshot_cycle_active  <= I_app_cycle_active;
                    S_snapshot_cycle_reset_ack <= I_app_cycle_reset_ack;
                    S_byte_idx               <= 6'd0;
                    S_state                  <= S_WAIT_WRITE;
                end
            end

            case (S_state)
                S_IDLE: begin
                    if ((S_refresh_due || W_cycle_state_changed) && !I_tx_busy) begin
                        S_refresh_due        <= 1'b0;
                        S_request_toggle_app <= ~S_request_toggle_app;
                        S_state              <= S_WAIT_CFG;
                    end
                end

                S_WAIT_CFG: begin
                end

                S_WAIT_WRITE: begin
                    if (!I_tx_busy)
                        S_state <= S_WRITE;
                end

                S_WRITE: begin
                    // A busy link never sees a mailbox write pulse.  The
                    // address and byte remain held while back-pressured.
                    if (!I_tx_busy) begin
                        O_wr_addr <= {4'd0, S_byte_idx};
                        O_wr_data <= f_app_byte(S_byte_idx);
                        O_wr_en   <= 1'b1;
                        if (S_byte_idx == 6'd31) begin
                            S_state <= S_COMMIT;
                        end else begin
                            S_byte_idx <= S_byte_idx + 1'b1;
                        end
                    end
                end

                S_COMMIT: begin
                    if (!I_tx_busy) begin
                        O_length <= 11'd32;
                        O_commit <= 1'b1;
                        S_state  <= S_WAIT_BUSY_UP;
                    end
                end

                S_WAIT_BUSY_UP: begin
                    if (I_tx_busy)
                        S_state <= S_WAIT_BUSY_DN;
                end

                S_WAIT_BUSY_DN: begin
                    if (!I_tx_busy) begin
                        S_cycle_sent_active    <= S_snapshot_cycle_active;
                        S_cycle_sent_reset_ack <= S_snapshot_cycle_reset_ack;
                        S_sample_seq <= S_sample_seq + 1'b1;
                        S_state      <= S_IDLE;
                    end
                end

                default: begin
                    S_state <= S_IDLE;
                end
            endcase
        end
    end

endmodule
