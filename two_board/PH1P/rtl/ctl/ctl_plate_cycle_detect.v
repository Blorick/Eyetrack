//=============================================================================
//  ctl_plate_cycle_detect  -- PH1P fresh plate-cycle active-state detector
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk（PH1P pixel/application domain）
//  复位     : I_rst_n，异步低有效；调用方同步释放
//  参数     : P_CLK_HZ（默认 75 MHz）
//  验证状态 : verification/plate_cycle_20261025/model 与本轮仿真证据
//=============================================================================
// A qualified fresh plate sample starts the cycle immediately. Once active,
// the detector expires only after more than 1000 ms without another qualified
// sample. I_reset_request is an independent toggle command; a mismatch is
// handled before the plate sample and is acknowledged in the same clock.
`timescale 1ns / 1ps

module ctl_plate_cycle_detect #(
    parameter integer P_CLK_HZ = 75000000
) (
    input  wire I_clk,                 // pixel/application clock
    input  wire I_rst_n,               // asynchronous active-low reset
    input  wire I_plate_sample,        // one-cycle frame-result submission
    input  wire I_plate_detected,      // fresh detection at the submission
    input  wire I_reset_request,       // reset toggle from the remote control
    output reg  O_cycle_active,        // current inactive/active cycle state
    output reg  O_reset_ack            // accepted reset-request toggle
);

    localparam integer P_LOST_LIMIT_CYCLES = P_CLK_HZ;

    reg [31:0] S_lost_count;
    wire W_plate_positive = I_plate_sample && I_plate_detected;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_lost_count  <= 32'd0;
            O_cycle_active <= 1'b0;
            O_reset_ack    <= 1'b0;
        end else if (I_reset_request != O_reset_ack) begin
            // A reset request has priority over a simultaneous plate sample.
            S_lost_count   <= 32'd0;
            O_cycle_active <= 1'b0;
            O_reset_ack    <= I_reset_request;
        end else if (W_plate_positive) begin
            // Only a frame-qualified fresh result starts or refreshes a cycle.
            S_lost_count   <= 32'd0;
            O_cycle_active <= 1'b1;
        end else if (O_cycle_active) begin
            // Keep the exact 1000 ms boundary active; expire on the next clock.
            if (S_lost_count >= P_LOST_LIMIT_CYCLES) begin
                S_lost_count   <= 32'd0;
                O_cycle_active <= 1'b0;
            end else begin
                S_lost_count   <= S_lost_count + 32'd1;
            end
        end else begin
            S_lost_count <= 32'd0;
        end
    end

endmodule
