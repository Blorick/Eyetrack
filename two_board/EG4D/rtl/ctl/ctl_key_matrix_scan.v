// ============================================================================
// ctl_key_matrix_scan.v -- EG4D 4x4 key-matrix scanner (dual orientation).
//
// Board contract (MLK-S200 hardware manual page 18, "13: matrix switch"):
//   group A pins 122/109/104/91, group B pins 119/118/115/114, 16 keys.
//   KEY1..KEY4 sit on (A0, B0..B3), KEY5..KEY8 on (A1, B0..B3),
//   KEY9..KEY12 on (A2, B0..B3), KEY13..KEY16 on (A3, B0..B3).
//
// The manual gives pin numbers only, not the drive/sense direction.  The board
// matrix is therefore scanned in both orientations and whichever orientation
// produces a contact is used:
//   phase A (0..3): drive group A low one line at a time, sense group B.
//   phase B (4..7): drive group B low one line at a time, sense group A.
// Both groups carry FPGA weak pull-ups and are driven only during their own
// phase (unselected lines of the driving group are released), so a single
// key press reads as a unique (row, column) pair in either orientation.
//
// Outputs the debounced current 5-bit key code 1..16 (0 = released or ambiguous), a valid flag, a change
// pulse and two raw-activity flags for board diagnostics.
// ============================================================================
`timescale 1ns / 1ps

module ctl_key_matrix_scan #(
    // Number of 25 MHz cycles each of the 8 scan phases is held.
    parameter integer P_PHASE_CYCLES = 1250,
    // Consecutive identical full scans required before a code is accepted.
    parameter integer P_DEBOUNCE     = 24
)(
    input  wire        clk,            // 25 MHz application clock
    input  wire        rst_n,
    inout  wire [3:0]  IO_key_row,     // group A, pins 122/109/104/91
    inout  wire [3:0]  IO_key_col,     // group B, pins 119/118/115/114
    output reg  [4:0]  O_key_code,     // 1..16, 0 = released or ambiguous
    output reg         O_key_valid,    // exactly one key held
    output reg         O_key_changed,  // one-cycle pulse when O_key_code changes
    output reg  [1:0]  O_raw_act       // {phase-B activity, phase-A activity}
);

    reg  [2:0]              S_phase;
    reg  [15:0]             S_dwell;
    reg  [15:0]             S_matrix;
    reg  [15:0]             S_matrix_next;
    reg  [4:0]              S_code_pending;
    reg  [4:0]              S_code_settled;
    reg  [7:0]              S_debounce_cnt;
    reg  [4:0]              S_key_latched;
    reg  [1:0]              S_act;
    reg                     S_row_oe;
    reg  [3:0]              S_row_val;
    reg                     S_col_oe;
    reg  [3:0]              S_col_val;

    // Selected line is driven low, the other lines of the driving group are
    // released.  The sensing group is released to the FPGA pull-ups.
    always @(*) begin
        if (S_phase < 3'd4) begin
            S_row_oe  = 1'b1;
            S_col_oe  = 1'b0;
            S_row_val = 4'b1111 ^ (4'b0001 << S_phase[1:0]);
            S_col_val = 4'b1111;
        end else begin
            S_row_oe  = 1'b0;
            S_col_oe  = 1'b1;
            S_row_val = 4'b1111;
            S_col_val = 4'b1111 ^ (4'b0001 << S_phase[1:0]);
        end
    end

    // Open-drain scanning: no two pressed keys can short a high driver to low.
    genvar key_pin;
    generate for (key_pin=0; key_pin<4; key_pin=key_pin+1) begin : g_key_pins
        assign IO_key_row[key_pin] = (S_row_oe && !S_row_val[key_pin]) ? 1'b0 : 1'bz;
        assign IO_key_col[key_pin] = (S_col_oe && !S_col_val[key_pin]) ? 1'b0 : 1'bz;
    end endgenerate

    (* async_reg = "true" *) reg [3:0] S_row_meta, S_row_sync;
    (* async_reg = "true" *) reg [3:0] S_col_meta, S_col_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_row_meta <= 4'hf; S_row_sync <= 4'hf;
            S_col_meta <= 4'hf; S_col_sync <= 4'hf;
        end else begin
            S_row_meta <= IO_key_row; S_row_sync <= S_row_meta;
            S_col_meta <= IO_key_col; S_col_sync <= S_col_meta;
        end
    end

    // Contact sense in the current phase.
    wire [3:0] W_row_pressed = ~S_row_sync;
    wire [3:0] W_col_pressed = ~S_col_sync;

    // Merge the fresh contact into the 16-bit matrix.
    //   key index k (0-based) = row * 4 + column.
    integer mi;
    always @(*) begin
        S_matrix_next = S_matrix;
        if (S_phase < 3'd4) begin
            S_matrix_next[S_phase[1:0]*4 +: 4] =
                S_matrix[S_phase[1:0]*4 +: 4] | W_col_pressed;
        end else begin
            for (mi = 0; mi < 4; mi = mi + 1) begin
                if (W_row_pressed[mi])
                    S_matrix_next[mi*4 + S_phase[1:0]] = 1'b1;
            end
        end
    end

    // Priority encode the 16-bit matrix into a key code 1..16 (0 = none).
    function [4:0] f_key_code;
        input [15:0] matrix;
        integer i;
        reg found;
        begin
            f_key_code = 5'd0;
            found = 1'b0;
            for (i = 15; i >= 0; i = i - 1) begin
                if (!found && matrix[i]) begin
                    f_key_code = {1'b0, i[3:0]} + 5'd1;
                    found = 1'b1;
                end
            end
        end
    endfunction

    wire W_one_key = (S_matrix_next != 16'd0) &&
                    ((S_matrix_next & (S_matrix_next - 16'd1)) == 16'd0);
    wire [4:0] W_code_fresh = W_one_key ? f_key_code(S_matrix_next) : 5'd0;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            S_phase         <= 3'd0;
            S_dwell         <= 16'd0;
            S_matrix        <= 16'd0;
            S_code_pending  <= 5'd0;
            S_code_settled  <= 5'd0;
            S_debounce_cnt  <= 8'd0;
            S_key_latched   <= 5'd0;
            S_act           <= 2'd0;
            O_key_code      <= 5'd0;
            O_key_valid     <= 1'b0;
            O_key_changed   <= 1'b0;
            O_raw_act       <= 2'd0;
        end else begin
            O_key_changed <= 1'b0;

            // Raw contact activity flags (accumulated over one full scan).
            if (S_dwell == P_PHASE_CYCLES - 1) begin
                if (S_phase < 3'd4) begin
                    if (|W_col_pressed) S_act[0] <= 1'b1;
                end else begin
                    if (|W_row_pressed) S_act[1] <= 1'b1;
                end
            end

            if (S_dwell == P_PHASE_CYCLES - 1) begin
                S_dwell <= 16'd0;
                if (S_phase == 3'd7) begin
                    S_phase <= 3'd0;
                    S_matrix <= 16'd0;   // start the next scan clean
                    if (W_code_fresh == S_code_pending) begin
                        if (S_debounce_cnt == P_DEBOUNCE - 1)
                            S_code_settled <= W_code_fresh;
                        else
                            S_debounce_cnt <= S_debounce_cnt + 1'b1;
                    end else begin
                        S_code_pending <= W_code_fresh;
                        S_debounce_cnt <= 8'd0;
                    end
                    O_raw_act <= S_act;
                    S_act     <= 2'd0;
                end else begin
                    S_matrix <= S_matrix_next;
                    S_phase  <= S_phase + 3'd1;
                end
            end else begin
                S_dwell <= S_dwell + 1'b1;
            end

            // Both press and release are state changes, including multi-key rejection.
            if (S_code_settled != S_key_latched) begin
                S_key_latched <= S_code_settled;
                O_key_code    <= S_code_settled;
                O_key_valid   <= (S_code_settled != 5'd0);
                O_key_changed <= 1'b1;
            end
        end
    end

endmodule
