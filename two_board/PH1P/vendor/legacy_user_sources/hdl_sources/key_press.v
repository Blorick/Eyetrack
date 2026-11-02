//======================================================================
//  key_press  --  debounced single-button press detector
//
//  Same front end as key_mode_switch (3-stage synchroniser + debounce
//  counter + falling-edge detect) but it only reports a one-cycle
//  press pulse.  Used for the parameter key (KEY3) that steps the
//  brightness level / saturation factor of img_process_96.
//
//  Button is active low (board buttons are pulled up).
//======================================================================
module key_press #(
    parameter integer DEBOUNCE_CYCLES = 500000   // ~21 ms @ 23.809524 MHz
)(
    input  wire I_clk,
    input  wire I_rst_n,
    input  wire I_key_n,        // button input, active low
    output reg  O_press         // one-cycle pulse on each press
);

    reg [2:0]  S_key_sync;
    reg [23:0] S_db_cnt;
    reg        S_key_state;
    reg        S_key_state_1d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_key_sync <= 3'b111;
        else
            S_key_sync <= {S_key_sync[1:0], I_key_n};
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_key_state <= 1'b1;
            S_db_cnt    <= 24'd0;
        end
        else if (S_key_sync[2] == S_key_state) begin
            S_db_cnt <= 24'd0;
        end
        else if (S_db_cnt >= DEBOUNCE_CYCLES) begin
            S_key_state <= S_key_sync[2];
            S_db_cnt    <= 24'd0;
        end
        else begin
            S_db_cnt <= S_db_cnt + 24'd1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_key_state_1d <= 1'b1;
        else
            S_key_state_1d <= S_key_state;
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            O_press <= 1'b0;
        else
            O_press <= S_key_state_1d & ~S_key_state;
    end

endmodule
