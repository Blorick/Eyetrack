//======================================================================
//  key_mode_switch  --  debounced push-button N-mode cycler  (v1.7)
//
//  v1.6 generalisation of the v1.5 module: instead of toggling one bit
//  between "colour" and "edge", each press advances a mode counter
//  through N_MODES values and wraps around at the end.
//
//  v1.7 adds the requested "hold to keep switching" behaviour:
//     * short press  -> advance exactly one mode (v1.6 behaviour)
//     * keep holding -> after HOLD_REPEAT_DELAY_CYCLES the counter keeps
//                       advancing once every HOLD_REPEAT_PERIOD_CYCLES
//                       until the button is released
//  Hold-repeat is disabled by default (HOLD_REPEAT_EN = 0), so the v1.6
//  behaviour and its regression testbench are preserved unchanged.
//
//  Button: KEY1 (HX1P35A board pin D5, active low, internal pull-up).
//
//  Press handling:
//     1) 3-stage register synchroniser for the asynchronous button
//     2) debounce counter: the level must be stable for DEBOUNCE_CYCLES
//        before it is accepted
//     3) falling-edge detect on the accepted level (low active)
//     4) mode counter advance + wrap; holding the button advances once
//        (or repeatedly when hold-repeat is enabled)
//
//  O_mode is a level signal.  Crossing into another clock domain only
//  needs two registers in the destination domain.
//  O_mode_pulse is a one-cycle pulse at the moment of the switch.
//======================================================================
module key_mode_switch #(
    parameter integer DEBOUNCE_CYCLES           = 500000,  // ~21 ms @ 23.809524 MHz
    parameter integer N_MODES                   = 17,      // number of selectable modes
    parameter integer INIT_MODE                 = 0,       // mode after reset
    parameter integer MODE_W                    = 5,       // width of O_mode
    // ---- v1.7 hold-to-repeat (KEY1) ----
    parameter integer HOLD_REPEAT_EN            = 0,       // 1: keep switching while held
    parameter integer HOLD_REPEAT_DELAY_CYCLES  = 11904762,// ~0.5 s @ 23.809524 MHz
    parameter integer HOLD_REPEAT_PERIOD_CYCLES = 2380952  // ~0.1 s @ 23.809524 MHz
)(
    input  wire              I_clk,
    input  wire              I_rst_n,
    input  wire              I_key_n,        // button input, active low
    output reg [MODE_W-1:0]  O_mode,         // current mode index
    output reg               O_mode_pulse    // one-cycle pulse on change
);

    //--------------------------------------------------------------
    // 1. synchroniser
    //--------------------------------------------------------------
    reg [2:0] S_key_sync;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_key_sync <= 3'b111;
        else
            S_key_sync <= {S_key_sync[1:0], I_key_n};
    end

    //--------------------------------------------------------------
    // 2. debounce
    //--------------------------------------------------------------
    reg  [23:0] S_db_cnt;
    reg         S_key_state;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_key_state <= 1'b1;          // released at power-up
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

    //--------------------------------------------------------------
    // 3. press edge (active low: 1 -> 0)
    //--------------------------------------------------------------
    reg S_key_state_1d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_key_state_1d <= 1'b1;
        else
            S_key_state_1d <= S_key_state;
    end

    wire S_press = S_key_state_1d & ~S_key_state;
    wire S_held  = ~S_key_state;              // low active: 1 while pressed

    //--------------------------------------------------------------
    // 3b. hold-to-repeat timing (v1.7)
    //     S_delay_cnt counts the hold time up to the repeat delay.
    //     Once the delay has elapsed, S_rep_cnt free-runs with the
    //     repeat period and S_rep_tick marks the first cycle of every
    //     period.  Both counters are cleared when the key is released,
    //     so every fresh press waits the full delay before repeating.
    //--------------------------------------------------------------
    reg [31:0] S_delay_cnt;
    reg [31:0] S_rep_cnt;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_delay_cnt <= 32'd0;
            S_rep_cnt   <= 32'd0;
        end
        else if (!S_held) begin
            S_delay_cnt <= 32'd0;
            S_rep_cnt   <= 32'd0;
        end
        else begin
            if (S_delay_cnt < HOLD_REPEAT_DELAY_CYCLES)
                S_delay_cnt <= S_delay_cnt + 32'd1;

            if (S_delay_cnt >= HOLD_REPEAT_DELAY_CYCLES) begin
                if (S_rep_cnt >= HOLD_REPEAT_PERIOD_CYCLES - 1)
                    S_rep_cnt <= 32'd0;
                else
                    S_rep_cnt <= S_rep_cnt + 32'd1;
            end
        end
    end

    wire S_rep_tick = (HOLD_REPEAT_EN != 0) && S_held &&
                      (S_delay_cnt >= HOLD_REPEAT_DELAY_CYCLES) &&
                      (S_rep_cnt == 32'd0);

    wire S_advance = S_press | S_rep_tick;

    //--------------------------------------------------------------
    // 4. mode counter with wrap
    //--------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            O_mode <= INIT_MODE;
        else if (S_advance) begin
            if (O_mode == (N_MODES - 1))
                O_mode <= {MODE_W{1'b0}};
            else
                O_mode <= O_mode + 1'b1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            O_mode_pulse <= 1'b0;
        else
            O_mode_pulse <= S_advance;
    end

endmodule
