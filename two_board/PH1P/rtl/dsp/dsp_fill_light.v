//=============================================================================
//  dsp_fill_light  --  WS2812 automatic fill light for dark scenes   (v1.13.2)
//  Target panel: 8 x 8 WS2812 (64 LEDs)
//
//  WHY
//  ---------------------------------------------------------------------
//  In a dark scene the AE loop (ae_auto_loop) pushes exposure to its limit and
//  the picture is still too dark. Nothing more can come from the sensor, so an
//  external light source is the only way out. This module drives the panel as
//  an automatic fill light: the darker the scene, the bigger the glow, growing
//  as a diamond from the centre of the panel.
//
//  INPUT
//  ---------------------------------------------------------------------
//  I_y_avg = frame-average luma from alg_luma_stat (0..255), already synced
//  into the 24M control domain by top_contest as S_y_avg_m.
//      y_avg >= Y_HI   bright enough -> panel off
//      y_avg <= Y_LO   very dark     -> whole panel
//      in between      8 linear steps
//
//  OPTICAL FEEDBACK
//  ---------------------------------------------------------------------
//  The panel lights the scene, the scene brightens, a proportional law dims
//  the panel, the scene darkens ... a closed optical loop that flickers.
//  Two mitigations are built in:
//    1) luma IIR (tau ~64 frames) so frame-rate changes are ignored;
//    2) the level moves at most one step per 200 ms.
//  The real fix is mechanical: keep the panel OUT of the camera's field of
//  view. Pointing the camera straight at the panel cannot be stabilised.
//
//  PANEL GEOMETRY (8 x 8, 64 LEDs)
//  ---------------------------------------------------------------------
//  Chain order on a typical 8x8 module is row-major with a SERPENTINE
//  (zig-zag) direction: row 0 left->right, row 1 right->left, ...
//      row = idx >> 3        col_in_row = idx & 7
//      x   = row[0] ? (7 - col_in_row) : col_in_row
//      y   = row
//  Set SERPENTINE = 0 if your panel is straight row-major.
//
//      centre (3,3)      d = |x-3| + |y-3|      range 0..8
//      R = level                                (level 0..8)
//      led on  <=>  level != 0 and d <= R
//
//  No frame buffer: the colour is a constant and on/off is computed on the fly
//  from (idx, level), so this costs 0 BRAM.
//
//  WS2812 TIMING (WS2812E..25.pdf)
//  ---------------------------------------------------------------------
//      bit period T >= 1.20us    T0H 0.2/0.3/0.4us
//      T1H 0.65/0.75/1.0us       T0L >= 0.8us     T1L >= 0.2us
//      reset > 200us             24 bit per led, GRB, MSB first
//
//  ***** CYCLE COUNTS ARE EXPLICIT PARAMETERS, NOT COMPUTED *****
//  The control clock is S_24m_clk = 23.809524 MHz (50MHz x 10/21), NOT
//  24.000 MHz. A "cycles-per-microsecond" formula rounds to 23 and T becomes
//  1.176us, below the 1.20us minimum. So the counts are hard-coded:
//      1 cycle = 42.0 ns
//      CYC_T   = 30  ->  1.260us   (spec >= 1.20  OK)
//      CYC_T0H =  7  ->  0.294us   (0.2 ~ 0.4     OK)
//      CYC_T1H = 18  ->  0.756us   (0.65 ~ 1.0    OK)
//
//  ***** PANEL POWER *****
//  64 LEDs at white=128 draw about 0.8A at 5V, so the 5V/3A module has plenty
//  of headroom. The panel VDD must still go through one silicon diode
//  (1N5408): panel ~4.2V makes VIH ~2.7V, so the FPGA's 3.3V data has margin.
//  Without the diode only the first few LEDs respond (VIH 3.1V vs 3.3V data).
//=============================================================================
module dsp_fill_light #(
    parameter integer N_LED      = 64,          // 8 x 8
    parameter integer SERPENTINE = 1,           // 1 = zig-zag rows
    parameter [7:0]   Y_HI       = 8'd60,       // at/above this: panel off
    parameter [7:0]   Y_LO       = 8'd30,       // at/below this: full panel
    parameter [3:0]   MAX_LV     = 4'd8,        // max level
    parameter [7:0]   LED_WHITE  = 8'd128,      // per-LED brightness
    parameter integer GAP_MS     = 10,          // inter-frame gap
    parameter integer CYC_T      = 30,          // see header
    parameter integer CYC_T0H    = 7,
    parameter integer CYC_T1H    = 18,
    parameter integer CLK_HZ     = 23809524
)(
    input  wire        I_clk,          // S_24m_clk
    input  wire        I_rst_n,
    input  wire [7:0]  I_y_avg,        // frame average luma
    input  wire        I_y_valid,      // unused (kept for compatibility)
    output reg         O_ws,           // -> WS2812 panel DIN
    output wire [3:0]  O_level         // current level (debug / OSD)
);

    //-------------------------------------------------------------------------
    // 0) constants
    //-------------------------------------------------------------------------
    localparam [31:0] GAP_CYC = (CLK_HZ / 1000) * GAP_MS;
    localparam [7:0]  STEP    = (Y_HI - Y_LO) / 8;

    //-------------------------------------------------------------------------
    // 1) luma -> level, with IIR (8.6 fixed point) + slow step
    //    The fractional accumulator matters: with a plain 8-bit value the
    //    update (y - yf) >> 6 rounds to zero whenever |y - yf| < 64 and the
    //    filter would freeze.
    //-------------------------------------------------------------------------
    reg  [7:0]  S_y_s1, S_y_s2;
    reg  [13:0] S_yf;              // 8.6 fixed point, integer part S_yf[13:6]
    reg  [3:0]  S_lv;
    reg  [23:0] S_lv_tick;

    wire [13:0] W_yin = {S_y_s2, 6'b000000};   // incoming luma * 64
    wire [7:0]  W_yf  = S_yf[13:6];

    wire [3:0] W_tgt =
        (W_yf >= Y_HI)                ? 4'd0 :
        (W_yf <= Y_LO)                ? MAX_LV :
        (W_yf <= (Y_LO + STEP*1))     ? 4'd7 :
        (W_yf <= (Y_LO + STEP*2))     ? 4'd6 :
        (W_yf <= (Y_LO + STEP*3))     ? 4'd5 :
        (W_yf <= (Y_LO + STEP*4))     ? 4'd4 :
        (W_yf <= (Y_LO + STEP*5))     ? 4'd3 :
        (W_yf <= (Y_LO + STEP*6))     ? 4'd2 :
                                        4'd1 ;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_y_s1    <= 8'd255;
            S_y_s2    <= 8'd255;
            S_yf      <= 14'd6400;          // 100 * 64
            S_lv      <= 4'd0;
            S_lv_tick <= 24'd0;
        end
        else begin
            S_y_s1 <= I_y_avg;
            S_y_s2 <= S_y_s1;

            // yf += (yin - yf)/64
            if(W_yin > S_yf)      S_yf <= S_yf + ((W_yin - S_yf) >> 6);
            else if(W_yin < S_yf) S_yf <= S_yf - ((S_yf - W_yin) >> 6);

            // one level step per 200 ms
            if(S_lv_tick >= (CLK_HZ / 5)) begin
                S_lv_tick <= 24'd0;
                if(S_lv < W_tgt)      S_lv <= S_lv + 4'd1;
                else if(S_lv > W_tgt) S_lv <= S_lv - 4'd1;
            end
            else begin
                S_lv_tick <= S_lv_tick + 24'd1;
            end
        end
    end

    assign O_level = S_lv;

    //-------------------------------------------------------------------------
    // 2) radius == level  (d ranges 0..8 on an 8x8 panel)
    //-------------------------------------------------------------------------
    wire [5:0] W_rad = {2'd0, S_lv};
    wire       W_any = (S_lv != 4'd0);

    //-------------------------------------------------------------------------
    // 3) per-LED predicate / payload
    //    idx 0..63, row = idx>>3, col_in_row = idx&7
    //-------------------------------------------------------------------------
    function automatic led_on;
        input [6:0] idx;
        input [5:0] rad;
        reg   [2:0] row;
        reg   [2:0] col;
        reg   [3:0] x;
        reg   [3:0] y;
        reg   [3:0] dx;
        reg   [3:0] dy;
        reg   [4:0] d;
        begin
            row = idx[5:3];
            col = idx[2:0];
            x   = (SERPENTINE != 0 && row[0]) ? (4'd7 - {1'd0, col}) : {1'd0, col};
            y   = {1'd0, row};
            dx  = (x >= 4'd3) ? (x - 4'd3) : (4'd3 - x);
            dy  = (y >= 4'd3) ? (y - 4'd3) : (4'd3 - y);
            d   = {1'b0, dx} + {1'b0, dy};
            led_on = (d <= rad);
        end
    endfunction

    function [23:0] led_grb;
        input [6:0] idx;
        input [5:0] rad;
        input       any;
        reg   [7:0] v;
        begin
            v = (any && led_on(idx, rad)) ? LED_WHITE : 8'd0;
            led_grb = {v, v, v};                    // GRB, all three equal
        end
    endfunction

    //-------------------------------------------------------------------------
    // 4) WS2812 bit stream
    //-------------------------------------------------------------------------
    localparam S_IDLE = 2'd0,
               S_BIT  = 2'd1,
               S_GAP  = 2'd2;

    reg  [1:0]  S_st;
    reg  [31:0] S_cy;
    reg  [4:0]  S_bit;
    reg  [6:0]  S_led;
    reg  [23:0] S_sr;
    reg  [31:0] S_gap;

    wire        W_this_bit = S_sr[5'd23 - S_bit];
    wire [31:0] W_high_cy  = W_this_bit ? CYC_T1H : CYC_T0H;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_st  <= S_IDLE;
            S_cy  <= 32'd0;
            S_bit <= 5'd0;
            S_led <= 7'd0;
            S_sr  <= 24'd0;
            S_gap <= 32'd0;
            O_ws  <= 1'b0;
        end
        else begin
            case(S_st)
                S_IDLE: begin
                    O_ws  <= 1'b0;
                    S_led <= 7'd0;
                    S_bit <= 5'd0;
                    S_cy  <= 32'd0;
                    S_sr  <= led_grb(7'd0, W_rad, W_any);
                    S_st  <= S_BIT;
                end

                S_BIT: begin
                    if(S_cy == 32'd0)     O_ws <= 1'b1;
                    if(S_cy == W_high_cy) O_ws <= 1'b0;

                    if(S_cy >= CYC_T - 32'd1) begin
                        S_cy <= 32'd0;
                        if(S_bit >= 5'd23) begin
                            S_bit <= 5'd0;
                            if(S_led >= N_LED - 1) begin
                                O_ws  <= 1'b0;
                                S_st  <= S_GAP;
                                S_gap <= 32'd0;
                            end
                            else begin
                                S_led <= S_led + 7'd1;
                                S_sr  <= led_grb(S_led + 7'd1, W_rad, W_any);
                            end
                        end
                        else begin
                            S_bit <= S_bit + 5'd1;
                        end
                    end
                    else begin
                        S_cy <= S_cy + 32'd1;
                    end
                end

                S_GAP: begin
                    O_ws <= 1'b0;
                    if(S_gap >= GAP_CYC - 32'd1) S_st <= S_IDLE;
                    else                         S_gap <= S_gap + 32'd1;
                end

                default: S_st <= S_IDLE;
            endcase
        end
    end

endmodule
