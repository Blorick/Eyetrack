//======================================================================
//  img_mode_ctrl  --  interactive parameters for img_process_96
//
//  Ported from udp_cam_ctrl.v of the reference design, where KEY3 steps
//  the parameter of the currently selected interactive mode:
//
//    mode 7 (brightness) : O_brightness steps by +32 and wraps back to
//                          BRIGHTNESS_MIN after it passes BRIGHTNESS_MAX
//    mode 8 (saturation) : O_sat_factor cycles 64 -> 128 -> 192 -> 64
//
//  Reference values kept verbatim:
//    BRIGHTNESS_MIN = -255, BRIGHTNESS_MAX = 255, BRIGHTNESS_STEP = 32
//    SAT_FACTOR_LOW = 64, SAT_FACTOR_NORMAL = 128, SAT_FACTOR_HIGH = 192
//
//  One documented deviation: the reference powers up with
//  brightness_level = BRIGHTNESS_MIN (-255), which renders mode 7 as a
//  black screen until KEY3 is pressed.  v1.6 powers up at 0 (neutral)
//  so that selecting mode 7 immediately shows the current image.
//
//  In any other mode the adjust key is ignored.
//======================================================================
module img_mode_ctrl (
    input  wire             I_clk,
    input  wire             I_rst_n,
    input  wire [4:0]       I_mode,        // current img_process_96 mode
    input  wire             I_adj_pulse,   // one-cycle pulse from KEY3
    output reg signed [8:0] O_brightness,
    output reg [7:0]        O_sat_factor
);

    localparam [4:0] MODE_BRIGHTNESS = 5'd7;
    localparam [4:0] MODE_SATURATION = 5'd8;

    localparam integer BRIGHTNESS_MAX = 255;
    localparam integer BRIGHTNESS_MIN = -255;
    localparam integer BRIGHTNESS_STEP = 32;

    localparam [7:0] SAT_FACTOR_LOW    = 8'd64;
    localparam [7:0] SAT_FACTOR_NORMAL = 8'd128;
    localparam [7:0] SAT_FACTOR_HIGH   = 8'd192;

    reg [1:0] S_sat_sel;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_brightness <= 9'sd0;      // neutral start (see header)
            S_sat_sel    <= 2'd1;       // normal saturation
        end
        else if (I_adj_pulse) begin
            if (I_mode == MODE_BRIGHTNESS) begin
                if (O_brightness >= BRIGHTNESS_MAX)
                    O_brightness <= -9'sd255;      // BRIGHTNESS_MIN
                else
                    O_brightness <= O_brightness + 9'sd32;
            end
            else if (I_mode == MODE_SATURATION) begin
                if (S_sat_sel == 2'd2)
                    S_sat_sel <= 2'd0;
                else
                    S_sat_sel <= S_sat_sel + 2'd1;
            end
        end
    end

    always @(*) begin
        case (S_sat_sel)
            2'd0:    O_sat_factor = SAT_FACTOR_LOW;
            2'd1:    O_sat_factor = SAT_FACTOR_NORMAL;
            2'd2:    O_sat_factor = SAT_FACTOR_HIGH;
            default: O_sat_factor = SAT_FACTOR_NORMAL;
        endcase
    end

endmodule
