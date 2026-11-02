//=============================================================================
//  dsp_status_overlay -- 模式、控制页及帧一致的测距测速数值显示
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk_pix
//  复位     : I_rst_n，异步低有效
//  参数     : P_IMAGE_WIDTH, P_IMAGE_HEIGHT
//  验证状态 : 见候选目录 reports；未据此宣称板测通过
//=============================================================================
module dsp_status_overlay #(
    parameter integer P_IMAGE_WIDTH = 1280,
    parameter integer P_IMAGE_HEIGHT = 720,
    parameter integer P_INK_DIAGNOSTICS = 0
) (
    input wire I_clk_pix,
    input wire I_rst_n,
    input wire I_de,
    input wire I_user,
    input wire [23:0] I_video,
    input wire [4:0] I_mode,
    input wire [1:0] I_profile,
    input wire [19:0] I_dist_mm,
    input wire [19:0] I_speed_mmps,
    input wire I_metrics_valid,
    input wire [7:0] I_metrics_state,
    input wire [15:0] I_ink_status,
    input wire [6:0] I_ink_raw_mask,
    input wire [15:0] I_ink_counts,
    output wire [23:0] O_video
);
    // PARAMS: 原有字形编码保留；新增字形只用于两个数值行。
    localparam [34:0] P_GLYPH_ZERO = 35'h3a39ace2e;
    localparam [34:0] P_GLYPH_ONE = 35'h388421184;
    localparam [34:0] P_GLYPH_TWO = 35'h7d041062e;
    localparam [34:0] P_GLYPH_THREE = 35'h78217043e;
    localparam [34:0] P_GLYPH_FOUR = 35'h085f928c2;
    localparam [34:0] P_GLYPH_FIVE = 35'h7821f421f;
    localparam [34:0] P_GLYPH_SIX = 35'h3a31f420e;
    localparam [34:0] P_GLYPH_SEVEN = 35'h21082083f;
    localparam [34:0] P_GLYPH_EIGHT = 35'h3a317462e;
    localparam [34:0] P_GLYPH_NINE = 35'h38217c62e;
    localparam [34:0] P_LETTER_M = 35'h4631ad771;
    localparam [34:0] P_LETTER_D = 35'h7a318c63e;
    localparam [34:0] P_LETTER_U = 35'h3a318c631;
    localparam [34:0] P_LETTER_I = 35'h38842108e;
    localparam [34:0] P_GLYPH_COLON = 35'h008401080;
    localparam [34:0] P_LETTER_S = 35'h78217420f;
    localparam [34:0] P_LETTER_P = 35'h4210f463e;
    localparam [34:0] P_LETTER_H = 35'h4631fc631;
    localparam [34:0] P_GLYPH_SLASH = 35'h420820821;
    localparam [34:0] P_GLYPH_DASH = 35'h0000f8000;
    localparam [34:0] P_LETTER_A = 35'h4631fc62e;
    localparam [34:0] P_LETTER_B = 35'h7a31f463e;
    localparam [34:0] P_LETTER_C = 35'h3e108420f;
    localparam [34:0] P_LETTER_E = 35'h7e10f421f;
    localparam [34:0] P_LETTER_F = 35'h4210f421f;
    localparam [34:0] P_LETTER_N = 35'h46319d731;
    localparam [34:0] P_LETTER_K = 35'h4654c5251;

    reg [10:0] S_x_pix;
    reg [9:0] S_y_pix;
    reg [4:0] S_character;
    reg [34:0] S_glyph;
    reg [13:0] S_dist_binary_pix;
    reg [13:0] S_speed_binary_pix;
    reg [15:0] S_dist_work_bcd_pix;
    reg [15:0] S_speed_work_bcd_pix;
    reg [3:0] S_conversion_step_pix;
    reg S_conversion_busy_pix;
    reg S_conversion_valid_pix;
    reg [7:0] S_conversion_state_pix;
    reg [15:0] S_dist_pending_bcd_pix;
    reg [15:0] S_speed_pending_bcd_pix;
    reg S_pending_ready_pix;
    reg S_pending_valid_pix;
    reg [7:0] S_pending_state_pix;
    reg [15:0] S_dist_active_bcd_pix;
    reg [15:0] S_speed_active_bcd_pix;
    reg S_active_valid_pix;
    reg [7:0] S_active_state_pix;
    reg [15:0] S_ink_status_pix;
    reg [6:0] S_ink_raw_mask_pix;
    reg [15:0] S_ink_counts_pix;
    reg [3:0] S_ink_digit;

    wire [10:0] W_x_pix;
    wire [9:0] W_y_pix;
    wire W_mode_row;
    wire W_profile_row;
    wire W_distance_row;
    wire W_speed_row;
    wire W_metric_row;
    wire W_ink_row;
    wire W_label_column;
    wire [10:0] W_local_x;
    wire [9:0] W_local_y;
    wire [3:0] W_slot;
    wire [2:0] W_font_column;
    wire [2:0] W_font_row;
    wire [5:0] W_row_address;
    wire [5:0] W_glyph_address;
    wire W_font_active;
    wire W_glyph_bit;
    wire W_label_row;
    wire W_label_active;
    wire W_draw;
    wire W_mode_tens;
    wire [3:0] W_mode_ones;
    wire [23:0] W_label_color;
    wire W_metric_available;
    wire W_metric_held;
    wire [15:0] W_metric_digits;
    wire [15:0] W_dist_adjusted;
    wire [15:0] W_speed_adjusted;
    wire [15:0] W_dist_next_bcd;
    wire [15:0] W_speed_next_bcd;

    // DATAPATH: 每拍只经过四个独立的加三器，不在像素颜色通路做除法。
    function [15:0] f_adjust_bcd;
        input [15:0] I_bcd;
        begin
            f_adjust_bcd[3:0] = (I_bcd[3:0] >= 4'd5) ? I_bcd[3:0] + 4'd3 : I_bcd[3:0];
            f_adjust_bcd[7:4] = (I_bcd[7:4] >= 4'd5) ? I_bcd[7:4] + 4'd3 : I_bcd[7:4];
            f_adjust_bcd[11:8] = (I_bcd[11:8] >= 4'd5) ? I_bcd[11:8] + 4'd3 : I_bcd[11:8];
            f_adjust_bcd[15:12] = (I_bcd[15:12] >= 4'd5) ? I_bcd[15:12] + 4'd3 : I_bcd[15:12];
        end
    endfunction

    assign W_dist_adjusted = f_adjust_bcd(S_dist_work_bcd_pix);
    assign W_speed_adjusted = f_adjust_bcd(S_speed_work_bcd_pix);
    assign W_dist_next_bcd = {W_dist_adjusted[14:0], S_dist_binary_pix[13]};
    assign W_speed_next_bcd = {W_speed_adjusted[14:0], S_speed_binary_pix[13]};

    // Freeze the diagnostic tuple at SOF; live changes never tear its digits.
    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_ink_status_pix <= 16'd0;
            S_ink_raw_mask_pix <= 7'd0;
            S_ink_counts_pix <= 16'd0;
        end else if (I_user && P_INK_DIAGNOSTICS != 0) begin
            S_ink_status_pix <= I_ink_status;
            S_ink_raw_mask_pix <= I_ink_raw_mask;
            S_ink_counts_pix <= I_ink_counts;
        end
    end

    // 一次 SOF 冻结一组输入；完成的结果只在下一次 SOF 成组发布。
    // 过短帧使未完成转换被新的 SOF 取代，不能混合两帧的数字和状态。
    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_dist_binary_pix <= 14'd0;
            S_speed_binary_pix <= 14'd0;
            S_dist_work_bcd_pix <= 16'd0;
            S_speed_work_bcd_pix <= 16'd0;
            S_conversion_step_pix <= 4'd0;
            S_conversion_busy_pix <= 1'b0;
            S_conversion_valid_pix <= 1'b0;
            S_conversion_state_pix <= 8'd0;
            S_dist_pending_bcd_pix <= 16'd0;
            S_speed_pending_bcd_pix <= 16'd0;
            S_pending_ready_pix <= 1'b0;
            S_pending_valid_pix <= 1'b0;
            S_pending_state_pix <= 8'd0;
            S_dist_active_bcd_pix <= 16'd0;
            S_speed_active_bcd_pix <= 16'd0;
            S_active_valid_pix <= 1'b0;
            S_active_state_pix <= 8'd0;
        end else if (I_user) begin
            if (S_pending_ready_pix) begin
                S_dist_active_bcd_pix <= S_dist_pending_bcd_pix;
                S_speed_active_bcd_pix <= S_speed_pending_bcd_pix;
                S_active_valid_pix <= S_pending_valid_pix;
                S_active_state_pix <= S_pending_state_pix;
            end
            S_pending_ready_pix <= 1'b0;
            S_dist_binary_pix <= (I_dist_mm > 20'd9999) ? 14'd9999 : I_dist_mm[13:0];
            S_speed_binary_pix <= (I_speed_mmps > 20'd9999) ? 14'd9999 : I_speed_mmps[13:0];
            S_dist_work_bcd_pix <= 16'd0;
            S_speed_work_bcd_pix <= 16'd0;
            S_conversion_step_pix <= 4'd0;
            S_conversion_busy_pix <= 1'b1;
            S_conversion_valid_pix <= I_metrics_valid;
            S_conversion_state_pix <= I_metrics_state;
        end else if (S_conversion_busy_pix) begin
            S_dist_work_bcd_pix <= W_dist_next_bcd;
            S_speed_work_bcd_pix <= W_speed_next_bcd;
            S_dist_binary_pix <= {S_dist_binary_pix[12:0], 1'b0};
            S_speed_binary_pix <= {S_speed_binary_pix[12:0], 1'b0};
            if (S_conversion_step_pix == 4'd13) begin
                S_dist_pending_bcd_pix <= W_dist_next_bcd;
                S_speed_pending_bcd_pix <= W_speed_next_bcd;
                S_pending_valid_pix <= S_conversion_valid_pix;
                S_pending_state_pix <= S_conversion_state_pix;
                S_pending_ready_pix <= 1'b1;
                S_conversion_busy_pix <= 1'b0;
            end else begin
                S_conversion_step_pix <= S_conversion_step_pix + 4'd1;
            end
        end
    end

    // OSD: 沿用原有坐标定义和最终 RGB 选择，不增加任何视频流水拍。
    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_x_pix <= 11'd0;
            S_y_pix <= 10'd0;
        end else if (I_user) begin
            S_x_pix <= 11'd1;
            S_y_pix <= 10'd0;
        end else if (I_de) begin
            if (S_x_pix == P_IMAGE_WIDTH - 1) begin
                S_x_pix <= 11'd0;
                if (S_y_pix == P_IMAGE_HEIGHT - 1) S_y_pix <= 10'd0;
                else S_y_pix <= S_y_pix + 10'd1;
            end else S_x_pix <= S_x_pix + 11'd1;
        end
    end

    assign W_x_pix = I_user ? 11'd0 : S_x_pix;
    assign W_y_pix = I_user ? 10'd0 : S_y_pix;
    assign W_mode_row = (W_y_pix >= 10'd400) && (W_y_pix < 10'd416);
    assign W_profile_row = (W_y_pix >= 10'd432) && (W_y_pix < 10'd448);
    assign W_distance_row = (W_y_pix >= 10'd464) && (W_y_pix < 10'd480);
    assign W_speed_row = (W_y_pix >= 10'd496) && (W_y_pix < 10'd512);
    assign W_metric_row = W_distance_row || W_speed_row;
    assign W_ink_row = (P_INK_DIAGNOSTICS != 0) &&
                       (W_y_pix >= 10'd528) && (W_y_pix < 10'd544);
    assign W_label_column = (W_x_pix >= 11'd32) &&
                            (W_ink_row ? (W_x_pix < 11'd288) :
                            W_metric_row ? (W_x_pix < 11'd240) : (W_x_pix < 11'd112));
    assign W_local_x = W_x_pix - 11'd32;
    assign W_local_y = W_ink_row ? W_y_pix - 10'd528 :
                       W_speed_row ? W_y_pix - 10'd496 :
                       W_distance_row ? W_y_pix - 10'd464 :
                       W_profile_row ? W_y_pix - 10'd432 : W_y_pix - 10'd400;
    assign W_slot = W_local_x[7:4];
    assign W_font_column = W_local_x[3:1];
    assign W_font_row = W_local_y[3:1];
    assign W_row_address = {W_font_row, 2'b00} + W_font_row;
    assign W_glyph_address = W_row_address + (6'd4 - W_font_column);
    assign W_font_active = (W_font_column < 3'd5) && (W_font_row < 3'd7);
    assign W_glyph_bit = W_font_active ? S_glyph[W_glyph_address] : 1'b0;
    assign W_label_row = W_mode_row || W_profile_row || W_metric_row || W_ink_row;
    assign W_label_active = W_label_row && W_label_column;
    assign W_draw = I_de && W_label_active && W_glyph_bit;
    assign W_mode_tens = (I_mode >= 5'd10);
    assign W_mode_ones = W_mode_tens ? (I_mode - 5'd10) : I_mode[3:0];
    assign W_metric_available = S_active_valid_pix &&
                                ((S_active_state_pix == 8'd1) || (S_active_state_pix == 8'd2));
    assign W_metric_held = W_metric_available && (S_active_state_pix == 8'd2);
    assign W_metric_digits = W_speed_row ? S_speed_active_bcd_pix : S_dist_active_bcd_pix;
    assign W_label_color = W_ink_row ? 24'hffffff :
                           W_metric_row ? (W_metric_held ? 24'hffb040 : 24'h80ff80) :
                           W_profile_row ? 24'h40ffff : 24'hffff40;
    assign O_video = W_draw ? W_label_color : I_video;

    always @(*) begin
        S_ink_digit = 4'd0;
        case (W_slot)
            4'd4: S_ink_digit = S_ink_status_pix[15:12];
            4'd5: S_ink_digit = S_ink_status_pix[11:8];
            4'd6: S_ink_digit = S_ink_status_pix[7:4];
            4'd7: S_ink_digit = S_ink_status_pix[3:0];
            4'd9: S_ink_digit = {1'b0, S_ink_raw_mask_pix[6:4]};
            4'd10: S_ink_digit = S_ink_raw_mask_pix[3:0];
            4'd12: S_ink_digit = S_ink_counts_pix[15:12];
            4'd13: S_ink_digit = S_ink_counts_pix[11:8];
            4'd14: S_ink_digit = S_ink_counts_pix[7:4];
            4'd15: S_ink_digit = S_ink_counts_pix[3:0];
            default: S_ink_digit = 4'd0;
        endcase
    end

    always @(*) begin
        S_character = 5'd31;
        if (W_ink_row) begin
            // INK:hhhh hh hhhh = status, mask, timeout/reject/result/EOF counts.
            case (W_slot)
                4'd0: S_character = 5'd13;
                4'd1: S_character = 5'd27;
                4'd2: S_character = 5'd28;
                4'd3: S_character = 5'd14;
                4'd8, 4'd11: S_character = 5'd31;
                default: S_character = (S_ink_digit < 4'd10) ?
                    {1'b0, S_ink_digit} : 5'd21 + {1'b0, S_ink_digit} - 5'd10;
            endcase
        end else if (W_metric_row) begin
            case (W_slot)
                4'd0: S_character = W_speed_row ? 5'd16 : 5'd11;
                4'd1: S_character = W_speed_row ? 5'd17 : 5'd13;
                4'd2: S_character = W_speed_row ? 5'd11 : 5'd16;
                4'd3: S_character = 5'd14;
                4'd4: S_character = W_metric_available ? {1'b0, W_metric_digits[15:12]} : 5'd20;
                4'd5: S_character = W_metric_available ? {1'b0, W_metric_digits[11:8]} : 5'd20;
                4'd6: S_character = W_metric_available ? {1'b0, W_metric_digits[7:4]} : 5'd20;
                4'd7: S_character = W_metric_available ? {1'b0, W_metric_digits[3:0]} : 5'd20;
                4'd8, 4'd9: S_character = 5'd10;
                4'd10: S_character = W_speed_row ? 5'd19 : 5'd31;
                4'd11: S_character = W_speed_row ? 5'd16 : 5'd31;
                4'd12: S_character = W_metric_held ? 5'd18 : 5'd31;
                default: S_character = 5'd31;
            endcase
        end else begin
            case (W_slot)
                4'd0: S_character = W_profile_row ? 5'd12 : 5'd10;
                4'd1: S_character = W_profile_row ? 5'd13 : 5'd11;
                4'd2: S_character = 5'd14;
                4'd3: S_character = W_profile_row ? 5'd0 : {4'd0, W_mode_tens};
                4'd4: S_character = W_profile_row ? {3'd0, I_profile} : {1'b0, W_mode_ones};
                default: S_character = 5'd31;
            endcase
        end
    end

    always @(*) begin
        S_glyph = 35'd0;
        case (S_character)
            5'd0: S_glyph = P_GLYPH_ZERO;
            5'd1: S_glyph = P_GLYPH_ONE;
            5'd2: S_glyph = P_GLYPH_TWO;
            5'd3: S_glyph = P_GLYPH_THREE;
            5'd4: S_glyph = P_GLYPH_FOUR;
            5'd5: S_glyph = P_GLYPH_FIVE;
            5'd6: S_glyph = P_GLYPH_SIX;
            5'd7: S_glyph = P_GLYPH_SEVEN;
            5'd8: S_glyph = P_GLYPH_EIGHT;
            5'd9: S_glyph = P_GLYPH_NINE;
            5'd10: S_glyph = P_LETTER_M;
            5'd11: S_glyph = P_LETTER_D;
            5'd12: S_glyph = P_LETTER_U;
            5'd13: S_glyph = P_LETTER_I;
            5'd14: S_glyph = P_GLYPH_COLON;
            5'd16: S_glyph = P_LETTER_S;
            5'd17: S_glyph = P_LETTER_P;
            5'd18: S_glyph = P_LETTER_H;
            5'd19: S_glyph = P_GLYPH_SLASH;
            5'd20: S_glyph = P_GLYPH_DASH;
            5'd21: S_glyph = P_LETTER_A;
            5'd22: S_glyph = P_LETTER_B;
            5'd23: S_glyph = P_LETTER_C;
            5'd24: S_glyph = P_LETTER_D;
            5'd25: S_glyph = P_LETTER_E;
            5'd26: S_glyph = P_LETTER_F;
            5'd27: S_glyph = P_LETTER_N;
            5'd28: S_glyph = P_LETTER_K;
            default: S_glyph = 35'd0;
        endcase
    end
endmodule
