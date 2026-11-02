//=============================================================================
//  tb_eg4d_mode18_width_equiv  --  RGB332 arithmetic width proof
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk (function equivalence only)
//  复位     : I_rst_n
//  参数     : all 256 RGB332 codes and all 256 yq values
//  验证状态 : Questa exhaustive arithmetic reference
//=============================================================================
`timescale 1ns/1ps

module tb_eg4d_mode18_width_equiv;
    reg W_clk = 1'b0;
    reg W_rst_n = 1'b0;
    reg [4:0] W_mode = 5'd18;
    reg [7:0] W_bin_th = 8'd128;
    reg [7:0] W_cartoon_th = 8'd80;
    reg W_vsync = 1'b0;
    reg W_hsync = 1'b0;
    reg W_de = 1'b0;
    reg W_user = 1'b0;
    reg W_last = 1'b0;
    reg [7:0] W_y = 8'd0;
    wire [4:0] W_o_mode;
    wire W_o_vsync, W_o_hsync, W_o_de, W_o_user, W_o_last;
    wire [7:0] W_o_y;
    wire W_o_edge, W_o_mask, W_o_window_valid;
    integer W_pixel;
    integer W_yq_value;
    integer W_errors;
    integer W_luma_checks;
    integer W_flat_checks;
    integer W_luma_min;
    integer W_luma_max;
    integer W_sum_min;
    integer W_sum_max;
    integer W_delta_min;
    integer W_delta_max;
    integer W_colour_min;
    integer W_colour_max;
    integer W_r8;
    integer W_g8;
    integer W_b8;
    integer W_gy;
    integer W_sum;
    integer W_dr;
    integer W_dg;
    integer W_db;
    integer W_sr;
    integer W_sg;
    integer W_sb;
    integer W_dut_luma;
    reg [7:0] W_dut_flat;
    reg [7:0] W_ref_flat;

    always #5 W_clk = ~W_clk;

    eg4d_mode_filter #(.IMG_W(8), .EDGE_RGB_TH(8'd80)) dut (
        .I_clk(W_clk), .I_rst_n(W_rst_n), .I_mode(W_mode),
        .I_bin_th(W_bin_th), .I_cartoon_th(W_cartoon_th),
        .I_vsync(W_vsync), .I_hsync(W_hsync), .I_de(W_de),
        .I_user(W_user), .I_last(W_last), .I_y(W_y),
        .O_mode(W_o_mode), .O_vsync(W_o_vsync), .O_hsync(W_o_hsync),
        .O_de(W_o_de), .O_user(W_o_user), .O_last(W_o_last),
        .O_y(W_o_y), .O_edge(W_o_edge), .O_mask(W_o_mask),
        .O_window_valid(W_o_window_valid)
    );

    function integer expand_r;
        input integer W_value;
        begin
            case (W_value)
                0: expand_r = 0;   1: expand_r = 36;
                2: expand_r = 73;  3: expand_r = 109;
                4: expand_r = 146; 5: expand_r = 182;
                6: expand_r = 219; default: expand_r = 255;
            endcase
        end
    endfunction

    function integer expand_b;
        input integer W_value;
        begin
            case (W_value)
                0: expand_b = 0;   1: expand_b = 85;
                2: expand_b = 170; default: expand_b = 255;
            endcase
        end
    endfunction

    function integer ref_luma;
        input integer W_value;
        integer W_ref_r;
        integer W_ref_g;
        integer W_ref_b;
        begin
            W_ref_r = expand_r((W_value >> 5) & 7);
            W_ref_g = expand_r((W_value >> 2) & 7);
            W_ref_b = expand_b(W_value & 3);
            ref_luma = (77 * W_ref_r + 150 * W_ref_g + 29 * W_ref_b) >> 8;
        end
    endfunction

    function integer quantize_delta;
        input integer W_delta;
        reg signed [9:0] W_delta10;
        begin
            W_delta10 = W_delta;
            W_delta10 = {W_delta10[9], W_delta10[8:4], 4'b0000};
            quantize_delta = W_delta10;
        end
    endfunction

    function [7:0] ref_flat;
        input integer W_value;
        input integer W_quantized_y;
        integer W_ref_r;
        integer W_ref_g;
        integer W_ref_b;
        integer W_ref_y;
        integer W_ref_rr;
        integer W_ref_gg;
        integer W_ref_bb;
        begin
            W_ref_r = expand_r((W_value >> 5) & 7);
            W_ref_g = expand_r((W_value >> 2) & 7);
            W_ref_b = expand_b(W_value & 3);
            W_ref_y = ref_luma(W_value);
            W_ref_rr = W_quantized_y + quantize_delta(W_ref_r - W_ref_y);
            W_ref_gg = W_quantized_y + quantize_delta(W_ref_g - W_ref_y);
            W_ref_bb = W_quantized_y + quantize_delta(W_ref_b - W_ref_y);
            if (W_ref_rr < 0) W_ref_rr = 0;
            else if (W_ref_rr > 255) W_ref_rr = 255;
            if (W_ref_gg < 0) W_ref_gg = 0;
            else if (W_ref_gg > 255) W_ref_gg = 255;
            if (W_ref_bb < 0) W_ref_bb = 0;
            else if (W_ref_bb > 255) W_ref_bb = 255;
            ref_flat = {W_ref_rr[7:5], W_ref_gg[7:5], W_ref_bb[7:6]};
        end
    endfunction

    initial begin
        W_errors = 0;
        W_luma_checks = 0;
        W_flat_checks = 0;
        W_luma_min = 999999;
        W_luma_max = -999999;
        W_sum_min = 999999;
        W_sum_max = -999999;
        W_delta_min = 999999;
        W_delta_max = -999999;
        W_colour_min = 999999;
        W_colour_max = -999999;
        repeat (3) @(posedge W_clk);
        W_rst_n = 1'b1;

        for (W_pixel = 0; W_pixel < 256; W_pixel = W_pixel + 1) begin
            W_r8 = expand_r((W_pixel >> 5) & 7);
            W_g8 = expand_r((W_pixel >> 2) & 7);
            W_b8 = expand_b(W_pixel & 3);
            W_sum = 77 * W_r8 + 150 * W_g8 + 29 * W_b8;
            W_gy = W_sum >> 8;
            W_dut_luma = dut.rgb332_luma(W_pixel[7:0]);
            if (W_dut_luma !== ref_luma(W_pixel)) begin
                W_errors = W_errors + 1;
                if (W_errors < 5)
                    $display("LUMA_MISMATCH p=%02h got=%0d exp=%0d",
                             W_pixel, W_dut_luma, ref_luma(W_pixel));
            end
            if (W_gy < W_luma_min) W_luma_min = W_gy;
            if (W_gy > W_luma_max) W_luma_max = W_gy;
            if (W_sum < W_sum_min) W_sum_min = W_sum;
            if (W_sum > W_sum_max) W_sum_max = W_sum;
            W_luma_checks = W_luma_checks + 1;

            for (W_yq_value = 0; W_yq_value < 256; W_yq_value = W_yq_value + 1) begin
                W_dr = quantize_delta(W_r8 - W_gy);
                W_dg = quantize_delta(W_g8 - W_gy);
                W_db = quantize_delta(W_b8 - W_gy);
                W_sr = W_yq_value + W_dr;
                W_sg = W_yq_value + W_dg;
                W_sb = W_yq_value + W_db;
                if (W_dr < W_delta_min) W_delta_min = W_dr;
                if (W_dg < W_delta_min) W_delta_min = W_dg;
                if (W_db < W_delta_min) W_delta_min = W_db;
                if (W_dr > W_delta_max) W_delta_max = W_dr;
                if (W_dg > W_delta_max) W_delta_max = W_dg;
                if (W_db > W_delta_max) W_delta_max = W_db;
                if (W_sr < W_colour_min) W_colour_min = W_sr;
                if (W_sg < W_colour_min) W_colour_min = W_sg;
                if (W_sb < W_colour_min) W_colour_min = W_sb;
                if (W_sr > W_colour_max) W_colour_max = W_sr;
                if (W_sg > W_colour_max) W_colour_max = W_sg;
                if (W_sb > W_colour_max) W_colour_max = W_sb;
                W_dut_flat = dut.rgb332_flat(W_pixel[7:0], W_yq_value[7:0]);
                W_ref_flat = ref_flat(W_pixel, W_yq_value);
                if (W_dut_flat !== W_ref_flat) begin
                    W_errors = W_errors + 1;
                    if (W_errors < 5)
                        $display("FLAT_MISMATCH p=%02h yq=%0d got=%02h exp=%02h",
                                 W_pixel, W_yq_value, W_dut_flat, W_ref_flat);
                end
                W_flat_checks = W_flat_checks + 1;
            end
        end

        if ((W_sum_min < 0) || (W_sum_max > 16'hffff) ||
            (W_colour_min < -512) || (W_colour_max > 511)) begin
            W_errors = W_errors + 1;
            $display("WIDTH_BOUND_FAIL sum=[%0d,%0d] colour=[%0d,%0d]",
                     W_sum_min, W_sum_max, W_colour_min, W_colour_max);
        end
        if (W_errors != 0)
            $fatal(1, "TB_RESULT: FAIL RGB332 width equivalence errors=%0d", W_errors);
        $display("TB_RESULT: PASS RGB332 width equivalence luma=%0d flat=%0d sum=[%0d,%0d] luma=[%0d,%0d] delta=[%0d,%0d] colour=[%0d,%0d]",
                 W_luma_checks, W_flat_checks, W_sum_min, W_sum_max,
                 W_luma_min, W_luma_max, W_delta_min, W_delta_max,
                 W_colour_min, W_colour_max);
        $finish;
    end
endmodule
