`timescale 1ns/1ps

// Mode-18 endpoint smoke: drive a real RGB332 raster through the slave
// filter, then check the valid centre window.  A uniform coloured patch must
// remain the same RGB332 code; this catches an accidental Y8/grayscale path.
module tb_eg4d_mode18_rgb332;
    localparam integer W = 8;
    localparam [7:0] P_COLOUR = 8'b101_011_10; // non-grey R3G3B2 sample

    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg [4:0] mode;
    reg [7:0] bin_th;
    reg [7:0] cartoon_th;
    reg vsync, hsync, de, user, last;
    reg [7:0] y;
    wire [4:0] out_mode;
    wire ovs, ohs, ode, ouser, olast, oedge, omask;
    wire owindow_valid;
    wire [7:0] out_y;
    integer in_row, in_col;
    integer out_row, out_col;
    integer errors;
    integer valid_pixels;
    integer edge_pixels;
    reg [7:0] expected_colour;

    always #5 clk = ~clk;

    function integer expand_r;
        input integer v;
        begin
            case (v)
                0: expand_r = 0;   1: expand_r = 36;
                2: expand_r = 73;  3: expand_r = 109;
                4: expand_r = 146; 5: expand_r = 182;
                6: expand_r = 219; default: expand_r = 255;
            endcase
        end
    endfunction

    function integer expand_b;
        input integer v;
        begin
            case (v)
                0: expand_b = 0; 1: expand_b = 85;
                2: expand_b = 170; default: expand_b = 255;
            endcase
        end
    endfunction

    // Independent reference for the mode-18 uniform-window result.  The
    // signed four-bit truncation mirrors the RTL's explicit two's-complement
    // concatenation, rather than assuming a grey output.
    function [7:0] ref_flat;
        input [7:0] p;
        integer r8, g8, b8, gy, yq, s, dr, dg, db, rr, gg, bb;
        reg signed [9:0] d10;
        begin
            r8 = expand_r(p[7:5]);
            g8 = expand_r(p[4:2]);
            b8 = expand_b(p[1:0]);
            gy = (77*r8 + 150*g8 + 29*b8) >> 8;
            s = 9 * gy;
            yq = ((57*s) >> 9) & 8'hE0;
            d10 = r8 - gy; dr = $signed({d10[9], d10[8:4], 4'b0});
            d10 = g8 - gy; dg = $signed({d10[9], d10[8:4], 4'b0});
            d10 = b8 - gy; db = $signed({d10[9], d10[8:4], 4'b0});
            rr = yq + dr; gg = yq + dg; bb = yq + db;
            if (rr < 0) rr = 0; else if (rr > 255) rr = 255;
            if (gg < 0) gg = 0; else if (gg > 255) gg = 255;
            if (bb < 0) bb = 0; else if (bb > 255) bb = 255;
            ref_flat = {rr[7:5], gg[7:5], bb[7:6]};
        end
    endfunction

    eg4d_mode_filter #(.IMG_W(W), .EDGE_RGB_TH(8'd80)) dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_mode(mode), .I_bin_th(bin_th),
        .I_cartoon_th(cartoon_th), .I_vsync(vsync), .I_hsync(hsync),
        .I_de(de), .I_user(user), .I_last(last), .I_y(y),
        .O_mode(out_mode), .O_vsync(ovs), .O_hsync(ohs), .O_de(ode),
        .O_user(ouser), .O_last(olast), .O_y(out_y), .O_edge(oedge),
        .O_mask(omask), .O_window_valid(owindow_valid)
    );

    task drive_line;
        input integer row;
        input [7:0] pixel;
        begin
            for (in_col = 0; in_col < W; in_col = in_col + 1) begin
                @(negedge clk);
                vsync = (row == 0 && in_col == 0);
                hsync = 1'b1;
                de = 1'b1;
                user = (in_col == 0);
                last = (in_col == W-1);
                y = pixel;
            end
            @(negedge clk);
            vsync = 1'b0; hsync = 1'b0; de = 1'b0;
            user = 1'b0; last = 1'b0; y = 8'd0;
        end
    endtask

    always @(posedge clk) begin
        if (rst_n && ode) begin
            if (ouser) begin
                out_row = out_row + 1;
                out_col = 0;
            end
            // The first two output rows and the first two pipeline columns
            // have no complete 3x3 window.  They must be black rather than
            // leaking a stale quantised value.  Valid centres thereafter
            // preserve the input's RGB332 code and remain non-edge.
            if ((out_row >= 2) && owindow_valid) begin
                valid_pixels = valid_pixels + 1;
                if (out_y !== expected_colour || oedge !== 1'b0 || out_mode !== 5'd18) begin
                    $display("FAIL mode18 row=%0d col=%0d y=%02h edge=%b mode=%0d",
                             out_row, out_col, out_y, oedge, out_mode);
                    errors = errors + 1;
                end
            end
            else if ((out_row >= 2) && (out_y !== 8'h00 || oedge !== 1'b0)) begin
                $display("FAIL mode18 invalid beat row=%0d col=%0d y=%02h edge=%b valid=%b",
                         out_row, out_col, out_y, oedge, owindow_valid);
                errors = errors + 1;
            end
            if (oedge)
                edge_pixels = edge_pixels + 1;
            out_col = out_col + 1;
        end
    end

    initial begin
        mode = 5'd18;
        bin_th = 8'd128;
        cartoon_th = 8'd80;
        vsync = 1'b0; hsync = 1'b0; de = 1'b0; user = 1'b0; last = 1'b0;
        y = 8'd0;
        out_row = -1; out_col = 0; errors = 0;
        valid_pixels = 0; edge_pixels = 0;
        expected_colour = ref_flat(P_COLOUR);
        $display("TB_NOTE mode18 input=%02h expected_quantized=%02h", P_COLOUR,
                 expected_colour);
        $display("TB_NOTE mode18 rtl_flat=%02h rtl_luma=%02h", dut.rgb332_flat(P_COLOUR, 8'h80),
                 dut.rgb332_luma(P_COLOUR));
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        for (in_row = 0; in_row < 6; in_row = in_row + 1)
            drive_line(in_row, P_COLOUR);
        repeat (20) @(posedge clk);
        if (valid_pixels != 15) begin
            $display("FAIL mode18 valid_pixels=%0d expected=15", valid_pixels);
            errors = errors + 1;
        end
        if (edge_pixels != 0) begin
            $display("FAIL mode18 uniform patch edge_pixels=%0d", edge_pixels);
            errors = errors + 1;
        end
        if (errors != 0)
            $fatal(1, "TB_RESULT: FAIL mode18 RGB332 errors=%0d", errors);
        $display("TB_RESULT: PASS mode18 RGB332 centre quantization=%02h pixels=%0d",
                 expected_colour, valid_pixels);
        $finish;
    end
endmodule
