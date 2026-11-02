`timescale 1ns/1ps

// Mode-18 edge semantics: a sharp RGB332 step must produce an edge flag and
// a black centre byte.  This is separate from the uniform-colour test because
// it exercises the local 3x3 range detector and the black-outline branch.
module tb_eg4d_mode18_edge;
    localparam integer W = 8;
    localparam [7:0] COLOUR = 8'hAE;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg [4:0] mode;
    reg [7:0] bin_th, cartoon_th;
    reg vsync, hsync, de, user, last;
    reg [7:0] y;
    wire [4:0] out_mode;
    wire ovs, ohs, ode, ouser, olast, oedge, omask, owindow_valid;
    wire [7:0] out_y;
    integer row, col, out_row, out_col;
    integer errors, valid_pixels, edge_pixels;
    reg [7:0] in_pixel;

    always #5 clk = ~clk;

    eg4d_mode_filter #(.IMG_W(W), .EDGE_RGB_TH(8'd80)) dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_mode(mode), .I_bin_th(bin_th),
        .I_cartoon_th(cartoon_th), .I_vsync(vsync), .I_hsync(hsync),
        .I_de(de), .I_user(user), .I_last(last), .I_y(y),
        .O_mode(out_mode), .O_vsync(ovs), .O_hsync(ohs), .O_de(ode),
        .O_user(ouser), .O_last(olast), .O_y(out_y), .O_edge(oedge),
        .O_mask(omask), .O_window_valid(owindow_valid)
    );

    task drive_line;
        input integer line;
        begin
            for (col = 0; col < W; col = col + 1) begin
                @(negedge clk);
                vsync = (line == 0 && col == 0);
                hsync = 1'b1; de = 1'b1;
                user = (col == 0); last = (col == W-1);
                y = (col < W/2) ? COLOUR : 8'h00;
            end
            @(negedge clk);
            vsync = 1'b0; hsync = 1'b0; de = 1'b0;
            user = 1'b0; last = 1'b0; y = 8'h00;
        end
    endtask

    always @(posedge clk) begin
        if (rst_n && ode) begin
            if (ouser) begin
                out_row = out_row + 1;
                out_col = 0;
            end
            if (out_row >= 2) begin
                if (owindow_valid) begin
                    valid_pixels = valid_pixels + 1;
                    if (out_mode !== 5'd18) begin
                        $display("FAIL mode18 edge mode=%0d", out_mode);
                        errors = errors + 1;
                    end
                    if (oedge) begin
                        edge_pixels = edge_pixels + 1;
                        if (out_y !== 8'h00) begin
                            $display("FAIL mode18 edge row=%0d col=%0d y=%02h",
                                     out_row, out_col, out_y);
                            errors = errors + 1;
                        end
                    end
                end
                else if (out_y !== 8'h00 || oedge !== 1'b0) begin
                    $display("FAIL mode18 edge invalid row=%0d col=%0d y=%02h edge=%b",
                             out_row, out_col, out_y, oedge);
                    errors = errors + 1;
                end
            end
            out_col = out_col + 1;
        end
    end

    initial begin
        mode = 5'd18; bin_th = 8'd128; cartoon_th = 8'd80;
        vsync = 1'b0; hsync = 1'b0; de = 1'b0;
        user = 1'b0; last = 1'b0; y = 8'h00;
        out_row = -1; out_col = 0; errors = 0;
        valid_pixels = 0; edge_pixels = 0;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        for (row = 0; row < 6; row = row + 1) drive_line(row);
        repeat (20) @(posedge clk);
        if (valid_pixels == 0) begin
            $display("FAIL mode18 edge no valid centres");
            errors = errors + 1;
        end
        if (edge_pixels == 0) begin
            $display("FAIL mode18 edge detector did not flag the RGB332 step");
            errors = errors + 1;
        end
        if (errors != 0)
            $fatal(1, "TB_RESULT: FAIL mode18 edge errors=%0d", errors);
        $display("TB_RESULT: PASS mode18 RGB332 edge valid=%0d black_edges=%0d",
                 valid_pixels, edge_pixels);
        $finish;
    end
endmodule
