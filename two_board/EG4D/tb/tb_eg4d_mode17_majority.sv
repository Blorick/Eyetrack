`timescale 1ns/1ps

module tb_eg4d_mode17_majority;
    localparam integer W = 8;
    reg clk = 1'b0;
    reg rst_n = 1'b0;
    reg vsync, hsync, de, user, last, mask;
    wire ovs, ohs, ode, ouser, olast, omask, ovalid;
    integer row, col;
    integer errors;
    integer out_line, out_col;

    always #5 clk = ~clk;

    eg4d_mode17_majority #(.IMG_W(W)) dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_vsync(vsync), .I_hsync(hsync),
        .I_de(de), .I_user(user), .I_last(last), .I_mask(mask),
        .O_vsync(ovs), .O_hsync(ohs), .O_de(ode), .O_user(ouser),
        .O_last(olast), .O_mask(omask), .O_window_valid(ovalid)
    );

    task drive_line;
        input integer y;
        begin
            for (col = 0; col < W; col = col + 1) begin
                @(negedge clk);
                vsync = (y == 0 && col == 0);
                hsync = 1'b1;
                de = 1'b1;
                user = (col == 0);
                last = (col == W-1);
                // Rows 1..3 are all ones.  A valid 3x3 majority result
                // must therefore be one at centre rows 2..3 and columns 1..6.
                mask = (y >= 1 && y <= 3);
            end
            @(negedge clk);
            vsync = 1'b0; hsync = 1'b0; de = 1'b0;
            user = 1'b0; last = 1'b0; mask = 1'b0;
        end
    endtask

    always @(posedge clk) begin
        if (rst_n && ode) begin
            if (ouser) begin
                out_line = out_line + 1;
                out_col = 0;
            end
            if (ovalid !== ((out_line >= 2) && (out_col >= 1) &&
                           (out_col <= W-2))) begin
                $display("ERROR valid line=%0d col=%0d got=%0b", out_line,
                         out_col, ovalid);
                errors = errors + 1;
            end
            if (omask !== (((out_line >= 2) && (out_col >= 1) &&
                           (out_col <= W-2)))) begin
                $display("ERROR mask line=%0d col=%0d got=%0b", out_line,
                         out_col, omask);
                errors = errors + 1;
            end
            $display("OUT t=%0t user=%0b last=%0b valid=%0b mask=%0b", $time,
                     ouser, olast, ovalid, omask);
            out_col = out_col + 1;
        end
    end

    initial begin
        vsync=0; hsync=0; de=0; user=0; last=0; mask=0; errors=0;
        out_line=-1; out_col=0;
        repeat (3) @(posedge clk);
        rst_n = 1'b1;
        drive_line(0);
        drive_line(1);
        drive_line(2);
        drive_line(3);
        drive_line(4);
        repeat (8) @(posedge clk);
        if (errors != 0) $fatal(1, "TB_RESULT: FAIL errors=%0d", errors);
        $display("TB_RESULT: PASS (mode17 majority timing smoke)");
        $finish;
    end
endmodule
