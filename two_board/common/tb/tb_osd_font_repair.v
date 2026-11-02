`timescale 1ns/1ps

module tb_osd_font_repair;
    reg        I_clk;
    reg [7:0]  I_char;
    reg [3:0]  I_row;
    wire [15:0] O_row_bits;
    integer T_code;
    integer T_row;
    integer T_nonzero;
    integer T_failures;

    osd_char_lib_sync u_dut (
        .I_clk      ( I_clk      ),
        .I_char     ( I_char     ),
        .I_row      ( I_row      ),
        .O_row_bits ( O_row_bits )
    );

    initial I_clk = 1'b0;
    always #5 I_clk = ~I_clk;

    task check_value;
        input [7:0]  T_char;
        input [3:0]  T_row_in;
        input [15:0] T_expected;
        begin
            @(negedge I_clk);
            I_char = T_char;
            I_row  = T_row_in;
            @(posedge I_clk);
            #1;
            if(O_row_bits !== T_expected) begin
                $display("FONT_FAIL char=%h row=%d got=%h expected=%h", T_char, T_row_in, O_row_bits, T_expected);
                T_failures = T_failures + 1;
            end
        end
    endtask

    initial begin
        I_char = 8'h00;
        I_row  = 4'd0;
        T_failures = 0;
        repeat(2) @(posedge I_clk);

        // The first checks include known old glyph rows and both repaired glyphs.
        check_value(8'h30, 4'd1, 16'h0380);
        check_value(8'h80, 4'd2, 16'h3ffc);
        check_value(8'h49, 4'd1, 16'h07e0);
        check_value(8'h4f, 4'd1, 16'h03c0);
        check_value(8'h20, 4'd1, 16'h0000);
        check_value(8'h00, 4'd7, 16'h0000);
        check_value(8'hff, 4'd9, 16'h0000);

        // Each ASCII letter must contain at least one lit row.
        for(T_code = 8'h41; T_code <= 8'h5a; T_code = T_code + 1) begin
            T_nonzero = 0;
            for(T_row = 0; T_row < 16; T_row = T_row + 1) begin
                @(negedge I_clk);
                I_char = T_code[7:0];
                I_row  = T_row[3:0];
                @(posedge I_clk);
                #1;
                if(O_row_bits !== 16'h0000)
                    T_nonzero = T_nonzero + 1;
            end
            if(T_nonzero == 0) begin
                $display("FONT_FAIL empty ASCII glyph %h", T_code);
                T_failures = T_failures + 1;
            end
        end

        if(T_failures == 0)
            $display("OSD_FONT_TB: PASS");
        else
            $display("OSD_FONT_TB: FAIL count=%0d", T_failures);
        $finish;
    end
endmodule
