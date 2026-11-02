`timescale 1ns/1ps
// tb_dsp_status_overlay -- compact shared MD/UI 5x7 status overlay contract
//
// This directed test reaches the production 1280x720 row coordinates and
// checks the shared MD/UI decoder, mode-18 tens/ones path, profile row, and
// blanking outside the two five-glyph windows.
module tb_dsp_status_overlay;
    reg I_clk_pix = 1'b0;
    reg I_rst_n = 1'b0;
    reg I_de = 1'b0;
    reg I_user = 1'b0;
    reg [23:0] I_video = 24'h112233;
    reg [4:0] I_mode = 5'd18;
    reg [1:0] I_profile = 2'd2;
    wire [23:0] O_video;
    integer S_checks = 0;
    integer S_errors = 0;
    integer S_cycles;

    always #5 I_clk_pix = ~I_clk_pix;

    dsp_status_overlay #(
        .P_IMAGE_WIDTH(1280),
        .P_IMAGE_HEIGHT(720)
    ) dut (
        .I_clk_pix(I_clk_pix),
        .I_rst_n(I_rst_n),
        .I_de(I_de),
        .I_user(I_user),
        .I_video(I_video),
        .I_mode(I_mode),
        .I_profile(I_profile),
        .O_video(O_video)
    );

    task tick;
        begin
            @(posedge I_clk_pix);
            #1;
        end
    endtask

    task check_pixel;
        input [23:0] expected;
        input integer check_id;
        begin
            S_checks = S_checks + 1;
            if (O_video !== expected) begin
                S_errors = S_errors + 1;
                $display("STATUS_TB_ERROR id=%0d expected=%h actual=%h", check_id, expected, O_video);
            end
        end
    endtask

    initial begin
        // Reset must pass the source video through while DE is low.
        repeat (2) tick;
        check_pixel(24'h112233, 1);

        // Start a production frame.  The user marker resets the pixel
        // coordinate; the following DE clocks advance the 1280x720 raster.
        I_rst_n = 1'b1;
        I_user = 1'b1;
        tick;
        I_user = 1'b0;
        I_de = 1'b1;

        // After the user marker the state is x=1,y=0.  Reach x=32,y=400,
        // the first pixel of the MD row and the M glyph's lit column.
        for (S_cycles = 0; S_cycles < 512031; S_cycles = S_cycles + 1) tick;
        check_pixel(24'hffff40, 2);

        // The M glyph's next source column is blank at x=34; this also checks
        // that a background pixel is preserved inside the label window.
        tick;
        tick;
        check_pixel(24'h112233, 3);

        // Slot 3 carries the mode tens digit.  For mode 18, x=84 selects a
        // lit column in the shared '1' glyph.
        for (S_cycles = 0; S_cycles < 50; S_cycles = S_cycles + 1) tick;
        check_pixel(24'hffff40, 4);

        // x=112 is outside the five glyph slots and must pass source video.
        for (S_cycles = 0; S_cycles < 28; S_cycles = S_cycles + 1) tick;
        check_pixel(24'h112233, 5);

        // Move from y=400,x=112 to the profile row y=432,x=32.  The shared
        // U glyph has a lit first column and uses the profile-row color.
        for (S_cycles = 0; S_cycles < 40880; S_cycles = S_cycles + 1) tick;
        check_pixel(24'h40ffff, 6);

        if (S_errors == 0)
            $display("TB_RESULT: PASS tb_dsp_status_overlay checks=%0d errors=0", S_checks);
        else
            $display("TB_RESULT: FAIL tb_dsp_status_overlay checks=%0d errors=%0d", S_checks, S_errors);
        $finish;
    end
endmodule
