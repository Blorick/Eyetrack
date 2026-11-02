`timescale 1ns/1ps
module tb_mode17_address_pipeline;
    parameter integer IMG_W = 1280;
    reg S_clk = 0;
    always #6.667 S_clk = ~S_clk;
    reg S_rst_n = 0;
    reg S_vs=0, S_hs=0, S_de=0, S_user=0, S_last=0, S_mask=0;
    reg [4:0] S_mode=17;
    reg [7:0] S_y=0;
    wire [6:0] W_new, W_ref;
    eg4d_mode17_majority #(.IMG_W(IMG_W)) u_candidate (
        .I_clk(S_clk), .I_rst_n(S_rst_n), .I_vsync(S_vs), .I_hsync(S_hs),
        .I_de(S_de), .I_user(S_user), .I_last(S_last), .I_mask(S_mask),
        .O_vsync(W_new[6]), .O_hsync(W_new[5]), .O_de(W_new[4]),
        .O_user(W_new[3]), .O_last(W_new[2]), .O_mask(W_new[1]), .O_window_valid(W_new[0]));
    eg4d_mode17_majority_ref #(.IMG_W(IMG_W)) u_reference (
        .I_clk(S_clk), .I_rst_n(S_rst_n), .I_vsync(S_vs), .I_hsync(S_hs),
        .I_de(S_de), .I_user(S_user), .I_last(S_last), .I_mask(S_mask),
        .O_vsync(W_ref[6]), .O_hsync(W_ref[5]), .O_de(W_ref[4]),
        .O_user(W_ref[3]), .O_last(W_ref[2]), .O_mask(W_ref[1]), .O_window_valid(W_ref[0]));
    wire [20:0] W_pipeline, W_primary;
    eg4d_mode_pipeline #(.IMG_W(IMG_W)) u_pipeline (
        .I_clk(S_clk), .I_rst_n(S_rst_n), .I_mode(S_mode), .I_bin_th(8'd111), .I_cartoon_th(8'd70),
        .I_vsync(S_vs), .I_hsync(S_hs), .I_de(S_de), .I_user(S_user), .I_last(S_last), .I_y(S_y),
        .O_mode(W_pipeline[20:16]), .O_vsync(W_pipeline[15]), .O_hsync(W_pipeline[14]),
        .O_de(W_pipeline[13]), .O_user(W_pipeline[12]), .O_last(W_pipeline[11]),
        .O_y(W_pipeline[10:3]), .O_edge(W_pipeline[2]), .O_mask(W_pipeline[1]), .O_window_valid(W_pipeline[0]));
    eg4d_mode_filter #(.IMG_W(IMG_W)) u_primary_golden (
        .I_clk(S_clk), .I_rst_n(S_rst_n), .I_mode(S_mode), .I_bin_th(8'd111), .I_cartoon_th(8'd70),
        .I_vsync(S_vs), .I_hsync(S_hs), .I_de(S_de), .I_user(S_user), .I_last(S_last), .I_y(S_y),
        .O_mode(W_primary[20:16]), .O_vsync(W_primary[15]), .O_hsync(W_primary[14]),
        .O_de(W_primary[13]), .O_user(W_primary[12]), .O_last(W_primary[11]),
        .O_y(W_primary[10:3]), .O_edge(W_primary[2]), .O_mask(W_primary[1]), .O_window_valid(W_primary[0]));
    wire [6:0] W_majority_golden;
    eg4d_mode17_majority_ref #(.IMG_W(IMG_W)) u_pipeline_majority_golden (
        .I_clk(S_clk), .I_rst_n(S_rst_n), .I_vsync(W_primary[15]), .I_hsync(W_primary[14]),
        .I_de(W_primary[13]), .I_user(W_primary[12]), .I_last(W_primary[11]), .I_mask(W_primary[1]),
        .O_vsync(W_majority_golden[6]), .O_hsync(W_majority_golden[5]), .O_de(W_majority_golden[4]),
        .O_user(W_majority_golden[3]), .O_last(W_majority_golden[2]),
        .O_mask(W_majority_golden[1]), .O_window_valid(W_majority_golden[0]));
    reg [6:0] S_ref_delay=0, S_majority_delay=0;
    reg [20:0] S_primary_delay [0:2];
    reg [20:0] S_expected_pipeline;
    reg S_image [0:2][0:IMG_W-1];
    integer S_row=-1, S_x=0, S_sum, S_i, S_j, S_checks=0, S_golden_checks=0;
    reg S_previous_de=0, S_expected_mask=0, S_expected_valid=0;
    reg S_next_mask, S_next_valid;
    always @(posedge S_clk) begin
        if (!S_rst_n) begin
            S_ref_delay=0; S_majority_delay=0;
            for (S_i=0; S_i<3; S_i=S_i+1) S_primary_delay[S_i]=0;
            S_row=-1; S_x=0; S_previous_de=0;
            S_expected_mask=0; S_expected_valid=0;
            #1;
            if (W_new !== 0 || W_pipeline !== 0) $fatal(1,"reset output mismatch");
        end else begin
            // Reference pipelines sample pre-edge outputs exactly like downstream RTL.
            S_expected_pipeline=S_primary_delay[1];
            if (S_expected_pipeline[20:16] == 17)
                S_expected_pipeline={S_expected_pipeline[20:16],W_majority_golden[6:2],
                    {8{W_majority_golden[1]}},1'b0,W_majority_golden[1:0]};
            S_primary_delay[2]=S_primary_delay[1];
            S_primary_delay[1]=S_primary_delay[0];
            S_primary_delay[0]=W_primary;
            S_majority_delay=W_majority_golden;
            // Independent coordinate oracle uses only accepted input pixels.
            if (S_user) begin
                if (S_vs) S_row=0; else S_row=S_row+1;
                S_x=0;
            end
            S_sum=0;
            S_next_mask=0; S_next_valid=0;
            if (S_de) begin
                S_image[S_row%3][S_x]=S_mask;
                if (S_row>=2 && S_x>=2) begin
                    for(S_i=0;S_i<3;S_i=S_i+1)
                        for(S_j=0;S_j<3;S_j=S_j+1)
                            S_sum=S_sum+S_image[(S_row-S_i)%3][S_x-S_j];
                    S_next_mask=(S_sum>=5);
                    S_next_valid=S_previous_de;
                end
                S_x=S_x+1;
            end
            #1;
            if (W_new !== S_ref_delay) $fatal(1,"retime mismatch check=%0d got=%b expected=%b",S_checks,W_new,S_ref_delay);
            if (W_new[1] !== S_expected_mask || W_new[0] !== S_expected_valid)
                $fatal(1,"coordinate golden mismatch row=%0d x=%0d got=%b expected=%b%b",S_row,S_x,W_new[1:0],S_expected_mask,S_expected_valid);
            if (W_pipeline !== S_expected_pipeline)
                $fatal(1,"pipeline mismatch mode=%0d check=%0d got=%h expected=%h",S_mode,S_checks,W_pipeline,S_expected_pipeline);
            S_ref_delay=W_ref;
            S_expected_mask=S_next_mask; S_expected_valid=S_next_valid;
            S_previous_de=S_de;
            S_checks=S_checks+1;
            if (W_new[0]) S_golden_checks=S_golden_checks+1;
        end
    end
    task automatic blank(input integer count);
        integer k;
        begin
            for(k=0;k<count;k=k+1) begin
                @(negedge S_clk);
                S_vs=0; S_hs=0; S_de=0; S_user=0; S_last=0; S_mask=0; S_y=0;
            end
        end
    endtask
    task automatic line(input integer row, input integer seed, input integer width);
        integer x;
        begin
            for(x=0;x<width;x=x+1) begin
                @(negedge S_clk);
                S_vs=(row==0 && x==0); S_hs=1; S_de=1; S_user=(x==0); S_last=(x==width-1);
                S_mask=(((x*17+row*29+seed*13) % 31) < 18);
                S_y=(x*37+row*61+seed*17)%256;
            end
        end
    endtask
    integer S_test_mode, S_test_row;
    initial begin
        repeat(4) @(negedge S_clk);
        S_rst_n=1;
        // All 20 modes; spatially varying values; consecutive and blanked lines.
        for(S_test_mode=0;S_test_mode<20;S_test_mode=S_test_mode+1) begin
            blank(17); S_mode=S_test_mode;
            for(S_test_row=0;S_test_row<6;S_test_row=S_test_row+1) begin
                line(S_test_row,S_test_mode,IMG_W);
                if(S_test_row%2==0) blank(1+S_test_row);
            end
        end
        // Frame-boundary mode17/bypass changes without a pipeline flush.
        for(S_test_mode=0;S_test_mode<8;S_test_mode=S_test_mode+1) begin
            S_mode=(S_test_mode%2==0)?17:(S_test_mode%3);
            for(S_test_row=0;S_test_row<4;S_test_row=S_test_row+1)
                line(S_test_row,70+S_test_mode,IMG_W);
        end
        // Missing source line and short last line: physical raster counts delivered lines.
        blank(30); S_mode=17;
        line(0,99,IMG_W); blank(11); line(1,99,IMG_W);
        blank(80); line(3,99,IMG_W); line(4,99,IMG_W);
        line(5,99,IMG_W/2); blank(20);
        // Asynchronous reset with pending addresses/taps, followed by new frame.
        line(0,21,IMG_W/2);
        @(negedge S_clk); S_rst_n=0; S_de=0; S_user=0; S_last=0;
        blank(3); S_rst_n=1;
        for(S_test_row=0;S_test_row<5;S_test_row=S_test_row+1) begin
            line(S_test_row,21,IMG_W); blank(2);
        end
        blank(30);
        if(S_golden_checks<100) $fatal(1,"insufficient valid golden samples");
        $display("PASS mode17 address pipeline W=%0d checks=%0d coordinate_valid=%0d all_modes=20",IMG_W,S_checks,S_golden_checks);
        $finish;
    end
endmodule
