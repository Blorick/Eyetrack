`timescale 1ns/1ps
module tb_eg4d_mode_filter_chroma_equiv;
    parameter integer IMG_W=1280;
    reg S_clk=0;
    always #6.667 S_clk=~S_clk;
    reg S_rst_n=0;
    reg [4:0] S_mode=0;
    reg [7:0] S_bin_th=111, S_cartoon_th=70, S_y=0;
    reg S_vs=0,S_hs=0,S_de=0,S_user=0,S_last=0;
    wire [20:0] W_new,W_ref;
    eg4d_mode_filter #(.IMG_W(IMG_W)) u_new (
        .I_clk(S_clk),.I_rst_n(S_rst_n),.I_mode(S_mode),.I_bin_th(S_bin_th),.I_cartoon_th(S_cartoon_th),
        .I_vsync(S_vs),.I_hsync(S_hs),.I_de(S_de),.I_user(S_user),.I_last(S_last),.I_y(S_y),
        .O_mode(W_new[20:16]),.O_vsync(W_new[15]),.O_hsync(W_new[14]),.O_de(W_new[13]),
        .O_user(W_new[12]),.O_last(W_new[11]),.O_y(W_new[10:3]),.O_edge(W_new[2]),.O_mask(W_new[1]),.O_window_valid(W_new[0]));
    eg4d_mode_filter_ref #(.IMG_W(IMG_W)) u_ref (
        .I_clk(S_clk),.I_rst_n(S_rst_n),.I_mode(S_mode),.I_bin_th(S_bin_th),.I_cartoon_th(S_cartoon_th),
        .I_vsync(S_vs),.I_hsync(S_hs),.I_de(S_de),.I_user(S_user),.I_last(S_last),.I_y(S_y),
        .O_mode(W_ref[20:16]),.O_vsync(W_ref[15]),.O_hsync(W_ref[14]),.O_de(W_ref[13]),
        .O_user(W_ref[12]),.O_last(W_ref[11]),.O_y(W_ref[10:3]),.O_edge(W_ref[2]),.O_mask(W_ref[1]),.O_window_valid(W_ref[0]));
    integer S_checks=0,S_math_checks=0;
    always @(posedge S_clk) begin
        #1;
        if(W_new !== W_ref) $fatal(1,"raster miter mode=%0d cycle=%0d new=%h ref=%h",S_mode,S_checks,W_new,W_ref);
        S_checks=S_checks+1;
    end
    function automatic [7:0] f_math_flat(input integer rgb,input integer level);
        integer r,g,b,y,dr,dg,db,rr,gg,bb;
        begin
            r=(((rgb>>5)&7)*255+3)/7;
            g=(((rgb>>2)&7)*255+3)/7;
            b=(rgb&3)*85;
            y=(77*r+150*g+29*b)/256;
            dr=(r-y)>>>4; dg=(g-y)>>>4; db=(b-y)>>>4;
            rr=level*32+dr*16; gg=level*32+dg*16; bb=level*32+db*16;
            if(rr<0)rr=0;else if(rr>255)rr=255;
            if(gg<0)gg=0;else if(gg>255)gg=255;
            if(bb<0)bb=0;else if(bb>255)bb=255;
            f_math_flat=((rr>>5)<<5)|((gg>>5)<<2)|(bb>>6);
        end
    endfunction
    task automatic blank(input integer count);
        integer k;
        for(k=0;k<count;k=k+1) begin
            @(negedge S_clk); S_vs=0;S_hs=0;S_de=0;S_user=0;S_last=0;S_y=0;
        end
    endtask
    task automatic line(input integer row,input integer width,input integer seed);
        integer x;
        for(x=0;x<width;x=x+1) begin
            @(negedge S_clk);S_vs=(row==0&&x==0);S_hs=1;S_de=1;S_user=(x==0);S_last=(x==width-1);
            S_y=(x+row*43+seed*17)%256;
        end
    endtask
    integer S_rgb,S_level,S_m,S_row;
    reg [14:0] S_delta;
    reg [7:0] S_flat,S_old,S_math;
    initial begin
        for(S_rgb=0;S_rgb<256;S_rgb=S_rgb+1) begin
            S_delta=u_new.rgb332_chroma_delta(S_rgb[7:0]);
            for(S_level=0;S_level<8;S_level=S_level+1) begin
                S_flat=u_new.rgb332_flat_delta(S_level*32,$signed(S_delta[14:10]),$signed(S_delta[9:5]),$signed(S_delta[4:0]));
                S_old=u_ref.rgb332_flat(S_rgb[7:0],S_level*32);
                S_math=f_math_flat(S_rgb,S_level);
                if(S_flat!==S_old || S_flat!==S_math)
                    $fatal(1,"math RGB=%0d level=%0d new=%h old=%h math=%h",S_rgb,S_level,S_flat,S_old,S_math);
                S_math_checks=S_math_checks+1;
            end
        end
        repeat(4)@(negedge S_clk);S_rst_n=1;
        for(S_m=0;S_m<20;S_m=S_m+1) begin
            S_mode=S_m;
            for(S_row=0;S_row<6;S_row=S_row+1) begin
                line(S_row,IMG_W,S_m);
                if(S_row%2==0)blank(3+S_row);
            end
            blank(7);S_bin_th=S_bin_th+9;S_cartoon_th=S_cartoon_th+13;
        end
        // In-flight controls and addresses must reset with the chroma stage.
        S_mode=18;line(0,IMG_W/2,77);
        @(negedge S_clk);S_rst_n=0;S_de=0;S_user=0;S_last=0;
        blank(3);S_rst_n=1;
        for(S_row=0;S_row<6;S_row=S_row+1)begin line(S_row,IMG_W,78);blank(S_row);end
        blank(30);
        $display("PASS chroma equivalence W=%0d raster_checks=%0d RGB_level_checks=%0d modes=20",IMG_W,S_checks,S_math_checks);
        $finish;
    end
endmodule
