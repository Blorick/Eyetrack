`timescale 1ns/1ps
module tb_ink_decoder_raster;
    reg clk=0;always #7 clk=~clk;
    reg rst=0,packet_valid=0,session_ready=1,barrier_abort=0,sampler_ready=1;
    reg geometry_ready=1,geometry_stall=0,vs_q=1;
    integer geometry_delay=0,geometry_generation=0;
    reg [7:0] packet[0:255],read_data=0;wire [7:0] read_addr;
    wire release_packet,hello,ink,de,vs,abort_frame,detected,complete,quiet;
    wire [31:0] helloepoch,hellochallenge,fid,bad;wire [6:0] charvalid;
    wire [10:0] up,down;wire [76:0] left,right;
    integer pixels=0,frames=0,aborts=0,line=0,i=0,expected_byte=0;
    reg check_pixels=0;
    ctl_slave_ink_decoder dut(.I_clk_app(clk),.I_rst_app_n(rst),.I_packet_valid(packet_valid),
        .O_packet_read_addr(read_addr),.I_packet_read_data(read_data),.O_packet_release(release_packet),
        .I_session_ready(session_ready),.I_epoch(32'h12345678),.I_barrier_abort(barrier_abort),
        .I_sampler_frame_ready(sampler_ready),.I_sampler_geometry_ready(geometry_ready),
        .O_hello_seen(hello),.O_hello_epoch(helloepoch),
        .O_hello_challenge(hellochallenge),.O_ink(ink),.O_ink_de(de),.O_vsync(vs),
        .O_frame_abort(abort_frame),.O_detected(detected),.O_char_valid(charvalid),
        .O_char_up(up),.O_char_down(down),.O_char_left(left),.O_char_right(right),
        .O_external_frame_index(fid),.O_complete_frame(complete),.O_bad_sequence_count(bad),.O_raw_quiet(quiet));
    always @(posedge clk)begin
        read_data<=packet[read_addr];
        vs_q<=vs;
        if(vs_q && !vs)begin
            geometry_ready<=0;geometry_delay<=11;geometry_generation<=geometry_generation+1;
        end else if(geometry_delay>0 && !geometry_stall)begin
            geometry_delay<=geometry_delay-1;
            if(geometry_delay==1)geometry_ready<=1;
        end
        if(release_packet)packet_valid<=0;
        if(de && !geometry_ready)$fatal(1,"pixels before current geometry ready");
        if(release_packet && packet[3]==1 && session_ready && !abort_frame && !geometry_ready && dut.S_frame_active)
            $fatal(1,"META released before current geometry ready");
        if(de && check_pixels)begin
            expected_byte=((pixels/1280)*37+((pixels%1280)/8)*13+5)&255;
            if(ink!==((expected_byte>>(7-pixels%8))&1))$fatal(1,"pixel golden at %0d",pixels);
            pixels=pixels+1;
        end
        if(complete)frames=frames+1;
        if(abort_frame)aborts=aborts+1;
    end
    task header(input [7:0] kind,input [31:0] frame_id,input integer length);
        begin
            for(i=0;i<256;i=i+1)packet[i]=0;
            packet[0]=8'h52;packet[1]=8'h53;packet[2]=1;packet[3]=kind;
            packet[4]=8'h12;packet[5]=8'h34;packet[6]=8'h56;packet[7]=8'h78;
            packet[8]=frame_id[31:24];packet[9]=frame_id[23:16];packet[10]=frame_id[15:8];packet[11]=frame_id[7:0];
            packet[12]=length>>8;packet[13]=length;
        end
    endtask
    task offer;
        begin
            @(negedge clk);packet_valid=1;
            wait(release_packet);@(negedge clk);
            wait(dut.S_state==0);@(negedge clk);
        end
    endtask
    task meta(input [31:0] frame_id,input [7:0] flags);
        begin
            header(1,frame_id,36);packet[15]=flags;
            packet[16]=5;packet[17]=0;packet[18]=2;packet[19]=8'hd0;
            packet[21]=10;packet[23]=50;
            for(i=0;i<7;i=i+1)begin packet[25+4*i]=i*20;packet[27+4*i]=i*20+19;end
            offer;
        end
    endtask
    task rawline(input [31:0] frame_id,input integer n);
        begin
            header(2,frame_id,162);packet[16]=n>>8;packet[17]=n;
            for(i=0;i<160;i=i+1)packet[18+i]=(n*37+i*13+5)&255;
            offer;
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;
        meta(1,8'hff);
        if(!detected || charvalid!=7'h7f || up!=10 || down!=50 || fid!=1 || vs)$fatal(1,"META geometry flags/FID");
        pixels=0;check_pixels=1;
        for(line=0;line<720;line=line+1)rawline(1,line);
        header(3,1,0);offer;check_pixels=0;
        if(pixels!=921600 || frames!=1 || !vs || bad!=0)$fatal(1,"complete frame");
        // Empty geometry remains a complete raster/history update.
        meta(2,0);pixels=0;check_pixels=1;
        for(line=0;line<720;line=line+1)rawline(2,line);
        header(3,2,0);offer;check_pixels=0;
        if(pixels!=921600 || frames!=2 || detected || charvalid)$fatal(1,"empty complete frame");
        meta(3,8'hff);rawline(3,0);rawline(3,0);
        header(3,3,0);offer;
        if(frames!=2 || !vs || bad<2)$fatal(1,"duplicate line/partial EOF completed");
        meta(4,8'hff);rawline(4,1);
        if(frames!=2 || !vs)$fatal(1,"out-of-order line");
        header(1,5,36);packet[15]=8'hff;packet[16]=5;packet[18]=2;packet[19]=8'hd0;
        packet[21]=50;packet[23]=10;offer;
        if(frames!=2 || !vs)$fatal(1,"bad META geometry");
        // A barrier releases a META held for sampler readiness.
        sampler_ready=0;header(1,6,36);packet[16]=5;packet[18]=2;packet[19]=8'hd0;
        @(negedge clk);packet_valid=1;wait(dut.S_state==5);
        @(negedge clk);session_ready=0;barrier_abort=1;
        @(negedge clk);barrier_abort=0;
        wait(quiet);@(negedge clk);
        if(packet_valid || !vs || de)$fatal(1,"barrier held META deadlock");
        // Current preparation may stall after fall; no old held ready or
        // incoming LINE can escape it, and HELLO barrier can still cancel it.
        session_ready=1;sampler_ready=1;geometry_stall=1;
        header(1,7,36);packet[16]=5;packet[18]=2;packet[19]=8'hd0;
        @(negedge clk);packet_valid=1;wait(dut.S_state==12);
        repeat(12)@(negedge clk);
        if(!packet_valid || de || release_packet || geometry_ready || vs)$fatal(1,"geometry generation wait bypass");
        session_ready=0;barrier_abort=1;
        @(negedge clk);barrier_abort=0;geometry_stall=0;
        wait(quiet);@(negedge clk);
        header(5,32'hdeadbeef,0);offer;
        if(helloepoch!==32'h12345678 || hellochallenge!==32'hdeadbeef)$fatal(1,"HELLO barrier decode");
        $display("TB_RESULT: PASS decoder 1843200 golden pixels META/FID full-empty/duplicate/outorder/current geometry low-high/prep stall/barrier HELLO");$finish;
    end
    initial begin #100000000;$fatal(1,"timeout");end
endmodule
