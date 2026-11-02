`timescale 1ns/1ps
module tb_ink_capture_raster;
    reg clk=0;always #7 clk=~clk;
    reg rst=0,ink=0,de=0,vs=1,detected=1;
    reg [6:0] charvalid=7'h7f;reg [76:0] left=0,right=0;
    reg nextbank=1,accept=0;
    wire we,wb,commit,cb,ready,eof,recover,quiet;
    wire [7:0] addr,data;wire [8:0] length;wire [31:0] epoch,fid,drops;
    reg [7:0] banks[0:511];
    integer metas=0,lines=0,eofs=0,aborts=0,eofpulses=0;
    integer row=0,x=0,index=0,base=0,lineid=0,value=0,frame=0;
    ctl_master_ink_capture dut(.I_clk_app(clk),.I_rst_app_n(rst),.I_ink(ink),.I_ink_de(de),.I_vsync(vs),
        .I_detected(detected),.I_char_up(11'd10),.I_char_down(11'd50),.I_char_valid(charvalid),
        .I_char_left(left),.I_char_right(right),.I_session_ready(1'b1),.I_epoch(32'h12345678),
        .I_control_valid(1'b0),.I_control_kind(8'd0),.I_control_epoch(32'd0),.I_control_challenge(32'd0),
        .O_control_ready(ready),.I_free_bank_mask(2'b11),.I_next_commit_bank(nextbank),.I_transport_quiet(1'b1),
        .O_write_en(we),.O_write_bank(wb),.O_write_addr(addr),.O_write_data(data),
        .O_commit(commit),.O_commit_bank(cb),.O_packet_length(length),.I_commit_accept(accept),
        .O_session_recover(recover),.O_eof_committed(eof),.O_eof_epoch(epoch),.O_eof_frame_index(fid),
        .O_frame_drop_count(drops),.O_source_quiet(quiet));
    always @(posedge clk)begin
        accept<=0;
        if(we)banks[{wb,addr}]<=data;
        if(commit)begin
            if(cb!==nextbank)$fatal(1,"commit ordering");
            base=cb*256;
            if({banks[base],banks[base+1],banks[base+2]}!==24'h525301)$fatal(1,"header");
            if({banks[base+4],banks[base+5],banks[base+6],banks[base+7]}!==32'h12345678)$fatal(1,"epoch");
            if(banks[base+14]!==0)$fatal(1,"reserved");
            case(banks[base+3])
                1:begin
                    if(length!=52 || banks[base+13]!=36 || banks[base+15]!={charvalid,detected})$fatal(1,"META length/flags");
                    if({banks[base+16],banks[base+17],banks[base+18],banks[base+19]}!==32'h050002d0)$fatal(1,"dimensions");
                    for(index=0;index<7;index=index+1)begin
                        if({banks[base+24+4*index],banks[base+25+4*index]}!==index*20 ||
                           {banks[base+26+4*index],banks[base+27+4*index]}!==index*20+19)$fatal(1,"geometry lane %0d",index);
                    end
                    metas=metas+1;
                end
                2:begin
                    lineid={banks[base+16],banks[base+17]};
                    if(length!=178 || lineid!==lines%720)$fatal(1,"line order %0d",lineid);
                    for(index=0;index<160;index=index+1)begin
                        value=detected?((lineid*37+index*13+5)&255):0;
                        if(banks[base+18+index]!==value)$fatal(1,"line bytes %0d %0d",lineid,index);
                    end
                    lines=lines+1;
                end
                3:begin if(length!=16 || lines%720)$fatal(1,"partial EOF");eofs=eofs+1;end
                4:begin if(length!=16)$fatal(1,"ABORT length");aborts=aborts+1;end
                default:$fatal(1,"unexpected kind");
            endcase
            accept<=1;nextbank<=~nextbank;
        end
        if(eof)begin
            if(epoch!==32'h12345678 || fid!==eofpulses)$fatal(1,"EOF ownership");
            eofpulses=eofpulses+1;
        end
    end
    initial begin
        for(index=0;index<7;index=index+1)begin left[index*11+:11]=index*20;right[index*11+:11]=index*20+19;end
        repeat(3)@(negedge clk);rst=1;repeat(4)@(negedge clk);
        for(frame=0;frame<2;frame=frame+1)begin
            detected=frame==0;charvalid=frame==0?7'h7f:0;
            vs=0;repeat(70)@(negedge clk);
            for(row=0;row<720;row=row+1)begin
                for(x=0;x<1280;x=x+1)begin
                    de=1;value=detected?((row*37+(x/8)*13+5)&255):0;
                    ink=(value>>(7-x%8))&1;@(negedge clk);
                end
                de=0;repeat(70)@(negedge clk);
            end
            vs=1;repeat(100)@(negedge clk);
        end
        if(metas!=2 || lines!=1440 || eofs!=2 || eofpulses!=2 || drops || recover)$fatal(1,"full/empty frame packet totals");
        detected=1;charvalid=7'h7f;vs=0;repeat(70)@(negedge clk);
        de=1;repeat(319)@(negedge clk);de=0;repeat(100)@(negedge clk);
        vs=1;repeat(100)@(negedge clk);
        if(aborts!=1 || drops!=1 || eofs!=2 || eofpulses!=2)$fatal(1,"short frame accepted");
        $display("TB_RESULT: PASS capture 1843200 source pixels/1440 golden LINE/META flags geometry/EOF ownership/partial ABORT");$finish;
    end
    initial begin #100000000;$fatal(1,"timeout");end
endmodule
