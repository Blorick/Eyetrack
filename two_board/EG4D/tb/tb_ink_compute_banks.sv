`timescale 1ns/1ps
module tb_ink_compute_banks;
    reg clk=0;always #20 clk=~clk;
    reg rst=0,barrier=0,tile_valid=0,snapshot_valid=0,result_ready=0;
    reg [40:0] tile_data=0;reg [71:0] snapshot_data=0;
    wire tile_ready,snapshot_ready,result_valid,invalid,busy,quiet,feature_valid;
    wire [188:0] result_data;wire [7:0] feature_data;wire [6:0] feature_index;
    reg [31:0] rows[0:223];reg [124:0] baseline;
    integer features=0,slot=0,case_id=0,job=0,bank=0,row=0,word=0,x=0;
    ctl_slave_ink_compute dut(.I_clk_app(clk),.I_rst_app_n(rst),.I_barrier(barrier),
        .I_tile_valid(tile_valid),.O_tile_ready(tile_ready),.I_tile_data(tile_data),
        .I_snapshot_valid(snapshot_valid),.O_snapshot_ready(snapshot_ready),.I_snapshot_data(snapshot_data),
        .O_result_valid(result_valid),.I_result_ready(result_ready),.O_result_data(result_data),
        .O_old_owner_invalid(invalid),.O_core_busy(busy),.O_quiet(quiet),
        .O_feature_valid(feature_valid),.O_feature_data(feature_data),.O_feature_index(feature_index));
    function integer pixel(input integer glyph,input integer yy,input integer xx);
        integer ry,rx;
        begin
            ry=yy;if(ry<0)ry=1;if(ry>31)ry=30;
            rx=xx;if(rx<0)rx=1;if(rx>31)rx=30;
            pixel=rows[glyph*32+ry][rx];
        end
    endfunction
    function integer golden(input integer glyph,input integer index);
        integer d,cy,cx,py,px,yy,xx,gx,gy,v,a,total;
        begin
            d=index/32;cy=(index%32)/8;cx=index%8;total=0;
            for(py=0;py<8;py=py+1)for(px=0;px<4;px=px+1)begin
                yy=cy*8+py;xx=cx*4+px;
                gx=pixel(glyph,yy-1,xx+1)-pixel(glyph,yy-1,xx-1)
                    +2*pixel(glyph,yy,xx+1)-2*pixel(glyph,yy,xx-1)
                    +pixel(glyph,yy+1,xx+1)-pixel(glyph,yy+1,xx-1);
                gy=pixel(glyph,yy+1,xx-1)+2*pixel(glyph,yy+1,xx)+pixel(glyph,yy+1,xx+1)
                    -pixel(glyph,yy-1,xx-1)-2*pixel(glyph,yy-1,xx)-pixel(glyph,yy-1,xx+1);
                case(d)0:v=gx;1:v=gy;2:v=gx+gy;default:v=gx-gy;endcase
                a=v;if(a<0)a=-a;
                if(d<2)v=(a*255+3)/6;else v=a*30;
                if(v>255)v=255;
                total=total+v;
            end
            golden=(total+16)/32;
        end
    endfunction
    always @(posedge clk)if(rst && feature_valid && dut.W_feature_ready)begin
        if($isunknown({feature_data,feature_index}))$fatal(1,"unknown feature bank=%0d",bank);
        if(feature_index!==features%128 || feature_data!==golden(features/128,features%128))
            $fatal(1,"feature golden bank=%0d count=%0d index=%0d got=%0d expected=%0d",bank,features,feature_index,feature_data,golden(features/128,features%128));
        features=features+1;
    end
    task write_tile(input integer address,input [31:0] data);
        begin
            @(negedge clk);tile_data={address[8:0],data};tile_valid=1;
            while(!tile_ready)@(negedge clk);
            @(negedge clk);tile_valid=0;
        end
    endtask
    task offer_snapshot(input integer selected_bank,input [31:0] fid);
        begin
            @(negedge clk);snapshot_data={32'h10203040,fid,7'h7f,selected_bank[0]};snapshot_valid=1;
            while(!snapshot_ready)@(negedge clk);
            @(negedge clk);snapshot_valid=0;
        end
    endtask
    initial begin
        for(word=0;word<224;word=word+1)begin
            rows[word]=0;
            for(x=0;x<32;x=x+1)rows[word][x]=((x*11+word*7+(x*word)%17)%23)<11;
        end
        for(case_id=0;case_id<6;case_id=case_id+1)begin
            bank=case_id%2;rst=0;features=0;
            repeat(3)@(negedge clk);rst=1;
            // Initialize the complete tail differently; the DUT cannot write
            // it. Contaminate the other bank with a different known pattern.
            for(word=448;word<512;word=word+1)
                dut.U_feature.tile_ram[word]=(case_id<2)?32'd0:((case_id<4)?(32'h13579bdf^(word*32'h1935ab)):32'hffffffff);
            for(word=0;word<224;word=word+1)
                write_tile((1-bank)*224+word,(case_id<2)?32'd0:((case_id<4)?(32'h2468ace0^(word*32'h7342f)):32'hffffffff));
            for(word=0;word<224;word=word+1)write_tile(bank*224+word,rows[word]);
            offer_snapshot(bank,case_id+1);
            // The next capture may write the inactive bank during arithmetic.
            for(word=0;word<32;word=word+1)write_tile((1-bank)*224+word,32'h5a5a5a5a ^ word);
            wait(result_valid);@(negedge clk);
            if(features!=896 || $isunknown(result_data))$fatal(1,"feature/result completeness");
            if(result_data[188:157]!==32'h10203040 || result_data[156:125]!==case_id+1)$fatal(1,"result tag");
            if(case_id==0)baseline=result_data[124:0];
            else if(result_data[124:0]!==baseline)$fatal(1,"bank/tail affected matcher");
            repeat(7)@(negedge clk);
            if(!result_valid || snapshot_ready || invalid)$fatal(1,"held result ownership");
            result_ready=1;@(negedge clk);result_ready=0;wait(!busy);
        end
        // HELLO cancellation must drain a snapshot even during old arithmetic.
        features=0;offer_snapshot(bank,32'h55);
        wait(feature_valid);@(negedge clk);tile_valid=1;tile_data={bank?9'd224:9'd0,32'hbad0bad0};
        repeat(4)@(negedge clk);
        if(tile_ready)$fatal(1,"owned bank admitted write");
        barrier=1;#1;
        if(!invalid)$fatal(1,"owner cancellation");
        @(negedge clk);tile_valid=0;
        offer_snapshot(1-bank,32'h66);
        if(!snapshot_ready)$fatal(1,"barrier mailbox blocked by busy");
        repeat(4)@(negedge clk);barrier=0;
        wait(!busy);
        if(result_valid)$fatal(1,"cancelled result leaked");
        $display("TB_RESULT: PASS compute 6 banks/tail zero/random/ones cases 5376 independent Sobel features matcher parity cancellation");$finish;
    end
    initial begin #100000000;$fatal(1,"timeout");end
endmodule
