`timescale 1ns/1ps
module tb_fill_wire;
    reg clk = 0;
    always #21 clk = ~clk;
    reg rst_n = 0;
    wire ws;
    wire [3:0] level;
    integer checks = 0, frames = 0, stable_frames = 0, bits = 0;
    reg [8:0] seen = 0;
    integer expected_counts [0:8];
    dsp_fill_light #(.CLK_HZ(1000000)) dut (
        .I_clk(clk), .I_rst_n(rst_n), .I_y_avg(8'd0),
        .I_y_valid(1'b0), .O_ws(ws), .O_level(level));

    function automatic bit on_at(input integer index, input integer lv);
        integer row, col, x, distance;
        begin
            row = index / 8;
            col = index % 8;
            x = (row % 2) ? 7-col : col;
            distance = ((x>=3)?x-3:3-x) + ((row>=3)?row-3:3-row);
            on_at = lv != 0 && distance <= lv;
        end
    endfunction
    task automatic check(input bit passed, input string label_text);
        begin
            checks=checks+1;
            if(!passed) $fatal(1,"wire %s frame=%0d level=%0d time=%0t",label_text,frames,level,$time);
        end
    endtask
    initial begin : decode
        integer b, p, lv_start, lit;
        reg [23:0] word_value;
        reg [23:0] pixels[0:63];
        time rise_at, previous_rise, high_ns, last_fall;
        expected_counts[0]=0; expected_counts[1]=5; expected_counts[2]=13;
        expected_counts[3]=25; expected_counts[4]=39; expected_counts[5]=51;
        expected_counts[6]=59; expected_counts[7]=63; expected_counts[8]=64;
        repeat(5) @(negedge clk);
        rst_n=1;
        last_fall=0;
        while(seen != 9'b111111111) begin
            word_value=0;
            previous_rise=0;
            for(b=0;b<1536;b=b+1) begin
                @(posedge ws);
                rise_at=$time;
                if(b==0) begin
                    lv_start=level;
                    if(frames>0) begin
                        check(rise_at-last_fall>=420000,"scaled resetlow>=420us");
                        check(rise_at-last_fall<422000,"scaled resetlow bound");
                    end
                end else check(rise_at-previous_rise==1260,"bit period30clocks");
                previous_rise=rise_at;
                @(negedge ws);
                high_ns=$time-rise_at;
                check(high_ns==294 || high_ns==756,"zero/one highpulse7/18clocks");
                word_value={word_value[22:0],(high_ns==756)};
                if(b%24==23) pixels[b/24]=word_value;
                last_fall=$time;
                bits=bits+1;
            end
            // The original controller samples level perLED, not perframe.
            // Only compare geometry on frames whose level stayed constant.
            if(level==lv_start) begin
                lit=0;
                for(p=0;p<64;p=p+1) begin
                    check(pixels[p] === (on_at(p,lv_start)?24'h808080:24'h000000),"serpentine diamond payloadGRB808080");
                    if(pixels[p]==24'h808080) lit=lit+1;
                end
                check(lit==expected_counts[lv_start],"independent expected LEDcount");
                seen[lv_start]=1;
                stable_frames=stable_frames+1;
            end
            frames=frames+1;
        end
        @(posedge ws);
        #100 rst_n=0;
        #1;
        check(ws==0 && level==0,"asynchronous reset interrupts stream");
        $display("TB_RESULT: PASS wire checks=%0d frames=%0d stable_frames=%0d checked_bits=%0d levels_seen=%b",checks,frames,stable_frames,bits,seen);
        $finish;
    end
    initial begin #100000000; $fatal(1,"wire timeout"); end
endmodule
