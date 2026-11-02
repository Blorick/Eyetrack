`timescale 1ns/1ps
module tb_fill_policy;
    localparam integer REAL_STEP = 4761905;
    localparam integer FAST_STEP = 100001;
    reg clk = 0;
    always #21 clk = ~clk;
    reg real_rst_n = 0, fast_rst_n = 0;
    reg [7:0] y = 100;
    wire real_ws, fast_ws, twin_ws;
    wire [3:0] real_level, fast_level, twin_level;
    integer checks = 0, fast_changes = 0;
    reg real_done = 0, gap_done = 0, fast_done = 0;
    integer previous_level = 0;
    integer since_change = 0;
    integer fast_cycle = 0;
    integer last_change = 0;
    dsp_fill_light production (
        .I_clk(clk), .I_rst_n(real_rst_n), .I_y_avg(8'd0),
        .I_y_valid(1'b0), .O_ws(real_ws), .O_level(real_level));
    dsp_fill_light #(.CLK_HZ(500000)) accelerated (
        .I_clk(clk), .I_rst_n(fast_rst_n), .I_y_avg(y),
        .I_y_valid(1'b0), .O_ws(fast_ws), .O_level(fast_level));
    dsp_fill_light #(.CLK_HZ(500000)) valid_twin (
        .I_clk(clk), .I_rst_n(fast_rst_n), .I_y_avg(y),
        .I_y_valid(1'b1), .O_ws(twin_ws), .O_level(twin_level));

    task automatic check(input bit passed, input string label_text);
        begin
            checks = checks + 1;
            if (!passed) $fatal(1, "policy %s time=%0t level=%0d y=%0d", label_text, $time, fast_level, y);
        end
    endtask
    task automatic clocks(input integer n);
        begin repeat(n) @(posedge clk); #1; end
    endtask
    task automatic await_level(input integer wanted);
        integer timeout;
        begin
            timeout = 0;
            while (fast_level != wanted && timeout < FAST_STEP * 12) begin
                @(posedge clk); #1;
                timeout = timeout + 1;
            end
            check(fast_level == wanted, "luma settles to intended target level");
        end
    endtask
    task automatic set_y(input [7:0] value);
        begin @(negedge clk); y = value; end
    endtask

    // Compare public outputs, proving the compatibility-valid input has no
    // effect. Monitor every level transition independently of the stimulus.
    always @(posedge clk) begin
        #1;
        if (!fast_rst_n) begin
            previous_level = 0;
            fast_cycle = 0;
            last_change = 0;
        end else begin
            fast_cycle = fast_cycle + 1;
            if (fast_level !== twin_level || fast_ws !== twin_ws)
                $fatal(1, "I_y_valid changed public outputs");
            if (fast_level != previous_level) begin
                check((fast_level == previous_level+1) || (fast_level == previous_level-1), "only one level per transition");
                check(fast_cycle-last_change >= FAST_STEP, "no level changes faster than step interval");
                previous_level = fast_level;
                last_change = fast_cycle;
                fast_changes = fast_changes + 1;
            end
        end
    end

    initial begin
        clocks(5);
        @(negedge clk); real_rst_n = 1;
        clocks(REAL_STEP-1);
        check(real_level == 0, "production level stays zero before full200ms");
        clocks(1);
        check(real_level == 1, "production first step at4761905clocks");
        clocks(1000);
        check(real_level == 1, "production level cannot immediately add again");
        real_done = 1;
    end
    // Decode the first two default frames while production level is still0.
    // This observes the actual10ms interframe reset gap and complete bit cells.
    initial begin : default_wire_check
        integer frame, b;
        time rise_at, previous_rise, high_ns, last_fall;
        wait(real_rst_n);
        last_fall = 0;
        for (frame=0; frame<2; frame=frame+1) begin
            previous_rise = 0;
            for (b=0; b<1536; b=b+1) begin
                @(posedge real_ws);
                rise_at = $time;
                if (b > 0) check(rise_at-previous_rise == 1260, "default bit period1260ns");
                if (b == 0 && frame > 0) begin
                    check(rise_at-last_fall >= 9999780, "production gap includes10msreset");
                    check(rise_at-last_fall < 10002000, "production gap has expected length");
                end
                previous_rise = rise_at;
                @(negedge real_ws);
                high_ns = $time-rise_at;
                check(high_ns == 294, "initial default frames are complete black payloads");
                last_fall = $time;
            end
        end
        gap_done = 1;
    end
    initial begin
        clocks(5);
        @(negedge clk); fast_rst_n = 1;
        clocks(FAST_STEP);
        check(fast_level == 0, "bright scene remains off");
        set_y(54); await_level(1);
        set_y(47); await_level(2);
        set_y(44); await_level(3);
        set_y(41); await_level(4);
        set_y(38); await_level(5);
        set_y(35); await_level(6);
        set_y(32); await_level(7);
        set_y(0);  await_level(8);
        clocks(2*FAST_STEP);
        check(fast_level == 8, "dark scene saturates atlevel8");
        // Preserve the supplied fixed-point filter's boundary residual:
        // approaching raw60 from below leaves the filtered integer at59.
        set_y(60); await_level(1);
        clocks(2*FAST_STEP);
        check(fast_level == 1, "raw60 frombelow retainsfiltered59boundary");
        set_y(61); await_level(0);
        set_y(59); await_level(1);
        check(fast_level == 1, "fractionalIIR moves on changes smallerthan64");
        set_y(255); await_level(0);
        clocks(FAST_STEP);
        check(fast_level == 0, "bright scene fully decays tooff");
        @(negedge clk); fast_rst_n = 0;
        #1;
        check(fast_level == 0 && fast_ws == 0, "asynchronous reset clearslevelandDIN");
        fast_done = 1;
    end
    initial begin
        wait(real_done && fast_done && gap_done);
        $display("TB_RESULT: PASS policy checks=%0d transitions=%0d real_interval_cycles=4761905 real_interval_ns=200000010 default_wire_bits=3072",checks,fast_changes);
        $finish;
    end
    initial begin #220000000; $fatal(1,"policy timeout"); end
endmodule
