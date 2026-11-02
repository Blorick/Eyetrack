`timescale 1ns/1ps

// Exercise the actual PH1P RGB frame statistic and the existing two-register
// control-domain connection to the unmodified teammate light controller.
// The 42 ns serial clock is real; only level/gap counters are accelerated.
module tb_fill_luma_integration;
    localparam integer PIXELS = 1280 * 720;
    localparam integer STEP_CYCLES = 500000 / 5 + 1;
    reg clk_pix = 0;
    reg clk_cfg = 0;
    always #6.667 clk_pix = ~clk_pix; // 75 MHz, rounded to simulator precision
    always #21 clk_cfg = ~clk_cfg;   // actual 23.809524 MHz control clock

    reg stat_rst_n = 0;
    reg sync_rst_n = 0;
    reg fill_rst_n = 0;
    reg [23:0] video = 0;
    reg de = 0, vsync = 0;
    wire [7:0] y_pix;
    wire y_valid;
    reg [7:0] y_meta = 100, y_cfg = 100;
    wire ws;
    wire [3:0] level;
    integer checks = 0;
    integer frames_seen = 0;
    integer level_changes = 0;
    integer cfg_cycles = 0, last_change = 0;
    reg [3:0] previous_level = 0;

    dsp_luma_stat #(.P_WIDTH(1280), .P_HEIGHT(720)) stat (
        .I_clk_pix(clk_pix), .I_rst_n(stat_rst_n), .I_video(video),
        .I_de(de), .I_vsync(vsync), .O_y_avg(y_pix), .O_valid(y_valid)
    );
    // Exactly the existing top-level destination-domain connection. No AE,
    // gain, telemetry, image-mode selector or Ethernet instance is involved.
    always @(posedge clk_cfg or negedge sync_rst_n) begin
        if (!sync_rst_n) begin
            y_meta <= 8'd100;
            y_cfg <= 8'd100;
        end else begin
            y_meta <= y_pix;
            y_cfg <= y_meta;
        end
    end
    dsp_fill_light #(.CLK_HZ(500000)) fill (
        .I_clk(clk_cfg), .I_rst_n(fill_rst_n), .I_y_avg(y_cfg),
        .I_y_valid(1'b0), .O_ws(ws), .O_level(level)
    );

    task automatic check(input bit ok, input string reason);
        begin
            checks = checks + 1;
            if (!ok) $fatal(1, "luma integration: %s time=%0t y_pix=%0d y_cfg=%0d level=%0d",
                            reason, $time, y_pix, y_cfg, level);
        end
    endtask

    always @(posedge clk_pix) begin
        #0.002;
        if (stat_rst_n && y_valid) frames_seen = frames_seen + 1;
    end
    // Bound public output changes independently of the light's internal state.
    always @(posedge clk_cfg) begin
        #0.003;
        if (!fill_rst_n) begin
            cfg_cycles = 0;
            last_change = 0;
            previous_level = 0;
        end else begin
            cfg_cycles = cfg_cycles + 1;
            if (level != previous_level) begin
                check(level <= 8, "level remains inside 0..8");
                check((level == previous_level + 1) || (previous_level == level + 1),
                      "a level transition changes exactly one step");
                check(cfg_cycles - last_change >= STEP_CYCLES,
                      "level changes are separated by a complete counter interval");
                last_change = cfg_cycles;
                level_changes = level_changes + 1;
            end
            previous_level = level;
        end
    end

    task automatic frame(input [23:0] rgb, input [7:0] expected_average);
        integer p, old_frames;
        reg [7:0] held_average;
        begin
            old_frames = frames_seen;
            held_average = y_pix;
            @(negedge clk_pix);
            video = rgb; de = 0; vsync = 1;
            @(posedge clk_pix); #0.001;
            check(y_valid === 0 && y_pix === held_average,
                  "vsync resynchronization does not publish an incomplete frame");
            @(negedge clk_pix); vsync = 0; de = 1;
            for (p = 0; p < PIXELS; p = p + 1) begin
                @(posedge clk_pix); #0.001;
                if (p == PIXELS-1) begin
                    check(y_valid === 1 && y_pix === expected_average,
                          "full 1280x720 frame publishes the expected statistic");
                end else begin
                    check(y_valid === 0 && y_pix === held_average,
                          "frame average stays held until all valid pixels arrive");
                end
            end
            @(negedge clk_pix); de = 0;
            check(frames_seen == old_frames + 1, "exactly one frame-valid pulse");
            @(posedge clk_pix); #0.001;
            check(y_valid === 0 && y_pix === expected_average,
                  "valid is one cycle and the published average stays latched");
            repeat (3) @(posedge clk_cfg);
            #1;
            check(y_cfg === expected_average,
                  "complete frame statistic reaches the 42 ns domain through both registers");
        end
    endtask

    initial begin : acceptance
        integer n;
        reg [3:0] held_level;
        repeat (5) @(negedge clk_cfg);
        stat_rst_n = 1;
        sync_rst_n = 1;

        // Collect a complete bright frame before releasing the light. This
        // isolates real frame-data behavior from the statistic's reset value.
        frame(24'hffffff, 8'd224);
        check(level === 0 && ws === 0, "light reset remains black while the first frame arrives");
        @(negedge clk_cfg); fill_rst_n = 1;
        repeat (STEP_CYCLES + 1000) @(posedge clk_cfg);
        #1;
        check(level === 0, "bright RGB frame keeps the panel off");

        frame(24'h000000, 8'd0);
        n = 0;
        while (level < 2 && n < 3*STEP_CYCLES) begin
            @(posedge clk_cfg); #1;
            n = n + 1;
        end
        check(level >= 2, "dark RGB frame causes gradual automatic fill without AE or Ethernet");

        // While this frame is collected the old dark value is still held.
        // After publication the real IIR settles and the level decays to zero.
        frame(24'hffffff, 8'd224);
        repeat (1000) @(posedge clk_cfg);
        #1;
        held_level = level;
        check(held_level > 0, "bright feedback starts from an illuminated panel");
        n = 0;
        while (level != 0 && n < 9*STEP_CYCLES) begin
            @(posedge clk_cfg); #1;
            check(level <= held_level, "stable bright feedback never increases fill");
            held_level = level;
            n = n + 1;
        end
        check(level === 0, "bright RGB feedback gradually switches all fill off");
        check(frames_seen == 3 && level_changes >= 4,
              "three real full frames exercise both increasing and decreasing feedback");
        @(negedge clk_cfg); fill_rst_n = 0;
        #1;
        check(level === 0 && ws === 0, "asynchronous light reset pulls DIN and level low");
        $display("TB_RESULT: PASS luma_integration checks=%0d frames=%0d level_changes=%0d ethernet_instances=0",
                 checks, frames_seen, level_changes);
        $finish;
    end
    initial begin
        #80000000;
        $fatal(1, "luma integration timeout");
    end
endmodule
