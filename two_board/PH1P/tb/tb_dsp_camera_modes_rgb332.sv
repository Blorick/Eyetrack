`timescale 1ns/1ps
// tb_dsp_camera_modes_rgb332 -- mode-18 RGB332 unpack contract
//
// The returned byte is expanded by preserving the source bits in the high
// positions and filling the unused low positions with zero.  This explicitly
// rejects the old bit-replication interpretation.
module tb_dsp_camera_modes_rgb332;
    localparam integer P_WIDTH = 4;
    localparam integer P_HDR = 16;
    localparam integer P_FRAME_LEN = P_HDR + P_WIDTH;
    localparam [4:0] P_MODE = 5'd18;

    reg S_clk = 1'b0;
    always #5 S_clk = ~S_clk;

    reg I_rst_n;
    reg [4:0] I_mode;
    reg signed [8:0] I_brightness;
    reg [7:0] I_sat_factor;
    reg I_vsync;
    reg I_hsync;
    reg I_de;
    reg I_user;
    reg I_last;
    reg [23:0] I_video;
    reg I_rx_valid;
    reg [15:0] I_rx_length;
    wire [10:0] O_rx_address;
    reg [7:0] I_rx_data;
    wire O_rx_release;
    reg I_tx_ready;
    wire O_tx_write;
    wire [10:0] O_tx_address;
    wire [7:0] O_tx_data;
    wire O_tx_commit;
    wire O_vsync;
    wire O_hsync;
    wire O_de;
    wire O_user;
    wire O_last;
    wire [23:0] O_video;
    wire O_link_ok;
    wire O_remote_used;

    reg [7:0] S_rx_mem [0:2047];
    reg S_rx_start;
    reg [15:0] S_rx_start_len;
    reg [23:0] S_remote_values [0:7];
    integer S_release_count;
    integer S_remote_count;
    integer S_checks;
    integer S_errors;
    integer S_line;
    integer S_col;
    integer S_i;

    dsp_camera_modes #(
        .P_WIDTH          ( P_WIDTH ),
        .P_REMOTE_LATENCY( 6        )
    ) dut (
        .I_clk_pix      ( S_clk          ),
        .I_rst_n        ( I_rst_n        ),
        .I_mode         ( I_mode         ),
        .I_brightness   ( I_brightness   ),
        .I_sat_factor   ( I_sat_factor   ),
        .I_vsync        ( I_vsync        ),
        .I_hsync        ( I_hsync        ),
        .I_de           ( I_de           ),
        .I_user         ( I_user         ),
        .I_last         ( I_last         ),
        .I_video        ( I_video        ),
        .I_rx_valid     ( I_rx_valid     ),
        .I_rx_length    ( I_rx_length    ),
        .O_rx_address   ( O_rx_address   ),
        .I_rx_data      ( I_rx_data      ),
        .O_rx_release   ( O_rx_release   ),
        .I_tx_ready     ( I_tx_ready     ),
        .O_tx_write     ( O_tx_write     ),
        .O_tx_address   ( O_tx_address   ),
        .O_tx_data      ( O_tx_data      ),
        .O_tx_commit    ( O_tx_commit    ),
        .O_vsync        ( O_vsync        ),
        .O_hsync        ( O_hsync        ),
        .O_de           ( O_de           ),
        .O_user         ( O_user         ),
        .O_last         ( O_last         ),
        .O_video        ( O_video        ),
        .O_link_ok      ( O_link_ok      ),
        .O_remote_used  ( O_remote_used  )
    );

    // Registered RX memory model used by the production bridge contract.
    always @(posedge S_clk) begin
        I_rx_data <= S_rx_mem[O_rx_address];
        if (S_rx_start) begin
            I_rx_valid <= 1'b1;
            I_rx_length <= S_rx_start_len;
            S_rx_start <= 1'b0;
        end else begin
            I_rx_valid <= 1'b0;
            if (O_rx_release) I_rx_length <= 16'd0;
        end
        if (O_rx_release) S_release_count = S_release_count + 1;
        #1;
        if (O_remote_used && O_de) begin
            if (S_remote_count < 8)
                S_remote_values[S_remote_count] = O_video;
            S_remote_count = S_remote_count + 1;
        end
    end

    task automatic pulse_frame;
        begin
            @(negedge S_clk);
            I_vsync = 1'b1;
            I_hsync = 1'b0;
            I_de = 1'b0;
            I_user = 1'b0;
            I_last = 1'b0;
            @(posedge S_clk);
            @(negedge S_clk);
            I_vsync = 1'b0;
        end
    endtask

    task automatic arm_rx_packet;
        begin
            for (S_i = 0; S_i < 2048; S_i = S_i + 1)
                S_rx_mem[S_i] = 8'd0;
            S_rx_mem[0] = 8'hA5;
            S_rx_mem[1] = 8'h01;
            S_rx_mem[2] = 8'd0;
            S_rx_mem[3] = 8'd0;
            S_rx_mem[4] = {3'd0, P_MODE};
            S_rx_mem[6] = P_WIDTH[7:0];
            S_rx_mem[7] = 8'd0;
            S_rx_mem[8] = 8'd1;
            S_rx_mem[9] = 8'd0;
            S_rx_mem[10] = 8'd0;
            S_rx_mem[11] = 8'd0;
            S_rx_mem[12] = 8'h01;
            // E3 -> E0_00_C0, 1C -> 00_E0_00,
            // A5 -> A0_20_40, 00 -> 00_00_00.
            S_rx_mem[P_HDR + 0] = 8'hE3;
            S_rx_mem[P_HDR + 1] = 8'h1C;
            S_rx_mem[P_HDR + 2] = 8'hA5;
            S_rx_mem[P_HDR + 3] = 8'h00;
            S_rx_start_len = P_FRAME_LEN;
            S_rx_start = 1'b1;
        end
    endtask

    task automatic drive_line;
        input V_user;
        begin
            for (S_col = 0; S_col < P_WIDTH; S_col = S_col + 1) begin
                @(negedge S_clk);
                I_mode = P_MODE;
                I_de = 1'b1;
                I_user = V_user && (S_col == 0);
                I_last = (S_col == P_WIDTH - 1);
                I_video = 24'h123456;
                @(posedge S_clk);
            end
            @(negedge S_clk);
            I_de = 1'b0;
            I_user = 1'b0;
            I_last = 1'b0;
            @(posedge S_clk);
        end
    endtask

    initial begin
        I_rst_n = 1'b0;
        I_mode = P_MODE;
        I_brightness = 9'sd0;
        I_sat_factor = 8'd128;
        I_vsync = 1'b0;
        I_hsync = 1'b0;
        I_de = 1'b0;
        I_user = 1'b0;
        I_last = 1'b0;
        I_video = 24'h123456;
        I_rx_valid = 1'b0;
        I_rx_length = 16'd0;
        I_rx_data = 8'd0;
        I_tx_ready = 1'b1;
        S_rx_start = 1'b0;
        S_rx_start_len = 16'd0;
        S_release_count = 0;
        S_remote_count = 0;
        S_checks = 0;
        S_errors = 0;
        for (S_i = 0; S_i < 8; S_i = S_i + 1)
            S_remote_values[S_i] = 24'd0;
        for (S_i = 0; S_i < 2048; S_i = S_i + 1)
            S_rx_mem[S_i] = 8'd0;

        repeat (4) @(posedge S_clk);
        I_rst_n = 1'b1;
        repeat (2) @(posedge S_clk);

        pulse_frame();
        // P_LAT=6: the response for line zero is displayed on line six.
        arm_rx_packet;
        drive_line(1'b1);
        for (S_line = 1; S_line <= 6; S_line = S_line + 1)
            drive_line(1'b0);
        repeat (20) @(posedge S_clk);

        S_checks = S_checks + 1;
        if (S_release_count == 0) begin
            $display("RGB332_TB_ERROR response packet was not released");
            S_errors = S_errors + 1;
        end
        S_checks = S_checks + 1;
        if (S_remote_count < P_WIDTH) begin
            $display("RGB332_TB_ERROR remote pixels=%0d expected=%0d", S_remote_count, P_WIDTH);
            S_errors = S_errors + 1;
        end
        S_checks = S_checks + 1;
        if ((S_remote_values[0] !== 24'hE000C0) ||
            (S_remote_values[1] !== 24'h00E000) ||
            (S_remote_values[2] !== 24'hA02040) ||
            (S_remote_values[3] !== 24'h000000)) begin
            $display("RGB332_TB_ERROR got=%06h,%06h,%06h,%06h expected=E000C0,00E000,A02040,000000",
                     S_remote_values[0], S_remote_values[1], S_remote_values[2], S_remote_values[3]);
            S_errors = S_errors + 1;
        end

        if (S_errors == 0)
            $display("TB_RESULT: PASS tb_dsp_camera_modes_rgb332 checks=%0d remote=%0d errors=0", S_checks, S_remote_count);
        else
            $display("TB_RESULT: FAIL tb_dsp_camera_modes_rgb332 checks=%0d remote=%0d errors=%0d", S_checks, S_remote_count, S_errors);
        $finish;
    end
endmodule
