`timescale 1ns/1ps
// tb_ph1p_line_bridge_late -- a response published halfway through a row
// must not switch that row from raw to styled.
module tb_ph1p_line_bridge_late;
    localparam integer P_IMG_W = 64;
    localparam integer P_HDR = 16;
    localparam integer P_FRAME_LEN = P_HDR + P_IMG_W;

    reg S_clk = 1'b0;
    always #5 S_clk = ~S_clk;

    reg I_rst_n;
    reg I_rx_valid;
    reg [15:0] I_rx_len;
    wire [10:0] O_rx_addr;
    reg [7:0] I_rx_rdata;
    wire O_rx_release;
    reg I_tx_ready;
    wire O_tx_we;
    wire [10:0] O_tx_addr;
    wire [7:0] O_tx_wdata;
    wire O_tx_commit;
    reg I_vsync;
    reg I_de;
    reg I_user;
    reg [7:0] I_y;
    reg [4:0] I_mode;
    reg I_enable;
    wire O_vsync;
    wire O_de;
    wire O_user;
    wire [7:0] O_y;
    wire O_styled;
    wire O_link_ok;
    wire [15:0] O_out_line;

    reg [7:0] S_rx_mem [0:2047];
    reg S_rx_start;
    reg [15:0] S_rx_start_len;
    integer S_release_count;
    integer S_line1_pixels;
    integer S_line1_styled;
    integer S_line1_raw_errors;
    integer S_checks;
    integer S_errors;
    reg S_watch_line1;
    integer S_i;
    integer S_col_i;

    ph1p_line_bridge #(
        .IMG_W            ( P_IMG_W ),
        .P_SLOT           ( 4 ),
        .P_LAT            ( 1 ),
        .P_VALIDATE_FRAME ( 1 ),
        .P_REMOTE_ONLY    ( 0 )
    ) dut (
        .I_clk        ( S_clk        ),
        .I_rst_n      ( I_rst_n      ),
        .I_rx_valid   ( I_rx_valid   ),
        .I_rx_len     ( I_rx_len     ),
        .O_rx_addr    ( O_rx_addr    ),
        .I_rx_rdata   ( I_rx_rdata   ),
        .O_rx_release ( O_rx_release ),
        .I_tx_ready   ( I_tx_ready   ),
        .O_tx_we      ( O_tx_we      ),
        .O_tx_addr    ( O_tx_addr    ),
        .O_tx_wdata   ( O_tx_wdata   ),
        .O_tx_commit  ( O_tx_commit  ),
        .I_vsync      ( I_vsync      ),
        .I_de         ( I_de         ),
        .I_user       ( I_user       ),
        .I_y          ( I_y          ),
        .I_mode       ( I_mode       ),
        .I_enable     ( I_enable     ),
        .O_vsync      ( O_vsync      ),
        .O_de         ( O_de         ),
        .O_user       ( O_user       ),
        .O_y          ( O_y          ),
        .O_styled     ( O_styled     ),
        .O_link_ok    ( O_link_ok    ),
        .O_out_line   ( O_out_line   )
    );

    // Registered link memory model: the bridge sees a returned byte two
    // clocks after it presents O_rx_addr, as on the production stream link.
    always @(posedge S_clk) begin
        I_rx_rdata <= S_rx_mem[O_rx_addr];
        if (S_rx_start) begin
            I_rx_valid <= 1'b1;
            I_rx_len <= S_rx_start_len;
            S_rx_start <= 1'b0;
        end else begin
            I_rx_valid <= 1'b0;
            if (O_rx_release) I_rx_len <= 16'd0;
        end
        if (O_rx_release) S_release_count = S_release_count + 1;
        #1;
        if (S_watch_line1 && O_de) begin
            S_line1_pixels = S_line1_pixels + 1;
            if (O_styled) S_line1_styled = S_line1_styled + 1;
            if (O_y !== (8'h40 + S_line1_pixels - 1))
                S_line1_raw_errors = S_line1_raw_errors + 1;
        end
    end

    task automatic pulse_frame;
        begin
            @(negedge S_clk);
            I_vsync = 1'b1;
            I_de = 1'b0;
            I_user = 1'b0;
            @(posedge S_clk);
            @(negedge S_clk);
            I_vsync = 1'b0;
        end
    endtask

    task automatic arm_rx_packet;
        integer S_payload_i;
        begin
            for (S_payload_i = 0; S_payload_i < 2048; S_payload_i = S_payload_i + 1)
                S_rx_mem[S_payload_i] = 8'd0;
            S_rx_mem[0] = 8'hA5;
            S_rx_mem[1] = 8'h01;
            S_rx_mem[2] = 8'd0;
            S_rx_mem[3] = 8'd0;
            S_rx_mem[4] = 8'd2;
            S_rx_mem[6] = P_IMG_W[7:0];
            S_rx_mem[7] = 8'd0;
            S_rx_mem[8] = 8'd1;
            S_rx_mem[9] = 8'd0;
            S_rx_mem[10] = 8'd0;
            S_rx_mem[11] = 8'd0;
            S_rx_mem[12] = 8'h01;
            for (S_payload_i = 0; S_payload_i < P_IMG_W; S_payload_i = S_payload_i + 1)
                S_rx_mem[P_HDR + S_payload_i] = 8'hD0 + S_payload_i[7:0];
            S_rx_start_len = P_FRAME_LEN;
            S_rx_start = 1'b1;
        end
    endtask

    task automatic drive_line;
        input [7:0] V_seed;
        input V_user;
        input V_arm_response;
        begin
            for (S_col_i = 0; S_col_i < P_IMG_W; S_col_i = S_col_i + 1) begin
                @(negedge S_clk);
                I_de = 1'b1;
                I_user = (S_col_i == 0) && V_user;
                I_y = V_seed + S_col_i[7:0];
                I_mode = 5'd2;
                I_tx_ready = 1'b1;
                // Starting the full returned packet at line zero means its
                // publication occurs around column 16 of line one.
                if (V_arm_response && (S_col_i == 0)) arm_rx_packet;
                @(posedge S_clk);
            end
            @(negedge S_clk);
            I_de = 1'b0;
            I_user = 1'b0;
            I_y = 8'd0;
            @(posedge S_clk);
        end
    endtask

    initial begin
        I_rst_n = 1'b0;
        I_rx_valid = 1'b0;
        I_rx_len = 16'd0;
        I_rx_rdata = 8'd0;
        I_tx_ready = 1'b1;
        I_vsync = 1'b0;
        I_de = 1'b0;
        I_user = 1'b0;
        I_y = 8'd0;
        I_mode = 5'd2;
        I_enable = 1'b1;
        S_rx_start = 1'b0;
        S_rx_start_len = 16'd0;
        S_release_count = 0;
        S_line1_pixels = 0;
        S_line1_styled = 0;
        S_line1_raw_errors = 0;
        S_checks = 0;
        S_errors = 0;
        S_watch_line1 = 1'b0;
        for (S_i = 0; S_i < 2048; S_i = S_i + 1)
            S_rx_mem[S_i] = 8'd0;

        repeat (4) @(posedge S_clk);
        I_rst_n = 1'b1;
        repeat (2) @(posedge S_clk);

        pulse_frame();
        // The matching packet starts at the first pixel of line zero and is
        // published after line one has already begun.  Its line-one payload
        // must therefore remain raw for all 64 pixels.
        drive_line(8'h10, 1'b1, 1'b1);
        S_watch_line1 = 1'b1;
        drive_line(8'h40, 1'b0, 1'b0);
        S_watch_line1 = 1'b0;

        S_checks = S_checks + 1;
        if (S_release_count == 0) begin
            $display("LATE_TB_ERROR matching response was not released");
            S_errors = S_errors + 1;
        end
        S_checks = S_checks + 1;
        if (S_line1_pixels != P_IMG_W) begin
            $display("LATE_TB_ERROR line-one pixels=%0d expected=%0d", S_line1_pixels, P_IMG_W);
            S_errors = S_errors + 1;
        end
        S_checks = S_checks + 1;
        if (S_line1_styled != 0) begin
            $display("LATE_TB_ERROR late response styled %0d pixels", S_line1_styled);
            S_errors = S_errors + 1;
        end
        S_checks = S_checks + 1;
        if (S_line1_raw_errors != 0) begin
            $display("LATE_TB_ERROR raw pixels mismatched=%0d", S_line1_raw_errors);
            S_errors = S_errors + 1;
        end

        if (S_errors == 0)
            $display("TB_RESULT: PASS tb_ph1p_line_bridge_late checks=%0d releases=%0d errors=0", S_checks, S_release_count);
        else
            $display("TB_RESULT: FAIL tb_ph1p_line_bridge_late checks=%0d releases=%0d errors=%0d", S_checks, S_release_count, S_errors);
        $finish;
    end
endmodule
