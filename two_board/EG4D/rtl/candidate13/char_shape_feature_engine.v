`timescale 1ns / 1ps
// Candidate13 binary-glyph feature engine / 候选 C13 二值字形方向特征引擎
//
// The capture wrapper writes complete 32-bit rows through i_tile_write_*.
// Two banks × seven glyphs × 32 rows is 2 * 7 * 32 * 32 = 14336 bits.
// This block reads one 3-pixel Sobel neighbourhood at a time and emits one
// Q0.8 feature byte per accepted cell.  It deliberately has no frame or RGB
// policy; char_shape_recognize owns geometry and frame alignment.
//
// Feature order is direction-major, then cell-row, then cell-column:
//   direction 0..3, cell-row 0..3, cell-column 0..7.
// Each cell is eight rows by four columns.  Horizontal/vertical pixel
// quantization follows the deployed integer reference (half-up constants),
// while the matcher applies ties-to-even only to its Q24 normalization.
module char_shape_feature_engine #(
    parameter integer TILE_WORDS = 448
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        i_tile_write_en,
    input  wire [8:0]  i_tile_write_addr,
    input  wire [31:0] i_tile_write_data,

    input  wire        i_process_start,
    input  wire [2:0]  i_process_slot,
    input  wire        i_process_bank,
    input  wire        i_process_valid,

    output reg         o_feature_valid,
    output reg  [7:0]  o_feature_data,
    input  wire        i_feature_ready,
    output reg         o_engine_done,
    output wire        o_busy,
    output reg  [6:0]  o_feature_index
);

    localparam [3:0] S_IDLE  = 4'd0;
    localparam [3:0] S_LOAD_A = 4'd1;
    localparam [3:0] S_LOAD_B = 4'd2;
    localparam [3:0] S_LOAD_C = 4'd3;
    localparam [3:0] S_PIXEL = 4'd4;
    localparam [3:0] S_EMIT = 4'd5;
    localparam [3:0] S_INVALID = 4'd6;
    localparam [3:0] S_LOAD_WAIT = 4'd7;

    // Pad storage to the complete nine-bit address space, while preserving
    // the 224-word bank stride and TILE_WORDS write boundary.  Unused words
    // are never written.  TD resource reports, rather than the attribute,
    // determine whether this organization actually maps to block RAM.
    localparam integer P_TILE_RAM_DEPTH = (TILE_WORDS < 512) ? 512 : TILE_WORDS;
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [31:0] tile_ram [0:P_TILE_RAM_DEPTH-1];

    reg [3:0] state;
    reg [2:0] process_slot_q;
    reg       process_bank_q;
    reg       process_valid_q;
    reg [1:0] direction_q;
    reg [2:0] cell_row_q;
    reg [3:0] cell_col_q;
    reg [5:0] pixel_x_q;
    reg [5:0] pixel_y_q;
    reg [5:0] cell_pixel_count_q;
    reg [13:0] cell_sum_q;
    reg [8:0] tile_base_q;

    reg [31:0] row_a_q;
    reg [31:0] row_b_q;
    reg [31:0] row_c_q;
    reg [31:0] S_tile_read_data_q;
    reg [31:0] S_next_row_a_q;
    reg [31:0] S_next_row_b_q;

    // The tile layout is exactly two banks x seven slots x 32 rows.  A
    // compact 224-word bank stride keeps the 448 logical words contiguous;
    // concatenating bank+slot+row would create a 512-word address space and
    // overrun the last slot of the second bank.
    wire [8:0] tile_base_input = (i_process_bank ? 9'd224 : 9'd0) +
                                 ({6'd0, i_process_slot} << 5);
    wire [8:0] tile_base_active = tile_base_q;

    // One shared synchronous read address feeds the complete tile RAM.  The
    // address is selected by the row-load state, while S_tile_read_data_q
    // captures the addressed row on the clock edge.  LOAD_B/C latch the
    // preceding row while launching the next read; LOAD_WAIT latches C.
    wire [8:0] W_tile_read_addr =
        (state == S_LOAD_A) ?
            ((pixel_y_q == 6'd0) ?
                (tile_base_active + 9'd1) :
                (tile_base_active + {3'd0, pixel_y_q} - 9'd1)) :
        (state == S_LOAD_B) ?
            (tile_base_active + {3'd0, pixel_y_q}) :
        (state == S_LOAD_C) ?
            ((pixel_y_q == 6'd31) ?
                (tile_base_active + 9'd30) :
                (tile_base_active + {3'd0, pixel_y_q} + 9'd1)) :
        (state == S_PIXEL) && (cell_pixel_count_q[1:0] == 2'd0) ?
            (tile_base_active + {3'd0, pixel_y_q}) :
        (state == S_PIXEL) && (cell_pixel_count_q[1:0] == 2'd1) ?
            ((pixel_y_q == 6'd31) ?
                (tile_base_active + 9'd31) :
                (tile_base_active + {3'd0, pixel_y_q} + 9'd1)) :
        (state == S_PIXEL) && (cell_pixel_count_q[1:0] == 2'd2) ?
            ((pixel_y_q >= 6'd30) ?
                (tile_base_active + 9'd30) :
                (tile_base_active + {3'd0, pixel_y_q} + 9'd2)) :
            tile_base_active;

    wire W_tile_read_en = (state == S_LOAD_A) ||
                          (state == S_LOAD_B) ||
                          (state == S_LOAD_C) ||
                          (state == S_PIXEL);

    assign o_busy = (state != S_IDLE);

    // REFLECT_101 for the fixed 32-pixel tile.  At x=0 the left sample is
    // source x=1; at x=31 the right sample is source x=30.
    function pixel_left;
        input [31:0] row_value;
        input [4:0]  x_value;
        begin
            if (x_value == 5'd0)
                pixel_left = row_value[1];
            else
                pixel_left = row_value[x_value - 5'd1];
        end
    endfunction

    function pixel_right;
        input [31:0] row_value;
        input [4:0]  x_value;
        begin
            if (x_value == 5'd31)
                pixel_right = row_value[30];
            else
                pixel_right = row_value[x_value + 5'd1];
        end
    endfunction

    reg p00;
    reg p01;
    reg p02;
    reg p10;
    reg p12;
    reg p20;
    reg p21;
    reg p22;
    reg signed [4:0] gx_calc;
    reg signed [4:0] gy_calc;
    reg signed [5:0] direction_calc;
    reg        [4:0] abs_calc;
    reg [7:0] pixel_q8_calc;
    reg [13:0] cell_mean_wide;

    // Binary 3x3 Sobel neighborhoods have |gx|,|gy| <= 4 and
    // |gx+/-gy| <= 6 (the proof also keeps entries through magnitude 8 for
    // a conservative diagonal table).  These exact positive half-up values
    // replace a wide multiply/divide datapath in the pixel loop.
    function [7:0] hv_q8_lut;
        input [4:0] magnitude;
        begin
            case (magnitude)
                5'd0: hv_q8_lut = 8'd0;
                5'd1: hv_q8_lut = 8'd43;
                5'd2: hv_q8_lut = 8'd85;
                5'd3: hv_q8_lut = 8'd128;
                5'd4: hv_q8_lut = 8'd170;
                5'd5: hv_q8_lut = 8'd213;
                default: hv_q8_lut = 8'd255;
            endcase
        end
    endfunction

    function [7:0] diagonal_q8_lut;
        input [4:0] magnitude;
        begin
            case (magnitude)
                5'd0: diagonal_q8_lut = 8'd0;
                5'd1: diagonal_q8_lut = 8'd30;
                5'd2: diagonal_q8_lut = 8'd60;
                5'd3: diagonal_q8_lut = 8'd90;
                5'd4: diagonal_q8_lut = 8'd120;
                5'd5: diagonal_q8_lut = 8'd150;
                5'd6: diagonal_q8_lut = 8'd180;
                5'd7: diagonal_q8_lut = 8'd210;
                5'd8: diagonal_q8_lut = 8'd240;
                default: diagonal_q8_lut = 8'd255;
            endcase
        end
    endfunction

    // One Sobel calculation is shared by all four direction passes.  The
    // direction/cell loops are sequential, so there is no 128-way fanout.
    always @(*) begin
        p00 = pixel_left(row_a_q, pixel_x_q[4:0]);
        p01 = row_a_q[pixel_x_q[4:0]];
        p02 = pixel_right(row_a_q, pixel_x_q[4:0]);
        p10 = pixel_left(row_b_q, pixel_x_q[4:0]);
        p12 = pixel_right(row_b_q, pixel_x_q[4:0]);
        p20 = pixel_left(row_c_q, pixel_x_q[4:0]);
        p21 = row_c_q[pixel_x_q[4:0]];
        p22 = pixel_right(row_c_q, pixel_x_q[4:0]);

        gx_calc = 5'sd0;
        if (p02) gx_calc = gx_calc + 5'sd1;
        if (p00) gx_calc = gx_calc - 5'sd1;
        if (p12) gx_calc = gx_calc + 5'sd2;
        if (p10) gx_calc = gx_calc - 5'sd2;
        if (p22) gx_calc = gx_calc + 5'sd1;
        if (p20) gx_calc = gx_calc - 5'sd1;

        gy_calc = 5'sd0;
        if (p20) gy_calc = gy_calc + 5'sd1;
        if (p21) gy_calc = gy_calc + 5'sd2;
        if (p22) gy_calc = gy_calc + 5'sd1;
        if (p00) gy_calc = gy_calc - 5'sd1;
        if (p01) gy_calc = gy_calc - 5'sd2;
        if (p02) gy_calc = gy_calc - 5'sd1;

        case (direction_q)
            2'd0: direction_calc = {gx_calc[4], gx_calc};
            2'd1: direction_calc = {gy_calc[4], gy_calc};
            2'd2: direction_calc = gx_calc + gy_calc;
            default: direction_calc = gx_calc - gy_calc;
        endcase
        if (direction_calc < 6'sd0)
            abs_calc = (~direction_calc[4:0]) + 5'd1;
        else
            abs_calc = direction_calc[4:0];

        if (direction_q < 2) begin
            pixel_q8_calc = hv_q8_lut(abs_calc);
        end else begin
            pixel_q8_calc = diagonal_q8_lut(abs_calc);
        end
    end

    // Output is held until the matcher accepts it.  This is deliberately a
    // registered FSM state rather than a pulse-only side channel.
    always @(*) begin
        o_feature_valid = (state == S_EMIT) && process_valid_q;
        cell_mean_wide = (cell_sum_q + 14'd16) >> 5;
        o_feature_data = cell_mean_wide[7:0];
    end

    // Keep the RAM and its synchronous output in a reset-free clocked block.
    // This is the TD single-port RAM inference form; the FSM reset below only
    // resets control and window state.  The adapter completes every tile write
    // before S_GLYPH_START, so the write and read ports do not overlap in the
    // production transaction.
    always @(posedge clk) begin
        if (i_tile_write_en && ({23'd0, i_tile_write_addr} < TILE_WORDS))
            tile_ram[i_tile_write_addr] <= i_tile_write_data;
        if (W_tile_read_en)
            S_tile_read_data_q <= tile_ram[W_tile_read_addr];
    end

    integer reset_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= S_IDLE;
            process_slot_q <= 3'd0;
            process_bank_q <= 1'b0;
            process_valid_q <= 1'b0;
            direction_q <= 2'd0;
            cell_row_q <= 3'd0;
            cell_col_q <= 4'd0;
            pixel_x_q <= 6'd0;
            pixel_y_q <= 6'd0;
            cell_pixel_count_q <= 6'd0;
            cell_sum_q <= 14'd0;
            tile_base_q <= 9'd0;
            row_a_q <= 32'd0;
            row_b_q <= 32'd0;
            row_c_q <= 32'd0;
            S_next_row_a_q <= 32'd0;
            S_next_row_b_q <= 32'd0;
            o_engine_done <= 1'b0;
            o_feature_index <= 7'd0;
        end else begin
            o_engine_done <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (i_process_start) begin
                        process_slot_q <= i_process_slot;
                        process_bank_q <= i_process_bank;
                        process_valid_q <= i_process_valid &&
                                           (i_process_slot < 3'd7);
                        tile_base_q <= tile_base_input;
                        o_feature_index <= 7'd0;
                        direction_q <= 2'd0;
                        cell_row_q <= 3'd0;
                        cell_col_q <= 4'd0;
                        // Every request starts at the first 32x32 pixel.
                        // Without this explicit reset, the final pixel
                        // coordinates of the prior glyph would leak into
                        // the first cell of the next request.
                        pixel_x_q <= 6'd0;
                        pixel_y_q <= 6'd0;
                        cell_pixel_count_q <= 6'd0;
                        cell_sum_q <= 14'd0;
                        if (i_process_valid && (i_process_slot < 3'd7))
                            state <= S_LOAD_A;
                        else
                            state <= S_INVALID;
                    end
                end

                S_LOAD_A: begin
                    // Launch A.  The synchronous RAM output is consumed by
                    // LOAD_B on the following edge.
                    state <= S_LOAD_B;
                end

                S_LOAD_B: begin
                    // Latch A and launch B in the same cycle.
                    row_a_q <= S_tile_read_data_q;
                    state <= S_LOAD_C;
                end

                S_LOAD_C: begin
                    // Latch B and launch C in the same cycle.
                    row_b_q <= S_tile_read_data_q;
                    state <= S_LOAD_WAIT;
                end

                S_LOAD_WAIT: begin
                    // Complete the three-row window before the first pixel.
                    row_c_q <= S_tile_read_data_q;
                    state <= S_PIXEL;
                end

                S_PIXEL: begin
                    cell_sum_q <= cell_sum_q + {6'd0, pixel_q8_calc};
                    // The four pixels in a source row hide the synchronous
                    // RAM latency for the following row window.  Pixel 0
                    // launches A, pixel 1 latches A and launches B, pixel 2
                    // latches B and launches C, and pixel 3 rotates the
                    // completed A/B/C window directly into the pixel loop.
                    if (cell_pixel_count_q[1:0] == 2'd1)
                        S_next_row_a_q <= S_tile_read_data_q;
                    if (cell_pixel_count_q[1:0] == 2'd2)
                        S_next_row_b_q <= S_tile_read_data_q;
                    if (cell_pixel_count_q == 6'd31) begin
                        state <= S_EMIT;
                    end else begin
                        cell_pixel_count_q <= cell_pixel_count_q + 1'b1;
                        if (pixel_x_q == ((cell_col_q * 4) + 3)) begin
                            pixel_x_q <= cell_col_q * 4;
                            pixel_y_q <= pixel_y_q + 1'b1;
                            row_a_q <= S_next_row_a_q;
                            row_b_q <= S_next_row_b_q;
                            row_c_q <= S_tile_read_data_q;
                            state <= S_PIXEL;
                        end else begin
                            pixel_x_q <= pixel_x_q + 1'b1;
                        end
                    end
                end

                S_EMIT: begin
                    if (i_feature_ready) begin
                        cell_sum_q <= 14'd0;
                        cell_pixel_count_q <= 6'd0;
                        if (o_feature_index == 7'd127) begin
                            o_engine_done <= 1'b1;
                            state <= S_IDLE;
                        end else begin
                            o_feature_index <= o_feature_index + 1'b1;
                            if (cell_col_q == 4'd7) begin
                                cell_col_q <= 4'd0;
                                if (cell_row_q == 3'd3) begin
                                    cell_row_q <= 3'd0;
                                    direction_q <= direction_q + 1'b1;
                                end else begin
                                    cell_row_q <= cell_row_q + 1'b1;
                                end
                            end else begin
                                cell_col_q <= cell_col_q + 1'b1;
                            end
                            // The new cell's first pixel starts at its own
                            // row/column.  Multiplication by constants is
                            // synthesized as shifts/adds.
                            if (cell_col_q == 4'd7) begin
                                pixel_x_q <= 6'd0;
                                if (cell_row_q == 3'd3)
                                    pixel_y_q <= 6'd0;
                                else
                                    pixel_y_q <= ({3'd0, cell_row_q} + 6'd1) << 3;
                            end else begin
                                pixel_x_q <= ({2'd0, cell_col_q} + 6'd1) << 2;
                                pixel_y_q <= {3'd0, cell_row_q} << 3;
                            end
                            state <= S_LOAD_A;
                        end
                    end
                end

                S_INVALID: begin
                    o_engine_done <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule
