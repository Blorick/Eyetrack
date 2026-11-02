`timescale 1ns/1ps
// ============================================================================
// tb_ctl_char_shape_feature_engine_miter -- C13 synchronous tile-read miter
// -----------------------------------------------------------------------------
// 所属域   : ctl
// 时钟域   : I_clk_25m
// 复位     : I_rst_n
// 验证目标 : legacy C13 与同步读 C13 的 128-feature 序列等价
// -----------------------------------------------------------------------------
// The legacy copy is intentionally an isolated golden model.  Writes are
// completed before each process request, matching ctl_eg4d_compute_adapter's
// RX_TILE_WRITE -> SESSION_CHECK -> GLYPH_START ordering.  Bank 0/1, boundary
// slots, deterministic random glyphs, ready backpressure, and mid-request
// reset are covered without changing any production top-level interface.
// ============================================================================

module tb_ctl_char_shape_feature_engine_miter;

    reg W_clk = 1'b0;
    reg W_rst_n = 1'b0;

    reg        W_tile_write_en = 1'b0;
    reg [8:0]  W_tile_write_addr = 9'd0;
    reg [31:0] W_tile_write_data = 32'd0;
    reg        W_process_start = 1'b0;
    reg [2:0]  W_process_slot = 3'd0;
    reg        W_process_bank = 1'b0;
    reg        W_process_valid = 1'b0;
    reg        W_feature_ready = 1'b1;

    wire       W_new_feature_valid;
    wire [7:0] W_new_feature_data;
    wire       W_new_engine_done;
    wire       W_new_busy;
    wire [6:0] W_new_feature_index;
    wire       W_gold_feature_valid;
    wire [7:0] W_gold_feature_data;
    wire       W_gold_engine_done;
    wire       W_gold_busy;
    wire [6:0] W_gold_feature_index;

    integer S_cycle;
    integer S_ready_mode;
    integer S_run_active;
    integer S_gold_count;
    integer S_new_count;
    integer S_gold_done_seen;
    integer S_new_done_seen;
    integer S_gold_done_cycle;
    integer S_new_done_cycle;
    integer S_errors;
    integer S_checks;
    integer S_timeout;
    integer S_index;
    reg [7:0] S_gold_data [0:127];
    reg [7:0] S_new_data [0:127];

    always #20 W_clk = ~W_clk;

    char_shape_feature_engine u_new (
        .clk              (W_clk),
        .rst_n            (W_rst_n),
        .i_tile_write_en  (W_tile_write_en),
        .i_tile_write_addr(W_tile_write_addr),
        .i_tile_write_data(W_tile_write_data),
        .i_process_start  (W_process_start),
        .i_process_slot   (W_process_slot),
        .i_process_bank   (W_process_bank),
        .i_process_valid  (W_process_valid),
        .o_feature_valid  (W_new_feature_valid),
        .o_feature_data   (W_new_feature_data),
        .i_feature_ready  (W_feature_ready),
        .o_engine_done    (W_new_engine_done),
        .o_busy           (W_new_busy),
        .o_feature_index  (W_new_feature_index)
    );

    ctl_char_shape_feature_engine_legacy u_gold (
        .clk              (W_clk),
        .rst_n            (W_rst_n),
        .i_tile_write_en  (W_tile_write_en),
        .i_tile_write_addr(W_tile_write_addr),
        .i_tile_write_data(W_tile_write_data),
        .i_process_start  (W_process_start),
        .i_process_slot   (W_process_slot),
        .i_process_bank   (W_process_bank),
        .i_process_valid  (W_process_valid),
        .o_feature_valid  (W_gold_feature_valid),
        .o_feature_data   (W_gold_feature_data),
        .i_feature_ready  (W_feature_ready),
        .o_engine_done    (W_gold_engine_done),
        .o_busy           (W_gold_busy),
        .o_feature_index  (W_gold_feature_index)
    );

    // Keep the ready pattern stable before each active clock edge.  Mode 1
    // deliberately holds the feature sink for several cycles at a time.
    always @(negedge W_clk) begin
        S_cycle = S_cycle + 1;
        case (S_ready_mode)
            1: W_feature_ready = ((S_cycle % 9) < 3);
            default: W_feature_ready = 1'b1;
        endcase
    end

    // Handshake samples are stored independently because the new pipeline is
    // one cycle longer per cell-row window and therefore cannot be compared
    // by same-cycle valid/data matching.
    always @(posedge W_clk) begin
        if (W_rst_n && S_run_active) begin
            if (W_gold_feature_valid && W_feature_ready) begin
                if (S_gold_count >= 128) begin
                    S_errors = S_errors + 1;
                    $display("GOLD_EXTRA_FEATURE cycle=%0d", S_cycle);
                end else begin
                    if (W_gold_feature_index !== S_gold_count[6:0]) begin
                        S_errors = S_errors + 1;
                        $display("GOLD_INDEX_MISMATCH index=%0d expected=%0d",
                                 W_gold_feature_index, S_gold_count);
                    end
                    S_gold_data[S_gold_count] = W_gold_feature_data;
                    S_gold_count = S_gold_count + 1;
                end
            end
            if (W_new_feature_valid && W_feature_ready) begin
                if (S_new_count >= 128) begin
                    S_errors = S_errors + 1;
                    $display("NEW_EXTRA_FEATURE cycle=%0d", S_cycle);
                end else begin
                    if (W_new_feature_index !== S_new_count[6:0]) begin
                        S_errors = S_errors + 1;
                        $display("NEW_INDEX_MISMATCH index=%0d expected=%0d",
                                 W_new_feature_index, S_new_count);
                    end
                    S_new_data[S_new_count] = W_new_feature_data;
                    S_new_count = S_new_count + 1;
                end
            end
            if (W_gold_engine_done) begin
                S_gold_done_seen = 1;
                S_gold_done_cycle = S_cycle;
            end
            if (W_new_engine_done) begin
                S_new_done_seen = 1;
                S_new_done_cycle = S_cycle;
            end
        end
    end

    function [31:0] glyph_word;
        input integer V_bank;
        input integer V_slot;
        input integer V_row;
        input integer V_pattern;
        integer W_bit;
        integer W_seed;
        reg [31:0] W_value;
        begin
            W_value = 32'd0;
            case (V_pattern)
                0: W_value = 32'h0000_0000;
                1: W_value = 32'hffff_ffff;
                2: begin
                    if ((V_row % 2) == 0)
                        W_value = 32'haaaa_aaaa;
                    else
                        W_value = 32'h5555_5555;
                end
                3: begin
                    for (W_bit = 0; W_bit < 32; W_bit = W_bit + 1)
                        W_value[W_bit] = ((W_bit == 0) || (W_bit == 31) ||
                                          (W_bit == (V_row % 32)) ||
                                          (W_bit == (31 - (V_row % 32))));
                end
                default: begin
                    W_seed = (V_bank * 32'h1357) ^
                             (V_slot * 32'h2468) ^
                             (V_row * 32'h31f1) ^ 32'h5a3c;
                    for (W_bit = 0; W_bit < 32; W_bit = W_bit + 1)
                        W_value[W_bit] = ((W_seed + W_bit * 17 +
                                           V_row * 3 + V_slot) ^
                                          (W_bit * 13 + V_bank * 7)) & 1;
                end
            endcase
            glyph_word = W_value;
        end
    endfunction

    task automatic write_glyph;
        input integer V_bank;
        input integer V_slot;
        input integer V_pattern;
        integer W_row;
        begin
            for (W_row = 0; W_row < 32; W_row = W_row + 1) begin
                @(negedge W_clk);
                W_tile_write_en = 1'b1;
                W_tile_write_addr = V_bank * 224 + V_slot * 32 + W_row;
                W_tile_write_data = glyph_word(V_bank, V_slot, W_row,
                                               V_pattern);
            end
            @(negedge W_clk);
            W_tile_write_en = 1'b0;
            W_tile_write_addr = 9'd0;
            W_tile_write_data = 32'd0;
        end
    endtask

    task automatic compare_process;
        input integer V_bank;
        input integer V_slot;
        input integer V_ready_mode;
        input integer V_expected_delta;
        integer W_wait;
        begin
            S_ready_mode = V_ready_mode;
            S_gold_count = 0;
            S_new_count = 0;
            S_gold_done_seen = 0;
            S_new_done_seen = 0;
            S_gold_done_cycle = -1;
            S_new_done_cycle = -1;
            @(negedge W_clk);
            S_run_active = 1;
            W_process_bank = V_bank;
            W_process_slot = V_slot;
            W_process_valid = 1'b1;
            W_process_start = 1'b1;
            @(negedge W_clk);
            W_process_start = 1'b0;

            W_wait = 0;
            while (!(S_gold_done_seen && S_new_done_seen) &&
                   (W_wait < 200000)) begin
                @(negedge W_clk);
                W_wait = W_wait + 1;
            end
            @(negedge W_clk);
            S_run_active = 0;

            if (!(S_gold_done_seen && S_new_done_seen)) begin
                S_errors = S_errors + 1;
                $display("C13_TIMEOUT bank=%0d slot=%0d mode=%0d wait=%0d",
                         V_bank, V_slot, V_ready_mode, W_wait);
            end
            if (S_gold_count != 128) begin
                S_errors = S_errors + 1;
                $display("GOLD_COUNT_FAIL bank=%0d slot=%0d count=%0d",
                         V_bank, V_slot, S_gold_count);
            end
            if (S_new_count != 128) begin
                S_errors = S_errors + 1;
                $display("NEW_COUNT_FAIL bank=%0d slot=%0d count=%0d",
                         V_bank, V_slot, S_new_count);
            end
            for (S_index = 0; S_index < 128; S_index = S_index + 1) begin
                S_checks = S_checks + 1;
                if (S_gold_data[S_index] !== S_new_data[S_index]) begin
                    S_errors = S_errors + 1;
                    if (S_errors < 12)
                        $display("FEATURE_MISMATCH bank=%0d slot=%0d index=%0d gold=%02h new=%02h",
                                 V_bank, V_slot, S_index,
                                 S_gold_data[S_index], S_new_data[S_index]);
                end
            end
            if (V_expected_delta >= 0) begin
                if ((S_new_done_cycle - S_gold_done_cycle) !=
                    V_expected_delta) begin
                    S_errors = S_errors + 1;
                    $display("LATENCY_DELTA_FAIL bank=%0d slot=%0d gold=%0d new=%0d expected=%0d",
                             V_bank, V_slot, S_gold_done_cycle,
                             S_new_done_cycle, V_expected_delta);
                end
            end
            $display("C13_MITER_RUN bank=%0d slot=%0d ready_mode=%0d features=%0d delta=%0d",
                     V_bank, V_slot, V_ready_mode, S_new_count,
                     S_new_done_cycle - S_gold_done_cycle);
        end
    endtask

    task automatic reset_mid_process;
        input integer V_bank;
        input integer V_slot;
        begin
            S_ready_mode = 0;
            S_gold_count = 0;
            S_new_count = 0;
            S_gold_done_seen = 0;
            S_new_done_seen = 0;
            @(negedge W_clk);
            S_run_active = 1;
            W_process_bank = V_bank;
            W_process_slot = V_slot;
            W_process_valid = 1'b1;
            W_process_start = 1'b1;
            @(negedge W_clk);
            W_process_start = 1'b0;
            repeat (90) @(negedge W_clk);
            S_run_active = 0;
            W_rst_n = 1'b0;
            repeat (3) @(negedge W_clk);
            W_rst_n = 1'b1;
            repeat (2) @(negedge W_clk);
            W_process_valid = 1'b0;
            $display("C13_RESET_ABORT bank=%0d slot=%0d", V_bank, V_slot);
        end
    endtask

    initial begin
        S_cycle = 0;
        S_ready_mode = 0;
        S_run_active = 0;
        S_gold_count = 0;
        S_new_count = 0;
        S_gold_done_seen = 0;
        S_new_done_seen = 0;
        S_errors = 0;
        S_checks = 0;
        W_feature_ready = 1'b1;
        W_process_valid = 1'b0;
        repeat (4) @(negedge W_clk);
        W_rst_n = 1'b1;
        repeat (2) @(negedge W_clk);

        // Boundary slots and both bank strides exercise address 0, 223, 224,
        // and 447 while alternating writes and complete feature requests.
        write_glyph(0, 0, 0);
        // The legacy implementation reloads all three rows for every one of
        // the eight source rows in a cell (57 clocks/feature).  The new
        // single-port pipeline hides seven of those reloads under the four
        // pixel cycles, so it is 20 clocks/feature faster while preserving
        // the exact feature stream: 128 * -20 = -2560 clocks per glyph.
        compare_process(0, 0, 0, -2560);
        write_glyph(0, 6, 1);
        compare_process(0, 6, 0, -2560);
        write_glyph(1, 0, 2);
        compare_process(1, 0, 0, -2560);
        write_glyph(1, 6, 3);
        compare_process(1, 6, 1, -1);

        // Abort a live request with reset, rewrite the same glyph, and prove
        // the next request produces a complete clean sequence.
        write_glyph(0, 3, 4);
        reset_mid_process(0, 3);
        write_glyph(0, 3, 4);
        compare_process(0, 3, 0, -2560);

        if (S_errors != 0)
            $fatal(1, "TB_RESULT: FAIL tb_ctl_char_shape_feature_engine_miter checks=%0d errors=%0d",
                   S_checks, S_errors);
        $display("TB_RESULT: PASS tb_ctl_char_shape_feature_engine_miter checks=%0d errors=0",
                 S_checks);
        $finish;
    end

endmodule
