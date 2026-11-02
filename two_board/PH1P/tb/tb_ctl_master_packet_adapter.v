//=============================================================================
//  tb_ctl_master_packet_adapter  --  directed app/packet adapter contract TB
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : clk (synchronous app model)
//  复位     : rst_n
//  参数     : none
//  验证状态 : Root Questa fixture pending after reset/read-pipeline update
//=============================================================================
`timescale 1ns / 1ps

// Directed application-boundary test.  The Ethernet common block is replaced
// by a synchronous byte-memory model at the fixed app API; no hierarchy force
// or internal-array deposit is used.
module tb_ctl_master_packet_adapter;
    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg rst_n;
    reg i_capture_write_en;
    reg [8:0] i_capture_write_addr;
    reg [31:0] i_capture_write_data;
    reg i_snapshot_valid;
    reg i_snapshot_drop;
    reg i_snapshot_bank;
    reg [6:0] i_snapshot_mask;
    reg [31:0] i_snapshot_frame_index;
    reg i_plate_valid;
    reg i_link_ready;

    wire o_capture_export_busy;
    wire o_tx_write_en;
    wire [9:0] o_tx_write_addr;
    wire [7:0] o_tx_write_data;
    wire o_tx_commit;
    wire [10:0] o_tx_length;
    reg i_tx_busy;
    reg i_tx_done;
    reg i_rx_valid;
    reg [10:0] i_rx_length;
    wire [9:0] o_rx_read_addr;
    reg [7:0] i_rx_read_data;
    wire o_rx_release;
    wire [55:0] o_res;
    wire [3:0] o_res_n;
    wire o_res_valid;
    wire [6:0] o_raw_valid;
    wire [55:0] o_raw_candidate_code;
    wire [6:0] o_raw_candidate_mask;
    wire o_frame_result;
    wire [31:0] o_snapshot_drop_count;

    reg [7:0] tx_mem [0:911];
    reg [7:0] rx_mem [0:31];
    reg response_armed;
    integer tx_write_count;
    integer commit_count;
    integer frame_result_count;
    integer checks;
    integer errors;
    integer n;

    ctl_master_packet_adapter #(
        .P_SESSION_ID    (32'h2026_1011),
        // Allow the fixed 912-byte TX stream to complete, then exercise the
        // bounded wait timeout without making the directed run excessive.
        .P_TIMEOUT_CYCLES(24'd1200)
    ) dut (
        .I_clk_pix                   (clk),
        .I_rst_pix_n             (rst_n),
        .I_capture_write_en     (i_capture_write_en),
        .I_capture_write_addr   (i_capture_write_addr),
        .I_capture_write_data   (i_capture_write_data),
        .I_snapshot_valid       (i_snapshot_valid),
        .I_snapshot_drop        (i_snapshot_drop),
        .I_snapshot_bank        (i_snapshot_bank),
        .I_snapshot_mask        (i_snapshot_mask),
        .I_snapshot_frame_index (i_snapshot_frame_index),
        .I_plate_valid          (i_plate_valid),
        .I_link_ready           (i_link_ready),
        .O_capture_export_busy  (o_capture_export_busy),
        .O_tx_write_en          (o_tx_write_en),
        .O_tx_write_addr        (o_tx_write_addr),
        .O_tx_write_data        (o_tx_write_data),
        .O_tx_commit            (o_tx_commit),
        .O_tx_length            (o_tx_length),
        .I_tx_busy              (i_tx_busy),
        .I_tx_done              (i_tx_done),
        .I_rx_valid             (i_rx_valid),
        .I_rx_length            (i_rx_length),
        .O_rx_read_addr         (o_rx_read_addr),
        .I_rx_read_data         (i_rx_read_data),
        .O_rx_release           (o_rx_release),
        .O_res                  (o_res),
        .O_res_n                (o_res_n),
        .O_res_valid            (o_res_valid),
        .O_raw_valid            (o_raw_valid),
        .O_raw_candidate_code   (o_raw_candidate_code),
        .O_raw_candidate_mask   (o_raw_candidate_mask),
        .O_frame_result         (o_frame_result),
        .O_snapshot_drop_count  (o_snapshot_drop_count)
    );

    // The link's one-cycle synchronous read response and one-cycle RX-valid
    // indication are modeled at the application boundary.
    always @(posedge clk) begin
        i_rx_read_data <= rx_mem[o_rx_read_addr];
        if (response_armed) begin
            i_rx_valid <= 1'b1;
            i_rx_length <= 11'd46;
            response_armed <= 1'b0;
        end else begin
            i_rx_valid <= 1'b0;
            i_rx_length <= 11'd0;
        end
        if (o_tx_write_en) begin
            tx_mem[o_tx_write_addr] <= o_tx_write_data;
            tx_write_count = tx_write_count + 1;
        end
        if (o_tx_commit)
            commit_count = commit_count + 1;
        if (o_frame_result)
            frame_result_count = frame_result_count + 1;
    end

    task automatic pulse_snapshot;
        begin
            i_snapshot_valid = 1'b1;
            @(posedge clk);
            i_snapshot_valid = 1'b0;
        end
    endtask

    task automatic fill_bank;
        integer j;
        begin
            for (j = 0; j < 224; j = j + 1) begin
                @(posedge clk);
                i_capture_write_en = 1'b1;
                i_capture_write_addr = j;
                i_capture_write_data = 32'hA0000000 + j;
            end
            @(posedge clk);
            i_capture_write_en = 1'b0;
            i_capture_write_addr = 9'd0;
            i_capture_write_data = 32'd0;
        end
    endtask

    task automatic wait_commit;
        integer j;
        begin
            j = 0;
            while (!o_tx_commit && (j < 1200)) begin
                @(posedge clk);
                j = j + 1;
            end
            if (!o_tx_commit) begin
                $display("FAIL wait_commit state=%0d busy=%b tx_count=%0d commits=%0d timeout=%0d", dut.ST_state_q, o_capture_export_busy, tx_write_count, commit_count, dut.S_timeout_q);
                errors = errors + 1;
            end
            // The monitor and DUT update their one-cycle outputs with NBAs;
            // give the monitor one settled edge before checking the final
            // write and before arming the synchronous RX model.
            @(posedge clk);
        end
    endtask

    task automatic wait_result;
        input integer target_count;
        integer j;
        begin
            j = 0;
            while ((frame_result_count < target_count) && (j < 200)) begin
                @(posedge clk);
                j = j + 1;
            end
            if (frame_result_count < target_count) begin
                $display("FAIL wait_result state=%0d res_valid=%b res=%h res_n=%h rawv=%h timeout=%0d", dut.ST_state_q, o_res_valid, o_res, o_res_n, o_raw_valid, dut.S_timeout_q);
                errors = errors + 1;
            end
        end
    endtask

    task automatic make_response;
        input [31:0] response_frame;
        input [31:0] response_session;
        begin
            rx_mem[0]  = 8'h4c; rx_mem[1]  = 8'h50;
            rx_mem[2]  = 8'h01; rx_mem[3]  = 8'h02;
            rx_mem[4]  = response_session[31:24]; rx_mem[5]  = response_session[23:16];
            rx_mem[6]  = response_session[15:8];  rx_mem[7]  = response_session[7:0];
            rx_mem[8]  = response_frame[31:24];
            rx_mem[9]  = response_frame[23:16];
            rx_mem[10] = response_frame[15:8];
            rx_mem[11] = response_frame[7:0];
            rx_mem[12] = 8'h00; rx_mem[13] = 8'h10;
            rx_mem[14] = 8'h7f; rx_mem[15] = 8'h01;
            rx_mem[16] = 8'h01; rx_mem[17] = 8'h02;
            rx_mem[18] = 8'h03; rx_mem[19] = 8'h04;
            rx_mem[20] = 8'h05; rx_mem[21] = 8'h06;
            rx_mem[22] = 8'h07; rx_mem[23] = 8'h03;
            rx_mem[24] = 8'h55;
            rx_mem[25] = 8'h10; rx_mem[26] = 8'h20;
            rx_mem[27] = 8'h30; rx_mem[28] = 8'h40;
            rx_mem[29] = 8'h50; rx_mem[30] = 8'h60;
            rx_mem[31] = 8'h70;
            response_armed = 1'b1;
        end
    endtask

    initial begin
        rst_n = 1'b0;
        i_capture_write_en = 1'b0;
        i_capture_write_addr = 9'd0;
        i_capture_write_data = 32'd0;
        i_snapshot_valid = 1'b0;
        i_snapshot_drop = 1'b0;
        i_snapshot_bank = 1'b0;
        i_snapshot_mask = 7'h00;
        i_snapshot_frame_index = 32'd0;
        i_plate_valid = 1'b0;
        i_link_ready = 1'b0;
        i_tx_busy = 1'b0;
        i_tx_done = 1'b0;
        i_rx_valid = 1'b0;
        i_rx_length = 11'd0;
        i_rx_read_data = 8'd0;
        response_armed = 1'b0;
        tx_write_count = 0;
        commit_count = 0;
        frame_result_count = 0;
        checks = 0;
        errors = 0;
        for (n = 0; n < 912; n = n + 1)
            tx_mem[n] = 8'd0;
        for (n = 0; n < 32; n = n + 1)
            rx_mem[n] = 8'd0;

        repeat (4) @(posedge clk);
        rst_n = 1'b1;
        repeat (2) @(posedge clk);

        // Link down: a snapshot cannot start a request and any old result is
        // invalidated.
        i_snapshot_mask = 7'h7f;
        i_snapshot_frame_index = 32'h0000_1234;
        i_plate_valid = 1'b1;
        pulse_snapshot();
        repeat (4) @(posedge clk);
        checks = checks + 1;
        if (commit_count != 0 || o_res_valid) begin
            $display("FAIL link_down commit=%0d res_valid=%b state=%0d", commit_count, o_res_valid, dut.ST_state_q);
            errors = errors + 1;
        end

        // One normal request.  Mutate the source metadata immediately after
        // the pulse; the TX header must use the adapter's latched snapshot.
        i_link_ready = 1'b1;
        fill_bank();
        i_snapshot_mask = 7'h55;
        i_snapshot_frame_index = 32'h0000_1234;
        i_plate_valid = 1'b1;
        pulse_snapshot();
        i_snapshot_mask = 7'h12;
        i_snapshot_frame_index = 32'h0000_beef;
        pulse_snapshot(); // single-pending: this one must be ignored
        // A snapshot can be dropped while the fixed-size export is in flight;
        // retain the event as telemetry without disturbing the transaction.
        i_snapshot_drop = 1'b1;
        @(posedge clk);
        i_snapshot_drop = 1'b0;
        wait_commit();
        checks = checks + 1;
        if (tx_write_count != 912 || o_tx_length != 11'd912) begin
            $display("FAIL tx_count count=%0d length=%0d commit=%0d state=%0d", tx_write_count, o_tx_length, commit_count, dut.ST_state_q);
            errors = errors + 1;
        end
        checks = checks + 1;
        if (tx_mem[0] != 8'h4c || tx_mem[1] != 8'h50 ||
            tx_mem[14] != 8'h55 || tx_mem[15] != 8'h01 ||
            tx_mem[8] != 8'h00 || tx_mem[9] != 8'h00 ||
            tx_mem[10] != 8'h12 || tx_mem[11] != 8'h34)
            begin
                $display("FAIL header b0=%02h b1=%02h b8..11=%02h%02h%02h%02h b14=%02h b15=%02h maskq=%02h frameq=%08h", tx_mem[0], tx_mem[1], tx_mem[8], tx_mem[9], tx_mem[10], tx_mem[11], tx_mem[14], tx_mem[15], dut.S_snapshot_mask_q, dut.S_frame_id_q);
                errors = errors + 1;
            end
        checks = checks + 1;
        if (tx_mem[16] != 8'h00 || tx_mem[17] != 8'h00 ||
            tx_mem[18] != 8'h00 || tx_mem[19] != 8'ha0 ||
            tx_mem[16 + 4*223] != 8'hdf)
            begin
                $display("FAIL payload p16..19=%02h%02h%02h%02h p908=%02h row0=%08h row223=%08h", tx_mem[16], tx_mem[17], tx_mem[18], tx_mem[19], tx_mem[16+4*223], dut.S_bank_ram[0], dut.S_bank_ram[223]);
                errors = errors + 1;
            end
        checks = checks + 1;
        if (o_snapshot_drop_count != 32'd1) begin
            $display("FAIL snapshot_drop_count=%0d expected=1", o_snapshot_drop_count);
            errors = errors + 1;
        end

        // A complete response is accepted only after all 32 synchronous
        // bytes have been read, then published atomically.
        make_response(32'h0000_1234, dut.S_session_id_q);
        wait_result(1);
        checks = checks + 1;
        if (!o_res_valid || o_res != 56'h07060504030201 ||
            o_res_n != 4'h3 || o_raw_valid != 7'h55 ||
            o_raw_candidate_mask != 7'h55 ||
            o_raw_candidate_code != 56'h70605040302010)
            begin
                $display("FAIL response state=%0d valid=%b result_count=%0d res=%h resn=%h rawv=%h rawmask=%h rawcode=%h rxlen=%0d rx0=%02h rx8..11=%02h%02h%02h%02h", dut.ST_state_q, o_res_valid, frame_result_count, o_res, o_res_n, o_raw_valid, o_raw_candidate_mask, o_raw_candidate_code, dut.S_rx_length_q, dut.S_rx_buf[0], dut.S_rx_buf[8], dut.S_rx_buf[9], dut.S_rx_buf[10], dut.S_rx_buf[11]);
                errors = errors + 1;
            end

        // Exercise the 16-bit historical boundary.  Both sides must carry
        // the full 32-bit frame id so 0x0000ffff and 0x00010000 remain
        // distinct transactions and responses.
        i_snapshot_mask = 7'h01;
        i_snapshot_frame_index = 32'h0000_ffff;
        pulse_snapshot();
        wait_commit();
        checks = checks + 1;
        if ({tx_mem[8], tx_mem[9], tx_mem[10], tx_mem[11]} != 32'h0000_ffff) begin
            $display("FAIL frame_wrap_low tx_frame=%08h expected=0000ffff", {tx_mem[8], tx_mem[9], tx_mem[10], tx_mem[11]});
            errors = errors + 1;
        end
        make_response(32'h0000_ffff, dut.S_session_id_q);
        wait_result(2);
        checks = checks + 1;
        if (!o_res_valid) begin
            $display("FAIL frame_wrap_low_response frame=%08h result_count=%0d", {tx_mem[8], tx_mem[9], tx_mem[10], tx_mem[11]}, frame_result_count);
            errors = errors + 1;
        end

        i_snapshot_frame_index = 32'h0001_0000;
        pulse_snapshot();
        wait_commit();
        checks = checks + 1;
        if ({tx_mem[8], tx_mem[9], tx_mem[10], tx_mem[11]} != 32'h0001_0000) begin
            $display("FAIL frame_wrap_high tx_frame=%08h expected=00010000", {tx_mem[8], tx_mem[9], tx_mem[10], tx_mem[11]});
            errors = errors + 1;
        end
        make_response(32'h0001_0000, dut.S_session_id_q);
        wait_result(3);
        checks = checks + 1;
        if (!o_res_valid) begin
            $display("FAIL frame_wrap_high_response frame=%08h result_count=%0d", {tx_mem[8], tx_mem[9], tx_mem[10], tx_mem[11]}, frame_result_count);
            errors = errors + 1;
        end

        // No response: bounded timeout returns to idle and clears OSD-valid.
        tx_write_count = 0;
        i_snapshot_frame_index = 32'h0000_0002;
        pulse_snapshot();
        wait_commit();
        repeat (1300) @(posedge clk);
        checks = checks + 1;
        if (o_res_valid || frame_result_count < 4 || commit_count != 4) begin
            $display("FAIL timeout res_valid=%b result_count=%0d state=%0d timeout=%0d commits=%0d tx_count=%0d", o_res_valid, frame_result_count, dut.ST_state_q, dut.S_timeout_q, commit_count, tx_write_count);
            errors = errors + 1;
        end

        // A response from the timed-out transaction arrives late while a new
        // transaction is waiting.  Its frame identity must be rejected once,
        // RX_VALID must be drained, and the next matching response must still
        // complete the following transaction.
        i_snapshot_frame_index = 32'h0000_0003;
        pulse_snapshot();
        wait_commit();
        make_response(32'h0000_1234, 32'h2026_1011);
        wait_result(5);
        repeat (2) @(posedge clk);
        checks = checks + 1;
        if (o_res_valid || dut.ST_state_q != dut.ST_IDLE) begin
            $display("FAIL late_response accepted=%b state=%0d result_count=%0d", o_res_valid, dut.ST_state_q, frame_result_count);
            errors = errors + 1;
        end
        // The invalid response completes and drains transaction 3.  Admit a
        // fresh transaction only after that drain, then accept its identity.
        i_snapshot_frame_index = 32'h0000_0004;
        pulse_snapshot();
        wait_commit();
        make_response(32'h0000_0004, dut.S_session_id_q);
        wait_result(6);
        checks = checks + 1;
        if (!o_res_valid) begin
            $display("FAIL recovery_after_late_response valid=%b state=%0d result_count=%0d", o_res_valid, dut.ST_state_q, frame_result_count);
            errors = errors + 1;
        end

        // A link drop also clears the visible result and blocks a new
        // snapshot.  After link recovery a fresh frame_id can complete.
        i_link_ready = 1'b0;
        repeat (2) @(posedge clk);
        checks = checks + 1;
        if (o_res_valid || commit_count != 6) begin
            $display("FAIL link_drop res_valid=%b commit=%0d state=%0d timeout=%0d", o_res_valid, commit_count, dut.ST_state_q, dut.S_timeout_q);
            errors = errors + 1;
        end
        i_link_ready = 1'b1;
        // One clock lets the bounded RX drain return to ST_IDLE after the
        // link's in-band ready status rises.
        @(posedge clk);
        i_snapshot_frame_index = 32'h0000_0005;
        pulse_snapshot();
        wait_commit();
        make_response(32'h0000_0005, dut.S_session_id_q);
        wait_result(7);
        checks = checks + 1;
        if (!o_res_valid) begin
            $display("FAIL link_recovery_response valid=%b state=%0d result_count=%0d", o_res_valid, dut.ST_state_q, frame_result_count);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PH1P_MASTER_PACKET_ADAPTER_TB_PASS checks=%0d errors=0", checks);
        else
            $display("PH1P_MASTER_PACKET_ADAPTER_TB_FAIL checks=%0d errors=%0d", checks, errors);
        $finish;
    end
endmodule
