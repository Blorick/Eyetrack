`timescale 1ns/1ps
module tb_ink_capture_recovery;
    reg clk=0;always #7 clk=~clk;
    reg rst=0,control=0,nextbank=1,txquiet=1,accept=0,reject=0;
    reg [1:0] freebanks=3;
    wire ready,we,wb,commit,cb,eof,recover,quiet;
    wire [7:0] addr,data;wire [8:0] length;
    wire [31:0] epoch,fid,drops;
    integer committed=0,writes=0;
    ctl_master_ink_capture #(.P_COMMIT_TIMEOUT_CYCLES(8)) dut(
        .I_clk_app(clk),.I_rst_app_n(rst),.I_ink(1'b0),.I_ink_de(1'b0),.I_vsync(1'b1),
        .I_detected(1'b0),.I_char_up(11'd0),.I_char_down(11'd0),.I_char_valid(7'd0),
        .I_char_left(77'd0),.I_char_right(77'd0),.I_session_ready(1'b0),.I_epoch(32'd0),
        .I_control_valid(control),.I_control_kind(8'd5),.I_control_epoch(32'habcdef12),
        .I_control_challenge(32'h12345678),.O_control_ready(ready),.I_free_bank_mask(freebanks),
        .I_next_commit_bank(nextbank),.I_transport_quiet(txquiet),.O_write_en(we),
        .O_write_bank(wb),.O_write_addr(addr),.O_write_data(data),.O_commit(commit),
        .O_commit_bank(cb),.O_packet_length(length),.I_commit_accept(accept),
        .O_session_recover(recover),.O_eof_committed(eof),.O_eof_epoch(epoch),
        .O_eof_frame_index(fid),.O_frame_drop_count(drops),.O_source_quiet(quiet));
    always @(posedge clk)begin
        accept<=0;
        if(we)begin
            if(wb!==nextbank || !freebanks[wb])$fatal(1,"owned/wrong parity write");
            writes<=writes+1;
        end
        if(commit)begin
            if(cb!==nextbank || length!=16)$fatal(1,"commit parity/length");
            if(!reject)begin accept<=1;nextbank<=~nextbank;committed<=committed+1;end
        end
    end
    initial begin
        repeat(3)@(negedge clk);rst=1;control=1;
        wait(ready);@(negedge clk);control=0;
        repeat(4)@(negedge clk);
        if(committed!=1 || nextbank!==0)$fatal(1,"startup parity");
        // Warm reset local source while an accepted transport bank is owned.
        rst=0;freebanks=0;txquiet=0;repeat(3)@(negedge clk);rst=1;control=1;
        writes=0;repeat(20)@(negedge clk);
        if(we || commit || writes)$fatal(1,"warm reset overwrote owned bank");
        freebanks=3;txquiet=1;
        wait(ready);@(negedge clk);control=0;
        repeat(4)@(negedge clk);
        if(committed!=2 || nextbank!==1)$fatal(1,"warm parity resync");
        reject=1;txquiet=0;control=1;
        wait(recover);@(negedge clk);writes=0;
        repeat(20)@(negedge clk);
        if(!recover || we || commit || writes)$fatal(1,"timeout recovery did not retain banks");
        control=0;txquiet=1;reject=0;
        wait(!recover);repeat(4)@(negedge clk);control=1;
        wait(ready);@(negedge clk);control=0;
        repeat(4)@(negedge clk);
        if(committed!=3 || drops!=1 || nextbank!==0)$fatal(1,"recovery failed");
        $display("TB_RESULT: PASS capture startup/warm parity owned bank/commit timeout recovery");$finish;
    end
    initial begin #200000;$fatal(1,"timeout");end
endmodule
