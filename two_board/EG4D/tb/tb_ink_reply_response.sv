`timescale 1ns/1ps
module tb_ink_reply_response;
    reg clk=0;always #7 clk=~clk;
    reg rst=0,barrier=0,cancel=0,control_valid=0,result_valid=0;
    reg [7:0] control_kind=6;
    reg [31:0] control_epoch=32'h12345678,control_challenge=32'h01020304;
    reg [188:0] result_data;
    wire control_ready,result_ready,we,wb,commit,cb,recover,source_quiet;
    wire [7:0] wa,wd;wire [8:0] length;
    reg nextbank=1,accept=0,packet_valid=0;
    reg [7:0] mem[0:255],rd=0;wire [7:0] ra;wire release_packet;
    reg eof=0;reg [31:0] eof_epoch=32'h12345678,eof_fid=32'h89abcdef;
    wire ack_seen,peer_hello,res_valid,frame_result,rx_quiet;
    wire [31:0] ack_epoch,ack_challenge,rejected;
    wire [55:0] res,rawcode;wire [3:0] res_n;wire [6:0] rawvalid;
    integer frames=0,acks=0,commits=0,reads=0;
    ctl_slave_ink_reply #(.P_COMMIT_TIMEOUT_CYCLES(8)) reply(
        .I_clk_app(clk),.I_rst_app_n(rst),.I_barrier(barrier),.I_cancel(cancel),
        .I_control_valid(control_valid),.O_control_ready(control_ready),
        .I_control_kind(control_kind),.I_control_epoch(control_epoch),.I_control_challenge(control_challenge),
        .I_result_valid(result_valid),.O_result_ready(result_ready),.I_result_data(result_data),
        .I_free_bank_mask(2'b11),.I_next_commit_bank(nextbank),
        .O_write_en(we),.O_write_bank(wb),.O_write_addr(wa),.O_write_data(wd),
        .O_commit(commit),.O_commit_bank(cb),.O_packet_length(length),.I_commit_accept(accept),
        .I_transport_quiet(!packet_valid),.O_session_recover(recover),.O_source_quiet(source_quiet));
    ctl_master_ink_response response(
        .I_clk_app(clk),.I_rst_app_n(rst),.I_barrier(barrier),.I_packet_valid(packet_valid),
        .O_packet_read_addr(ra),.I_packet_read_data(rd),.O_packet_release(release_packet),
        .I_eof_committed(eof),.I_eof_epoch(eof_epoch),.I_eof_frame_index(eof_fid),
        .O_ack_seen(ack_seen),.O_ack_epoch(ack_epoch),.O_ack_challenge(ack_challenge),
        .O_peer_hello(peer_hello),.O_res(res),.O_res_n(res_n),.O_res_valid(res_valid),
        .O_frame_result(frame_result),.O_raw_code(rawcode),.O_raw_valid(rawvalid),
        .O_rejected_result_count(rejected),.O_rx_quiet(rx_quiet));
    always @(posedge clk)begin
        rd<=mem[ra];accept<=0;
        if(we)begin
            if(wb!==nextbank)$fatal(1,"wrong bank write");
            mem[wa]<=wd;
        end
        if(commit)begin
            if(cb!==nextbank)$fatal(1,"wrong bank commit");
            if(packet_valid)$fatal(1,"packet overwrite");
            accept<=1;nextbank<=~nextbank;packet_valid<=1;commits<=commits+1;
            if(length==32)begin
                if({mem[0],mem[1],mem[2],mem[3]}!==32'h52530107)$fatal(1,"wire header");
                if(mem[23]!==8'h87 || mem[24]!==8'h55)$fatal(1,"wire valid/count");
                if({mem[22],mem[21],mem[20],mem[19],mem[18],mem[17],mem[16]}!==56'h47464544434241)$fatal(1,"wire ASCII");
                if({mem[31],mem[30],mem[29],mem[28],mem[27],mem[26],mem[25]}!==56'h17161514131211)$fatal(1,"wire raw code");
            end
        end
        if(release_packet)packet_valid<=0;
        if(frame_result)frames<=frames+1;
        if(ack_seen)acks<=acks+1;
    end
    task send_result(input [31:0] fid);
        begin
            @(negedge clk);result_data={eof_epoch,fid,56'h47464544434241,4'd7,1'b1,1'b1,56'h17161514131211,7'h55};result_valid=1;
            while(!result_ready)@(negedge clk);
            @(negedge clk);result_valid=0;
            wait(commits>reads);reads=commits;
            wait(rx_quiet);repeat(3)@(negedge clk);
        end
    endtask
    initial begin
        repeat(3)@(negedge clk);rst=1;
        @(negedge clk);eof=1;@(negedge clk);eof=0;
        send_result(eof_fid);
        if(frames!=1 || res!==56'h47464544434241 || !res_valid || res_n!=7 || rawvalid!==7'h55)$fatal(1,"accepted tuple");
        send_result(eof_fid);
        if(frames!=1 || rejected!=1)$fatal(1,"duplicate accepted");
        @(negedge clk);eof_fid=eof_fid+1;eof=1;@(negedge clk);eof=0;
        send_result(eof_fid-1);
        if(frames!=1 || rejected!=2)$fatal(1,"stale accepted");
        barrier=1;repeat(2)@(negedge clk);
        if(res_valid || res_n || res || rawvalid)$fatal(1,"barrier did not clear");
        control_valid=1;wait(control_ready);@(negedge clk);control_valid=0;
        wait(rx_quiet);repeat(3)@(negedge clk);
        if(acks!=1 || ack_epoch!==control_epoch || ack_challenge!==control_challenge)$fatal(1,"barrier ACK decode");
        barrier=0;control_valid=1;repeat(4)@(negedge clk);cancel=1;
        @(negedge clk);control_challenge=32'h11223344;cancel=0;
        wait(control_ready);@(negedge clk);control_valid=0;
        wait(rx_quiet);repeat(3)@(negedge clk);
        if(acks!=2 || ack_challenge!==32'h11223344)$fatal(1,"uncommitted ACK cancellation");
        $display("TB_RESULT: PASS reply response golden/stale/barrier/cancel/parity");$finish;
    end
    initial begin #200000;$fatal(1,"timeout");end
endmodule
