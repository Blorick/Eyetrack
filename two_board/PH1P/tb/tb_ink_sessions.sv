`timescale 1ns/1ps
module tb_ink_sessions;
    reg clk=0;always #7 clk=~clk;
    reg mrst=0,erst=0,restart=0,peerhello=0,ackseen=0;
    reg [31:0] ackepoch=0,ackchallenge=1;
    reg rxquiet=1,txquiet=1;
    wire mvalid,mready,mbarrier,mclear;wire [7:0] mkind;wire [31:0] mepoch,mchallenge,mactiveepoch;
    reg helloseen=0;reg [31:0] helloepoch=32'h55,hellochallenge=32'h11;
    reg rawquiet=1,tilequiet=1,snapshotquiet=1,ownerinvalid=1,replyquiet=1,eg_rxquiet=1;
    wire evalid,eready,ebarrier,eabort;wire [7:0] ekind;wire [31:0] eepoch,echallenge,eactiveepoch;
    integer round=0;
    ctl_master_ink_session #(.P_DRAIN_GUARD_CYCLES(12),.P_RX_QUIET_CYCLES(4),.P_ACK_TIMEOUT_CYCLES(100)) master(
        .I_clk_app(clk),.I_rst_app_n(mrst),.I_restart(restart),.I_peer_hello(peerhello),
        .I_ack_seen(ackseen),.I_ack_epoch(ackepoch),.I_ack_challenge(ackchallenge),
        .I_transport_quiet(txquiet),.I_rx_quiet(rxquiet),.I_control_ready(1'b1),
        .O_control_valid(mvalid),.O_control_kind(mkind),.O_control_epoch(mepoch),
        .O_control_challenge(mchallenge),.O_session_ready(mready),.O_barrier(mbarrier),
        .O_result_clear(mclear),.O_epoch(mactiveepoch));
    ctl_slave_ink_session slave(.I_clk_app(clk),.I_rst_app_n(erst),
        .I_hello_seen(helloseen),.I_hello_epoch(helloepoch),.I_hello_challenge(hellochallenge),
        .I_raw_quiet(rawquiet),.I_tile_quiet(tilequiet),.I_snapshot_quiet(snapshotquiet),
        .I_old_owner_invalid(ownerinvalid),.I_legacy_tx_quiet(1'b1),.I_reply_tx_quiet(replyquiet),
        .I_rx_quiet(eg_rxquiet),.I_control_ready(1'b1),.O_control_valid(evalid),
        .O_control_kind(ekind),.O_control_epoch(eepoch),.O_control_challenge(echallenge),
        .O_session_ready(eready),.O_barrier(ebarrier),.O_abort_frame(eabort),.O_epoch(eactiveepoch));
    task hello(input [31:0] challenge);
        begin @(negedge clk);hellochallenge=challenge;helloseen=1;@(negedge clk);helloseen=0;end
    endtask
    initial begin
        repeat(3)@(negedge clk);erst=1;
        ownerinvalid=0;rawquiet=0;tilequiet=0;snapshotquiet=0;replyquiet=0;eg_rxquiet=0;
        hello(32'h11);
        repeat(5)@(negedge clk);
        if(eready || evalid || !ebarrier)$fatal(1,"ACK before drain");
        // A newer HELLO replaces the not-yet-committed old ACK.
        hello(32'h22);ownerinvalid=1;rawquiet=1;tilequiet=1;snapshotquiet=1;replyquiet=1;eg_rxquiet=1;
        wait(eready);@(negedge clk);
        if(echallenge!==32'h22 || eepoch!==helloepoch)$fatal(1,"old challenge retained");
        // Repeated PH power reset produces the same final challenge. Its
        // queued old matching ACK is drained while GUARD still rejects ACKs.
        for(round=0;round<2;round=round+1)begin
            mrst=0;repeat(3)@(negedge clk);mrst=1;
            wait(master.S_state==1);@(negedge clk);rxquiet=0;ackseen=1;ackepoch=0;ackchallenge=1;
            repeat(16)@(negedge clk);
            if(mready || master.S_state!=1)$fatal(1,"guard stale same-seed ACK accepted");
            ackseen=0;rxquiet=1;
            wait(master.S_state==3);@(negedge clk);ackseen=1;
            @(negedge clk);ackseen=0;rxquiet=0;
            repeat(10)@(negedge clk);
            if(mready)$fatal(1,"RX quiet interval bypassed");
            rxquiet=1;wait(mready);@(negedge clk);
            if(mactiveepoch!==0 || mchallenge!==1)$fatal(1,"final identity");
        end
        // Independent EG restart requests immediate PH owner invalidation.
        peerhello=1;#1;if(mready)$fatal(1,"peer HELLO not immediate barrier");
        @(negedge clk);peerhello=0;
        if(!mbarrier || mactiveepoch!==1)$fatal(1,"peer recovery epoch");
        $display("TB_RESULT: PASS sessions stale final same-seed/drain/replacement/independent reset");$finish;
    end
    initial begin #200000;$fatal(1,"timeout");end
endmodule
