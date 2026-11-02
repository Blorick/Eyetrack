`timescale 1ns/1ps
// Isolated contract TB for the new raw packet receiver and three-way TX owner.
// This file is intentionally not connected to a production top.  It contains
// independent frame construction and scheduler ownership checks; bad frames
// must never make O_packet_valid observable.
module tb_raw_scheduler_rx_isolated;
  reg rxclk=0, appclk=0, txclk=0;
  always #4 rxclk=~rxclk;
  always #6.5 appclk=~appclk;
  always #5 txclk=~txclk;
  reg rrst_n=0, arst_n=0, trst_n=0;
  reg [7:0] rxd=0; reg rdv=0; reg rerr=0;
  wire pvalid; reg [7:0] raddr=0; wire [7:0] rdata;
  reg prelease=0; wire [31:0] bad_count, drop_count;
  reg [2:0] req=0, done=0, ten=0, ter=0; reg [7:0] t0=0,t1=0,t2=0;
  wire [2:0] grant; wire [7:0] txd; wire txen,txer,owner_valid; wire [1:0] owner;
  wire [2:0] grant_fixed; wire [7:0] txd_fixed; wire txen_fixed,txer_fixed;
  wire owner_valid_fixed; wire [1:0] owner_fixed;
  ctl_ink_packet_rx u_rx(.I_clk_rx(rxclk),.I_rst_rx_n(rrst_n),.I_gmii_rxd(rxd),
    .I_gmii_rx_dv(rdv),.I_gmii_rx_er(rerr),.I_clk_app(appclk),.I_rst_app_n(arst_n),
    .O_packet_valid(pvalid),.I_packet_read_addr(raddr),.O_packet_read_data(rdata),
    .I_packet_release(prelease),.O_bad_frame_count(bad_count),.O_full_drop_count(drop_count));
  ctl_gmii_tx_scheduler3 #(.P_TX_SCHEDULE(1)) u_tx(.I_clk_tx(txclk),.I_rst_tx_n(trst_n),
    .I_tx_request(req),.I_tx_done(done),.O_tx_grant(grant),.I_tx0_txd(t0),
    .I_tx1_txd(t1),.I_tx2_txd(t2),.I_tx_en(ten),.I_tx_er(ter),.O_txd(txd),
    .O_tx_en(txen),.O_tx_er(txer),.O_owner_valid(owner_valid),.O_owner(owner));
  ctl_gmii_tx_scheduler3 #(.P_TX_SCHEDULE(0)) u_tx_fixed(.I_clk_tx(txclk),.I_rst_tx_n(trst_n),
    .I_tx_request(req),.I_tx_done(done),.O_tx_grant(grant_fixed),.I_tx0_txd(t0),
    .I_tx1_txd(t1),.I_tx2_txd(t2),.I_tx_en(ten),.I_tx_er(ter),.O_txd(txd_fixed),
    .O_tx_en(txen_fixed),.O_tx_er(txer_fixed),.O_owner_valid(owner_valid_fixed),.O_owner(owner_fixed));
  reg [31:0] bad_prev=0, drop_prev=0;
  always @(posedge appclk) begin
    if (bad_count < bad_prev) $display("RX_BAD_COUNTER_DECREASE");
    if (drop_count < drop_prev) $display("RX_DROP_COUNTER_DECREASE");
    bad_prev <= bad_count; drop_prev <= drop_count;
  end

  reg [7:0] frame [0:1023]; integer flen, i, k; integer force_bad_flags=0;
  function [31:0] crc8;
    input [31:0] c; input [7:0] d; reg [31:0] m;
    begin
      // Bitwise reflected CRC-32 (IEEE), independent of DUT's nibble table.
      m=c; for(k=0;k<8;k=k+1) begin
        if((m[0]^d[k])!=0) m=(m>>1)^32'hEDB88320; else m=m>>1;
      end crc8=m;
    end
  endfunction
  function [7:0] expected_meta_byte;
    input integer idx;
    begin
      case (idx)
        0: expected_meta_byte=8'h52;
        1: expected_meta_byte=8'h53;
        2: expected_meta_byte=8'h01;
        3: expected_meta_byte=8'h01;
        12: expected_meta_byte=8'h00;
        13: expected_meta_byte=8'h24;
        15: expected_meta_byte=8'h01;
        default: expected_meta_byte=(idx>=16)?((idx-16)^8'h5A):8'h00;
      endcase
    end
  endfunction
  task make_frame;
    input integer kind; input integer plen; input integer bad_at; input integer rx_fault;
    input integer preamble_fault; input integer trunc_bytes; input integer padding_fault;
    integer b, j; reg [31:0] c; reg [7:0] v;
    begin
      for(j=0;j<1024;j=j+1) frame[j]=0;
      for(j=0;j<7;j=j+1) frame[j]=8'h55; frame[7]=8'hD5; b=8;
      if(preamble_fault==1) frame[0]=8'h54;
      if(preamble_fault==2) frame[7]=8'h54;
      frame[b+0]=8'h02; frame[b+1]=8'h00; frame[b+2]=8'h00; frame[b+3]=8'h65; frame[b+4]=8'h43; frame[b+5]=8'h21;
      frame[b+6]=8'h02; frame[b+7]=8'h00; frame[b+8]=8'h00; frame[b+9]=8'h12; frame[b+10]=8'h34; frame[b+11]=8'h56;
      frame[b+12]=8'h88; frame[b+13]=8'hB7; frame[b+14]=8'h52; frame[b+15]=8'h53;
      frame[b+16]=8'h01; frame[b+17]=kind[7:0];
      // App header bytes 4..11 are reserved in this contract; length is
      // header byte 12/13, followed by reserved byte 14 and flags byte 15.
      for(j=18;j<26;j=j+1) frame[b+j]=0;
      frame[b+26]=plen[15:8]; frame[b+27]=plen[7:0];
      frame[b+28]=0; frame[b+29]=(kind==1)?8'h01:8'h00;
      if(force_bad_flags!=0) frame[b+29]=8'h80;
      for(j=0;j<plen;j=j+1) frame[b+30+j]=j[7:0]^8'h5A;
      flen=b+30+plen; if(flen-b < 46) begin for(j=flen;j<b+46;j=j+1) frame[j]=0; flen=b+46; end
      c=32'hFFFF_FFFF; for(j=b;j<flen;j=j+1) c=crc8(c,frame[j]); c=~c;
      frame[flen+0]=c[7:0]; frame[flen+1]=c[15:8]; frame[flen+2]=c[23:16]; frame[flen+3]=c[31:24]; flen=flen+4;
      if(bad_at>=0) frame[b+bad_at]=frame[b+bad_at]^8'h01;
      if(padding_fault!=0 && plen<16) frame[b+30+plen]=8'hA5;
      if(trunc_bytes>0) flen=flen-trunc_bytes;
      @(negedge rxclk); for(j=0;j<flen;j=j+1) begin rxd=frame[j]; rdv=1; rerr=(rx_fault!=0 && j==rx_fault); @(negedge rxclk); end
      rdv=0; rerr=0; rxd=0; @(negedge rxclk);
    end
  endtask

  initial begin
    repeat(3) @(posedge rxclk); rrst_n=1; arst_n=1; trst_n=1;
    // Valid META and LINE are accepted; independent data read is checked.
    make_frame(1,36,-1,0,0,0,0); repeat(6) @(posedge appclk);
    if(!pvalid) $display("RX_VALID_FAIL");
    for(i=0;i<52;i=i+1) begin
      raddr=i[7:0]; @(posedge appclk); #1;
      if(rdata!==expected_meta_byte(i)) $display("RX_DATA_FAIL idx=%0d got=%02x exp=%02x",i,rdata,expected_meta_byte(i));
    end
    prelease=1; @(posedge appclk); prelease=0;
    // Other-MAC/header fault, RX_ER, and contract-length fault are never published.
    make_frame(1,36,0,0,0,0,0); make_frame(2,162,-1,10,0,0,0); make_frame(1,35,-1,0,0,0,0);
    // Strict preamble, CRC/body, MAC/type/version/contract/header faults.
    make_frame(1,36,-1,0,1,0,0); make_frame(1,36,-1,0,2,0,0);
    make_frame(1,36,6,0,0,0,0); make_frame(1,36,12,0,0,0,0);
    make_frame(1,36,16,0,0,0,0); make_frame(9,0,-1,0,0,0,0);
    make_frame(1,65535,-1,0,0,8,0); make_frame(1,0,-1,0,0,0,1);
    make_frame(2,162,28,0,0,0,0); make_frame(2,162,29,0,0,0,0);
    make_frame(5,0,-1,0,0,0,0); make_frame(6,0,-1,0,0,0,0);
    make_frame(7,16,-1,0,0,0,0); make_frame(8,0,-1,0,0,0,0);
    make_frame(1,36,-1,1,0,0,0); make_frame(1,36,-1,20,0,0,0);
    make_frame(1,36,-1,46,0,0,0);
    if(pvalid) $display("RX_BAD_FRAME_EXPOSED");
    // Eight committed banks fill the ring; ninth good target must drop.
    for(i=0;i<9;i=i+1) make_frame(1,36,-1,0,0,0,0);
    repeat(12) @(posedge appclk);
    if(drop_count==0) $display("RX_FULL_DROP_NOT_COUNTED");
    prelease=1; repeat(8) @(posedge appclk); prelease=0;
    // Repeated release/reuse crosses the 4-bit pointer wrap at least 32 times.
    for(i=0;i<32;i=i+1) begin
      make_frame(2,162,-1,0,0,0,0); repeat(4) @(posedge appclk);
      prelease=1; @(posedge appclk); prelease=0;
    end
    // Scheduler: one-hot grant, persistent owner, normal done+withdraw, abort IFG.
    @(negedge txclk); req=3'b111; @(posedge txclk); #1 if(grant!==3'b001) $display("TX_PICK_FAIL");
    if(!owner_valid || owner!==0) $display("TX_OWNER_FAIL");
    done=3'b001; req=3'b110; @(posedge txclk); done=0; @(posedge txclk);
    if(grant!=='0 && (grant&(grant-1))!==0) $display("TX_GRANT_NOT_ONEHOT");
    req=3'b010; repeat(14) @(posedge txclk); if(owner_valid && owner==0) $display("TX_ABORT_IFG_FAIL");
    // Thirty all-request rounds exercise RR fairness; reset both schedulers so
    // fixed-priority's first acquisition is unambiguously owner 0.
    req=0; done=0; trst_n=0; repeat(2) @(posedge txclk); trst_n=1; req=3'b111;
    for(i=0;i<30;i=i+1) begin
      repeat(2) @(posedge txclk); done = (3'b001 << owner); req = req & ~(3'b001 << owner);
      @(posedge txclk); done=0; req=3'b111;
    end
    if(owner_valid_fixed && owner_fixed!==2'd0) begin
      $display("TX_FIXED_PRIORITY_FAIL owner=%0d req=%b done=%b grant=%b",owner_fixed,req,done,grant_fixed);
    end
    // Persistent done while request is withdrawn is the abort path and must hold IFG.
    req=0; done=0; repeat(3) @(posedge txclk); req=3'b001;
    repeat(3) @(posedge txclk); req=0; done=3'b001;
    repeat(4) @(posedge txclk); if(owner_valid) $display("TX_PERSISTENT_DONE_ABORT_FAIL");
    if(bad_count!==32'd19) $display("RX_BAD_COUNT_MISMATCH got=%0d exp=19",bad_count);
    if(drop_count!==32'd1) $display("RX_DROP_COUNT_MISMATCH got=%0d exp=1",drop_count);
    $display("RAW_SCHED_RX_TB_COMPLETE bad=%0d drop=%0d",bad_count,drop_count); $finish;
  end
endmodule
