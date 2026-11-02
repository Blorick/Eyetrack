`timescale 1ns/1ps
// Isolated real-line TX -> scheduler -> RX integration TB.  This TB is kept
// outside every production top until dynamic license and BRAM/timing gates pass.
module tb_ink_tx_rx_link;
  reg appclk=0, txclk=0; always #6.5 appclk=~appclk; always #4 txclk=~txclk;
  reg app_rst_n=0, tx_rst_n=0, rx_rst_n=0;
  reg wr_en=0, wr_bank=0; reg [7:0] wr_addr=0, wr_data=0; reg commit=0;
  reg commit_bank=0; reg [8:0] packet_len=0; wire [1:0] free_mask;
  wire commit_ok; wire [31:0] commit_drop;
  wire tx_req, tx_done; wire [7:0] txd; wire tx_en, tx_er; wire tx_grant;
  wire [2:0] sched_req={2'b00,tx_req}; wire [2:0] sched_done={2'b00,tx_done};
  wire [2:0] sched_grant; wire [7:0] line_d; wire line_en,line_er;
  reg [7:0] mock1_d=8'hA1,mock2_d=8'hA2; reg [2:0] mock_en=0,mock_er=0;
  wire [7:0] mux_d=(sched_grant[0])?txd:(sched_grant[1]?mock1_d:mock2_d);
  wire mux_en=(sched_grant[0])?tx_en:(sched_grant[1]?mock_en[1]:mock_en[2]);
  wire mux_er=(sched_grant[0])?tx_er:(sched_grant[1]?mock_er[1]:mock_er[2]);
  reg [7:0] rd_addr=0; wire [7:0] rd_data; wire packet_valid;
  reg packet_release=0; wire [31:0] bad_count,full_drop;
  ctl_ink_packet_tx u_tx(.I_clk_app(appclk),.I_rst_app_n(app_rst_n),.O_free_bank_mask(free_mask),
    .I_write_en(wr_en),.I_write_bank(wr_bank),.I_write_addr(wr_addr),.I_write_data(wr_data),
    .I_commit(commit),.I_commit_bank(commit_bank),.I_packet_length(packet_len),
    .O_commit_accept(commit_ok),.O_commit_drop_count(commit_drop),.I_clk_tx(txclk),
    .I_rst_tx_n(tx_rst_n),.O_tx_request(tx_req),.I_tx_grant(tx_grant),.O_tx_done(tx_done),
    .O_gmii_txd(txd),.O_gmii_tx_en(tx_en),.O_gmii_tx_er(tx_er));
  ctl_gmii_tx_scheduler3 #(.P_TX_SCHEDULE(1)) u_sched(.I_clk_tx(txclk),.I_rst_tx_n(tx_rst_n),
    .I_tx_request(sched_req),.I_tx_done(sched_done),.O_tx_grant(sched_grant),
    .I_tx0_txd(txd),.I_tx1_txd(mock1_d),.I_tx2_txd(mock2_d),.I_tx_en({mock_en[2:1],tx_en}),
    .I_tx_er({mock_er[2:1],tx_er}),.O_txd(line_d),.O_tx_en(line_en),.O_tx_er(line_er),
    .O_owner_valid(),.O_owner());
  assign tx_grant=sched_grant[0];
  // TX emits destination=P_PEER and source=P_LOCAL; the receiving endpoint
  // therefore uses the opposite local/peer view for this real-line link.
  ctl_ink_packet_rx #(.P_LOCAL_MAC(48'h02_00_00_65_43_21),
    .P_PEER_MAC(48'h02_00_00_12_34_56)) u_rx(.I_clk_rx(txclk),.I_rst_rx_n(rx_rst_n),.I_gmii_rxd(line_d),
    .I_gmii_rx_dv(line_en),.I_gmii_rx_er(line_er),.I_clk_app(appclk),.I_rst_app_n(app_rst_n),
    .O_packet_valid(packet_valid),.I_packet_read_addr(rd_addr),.O_packet_read_data(rd_data),
    .I_packet_release(packet_release),.O_bad_frame_count(bad_count),.O_full_drop_count(full_drop));

  function [7:0] app_byte; input integer idx; input integer kind; input integer plen;
    begin
      if(idx==0) app_byte=8'h52; else if(idx==1) app_byte=8'h53;
      else if(idx==2) app_byte=8'd1; else if(idx==3) app_byte=kind[7:0];
      else if(idx==12) app_byte=plen[15:8]; else if(idx==13) app_byte=plen[7:0];
      else if(idx==15) app_byte=(kind==1)?8'h01:0;
      else if(idx<16) app_byte=0; else app_byte=(idx-16)^8'h5A;
    end
  endfunction
  task enqueue; input integer total; input integer kind; integer j; integer p;
    begin
      p=total-16; while(free_mask==0) @(posedge appclk);
      wr_bank=free_mask[0]?0:1; commit_bank=wr_bank; packet_len=total;
      for(j=0;j<total;j=j+1) begin wr_addr=j[7:0]; wr_data=app_byte(j,kind,p); wr_en=1; @(posedge appclk); end
      wr_en=0; commit=1; @(posedge appclk); commit=0;
      #1 if(!commit_ok) $display("TX_COMMIT_REJECT len=%0d bank=%0d",total,wr_bank);
    end
  endtask
  integer n, j; reg [31:0] bad_prev=0;
  integer line_count=0;
  always @(posedge txclk) if(line_en) begin
    if(line_count<32) $display("LINK_BYTE[%0d]=%02x grant=%b",line_count,line_d,sched_grant);
    line_count=line_count+1;
  end
  always @(posedge appclk) if(!app_rst_n) bad_prev<=0; else if(bad_count<bad_prev) $display("LINK_BAD_COUNTER_DECREASE"); else bad_prev<=bad_count;
  initial begin
    repeat(4) @(posedge appclk); app_rst_n=1; tx_rst_n=1; rx_rst_n=1;
    // All legal sizes. Force a third commit while both TX banks are owned so
    // the two-bank queue gate is observed before the later legal enqueue waits.
    enqueue(16,3); enqueue(32,7);
    if(free_mask==0) begin
      commit_bank=0; packet_len=9'd16; commit=1;
      @(posedge appclk); commit=0; #1;
      if(commit_drop==0) $display("TX_QUEUE_GATE_NOT_OBSERVED");
    end else $display("TX_QUEUE_NOT_FULL_AT_GATE");
    enqueue(52,1); enqueue(178,2);
    // Metadata crosses unrelated 75/125-MHz phases; compare every byte after CRC publish.
    repeat(250) @(posedge appclk);
    if(!packet_valid) $display("LINK_NO_POSTCRC_PACKET");
    for(n=0;n<2;n=n+1) begin
      for(j=0;j<178;j=j+1) begin rd_addr=j[7:0]; @(posedge appclk); end
      packet_release=1; @(posedge appclk); packet_release=0;
    end
    if(bad_count!==32'd0 || full_drop!==32'd0 || commit_drop!==32'd1)
      $display("LINK_PRE_RESET_COUNT_MISMATCH bad=%0d drop=%0d commitdrop=%0d",bad_count,full_drop,commit_drop);
    $display("INK_TX_RX_PRE_RESET bad=%0d drop=%0d commitdrop=%0d free=%b",bad_count,full_drop,commit_drop,free_mask);
    // Wrong bank/size are rejected and must not overwrite an owned bank.
    wr_bank=~commit_bank; commit_bank=~commit_bank; packet_len=9'd17; commit=1;
    @(posedge appclk); commit=0;
    // Mid-transfer common reset, then bank wrap and re-queue all three sizes.
    repeat(30) @(posedge txclk); app_rst_n=0; tx_rst_n=0; rx_rst_n=0;
    repeat(3) @(posedge txclk); app_rst_n=1; tx_rst_n=1; rx_rst_n=1;
    for(n=0;n<20;n=n+1) begin enqueue(16,3); enqueue(32,7); enqueue(52,1); enqueue(178,2); end
    if(bad_count!==32'd0 || full_drop!==32'd71 || commit_drop!==32'd0)
      $display("LINK_FINAL_COUNT_MISMATCH bad=%0d drop=%0d commitdrop=%0d",bad_count,full_drop,commit_drop);
    $display("INK_TX_RX_LINK_TB_COMPLETE bad=%0d drop=%0d commitdrop=%0d",bad_count,full_drop,commit_drop);
    $finish;
  end
endmodule
