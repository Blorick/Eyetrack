`timescale 1ns/1ps
// Isolated mailbox CDC contract TB.  No production top includes this file.
module tb_mailbox_cdc_isolated;
  reg sclk=0,dclk=0; always #5 sclk=~sclk; always #20 dclk=~dclk;
  reg rsn=0, rdn=0; reg v41=0,v72=0,ready41=0,ready72=0;
  reg [40:0] data41=0; reg [71:0] data72=0;
  wire ready_src41,pending41,valid41; wire [40:0] out41;
  wire ready_src72,pending72,valid72; wire [71:0] out72;
  integer errors=0, transfers=0, i, phase, j;
  integer sent41=0, sent72=0, recv41=0, recv72=0;
  reg [40:0] held41; reg [71:0] held72;
  reg [40:0] seen41 [0:1119]; reg [71:0] seen72 [0:1119];
  ctl_mailbox_cdc #(.P_DATA_WIDTH(41)) u41(.I_clk_src(sclk),.I_rst_src_n(rsn),
    .I_src_valid(v41),.I_src_data(data41),.O_src_ready(ready_src41),.O_src_pending(pending41),
    .I_clk_dst(dclk),.I_rst_dst_n(rdn),.O_dst_valid(valid41),.O_dst_data(out41),.I_dst_ready(ready41));
  ctl_mailbox_cdc #(.P_DATA_WIDTH(72)) u72(.I_clk_src(sclk),.I_rst_src_n(rsn),
    .I_src_valid(v72),.I_src_data(data72),.O_src_ready(ready_src72),.O_src_pending(pending72),
    .I_clk_dst(dclk),.I_rst_dst_n(rdn),.O_dst_valid(valid72),.O_dst_data(out72),.I_dst_ready(ready72));

  task send41; input integer n; input integer hold; integer t; reg [40:0] exp;
    begin
      exp=n ^ (phase*41); while(!ready_src41) @(posedge sclk);
      if (!ready_src41 || pending41) begin errors=errors+1; $display("MB41_ACCEPT_GATE_FAIL phase=%0d n=%0d",phase,n); end
      data41=exp; v41=1; @(posedge sclk); #1 v41=0;
      sent41=sent41+1; transfers=transfers+1;
      if (!pending41 || ready_src41) begin errors=errors+1; $display("MB41_PENDING_FAIL phase=%0d n=%0d",phase,n); end
      while(!valid41) @(posedge dclk);
      #1 if(out41!==exp) begin errors=errors+1; $display("MB41_ORDER_FAIL phase=%0d n=%0d exp=%h got=%h",phase,n,exp,out41); end
      for (j=0;j<recv41;j=j+1)
        if (seen41[j]===out41) begin errors=errors+1; $display("MB41_DUP_FAIL phase=%0d n=%0d value=%h",phase,n,out41); end
      seen41[recv41]=out41; recv41=recv41+1;
      if (!pending41 || ready_src41) begin errors=errors+1; $display("MB41_HELD_GATE_FAIL phase=%0d n=%0d",phase,n); end
      held41=out41;
      repeat(hold) begin @(posedge dclk); #1 if(!valid41 || out41!==held41 || !pending41 || ready_src41) begin errors=errors+1; $display("MB41_HOLD_FAIL phase=%0d n=%0d",phase,n); end end
      ready41=1; @(posedge dclk); #1;
      if (valid41 || ready_src41 || !pending41) begin errors=errors+1; $display("MB41_CONSUME_EDGE_FAIL phase=%0d n=%0d",phase,n); end
      ready41=0;
    end
  endtask
  task send72; input integer n; input integer hold; integer t; reg [71:0] exp;
    begin
      exp=n ^ (phase*72); while(!ready_src72) @(posedge sclk);
      if (!ready_src72 || pending72) begin errors=errors+1; $display("MB72_ACCEPT_GATE_FAIL phase=%0d n=%0d",phase,n); end
      data72=exp; v72=1; @(posedge sclk); #1 v72=0;
      sent72=sent72+1; transfers=transfers+1;
      if (!pending72 || ready_src72) begin errors=errors+1; $display("MB72_PENDING_FAIL phase=%0d n=%0d",phase,n); end
      while(!valid72) @(posedge dclk);
      #1 if(out72!==exp) begin errors=errors+1; $display("MB72_ORDER_FAIL phase=%0d n=%0d exp=%h got=%h",phase,n,exp,out72); end
      for (j=0;j<recv72;j=j+1)
        if (seen72[j]===out72) begin errors=errors+1; $display("MB72_DUP_FAIL phase=%0d n=%0d value=%h",phase,n,out72); end
      seen72[recv72]=out72; recv72=recv72+1;
      if (!pending72 || ready_src72) begin errors=errors+1; $display("MB72_HELD_GATE_FAIL phase=%0d n=%0d",phase,n); end
      held72=out72;
      repeat(hold) begin @(posedge dclk); #1 if(!valid72 || out72!==held72 || !pending72 || ready_src72) begin errors=errors+1; $display("MB72_HOLD_FAIL phase=%0d n=%0d",phase,n); end end
      ready72=1; @(posedge dclk); #1;
      if (valid72 || ready_src72 || !pending72) begin errors=errors+1; $display("MB72_CONSUME_EDGE_FAIL phase=%0d n=%0d",phase,n); end
      ready72=0;
    end
  endtask
  initial begin
    repeat(3) @(posedge sclk); rsn=1; rdn=1;
    // 40 unrelated phase/latency points, 28 words per width, with long stalls.
    for(phase=0;phase<40;phase=phase+1) begin
      rsn=0; rdn=0; #(phase+1);
      if ((phase%2)==0) begin rsn=1; #(phase%3+1); rdn=1; end
      else begin rdn=1; #(phase%3+1); rsn=1; end
      #1 if (valid41 || valid72 || pending41 || pending72 || !ready_src41 || !ready_src72) begin
        errors=errors+1; $display("MB_RESET_CLEAR_FAIL phase=%0d v41=%b v72=%b p41=%b p72=%b r41=%b r72=%b",phase,valid41,valid72,pending41,pending72,ready_src41,ready_src72);
      end
      for(i=0;i<28;i=i+1) begin send41(i, (i%5)); send72(i, ((i+phase)%7)); end
    end
    // Reset while source has a pending tuple, while destination valid, and clear.
    while(!ready_src41) @(posedge sclk); data41=41'h155; v41=1; @(posedge sclk); #1 v41=0;
    if (!pending41 || ready_src41) begin errors=errors+1; $display("MB_RESET_PENDING_SETUP_FAIL"); end
    repeat(1) @(posedge dclk); rsn=0; rdn=0; repeat(2) @(posedge sclk); rsn=1; rdn=1;
    #1 if (valid41 || valid72 || pending41 || pending72 || !ready_src41 || !ready_src72) begin
      errors=errors+1; $display("MB_RESET_INFLIGHT_CLEAR_FAIL v41=%b v72=%b p41=%b p72=%b r41=%b r72=%b",valid41,valid72,pending41,pending72,ready_src41,ready_src72);
    end
    repeat(6) @(posedge dclk); ready41=1; @(posedge dclk); #1;
    if (valid41) begin errors=errors+1; $display("MB_RESET_STALE_OUTPUT_FAIL"); end
    ready41=0;
    if (sent41!=1120 || recv41!=1120 || sent72!=1120 || recv72!=1120 || transfers!=2240)
      begin errors=errors+1; $display("MB_COUNT_FAIL sent41=%0d recv41=%0d sent72=%0d recv72=%0d transfers=%0d",sent41,recv41,sent72,recv72,transfers); end
    if(errors==0) $display("MAILBOX_CDC_TB_PASS transfers=%0d phases=40 sent41=%0d recv41=%0d sent72=%0d recv72=%0d unique=2240",transfers,sent41,recv41,sent72,recv72);
    else $display("MAILBOX_CDC_TB_FAIL errors=%0d transfers=%0d sent41=%0d recv41=%0d sent72=%0d recv72=%0d",errors,transfers,sent41,recv41,sent72,recv72);
    $finish;
  end
endmodule
