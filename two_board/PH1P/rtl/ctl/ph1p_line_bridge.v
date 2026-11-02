`timescale 1ns/1ps
//=============================================================================
//  ph1p_line_bridge -- v2.5 integrated source
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk
//  复位     : 按本模块接口定义
//  参数     : 见模块参数
//  验证状态 : 见 docs/validation.md
//=============================================================================
//=============================================================================
//  ph1p_line_bridge  --  master-side per-line bridge to the EG4D slave
//-----------------------------------------------------------------------------
//  One Ethernet frame per video line carries that line's LUMA (Y8) to the EG4D
//  slave, which runs the neighbourhood picture modes (v1.13 modes 9..17) and
//  sends the styled line back.  This module puts the returned line back into the
//  display raster.
//
//  WHY LUMA AND NOT RGB565 (measured, see docs/design/v1.19-eg4d-offload-evaluation.md):
//    the master's send path is raster-locked, so the demand is set by the pixel
//    rate, not by the frame rate.  16-bit RGB565 needs 116.4 MB/s = 94.6% of a
//    1 GbE link (infeasible); 8-bit luma needs 58.2 MB/s = 47.3% (comfortable).
//    Modes 9..17 are monochrome anyway, so luma is also the honest payload.
//
//  ELASTIC ALIGNMENT (the user chose "least harm to the picture"):
//    a returned line arrives a few lines after the line it belongs to was sent,
//    so line L cannot be displayed at raster line L.  We therefore display the
//    returned line for (L - P_LAT) at raster line L, i.e. the picture shifts UP
//    by P_LAT lines.  P_LAT is deliberately small (default 3).  Each slot stores
//    the line number it holds, and a read only happens when that number matches,
//    so a lost or late line degrades to the RAW pixel instead of tearing.
//
//  STATUS: written, NOT yet simulated or integrated.  Do not treat as verified.
//=============================================================================
module ph1p_line_bridge #(
    parameter integer IMG_W      = 1280,
    parameter integer P_SLOT     = 8,      // elastic line slots, power of two
    parameter integer P_LAT      = 3,      // raster lines of pipeline delay
    // MUST match ctl_stream_link's P_APP_MAGIC (byte0 = [7:0], byte1 = [15:8]),
    // which is 16'h01A5 by default.  The bridge originally sent 5A/C3, so the
    // link's receive FSM rejected every frame -- a mismatch that only a
    // dual-board simulation can expose.
    parameter [7:0]   P_MAGIC_LO = 8'hA5,
    parameter [7:0]   P_MAGIC_HI = 8'h01,
    parameter integer P_VALIDATE_FRAME = 0,
    parameter integer P_REMOTE_ONLY = 0
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    //---------------------------------------------------- link, receive side
    input  wire        I_rx_valid,
    input  wire [15:0] I_rx_len,
    output reg  [10:0] O_rx_addr,
    input  wire [7:0]  I_rx_rdata,
    output reg         O_rx_release,

    //---------------------------------------------------- link, transmit side
    input  wire        I_tx_ready,
    output reg         O_tx_we,
    output reg  [10:0] O_tx_addr,
    output reg  [7:0]  O_tx_wdata,
    output reg         O_tx_commit,

    //---------------------------------------------------- raster in (from DDR)
    input  wire        I_vsync,
    input  wire        I_de,
    input  wire        I_user,
    input  wire [7:0]  I_y,
    input  wire [4:0]  I_mode,
    input  wire        I_enable,

    //---------------------------------------------------- raster out
    output reg         O_vsync,
    output reg         O_de,
    output reg         O_user,
    output reg  [7:0]  O_y,
    output reg         O_styled,     // 1 = this pixel came back from the slave

    //---------------------------------------------------- status
    output reg         O_link_ok,
    output reg  [15:0] O_out_line
);

    localparam integer P_HDR = 16;

    //=========================================================================
    //  Elastic store: P_SLOT lines of IMG_W luma bytes + per-slot metadata.
    //  Written by the receive path, read by the raster path.  Same clock.
    //=========================================================================
    reg [7:0]  S_mem [0:P_SLOT*IMG_W-1];
    reg [15:0] S_slot_line [0:P_SLOT-1];
    reg        S_slot_ok   [0:P_SLOT-1];
    reg [31:0] S_slot_frame [0:P_SLOT-1];
    reg [4:0] S_slot_mode [0:P_SLOT-1];

    integer i;
    initial begin
        for (i = 0; i < P_SLOT; i = i + 1) begin
            S_slot_line[i] = 16'd0;
            S_slot_ok[i]   = 1'b0;
            S_slot_frame[i] = 32'd0;
            S_slot_mode[i] = 5'd0;
        end
    end

    //=========================================================================
    //  Raster position: line counter and column counter
    //=========================================================================
    reg [15:0] S_line;
    reg [11:0] S_col;          // 0..IMG_W-1 while I_de is high

    wire W_sof = I_vsync;
    reg S_vsync_d;
    reg [31:0] S_frame;
    reg S_de_d;
    wire W_capture_enable = P_REMOTE_ONLY ? I_enable : 1'b1;
    wire W_line_start = I_de && !S_de_d;
    wire [15:0] W_raster_line = W_line_start ? (S_line + 16'd1) : S_line;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_vsync_d <= 1'b0;
            S_frame <= 32'd0;
            S_de_d <= 1'b0;
        end else begin
            S_vsync_d <= I_vsync;
            S_de_d <= I_de;
            if (I_vsync && !S_vsync_d) S_frame <= S_frame + 32'd1;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_line <= 16'hFFFF;
            S_col  <= 12'd0;
        end
        else begin
            if (W_sof) begin
                // The first active DE edge increments -1 to line zero.
                // Production I_user denotes the first pixel of a frame.
                S_line <= 16'hFFFF;
                S_col  <= 12'd0;
            end
            else begin
                if (I_de) begin
                    if (S_col == IMG_W - 1) S_col <= 12'd0;
                    else                               S_col <= S_col + 12'd1;
                end
                // The production VTC user pulse marks a frame, not every line.
                if (W_line_start) S_line <= S_line + 16'd1;
            end
        end
    end

    //=========================================================================
    //  Read side: which stored line should be on screen for raster line L?
    //    want = L - P_LAT   (mod 2^16 -- a negative want simply never matches,
    //    which is exactly the desired "not ready yet -> raw" behaviour)
    //=========================================================================
    wire [15:0] W_want      = W_raster_line - P_LAT[15:0];
    wire [15:0] W_want_slot = W_want & (P_SLOT-1);
    reg  [7:0]  S_rd_data;
    reg         S_rd_ok;
    reg  [11:0] S_col_d;
    reg S_line_styled;
    wire W_slot_matches = S_slot_ok[W_want_slot] &&
                          (S_slot_line[W_want_slot] == W_want) &&
                          (!P_VALIDATE_FRAME ||
                          ((S_slot_frame[W_want_slot] == S_frame) &&
                           (S_slot_mode[W_want_slot] == I_mode)));

    // A row arriving after its first display pixel stays raw for that whole row.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) S_line_styled <= 1'b0;
        else if (W_line_start) S_line_styled <= W_slot_matches;
    end

    // Registered read: address this cycle, data next cycle.  O_de/O_user follow.
    wire [15:0] W_rd_addr = (W_want_slot * IMG_W) + S_col;

    always @(posedge I_clk) begin
        S_rd_data <= S_mem[W_rd_addr];
        S_rd_ok   <= W_line_start ? W_slot_matches : S_line_styled;
        S_col_d   <= S_col;
    end

    // (S_col_d kept only for waveform visibility)
    wire W_use_slave = S_rd_ok && I_de;

    // ★ S_rd_data is ALREADY the registered memory output, so for column c the
    //   data is available one cycle after the address is presented.  O_de/O_user
    //   are registered by that same one cycle, so the luma must be taken
    //   COMBINATIONALLY from S_rd_data -- registering it again put every styled
    //   pixel one column late (the bench caught this as "position c holds column
    //   c-1").  The raw fallback therefore needs its own one-cycle delay so that
    //   raw and styled agree on which column they describe.
    reg [7:0] S_y_d;
    always @(posedge I_clk) S_y_d <= I_y;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_vsync <= 1'b0;
            O_de    <= 1'b0;
            O_user  <= 1'b0;
        end
        else begin
            O_vsync <= I_vsync;
            O_de    <= I_de;
            O_user  <= I_user;
        end
    end

    always @(*) begin
        O_y      = S_rd_ok ? S_rd_data : S_y_d;
        O_styled = S_rd_ok;
    end

    //=========================================================================
    //  Transmit side: push this raster line's luma into the link's TX buffer
    //  while it is being displayed, then write the header and commit.
    //=========================================================================
    reg  [4:0]  S_tx_i;         // header byte index
    reg         S_tx_active;    // a header/commit sequence is in progress
    reg         S_tx_commit_r;
    reg         S_line_done;    // the line just ended -> needs a header+commit
    reg S_tx_line_ok;
    reg [31:0] S_tx_frame;
    reg [15:0] S_tx_line;
    reg [4:0] S_tx_mode;

    wire W_tx_pix = I_de && !S_tx_active && I_tx_ready && W_capture_enable &&
                    ((S_col == 0) || S_tx_line_ok) && (S_col < IMG_W);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tx_we      <= 1'b0;
            O_tx_addr    <= 11'd0;
            O_tx_wdata   <= 8'd0;
            O_tx_commit  <= 1'b0;
            S_tx_i       <= 5'd0;
            S_tx_active  <= 1'b0;
            S_line_done  <= 1'b0;
            S_tx_line_ok <= 1'b0;
            S_tx_frame <= 32'd0;
            S_tx_line <= 16'd0;
            S_tx_mode <= 5'd0;
        end
        else begin
            O_tx_we     <= 1'b0;
            O_tx_commit <= 1'b0;

            // end of line: the last active pixel saw S_col == IMG_W-1
            if (I_de && (S_col == 0))
                S_tx_line_ok <= I_tx_ready && !S_tx_active && W_capture_enable;
            if (I_de && (!I_tx_ready || S_tx_active || !W_capture_enable))
                S_tx_line_ok <= 1'b0;
            if (I_de && (S_col == IMG_W - 1) && W_tx_pix) begin
                S_line_done <= 1'b1;
                S_tx_frame <= S_frame;
                S_tx_line <= W_raster_line;
                S_tx_mode <= I_mode;
            end

            if (W_tx_pix) begin
                O_tx_we    <= 1'b1;
                O_tx_addr  <= P_HDR[10:0] + S_col[10:0];
                O_tx_wdata <= I_y;
            end
            else if (S_line_done && !S_tx_active) begin
                S_tx_active <= 1'b1;
                S_tx_i      <= 5'd0;
            end

            if (S_tx_active) begin
                // 16 header bytes, one per cycle
                O_tx_we    <= 1'b1;
                O_tx_addr  <= {6'd0, S_tx_i};   // S_tx_i is 5 bits: S_tx_i[10:0] is an out-of-range part-select and reads X
                O_tx_wdata <= (S_tx_i == 5'd0)  ? P_MAGIC_LO :
                              (S_tx_i == 5'd1)  ? P_MAGIC_HI :
                              (S_tx_i == 5'd2)  ? S_tx_line[7:0] :
                              (S_tx_i == 5'd3)  ? S_tx_line[15:8] :
                              (S_tx_i == 5'd4)  ? {3'd0, S_tx_mode} :
                              (S_tx_i == 5'd5)  ? {7'd0, W_sof} :
                              // ★ ctl_stream_link defines W_r_flen = (this field) +
                              //   P_HDR_BYTES, so bytes 6..7 hold the PAYLOAD length,
                              //   not the whole application frame.  Writing
                              //   IMG_W + P_HDR here made the peer believe the frame
                              //   was 40 bytes instead of 24.  The slave adapter
                              //   (eg4d_line_adapter.v) already writes IMG_W.
                              (S_tx_i == 5'd6)  ? IMG_W[7:0] :
                              (S_tx_i == 5'd7)  ? {5'd0, IMG_W[10:8]} :
                              (S_tx_i == 5'd8)  ? S_tx_frame[7:0] :
                              (S_tx_i == 5'd9)  ? S_tx_frame[15:8] :
                              (S_tx_i == 5'd10) ? S_tx_frame[23:16] :
                              (S_tx_i == 5'd11) ? S_tx_frame[31:24] :
                              8'h00;
                if (S_tx_i == 5'd15) begin
                    S_tx_active <= 1'b0;
                    S_line_done <= 1'b0;
                    O_tx_commit <= 1'b1;
                end
                else S_tx_i <= S_tx_i + 5'd1;
            end
        end
    end

    //=========================================================================
    //  Receive side: header -> slot number, then stream the payload into the
    //  slot.  Same frame layout as the transmit side (and as the slave's
    //  eg4d_line_adapter).
    //=========================================================================
    localparam [2:0] R_IDLE = 3'd0, R_HDR = 3'd1, R_PAY = 3'd2, R_REL = 3'd3;
    localparam [2:0] ST_RX_RELEASE_WAIT = 3'd4;

    reg [2:0]  S_rst;
    // Two-deep address pipeline: S_r_a1 is the address whose data is on
    // I_rx_rdata right now (the link registers its read output).
    reg [10:0] S_r_a1;
    always @(posedge I_clk) begin
        S_r_a1 <= O_rx_addr;
    end
    reg [15:0] S_r_line;
    reg [4:0]  S_r_mode;
    reg [15:0] S_r_flen;
    reg [7:0]  S_r_magic;
    reg [31:0] S_r_frame;
    reg S_r_valid;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_rst       <= R_IDLE;
            S_r_line    <= 16'd0;
            S_r_mode    <= 5'd0;
            S_r_flen    <= 16'd0;
            S_r_magic   <= 8'd0;
            S_r_frame <= 32'd0;
            S_r_valid <= 1'b0;
            O_rx_addr   <= 11'd0;
            // ★ O_link_ok must NOT be reset here: it is owned by the link-health
            //   block below.  Two always blocks driving one reg is exactly the
            //   defect TD rejects with HDL-8007 (EXP-036) -- and Questa hid it,
            //   so the M1 bench passed while TD refused to elaborate the module.
            O_rx_release<= 1'b0;
        end
        else begin
            O_rx_release <= 1'b0;
            case (S_rst)
                R_IDLE: begin
                    if (I_rx_valid) begin
                        O_rx_addr <= 11'd0;      // address the link sees next cycle
                        S_r_magic <= 8'd0;
                        S_r_frame <= 32'd0;
                        S_r_valid <= 1'b0;
                        S_rst     <= R_HDR;
                    end
                end
                // ctl_stream_link's receive read port is REGISTERED:
                //   assign O_rx_rdata = S_rx_rdata;  S_rx_rdata <= mem[I_rx_addr];
                // so a byte appears on I_rx_rdata TWO clocks after this module
                // assigns O_rx_addr.  S_r_a1 recovers which address the
                // byte currently on the bus belongs to.  Sampling at the counter
                // instead read every field two bytes early, so the slot's stored
                // line number never matched what the raster asked for and no
                // pixel was ever styled.  eg4d_line_adapter.v documents this same
                // two-cycle latency; this module had it wrong.
                R_HDR: begin
                    if (S_r_a1 == 11'd0) S_r_magic[0] <= (I_rx_rdata == P_MAGIC_LO);
                    if (S_r_a1 == 11'd1) S_r_magic[1] <= (I_rx_rdata == P_MAGIC_HI);
                    if (S_r_a1 == 11'd2) S_r_line[7:0]   <= I_rx_rdata;
                    if (S_r_a1 == 11'd3) S_r_line[15:8]  <= I_rx_rdata;
                    if (S_r_a1 == 11'd4) S_r_mode        <= I_rx_rdata[4:0];
                    if (S_r_a1 == 11'd6) S_r_flen[7:0]   <= I_rx_rdata;
                    if (S_r_a1 == 11'd7) S_r_flen[15:8]  <= I_rx_rdata;
                    if (S_r_a1 == 11'd8) S_r_frame[7:0] <= I_rx_rdata;
                    if (S_r_a1 == 11'd9) S_r_frame[15:8] <= I_rx_rdata;
                    if (S_r_a1 == 11'd10) S_r_frame[23:16] <= I_rx_rdata;
                    if (S_r_a1 == 11'd11) S_r_frame[31:24] <= I_rx_rdata;
                    if (S_r_a1 == 11'd12) S_r_valid <= I_rx_rdata[0];
                    if (S_r_a1 == P_HDR[10:0] - 11'd1) begin
                        O_rx_addr <= P_HDR[10:0];      // first payload byte
                        if ((S_r_magic == 2'b11) && (S_r_flen == IMG_W) &&
                            (I_rx_len == P_HDR + IMG_W) &&
                            (!P_VALIDATE_FRAME || ((S_r_frame == S_frame) &&
                             (S_r_mode == I_mode) && S_r_valid)))
                            S_rst <= R_PAY;
                        else S_rst <= R_REL;
                    end
                    else O_rx_addr <= O_rx_addr + 11'd1;
                end
                R_PAY: begin
                    if ((S_r_a1 >= P_HDR[10:0]) && (S_r_a1 < P_HDR[10:0] + IMG_W))
                        S_mem[(S_r_line & (P_SLOT-1)) * IMG_W + (S_r_a1 - P_HDR[10:0])] <= I_rx_rdata;
                    if (S_r_a1 == P_HDR[10:0] + IMG_W - 1) S_rst <= R_REL;
                    else O_rx_addr <= O_rx_addr + 11'd1;
                end
                R_REL: begin
                    O_rx_release <= 1'b1;
                    S_rst <= ST_RX_RELEASE_WAIT;
                end
                // The link samples the registered release at this edge. A new
                // ping-pong buffer may be valid immediately after that sample.
                ST_RX_RELEASE_WAIT: begin
                    S_rst <= R_IDLE;
                end
                default: S_rst <= R_IDLE;
            endcase
        end
    end

    // Publish the slot only once the whole payload has landed.
    reg S_pub;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_pub <= 1'b0;
            for (i = 0; i < P_SLOT; i = i + 1) S_slot_ok[i] <= 1'b0;
        end
        else begin
            S_pub <= (S_rst == R_PAY) && (S_r_a1 == P_HDR[10:0] + IMG_W - 1);
            if ((S_rst == R_HDR) && (S_r_a1 == P_HDR[10:0] - 11'd1))
                S_slot_ok[S_r_line & (P_SLOT-1)] <= 1'b0;
            if (S_pub) begin
                S_slot_line[S_r_line & (P_SLOT-1)] <= S_r_line;
                S_slot_frame[S_r_line & (P_SLOT-1)] <= S_r_frame;
                S_slot_mode[S_r_line & (P_SLOT-1)] <= S_r_mode;
                // S_r_magic is a two-bit FLAG MASK (bit0/bit1 = byte0/byte1
                // matched), not the magic byte itself.  Comparing it against
                // P_MAGIC_LO left every slot marked invalid, so nothing was ever
                // styled even though the parse was perfect.
                S_slot_ok  [S_r_line & (P_SLOT-1)] <= (S_r_magic == 2'b11);
            end
        end
    end

    //=========================================================================
    //  Link health: a good line is published regularly or we fall back to raw.
    //=========================================================================
    reg [6:0] S_quiet;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_quiet   <= 7'd0;
            O_link_ok <= 1'b0;
        end
        else begin
            if (I_user) begin
                if (S_quiet != 7'h7F) S_quiet <= S_quiet + 7'd1;
                O_link_ok <= (S_quiet < 7'd8);   // up to 8 quiet lines tolerated
            end
            if (S_pub) S_quiet <= 7'd0;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) O_out_line <= 16'd0;
        else          O_out_line <= W_want;
    end

endmodule
