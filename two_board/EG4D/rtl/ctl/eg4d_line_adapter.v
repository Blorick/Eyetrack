`timescale 1ns/1ps
//=============================================================================
//  eg4d_line_adapter -- v2.5 integrated source
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk
//  复位     : 按本模块接口定义
//  参数     : 见模块参数
//  验证状态 : 见 docs/validation.md
//=============================================================================
//=============================================================================
//  eg4d_line_adapter  --  link <-> raster bridge for the EG4D slave
//-----------------------------------------------------------------------------
//  Domain : the slave's application (pixel) clock.
//
//  The link delivers ONE video line per Ethernet frame:
//      [16-byte application header][1280 bytes of Y8 or RGB332]
//  and expects the same shape back.  A 3x3 filter wants a continuous raster, so
//  this module is the bridge:
//
//    * read the received header (line number, mode, SOF), then stream its 1280
//      luma bytes into eg4d_mode_filter as one active video line;
//    * WHILE line L is streamed in, the filter is emitting line L-1 (its
//      latency is exactly one line), so those outputs are written straight into
//      the link's transmit buffer;
//    * then write the 16-byte application header and commit the frame.
//
//  Line 0 of a frame is fed but NOT transmitted: the filter has no row history
//  yet.  Feeding it is what primes the two row memories.
//
//  ★ Every one of the 16 header bytes must be written.  The link's transmit FSM
//    takes the payload length from header bytes 6..7, so a partially written
//    header silently produces a malformed frame (it truncates or over-runs).
//
//  Mode 18 uses the same one-byte slot for RGB332 (R3G3B2).  The edge decision
//  is applied inside the slave filter, so no second edge plane is needed and
//  the PH1P bridge can unpack a real colour result without pretending it is Y8.
//
//  Verification: tb_eg4d_line_adapter.sv exercises sequence/mode/line-gap,
//  mode-17 latency, valid-bit propagation, and RX release/recovery.
//=============================================================================
module eg4d_line_adapter #(
    parameter integer IMG_W       = 1280,
    parameter integer P_HDR_BYTES = 16,
    parameter [15:0]  P_APP_MAGIC = 16'h01A5
)(
    input  wire        I_clk,
    input  wire        I_rst_n,

    //------------------------------------------------- link receive side
    input  wire        I_rx_valid,        // a fully validated frame is held
    input  wire [15:0] I_rx_len,
    output reg  [10:0] O_rx_addr,
    input  wire [7:0]  I_rx_rdata,        // registered read
    output reg         O_rx_release,

    //------------------------------------------------- link transmit side
    input  wire        I_tx_ready,        // at least one TX buffer free
    output wire        O_tx_wbuf,
    output reg         O_tx_we,
    output reg  [10:0] O_tx_addr,
    output reg  [7:0]  O_tx_wdata,
    output reg         O_tx_commit,

    //------------------------------------------------- video to the filter
    output reg         O_fil_vsync,
    output reg         O_fil_hsync,
    output reg         O_fil_de,
    output reg         O_fil_user,
    output reg         O_fil_last,
    output reg  [7:0]  O_fil_y,
    output reg  [4:0]  O_fil_mode,

    //------------------------------------------------- video from the filter
    input  wire        I_fil_vsync,
    input  wire        I_fil_hsync,
    input  wire        I_fil_de,
    input  wire        I_fil_user,
    input  wire        I_fil_last,
    input  wire [7:0]  I_fil_y,
    input  wire        I_fil_edge,
    input  wire        I_fil_window_valid
);

    localparam [10:0] W_IMG_W = IMG_W[10:0];

    localparam [2:0] S_IDLE = 3'd0,
                     S_HDR  = 3'd1,
                     S_LINE = 3'd2,
                     S_TAIL = 3'd3,
                     S_CMT  = 3'd4,
                     S_REL  = 3'd5,
                     S_WAIT = 3'd6,
                     S_DRAIN = 3'd7;

    reg [2:0]  S_state;
    reg [4:0]  S_hdr_i;        // header byte index being fetched
    reg [10:0] S_k;            // payload byte index on I_rx_rdata
    reg [10:0] S_oidx;         // filter output index within the outgoing line
    reg [4:0]  S_tail_i;       // outgoing header byte index
    reg [4:0]  S_mode;
    reg [15:0] S_line;         // line number of the frame being received
    reg [31:0] S_frame_seq;
    reg        S_sof;
    reg [4:0]  S_drain;        // consecutive output-idle cycles before the header
    reg        S_first_line;
    reg        S_seq_seen;
    reg [31:0] S_active_frame_seq;
    reg [15:0] S_last_line;
    reg [15:0] S_segment_line;
    reg        S_force_vsync;
    reg        S_frame_gap;
    reg        S_output_valid;

    // ★ The transmit buffer has ONE write port and ONE owner.  The header bytes
    //   (written from the sequencer) and the payload bytes (written as the
    //   filter emits them) must not be driven from two always blocks: that is a
    //   multi-driver race whose winner is simulator-dependent.  The sequencer
    //   therefore only raises a REQUEST here; the single block below arbitrates,
    //   giving the header priority.  The two never overlap in time (the filter
    //   is idle while the tail is written), so no payload byte is lost.
    // ★ Ownership: a register must have exactly ONE always block writing it.
    //   The first version had the sequencer SET S_tw_req and the payload
    //   collector CLEAR it -- two drivers on one register.  Questa's
    //   last-write-wins made the simulation pass, but TD rejected the design
    //   outright ("net S_tw_req is constantly driven from multiple places").
    //   Now: S_hdr_req is written ONLY by the sequencer, S_hdr_ack ONLY by the
    //   collector, and the two form a request/acknowledge handshake.
    reg        S_hdr_req;
    reg        S_hdr_ack;

    assign O_tx_wbuf = 1'b0;   // the link picks the free buffer itself

    // outgoing header byte mux
    // The primary 3x3 path emits the previous centre row.  Mode 17 adds a
    // second 3x3 mask pass, so its centre is two input rows behind.  Clamp the
    // underflow; such early rows carry processed_line_valid=0 and are ignored
    // by the PH1P bridge.
    wire [15:0] W_out_line =
        (S_mode == 5'd17) ? ((S_line >= 16'd2) ? (S_line - 16'd2) : 16'd0) :
                            ((S_line >= 16'd1) ? (S_line - 16'd1) : 16'd0);
    wire [7:0]  W_tail_byte =
          (S_tail_i == 5'd0)  ? P_APP_MAGIC[7:0]
        : (S_tail_i == 5'd1)  ? P_APP_MAGIC[15:8]
        : (S_tail_i == 5'd2)  ? W_out_line[7:0]
        : (S_tail_i == 5'd3)  ? W_out_line[15:8]
        : (S_tail_i == 5'd4)  ? {3'b000, S_mode}
        : (S_tail_i == 5'd5)  ? {7'b0, S_sof}
        : (S_tail_i == 5'd6)  ? W_IMG_W[7:0]        // payload length, low
        : (S_tail_i == 5'd7)  ? {5'b0, W_IMG_W[10:8]}  // payload length, high
        : (S_tail_i == 5'd8)  ? S_frame_seq[7:0]
        : (S_tail_i == 5'd9)  ? S_frame_seq[15:8]
        : (S_tail_i == 5'd10) ? S_frame_seq[23:16]
        : (S_tail_i == 5'd11) ? S_frame_seq[31:24]
        : (S_tail_i == 5'd12) ? {7'd0, S_output_valid}
        : 8'h00;

    //=========================================================================
    // filter output -> transmit buffer
    //   I_fil_user marks the start of an output line, so the index is reset
    //   there and every following I_fil_de writes the next byte.
    //=========================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_oidx    <= 11'd0;
            S_hdr_ack <= 1'b0;
            O_tx_we   <= 1'b0;
            O_tx_addr <= 11'd0;
            O_tx_wdata <= 8'd0;
        end
        else begin
            // single owner of the transmit write port; header request has
            // priority, payload otherwise
            if (S_hdr_req && !S_hdr_ack) begin
                // header byte for the outgoing frame; W_tail_byte is a pure
                // function of S_tail_i and the latched header fields
                O_tx_we    <= 1'b1;
                O_tx_addr  <= {6'd0, S_tail_i};
                O_tx_wdata <= W_tail_byte;
                S_hdr_ack  <= 1'b1;
            end
            else begin
                S_hdr_ack <= 1'b0;
                O_tx_we <= 1'b0;
                if (I_fil_de) begin
                    // I_fil_user and I_fil_de are high TOGETHER on the first
                    // pixel of a line.  Writing
                    //     if (I_fil_user) S_oidx <= 0;
                    //     if (I_fil_de)   S_oidx <= S_oidx + 1;
                    // meant the increment won (last assignment wins), so the
                    // index never reset and the second line's bytes landed at
                    // 16+IMG_W onwards instead of 16.  The reset must have
                    // PRIORITY, hence the ternary.
                    O_tx_we    <= 1'b1;
                    O_tx_addr  <= P_HDR_BYTES[10:0] + (I_fil_user ? 11'd0 : S_oidx);
                    // A line is still returned with a fixed 1280-byte
                    // payload, but a border/early row without a complete
                    // 3x3 centre must be black.  Do not let stale pipeline
                    // data masquerade as a processed pixel.
                    O_tx_wdata <= I_fil_window_valid ? I_fil_y : 8'h00;
                    S_oidx     <= I_fil_user ? 11'd1 : (S_oidx + 11'd1);
                end
            end
        end
    end

    //=========================================================================
    // main sequencer
    //=========================================================================
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_state <= S_IDLE;
            S_hdr_i <= 5'd0; S_k <= 11'd0; S_tail_i <= 5'd0;
            S_hdr_req <= 1'b0;   // else X   (S_hdr_ack is reset by the collector)
            S_drain   <= 5'd0;
            S_mode <= 5'd0; S_line <= 16'd0; S_frame_seq <= 32'd0; S_sof <= 1'b0;
            S_first_line <= 1'b1;
            S_seq_seen <= 1'b0; S_active_frame_seq <= 32'd0; S_last_line <= 16'd0;
            S_segment_line <= 16'd0; S_force_vsync <= 1'b0;
            S_frame_gap <= 1'b0; S_output_valid <= 1'b0;
            O_rx_addr <= 11'd0; O_rx_release <= 1'b0; O_tx_commit <= 1'b0;
            O_fil_vsync <= 1'b0; O_fil_hsync <= 1'b0; O_fil_de <= 1'b0;
            O_fil_user <= 1'b0; O_fil_last <= 1'b0; O_fil_y <= 8'd0;
            O_fil_mode <= 5'd0;
        end
        else begin
            O_rx_release <= 1'b0;
            O_tx_commit  <= 1'b0;
            O_fil_de     <= 1'b0;
            O_fil_user   <= 1'b0;
            O_fil_last   <= 1'b0;
            O_fil_hsync  <= 1'b0;

            case (S_state)
            //-------------------------------------------------------------
            S_IDLE: begin
                if (I_rx_valid && I_tx_ready) begin
                    S_hdr_i   <= 5'd1;
                    O_rx_addr <= 11'd0;    // byte 0 lands on I_rx_rdata next cycle
                    S_state   <= S_HDR;
                end
            end

            //-------------------------------------------------------------
            // I_rx_rdata is a registered read: the byte for address a is
            // presented on the cycle after a is set.
            //-------------------------------------------------------------
            S_HDR: begin
                if (S_hdr_i < P_HDR_BYTES[4:0]) begin
                    O_rx_addr <= {6'd0, S_hdr_i};
                    // ★ At cycle S_hdr_i = k the registered read presents byte
                    //   (k-2): address k-1 was set during the previous cycle and
                    //   its data only appears now.  Using k-1 here shifted every
                    //   latched field by one byte (line number read as 1, mode as
                    //   0, and SOF read as 1 because it picked up the mode byte).
                    if (S_hdr_i == 5'd4) S_line[7:0]  <= I_rx_rdata;   // header byte 2
                    if (S_hdr_i == 5'd5) S_line[15:8] <= I_rx_rdata;   // header byte 3
                    if (S_hdr_i == 5'd6) S_mode       <= I_rx_rdata[4:0];  // byte 4
                    if (S_hdr_i == 5'd7) S_sof        <= I_rx_rdata[0];    // byte 5
                    if (S_hdr_i == 5'd10) S_frame_seq[7:0]   <= I_rx_rdata; // byte 8
                    if (S_hdr_i == 5'd11) S_frame_seq[15:8]  <= I_rx_rdata; // byte 9
                    if (S_hdr_i == 5'd12) S_frame_seq[23:16] <= I_rx_rdata; // byte 10
                    if (S_hdr_i == 5'd13) S_frame_seq[31:24] <= I_rx_rdata; // byte 11
                    S_hdr_i <= S_hdr_i + 5'd1;
                end
                else begin
                    O_rx_addr  <= P_HDR_BYTES[10:0];   // first payload byte
                    S_k        <= 11'd0;
                    O_fil_mode <= S_mode;
                    // A new frame sequence, SOF, or a line-number jump starts
                    // a fresh filter segment.  The current line is fed with
                    // vsync asserted so the row memories cannot mix frames.
                    S_force_vsync <= !S_seq_seen || S_sof ||
                                     (S_frame_seq != S_active_frame_seq) ||
                                     (S_seq_seen && !S_sof &&
                                      (S_line != (S_last_line + 16'd1)));
                    if (!S_seq_seen || S_sof ||
                        (S_frame_seq != S_active_frame_seq)) begin
                        S_segment_line <= 16'd0;
                        S_frame_gap <= 1'b0;
                        S_output_valid <= 1'b0;
                    end
                    else if (S_line != (S_last_line + 16'd1)) begin
                        S_segment_line <= 16'd0;
                        S_frame_gap <= 1'b1;
                        S_output_valid <= 1'b0;
                    end
                    else begin
                        S_segment_line <= S_segment_line + 16'd1;
                        // One contiguous line after a gap is enough to clear
                        // the diagnostic latch; the window-valid threshold
                        // below still prevents premature output.
                        S_frame_gap <= 1'b0;
                        S_output_valid <= ((S_mode == 5'd17) ?
                                           ((S_segment_line + 16'd1) >= 16'd3) :
                                           ((S_segment_line + 16'd1) >= 16'd2));
                    end
                    S_seq_seen <= 1'b1;
                    S_active_frame_seq <= S_frame_seq;
                    S_last_line <= S_line;
                    S_state    <= S_LINE;
                end
            end

            //-------------------------------------------------------------
            // ★ Phase bookkeeping (measured, not guessed).  With the registered
            //   read, at the cycle where S_k = j the input I_rx_rdata holds
            //   payload byte (j-1), and O_fil_y -- itself registered -- holds
            //   byte (j-2).  So the FIRST payload byte appears on the filter
            //   input at j = 2 and the LAST at j = IMG_W+1.  The first version
            //   ran j = 0 .. IMG_W-1 with de/user straight through, which lost
            //   the user pulse (de was still low on the first cycle) and dropped
            //   the last two bytes.  Assignment here is to the NEXT cycle, hence
            //   the shifted thresholds.
            S_LINE: begin
                O_fil_vsync <= S_force_vsync && (S_k == 11'd1);
                O_fil_hsync <= 1'b1;
                // NOTE these thresholds are set by THIS module's own registered
                // read: O_fil_y at cycle j holds payload byte (j-2).  They do
                // NOT depend on the filter's latency -- the filter marks its own
                // output with user/de, and the collector keys off those.  An
                // earlier "fix" shifted them when the filter gained a pipeline
                // stage and broke the bench: wrong premise.
                O_fil_de    <= (S_k >= 11'd1) && (S_k <= W_IMG_W);
                O_fil_user  <= (S_k == 11'd1);
                O_fil_last  <= (S_k == W_IMG_W);
                O_fil_y     <= I_rx_rdata;
                O_rx_addr   <= P_HDR_BYTES[10:0] + S_k + 11'd1;
                S_k         <= S_k + 11'd1;
                if (S_k == W_IMG_W + 11'd1) begin
                    S_tail_i <= 5'd0;
                    S_drain  <= 5'd0;
                    S_state  <= S_DRAIN;
                end
            end

            //-------------------------------------------------------------
            // ★ Let the filter finish emitting before the header is written.
            //   The filter's output lags its input by one line PLUS its pipeline
            //   depth (6 cycles after the registered-address stage was added), so
            //   the last outputs of the line arrive AFTER the last input pixel.
            //   Writing the header into the same single-write-port buffer during
            //   that window delayed and could overwrite payload bytes.  Wait for
            //   16 consecutive cycles with no filter output.
            //-------------------------------------------------------------
            S_DRAIN: begin
                if (I_fil_de) S_drain <= 5'd0;
                else if (S_drain == 5'd15) S_state <= S_TAIL;
                else S_drain <= S_drain + 5'd1;
            end

            //-------------------------------------------------------------
            // request all 16 header bytes, one per cycle, then commit.  The
            // first line of a frame is fed but not sent.
            //-------------------------------------------------------------
            S_TAIL: begin
                // The line that carries SOF is the first line of a video frame:
                // the filter has no row history for it, so its result is
                // meaningless and must not be transmitted.  Feeding it is what
                // primes the row memories.  Keying this off the SOF FLAG (rather
                // than a one-shot "first frame ever" register) also re-arms it
                // correctly at every new video frame.
                if (S_segment_line == 16'd0) begin
                    S_state <= S_REL;
                end
                else if (I_tx_ready) begin
                    if (!S_hdr_req) S_hdr_req <= 1'b1;
                    else if (S_hdr_ack) begin
                        S_hdr_req <= 1'b0;
                        if (S_tail_i == P_HDR_BYTES[4:0] - 5'd1) begin
                            S_tail_i <= 5'd0;
                            S_state  <= S_CMT;
                        end
                        else S_tail_i <= S_tail_i + 5'd1;
                    end
                end
            end

            //-------------------------------------------------------------
            // the commit must not share a cycle with a header byte write
            //-------------------------------------------------------------
            S_CMT: begin
                if (!S_hdr_ack) begin
                    O_tx_commit <= 1'b1;
                    S_state     <= S_REL;
                end
            end

            //-------------------------------------------------------------
            S_REL: begin
                O_rx_release <= 1'b1;
                S_state      <= S_WAIT;
            end

            //-------------------------------------------------------------
            // ★ Leave one complete application-clock interval after release.
            //   ctl_stream_link publishes RX buffers with a toggle; when the
            //   other ping-pong buffer is already held, O_rx_valid can remain
            //   high continuously across this release.  Waiting for the level
            //   to fall would then deadlock the adapter.  The one-cycle state
            //   boundary is enough to avoid re-reading the just-released
            //   buffer, while allowing a continuously-valid next buffer.
            //-------------------------------------------------------------
            S_WAIT: begin
                S_state <= S_IDLE;
            end

            default: S_state <= S_IDLE;
            endcase
        end
    end

endmodule
