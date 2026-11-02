`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// Module: ctl_gmii_tx_scheduler
// Function: arbitrate the PH1P control packet and pixel-stream GMII TX clients
// Domain: one shared 125 MHz GMII TX clock
// Reset: active-low, asynchronous assertion
// Policy: P_TX_SCHEDULE=0 gives client 0 fixed priority; non-zero is round robin
// Contract: a grant is a one-cycle start permission; the owner is held until done
// -----------------------------------------------------------------------------
module ctl_gmii_tx_scheduler #(
    // Client 0 is the 0x88B5 OCR/control link.  Client 1 is the 0x88B6 stream.
    parameter integer P_TX_SCHEDULE = 0
) (
    input  wire       I_clk_tx,
    input  wire       I_rst_tx_n,

    // Each endpoint keeps its request asserted while a frame is pending or
    // active.  O_tx*_done is a one-cycle pulse after that endpoint's IFG.
    input  wire       I_tx0_request,
    input  wire       I_tx1_request,
    input  wire       I_tx0_done,
    input  wire       I_tx1_done,
    output reg        O_tx0_grant,
    output reg        O_tx1_grant,

    // TX data from the two endpoints.  The scheduler owns the shared GMII
    // pins, so there is no combinational OR of the two clients on the board.
    input  wire [7:0] I_tx0_txd,
    input  wire       I_tx0_tx_en,
    input  wire       I_tx0_tx_er,
    input  wire [7:0] I_tx1_txd,
    input  wire       I_tx1_tx_en,
    input  wire       I_tx1_tx_er,
    output reg  [7:0] O_txd,
    output reg        O_tx_en,
    output reg        O_tx_er,

    output wire       O_owner_valid,
    output wire       O_owner
);

    reg S_owner_valid;
    reg S_owner;
    reg S_rr_next;
    // If an endpoint is reset while it owns GMII, its request and done pulses
    // both disappear.  Keep a bounded abort interframe gap so the dead owner
    // cannot block the other client forever and no new frame starts before the
    // required Ethernet idle interval has elapsed.
    reg [3:0] S_abort_ifg;
    // A normal endpoint may deassert request on the same edge that its done
    // pulse becomes visible.  Hold that ambiguous combination for one clock:
    // a persistent done level is the mode-PLL abort indication used by the
    // top-level, while a one-cycle done pulse is normal completion.
    reg       S_done_wait;

    wire W_pick0 = I_tx0_request &&
                   ((!I_tx1_request) || (P_TX_SCHEDULE == 0) || !S_rr_next);
    wire W_pick1 = I_tx1_request &&
                   ((!I_tx0_request) || (P_TX_SCHEDULE != 0 && S_rr_next));
    wire W_owner_request = (S_owner == 1'b0) ? I_tx0_request : I_tx1_request;
    wire W_owner_done    = (S_owner == 1'b0) ? I_tx0_done    : I_tx1_done;

    assign O_owner_valid = S_owner_valid;
    assign O_owner       = S_owner;

    // Directly mux only the locked owner.  During the one-cycle grant pulse
    // the selected endpoint is still idle; its preamble begins on the next
    // edge after it samples the grant.
    always @(*) begin
        O_txd   = 8'h00;
        O_tx_en = 1'b0;
        O_tx_er = 1'b0;
        if (S_owner_valid) begin
            if (S_owner == 1'b0) begin
                O_txd   = I_tx0_txd;
                O_tx_en = I_tx0_tx_en;
                O_tx_er = I_tx0_tx_er;
            end
            else begin
                O_txd   = I_tx1_txd;
                O_tx_en = I_tx1_tx_en;
                O_tx_er = I_tx1_tx_er;
            end
        end
    end

    always @(posedge I_clk_tx or negedge I_rst_tx_n) begin
        if (!I_rst_tx_n) begin
            S_owner_valid <= 1'b0;
            S_owner       <= 1'b0;
            S_rr_next     <= 1'b0;
            S_abort_ifg   <= 4'd0;
            S_done_wait   <= 1'b0;
            O_tx0_grant   <= 1'b0;
            O_tx1_grant   <= 1'b0;
        end
        else begin
            // Grants are start pulses, never a second data-path enable.  The
            // owner lock below protects the whole preamble/frame/FCS/IFG.
            O_tx0_grant <= 1'b0;
            O_tx1_grant <= 1'b0;

            if (S_owner_valid) begin
                // Endpoint completion and request withdrawal can be observed
                // together.  Give a done pulse one settling cycle before
                // declaring it a reset abort; the mode reset indication stays
                // high and therefore takes the abort path on the next clock.
                if (S_done_wait) begin
                    S_done_wait <= 1'b0;
                    if (W_owner_done && !W_owner_request) begin
                        S_owner_valid <= 1'b0;
                        S_rr_next     <= ~S_owner;
                        S_abort_ifg   <= 4'd12;
                    end
                    else if (!W_owner_request || W_owner_done) begin
                        S_owner_valid <= 1'b0;
                        S_rr_next     <= ~S_owner;
                        S_abort_ifg   <= 4'd0;
                    end
                end
                else if (W_owner_done && !W_owner_request) begin
                    S_done_wait <= 1'b1;
                end
                else if (!W_owner_request) begin
                    S_owner_valid <= 1'b0;
                    S_rr_next     <= ~S_owner;
                    S_abort_ifg   <= 4'd12;
                end
                else if ((S_owner == 1'b0) && I_tx0_done) begin
                    S_owner_valid <= 1'b0;
                    S_rr_next     <= 1'b1;
                    S_abort_ifg   <= 4'd0;
                end
                else if ((S_owner == 1'b1) && I_tx1_done) begin
                    S_owner_valid <= 1'b0;
                    S_rr_next     <= 1'b0;
                    S_abort_ifg   <= 4'd0;
                end
            end
            else if (S_abort_ifg != 4'd0) begin
                S_abort_ifg <= S_abort_ifg - 4'd1;
            end
            else if (W_pick0) begin
                S_owner_valid <= 1'b1;
                S_owner       <= 1'b0;
                S_done_wait   <= 1'b0;
                O_tx0_grant   <= 1'b1;
            end
            else if (W_pick1) begin
                S_owner_valid <= 1'b1;
                S_owner       <= 1'b1;
                S_done_wait   <= 1'b0;
                O_tx1_grant   <= 1'b1;
            end
        end
    end

endmodule
