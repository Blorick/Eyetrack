//=============================================================================
//  ctl_display_control_snapshot -- coherent display settings at frame boundaries
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk_24m, I_clk_pix
//  复位     : I_rst_n，异步低有效，各域同步释放由顶层保证
//  参数     : 无
//  验证状态 : 见 docs/validation.md
//=============================================================================
module ctl_display_control_snapshot (
    input wire I_clk_24m,
    input wire I_clk_pix,
    input wire I_rst_n,
    input wire [4:0] I_mode,
    input wire signed [8:0] I_brightness,
    input wire [7:0] I_sat_factor,
    input wire I_frame_pix,
    output reg [4:0] O_mode,
    output reg signed [8:0] O_brightness,
    output reg [7:0] O_sat_factor
);
    // The source holds the complete tuple until the destination acknowledges it.
    reg [21:0] S_hold_24m;
    reg S_request_24m;
    reg S_ack_meta_24m;
    reg S_ack_sync_24m;
    reg S_request_meta_pix;
    reg S_request_sync_pix;
    reg S_ack_pix;
    reg [21:0] S_pending_pix;
    reg S_pending_valid_pix;
    wire [21:0] W_requested_24m;
    wire W_source_idle_24m;
    wire W_source_changed_24m;
    wire W_destination_new_pix;
    reg [1:0] S_release_24m;
    reg [1:0] S_release_pix;
    wire W_rst_24m_n;
    wire W_rst_pix_n;

    assign W_rst_24m_n = S_release_24m[1];
    assign W_rst_pix_n = S_release_pix[1];

    always @(posedge I_clk_24m or negedge I_rst_n) begin
        if (!I_rst_n) S_release_24m <= 2'b00;
        else S_release_24m <= {S_release_24m[0], 1'b1};
    end

    always @(posedge I_clk_pix or negedge I_rst_n) begin
        if (!I_rst_n) S_release_pix <= 2'b00;
        else S_release_pix <= {S_release_pix[0], 1'b1};
    end

    assign W_requested_24m = {I_mode, I_brightness, I_sat_factor};
    assign W_source_idle_24m = (S_request_24m == S_ack_sync_24m);
    assign W_source_changed_24m = (W_requested_24m != S_hold_24m);
    assign W_destination_new_pix = (S_request_sync_pix != S_ack_pix);

    always @(posedge I_clk_24m or negedge W_rst_24m_n) begin
        if (!W_rst_24m_n) begin
            S_hold_24m <= {5'd0, 9'd64, 8'd128};
            S_request_24m <= 1'b0;
            S_ack_meta_24m <= 1'b0;
            S_ack_sync_24m <= 1'b0;
        end else begin
            S_ack_meta_24m <= S_ack_pix;
            S_ack_sync_24m <= S_ack_meta_24m;
            if (W_source_idle_24m && W_source_changed_24m) begin
                S_hold_24m <= W_requested_24m;
                S_request_24m <= ~S_request_24m;
            end
        end
    end

    // A pending tuple may be replaced by a newer acknowledged tuple before SOF.
    // The visible tuple changes together on the next frame boundary only.
    always @(posedge I_clk_pix or negedge W_rst_pix_n) begin
        if (!W_rst_pix_n) begin
            S_request_meta_pix <= 1'b0;
            S_request_sync_pix <= 1'b0;
            S_ack_pix <= 1'b0;
            S_pending_pix <= {5'd0, 9'd64, 8'd128};
            S_pending_valid_pix <= 1'b0;
            O_mode <= 5'd0;
            O_brightness <= 9'sd64;
            O_sat_factor <= 8'd128;
        end else begin
            S_request_meta_pix <= S_request_24m;
            S_request_sync_pix <= S_request_meta_pix;
            if (I_frame_pix && S_pending_valid_pix) begin
                {O_mode, O_brightness, O_sat_factor} <= S_pending_pix;
                S_pending_valid_pix <= 1'b0;
            end
            if (W_destination_new_pix) begin
                S_pending_pix <= S_hold_24m;
                S_pending_valid_pix <= 1'b1;
                S_ack_pix <= S_request_sync_pix;
            end
        end
    end
endmodule
