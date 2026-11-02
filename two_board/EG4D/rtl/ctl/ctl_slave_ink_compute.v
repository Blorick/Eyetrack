`timescale 1ns/1ps
//=============================================================================
//  ctl_slave_ink_compute -- tagged tile-bank C13/C14 controller
//  所属域 : ctl
//  时钟域 : I_clk_app, EG 25 MHz compute clock
//  复位   : I_rst_app_n, asynchronous active low
//  Contract: mailbox inputs already synchronized; barrier drains old offers
//  Ownership: cancellation invalidates tag immediately, arithmetic drains
//=============================================================================
module ctl_slave_ink_compute (
    input wire I_clk_app,
    input wire I_rst_app_n,
    input wire I_barrier,
    input wire I_tile_valid,
    output wire O_tile_ready,
    input wire [40:0] I_tile_data,
    input wire I_snapshot_valid,
    output wire O_snapshot_ready,
    input wire [71:0] I_snapshot_data,
    output wire O_result_valid,
    input wire I_result_ready,
    output wire [188:0] O_result_data,
    output wire O_old_owner_invalid,
    output wire O_core_busy,
    output wire O_quiet,
    output wire O_feature_valid,
    output wire [7:0] O_feature_data,
    output wire [6:0] O_feature_index
);
    localparam [2:0] ST_IDLE=0, ST_RESET=1, ST_FRAME=2, ST_GLYPH=3,
        ST_GLYPH_WAIT=4, ST_FRAME_WAIT=5, ST_RESULT=6;
    reg [2:0] S_state, S_next_state;
    reg [31:0] S_epoch, S_frame_index, S_matcher_epoch;
    reg S_bank, S_owner_valid, S_matcher_epoch_valid, S_cancelled;
    reg [6:0] S_mask;
    reg [2:0] S_slot;
    reg [188:0] S_result_data;
    wire W_engine_busy, W_engine_done, W_feature_ready;
    wire W_glyph_done, W_frame_result, W_res_valid;
    wire [55:0] W_res, W_raw_code;
    wire [3:0] W_res_n;
    wire [6:0] W_raw_valid;
    wire W_idle = S_state==ST_IDLE && !W_engine_busy;
    wire W_tile_bank = I_tile_data[40:32]>=9'd224;
    wire W_tile_owned = S_state!=ST_IDLE && W_tile_bank==S_bank;
    wire W_snapshot_accept = I_snapshot_valid && O_snapshot_ready && !I_barrier;
    wire W_new_epoch = !S_matcher_epoch_valid || I_snapshot_data[71:40]!=S_matcher_epoch;
    wire W_glyph_valid = S_mask[S_slot];
    wire W_tile_write = I_tile_valid && O_tile_ready && !I_barrier && I_tile_data[40:32]<448;
    wire W_frame_done = S_state==ST_GLYPH_WAIT && S_slot==6 && W_glyph_done;
    wire W_matcher_rst_n = I_rst_app_n && S_state!=ST_RESET;
    assign O_tile_ready = I_barrier || !W_tile_owned;
    assign O_snapshot_ready = I_barrier || W_idle;
    assign O_result_valid = S_state==ST_RESULT && S_owner_valid && !I_barrier;
    assign O_result_data = S_result_data;
    assign O_old_owner_invalid = !S_owner_valid || I_barrier;
    assign O_core_busy = S_state!=ST_IDLE || W_engine_busy;
    assign O_quiet = W_idle && !I_tile_valid && !I_snapshot_valid;

    char_shape_feature_engine #(.TILE_WORDS(448)) U_feature (
        .clk(I_clk_app), .rst_n(I_rst_app_n),
        .i_tile_write_en(W_tile_write), .i_tile_write_addr(I_tile_data[40:32]),
        .i_tile_write_data(I_tile_data[31:0]),
        .i_process_start(S_state==ST_GLYPH), .i_process_slot(S_slot),
        .i_process_bank(S_bank), .i_process_valid(W_glyph_valid),
        .o_feature_valid(O_feature_valid), .o_feature_data(O_feature_data),
        .i_feature_ready(W_feature_ready), .o_engine_done(W_engine_done),
        .o_busy(W_engine_busy), .o_feature_index(O_feature_index)
    );
    char_match #(.STABLE_EN(1), .HOLD_K(5), .LOST_K(8)) U_match (
        .clk(I_clk_app), .rst_n(W_matcher_rst_n),
        .i_glyph_start(S_state==ST_GLYPH), .i_glyph_slot(S_slot),
        .i_glyph_valid(W_glyph_valid), .i_feature_valid(O_feature_valid),
        .i_feature_data(O_feature_data), .o_feature_ready(W_feature_ready),
        .o_result_valid(), .o_result_code(), .o_result_accepted(),
        .o_glyph_done(W_glyph_done), .o_result_slot(),
        .i_frame_start(S_state==ST_FRAME), .i_frame_done(W_frame_done),
        .o_res(W_res), .o_res_n(W_res_n), .o_res_valid(W_res_valid),
        .o_raw_valid(W_raw_valid), .o_frame_result(W_frame_result),
        .o_raw_candidate_code(W_raw_code), .o_raw_candidate_mask(), .o_match_state()
    );

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if(!I_rst_app_n) S_state<=ST_IDLE;
        else S_state<=S_next_state;
    end
    always @(*) begin
        S_next_state=S_state;
        case(S_state)
            ST_IDLE: if(W_snapshot_accept) begin
                if(W_new_epoch) S_next_state=ST_RESET;
                else S_next_state=ST_FRAME;
            end
            ST_RESET:S_next_state=ST_FRAME;
            ST_FRAME:S_next_state=ST_GLYPH;
            ST_GLYPH:S_next_state=ST_GLYPH_WAIT;
            ST_GLYPH_WAIT:if(W_glyph_done) begin
                if(S_slot==6)S_next_state=ST_FRAME_WAIT;
                else S_next_state=ST_GLYPH;
            end
            ST_FRAME_WAIT:if(W_frame_result) begin
                if(S_cancelled || I_barrier)S_next_state=ST_IDLE;
                else S_next_state=ST_RESULT;
            end
            ST_RESULT:if(I_result_ready || I_barrier || S_cancelled)S_next_state=ST_IDLE;
            default:S_next_state=ST_IDLE;
        endcase
    end
    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if(!I_rst_app_n) begin
            S_epoch<=0;S_frame_index<=0;S_matcher_epoch<=0;
            S_bank<=0;S_owner_valid<=0;S_matcher_epoch_valid<=0;S_cancelled<=0;
            S_mask<=0;S_slot<=0;S_result_data<=0;
        end else begin
            // Do not reset an active arithmetic core or change its bank/tag.
            // Finish its old job and suppress publication after cancellation.
            if(I_barrier) begin
                S_owner_valid<=0;S_cancelled<=1;S_matcher_epoch_valid<=0;
            end
            if(S_state==ST_IDLE) begin
                S_owner_valid<=0;
                if(W_snapshot_accept) begin
                    S_epoch<=I_snapshot_data[71:40];S_frame_index<=I_snapshot_data[39:8];
                    S_mask<=I_snapshot_data[7:1];S_bank<=I_snapshot_data[0];S_slot<=0;
                    S_owner_valid<=1;S_cancelled<=0;
                    S_matcher_epoch<=I_snapshot_data[71:40];S_matcher_epoch_valid<=1;
                end
            end
            if(S_state==ST_GLYPH_WAIT && W_glyph_done && S_slot!=6)S_slot<=S_slot+1;
            if(S_state==ST_FRAME_WAIT && W_frame_result) begin
                S_result_data<={S_epoch,S_frame_index,W_res,W_res_n,W_res_valid,1'b1,W_raw_code,W_raw_valid};
                if(S_cancelled || I_barrier)S_owner_valid<=0;
            end
            if(S_state==ST_RESULT && (I_result_ready || I_barrier))S_owner_valid<=0;
        end
    end
endmodule
