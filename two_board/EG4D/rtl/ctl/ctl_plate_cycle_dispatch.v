`timescale 1ns / 1ps

//=============================================================================
//  ctl_plate_cycle_dispatch  --  周期检测自动语音/OSD派发器
//-----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : I_clk（25 MHz EG4D 应用域）
//  复位     : I_rst_n（异步断言）
//  参数     : P_CLK_HZ
//  验证状态 : semantic model PASS；ModelSim/TD/hardware not run in this task
//=============================================================================

////////////////////////////////////English///////////////////////////////////////
// Company:         Anlogic contest team
// Engineer:        cycle_dispatch_rtl
//
// Create Date:     2026/10/25 00:00:00
// Design Name:     plate_cycle_dispatch
// Module Name:     ctl_plate_cycle_dispatch
// Description:     ctl_plate_cycle_dispatch_spec.md
// Simulations:     verification/plate_cycle_20261025/simulation
//
// Referrences:     None
//
// Dependencies:    None
//
// Version:         V1.0
// Revision Date:   2026/10/25 00:00:00
// History:
// Time             Version     Revised by        Contents
// 2026/10/25       V1.0        cycle_dispatch_rtl Create periodic FIFO dispatcher.
///////////////////////////////////Chinese////////////////////////////////////////
// 版权归属:        安路科技竞赛项目
// 开发人员:        cycle_dispatch_rtl
//
// 创建日期:        2026年10月25日
// 设计名称:        plate_cycle_dispatch
// 模块名称:        ctl_plate_cycle_dispatch
// 模块说明:        ctl_plate_cycle_dispatch_spec.md
// 仿真工程:        verification/plate_cycle_20261025/simulation
//
// 参考资料:        None
//
// 依赖文件:        None
//
// 当前版本:        V1.0
// 修订日期:        2026年10月25日
// 修订历史:
// 时间             版本        修订人            修订内容
// 2026年10月25日   V1.0        cycle_dispatch_rtl 创建周期FIFO派发器

module ctl_plate_cycle_dispatch #(
    parameter integer P_CLK_HZ = 25000000
) (
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire        I_update,
    input  wire        I_cycle_active,
    input  wire        I_cycle_reset_ack,
    input  wire        I_metrics_valid,
    input  wire [19:0] I_speed_mmps,
    input  wire [19:0] I_dist_mm,
    input  wire        I_reset_cycle,
    input  wire        I_audio_busy,
    input  wire        I_audio_sample,
    output wire        O_reset_request,
    output wire        O_cycle_active,
    output wire [3:0]  O_cycle_index,
    output wire        O_auto_busy,
    output reg         O_play_req,
    output reg  [4:0]  O_track,
    output reg  [4:0]  O_osd_key,
    output reg         O_queue_overflow
);

    localparam integer P_OBSERVE_CYCLES = P_CLK_HZ + (P_CLK_HZ / 2);
    localparam integer P_WARNING_CYCLES = P_CLK_HZ * 2;
    localparam integer P_OBS_W = (P_OBSERVE_CYCLES < 2) ?
                                 1 : $clog2(P_OBSERVE_CYCLES + 1);
    localparam integer P_WARN_W = (P_WARNING_CYCLES < 2) ?
                                  1 : $clog2(P_WARNING_CYCLES + 1);

    localparam [2:0] ST_IDLE        = 3'd0;
    localparam [2:0] ST_WARN_SAMPLE = 3'd1;
    localparam [2:0] ST_WARN_SLOT   = 3'd2;
    localparam [2:0] ST_PLATE_START = 3'd3;
    localparam [2:0] ST_PLATE_DONE  = 3'd4;

    reg        S_reset_request_q;
    reg        S_link_armed_q;

    reg [7:0]  S_queue_mem [0:63];
    reg [5:0]  S_head_ptr_q;
    reg [5:0]  S_tail_ptr_q;
    reg [6:0]  S_queue_count_q;

    reg        S_active_seen_q;
    reg        S_active_accepted_q;
    reg [5:0]  S_active_slot_q;
    reg [3:0]  S_active_cycle_id_q;
    reg [3:0]  S_next_cycle_q;

    reg        S_collect_valid_q;
    reg [5:0]  S_collect_slot_q;
    reg [7:0]  S_collect_record_q;
    reg [P_OBS_W-1:0] S_collect_elapsed_q;

    reg        S_head_speed_sent_q;
    reg        S_head_dist_sent_q;
    reg [2:0]  S_state_q;
    reg [P_WARN_W-1:0] S_warning_timer_q;

    wire [7:0] W_head_record = S_queue_mem[S_head_ptr_q];
    wire       W_head_closed = W_head_record[3];
    wire       W_head_speed  = W_head_record[2];
    wire       W_head_dist   = W_head_record[1];
    wire       W_head_first_dist = W_head_record[0];
    wire [4:0] W_head_plate_track = {1'b0, W_head_record[7:4]} + 5'd4;

    wire W_input_speed_hit = I_metrics_valid && (I_speed_mmps > 20'd300);
    wire W_input_dist_hit  = I_metrics_valid && (I_dist_mm < 20'd200);
    wire W_initial_first_dist = W_input_dist_hit && !W_input_speed_hit;
    wire [7:0] W_initial_record = {
        S_next_cycle_q,
        1'b0,
        W_input_speed_hit,
        W_input_dist_hit,
        W_initial_first_dist
    };

    wire W_metric_speed_hit = W_input_speed_hit && !S_collect_record_q[2];
    wire W_metric_dist_hit  = W_input_dist_hit && !S_collect_record_q[1];
    wire W_metric_first_dist = S_collect_record_q[0] |
                               (W_metric_dist_hit &&
                                !S_collect_record_q[2] &&
                                !S_collect_record_q[1] &&
                                !W_metric_speed_hit);
    wire [7:0] W_metric_record = {
        S_collect_record_q[7:4],
        S_collect_record_q[3],
        S_collect_record_q[2] | W_metric_speed_hit,
        S_collect_record_q[1] | W_metric_dist_hit,
        W_metric_first_dist
    };
    wire [7:0] W_closed_record = {
        S_collect_record_q[7:4],
        1'b1,
        S_collect_record_q[2:0]
    };

    wire W_epoch_ok = I_update &&
                       (I_cycle_reset_ack == S_reset_request_q);
    wire W_active_rise = W_epoch_ok && I_cycle_active && !S_active_seen_q;
    wire W_active_fall = W_epoch_ok && !I_cycle_active && S_active_seen_q;
    wire W_enqueue = W_active_rise && (S_queue_count_q < 7'd64);

    // A full queue deliberately does not admit an enqueue even when a plate
    // completes on the same edge.  This makes the no-overwrite guarantee
    // explicit and exposes the condition through O_queue_overflow.
    wire W_pop = S_link_armed_q &&
                 (S_queue_count_q != 7'd0) &&
                 (S_state_q == ST_PLATE_DONE) && !I_audio_busy;
    wire W_collect_timer_enable = S_collect_valid_q && S_active_seen_q &&
                                  !W_active_rise && !W_active_fall;
    wire W_dispatch_enable = S_link_armed_q &&
                             (S_queue_count_q != 7'd0);
    wire W_current_head_active = S_active_accepted_q &&
                                 (S_active_slot_q == S_head_ptr_q) &&
                                 !W_active_fall;

    assign O_reset_request = S_reset_request_q;
    assign O_cycle_active = S_active_accepted_q;
    // Retain the latest allocated ID after active falls; KEY3/global reset
    // explicitly returns it to zero for the next epoch.
    assign O_cycle_index  = S_active_cycle_id_q;
    // Waiting for reset acknowledgement fences stale PH input but does not
    // suppress physical keys while the automatic queue is empty.
    assign O_auto_busy = (S_queue_count_q != 7'd0);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_reset_request_q   <= 1'b0;
            S_link_armed_q      <= 1'b1;
            S_head_ptr_q        <= 6'd0;
            S_tail_ptr_q        <= 6'd0;
            S_queue_count_q     <= 7'd0;
            S_active_seen_q     <= 1'b0;
            S_active_accepted_q <= 1'b0;
            S_active_slot_q     <= 6'd0;
            S_active_cycle_id_q <= 4'd0;
            S_next_cycle_q      <= 4'd1;
            S_collect_valid_q   <= 1'b0;
            S_collect_slot_q    <= 6'd0;
            S_collect_record_q  <= 8'd0;
            S_collect_elapsed_q <= {P_OBS_W{1'b0}};
            S_head_speed_sent_q <= 1'b0;
            S_head_dist_sent_q  <= 1'b0;
            S_state_q           <= ST_IDLE;
            S_warning_timer_q   <= {P_WARN_W{1'b0}};
            O_play_req          <= 1'b0;
            O_track             <= 5'd0;
            O_osd_key           <= 5'd0;
            O_queue_overflow    <= 1'b0;
        end else begin
            // O_play_req is a one-clock event; O_track is retained for
            // downstream trigger capture and diagnostic visibility.
            O_play_req <= 1'b0;

            if (I_reset_cycle) begin
                // KEY3 is a local epoch reset.  The physical audio path has
                // no abort port in this API, so an already-running sample
                // stream may finish outside this scheduler.
                S_reset_request_q   <= ~S_reset_request_q;
                S_link_armed_q      <= 1'b0;
                S_head_ptr_q        <= 6'd0;
                S_tail_ptr_q        <= 6'd0;
                S_queue_count_q     <= 7'd0;
                S_active_seen_q     <= 1'b0;
                S_active_accepted_q <= 1'b0;
                S_active_slot_q     <= 6'd0;
                S_active_cycle_id_q <= 4'd0;
                S_next_cycle_q      <= 4'd1;
                S_collect_valid_q   <= 1'b0;
                S_collect_slot_q    <= 6'd0;
                S_collect_record_q  <= 8'd0;
                S_collect_elapsed_q <= {P_OBS_W{1'b0}};
                S_head_speed_sent_q <= 1'b0;
                S_head_dist_sent_q  <= 1'b0;
                S_state_q           <= ST_IDLE;
                S_warning_timer_q   <= {P_WARN_W{1'b0}};
                O_play_req          <= 1'b0;
                O_track             <= 5'd0;
                O_osd_key           <= 5'd0;
                O_queue_overflow    <= 1'b0;
            end else begin
                // The acknowledgement is sampled only on an update packet.
                // A stale active packet therefore cannot create a cycle in a
                // new epoch, while a matching active packet may start cycle 1
                // immediately.
                // A mismatched ACK leaves existing collection, its window
                // clock, OSD and queued audio untouched; only W_epoch_ok
                // enters packet-driven active/metric handling below.
                if (W_epoch_ok) begin
                    S_link_armed_q <= 1'b1;

                    if (W_active_fall) begin
                        S_active_seen_q     <= 1'b0;
                        S_active_accepted_q <= 1'b0;
                        O_osd_key            <= 5'd0;
                        if (S_collect_valid_q) begin
                            S_collect_record_q <= W_closed_record;
                            S_queue_mem[S_collect_slot_q] <= W_closed_record;
                            S_collect_valid_q <= 1'b0;
                        end
                    end else if (W_active_rise) begin
                        S_active_seen_q <= 1'b1;
                        // A new PH cycle clears any OSD from the preceding
                        // cycle, including a queued old-cycle audio item.
                        O_osd_key <= 5'd0;
                        if (W_enqueue) begin
                            S_active_accepted_q <= 1'b1;
                            S_active_slot_q     <= S_tail_ptr_q;
                            S_active_cycle_id_q <= S_next_cycle_q;
                            S_queue_mem[S_tail_ptr_q] <= W_initial_record;
                            S_tail_ptr_q        <= S_tail_ptr_q + 1'b1;
                            S_collect_valid_q   <= 1'b1;
                            S_collect_slot_q    <= S_tail_ptr_q;
                            S_collect_record_q  <= W_initial_record;
                            S_collect_elapsed_q <= {P_OBS_W{1'b0}};
                            if (S_next_cycle_q == 4'd12) begin
                                S_next_cycle_q <= 4'd1;
                            end else begin
                                S_next_cycle_q <= S_next_cycle_q + 4'd1;
                            end
                        end else begin
                            // Do not allocate a partial record when all 64
                            // unfinished cycle slots are occupied.
                            S_active_accepted_q <= 1'b0;
                            S_collect_valid_q   <= 1'b0;
                            O_queue_overflow    <= 1'b1;
                        end
                    end else begin
                        S_active_seen_q <= I_cycle_active;
                    end
                end

                // The collector's clock window is independent of telemetry
                // update cadence.  At elapsed==P_OBSERVE_CYCLES-1 this edge
                // is exactly the 1.5 s boundary, so its sample is excluded.
                // A fall/rise/epoch event has priority over this timer.
                if (W_collect_timer_enable) begin
                    if (S_collect_elapsed_q < P_OBSERVE_CYCLES - 1) begin
                        S_collect_elapsed_q <= S_collect_elapsed_q + 1'b1;
                        if (W_epoch_ok && I_cycle_active && I_metrics_valid) begin
                            S_collect_record_q <= W_metric_record;
                            S_queue_mem[S_collect_slot_q] <= W_metric_record;
                        end
                    end else begin
                        S_collect_record_q <= W_closed_record;
                        S_queue_mem[S_collect_slot_q] <= W_closed_record;
                        S_collect_valid_q <= 1'b0;
                    end
                end

                // FIFO dispatcher.  A warning starts as soon as its event is
                // observed and the player is idle; its two-second slot starts
                // only on the first valid PCM sample.
                if (W_dispatch_enable) begin
                    case (S_state_q)
                        ST_IDLE: begin
                            if (!I_audio_busy) begin
                                if (W_head_first_dist) begin
                                    if (W_head_dist && !S_head_dist_sent_q) begin
                                        O_play_req        <= 1'b1;
                                        O_track           <= 5'd2;
                                        S_head_dist_sent_q <= 1'b1;
                                        S_state_q          <= ST_WARN_SAMPLE;
                                    end else if (W_head_speed && !S_head_speed_sent_q) begin
                                        O_play_req        <= 1'b1;
                                        O_track           <= 5'd1;
                                        S_head_speed_sent_q <= 1'b1;
                                        S_state_q          <= ST_WARN_SAMPLE;
                                    end else if (W_head_closed) begin
                                        O_play_req <= 1'b1;
                                        O_track    <= W_head_plate_track;
                                        if (W_current_head_active) begin
                                            O_osd_key <= W_head_plate_track;
                                        end
                                        S_state_q <= ST_PLATE_START;
                                    end
                                end else if (W_head_speed && !S_head_speed_sent_q) begin
                                    O_play_req         <= 1'b1;
                                    O_track            <= 5'd1;
                                    S_head_speed_sent_q <= 1'b1;
                                    S_state_q           <= ST_WARN_SAMPLE;
                                end else if (W_head_dist && !S_head_dist_sent_q) begin
                                    O_play_req        <= 1'b1;
                                    O_track           <= 5'd2;
                                    S_head_dist_sent_q <= 1'b1;
                                    S_state_q          <= ST_WARN_SAMPLE;
                                end else if (W_head_closed) begin
                                    O_play_req <= 1'b1;
                                    O_track    <= W_head_plate_track;
                                    if (W_current_head_active) begin
                                        O_osd_key <= W_head_plate_track;
                                    end
                                    S_state_q <= ST_PLATE_START;
                                end
                            end
                        end

                        ST_WARN_SAMPLE: begin
                            if (I_audio_sample) begin
                                S_warning_timer_q <= P_WARNING_CYCLES - 1;
                                S_state_q <= ST_WARN_SLOT;
                            end
                        end

                        ST_WARN_SLOT: begin
                            if (S_warning_timer_q != {P_WARN_W{1'b0}}) begin
                                S_warning_timer_q <= S_warning_timer_q - 1'b1;
                            end else if (!I_audio_busy) begin
                                S_state_q <= ST_IDLE;
                            end
                        end

                        ST_PLATE_START: begin
                            if (I_audio_busy) begin
                                S_state_q <= ST_PLATE_DONE;
                            end
                        end

                        ST_PLATE_DONE: begin
                            if (!I_audio_busy) begin
                                S_head_ptr_q        <= S_head_ptr_q + 1'b1;
                                S_head_speed_sent_q <= 1'b0;
                                S_head_dist_sent_q  <= 1'b0;
                                S_state_q            <= ST_IDLE;
                            end
                        end

                        default: begin
                            S_state_q <= ST_IDLE;
                        end
                    endcase
                end

                // Count is updated once, after both producer and consumer
                // logic, so an overlapping enqueue/pop preserves FIFO depth.
                case ({W_enqueue, W_pop})
                    2'b10: S_queue_count_q <= S_queue_count_q + 1'b1;
                    2'b01: S_queue_count_q <= S_queue_count_q - 1'b1;
                    default: begin end
                endcase

                if (W_pop && (S_queue_count_q == 7'd1)) begin
                    // Keep the pointers deterministic when the queue drains;
                    // tail remains the next free slot for a concurrent/new
                    // enqueue and the count remains the source of truth.
                    S_head_ptr_q <= S_head_ptr_q + 1'b1;
                end
            end
        end
    end

endmodule
