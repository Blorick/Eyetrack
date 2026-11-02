//=============================================================================
//  dsp_telemetry_seg -- MLK-S200 74HC595 telemetry display
//-----------------------------------------------------------------------------
//  所属域 : dsp
//  时钟域 : I_clk_25m（EG4D 应用域）
//  复位   : I_rst_n，异步低有效
//  功能   : 左四位显示 DIS(mm)，右四位显示 SPD(mm/s)
//  约束   : 输入大于 9999 时饱和为 9999；无效快照显示横杠
//  硬件   : 两片串联 74HC595，串行顺序为位选字节、段选字节，MSB first
//=============================================================================
//  MLK-S200 的数码管是共阳极结构。段码低有效，位选高有效。FPGA 只驱动
//  595 的串行接口：一帧先移入位选字节，再移入段选字节，锁存后点亮一位。
//  串行移位期间保持锁存器输出不变；每一位先锁存全灭帧，等待一个移位
//  时钟周期，再锁存目标位，避免 74HC595 输出改变时的重影。
//=============================================================================
`timescale 1ns / 1ps

module dsp_telemetry_seg #(
    parameter integer P_SHIFT_DIV = 25,
    parameter integer P_SLOT_DIV  = 25000
) (
    input  wire        I_clk_25m,
    input  wire        I_rst_n,
    input  wire        I_update,
    input  wire        I_metrics_valid,
    input  wire [19:0] I_dist_mm,
    input  wire [19:0] I_speed_mmps,
    output reg         O_seg_sclk,
    output reg         O_seg_latch,
    output wire        O_seg_data
);

    localparam [2:0] ST_IDLE         = 3'd0;
    localparam [2:0] ST_BLANK_SHIFT  = 3'd1;
    localparam [2:0] ST_BLANK_LATCH  = 3'd2;
    localparam [2:0] ST_BLANK_GAP    = 3'd3;
    localparam [2:0] ST_ACTIVE_SHIFT = 3'd4;
    localparam [2:0] ST_ACTIVE_LATCH = 3'd5;
    localparam integer P_TRANSFER_CYCLES = 67 * P_SHIFT_DIV;
    localparam integer P_IDLE_DIV = P_SLOT_DIV - P_TRANSFER_CYCLES;

    reg [2:0]  S_state;
    reg [15:0] S_shift_data;
    reg [4:0]  S_shift_count;
    reg        S_shift_high;
    reg [15:0] S_shift_tick;
    reg [15:0] S_latch_tick;
    reg [15:0] S_slot_tick;
    reg [2:0]  S_digit_index;
    reg [15:0] S_active_frame;

    reg [13:0] S_dist_binary;
    reg [13:0] S_speed_binary;
    reg [15:0] S_dist_work_bcd;
    reg [15:0] S_speed_work_bcd;
    reg [3:0]  S_convert_count;
    reg        S_convert_active;
    reg        S_convert_valid;
    reg [15:0] S_pending_dist_bcd;
    reg [15:0] S_pending_speed_bcd;
    reg        S_pending_valid;
    reg        S_pending_toggle;

    reg [15:0] S_display_dist_bcd;
    reg [15:0] S_display_speed_bcd;
    reg        S_display_valid;
    reg        S_display_toggle;

    assign O_seg_data = S_shift_data[15];

    function [15:0] f_adjust_bcd;
        input [15:0] I_bcd;
        reg [3:0] W_d3;
        reg [3:0] W_d2;
        reg [3:0] W_d1;
        reg [3:0] W_d0;
        begin
            W_d3 = (I_bcd[15:12] >= 4'd5) ? I_bcd[15:12] + 4'd3 : I_bcd[15:12];
            W_d2 = (I_bcd[11:8]  >= 4'd5) ? I_bcd[11:8]  + 4'd3 : I_bcd[11:8];
            W_d1 = (I_bcd[7:4]   >= 4'd5) ? I_bcd[7:4]   + 4'd3 : I_bcd[7:4];
            W_d0 = (I_bcd[3:0]   >= 4'd5) ? I_bcd[3:0]   + 4'd3 : I_bcd[3:0];
            f_adjust_bcd = {W_d3, W_d2, W_d1, W_d0};
        end
    endfunction

    function [7:0] f_seg_active_low;
        input [3:0] I_digit;
        begin
            case (I_digit)
                4'd0: f_seg_active_low = 8'hc0;
                4'd1: f_seg_active_low = 8'hf9;
                4'd2: f_seg_active_low = 8'ha4;
                4'd3: f_seg_active_low = 8'hb0;
                4'd4: f_seg_active_low = 8'h99;
                4'd5: f_seg_active_low = 8'h92;
                4'd6: f_seg_active_low = 8'h82;
                4'd7: f_seg_active_low = 8'hf8;
                4'd8: f_seg_active_low = 8'h80;
                4'd9: f_seg_active_low = 8'h90;
                4'd10: f_seg_active_low = 8'hbf;
                default: f_seg_active_low = 8'hff;
            endcase
        end
    endfunction

    function [15:0] f_frame_for_digit;
        input [2:0]  I_index;
        input [15:0] I_dist_bcd;
        input [15:0] I_speed_bcd;
        input        I_valid;
        reg [3:0] W_digit;
        reg [7:0] W_digit_sel;
        begin
            case (I_index)
                3'd0: W_digit = I_dist_bcd[15:12];
                3'd1: W_digit = I_dist_bcd[11:8];
                3'd2: W_digit = I_dist_bcd[7:4];
                3'd3: W_digit = I_dist_bcd[3:0];
                3'd4: W_digit = I_speed_bcd[15:12];
                3'd5: W_digit = I_speed_bcd[11:8];
                3'd6: W_digit = I_speed_bcd[7:4];
                default: W_digit = I_speed_bcd[3:0];
            endcase
            case (I_index)
                3'd0: W_digit_sel = 8'h01;
                3'd1: W_digit_sel = 8'h02;
                3'd2: W_digit_sel = 8'h04;
                3'd3: W_digit_sel = 8'h08;
                3'd4: W_digit_sel = 8'h10;
                3'd5: W_digit_sel = 8'h20;
                3'd6: W_digit_sel = 8'h40;
                default: W_digit_sel = 8'h80;
            endcase
            f_frame_for_digit = {
                W_digit_sel,
                I_valid ? f_seg_active_low(W_digit) : 8'hbf
            };
        end
    endfunction

    wire [15:0] W_dist_adjusted = f_adjust_bcd(S_dist_work_bcd);
    wire [15:0] W_speed_adjusted = f_adjust_bcd(S_speed_work_bcd);
    wire [15:0] W_dist_next_bcd = {W_dist_adjusted[14:0], S_dist_binary[13]};
    wire [15:0] W_speed_next_bcd = {W_speed_adjusted[14:0], S_speed_binary[13]};
    wire [13:0] W_dist_next_binary = {S_dist_binary[12:0], 1'b0};
    wire [13:0] W_speed_next_binary = {S_speed_binary[12:0], 1'b0};

    always @(posedge I_clk_25m or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_dist_binary      <= 14'd0;
            S_speed_binary     <= 14'd0;
            S_dist_work_bcd    <= 16'd0;
            S_speed_work_bcd   <= 16'd0;
            S_convert_count    <= 4'd0;
            S_convert_active   <= 1'b0;
            S_convert_valid    <= 1'b0;
            S_pending_dist_bcd <= 16'd0;
            S_pending_speed_bcd<= 16'd0;
            S_pending_valid    <= 1'b0;
            S_pending_toggle   <= 1'b0;
        end else if (I_update) begin
            S_dist_binary    <= (I_dist_mm > 20'd9999) ? 14'd9999 : I_dist_mm[13:0];
            S_speed_binary   <= (I_speed_mmps > 20'd9999) ? 14'd9999 : I_speed_mmps[13:0];
            S_dist_work_bcd  <= 16'd0;
            S_speed_work_bcd <= 16'd0;
            S_convert_count  <= 4'd0;
            S_convert_valid  <= I_metrics_valid;
            S_convert_active <= 1'b1;
        end else if (S_convert_active) begin
            S_dist_work_bcd  <= W_dist_next_bcd;
            S_speed_work_bcd <= W_speed_next_bcd;
            S_dist_binary    <= W_dist_next_binary;
            S_speed_binary   <= W_speed_next_binary;
            if (S_convert_count == 4'd13) begin
                S_pending_dist_bcd  <= W_dist_next_bcd;
                S_pending_speed_bcd <= W_speed_next_bcd;
                S_pending_valid     <= S_convert_valid;
                S_pending_toggle    <= ~S_pending_toggle;
                S_convert_active    <= 1'b0;
            end else begin
                S_convert_count <= S_convert_count + 4'd1;
            end
        end
    end

    always @(posedge I_clk_25m or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_state            <= ST_IDLE;
            S_shift_data      <= 16'h00ff;
            S_shift_count     <= 5'd0;
            S_shift_high      <= 1'b0;
            S_shift_tick      <= 16'd0;
            S_latch_tick      <= 16'd0;
            S_slot_tick       <= 16'd0;
            S_digit_index     <= 3'd0;
            S_active_frame    <= 16'h00ff;
            S_display_dist_bcd<= 16'd0;
            S_display_speed_bcd<=16'd0;
            S_display_valid   <= 1'b0;
            S_display_toggle  <= 1'b0;
            O_seg_sclk        <= 1'b0;
            O_seg_latch       <= 1'b0;
        end else begin
            case (S_state)
                ST_IDLE: begin
                    O_seg_latch <= 1'b0;
                    S_shift_tick <= 16'd0;
                    if (S_slot_tick == P_IDLE_DIV - 1) begin
                        S_slot_tick <= 16'd0;
                        S_shift_data <= 16'h00ff;
                        S_shift_count <= 5'd0;
                        S_shift_high <= 1'b0;
                        if ((S_digit_index == 3'd0) &&
                            (S_pending_toggle != S_display_toggle)) begin
                            S_active_frame <= f_frame_for_digit(
                                S_digit_index, S_pending_dist_bcd,
                                S_pending_speed_bcd, S_pending_valid);
                            S_display_dist_bcd  <= S_pending_dist_bcd;
                            S_display_speed_bcd <= S_pending_speed_bcd;
                            S_display_valid     <= S_pending_valid;
                            S_display_toggle    <= S_pending_toggle;
                        end else begin
                            S_active_frame <= f_frame_for_digit(
                                S_digit_index, S_display_dist_bcd,
                                S_display_speed_bcd, S_display_valid);
                        end
                        S_state <= ST_BLANK_SHIFT;
                    end else begin
                        S_slot_tick <= S_slot_tick + 16'd1;
                    end
                end

                ST_BLANK_SHIFT: begin
                    O_seg_latch <= 1'b0;
                    if (S_shift_tick == P_SHIFT_DIV - 1) begin
                        S_shift_tick <= 16'd0;
                        if (!S_shift_high) begin
                            O_seg_sclk  <= 1'b1;
                            S_shift_high<= 1'b1;
                        end else begin
                            O_seg_sclk   <= 1'b0;
                            S_shift_high <= 1'b0;
                            if (S_shift_count == 5'd15) begin
                                S_state <= ST_BLANK_LATCH;
                            end else begin
                                S_shift_data  <= {S_shift_data[14:0], 1'b0};
                                S_shift_count <= S_shift_count + 5'd1;
                            end
                        end
                    end else begin
                        S_shift_tick <= S_shift_tick + 16'd1;
                    end
                end

                ST_BLANK_LATCH: begin
                    O_seg_latch <= 1'b1;
                    if (S_latch_tick == P_SHIFT_DIV - 1) begin
                        S_latch_tick <= 16'd0;
                        S_shift_tick <= 16'd0;
                        S_state <= ST_BLANK_GAP;
                    end else begin
                        S_latch_tick <= S_latch_tick + 16'd1;
                    end
                end

                ST_BLANK_GAP: begin
                    O_seg_latch <= 1'b0;
                    if (S_shift_tick == P_SHIFT_DIV - 1) begin
                        S_shift_tick <= 16'd0;
                        S_shift_data <= S_active_frame;
                        S_shift_count <= 5'd0;
                        S_shift_high <= 1'b0;
                        S_state <= ST_ACTIVE_SHIFT;
                    end else begin
                        S_shift_tick <= S_shift_tick + 16'd1;
                    end
                end

                ST_ACTIVE_SHIFT: begin
                    O_seg_latch <= 1'b0;
                    if (S_shift_tick == P_SHIFT_DIV - 1) begin
                        S_shift_tick <= 16'd0;
                        if (!S_shift_high) begin
                            O_seg_sclk  <= 1'b1;
                            S_shift_high<= 1'b1;
                        end else begin
                            O_seg_sclk   <= 1'b0;
                            S_shift_high <= 1'b0;
                            if (S_shift_count == 5'd15) begin
                                S_state <= ST_ACTIVE_LATCH;
                            end else begin
                                S_shift_data  <= {S_shift_data[14:0], 1'b0};
                                S_shift_count <= S_shift_count + 5'd1;
                            end
                        end
                    end else begin
                        S_shift_tick <= S_shift_tick + 16'd1;
                    end
                end

                ST_ACTIVE_LATCH: begin
                    O_seg_latch <= 1'b1;
                    if (S_latch_tick == P_SHIFT_DIV - 1) begin
                        S_latch_tick <= 16'd0;
                        S_shift_tick <= 16'd0;
                        S_digit_index <= (S_digit_index == 3'd7) ? 3'd0 : S_digit_index + 3'd1;
                        S_state <= ST_IDLE;
                    end else begin
                        S_latch_tick <= S_latch_tick + 16'd1;
                    end
                end

                default: begin
                    O_seg_latch <= 1'b0;
                    S_state <= ST_IDLE;
                end
            endcase
        end
    end

endmodule
