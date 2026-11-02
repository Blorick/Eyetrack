// AWB：完整帧快照计算候选增益，下一帧 SOF 原子应用；帧内增益固定。
// 基于原米联客 awb 接口；保留四像素 RGB888 和 ready/valid 流接口。
module awb #(
    parameter IMG_HEIGHT = 1080,
    parameter IMG_WIDTH = 1920,
    parameter [7:0] SAMPLE_MIN = 8'd32,
    parameter [7:0] SAMPLE_MAX = 8'd240,
    parameter [7:0] NEUTRAL_DELTA = 8'd24,
    parameter [19:0] GAIN_MIN = 20'h08000,
    parameter [19:0] GAIN_MAX = 20'h20000
)(
    input I_clk,
    input I_rst_n,
    input I_tlast,
    input I_tuser,
    input [95:0] I_tdata,
    input I_tvalid,
    output I_tready,
    output O_tlast,
    output O_tuser,
    output [95:0] O_tdata,
    output O_tvalid,
    input O_tready
);
    localparam integer FRAME_WORDS = IMG_HEIGHT * IMG_WIDTH / 4;
    localparam [19:0] UNITY_GAIN = 20'h10000;

    reg stage_valid;
    reg stage_last;
    reg stage_user;
    reg output_valid;
    reg output_last;
    reg output_user;
    reg [95:0] output_data;
    // 独立SOF/行末脉冲也占用一个槽；背压时不能被后续无valid拍覆盖。
    wire output_occupied = output_valid || output_user || output_last;
    wire stage_occupied = stage_valid || stage_user || stage_last;
    wire output_ready = !output_occupied || O_tready;
    wire input_ready = !stage_occupied || output_ready;
    wire accept_word = I_tvalid && input_ready;
    wire frame_sof = I_tuser && input_ready;

    assign I_tready = input_ready;
    assign O_tvalid = output_valid;
    assign O_tlast = output_last;
    assign O_tuser = output_user;
    assign O_tdata = output_valid ? output_data : 96'd0;

    // 每个四像素组均匀采样一像素；只让近中性、非暗/非饱和像素参与估计。
    wire [7:0] sample_r = I_tdata[23:16];
    wire [7:0] sample_g = I_tdata[15:8];
    wire [7:0] sample_b = I_tdata[7:0];
    // 已接收事务进入独立的三拍统计流水；不再次受视频背压停顿。
    // 分开 RG 比较、RGB 比较和中性判定，切断输入到快照使能的长组合路径。
    reg [23:0] range_sample, classify_sample, statistic_sample;
    reg [7:0] range_max, range_min, classify_max, classify_min;
    reg range_accept, range_sof, classify_accept, classify_sof;
    reg statistic_accept, statistic_sof, statistic_eligible;
    wire neutral_sample = (classify_min >= SAMPLE_MIN) && (classify_max <= SAMPLE_MAX) &&
                          ((classify_max - classify_min) <= NEUTRAL_DELTA);
    wire [31:0] sample_r_value = {24'd0, statistic_sample[23:16]};
    wire [31:0] sample_g_value = {24'd0, statistic_sample[15:8]};
    wire [31:0] sample_b_value = {24'd0, statistic_sample[7:0]};

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            range_sample <= 24'd0; classify_sample <= 24'd0; statistic_sample <= 24'd0;
            range_max <= 8'd0; range_min <= 8'd0;
            classify_max <= 8'd0; classify_min <= 8'd0;
            range_accept <= 1'b0; range_sof <= 1'b0;
            classify_accept <= 1'b0; classify_sof <= 1'b0;
            statistic_accept <= 1'b0; statistic_sof <= 1'b0; statistic_eligible <= 1'b0;
        end else begin
            range_sample <= I_tdata[23:0];
            range_max <= sample_r > sample_g ? sample_r : sample_g;
            range_min <= sample_r < sample_g ? sample_r : sample_g;
            range_accept <= accept_word; range_sof <= frame_sof;
            classify_sample <= range_sample;
            classify_max <= range_max > range_sample[7:0] ? range_max : range_sample[7:0];
            classify_min <= range_min < range_sample[7:0] ? range_min : range_sample[7:0];
            classify_accept <= range_accept; classify_sof <= range_sof;
            statistic_sample <= neutral_sample ? classify_sample : 24'd0;
            statistic_eligible <= neutral_sample;
            statistic_accept <= classify_accept; statistic_sof <= classify_sof;
        end
    end

    reg frame_active;
    reg [19:0] frame_word_count;
    reg [19:0] sample_count;
    reg [31:0] sum_r;
    reg [31:0] sum_g;
    reg [31:0] sum_b;
    wire frame_complete = statistic_accept &&
        ((statistic_sof && (FRAME_WORDS == 1)) ||
         (!statistic_sof && frame_active && (frame_word_count == FRAME_WORDS - 1)));
    wire [31:0] closed_sum_r = statistic_sof ? sample_r_value : sum_r + sample_r_value;
    wire [31:0] closed_sum_g = statistic_sof ? sample_g_value : sum_g + sample_g_value;
    wire [31:0] closed_sum_b = statistic_sof ? sample_b_value : sum_b + sample_b_value;
    wire closed_has_samples = statistic_sof ? statistic_eligible : ((sample_count != 0) || statistic_eligible);

    reg [31:0] snapshot_r;
    reg [31:0] snapshot_g;
    reg [31:0] snapshot_b;
    reg calc_start;
    reg calc_busy;
    reg result_r_seen;
    reg result_b_seen;
    reg pending_valid;
    reg [19:0] pending_r_gain;
    reg [19:0] pending_b_gain;
    reg [19:0] applied_r_gain;
    reg [19:0] applied_b_gain;
    // 完整商与同拍done保留为可观察信号，候选汇合逻辑无需改变。
    reg [47:0] quotient_r;
    reg [47:0] quotient_b;
    reg done_r;
    reg done_b;
    wire [19:0] limited_r = quotient_r < {28'd0, GAIN_MIN} ? GAIN_MIN :
        (quotient_r > {28'd0, GAIN_MAX} ? GAIN_MAX : quotient_r[19:0]);
    wire [19:0] limited_b = quotient_b < {28'd0, GAIN_MIN} ? GAIN_MIN :
        (quotient_b > {28'd0, GAIN_MAX} ? GAIN_MAX : quotient_b[19:0]);
    // SOF 与首个有效组同拍时，首组也必须使用新一帧的候选增益。
    wire [19:0] pixel_r_gain = frame_sof && pending_valid ? pending_r_gain : applied_r_gain;
    wire [19:0] pixel_b_gain = frame_sof && pending_valid ? pending_b_gain : applied_b_gain;

    // 一帧只需两个比值，串行复用48轮恢复除法器代替全流水除法。
    localparam [1:0] DIV_IDLE = 2'd0;
    localparam [1:0] DIV_RED = 2'd1;
    localparam [1:0] DIV_BLUE = 2'd2;
    reg [1:0] division_state;
    reg [1:0] division_next_state;
    reg [47:0] division_quotient;
    reg [31:0] division_remainder;
    reg [31:0] division_denominator;
    reg [5:0] division_step;
    // 移位后的余数可超过32位；比较和试减必须保留第33位。
    wire [32:0] division_trial = {division_remainder, division_quotient[47]};
    wire [32:0] division_subtracted = division_trial - {1'b0, division_denominator};
    wire division_take = division_trial >= {1'b0, division_denominator};
    wire [31:0] division_next_remainder = division_take ? division_subtracted[31:0] : division_trial[31:0];
    wire [47:0] division_next_quotient = {division_quotient[46:0], division_take};
    wire division_inputs_valid = snapshot_r != 0 && snapshot_b != 0;

    // 状态寄存只跟随组合下一状态，复位取消未完成的除法事务。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) division_state <= DIV_IDLE;
        else division_state <= division_next_state;
    end

    // calc_start在忙状态不重入；两通道之间直接重装共享运算寄存器。
    always @(*) begin
        division_next_state = division_state;
        case (division_state)
            DIV_IDLE: if (calc_start && division_inputs_valid) division_next_state = DIV_RED;
            DIV_RED: if (division_step == 6'd47) division_next_state = DIV_BLUE;
            DIV_BLUE: if (division_step == 6'd47) division_next_state = DIV_IDLE;
            default: division_next_state = DIV_IDLE;
        endcase
    end

    // 商寄存初值即分子；每轮移出一位分子并追加一位商，不另存分子移位器。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            division_quotient <= 48'd0;
            division_remainder <= 32'd0;
            division_denominator <= 32'd1;
            division_step <= 6'd0;
            quotient_r <= 48'd0;
            quotient_b <= 48'd0;
            done_r <= 1'b0;
            done_b <= 1'b0;
        end else begin
            done_r <= 1'b0;
            done_b <= 1'b0;
            case (division_state)
                DIV_IDLE: if (calc_start) begin
                    division_step <= 6'd0;
                    division_remainder <= 32'd0;
                    if (division_inputs_valid) begin
                        division_quotient <= {snapshot_g, 16'd0};
                        division_denominator <= snapshot_r;
                    end else begin
                        // 正常帧提交已排除零分母；防御分支也不能产生除零回绕。
                        quotient_r <= {28'd0, UNITY_GAIN};
                        quotient_b <= {28'd0, UNITY_GAIN};
                        done_r <= 1'b1;
                        done_b <= 1'b1;
                    end
                end
                DIV_RED, DIV_BLUE: begin
                    division_quotient <= division_next_quotient;
                    division_remainder <= division_next_remainder;
                    division_step <= division_step + 1'b1;
                    if (division_step == 6'd47) begin
                        division_step <= 6'd0;
                        if (division_state == DIV_RED) begin
                            // 末轮必须包含刚追加的商最低位，然后从同一快照重装B。
                            quotient_r <= division_next_quotient;
                            division_quotient <= {snapshot_g, 16'd0};
                            division_remainder <= 32'd0;
                            division_denominator <= snapshot_b;
                        end else begin
                            quotient_b <= division_next_quotient;
                            done_r <= 1'b1;
                            done_b <= 1'b1;
                        end
                    end
                end
                default: begin
                    division_step <= 6'd0;
                    division_remainder <= 32'd0;
                end
            endcase
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            frame_active <= 1'b0;
            frame_word_count <= 20'd0;
            sample_count <= 20'd0;
            sum_r <= 32'd0;
            sum_g <= 32'd0;
            sum_b <= 32'd0;
            snapshot_r <= 32'd1;
            snapshot_g <= 32'd1;
            snapshot_b <= 32'd1;
            calc_start <= 1'b0;
            calc_busy <= 1'b0;
            result_r_seen <= 1'b0;
            result_b_seen <= 1'b0;
            pending_valid <= 1'b0;
            pending_r_gain <= UNITY_GAIN;
            pending_b_gain <= UNITY_GAIN;
            applied_r_gain <= UNITY_GAIN;
            applied_b_gain <= UNITY_GAIN;
        end else begin
            calc_start <= 1'b0;
            // 增益仍在输入 SOF 应用；统计 SOF 只控制延迟后的累加。
            if (frame_sof && pending_valid) begin
                applied_r_gain <= pending_r_gain;
                applied_b_gain <= pending_b_gain;
                pending_valid <= 1'b0;
            end
            if (statistic_sof) begin
                frame_active <= 1'b1;
                frame_word_count <= statistic_accept ? 20'd1 : 20'd0;
                sample_count <= statistic_accept && statistic_eligible ? 20'd1 : 20'd0;
                sum_r <= statistic_accept ? sample_r_value : 32'd0;
                sum_g <= statistic_accept ? sample_g_value : 32'd0;
                sum_b <= statistic_accept ? sample_b_value : 32'd0;
            end else if (statistic_accept && frame_active) begin
                frame_word_count <= frame_word_count + 1'b1;
                sample_count <= sample_count + statistic_eligible;
                sum_r <= sum_r + sample_r_value;
                sum_g <= sum_g + sample_g_value;
                sum_b <= sum_b + sample_b_value;
            end
            if (frame_complete) begin
                frame_active <= 1'b0;
                // 未完整的帧不触发计算；空样本/零分母/忙时保留当前增益。
                if (closed_has_samples && (closed_sum_r != 0) && (closed_sum_b != 0) &&
                    !calc_busy && !pending_valid) begin
                    snapshot_r <= closed_sum_r;
                    snapshot_g <= closed_sum_g;
                    snapshot_b <= closed_sum_b;
                    calc_start <= 1'b1;
                    calc_busy <= 1'b1;
                    result_r_seen <= 1'b0;
                    result_b_seen <= 1'b0;
                end
            end
            if (calc_busy) begin
                if (done_r) begin
                    pending_r_gain <= limited_r;
                    result_r_seen <= 1'b1;
                end
                if (done_b) begin
                    pending_b_gain <= limited_b;
                    result_b_seen <= 1'b1;
                end
                if ((result_r_seen || done_r) && (result_b_seen || done_b)) begin
                    pending_valid <= 1'b1;
                    calc_busy <= 1'b0;
                end
            end
        end
    end

    // 两个弹性寄存级。metadata 与 RGB 共同推进，背压时保持输出不变。
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            stage_valid <= 1'b0;
            stage_last <= 1'b0;
            stage_user <= 1'b0;
            output_valid <= 1'b0;
            output_last <= 1'b0;
            output_user <= 1'b0;
        end else begin
            if (input_ready) begin
                stage_valid <= I_tvalid;
                stage_last <= I_tlast;
                stage_user <= I_tuser;
            end
            if (output_ready) begin
                output_valid <= stage_valid;
                output_last <= stage_last;
                output_user <= stage_user;
            end
        end
    end

    genvar lane;
    generate for (lane = 0; lane < 4; lane = lane + 1) begin : g_pixel
        reg [27:0] multiplied_r;
        reg [27:0] multiplied_b;
        reg [7:0] delayed_g;
        wire [7:0] saturated_r = |multiplied_r[27:24] ? 8'd255 : multiplied_r[23:16];
        wire [7:0] saturated_b = |multiplied_b[27:24] ? 8'd255 : multiplied_b[23:16];
        always @(posedge I_clk or negedge I_rst_n) begin
            if (!I_rst_n) begin
                multiplied_r <= 28'd0;
                multiplied_b <= 28'd0;
                delayed_g <= 8'd0;
                output_data[lane * 24 +: 24] <= 24'd0;
            end else begin
                if (input_ready && I_tvalid) begin
                    multiplied_r <= pixel_r_gain * I_tdata[lane * 24 + 16 +: 8];
                    multiplied_b <= pixel_b_gain * I_tdata[lane * 24 +: 8];
                    delayed_g <= I_tdata[lane * 24 + 8 +: 8];
                end
                if (output_ready)
                    output_data[lane * 24 +: 24] <= {saturated_r, delayed_g, saturated_b};
            end
        end
    end endgenerate
endmodule
