//=============================================================================
// ctl_mdio_strap_read -- YT8531 clause-22 indirect-register sequencer
// 领域 / Domain : board management
// 时钟 / Clock  : I_clk_25m (25 MHz)
// 复位 / Reset  : I_rst_n; PHY reset release is followed by startup wait
// 验证 / Status : pure-RTL testbench is supplied under tests/eth_phy_fix_20261018
//=============================================================================
// 本模块使用 MDIO 的标准 clause-22 两阶段访问读取 YT8531 扩展寄存器。每个
// 扩展寄存器访问都先向 0x1E 写入索引，再读写 0x1F，避免依赖 PHY 保留索引。
//
// MDC 在每个低电平期开始时建立 MDIO，经过完整的低电平分频后才产生上升沿；
// 读事务在高电平末拍采样 TA 与数据，覆盖 PHY 的输出有效延迟。
// 失败的读事务按当前扩展寄存器重新开始，最多执行三次（初次访问加两次重试）。
//
// P_CONFIGURE_DELAYS=0 时只读 A001/A003。默认值 1 时，A001 的受控字段
// (mask 0x0300) 写成 0x0100，A003 的受控字段按 0x7C0F 清零；两者都会写回并完整回读，
// 回读仅校验受控字段，输出则保留回读的完整 16 位。
//=============================================================================
module ctl_mdio_strap_read #(
    parameter [4:0]  P_PHY_ADDR = 5'd0,
    parameter [15:0] P_EXT_ADDR = 16'hA001,
    parameter [4:0]  P_REG_OFF  = 5'h1E,
    parameter [4:0]  P_REG_DAT  = 5'h1F,
    parameter [7:0]  P_HALF     = 8'd4,
    parameter integer P_STARTUP_CYCLES = 250000,
    parameter integer P_CONFIGURE_DELAYS = 1
)(
    // The first eight ports keep their historical order.
    input  wire        I_clk_25m,
    output reg         O_mdc,
    output reg         O_mdio_oe,
    output reg         O_mdio_out,
    input  wire        I_mdio_in,
    output reg  [15:0] O_ext_val,
    output reg         O_rxc_dly_en,
    output reg         O_done,
    // All callers must connect I_rst_n; an omitted reset is not supported.
    input  wire        I_rst_n,
    output reg         O_error,
    output reg  [15:0] O_rgmii_cfg
);

    // -------------------------------------------------------------------------
    // Static protocol constants
    // -------------------------------------------------------------------------
    localparam [7:0]  C_HALF = (P_HALF < 8'd2) ? 8'd2 : P_HALF;
    localparam integer C_RETRY_LIMIT = 3;
    localparam [15:0] C_EXT_ADDR_SECOND = P_EXT_ADDR + 16'h0002;
    localparam [15:0] C_A001_FIELD_MASK = 16'h0300;
    localparam [15:0] C_A001_FIELD_VALUE = 16'h0100;
    localparam [15:0] C_A003_FIELD_MASK = 16'h7C0F;
    localparam [15:0] C_A003_FIELD_VALUE = 16'h0000;

    // Transaction states.  ST_FRAME is the only state that toggles MDC.
    localparam [3:0] ST_STARTUP    = 4'd0;
    localparam [3:0] ST_NEXT_OP    = 4'd1;
    localparam [3:0] ST_FRAME      = 4'd2;
    localparam [3:0] ST_RETRY_WAIT = 4'd3;
    localparam [3:0] ST_DONE       = 4'd4;
    localparam [3:0] ST_ERROR      = 4'd5;

    // A target access is a write of the indirect index, followed by a read of
    // the data register.  Configuration adds a write-index/write-data/readback
    // triplet after the first read.
    localparam [2:0] STEP_ADDR        = 3'd0;
    localparam [2:0] STEP_READ_OLD    = 3'd1;
    localparam [2:0] STEP_WRITE_ADDR  = 3'd2;
    localparam [2:0] STEP_WRITE_DATA  = 3'd3;
    localparam [2:0] STEP_FINAL_ADDR  = 3'd4;
    localparam [2:0] STEP_READ_FINAL  = 3'd5;

    reg [3:0]  S_state;
    reg [2:0]  S_step;
    reg        S_target_index;
    reg        S_frame_read;
    reg [4:0]  S_frame_reg;
    reg [15:0] S_frame_data;
    reg [13:0] S_frame_header;
    reg [6:0]  S_frame_bit;
    reg [7:0]  S_div;
    reg        S_phase_high;
    reg [15:0] S_rx_data;
    reg        S_frame_bad;
    reg [1:0]  S_retry_count;
    reg [31:0] S_startup_count;
    reg [7:0]  S_gap_count;
    reg [15:0] S_old_data;
    reg [15:0] S_expected_data;

    wire [15:0] W_target_ext_addr = S_target_index ?
                                    C_EXT_ADDR_SECOND : P_EXT_ADDR;
    wire W_final_fields_ok = (S_target_index == 1'b0) ?
                             ((S_rx_data & C_A001_FIELD_MASK) ==
                              (S_expected_data & C_A001_FIELD_MASK)) :
                             ((S_rx_data & C_A003_FIELD_MASK) ==
                              (S_expected_data & C_A003_FIELD_MASK));

    // -------------------------------------------------------------------------
    // Clause-22 sequencer
    // -------------------------------------------------------------------------
    // I_rst_n is the PHY reset release input.  The startup counter is therefore
    // a deterministic post-reset wait, rather than a second self-starting POR.
    always @(posedge I_clk_25m or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_state         <= ST_STARTUP;
            S_step          <= STEP_ADDR;
            S_target_index  <= 1'b0;
            S_frame_read    <= 1'b0;
            S_frame_reg     <= 5'd0;
            S_frame_data    <= 16'd0;
            S_frame_header  <= 14'd0;
            S_frame_bit     <= 7'd0;
            S_div           <= 8'd0;
            S_phase_high    <= 1'b0;
            S_rx_data       <= 16'd0;
            S_frame_bad     <= 1'b0;
            S_retry_count   <= 2'd0;
            S_startup_count <= 32'd0;
            S_gap_count     <= 8'd0;
            S_old_data      <= 16'd0;
            S_expected_data <= 16'd0;
            O_mdc           <= 1'b0;
            O_mdio_oe       <= 1'b0;
            O_mdio_out      <= 1'b1;
            O_ext_val       <= 16'd0;
            O_rxc_dly_en    <= 1'b0;
            O_done          <= 1'b0;
            O_error         <= 1'b0;
            O_rgmii_cfg     <= 16'd0;
        end else begin
            case (S_state)
                // -------------------------------------------------------------
                ST_STARTUP: begin
                    O_mdc      <= 1'b0;
                    O_mdio_oe  <= 1'b0;
                    O_mdio_out <= 1'b1;
                    if (P_STARTUP_CYCLES == 0) begin
                        S_state        <= ST_NEXT_OP;
                        S_target_index <= 1'b0;
                        S_step         <= STEP_ADDR;
                        S_retry_count  <= 2'd0;
                    end else if (S_startup_count >= P_STARTUP_CYCLES - 1) begin
                        S_state        <= ST_NEXT_OP;
                        S_target_index <= 1'b0;
                        S_step         <= STEP_ADDR;
                        S_retry_count  <= 2'd0;
                    end else begin
                        S_startup_count <= S_startup_count + 32'd1;
                    end
                end

                // -------------------------------------------------------------
                // Load one 64-bit clause-22 frame.  Output bits are established
                // by ST_FRAME on a later low-phase clock, leaving setup time.
                ST_NEXT_OP: begin
                    O_mdc       <= 1'b0;
                    O_mdio_oe   <= 1'b0;
                    O_mdio_out  <= 1'b1;
                    S_frame_bit <= 7'd0;
                    S_div       <= 8'd0;
                    S_phase_high<= 1'b0;
                    S_frame_bad <= 1'b0;
                    S_rx_data   <= 16'd0;

                    case (S_step)
                        STEP_ADDR: begin
                            S_frame_read   <= 1'b0;
                            S_frame_reg    <= P_REG_OFF;
                            S_frame_data   <= W_target_ext_addr;
                            S_frame_header <= {2'b01, 2'b01,
                                               P_PHY_ADDR, P_REG_OFF};
                        end
                        STEP_READ_OLD: begin
                            S_frame_read   <= 1'b1;
                            S_frame_reg    <= P_REG_DAT;
                            S_frame_data   <= 16'd0;
                            S_frame_header <= {2'b01, 2'b10,
                                               P_PHY_ADDR, P_REG_DAT};
                        end
                        STEP_WRITE_ADDR: begin
                            S_frame_read   <= 1'b0;
                            S_frame_reg    <= P_REG_OFF;
                            S_frame_data   <= W_target_ext_addr;
                            S_frame_header <= {2'b01, 2'b01,
                                               P_PHY_ADDR, P_REG_OFF};
                        end
                        STEP_WRITE_DATA: begin
                            S_frame_read   <= 1'b0;
                            S_frame_reg    <= P_REG_DAT;
                            S_frame_data   <= S_expected_data;
                            S_frame_header <= {2'b01, 2'b01,
                                               P_PHY_ADDR, P_REG_DAT};
                        end
                        STEP_FINAL_ADDR: begin
                            S_frame_read   <= 1'b0;
                            S_frame_reg    <= P_REG_OFF;
                            S_frame_data   <= W_target_ext_addr;
                            S_frame_header <= {2'b01, 2'b01,
                                               P_PHY_ADDR, P_REG_OFF};
                        end
                        default: begin
                            S_frame_read   <= 1'b1;
                            S_frame_reg    <= P_REG_DAT;
                            S_frame_data   <= 16'd0;
                            S_frame_header <= {2'b01, 2'b10,
                                               P_PHY_ADDR, P_REG_DAT};
                        end
                    endcase
                    S_state <= ST_FRAME;
                end

                // -------------------------------------------------------------
                // One frame.  The low phase establishes a bit; the following
                // high phase holds it and then advances at MDC falling edge.
                ST_FRAME: begin
                    if (!S_phase_high) begin
                        if (S_div == 8'd0) begin
                            if (S_frame_bit < 7'd32) begin
                                O_mdio_oe  <= 1'b1;
                                O_mdio_out <= 1'b1;
                            end else if (S_frame_bit < 7'd46) begin
                                O_mdio_oe  <= 1'b1;
                                O_mdio_out <= S_frame_header[
                                    13 - (S_frame_bit - 7'd32)];
                            end else if (S_frame_bit < 7'd48) begin
                                if (S_frame_read) begin
                                    O_mdio_oe  <= 1'b0;
                                    O_mdio_out <= 1'b1;
                                end else begin
                                    O_mdio_oe  <= 1'b1;
                                    O_mdio_out <= (S_frame_bit == 7'd46);
                                end
                            end else if (S_frame_read) begin
                                O_mdio_oe  <= 1'b0;
                                O_mdio_out <= 1'b1;
                            end else begin
                                O_mdio_oe  <= 1'b1;
                                O_mdio_out <= S_frame_data[
                                    15 - (S_frame_bit - 7'd48)];
                            end
                        end

                        if (S_div == C_HALF - 8'd1) begin
                            O_mdc        <= 1'b1;
                            S_phase_high <= 1'b1;
                            S_div        <= 8'd0;

                        end else begin
                            S_div <= S_div + 8'd1;
                        end
                    end else begin
                        // YT8531 Table 96 allows MDIO output to become valid
                        // after MDC rising.  Sample at the end of the high
                        // phase so both the 20 ns device delay and the
                        // generic 300 ns Clause-22 budget are covered when
                        // P_HALF is set to 13 at 25 MHz.
                        if (S_div == C_HALF - 8'd1) begin
                            if (S_frame_read && (S_frame_bit == 7'd46)) begin
                                if (I_mdio_in != 1'b0)
                                    S_frame_bad <= 1'b1;
                            end else if (S_frame_read &&
                                         (S_frame_bit >= 7'd47) &&
                                         (S_frame_bit <= 7'd62)) begin
                                S_rx_data <= {S_rx_data[14:0], I_mdio_in};
                            end
                        end

                        if (S_div == C_HALF - 8'd1) begin
                            O_mdc        <= 1'b0;
                            S_phase_high <= 1'b0;
                            S_div        <= 8'd0;

                            // A bad TA is latched, but the frame is still
                            // clocked through bit 63 so the PHY sees a
                            // complete clause-22 frame before a retry.
                            if (S_frame_bad && (S_frame_bit == 7'd63)) begin
                                // A read without a valid TA is retried from
                                // the index write, never from a stale pointer.
                                O_mdio_oe  <= 1'b0;
                                O_mdio_out <= 1'b1;
                                if (S_retry_count < C_RETRY_LIMIT - 1) begin
                                    S_retry_count <= S_retry_count + 2'd1;
                                    S_step        <= STEP_ADDR;
                                    S_gap_count   <= 8'd0;
                                    S_state       <= ST_RETRY_WAIT;
                                end else begin
                                    O_error <= 1'b1;
                                    O_done  <= 1'b1;
                                    S_state <= ST_ERROR;
                                end
                            end else if (S_frame_bit == 7'd63) begin
                                S_frame_bit <= 7'd0;
                                if (!S_frame_read) begin
                                    if (S_step == STEP_ADDR)
                                        S_step <= STEP_READ_OLD;
                                    else if (S_step == STEP_WRITE_ADDR)
                                        S_step <= STEP_WRITE_DATA;
                                    else if (S_step == STEP_WRITE_DATA)
                                        S_step <= STEP_FINAL_ADDR;
                                    else
                                        S_step <= STEP_READ_FINAL;
                                    S_state <= ST_NEXT_OP;
                                end else if (S_step == STEP_READ_OLD) begin
                                    S_old_data <= S_rx_data;
                                    if (P_CONFIGURE_DELAYS != 0) begin
                                        if (S_target_index == 1'b0)
                                            S_expected_data <=
                                                (S_rx_data & ~C_A001_FIELD_MASK) |
                                                C_A001_FIELD_VALUE;
                                        else
                                            S_expected_data <=
                                                (S_rx_data & ~C_A003_FIELD_MASK) |
                                                C_A003_FIELD_VALUE;
                                        S_step  <= STEP_WRITE_ADDR;
                                        S_state <= ST_NEXT_OP;
                                    end else begin
                                        if (S_target_index == 1'b0) begin
                                            O_ext_val    <= S_rx_data;
                                            O_rxc_dly_en <= S_rx_data[8];
                                        end else begin
                                            O_rgmii_cfg <= S_rx_data;
                                        end
                                        if (S_target_index == 1'b0) begin
                                            S_target_index <= 1'b1;
                                            S_retry_count  <= 2'd0;
                                            S_step         <= STEP_ADDR;
                                            S_state        <= ST_NEXT_OP;
                                        end else begin
                                            S_state <= ST_DONE;
                                        end
                                    end
                                end else begin
                                    // Final read: validate only fields owned by
                                    // this diagnostic, and expose the complete
                                    // readback even when validation requests a
                                    // retry.
                                    if (S_target_index == 1'b0) begin
                                        O_ext_val    <= S_rx_data;
                                        O_rxc_dly_en <= S_rx_data[8];
                                    end else begin
                                        O_rgmii_cfg <= S_rx_data;
                                    end
                                    if (W_final_fields_ok) begin
                                        if (S_target_index == 1'b0) begin
                                            S_target_index <= 1'b1;
                                            S_retry_count  <= 2'd0;
                                            S_step         <= STEP_ADDR;
                                            S_state        <= ST_NEXT_OP;
                                        end else begin
                                            S_state <= ST_DONE;
                                        end
                                    end else if (S_retry_count <
                                                C_RETRY_LIMIT - 1) begin
                                        S_retry_count <= S_retry_count + 2'd1;
                                        S_step        <= STEP_ADDR;
                                        S_gap_count   <= 8'd0;
                                        S_state       <= ST_RETRY_WAIT;
                                    end else begin
                                        O_error <= 1'b1;
                                        O_done  <= 1'b1;
                                        S_state <= ST_ERROR;
                                    end
                                end
                            end else begin
                                S_frame_bit <= S_frame_bit + 7'd1;
                            end
                        end else begin
                            S_div <= S_div + 8'd1;
                        end
                    end
                end

                // -------------------------------------------------------------
                // Keep MDC low for one half period after a failed read before
                // restarting at STEP_ADDR.
                ST_RETRY_WAIT: begin
                    O_mdc      <= 1'b0;
                    O_mdio_oe  <= 1'b0;
                    O_mdio_out <= 1'b1;
                    if (S_gap_count == C_HALF - 8'd1) begin
                        S_gap_count <= 8'd0;
                        S_state     <= ST_NEXT_OP;
                    end else begin
                        S_gap_count <= S_gap_count + 8'd1;
                    end
                end

                // -------------------------------------------------------------
                ST_DONE: begin
                    O_mdc      <= 1'b0;
                    O_mdio_oe  <= 1'b0;
                    O_mdio_out <= 1'b1;
                    O_done     <= 1'b1;
                end

                // -------------------------------------------------------------
                ST_ERROR: begin
                    O_mdc      <= 1'b0;
                    O_mdio_oe  <= 1'b0;
                    O_mdio_out <= 1'b1;
                    O_done     <= 1'b1;
                    O_error    <= 1'b1;
                end

                default: begin
                    S_state <= ST_STARTUP;
                end
            endcase
        end
    end

endmodule
