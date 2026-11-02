`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : declared module clock domain(s)
//  复位     : declared asynchronous active-low reset(s)
//  参数     : frozen module parameters; no interface change
//  Contract: comment-only style clone; ports and expressions are unchanged
// Module: ctl_ink_packet_tx
// Function: transmit two ordered ink-packet banks through a granted GMII port
// Domain: ctl; independent application and 125 MHz GMII TX clocks
// Reset: both domains receive the same asynchronous reset assertion
// Parameters: MAC addresses and dedicated ink EtherType
// Contract: write only a free bank; commit next bank with length 16/32/52/178
// Verification: independent Ethernet vectors, ownership, wrap and reset tests
// -----------------------------------------------------------------------------
module ctl_ink_packet_tx #(
    parameter [47:0] P_LOCAL_MAC = 48'h02_00_00_12_34_56,
    parameter [47:0] P_PEER_MAC = 48'h02_00_00_65_43_21,
    parameter [15:0] P_ETHERTYPE = 16'h88B7
) (
    input wire I_clk_app,
    input wire I_rst_app_n,
    output wire [1:0] O_free_bank_mask,
    output wire O_next_commit_bank,
    input wire I_write_en,
    input wire I_write_bank,
    input wire [7:0] I_write_addr,
    input wire [7:0] I_write_data,
    input wire I_commit,
    input wire I_commit_bank,
    input wire [8:0] I_packet_length,
    output reg O_commit_accept,
    output reg [31:0] O_commit_drop_count,
    input wire I_clk_tx,
    input wire I_rst_tx_n,
    output wire O_tx_request,
    input wire I_tx_grant,
    output reg O_tx_done,
    output reg [7:0] O_gmii_txd,
    output reg O_gmii_tx_en,
    output wire O_gmii_tx_er
);

    localparam [2:0] P_IDLE = 3'd0;
    localparam [2:0] P_PREAMBLE = 3'd1;
    localparam [2:0] P_FRAME = 3'd2;
    localparam [2:0] P_FCS = 3'd3;
    localparam [2:0] P_IFG = 3'd4;
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [7:0] S_packet_mem [0:511];
    reg [8:0] S_length_mem [0:1];
    // This cache belongs to the same committed bank as its packet length.
    // Publishing the existing Gray pointer publishes both metadata fields.
    reg [8:0] S_wire_length_mem [0:1];
    reg [1:0] S_write_bin_app;
    reg [1:0] S_write_gray_app;
    reg [1:0] S_read_gray_s1_app;
    reg [1:0] S_read_gray_s2_app;
    reg [1:0] S_read_bin_tx;
    reg [1:0] S_read_gray_tx;
    reg [1:0] S_write_gray_s1_tx;
    reg [1:0] S_write_gray_s2_tx;
    reg [2:0] S_state_tx;
    reg [3:0] S_preamble_pos_tx;
    reg [8:0] S_length_tx;
    reg [8:0] S_wire_length_tx;
    reg [8:0] S_frame_pos_tx;
    reg [1:0] S_fcs_pos_tx;
    reg [3:0] S_ifg_pos_tx;
    reg [31:0] S_crc_tx;
    reg [31:0] S_fcs_tx;
    reg [7:0] S_mem_q_tx;
    reg S_pad_q_tx;
    reg [7:0] S_data_q_tx;
    reg [7:0] W_frame_data_tx;

    wire W_full_app = S_write_gray_app == ~S_read_gray_s2_app;
    wire W_empty_app = S_write_gray_app == S_read_gray_s2_app;
    wire [1:0] W_write_next_app = S_write_bin_app + 2'd1;
    wire [1:0] W_read_next_tx = S_read_bin_tx + 2'd1;
    wire W_length_valid_app = I_packet_length == 9'd16 || I_packet_length == 9'd32 ||
        I_packet_length == 9'd52 || I_packet_length == 9'd178;
    wire W_commit_ok_app = !W_full_app &&
        I_commit_bank == S_write_bin_app[0] && W_length_valid_app;
    wire [8:0] W_payload_index_tx = S_frame_pos_tx - 9'd12;
    wire W_next_pad_tx = S_frame_pos_tx >= 9'd12 &&
        W_payload_index_tx >= S_length_tx;

    assign O_free_bank_mask = W_full_app ? 2'b00 : W_empty_app ? 2'b11 :
        S_write_bin_app[0] ? 2'b10 : 2'b01;
    // A builder can restart after the transport has drained without resetting
    // either FIFO pointer. Both banks are free then, so the mask alone cannot
    // identify the next ordered bank; expose its source-domain parity.
    assign O_next_commit_bank = S_write_bin_app[0];
    assign O_tx_request = S_read_gray_tx != S_write_gray_s2_tx;
    assign O_gmii_tx_er = 1'b0;

    function [7:0] f_mac_byte;
        input [47:0] I_mac;
        input [3:0] I_index;
        begin
            case (I_index)
                4'd0: f_mac_byte = I_mac[47:40];
                4'd1: f_mac_byte = I_mac[39:32];
                4'd2: f_mac_byte = I_mac[31:24];
                4'd3: f_mac_byte = I_mac[23:16];
                4'd4: f_mac_byte = I_mac[15:8];
                default: f_mac_byte = I_mac[7:0];
            endcase
        end
    endfunction

    function [31:0] f_crc_nibble;
        input [3:0] I_index;
        begin
            case (I_index)
                4'h0: f_crc_nibble = 32'h0000_0000;
                4'h1: f_crc_nibble = 32'h1DB7_1064;
                4'h2: f_crc_nibble = 32'h3B6E_20C8;
                4'h3: f_crc_nibble = 32'h26D9_30AC;
                4'h4: f_crc_nibble = 32'h76DC_4190;
                4'h5: f_crc_nibble = 32'h6B6B_51F4;
                4'h6: f_crc_nibble = 32'h4DB2_6158;
                4'h7: f_crc_nibble = 32'h5005_713C;
                4'h8: f_crc_nibble = 32'hEDB8_8320;
                4'h9: f_crc_nibble = 32'hF00F_9344;
                4'hA: f_crc_nibble = 32'hD6D6_A3E8;
                4'hB: f_crc_nibble = 32'hCB61_B38C;
                4'hC: f_crc_nibble = 32'h9B64_C2B0;
                4'hD: f_crc_nibble = 32'h86D3_D2D4;
                4'hE: f_crc_nibble = 32'hA00A_E278;
                default: f_crc_nibble = 32'hBDBD_F21C;
            endcase
        end
    endfunction

    function [31:0] f_crc_byte;
        input [31:0] I_crc;
        input [7:0] I_data;
        reg [31:0] W_mid;
        begin
            W_mid = (I_crc >> 4) ^ f_crc_nibble(I_crc[3:0] ^ I_data[3:0]);
            f_crc_byte = (W_mid >> 4) ^
                f_crc_nibble(W_mid[3:0] ^ I_data[7:4]);
        end
    endfunction

    function [31:0] f_header_crc;
        input [47:0] I_destination;
        input [47:0] I_source;
        input [15:0] I_type;
        reg [31:0] W_crc;
        integer W_index;
        begin
            W_crc = 32'hFFFF_FFFF;
            for (W_index = 0; W_index < 6; W_index = W_index + 1)
                W_crc = f_crc_byte(W_crc, f_mac_byte(I_destination, W_index[3:0]));
            for (W_index = 0; W_index < 6; W_index = W_index + 1)
                W_crc = f_crc_byte(W_crc, f_mac_byte(I_source, W_index[3:0]));
            W_crc = f_crc_byte(W_crc, I_type[15:8]);
            f_header_crc = f_crc_byte(W_crc, I_type[7:0]);
        end
    endfunction
    localparam [31:0] P_HEADER_CRC = f_header_crc(P_PEER_MAC, P_LOCAL_MAC, P_ETHERTYPE);

    // A free-bank write is the only write port. No reset is attached to RAM.
    always @(posedge I_clk_app) begin
        if (I_write_en && O_free_bank_mask[I_write_bank])
            S_packet_mem[{I_write_bank, I_write_addr}] <= I_write_data;
    end
    always @(posedge I_clk_tx) begin
        S_mem_q_tx <= S_packet_mem[{S_read_bin_tx[0], W_payload_index_tx[7:0]}];
        S_pad_q_tx <= W_next_pad_tx;
        S_data_q_tx <= S_pad_q_tx ? 8'd0 : S_mem_q_tx;
    end

    always @(*) begin
        if (S_frame_pos_tx < 9'd6)
            W_frame_data_tx = f_mac_byte(P_PEER_MAC, S_frame_pos_tx[3:0]);
        else if (S_frame_pos_tx < 9'd12)
            W_frame_data_tx = f_mac_byte(P_LOCAL_MAC, S_frame_pos_tx[3:0] - 4'd6);
        else if (S_frame_pos_tx == 9'd12)
            W_frame_data_tx = P_ETHERTYPE[15:8];
        else if (S_frame_pos_tx == 9'd13)
            W_frame_data_tx = P_ETHERTYPE[7:0];
        else
            W_frame_data_tx = S_data_q_tx;
    end

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            S_write_bin_app <= 2'd0;
            S_write_gray_app <= 2'd0;
            S_read_gray_s1_app <= 2'd0;
            S_read_gray_s2_app <= 2'd0;
            S_length_mem[0] <= 9'd0;
            S_length_mem[1] <= 9'd0;
            S_wire_length_mem[0] <= 9'd46;
            S_wire_length_mem[1] <= 9'd46;
            O_commit_accept <= 1'b0;
            O_commit_drop_count <= 32'd0;
        end else begin
            S_read_gray_s1_app <= S_read_gray_tx;
            S_read_gray_s2_app <= S_read_gray_s1_app;
            O_commit_accept <= 1'b0;
            if (I_commit) begin
                if (W_commit_ok_app) begin
                    S_length_mem[S_write_bin_app[0]] <= I_packet_length;
                    S_wire_length_mem[S_write_bin_app[0]] <= I_packet_length < 9'd46 ?
                        9'd46 : I_packet_length;
                    S_write_bin_app <= W_write_next_app;
                    S_write_gray_app <= (W_write_next_app >> 1) ^ W_write_next_app;
                    O_commit_accept <= 1'b1;
                end else
                    O_commit_drop_count <= O_commit_drop_count + 32'd1;
            end
        end
    end

    always @(posedge I_clk_tx or negedge I_rst_tx_n) begin
        if (!I_rst_tx_n) begin
            S_read_bin_tx <= 2'd0;
            S_read_gray_tx <= 2'd0;
            S_write_gray_s1_tx <= 2'd0;
            S_write_gray_s2_tx <= 2'd0;
            S_state_tx <= P_IDLE;
            S_preamble_pos_tx <= 4'd0;
            S_length_tx <= 9'd0;
            S_wire_length_tx <= 9'd0;
            S_frame_pos_tx <= 9'd0;
            S_fcs_pos_tx <= 2'd0;
            S_ifg_pos_tx <= 4'd0;
            S_crc_tx <= P_HEADER_CRC;
            S_fcs_tx <= 32'd0;
            O_tx_done <= 1'b0;
            O_gmii_txd <= 8'd0;
            O_gmii_tx_en <= 1'b0;
        end else begin
            S_write_gray_s1_tx <= S_write_gray_app;
            S_write_gray_s2_tx <= S_write_gray_s1_tx;
            O_tx_done <= 1'b0;
            O_gmii_txd <= 8'd0;
            O_gmii_tx_en <= 1'b0;
            case (S_state_tx)
                P_IDLE: begin
                    if (O_tx_request && I_tx_grant) begin
                        S_length_tx <= S_length_mem[S_read_bin_tx[0]];
                        S_wire_length_tx <= S_wire_length_mem[S_read_bin_tx[0]];
                        S_preamble_pos_tx <= 4'd0;
                        S_frame_pos_tx <= 9'd0;
                        S_crc_tx <= P_HEADER_CRC;
                        S_state_tx <= P_PREAMBLE;
                    end
                end
                P_PREAMBLE: begin
                    O_gmii_tx_en <= 1'b1;
                    O_gmii_txd <= S_preamble_pos_tx == 4'd7 ? 8'hD5 : 8'h55;
                    if (S_preamble_pos_tx == 4'd7)
                        S_state_tx <= P_FRAME;
                    else
                        S_preamble_pos_tx <= S_preamble_pos_tx + 4'd1;
                end
                P_FRAME: begin
                    O_gmii_tx_en <= 1'b1;
                    O_gmii_txd <= W_frame_data_tx;
                    // The fixed MAC/type header is already in P_HEADER_CRC.
                    // Every CRC update here consumes a payload/padding byte;
                    // the last byte is also payload because wire length >=46.
                    // Keep the header-position decoder out of the CRC cone.
                    if (S_frame_pos_tx >= 9'd14)
                        S_crc_tx <= f_crc_byte(S_crc_tx, S_data_q_tx);
                    if (S_frame_pos_tx == S_wire_length_tx + 9'd13) begin
                        S_fcs_tx <= ~f_crc_byte(S_crc_tx, S_data_q_tx);
                        S_fcs_pos_tx <= 2'd0;
                        S_state_tx <= P_FCS;
                    end else
                        S_frame_pos_tx <= S_frame_pos_tx + 9'd1;
                end
                P_FCS: begin
                    O_gmii_tx_en <= 1'b1;
                    case (S_fcs_pos_tx)
                        2'd0: O_gmii_txd <= S_fcs_tx[7:0];
                        2'd1: O_gmii_txd <= S_fcs_tx[15:8];
                        2'd2: O_gmii_txd <= S_fcs_tx[23:16];
                        default: O_gmii_txd <= S_fcs_tx[31:24];
                    endcase
                    if (S_fcs_pos_tx == 2'd3) begin
                        S_ifg_pos_tx <= 4'd0;
                        S_state_tx <= P_IFG;
                    end else
                        S_fcs_pos_tx <= S_fcs_pos_tx + 2'd1;
                end
                P_IFG: begin
                    if (S_ifg_pos_tx == 4'd11) begin
                        S_read_bin_tx <= W_read_next_tx;
                        S_read_gray_tx <= (W_read_next_tx >> 1) ^ W_read_next_tx;
                        O_tx_done <= 1'b1;
                        S_state_tx <= P_IDLE;
                    end else
                        S_ifg_pos_tx <= S_ifg_pos_tx + 4'd1;
                end
                default: S_state_tx <= P_IDLE;
            endcase
        end
    end
endmodule
