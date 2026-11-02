`timescale 1ns/1ps
// -----------------------------------------------------------------------------
//  所属域   : ctl
//  时钟域   : declared module clock domain(s)
//  复位     : declared asynchronous active-low reset(s)
//  参数     : frozen module parameters; no interface change
//  Contract: comment-only style clone; ports and expressions are unchanged
// Module: ctl_ink_packet_rx
// Function: CRC-commit eight bounded ink packets before application delivery
// Domain: ctl; GMII receive and independent application clock domains
// Reset: both domains must receive the same asynchronous reset assertion
// Parameters: peer/local MAC addresses and dedicated ink EtherType
// Contract: RS/v1; META36, LINE162, control0, and RESULT16 payload bytes
// Verification: independent FCS vectors, ring capacity, faults, and CDC tests
// -----------------------------------------------------------------------------
module ctl_ink_packet_rx #(
    parameter [47:0] P_LOCAL_MAC = 48'h02_00_00_65_43_21,
    parameter [47:0] P_PEER_MAC = 48'h02_00_00_12_34_56,
    parameter [15:0] P_ETHERTYPE = 16'h88B7
) (
    input wire I_clk_rx,
    input wire I_rst_rx_n,
    input wire [7:0] I_gmii_rxd,
    input wire I_gmii_rx_dv,
    input wire I_gmii_rx_er,
    input wire I_clk_app,
    input wire I_rst_app_n,
    output wire O_packet_valid,
    input wire [7:0] I_packet_read_addr,
    output reg [7:0] O_packet_read_data,
    input wire I_packet_release,
    output reg [31:0] O_bad_frame_count,
    output reg [31:0] O_full_drop_count
);

    // The writer can change only its uncommitted bank. Publishing a Gray
    // pointer follows all RAM writes and CRC checks. Reuse follows the reader
    // release pointer through two RX clocks. There is no live cut-through.
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [7:0] S_ring_mem [0:2047];
    reg [3:0] S_write_bin_rx;
    reg [3:0] S_write_gray_rx;
    reg [3:0] S_read_gray_s1_rx;
    reg [3:0] S_read_gray_s2_rx;
    reg [3:0] S_read_bin_app;
    reg [3:0] S_read_gray_app;
    reg [3:0] S_write_gray_s1_app;
    reg [3:0] S_write_gray_s2_app;

    reg S_active_rx;
    reg S_body_rx;
    reg [3:0] S_preamble_pos_rx;
    reg S_preamble_ok_rx;
    reg S_admitted_rx;
    reg S_header_ok_rx;
    reg S_ink_type_rx;
    reg S_error_rx;
    reg S_overflow_rx;
    reg [2:0] S_tail_count_rx;
    reg [7:0] S_tail0_rx;
    reg [7:0] S_tail1_rx;
    reg [7:0] S_tail2_rx;
    reg [7:0] S_tail3_rx;
    reg [10:0] S_body_pos_rx;
    reg [10:0] S_app_count_rx;
    reg [31:0] S_crc_rx;
    reg [7:0] S_magic0_rx;
    reg [7:0] S_magic1_rx;
    reg [7:0] S_version_rx;
    reg [7:0] S_kind_rx;
    reg [7:0] S_length_hi_rx;
    reg [7:0] S_length_lo_rx;
    reg [7:0] S_reserved_rx;
    reg [7:0] S_flags_rx;
    reg S_padding_ok_rx;
    reg S_contract_ok_rx;
    reg [16:0] S_wire_length_rx;

    wire W_full_rx = S_write_gray_rx ==
        {~S_read_gray_s2_rx[3:2], S_read_gray_s2_rx[1:0]};
    wire [3:0] W_write_next_rx = S_write_bin_rx + 4'd1;
    wire [3:0] W_read_next_app = S_read_bin_app + 4'd1;
    wire [15:0] W_payload_length_rx = {S_length_hi_rx, S_length_lo_rx};
    wire [16:0] W_app_length_rx = 17'd16 + {1'b0, W_payload_length_rx};
    wire W_contract_rx = S_magic0_rx == 8'h52 &&
        S_magic1_rx == 8'h53 && S_version_rx == 8'd1 &&
        S_reserved_rx == 8'd0 &&
        ((S_kind_rx == 8'd1 && W_payload_length_rx == 16'd36) ||
            (S_kind_rx == 8'd2 && W_payload_length_rx == 16'd162 &&
            S_flags_rx == 8'd0) ||
            ((S_kind_rx >= 8'd3 && S_kind_rx <= 8'd6) &&
            W_payload_length_rx == 16'd0 && S_flags_rx == 8'd0) ||
            (S_kind_rx == 8'd7 && W_payload_length_rx == 16'd16 &&
            S_flags_rx == 8'd0));
    wire W_body_pop_rx = I_gmii_rx_dv && S_active_rx &&
        S_body_rx && S_tail_count_rx == 3'd4;
    wire W_store_rx = W_body_pop_rx && S_body_pos_rx >= 11'd14 &&
        S_admitted_rx && S_header_ok_rx && S_app_count_rx < 11'd256;
    wire W_complete_rx = S_preamble_ok_rx && S_header_ok_rx &&
        !S_error_rx && !S_overflow_rx && S_tail_count_rx == 3'd4 &&
        S_contract_ok_rx && S_padding_ok_rx &&
        {6'd0, S_app_count_rx} == S_wire_length_rx &&
        (~S_crc_rx) == {S_tail3_rx, S_tail2_rx, S_tail1_rx, S_tail0_rx};

    assign O_packet_valid = S_read_gray_app != S_write_gray_s2_app;

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

    // Neither memory port is reset, allowing true independent-clock ERAM.
    always @(posedge I_clk_rx) begin
        if (W_store_rx)
            S_ring_mem[{S_write_bin_rx[2:0], S_app_count_rx[7:0]}] <= S_tail0_rx;
    end
    always @(posedge I_clk_app) begin
        O_packet_read_data <= S_ring_mem[{S_read_bin_app[2:0], I_packet_read_addr}];
    end

    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if (!I_rst_app_n) begin
            S_read_bin_app <= 4'd0;
            S_read_gray_app <= 4'd0;
            S_write_gray_s1_app <= 4'd0;
            S_write_gray_s2_app <= 4'd0;
        end else begin
            S_write_gray_s1_app <= S_write_gray_rx;
            S_write_gray_s2_app <= S_write_gray_s1_app;
            if (I_packet_release && O_packet_valid) begin
                S_read_bin_app <= W_read_next_app;
                S_read_gray_app <= (W_read_next_app >> 1) ^ W_read_next_app;
            end
        end
    end

    always @(posedge I_clk_rx or negedge I_rst_rx_n) begin
        if (!I_rst_rx_n) begin
            S_write_bin_rx <= 4'd0;
            S_write_gray_rx <= 4'd0;
            S_read_gray_s1_rx <= 4'd0;
            S_read_gray_s2_rx <= 4'd0;
            S_active_rx <= 1'b0;
            S_body_rx <= 1'b0;
            S_preamble_pos_rx <= 4'd0;
            S_preamble_ok_rx <= 1'b0;
            S_admitted_rx <= 1'b0;
            S_header_ok_rx <= 1'b0;
            S_ink_type_rx <= 1'b0;
            S_error_rx <= 1'b0;
            S_overflow_rx <= 1'b0;
            S_tail_count_rx <= 3'd0;
            S_tail0_rx <= 8'd0;
            S_tail1_rx <= 8'd0;
            S_tail2_rx <= 8'd0;
            S_tail3_rx <= 8'd0;
            S_body_pos_rx <= 11'd0;
            S_app_count_rx <= 11'd0;
            S_crc_rx <= 32'hFFFF_FFFF;
            S_magic0_rx <= 8'd0;
            S_magic1_rx <= 8'd0;
            S_version_rx <= 8'd0;
            S_kind_rx <= 8'd0;
            S_length_hi_rx <= 8'd0;
            S_length_lo_rx <= 8'd0;
            S_reserved_rx <= 8'd0;
            S_flags_rx <= 8'd0;
            S_padding_ok_rx <= 1'b1;
            S_contract_ok_rx <= 1'b0;
            S_wire_length_rx <= 17'd46;
            O_bad_frame_count <= 32'd0;
            O_full_drop_count <= 32'd0;
        end else begin
            S_read_gray_s1_rx <= S_read_gray_app;
            S_read_gray_s2_rx <= S_read_gray_s1_rx;
            S_contract_ok_rx <= W_contract_rx;
            S_wire_length_rx <= (W_app_length_rx < 17'd46) ?
                17'd46 : W_app_length_rx;
            if (I_gmii_rx_dv) begin
                if (!S_active_rx) begin
                    S_active_rx <= 1'b1;
                    S_body_rx <= 1'b0;
                    S_preamble_pos_rx <= 4'd1;
                    S_preamble_ok_rx <= I_gmii_rxd == 8'h55;
                    S_admitted_rx <= !W_full_rx;
                    S_header_ok_rx <= 1'b1;
                    S_ink_type_rx <= 1'b0;
                    S_error_rx <= I_gmii_rx_er;
                    S_overflow_rx <= 1'b0;
                    S_tail_count_rx <= 3'd0;
                    S_body_pos_rx <= 11'd0;
                    S_app_count_rx <= 11'd0;
                    S_crc_rx <= 32'hFFFF_FFFF;
                    S_magic0_rx <= 8'd0;
                    S_magic1_rx <= 8'd0;
                    S_version_rx <= 8'd0;
                    S_kind_rx <= 8'd0;
                    S_length_hi_rx <= 8'd0;
                    S_length_lo_rx <= 8'd0;
                    S_reserved_rx <= 8'd0;
                    S_flags_rx <= 8'd0;
                    S_padding_ok_rx <= 1'b1;
                    S_contract_ok_rx <= 1'b0;
                    S_wire_length_rx <= 17'd46;
                end else if (!S_body_rx) begin
                    S_error_rx <= S_error_rx || I_gmii_rx_er;
                    if (S_preamble_pos_rx < 4'd7) begin
                        S_preamble_ok_rx <= S_preamble_ok_rx && I_gmii_rxd == 8'h55;
                        S_preamble_pos_rx <= S_preamble_pos_rx + 4'd1;
                    end else begin
                        S_preamble_ok_rx <= S_preamble_ok_rx && I_gmii_rxd == 8'hD5;
                        S_body_rx <= 1'b1;
                    end
                end else begin
                    S_error_rx <= S_error_rx || I_gmii_rx_er;
                    if (S_tail_count_rx < 3'd4) begin
                        case (S_tail_count_rx)
                            3'd0: S_tail0_rx <= I_gmii_rxd;
                            3'd1: S_tail1_rx <= I_gmii_rxd;
                            3'd2: S_tail2_rx <= I_gmii_rxd;
                            default: S_tail3_rx <= I_gmii_rxd;
                        endcase
                        S_tail_count_rx <= S_tail_count_rx + 3'd1;
                    end else begin
                        S_tail0_rx <= S_tail1_rx;
                        S_tail1_rx <= S_tail2_rx;
                        S_tail2_rx <= S_tail3_rx;
                        S_tail3_rx <= I_gmii_rxd;
                        S_crc_rx <= f_crc_byte(S_crc_rx, S_tail0_rx);
                        if (S_body_pos_rx < 11'd14) begin
                            if (S_body_pos_rx < 11'd6)
                                S_header_ok_rx <= S_header_ok_rx &&
                                    S_tail0_rx == f_mac_byte(P_LOCAL_MAC, S_body_pos_rx[3:0]);
                            else if (S_body_pos_rx < 11'd12)
                                S_header_ok_rx <= S_header_ok_rx &&
                                    S_tail0_rx == f_mac_byte(P_PEER_MAC, S_body_pos_rx[3:0] - 4'd6);
                            else if (S_body_pos_rx == 11'd12) begin
                                S_header_ok_rx <= S_header_ok_rx && S_tail0_rx == P_ETHERTYPE[15:8];
                                S_ink_type_rx <= S_tail0_rx == P_ETHERTYPE[15:8];
                            end else begin
                                S_header_ok_rx <= S_header_ok_rx && S_tail0_rx == P_ETHERTYPE[7:0];
                                S_ink_type_rx <= S_ink_type_rx && S_tail0_rx == P_ETHERTYPE[7:0];
                            end
                        end else begin
                            case (S_app_count_rx)
                                11'd0: S_magic0_rx <= S_tail0_rx;
                                11'd1: S_magic1_rx <= S_tail0_rx;
                                11'd2: S_version_rx <= S_tail0_rx;
                                11'd3: S_kind_rx <= S_tail0_rx;
                                11'd12: S_length_hi_rx <= S_tail0_rx;
                                11'd13: S_length_lo_rx <= S_tail0_rx;
                                11'd14: S_reserved_rx <= S_tail0_rx;
                                11'd15: S_flags_rx <= S_tail0_rx;
                                default: begin
                                end
                            endcase
                            if (S_app_count_rx >= 11'd16 &&
                                {6'd0, S_app_count_rx} >= W_app_length_rx && S_tail0_rx != 8'd0)
                                S_padding_ok_rx <= 1'b0;
                            if (S_app_count_rx >= 11'd256)
                                S_overflow_rx <= 1'b1;
                            if (S_app_count_rx != 11'h7FF)
                                S_app_count_rx <= S_app_count_rx + 11'd1;
                            else
                                S_overflow_rx <= 1'b1;
                        end
                        if (S_body_pos_rx != 11'h7FF)
                            S_body_pos_rx <= S_body_pos_rx + 11'd1;
                        else
                            S_overflow_rx <= 1'b1;
                    end
                end
            end else if (S_active_rx) begin
                S_active_rx <= 1'b0;
                S_body_rx <= 1'b0;
                if (S_ink_type_rx) begin
                    if (!W_complete_rx)
                        O_bad_frame_count <= O_bad_frame_count + 32'd1;
                    else if (!S_admitted_rx)
                        O_full_drop_count <= O_full_drop_count + 32'd1;
                    else begin
                        S_write_bin_rx <= W_write_next_rx;
                        S_write_gray_rx <= (W_write_next_rx >> 1) ^ W_write_next_rx;
                    end
                end
            end
        end
    end
endmodule
