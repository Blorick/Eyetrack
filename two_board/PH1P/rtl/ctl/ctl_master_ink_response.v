`timescale 1ns/1ps
//=============================================================================
//  ctl_master_ink_response -- committed EOF ownership and recognition decoder
//  所属域 : ctl
//  时钟域 : I_clk_app, PH 75 MHz application domain
//  复位   : I_rst_app_n, asynchronous active low
//  Contract: CRC-committed packet RAM, synchronous read, barrier always drains
//  RESULT: byte23={valid,3'b0,count}; byte24={1'b0,raw_valid}
//=============================================================================
module ctl_master_ink_response (
    input wire I_clk_app,
    input wire I_rst_app_n,
    input wire I_barrier,
    input wire I_packet_valid,
    output wire [7:0] O_packet_read_addr,
    input wire [7:0] I_packet_read_data,
    output wire O_packet_release,
    input wire I_eof_committed,
    input wire [31:0] I_eof_epoch,
    input wire [31:0] I_eof_frame_index,
    output wire O_ack_seen,
    output wire [31:0] O_ack_epoch,
    output wire [31:0] O_ack_challenge,
    output wire O_peer_hello,
    output wire [55:0] O_res,
    output wire [3:0] O_res_n,
    output wire O_res_valid,
    output wire O_frame_result,
    output wire [55:0] O_raw_code,
    output wire [6:0] O_raw_valid,
    output wire [31:0] O_rejected_result_count,
    output wire O_rx_quiet
);
    localparam [2:0] ST_IDLE=0, ST_WAIT=1, ST_CAPTURE=2, ST_CHECK=3,
        ST_RELEASE=4, ST_RELEASE_WAIT=5;
    reg [2:0] S_state,S_next_state;
    reg [7:0] S_read_addr;
    reg [7:0] S_bytes[0:31];
    reg [31:0] S_recent_epoch,S_recent_frame_index,S_reject_count;
    reg S_recent_valid,S_result_seen;
    reg S_ack_seen,S_peer_hello,S_frame_result,S_res_valid;
    reg [31:0] S_ack_epoch,S_ack_challenge;
    reg [55:0] S_res,S_raw_code;
    reg [3:0] S_res_n;
    reg [6:0] S_raw_valid;
    integer S_init;
    wire [31:0] W_epoch={S_bytes[4],S_bytes[5],S_bytes[6],S_bytes[7]};
    wire [31:0] W_frame_index={S_bytes[8],S_bytes[9],S_bytes[10],S_bytes[11]};
    wire W_header_ok=S_bytes[0]==8'h52 && S_bytes[1]==8'h53 && S_bytes[2]==1 &&
        S_bytes[12]==0 && S_bytes[14]==0 && S_bytes[15]==0;
    wire W_result_shape=S_bytes[3]==7 && S_bytes[13]==16 &&
        S_bytes[23][6:4]==0 && S_bytes[23][3:0]<=7 && !S_bytes[24][7];
    wire W_result_owned=S_recent_valid && !S_result_seen &&
        W_epoch==S_recent_epoch && W_frame_index==S_recent_frame_index;
    assign O_packet_read_addr=S_read_addr;
    assign O_packet_release=S_state==ST_RELEASE;
    assign O_ack_seen=S_ack_seen;
    assign O_ack_epoch=S_ack_epoch;
    assign O_ack_challenge=S_ack_challenge;
    assign O_peer_hello=S_peer_hello;
    assign O_res=S_res;
    assign O_res_n=S_res_n;
    assign O_res_valid=S_res_valid;
    assign O_frame_result=S_frame_result;
    assign O_raw_code=S_raw_code;
    assign O_raw_valid=S_raw_valid;
    assign O_rejected_result_count=S_reject_count;
    assign O_rx_quiet=S_state==ST_IDLE && !I_packet_valid;
    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if(!I_rst_app_n) S_state<=ST_IDLE;
        else S_state<=S_next_state;
    end
    always @(*) begin
        S_next_state=S_state;
        case(S_state)
            ST_IDLE:if(I_packet_valid)S_next_state=ST_WAIT;
            ST_WAIT:S_next_state=ST_CAPTURE;
            ST_CAPTURE:if(S_read_addr==31)S_next_state=ST_CHECK;else S_next_state=ST_WAIT;
            ST_CHECK:S_next_state=ST_RELEASE;
            ST_RELEASE:S_next_state=ST_RELEASE_WAIT;
            ST_RELEASE_WAIT:S_next_state=ST_IDLE;
            default:S_next_state=ST_IDLE;
        endcase
    end
    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if(!I_rst_app_n) begin
            S_read_addr<=0;S_recent_epoch<=0;S_recent_frame_index<=0;
            S_recent_valid<=0;S_result_seen<=0;S_ack_seen<=0;S_peer_hello<=0;
            S_ack_epoch<=0;S_ack_challenge<=0;S_res<=0;S_res_n<=0;
            S_res_valid<=0;S_frame_result<=0;S_raw_code<=0;S_raw_valid<=0;S_reject_count<=0;
            for(S_init=0;S_init<32;S_init=S_init+1)S_bytes[S_init]<=0;
        end else begin
            S_ack_seen<=0;S_peer_hello<=0;S_frame_result<=0;
            if(I_eof_committed && !I_barrier) begin
                S_recent_epoch<=I_eof_epoch;S_recent_frame_index<=I_eof_frame_index;
                S_recent_valid<=1;S_result_seen<=0;
            end
            case(S_state)
                ST_IDLE:S_read_addr<=0;
                ST_CAPTURE:begin
                    S_bytes[S_read_addr]<=I_packet_read_data;
                    if(S_read_addr!=31)S_read_addr<=S_read_addr+1;
                end
                ST_CHECK:if(W_header_ok) begin
                    if(S_bytes[3]==6 && S_bytes[13]==0) begin
                        S_ack_seen<=1;S_ack_epoch<=W_epoch;S_ack_challenge<=W_frame_index;
                    end else if(S_bytes[3]==5 && S_bytes[13]==0)S_peer_hello<=1;
                    else if(S_bytes[3]==7) begin
                        if(W_result_shape && W_result_owned && !I_barrier && !I_eof_committed)begin
                            S_res<={S_bytes[22],S_bytes[21],S_bytes[20],S_bytes[19],S_bytes[18],S_bytes[17],S_bytes[16]};
                            S_res_n<=S_bytes[23][3:0];S_res_valid<=S_bytes[23][7];
                            S_raw_valid<=S_bytes[24][6:0];
                            S_raw_code<={S_bytes[31],S_bytes[30],S_bytes[29],S_bytes[28],S_bytes[27],S_bytes[26],S_bytes[25]};
                            S_frame_result<=1;S_result_seen<=1;
                        end else S_reject_count<=S_reject_count+1;
                    end
                end
                default:begin end
            endcase
            if(I_barrier)begin
                S_recent_valid<=0;S_result_seen<=0;S_res_valid<=0;S_res_n<=0;
                S_res<=0;S_raw_valid<=0;S_raw_code<=0;S_frame_result<=0;
            end
        end
    end
endmodule
