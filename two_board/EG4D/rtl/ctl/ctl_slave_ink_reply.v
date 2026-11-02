`timescale 1ns/1ps
//=============================================================================
//  ctl_slave_ink_reply -- control-priority tagged recognition packet builder
//  所属域 : ctl
//  时钟域 : I_clk_app, EG 75 MHz application domain
//  复位   : I_rst_app_n, asynchronous active low
//  Contract: TX parity is exported by transport; ready means commit accepted
//  Cancellation: HELLO discards uncommitted replies; committed banks drain
//=============================================================================
module ctl_slave_ink_reply #(
    parameter integer P_COMMIT_TIMEOUT_CYCLES=64
)(
    input wire I_clk_app,
    input wire I_rst_app_n,
    input wire I_barrier,
    input wire I_cancel,
    input wire I_control_valid,
    output wire O_control_ready,
    input wire [7:0] I_control_kind,
    input wire [31:0] I_control_epoch,
    input wire [31:0] I_control_challenge,
    input wire I_result_valid,
    output wire O_result_ready,
    input wire [188:0] I_result_data,
    input wire [1:0] I_free_bank_mask,
    input wire I_next_commit_bank,
    output wire O_write_en,
    output wire O_write_bank,
    output wire [7:0] O_write_addr,
    output wire [7:0] O_write_data,
    output wire O_commit,
    output wire O_commit_bank,
    output wire [8:0] O_packet_length,
    input wire I_commit_accept,
    input wire I_transport_quiet,
    output wire O_session_recover,
    output wire O_source_quiet
);
    localparam [2:0] ST_IDLE=0, ST_WRITE=1, ST_COMMIT=2, ST_WAIT=3,
        ST_RELEASE=4, ST_RECOVER=5;
    reg [2:0] S_state,S_next_state;
    reg S_bank,S_control;
    reg [7:0] S_kind,S_addr;
    reg [31:0] S_epoch,S_frame_index,S_timer;
    reg [188:0] S_result;
    reg [7:0] W_packet_byte;
    wire W_free=I_free_bank_mask[I_next_commit_bank];
    wire W_cancel=I_cancel || (I_barrier && !S_control);
    assign O_control_ready=S_state==ST_WAIT && I_commit_accept && S_control && !I_cancel;
    assign O_result_ready=I_barrier || (S_state==ST_IDLE && !I_control_valid && W_free && !I_cancel);
    assign O_write_en=S_state==ST_WRITE && !W_cancel;
    assign O_write_bank=S_bank;
    assign O_write_addr=S_addr;
    assign O_write_data=W_packet_byte;
    assign O_commit=S_state==ST_COMMIT && !W_cancel;
    assign O_commit_bank=S_bank;
    assign O_packet_length=S_control ? 9'd16 : 9'd32;
    assign O_session_recover=S_state==ST_RECOVER;
    assign O_source_quiet=S_state==ST_IDLE && !O_write_en && !O_commit;
    // Fixed byte lanes keep ASCII ordering identical to the original matcher.
    always @(*)begin
        W_packet_byte=0;
        case(S_addr)
            0:W_packet_byte=8'h52;1:W_packet_byte=8'h53;2:W_packet_byte=1;
            3:W_packet_byte=S_kind;
            4:W_packet_byte=S_epoch[31:24];5:W_packet_byte=S_epoch[23:16];
            6:W_packet_byte=S_epoch[15:8];7:W_packet_byte=S_epoch[7:0];
            8:W_packet_byte=S_frame_index[31:24];9:W_packet_byte=S_frame_index[23:16];
            10:W_packet_byte=S_frame_index[15:8];11:W_packet_byte=S_frame_index[7:0];
            13:W_packet_byte=S_control ? 0 : 16;
            16:W_packet_byte=S_result[76:69];17:W_packet_byte=S_result[84:77];
            18:W_packet_byte=S_result[92:85];19:W_packet_byte=S_result[100:93];
            20:W_packet_byte=S_result[108:101];21:W_packet_byte=S_result[116:109];
            22:W_packet_byte=S_result[124:117];
            23:W_packet_byte={S_result[64],3'b000,S_result[68:65]};
            24:W_packet_byte={1'b0,S_result[6:0]};
            25:W_packet_byte=S_result[14:7];26:W_packet_byte=S_result[22:15];
            27:W_packet_byte=S_result[30:23];28:W_packet_byte=S_result[38:31];
            29:W_packet_byte=S_result[46:39];30:W_packet_byte=S_result[54:47];
            31:W_packet_byte=S_result[62:55];
            default:begin end
        endcase
    end
    always @(posedge I_clk_app or negedge I_rst_app_n)begin
        if(!I_rst_app_n)S_state<=ST_IDLE;
        else S_state<=S_next_state;
    end
    always @(*)begin
        S_next_state=S_state;
        case(S_state)
            ST_IDLE:if(!I_cancel && W_free && (I_control_valid || (I_result_valid && !I_barrier)))S_next_state=ST_WRITE;
            ST_WRITE:if(W_cancel)S_next_state=ST_IDLE;
                else if(S_addr==(S_control?15:31))S_next_state=ST_COMMIT;
            ST_COMMIT:if(W_cancel)S_next_state=ST_IDLE;else S_next_state=ST_WAIT;
            ST_WAIT:if(I_commit_accept)S_next_state=ST_RELEASE;
                else if(S_timer>=P_COMMIT_TIMEOUT_CYCLES)S_next_state=ST_RECOVER;
            ST_RELEASE:S_next_state=ST_IDLE;
            ST_RECOVER:if(I_transport_quiet)S_next_state=ST_IDLE;
            default:S_next_state=ST_IDLE;
        endcase
    end
    always @(posedge I_clk_app or negedge I_rst_app_n)begin
        if(!I_rst_app_n)begin
            S_bank<=0;S_control<=0;S_kind<=0;S_addr<=0;
            S_epoch<=0;S_frame_index<=0;S_result<=0;S_timer<=0;
        end else begin
            if(S_state==ST_IDLE)begin
                S_addr<=0;S_timer<=0;
                if(!I_cancel && W_free)begin
                    S_bank<=I_next_commit_bank;S_control<=I_control_valid;
                    if(I_control_valid)begin
                        S_kind<=I_control_kind;S_epoch<=I_control_epoch;S_frame_index<=I_control_challenge;
                    end else if(I_result_valid && !I_barrier)begin
                        S_kind<=7;S_result<=I_result_data;
                        S_epoch<=I_result_data[188:157];S_frame_index<=I_result_data[156:125];
                    end
                end
            end
            if(S_state==ST_WRITE && !W_cancel)S_addr<=S_addr+1;
            if(S_state==ST_WAIT)S_timer<=S_timer+1;
        end
    end
endmodule
