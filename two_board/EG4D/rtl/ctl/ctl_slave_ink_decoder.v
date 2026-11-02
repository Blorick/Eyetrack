`timescale 1ns/1ps
//=============================================================================
//  ctl_slave_ink_decoder -- CRC-committed raw packets to continuous ink raster
//  所属域 : ctl
//  时钟域 : I_clk_app, existing EG 75 MHz mode/application clock
//  复位   : I_rst_app_n, common assertion with packet RX domain
//  Contract: exactly ordered 720 complete 160-byte lines; no cut-through
//  Verification: full/partial frame golden bench authored; EDA pending
//=============================================================================
module ctl_slave_ink_decoder #(
    parameter integer P_WIDTH=1280,
    parameter integer P_HEIGHT=720,
    parameter integer P_FRAME_TIMEOUT_CYCLES=3000000
)(
    input wire I_clk_app,
    input wire I_rst_app_n,
    input wire I_packet_valid,
    output reg [7:0] O_packet_read_addr,
    input wire [7:0] I_packet_read_data,
    output reg O_packet_release,
    input wire I_session_ready,
    input wire [31:0] I_epoch,
    input wire I_barrier_abort,
    input wire I_sampler_frame_ready,
    input wire I_sampler_geometry_ready,
    output reg O_hello_seen,
    output reg [31:0] O_hello_epoch,
    output reg [31:0] O_hello_challenge,
    output reg O_ink,
    output reg O_ink_de,
    output reg O_vsync,
    output reg O_frame_abort,
    output reg O_detected,
    output reg [6:0] O_char_valid,
    output reg [10:0] O_char_up,
    output reg [10:0] O_char_down,
    output reg [76:0] O_char_left,
    output reg [76:0] O_char_right,
    output reg [31:0] O_external_frame_index,
    output reg O_complete_frame,
    output reg [31:0] O_bad_sequence_count,
    output wire O_raw_quiet
);
    localparam [3:0] P_IDLE=0,P_FETCH_WAIT=1,P_FETCH_CAPTURE=2,P_HEADER_CHECK=3,
        P_META_CHECK=4,P_META_READY=5,P_META_START=6,P_LINE_CHECK=7,
        P_FIRST_BYTE=8,P_UNPACK=9,P_RELEASE=10,P_RELEASE_WAIT=11,P_GEOMETRY_WAIT=12;
    reg [3:0] S_state;
    reg [7:0] S_header[0:15];
    reg [15:0] S_meta_word[0:17];
    reg [7:0] S_read_index,S_read_limit;
    reg [1:0] S_fetch_class;
    reg [15:0] S_packet_line;
    reg S_frame_active;
    reg [31:0] S_frame_epoch,S_frame_id,S_frame_timer;
    reg [15:0] S_next_line;
    reg [7:0] S_byte,S_prefetch_byte,S_byte_index;
    reg [2:0] S_bit_index;
    reg S_seen_frame;
    reg S_geometry_seen_low;
    reg [1:0] S_geometry_guard;
    reg [31:0] S_last_frame;
    integer S_init;
    wire [31:0] W_packet_epoch={S_header[4],S_header[5],S_header[6],S_header[7]};
    wire [31:0] W_packet_fid={S_header[8],S_header[9],S_header[10],S_header[11]};
    wire [15:0] W_length={S_header[12],S_header[13]};
    wire [7:0] W_kind=S_header[3];
    wire W_header_ok=S_header[0]==8'h52 && S_header[1]==8'h53 && S_header[2]==1 &&
        S_header[14]==0 && (W_kind==1 || S_header[15]==0);
    wire W_same_frame=S_frame_active && W_packet_epoch==S_frame_epoch && W_packet_fid==S_frame_id;
    reg W_geometry_ok;
    integer W_slot;
    always @(*)begin
        W_geometry_ok=S_meta_word[0]==P_WIDTH && S_meta_word[1]==P_HEIGHT;
        for(W_slot=2;W_slot<18;W_slot=W_slot+1)
            if(S_meta_word[W_slot][15:11]!=0)W_geometry_ok=0;
        if(S_header[15][0] && |S_header[15][7:1])begin
            if(S_meta_word[2]>S_meta_word[3] || S_meta_word[3]>=P_HEIGHT)W_geometry_ok=0;
            for(W_slot=0;W_slot<7;W_slot=W_slot+1)begin
                if(S_header[15][W_slot+1] &&
                    (S_meta_word[4+2*W_slot]>S_meta_word[5+2*W_slot] || S_meta_word[5+2*W_slot]>=P_WIDTH))W_geometry_ok=0;
            end
        end
    end
    assign O_raw_quiet=!S_frame_active && S_state==P_IDLE && !O_ink_de && !O_packet_release;
    task t_abort;
        begin
            O_frame_abort<=1;O_vsync<=1;O_ink_de<=0;S_frame_active<=0;
            S_next_line<=0;S_frame_timer<=0;O_bad_sequence_count<=O_bad_sequence_count+1;
        end
    endtask
    task t_read;
        input [7:0] I_index;
        begin O_packet_read_addr<=I_index;S_read_index<=I_index;S_state<=P_FETCH_WAIT;end
    endtask
    always @(posedge I_clk_app or negedge I_rst_app_n)begin
        if(!I_rst_app_n)begin
            S_state<=P_IDLE;S_read_index<=0;S_read_limit<=15;S_fetch_class<=0;
            S_packet_line<=0;S_frame_active<=0;S_frame_epoch<=0;S_frame_id<=0;S_frame_timer<=0;
            S_next_line<=0;S_byte<=0;S_prefetch_byte<=0;S_byte_index<=0;S_bit_index<=0;
            S_seen_frame<=0;S_last_frame<=0;
            S_geometry_seen_low<=0;S_geometry_guard<=0;
            O_packet_read_addr<=0;O_packet_release<=0;O_hello_seen<=0;O_hello_epoch<=0;O_hello_challenge<=0;
            O_ink<=0;O_ink_de<=0;O_vsync<=1;O_frame_abort<=0;
            O_detected<=0;O_char_valid<=0;O_char_up<=0;O_char_down<=0;O_char_left<=0;O_char_right<=0;
            O_external_frame_index<=0;O_complete_frame<=0;O_bad_sequence_count<=0;
            for(S_init=0;S_init<16;S_init=S_init+1)S_header[S_init]<=0;
            for(S_init=0;S_init<18;S_init=S_init+1)S_meta_word[S_init]<=0;
        end else begin
            O_packet_release<=0;O_hello_seen<=0;O_frame_abort<=0;O_complete_frame<=0;O_ink_de<=0;
            if(!I_session_ready)S_seen_frame<=0;
            if(S_frame_active)S_frame_timer<=S_frame_timer+1;else S_frame_timer<=0;
            // A barrier interrupts even a continuous raster beat. Offered CDC
            // words are drained externally; abort clears only unoffered work.
            if(I_barrier_abort || (!I_session_ready && (S_frame_active || S_state==P_META_READY || S_state==P_META_START || S_state==P_GEOMETRY_WAIT)))begin
                t_abort();S_seen_frame<=0;
                if(S_state!=P_IDLE && S_state!=P_RELEASE_WAIT)S_state<=P_RELEASE;
            end else if(S_frame_active && S_frame_timer>=P_FRAME_TIMEOUT_CYCLES)begin
                t_abort();if(S_state!=P_IDLE)S_state<=P_RELEASE;
            end else case(S_state)
                P_IDLE: if(I_packet_valid)begin S_fetch_class<=0;S_read_limit<=15;t_read(0);end
                P_FETCH_WAIT:S_state<=P_FETCH_CAPTURE;
                P_FETCH_CAPTURE:begin
                    if(S_fetch_class==0)S_header[S_read_index]<=I_packet_read_data;
                    else if(S_fetch_class==1)begin
                        if(!S_read_index[0])S_meta_word[(S_read_index-16)>>1][15:8]<=I_packet_read_data;
                        else S_meta_word[(S_read_index-16)>>1][7:0]<=I_packet_read_data;
                    end else if(S_fetch_class==2)begin
                        if(S_read_index==16)S_packet_line[15:8]<=I_packet_read_data;
                        else S_packet_line[7:0]<=I_packet_read_data;
                    end else S_byte<=I_packet_read_data;
                    if(S_read_index<S_read_limit)t_read(S_read_index+1);
                    else case(S_fetch_class)
                        0:S_state<=P_HEADER_CHECK;
                        1:S_state<=P_META_CHECK;
                        2:S_state<=P_LINE_CHECK;
                        3:S_state<=P_FIRST_BYTE;
                    endcase
                end
                P_HEADER_CHECK:begin
                    if(!W_header_ok)begin t_abort();S_state<=P_RELEASE;end
                    else if(W_kind==5 && W_length==0)begin
                        t_abort();S_seen_frame<=0;
                        O_hello_seen<=1;O_hello_epoch<=W_packet_epoch;O_hello_challenge<=W_packet_fid;S_state<=P_RELEASE;
                    end else if(!I_session_ready || W_packet_epoch!=I_epoch)S_state<=P_RELEASE;
                    else if(W_kind==1 && W_length==36)begin
                        if(S_frame_active)t_abort();
                        S_fetch_class<=1;S_read_limit<=51;t_read(16);
                    end else if(W_kind==2 && W_length==162 && W_same_frame)begin
                        S_fetch_class<=2;S_read_limit<=17;t_read(16);
                    end else if(W_kind==3 && W_length==0 && W_same_frame && S_next_line==P_HEIGHT)begin
                        O_vsync<=1;S_frame_active<=0;O_complete_frame<=1;S_state<=P_RELEASE;
                    end else if(W_kind==4 && W_length==0)begin
                        if(W_same_frame)t_abort();S_state<=P_RELEASE;
                    end else begin t_abort();S_state<=P_RELEASE;end
                end
                P_META_CHECK:begin
                    // Accept only increasing FIDs within an epoch; signed
                    // distance supports the normal 32-bit wrap, with no
                    // admitted old frame farther than half the sequence space.
                    if(!W_geometry_ok || (S_seen_frame && W_packet_epoch==S_frame_epoch &&
                        $signed(W_packet_fid-S_last_frame)<=0))begin t_abort();S_state<=P_RELEASE;end
                    else begin
                        O_detected<=S_header[15][0];O_char_valid<=S_header[15][7:1];
                        O_char_up<=S_meta_word[2][10:0];O_char_down<=S_meta_word[3][10:0];
                        for(S_init=0;S_init<7;S_init=S_init+1)begin
                            O_char_left[S_init*11+:11]<=S_meta_word[4+2*S_init][10:0];
                            O_char_right[S_init*11+:11]<=S_meta_word[5+2*S_init][10:0];
                        end
                        S_frame_epoch<=W_packet_epoch;S_frame_id<=W_packet_fid;
                        O_external_frame_index<=W_packet_fid;S_next_line<=0;
                        S_last_frame<=W_packet_fid;S_seen_frame<=1;S_state<=P_META_READY;
                    end
                end
                P_META_READY:if(I_sampler_frame_ready)begin
                    O_vsync<=0;S_frame_active<=1;S_frame_timer<=0;
                    S_geometry_seen_low<=0;S_geometry_guard<=0;S_state<=P_META_START;
                end
                P_META_START:begin
                    // Start preparation with this frame's actual falling VS.
                    // A prior frame's held ready level cannot admit its LINEs.
                    if(!I_sampler_geometry_ready)S_geometry_seen_low<=1;
                    S_state<=P_GEOMETRY_WAIT;
                end
                P_GEOMETRY_WAIT:begin
                    if(!I_sampler_geometry_ready)S_geometry_seen_low<=1;
                    if(S_geometry_guard<2)S_geometry_guard<=S_geometry_guard+1;
                    else if(S_geometry_seen_low && I_sampler_geometry_ready)S_state<=P_RELEASE;
                end
                P_LINE_CHECK:if(S_packet_line!=S_next_line || S_next_line>=P_HEIGHT)begin t_abort();S_state<=P_RELEASE;end
                    else begin S_fetch_class<=3;S_read_limit<=18;t_read(18);end
                P_FIRST_BYTE:begin S_byte_index<=0;S_bit_index<=0;S_state<=P_UNPACK;end
                P_UNPACK:begin
                    O_ink_de<=1;O_ink<=S_byte[7-S_bit_index];
                    // Two-edge synchronous read latency is hidden inside the
                    // current eight-pixel byte; DE never pauses for prefetch.
                    if(S_bit_index==0 && S_byte_index<(P_WIDTH/8)-1)O_packet_read_addr<=8'd19+S_byte_index;
                    if(S_bit_index==2 && S_byte_index<(P_WIDTH/8)-1)S_prefetch_byte<=I_packet_read_data;
                    if(S_bit_index==7)begin
                        S_bit_index<=0;
                        if(S_byte_index==(P_WIDTH/8)-1)begin
                            S_next_line<=S_next_line+1;S_frame_timer<=0;S_state<=P_RELEASE;
                        end else begin S_byte_index<=S_byte_index+1;S_byte<=S_prefetch_byte;end
                    end else S_bit_index<=S_bit_index+1;
                end
                P_RELEASE:begin O_packet_release<=1;S_state<=P_RELEASE_WAIT;end
                P_RELEASE_WAIT:S_state<=P_IDLE;
                default:begin t_abort();S_state<=P_RELEASE;end
            endcase
        end
    end
endmodule
