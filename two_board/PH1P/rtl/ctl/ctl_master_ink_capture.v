`timescale 1ns/1ps
//=============================================================================
//  ctl_master_ink_capture -- nonblocking packed-ink frame packet producer
//  所属域 : ctl
//  时钟域 : I_clk_app, same pixel domain as the original PH sampler
//  复位   : I_rst_app_n, common assertion with both TX transport domains
//  Contract: META flags={char_valid,detected}; 1280x720; bit7 first
//  Verification: isolated bank/golden/fault bench authored; EDA pending
//=============================================================================
module ctl_master_ink_capture #(
    parameter integer P_WIDTH = 1280,
    parameter integer P_HEIGHT = 720,
    parameter integer P_COMMIT_TIMEOUT_CYCLES = 64
)(
    input wire I_clk_app,
    input wire I_rst_app_n,
    input wire I_ink,
    input wire I_ink_de,
    input wire I_vsync,
    input wire I_detected,
    input wire [10:0] I_char_up,
    input wire [10:0] I_char_down,
    input wire [6:0] I_char_valid,
    input wire [76:0] I_char_left,
    input wire [76:0] I_char_right,
    input wire I_session_ready,
    input wire [31:0] I_epoch,
    input wire I_control_valid,
    input wire [7:0] I_control_kind,
    input wire [31:0] I_control_epoch,
    input wire [31:0] I_control_challenge,
    output reg O_control_ready,
    input wire [1:0] I_free_bank_mask,
    input wire I_next_commit_bank,
    input wire I_transport_quiet,
    output reg O_write_en,
    output reg O_write_bank,
    output reg [7:0] O_write_addr,
    output reg [7:0] O_write_data,
    output reg O_commit,
    output reg O_commit_bank,
    output reg [8:0] O_packet_length,
    input wire I_commit_accept,
    output wire O_session_recover,
    output reg O_eof_committed,
    output reg [31:0] O_eof_epoch,
    output reg [31:0] O_eof_frame_index,
    output reg [31:0] O_frame_drop_count,
    output wire O_source_quiet
);
    localparam [1:0] P_FREE=0, P_BUILD=1, P_READY=2, P_COMMIT_WAIT=3;
    reg [1:0] S_bank_state[0:1];
    reg [7:0] S_bank_kind[0:1];
    reg [31:0] S_bank_epoch[0:1], S_bank_fid[0:1];
    reg [15:0] S_bank_line[0:1];
    reg [7:0] S_bank_build_addr[0:1], S_bank_build_limit[0:1];
    reg S_bank_header_done[0:1], S_bank_line_done[0:1], S_bank_control[0:1];
    reg S_alloc_bank, S_commit_bank, S_commit_wait;
    reg S_parity_valid, S_recover;
    reg [31:0] S_commit_timer;
    reg S_line_bank, S_in_line, S_frame_active;
    reg [10:0] S_pixel_x;
    reg [15:0] S_line_count;
    reg [7:0] S_ink_shift;
    reg [31:0] S_frame_counter, S_frame_id, S_frame_epoch;
    reg [7:0] S_meta_flags;
    reg [10:0] S_meta_up, S_meta_down;
    reg [10:0] S_meta_left[0:6], S_meta_right[0:6];
    reg S_vsync_q, S_end_pending, S_abort_pending;
    reg S_control_latched;
    integer S_init;
    wire W_vs_fall=S_vsync_q && !I_vsync;
    wire W_vs_rise=!S_vsync_q && I_vsync;
    wire W_can_allocate=S_parity_valid && !S_recover && S_bank_state[S_alloc_bank]==P_FREE && I_free_bank_mask[S_alloc_bank];
    wire W_pixel_live=S_frame_active && I_session_ready && I_ink_de && S_in_line && !S_recover;
    wire W_packed_write=W_pixel_live && S_pixel_x[2:0]==3'd7;
    wire W_meta_build=(S_bank_state[0]==P_BUILD && S_bank_kind[0]==1) ||
        (S_bank_state[1]==P_BUILD && S_bank_kind[1]==1);
    reg W_build_valid, W_build_bank;
    always @(*) begin
        W_build_valid=0; W_build_bank=S_commit_bank;
        if(S_bank_state[S_commit_bank]==P_BUILD && !S_bank_header_done[S_commit_bank]) W_build_valid=1;
        else if(S_bank_state[~S_commit_bank]==P_BUILD && !S_bank_header_done[~S_commit_bank]) begin
            W_build_valid=1; W_build_bank=~S_commit_bank;
        end
    end
    // Source quiet does not assert for a held packet/header/commit transaction.
    assign O_source_quiet=!S_frame_active && !S_in_line && !S_end_pending && !S_abort_pending &&
        !S_commit_wait && S_bank_state[0]==P_FREE && S_bank_state[1]==P_FREE && !O_write_en;
    assign O_session_recover=S_recover;

    function [7:0] f_packet_byte;
        input I_bank;
        input [7:0] I_addr;
        integer W_slot;
        begin
            f_packet_byte=0;
            case(I_addr)
                0:f_packet_byte=8'h52;
                1:f_packet_byte=8'h53;
                2:f_packet_byte=1;
                3:f_packet_byte=S_bank_kind[I_bank];
                4:f_packet_byte=S_bank_epoch[I_bank][31:24];
                5:f_packet_byte=S_bank_epoch[I_bank][23:16];
                6:f_packet_byte=S_bank_epoch[I_bank][15:8];
                7:f_packet_byte=S_bank_epoch[I_bank][7:0];
                8:f_packet_byte=S_bank_fid[I_bank][31:24];
                9:f_packet_byte=S_bank_fid[I_bank][23:16];
                10:f_packet_byte=S_bank_fid[I_bank][15:8];
                11:f_packet_byte=S_bank_fid[I_bank][7:0];
                12:f_packet_byte=0;
                13:f_packet_byte=(S_bank_kind[I_bank]==1)?36:((S_bank_kind[I_bank]==2)?162:0);
                14:f_packet_byte=0;
                15:f_packet_byte=(S_bank_kind[I_bank]==1)?S_meta_flags:0;
                default: begin
                    if(S_bank_kind[I_bank]==1) begin
                        case(I_addr)
                            16:f_packet_byte=(P_WIDTH>>8)&255;
                            17:f_packet_byte=P_WIDTH&255;
                            18:f_packet_byte=(P_HEIGHT>>8)&255;
                            19:f_packet_byte=P_HEIGHT&255;
                            20:f_packet_byte={5'd0,S_meta_up[10:8]};
                            21:f_packet_byte=S_meta_up[7:0];
                            22:f_packet_byte={5'd0,S_meta_down[10:8]};
                            23:f_packet_byte=S_meta_down[7:0];
                            default: if(I_addr>=24 && I_addr<52) begin
                                W_slot=(I_addr-24)>>2;
                                case(I_addr[1:0])
                                    0:f_packet_byte={5'd0,S_meta_left[W_slot][10:8]};
                                    1:f_packet_byte=S_meta_left[W_slot][7:0];
                                    2:f_packet_byte={5'd0,S_meta_right[W_slot][10:8]};
                                    3:f_packet_byte=S_meta_right[W_slot][7:0];
                                endcase
                            end
                        endcase
                    end else if(S_bank_kind[I_bank]==2) begin
                        if(I_addr==16)f_packet_byte=S_bank_line[I_bank][15:8];
                        else if(I_addr==17)f_packet_byte=S_bank_line[I_bank][7:0];
                    end
                end
            endcase
        end
    endfunction
    task t_allocate;
        input [7:0] I_kind;
        input [31:0] I_packet_epoch,I_packet_fid;
        input [15:0] I_line;
        input I_control;
        begin
            S_bank_state[S_alloc_bank]<=P_BUILD;
            S_bank_kind[S_alloc_bank]<=I_kind;
            S_bank_epoch[S_alloc_bank]<=I_packet_epoch;
            S_bank_fid[S_alloc_bank]<=I_packet_fid;
            S_bank_line[S_alloc_bank]<=I_line;
            S_bank_control[S_alloc_bank]<=I_control;
            S_bank_build_addr[S_alloc_bank]<=0;
            S_bank_build_limit[S_alloc_bank]<=(I_kind==1)?52:((I_kind==2)?18:16);
            S_bank_header_done[S_alloc_bank]<=0;
            S_bank_line_done[S_alloc_bank]<=(I_kind!=2);
            S_alloc_bank<=~S_alloc_bank;
        end
    endtask
    task t_abort_frame;
        begin
            S_frame_active<=0; S_end_pending<=0;
            O_frame_drop_count<=O_frame_drop_count+1;
            if(S_in_line) begin
                // Preserve commit order by converting the reserved partial
                // LINE bank into a complete ABORT packet, never skipping it.
                S_bank_kind[S_line_bank]<=4;
                S_bank_build_addr[S_line_bank]<=0;
                S_bank_build_limit[S_line_bank]<=16;
                S_bank_header_done[S_line_bank]<=0;
                S_bank_line_done[S_line_bank]<=1;
                S_bank_state[S_line_bank]<=P_BUILD;
                S_abort_pending<=0;
            end else S_abort_pending<=1;
            S_in_line<=0; S_pixel_x<=0; S_ink_shift<=0;
        end
    endtask
    always @(posedge I_clk_app or negedge I_rst_app_n) begin
        if(!I_rst_app_n) begin
            S_alloc_bank<=0;S_commit_bank<=0;S_commit_wait<=0;
            S_parity_valid<=0;S_recover<=0;S_commit_timer<=0;
            S_line_bank<=0;S_in_line<=0;S_frame_active<=0;
            S_pixel_x<=0;S_line_count<=0;S_ink_shift<=0;
            S_frame_counter<=0;S_frame_id<=0;S_frame_epoch<=0;
            S_meta_flags<=0;S_meta_up<=0;S_meta_down<=0;
            S_vsync_q<=1;S_end_pending<=0;S_abort_pending<=0;S_control_latched<=0;
            O_write_en<=0;O_write_bank<=0;O_write_addr<=0;O_write_data<=0;
            O_commit<=0;O_commit_bank<=0;O_packet_length<=0;O_control_ready<=0;
            O_eof_committed<=0;O_eof_epoch<=0;O_eof_frame_index<=0;O_frame_drop_count<=0;
            for(S_init=0;S_init<2;S_init=S_init+1) begin
                S_bank_state[S_init]<=P_FREE;S_bank_kind[S_init]<=0;
                S_bank_epoch[S_init]<=0;S_bank_fid[S_init]<=0;S_bank_line[S_init]<=0;
                S_bank_build_addr[S_init]<=0;S_bank_build_limit[S_init]<=0;
                S_bank_header_done[S_init]<=0;S_bank_line_done[S_init]<=0;S_bank_control[S_init]<=0;
            end
            for(S_init=0;S_init<7;S_init=S_init+1)begin S_meta_left[S_init]<=0;S_meta_right[S_init]<=0;end
        end else begin
            S_vsync_q<=I_vsync;O_write_en<=0;O_commit<=0;O_control_ready<=0;O_eof_committed<=0;
            if(!I_control_valid)S_control_latched<=0;
            if(W_packed_write)begin
                O_write_en<=1;O_write_bank<=S_line_bank;
                O_write_addr<=8'd18+S_pixel_x[10:3];O_write_data<={S_ink_shift[6:0],I_ink};
            end else if(W_build_valid && !S_recover)begin
                O_write_en<=1;O_write_bank<=W_build_bank;
                O_write_addr<=S_bank_build_addr[W_build_bank];
                O_write_data<=f_packet_byte(W_build_bank,S_bank_build_addr[W_build_bank]);
                if(S_bank_build_addr[W_build_bank]+1==S_bank_build_limit[W_build_bank])begin
                    S_bank_header_done[W_build_bank]<=1;
                    if(S_bank_line_done[W_build_bank])S_bank_state[W_build_bank]<=P_READY;
                end else S_bank_build_addr[W_build_bank]<=S_bank_build_addr[W_build_bank]+1;
            end
            if(I_commit_accept && S_commit_wait && !S_recover)begin
                S_commit_wait<=0;S_bank_state[S_commit_bank]<=P_FREE;
                if(S_bank_control[S_commit_bank])O_control_ready<=1;
                if(S_bank_kind[S_commit_bank]==3)begin
                    O_eof_committed<=1;O_eof_epoch<=S_bank_epoch[S_commit_bank];O_eof_frame_index<=S_bank_fid[S_commit_bank];
                end
                S_commit_bank<=~S_commit_bank;
            end else if(!S_recover && !S_commit_wait && S_bank_state[S_commit_bank]==P_READY)begin
                O_commit<=1;O_commit_bank<=S_commit_bank;
                O_packet_length<=(S_bank_kind[S_commit_bank]==1)?52:((S_bank_kind[S_commit_bank]==2)?178:16);
                S_bank_state[S_commit_bank]<=P_COMMIT_WAIT;S_commit_wait<=1;S_commit_timer<=0;
            end
            if(S_commit_wait && !I_commit_accept && !S_recover)begin
                if(S_commit_timer>=P_COMMIT_TIMEOUT_CYCLES)begin
                    // A rejected commit cannot hold the producer forever.
                    // Keep every reserved bank intact until transport drains.
                    S_recover<=1;S_parity_valid<=0;
                    S_frame_active<=0;S_in_line<=0;
                    O_frame_drop_count<=O_frame_drop_count+1;
                end else S_commit_timer<=S_commit_timer+1;
            end
            // No source pixel can wait. Control/background allocation is
            // forbidden on a line-start beat, which reserves its own bank.
            if(!S_in_line && !(S_frame_active && I_session_ready && I_ink_de) && W_can_allocate)begin
                if(S_abort_pending)begin
                    t_allocate(4,S_frame_epoch,S_frame_id,0,0);S_abort_pending<=0;
                end else if(S_end_pending)begin
                    t_allocate(3,S_frame_epoch,S_frame_id,0,0);S_end_pending<=0;
                end else if(I_control_valid && !S_control_latched && !S_frame_active)begin
                    t_allocate(I_control_kind,I_control_epoch,I_control_challenge,0,1);S_control_latched<=1;
                end
            end
            if(W_vs_fall && !S_recover)begin
                S_frame_counter<=S_frame_counter+1;
                if(I_session_ready && !S_frame_active && !S_in_line && !S_end_pending && !S_abort_pending &&
                    !W_meta_build && !(I_control_valid && !S_control_latched) && W_can_allocate)begin
                    S_frame_id<=S_frame_counter;S_frame_epoch<=I_epoch;S_frame_active<=1;S_line_count<=0;
                    S_meta_flags<={I_char_valid,I_detected};S_meta_up<=I_char_up;S_meta_down<=I_char_down;
                    for(S_init=0;S_init<7;S_init=S_init+1)begin
                        S_meta_left[S_init]<=I_char_left[S_init*11+:11];S_meta_right[S_init]<=I_char_right[S_init*11+:11];
                    end
                    t_allocate(1,I_epoch,S_frame_counter,0,0);
                end else if(S_frame_active)t_abort_frame();
                else O_frame_drop_count<=O_frame_drop_count+1;
            end
            if(S_frame_active && !I_session_ready && !S_recover)t_abort_frame();
            else if(S_frame_active && I_ink_de && !W_vs_fall && !W_vs_rise)begin
                if(!S_in_line)begin
                    if(S_line_count<P_HEIGHT && W_can_allocate)begin
                        t_allocate(2,S_frame_epoch,S_frame_id,S_line_count,0);
                        S_line_bank<=S_alloc_bank;S_in_line<=1;S_pixel_x<=1;S_ink_shift<={7'd0,I_ink};
                    end else t_abort_frame();
                end else begin
                    S_ink_shift<={S_ink_shift[6:0],I_ink};
                    if(S_pixel_x==P_WIDTH-1)begin
                        S_in_line<=0;S_pixel_x<=0;S_line_count<=S_line_count+1;
                        S_bank_line_done[S_line_bank]<=1;
                        if(S_bank_header_done[S_line_bank])S_bank_state[S_line_bank]<=P_READY;
                    end else S_pixel_x<=S_pixel_x+1;
                end
            end else if(S_frame_active && S_in_line && !I_ink_de)t_abort_frame();
            if(W_vs_rise && S_frame_active)begin
                if(S_line_count==P_HEIGHT && !S_in_line)begin S_end_pending<=1;S_frame_active<=0;end
                else t_abort_frame();
            end
            if((!S_parity_valid || S_recover) && I_transport_quiet && I_free_bank_mask==2'b11)begin
                // This is source FSM resynchronization, never a transport
                // pointer reset. Only free, uncommitted source reservations
                // are discarded after all accepted TX ownership has drained.
                S_alloc_bank<=I_next_commit_bank;S_commit_bank<=I_next_commit_bank;
                S_commit_wait<=0;S_commit_timer<=0;S_parity_valid<=1;S_recover<=0;
                S_bank_state[0]<=P_FREE;S_bank_state[1]<=P_FREE;
                S_frame_active<=0;S_in_line<=0;S_end_pending<=0;S_abort_pending<=0;
                S_control_latched<=0;O_write_en<=0;O_commit<=0;
            end
            if(S_commit_wait && !I_commit_accept && !S_recover && S_commit_timer>=P_COMMIT_TIMEOUT_CYCLES)begin
                S_frame_active<=0;S_in_line<=0;O_write_en<=0;O_commit<=0;
            end
        end
    end
endmodule
