`timescale 1ns / 1ps
// Manual plate-number ROM. No pixel or character recognition takes place here.
// Characters are packed char0 at bits 7:0; province codes share the OSD font.
module dsp_key_plate_osd (
    input wire I_clk,
    input wire I_rst_n,
    input wire [4:0] I_key_code,
    input wire I_key_valid,
    input wire I_vsync,
    output reg [55:0] O_res,
    output reg [3:0] O_res_n,
    output reg O_res_valid
);
    reg S_vsync_d;
    reg [55:0] S_plate_text;
    reg [3:0] S_plate_count;
    // 2026-10-18 重映射：12 张车牌 -> KEY5..KEY16（KEY1..KEY4 不再显示）
    wire W_active = I_key_valid && (I_key_code >= 5'd5) && (I_key_code <= 5'd16);
    always @(*) begin
        S_plate_text = 56'd0;
        S_plate_count = 4'd7;
        case (I_key_code)
            5'd5:  S_plate_text = {"9","J","6","X","2","F",8'h97}; // 贵F2X6J9
            5'd6:  S_plate_text = {"2","V","6","P","8","E",8'h89}; // 黑E8P6V2
            5'd7:  S_plate_text = {"1","W","8","R","5","D",8'h88}; // 吉D5R8W1
            5'd8:  S_plate_text = {"1","Q","7","N","4","D",8'h88}; // 吉D4N7Q1
            5'd9:  S_plate_text = {"5","H","2","P","9","C",8'h84}; // 冀C9P2H5
            5'd10: S_plate_text = {"7","H","2","R","9","C",8'h85}; // 晋C9R2H7
            5'd11: S_plate_text = {"5","D","7","N","3","H",8'h85}; // 晋H3N7D5
            5'd12: S_plate_text = {"3","T","8","M","6","A",8'h80}; // 京A6M8T3
            5'd13: S_plate_text = {"2","N","7","K","4","B",8'h8f}; // 鲁B4K7N2
            5'd14: S_plate_text = {"6","L","4","Q","7","E",8'h86}; // 蒙E7Q4L6
            5'd15: S_plate_text = {"4","V","3","C","8","G",8'h9d}; // 宁G8C3V4
            5'd16: S_plate_text = {"4","B","8","S","1","J",8'h98}; // 云J1S8B4
            default: S_plate_count=4'd0;
        endcase
    end
    // Commit one tuple during VS so a press/release cannot tear a text row.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_vsync_d <= 1'b0;
            O_res <= 56'd0; O_res_n <= 4'd0; O_res_valid <= 1'b0;
        end else begin
            S_vsync_d <= I_vsync;
            if (I_vsync && !S_vsync_d) begin
                O_res <= W_active ? S_plate_text : 56'd0;
                O_res_n <= W_active ? S_plate_count : 4'd0;
                O_res_valid <= W_active;
            end
        end
    end
endmodule
