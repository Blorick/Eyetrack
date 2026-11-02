


module video_out (
    input wire        I_rst_n,

    input wire        I_ddr_clk,
    output wire       O_video_out_rd_busy,  
    input wire        I_video_in_wr_busy,   
    input wire[1:0]   I_video_out_rp,
    output wire       O_ddr_user_rd_en,
    output reg[24:0]  O_ddr_user_addr,
    input wire        I_ddr_user_ready,
    input wire        I_ddr_user_rd_valid,
    input wire[127:0] I_ddr_user_rd_data,

    input wire        I_dsi_clk,
    input wire        I_video_vsync,
    input wire        I_video_rd_en,  
    output reg[23:0]  O_vdieo_data,
    // 与 RGB 同拍的请求槽有效标志；欠流像素填黑，仍保留完整光栅坐标。
    output reg        O_video_data_valid
);

    reg         S_fifo_rst;     
    reg[7:0]    S_fifo_rst_cnt;
    wire[8:0]   S_fifo_wr_num;       
    wire        S_fifo_rd_en;        
    wire[127:0] S_fifo_rd_data;      
    reg         S_video_vsync_1d;
    reg         S_video_vsync_2d;
    reg         S_video_vsync_3d;
    wire        S_video_frame_start;   
    wire        S_ddr_rd_trig;       
    reg         S_ddr_rd_valid;      
    reg[9:0]    S_ddr_rd_cnt;        
    reg[3:0]    S_fifo_rd_cnt;       
    reg         S_video_rd_en_1d;
    reg[3:0]    S_fifo_rd_cnt_1d;     
    reg[127:0]  S_fifo_rd_data_1d;    
    	wire        S_fifo_emtpy;
    wire        S_fifo_data_valid;
    reg         S_fifo_word_valid;
    reg         S_video_underflow;
    wire        S_word_start = (S_fifo_rd_cnt_1d == 4'd0) ||
                               (S_fifo_rd_cnt_1d == 4'd5) ||
                               (S_fifo_rd_cnt_1d == 4'd10);
    wire        S_split_pixel = (S_fifo_rd_cnt_1d == 4'd5) ||
                                (S_fifo_rd_cnt_1d == 4'd10);
    wire        S_pixel_available = S_word_start ?
                    (S_fifo_data_valid && (!S_split_pixel || S_fifo_word_valid)) :
                    S_fifo_word_valid;



    localparam IMAGE_BASE_ADDR_0 = 25'd0;
    localparam IMAGE_BASE_ADDR_1 = 25'd7000000;
    localparam IMAGE_BASE_ADDR_2 = 25'd14000000;
    localparam IMAGE_BASE_ADDR_3 = 25'd21000000;


    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_vsync_3d <= 1'b0;
        end else begin
            S_video_vsync_1d <= I_video_vsync;
            S_video_vsync_2d <= S_video_vsync_1d;
            S_video_vsync_3d <= S_video_vsync_2d;
        end
    end

    assign S_video_frame_start = ~S_video_vsync_3d & S_video_vsync_2d;


    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_fifo_rst <= 1'b0;
        else
            if(S_video_frame_start)
                S_fifo_rst <= 1'b1;
            else if(S_fifo_rst_cnt == 'd10)
                S_fifo_rst <= 1'b0;
            else
                S_fifo_rst <= S_fifo_rst;
    end


    always @(posedge I_ddr_clk) begin
        if(S_fifo_rst)
            S_fifo_rst_cnt <= S_fifo_rst_cnt + 'd1;
        else
            S_fifo_rst_cnt <= 'd0;
    end

    /*
        这里需要注意fifo的剩余空间一定要大于将要写入的长度，因为ddr读出的时候会有至少40个时钟周期的延时，
        例如每次从ddr读出240个长度的数据，那么fifo的剩余空间最少要设为280，否则会引起fifo写满丢失数据。
    */
    assign S_ddr_rd_trig = (('d511 - S_fifo_wr_num) >= 'd300) && (!I_video_in_wr_busy) && (!S_ddr_rd_valid) && (!S_fifo_rst) && I_rst_n ? 1'b1 : 1'b0;

    /*
        fifo的深度是512，实际的深度是510，因此需要注意每次读出的长度避免为256，否则连续两次写入会引起fifo写满，丢失数据
    */
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_ddr_rd_valid <= 1'b0;
        else
			if(O_ddr_user_rd_en && S_ddr_rd_cnt == 'd239)
                S_ddr_rd_valid <= 1'b0;
            else if(S_ddr_rd_trig)
                S_ddr_rd_valid <= 1'b1;
            else
                S_ddr_rd_valid <= S_ddr_rd_valid;
    end

    assign O_ddr_user_rd_en = S_ddr_rd_valid & I_ddr_user_ready;

    always @(posedge I_ddr_clk) begin
        if(S_ddr_rd_valid)
            begin
                if(O_ddr_user_rd_en)
                    S_ddr_rd_cnt <= S_ddr_rd_cnt + 'd1;
                else
                    S_ddr_rd_cnt <= S_ddr_rd_cnt;
            end
        else    
            S_ddr_rd_cnt <= 'd0;
    end


    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            O_ddr_user_addr <= 'd0;
        else
            if(S_video_frame_start)
                begin
                    case(I_video_out_rp)
                        'd0: O_ddr_user_addr <= IMAGE_BASE_ADDR_0;
                        'd1: O_ddr_user_addr <= IMAGE_BASE_ADDR_1;
                        'd2: O_ddr_user_addr <= IMAGE_BASE_ADDR_2;
                        'd3: O_ddr_user_addr <= IMAGE_BASE_ADDR_3;
                    endcase
                end
            else if(O_ddr_user_rd_en)
                O_ddr_user_addr <= O_ddr_user_addr + 'd8;
            else    
                O_ddr_user_addr <= O_ddr_user_addr;
    end

    
    assign O_video_out_rd_busy = S_ddr_rd_trig | S_ddr_rd_valid;


    w128_d512_fifo U_w128_d512_fifo(
        .rst        ( S_fifo_rst | ~I_rst_n ),

        .clkw       ( I_ddr_clk           ),     
        .we         ( I_ddr_user_rd_valid ),
        .di         ( I_ddr_user_rd_data  ),
        .wrusedw    ( S_fifo_wr_num       ),
        .afull      (),
        .full_flag  (),
 
        .clkr       ( I_dsi_clk           ),
        .re         ( S_fifo_rd_en        ), 
        .dout       ( S_fifo_rd_data      ),  
        .rdusedw    (), 
        .valid      ( S_fifo_data_valid   ),
        .empty_flag ( S_fifo_emtpy        ),
        .aempty     ()
    );


   always @(posedge I_dsi_clk or negedge I_rst_n) begin
       if (!I_rst_n)
           S_fifo_rd_cnt <= 4'd0;
       else if (I_video_vsync)
           S_fifo_rd_cnt <= 4'd0;
       else if(I_video_rd_en)
           S_fifo_rd_cnt <= S_fifo_rd_cnt + 'd1;
       else
           S_fifo_rd_cnt <= 'd0;
   end

   assign S_fifo_rd_en = I_video_rd_en && (S_fifo_rd_cnt == 'd0)  ? 1'b1 :
                         I_video_rd_en && (S_fifo_rd_cnt == 'd5)  ? 1'b1 :
                         I_video_rd_en && (S_fifo_rd_cnt == 'd10) ? 1'b1 : 1'b0;

   always @(posedge I_dsi_clk or negedge I_rst_n) begin
       if (!I_rst_n) begin
           S_video_rd_en_1d <= 1'b0;
           S_fifo_rd_cnt_1d <= 4'd0;
           S_fifo_rd_data_1d <= 128'd0;
       end else begin
           S_video_rd_en_1d  <= I_video_rd_en;
           S_fifo_rd_cnt_1d  <= S_fifo_rd_cnt;
           S_fifo_rd_data_1d <= S_fifo_rd_data;
       end
   end

   // 非 show-ahead、无额外输出寄存的实际 FIFO 在读边沿后给出 word/valid。
   // 下一边沿按延迟后的请求注册 RGB 和 valid；FIFO 欠流后本帧后续槽均填黑。
   always @(posedge I_dsi_clk or negedge I_rst_n) begin
       if (!I_rst_n) begin
           O_vdieo_data <= 24'd0;
           O_video_data_valid <= 1'b0;
           S_fifo_word_valid <= 1'b0;
           S_video_underflow <= 1'b0;
       end else begin
           O_video_data_valid <= S_video_rd_en_1d;
           if (I_video_vsync) begin
               S_fifo_word_valid <= 1'b0;
               S_video_underflow <= 1'b0;
           end else if (S_video_rd_en_1d && S_word_start) begin
               S_fifo_word_valid <= S_fifo_data_valid;
               if (!S_fifo_data_valid)
                   S_video_underflow <= 1'b1;
           end
       if(S_video_rd_en_1d && !I_video_vsync &&
          !S_video_underflow && S_pixel_available)
           begin
               case(S_fifo_rd_cnt_1d)
                   'd0: O_vdieo_data  <= S_fifo_rd_data[127:104];
                   'd1: O_vdieo_data  <= S_fifo_rd_data[103:80];
                   'd2: O_vdieo_data  <= S_fifo_rd_data[79:56];
                   'd3: O_vdieo_data  <= S_fifo_rd_data[55:32];
                   'd4: O_vdieo_data  <= S_fifo_rd_data[31:8];
                   'd5: O_vdieo_data  <= {S_fifo_rd_data_1d[7:0],S_fifo_rd_data[127:112]};
                   'd6: O_vdieo_data  <= S_fifo_rd_data[111:88];
                   'd7: O_vdieo_data  <= S_fifo_rd_data[87:64];
                   'd8: O_vdieo_data  <= S_fifo_rd_data[63:40];
                   'd9: O_vdieo_data  <= S_fifo_rd_data[39:16];
                   'd10: O_vdieo_data <= {S_fifo_rd_data_1d[15:0],S_fifo_rd_data[127:120]};
                   'd11: O_vdieo_data <= S_fifo_rd_data[119:96];
                   'd12: O_vdieo_data <= S_fifo_rd_data[95:72];
                   'd13: O_vdieo_data <= S_fifo_rd_data[71:48];
                   'd14: O_vdieo_data <= S_fifo_rd_data[47:24];
                    'd15: O_vdieo_data <= S_fifo_rd_data[23:0];
                    default: O_vdieo_data <= 24'd0;
               endcase
           end
       else
            O_vdieo_data <= 'd0;
       end
   end

    
endmodule
