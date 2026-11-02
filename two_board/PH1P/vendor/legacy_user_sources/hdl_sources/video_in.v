

module video_in (
    input wire         I_rst_n,

    input wire         I_camera_clk,
    input wire         I_camera_frame_start,
    input wire         I_camera_valid,
    input wire[127:0]  I_camera_data,
    input wire         I_mipi_rx_error,

    input wire         I_ddr_clk,

    input wire         I_display_pause,
    input wire         I_video_out_rd_busy,
    output wire        O_video_in_wr_busy,
    output reg[1:0]    O_video_out_rp,     ///读指针

    output reg         O_ddr_user_wr_en,
    output reg[24:0]   O_ddr_user_addr,
    output wire[127:0] O_ddr_user_wr_data,
    input wire         I_ddr_user_ready
);

    wire        S_fifo_rst;
    reg         S_camera_frame_start_extend; ///脉冲展宽，同步到ddr_clk时钟域
    reg[3:0]    S_extend_cnt;
    reg         S_camera_frame_start_extend_1d;
    reg         S_camera_frame_start_extend_2d;
    reg         S_camera_frame_start_extend_3d;
    wire        S_frame_start;
    reg[1:0]    S_video_in_wp;       ///写指针
    wire[8:0]   S_fifo_rd_num;      
    wire        S_fifo_rd_en;   
    wire        S_fifo_rd_valid;
    wire[127:0] S_fifo_rd_data; 
    reg         S_ddr_wr_valid;
    reg[9:0]    S_ddr_wr_cnt;
    // SOF 和首字可以同拍。摄像头无 ready，先缓存 FIFO 复位期间的前缀字。
    reg         S_camera_fifo_rst;
    reg [3:0]   S_camera_fifo_rst_cnt;
    reg [2:0]   S_camera_fifo_release;
    reg [127:0] S_camera_prefix [0:15];
    reg [3:0]   S_camera_prefix_wr;
    reg [3:0]   S_camera_prefix_rd;
    reg [4:0]   S_camera_prefix_count;
    reg         S_camera_prefix_overflow;
    reg [127:0] S_camera_front_data;
    reg         S_camera_front_valid;
    wire        S_fifo_full;
    wire        S_camera_fifo_write;
    wire        S_camera_prefix_accept;
    wire        S_camera_prefix_fetch;

    localparam IMAGE_BASE_ADDR_0 = 25'd0;
    localparam IMAGE_BASE_ADDR_1 = 25'd7000000;
    localparam IMAGE_BASE_ADDR_2 = 25'd14000000;
    localparam IMAGE_BASE_ADDR_3 = 25'd21000000;


    // 复位由输入 SOF 所在域立即发起；DDR 同步事件仅用于帧地址/指针。
    assign S_fifo_rst = S_camera_fifo_rst | ~I_rst_n;
    assign S_camera_fifo_write = S_camera_front_valid &&
        !S_camera_fifo_release[2] && !S_fifo_rst && !S_fifo_full &&
        !I_camera_frame_start;
    assign S_camera_prefix_accept = I_camera_valid &&
        ((S_camera_prefix_count < 16) || S_camera_prefix_fetch);
    assign S_camera_prefix_fetch = (S_camera_prefix_count != 0) &&
        (!S_camera_front_valid || S_camera_fifo_write) && !I_camera_frame_start;

    always @(posedge I_camera_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_camera_fifo_rst <= 1'b1;
            S_camera_fifo_rst_cnt <= 4'd0;
            S_camera_fifo_release <= 3'b111;
        end else begin
            if (I_camera_frame_start) begin
                S_camera_fifo_rst <= 1'b1;
                S_camera_fifo_rst_cnt <= 4'd0;
            end else if (S_camera_fifo_rst) begin
                if (S_camera_fifo_rst_cnt == 4'd7)
                    S_camera_fifo_rst <= 1'b0;
                else
                    S_camera_fifo_rst_cnt <= S_camera_fifo_rst_cnt + 1'b1;
            end
            // 实际 FIFO 内部两拍同步释放，再额外保留一拍写入裕量。
            if (S_fifo_rst)
                S_camera_fifo_release <= 3'b111;
            else
                S_camera_fifo_release <= {S_camera_fifo_release[1:0], 1'b0};
        end
    end

    always @(posedge I_camera_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_camera_prefix_wr <= 4'd0;
            S_camera_prefix_rd <= 4'd0;
            S_camera_prefix_count <= 5'd0;
            S_camera_prefix_overflow <= 1'b0;
            S_camera_front_data <= 128'd0;
            S_camera_front_valid <= 1'b0;
        end else if (I_camera_frame_start) begin
            S_camera_prefix_wr <= I_camera_valid ? 4'd1 : 4'd0;
            S_camera_prefix_rd <= 4'd0;
            S_camera_prefix_count <= I_camera_valid ? 5'd1 : 5'd0;
            S_camera_prefix_overflow <= 1'b0;
            S_camera_front_valid <= 1'b0;
            if (I_camera_valid)
                S_camera_prefix[0] <= I_camera_data;
        end else begin
            if (S_camera_prefix_accept) begin
                S_camera_prefix[S_camera_prefix_wr] <= I_camera_data;
                S_camera_prefix_wr <= S_camera_prefix_wr + 1'b1;
            end else if (I_camera_valid) begin
                S_camera_prefix_overflow <= 1'b1;
            end
            // RAM 读数据先同步注册；与入队写使用不同地址，首字等待FIFO释放。
            if (S_camera_prefix_fetch) begin
                S_camera_front_data <= S_camera_prefix[S_camera_prefix_rd];
                S_camera_front_valid <= 1'b1;
                S_camera_prefix_rd <= S_camera_prefix_rd + 1'b1;
            end else if (S_camera_fifo_write) begin
                S_camera_front_valid <= 1'b0;
            end
            case ({S_camera_prefix_accept, S_camera_prefix_fetch})
                2'b10: S_camera_prefix_count <= S_camera_prefix_count + 1'b1;
                2'b01: S_camera_prefix_count <= S_camera_prefix_count - 1'b1;
                default: S_camera_prefix_count <= S_camera_prefix_count;
            endcase
        end
    end

    // 以完整DDR写突发的占用标志仲裁，避免连续视频写入长期阻塞读通道。
    assign O_video_in_wr_busy = S_ddr_wr_valid | O_ddr_user_wr_en;

    w128_d512_fifo U_w128_d512_fifo(
        .rst        ( S_fifo_rst         ),   

        .clkw       ( I_camera_clk       ),     
        .we         ( S_camera_fifo_write ),
        .di         ( S_camera_front_data ),
        .afull      (),
        .full_flag  ( S_fifo_full        ),
        .wrusedw    (),
 
        .clkr       ( I_ddr_clk          ),
        .re         ( S_fifo_rd_en       ), 
        .dout       ( O_ddr_user_wr_data ),  
        .rdusedw    ( S_fifo_rd_num      ),
        .valid      (), 
        .empty_flag (),
        .aempty     ()
    );


    always @(posedge I_camera_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_camera_frame_start_extend <= 1'b0;
        else
            if(I_camera_frame_start)
                S_camera_frame_start_extend <= 1'b1;
            else if(S_extend_cnt == 'd7)
                S_camera_frame_start_extend <= 1'b0;
            else
                S_camera_frame_start_extend <= S_camera_frame_start_extend;
    end

	
    always @ (posedge I_camera_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            S_extend_cnt <= 4'd0;
    	else if(S_camera_frame_start_extend)
        	S_extend_cnt <= S_extend_cnt + 'd1;
      	else
          	S_extend_cnt <= 'd0;
    end


    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_camera_frame_start_extend_1d <= 1'b0;
            S_camera_frame_start_extend_2d <= 1'b0;
            S_camera_frame_start_extend_3d <= 1'b0;
        end else begin
            S_camera_frame_start_extend_1d <= S_camera_frame_start_extend;
            S_camera_frame_start_extend_2d <= S_camera_frame_start_extend_1d;
            S_camera_frame_start_extend_3d <= S_camera_frame_start_extend_2d;
        end
    end

    assign S_frame_start = ~S_camera_frame_start_extend_3d & S_camera_frame_start_extend_2d;

	/*
		如果前一帧图像有误少行了，那么地址回退为之前的，覆盖有问题的那帧图像
	*/
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_video_in_wp <= 'd0;
        else
            if(I_display_pause)
                S_video_in_wp <= S_video_in_wp;
            else if(S_frame_start)
                begin
                    if(I_mipi_rx_error)
                        S_video_in_wp <= S_video_in_wp;
                    else
                        S_video_in_wp <= S_video_in_wp + 'd1;
                end
            else
                S_video_in_wp <= S_video_in_wp;
    end


    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            O_video_out_rp <= 'd0;
        else
            if(I_display_pause)
                O_video_out_rp <= O_video_out_rp;
            else if(S_frame_start)
                begin
                    case(S_video_in_wp)
                        'd0: O_video_out_rp <= 'd2;
                        'd1: O_video_out_rp <= 'd3;
                        'd2: O_video_out_rp <= 'd0;
                        'd3: O_video_out_rp <= 'd1;
                    endcase
                end
            else
                O_video_out_rp <= O_video_out_rp;
    end

    /*
        fifo的深度是512，实际的深度是510，因此需要注意每次读出的长度避免为256，否则连续两次读写会引起fifo读空，丢失数据
    */
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            S_ddr_wr_valid <= 1'b0;
        else
            if(S_ddr_wr_valid && I_ddr_user_ready && S_ddr_wr_cnt == 'd239)
                S_ddr_wr_valid <= 1'b0;
            else if(S_fifo_rd_num >= 'd240 && (!I_video_out_rd_busy) && (!S_fifo_rst))
                S_ddr_wr_valid <= 1'b1;
            else
                S_ddr_wr_valid <= S_ddr_wr_valid;
    end

    always @(posedge I_ddr_clk) begin
        if(S_ddr_wr_valid)
            begin
                if(I_ddr_user_ready)
                    S_ddr_wr_cnt <= S_ddr_wr_cnt + 'd1;
                else
                    S_ddr_wr_cnt <= S_ddr_wr_cnt;
            end
        else
            S_ddr_wr_cnt <= 'd0;
    end

    assign S_fifo_rd_en = S_ddr_wr_valid & I_ddr_user_ready;

    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            O_ddr_user_wr_en <= 1'b0;
        else
            O_ddr_user_wr_en <= S_fifo_rd_en;
    end

	/*
		如果前一帧图像有误少行了，那么地址回退为之前的，覆盖有问题的那帧图像
	*/
    always @(posedge I_ddr_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            O_ddr_user_addr <= 1'b0;
        else
            if(S_frame_start && (!I_mipi_rx_error))
                begin
                    case(S_video_in_wp)
                        'd0: O_ddr_user_addr <= IMAGE_BASE_ADDR_0;
                        'd1: O_ddr_user_addr <= IMAGE_BASE_ADDR_1;
                        'd2: O_ddr_user_addr <= IMAGE_BASE_ADDR_2;
                        'd3: O_ddr_user_addr <= IMAGE_BASE_ADDR_3;
                    endcase
                end
			else if(S_frame_start && I_mipi_rx_error)
				begin
                	case(S_video_in_wp)
                        'd0: O_ddr_user_addr <= IMAGE_BASE_ADDR_3;
                        'd1: O_ddr_user_addr <= IMAGE_BASE_ADDR_0;
                        'd2: O_ddr_user_addr <= IMAGE_BASE_ADDR_1;
                        'd3: O_ddr_user_addr <= IMAGE_BASE_ADDR_2;
                    endcase
                end
            else if(O_ddr_user_wr_en)
                O_ddr_user_addr <= O_ddr_user_addr + 'd8;
            else    
                O_ddr_user_addr <= O_ddr_user_addr;
    end

    
endmodule
