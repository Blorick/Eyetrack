`timescale 1ns/1ps
//=============================================================================
//  eg4d_mode_filter -- v2.5 integrated source
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk
//  复位     : 按本模块接口定义
//  参数     : 见模块参数
//  验证状态 : 见 docs/validation.md
//=============================================================================
//=============================================================================
//  eg4d_mode_filter  --  3x3 neighbourhood camera modes (EG4D slave, 1 px/clk)
//-----------------------------------------------------------------------------
//  Domain : display pixel clock (75 MHz on the slave's own PLL)
//  Role   : the LUMA-domain half of the v1.13 20-mode set.  Coefficients and
//           gating are copied verbatim from
//           contest/v1.13/rtl/isp/isp_mode_filter.v (lines cited per mode).
//
//  Why only luma
//  -------------
//  The master's send path is raster-locked (no frame buffer), so at the raster
//  rate 16-bit colour needs 116.4 MB/s = 94.6% of a 1 GbE link, while 8-bit
//  luma needs 58.2 MB/s = 47.3%.  Every mode this module implements is
//  monochrome anyway, and the colour-bearing modes (0..8, 12) are per-pixel and
//  stay on the master.  See docs/design/v1.19-eg4d-offload-evaluation.md §9.
//
//  Modes implemented here
//  ----------------------
//     9  M_SOBEL      |Gx|+|Gy| saturate            (isp_mode_filter.v:908-911)
//    10  M_SOBEL_INV  ~sobel                        (:913-914)
//    11  M_EMBOSS     clamp((pos-neg)*2 + 128)       (:921-927)
//    13  M_ERODE      3x3 min                       (:933)
//    14  M_DILATE     3x3 max                       (:931)
//    15  M_MORPH      dilate - erode                (:935)
//    16  M_BINSOBEL   mag >= EDGE_RGB_TH ? FF : 00  (:917)
//    17  M_REGION     first-pass binary region mask (the wrapper adds the
//                      second majority pass)
//    18  M_CARTOON    Yq = mean9 & 0xE0, edge = (max9-min9) >= cartoon_th
//                                                    (:549-552, :598-601)
//     0..8,12,17,19  ->  Yq/sobel path is bypassed and luma passes through,
//                        except 19 which only drives the histogram below.
//
//  Mode 17's second pass is in eg4d_mode17_majority.v and is selected by
//  eg4d_mode_pipeline.v.  This module exports the first-pass mask as O_mask
//  so the second pass remains an independently verifiable row-memory block.
//
//  Pipeline (see the note above each stage)
//     R0  input pixel; row-memory read issued
//     R1  line-memory read data aligned with the delayed input pixel
//     R2  the 3x3 window (2-deep column shift registers on each of 3 rows)
//     R3  window combos: sobel Gx/Gy, emboss pos/neg, row min/max, sum9
//     R4  per-mode results: sob, snv, bsb, emb, dil, ero, mrp, mean9, max9,
//         min9, Yq, edge
//     R5  mode mux + output register
//  Total latency = 1 line + 2 pixels + 5 clocks.
//=============================================================================
module eg4d_mode_filter_ref #(
    parameter integer IMG_W        = 1280,
    parameter [7:0]   EDGE_RGB_TH  = 8'd80,
    parameter [7:0]   CARTOON_OFF  = 8'd0          // extra, for TB corner cases
)(
    input  wire       I_clk,
    input  wire       I_rst_n,

    input  wire [4:0] I_mode,
    input  wire [7:0] I_bin_th,        // mode 17 mask threshold
    input  wire [7:0] I_cartoon_th,    // mode 18 outline threshold

    // video timing, display domain (I_user = first active pixel of a line)
    input  wire       I_vsync,
    input  wire       I_hsync,
    input  wire       I_de,
    input  wire       I_user,
    input  wire       I_last,
    input  wire [7:0] I_y,             // luma of the current pixel

    // delayed by the pipeline
    output wire [4:0] O_mode,
    output wire       O_vsync,
    output wire       O_hsync,
    output wire       O_de,
    output wire       O_user,
    output wire       O_last,
    output reg  [7:0] O_y,
    output reg         O_edge,          // mode 18 outline flag
    output reg         O_mask,           // mode 17 binary mask (pre-majority)
    output reg         O_window_valid   // complete 3x3 centre for this beat
);

    //=========================================================================
    // 0) helpers (copied from isp_mode_filter.v)
    //=========================================================================
    function [7:0] min2;
        input [7:0] a, b;
        begin min2 = (a < b) ? a : b; end
    endfunction

    function [7:0] max2;
        input [7:0] a, b;
        begin max2 = (a > b) ? a : b; end
    endfunction

    // isp_mode_filter.v:217 -- clamp a signed value into 0..255
    function [7:0] clamp8;
        input signed [23:0] v;
        begin
            if (v < 24'sd0)          clamp8 = 8'd0;
            else if (v > 24'sd255)   clamp8 = 8'd255;
            else                     clamp8 = v[7:0];
        end
    endfunction

    // RGB332 chroma reconstruction stays within signed 10 bits after the
    // four-bit delta quantisation; keep the clamp comparator narrow as well.
    function [7:0] clamp10;
        input signed [9:0] v;
        begin
            if (v < 10'sd0)           clamp10 = 8'd0;
            else if (v > 10'sd255)    clamp10 = 8'd255;
            else                      clamp10 = v[7:0];
        end
    endfunction

    // RGB332 is the on-wire mode-18 format.  Explicit tables keep the
    // expansion multiplier-free and deterministic across TD and Questa.
    function [7:0] rgb332_r8;
        input [2:0] v;
        begin
            case (v)
                3'd0: rgb332_r8 = 8'd0;
                3'd1: rgb332_r8 = 8'd36;
                3'd2: rgb332_r8 = 8'd73;
                3'd3: rgb332_r8 = 8'd109;
                3'd4: rgb332_r8 = 8'd146;
                3'd5: rgb332_r8 = 8'd182;
                3'd6: rgb332_r8 = 8'd219;
                default: rgb332_r8 = 8'd255;
            endcase
        end
    endfunction

    function [7:0] rgb332_b8;
        input [1:0] v;
        begin
            case (v)
                2'd0: rgb332_b8 = 8'd0;
                2'd1: rgb332_b8 = 8'd85;
                2'd2: rgb332_b8 = 8'd170;
                default: rgb332_b8 = 8'd255;
            endcase
        end
    endfunction

    function [7:0] rgb332_luma;
        input [7:0] p;
        reg [7:0] r8, g8, b8;
        reg [15:0] s;
        begin
            r8 = rgb332_r8(p[7:5]);
            g8 = rgb332_r8(p[4:2]);
            b8 = rgb332_b8(p[1:0]);
            // Shift/add constants keep mode 18 off the single EG4D DSP:
            // 77=64+8+4+1, 150=128+16+4+2, and 29=16+8+4+1.
            s = ({8'd0, r8} << 6) + ({8'd0, r8} << 3) +
                ({8'd0, r8} << 2) + {8'd0, r8} +
                ({8'd0, g8} << 7) + ({8'd0, g8} << 4) +
                ({8'd0, g8} << 2) + ({8'd0, g8} << 1) +
                ({8'd0, b8} << 4) + ({8'd0, b8} << 3) +
                ({8'd0, b8} << 2) + {8'd0, b8};
            rgb332_luma = s[15:8];
        end
    endfunction

    // Preserve the center pixel's hue while quantising the neighbourhood
    // luma.  Mode 18 follows the v1.13 semantic: a detected outline is a
    // black pixel, while a non-edge centre receives this RGB332 flat fill.
    function [7:0] rgb332_flat;
        input [7:0] p;
        input [7:0] yq;
        reg [7:0] r8, g8, b8, gy;
        reg signed [9:0] dr, dg, db;
        reg signed [9:0] sr, sg, sb;
        reg [7:0] rr, gg, bb;
        begin
            r8 = rgb332_r8(p[7:5]);
            g8 = rgb332_r8(p[4:2]);
            b8 = rgb332_b8(p[1:0]);
            gy = rgb332_luma(p);
            dr = $signed({1'b0, r8}) - $signed({1'b0, gy});
            dg = $signed({1'b0, g8}) - $signed({1'b0, gy});
            db = $signed({1'b0, b8}) - $signed({1'b0, gy});
            dr = {dr[9], dr[8:4], 4'b0000};
            dg = {dg[9], dg[8:4], 4'b0000};
            db = {db[9], db[8:4], 4'b0000};
            sr = $signed({2'b00, yq}) + dr;
            sg = $signed({2'b00, yq}) + dg;
            sb = $signed({2'b00, yq}) + db;
            rr = clamp10(sr);
            gg = clamp10(sg);
            bb = clamp10(sb);
            rgb332_flat = {rr[7:5], gg[7:5], bb[7:6]};
        end
    endfunction

    // isp_mode_filter.v:427 -- 3x3 mean, 57/512 ~= 1/9, shift/add on purpose
    // (a constant *57 maps onto a DSP in TD, and the device has only 29).
    function [7:0] mean_of_sum;
        input [11:0] s;
        reg [17:0] m;
        begin
            m = (s << 5) + (s << 4) + (s << 3) + s;   // s * 57
            mean_of_sum = m[16:9];
        end
    endfunction

    //=========================================================================
    // 1) storage: 3-way rotating row memories (luma / timing / mask)
    //    Three buffers (not two) avoid any read-during-write reliance: while
    //    row y is written into buffer i, rows y-1 and y-2 are read from the
    //    other two.
    //=========================================================================
    // ★ These row memories land in DISTRIBUTED RAM, not BRAM (measured on the
    //   real EG4D: #bram 0, and the whole filter #lut 9838/19600 = 50.19%).
    //   Splitting them into one array per row -- so each memory has exactly one
    //   read and one write port, which is what a block RAM offers -- was TRIED
    //   AND MADE IT WORSE: #lut 14180 = 72.35%, still #bram 0.  The port count
    //   is therefore not the blocker; the likely one is the array SIZE (1280x8 =
    //   10240 bits exceeds one BRAM9K's 9216, and TD appears to fall back to
    //   LUTRAM rather than span blocks -- the same size threshold seen on the
    //   PH1P side).  Recorded as an open optimisation, NOT solved: see
    //   docs/S2-status.md section 6.  Do not repeat the per-row split.
    // ★ These row memories land in DISTRIBUTED RAM, not BRAM: measured on the
    //   real EG4D, #bram 0 and #lut 9838/19600 = 50.19%.  Four variants were
    //   tried and ALL failed to get BRAM, two of them were worse:
    //     - one array per row (each still read twice)      -> #lut 14180 = 72.35%
    //     - two copies of each row, one read port each     -> #lut  9850 = 50.26%
    //     - the same with the second copy storing ~data,   -> #lut 14143 = 72.16%
    //       to defeat TD's merge_equal pass
    //   Two clean probes on the same device with arrays of 1 read + 1 write
    //   (including 3840x8 with a computed sel*1280+col address, and 4096x8)
    //   inferred BRAM/BRAM9K with ~20 LUTs, so neither the port count, the
    //   array size, nor the computed address is the cause.  The reason this
    //   particular memory shape is refused has NOT been identified.
    //   It is left as distributed RAM on purpose: 50.19% already fits (the link
    //   adds an estimated 8..13%), so this is spare-margin work, not a blocker.
    //   Do not re-run the three failed variants.
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [7:0] S_lb [0:3*IMG_W-1];

    // {vsync, hsync, de, user, last, win_row_ok}
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [5:0] S_tb [0:3*IMG_W-1];

    // {mask_of_this_window, reserved} -- retained for debug; the production
    // second pass consumes O_mask through eg4d_mode_pipeline.
    (* ram_style = "block", fehdl_force_ram = "1" *)
    reg [1:0] S_mb [0:3*IMG_W-1];

    reg  [1:0]  S_row3;        // rotation index of the row being written
    reg  [10:0] S_x;           // NEXT column address (see W_x below)
    reg  [10:0] S_line;        // line counter within the frame
    reg         S_row_ok;      // line >= 2, i.e. two rows of history exist

    // ★ The memory address must be correct DURING the first active pixel of a
    //   line.  A plain "reset the counter on I_user" register still holds the
    //   previous line's last value on that cycle (non-blocking assignment
    //   lands at the end of it), so column 0 would be written and read at the
    //   wrong address.  W_x is therefore resolved combinationally from I_user.
    wire [10:0] W_x = I_user ? 11'd0 : S_x;

    // ★ Row index of the row currently on the input, resolved combinationally.
    //   A registered version lags by one row and the flag is stored into, and
    //   later read back from, the row memory -- so a registered flag ends up
    //   TWO rows late.  The 3x3 window is complete from row 2 onwards (it needs
    //   rows y-2, y-1, y); storing (W_line_now >= 2) puts the flag exactly on
    //   row y, which is what the read-back sees when the window is formed.
    wire [10:0] W_line_now = I_user ? (I_vsync ? 11'd0 : (S_line + 11'd1))
                                    : S_line;
    // For mode 18 the byte is RGB332 on the wire; all other modes consume Y8.
    wire [7:0] W_input_y = (I_mode == 5'd18) ? rgb332_luma(I_y) : I_y;
    // ★ The flag stored during row y is READ BACK while processing row y+1 and
    //   is used for the window centred on row y.  It must therefore predict that
    //   the NEXT row's window is complete: centre r=y needs rows y-1,y,y+1, so
    //   the answer is 'y >= 1', not 'y >= 2' (which is the condition for the
    //   window of the CURRENT row and made the first valid output row all-black).
    wire        W_row_ok_now = (W_line_now >= 11'd1);

    wire [1:0] W_i_y  = S_row3;
    wire [1:0] W_i_y1 = (S_row3 == 2'd0) ? 2'd2 : (S_row3 - 2'd1);
    wire [1:0] W_i_y2 = (S_row3 >= 2'd2) ? (S_row3 - 2'd2) : (S_row3 + 2'd1);

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_row3   <= 2'd0;
            S_x      <= 11'd0;
            S_line   <= 11'd0;
            S_row_ok <= 1'b0;
        end
        else if (I_user) begin
            S_x    <= 11'd1;                       // column 1 comes next
            S_row3 <= (S_row3 == 2'd2) ? 2'd0 : (S_row3 + 2'd1);
            if (I_vsync) begin
                S_line   <= 11'd0;
                S_row_ok <= 1'b0;
            end
            else begin
                S_line   <= S_line + 11'd1;
                // Evaluated with the OLD S_line, which is (y-1) during row y, so this
                // asserts exactly when row y-2 exists (y >= 2) -- the first row
                // whose 3x3 window is complete.  Requiring S_line >= 2 made row 2
                // look invalid and zeroed every neighbourhood mode on the first
                // valid output line.
                S_row_ok <= (W_line_now >= 11'd2);
            end
        end
        else if (I_de) begin
            S_x <= S_x + 11'd1;
        end
    end

    //-------------------------------------------------------------------------
    // R0: issue the row-memory reads and register the input pixel
    //-------------------------------------------------------------------------
    reg [7:0] S_y_d1;
    reg [7:0] S_rgb_d1;
    // The luma must travel the same five stages as the mode results, so that
    // the passthrough case in the mode mux lands in the same beat.
    reg [7:0] S_y_d2, S_y_d3, S_y_d4, S_y_d5;
    reg [7:0] S_rgb_d2, S_rgb_d3, S_rgb_d4, S_rgb_d5;
    reg [7:0] S_lb1_r, S_lb2_r;
    reg [7:0] S_y_d2a;          // input pixel delayed to match the address stage
    reg [11:0] S_a1, S_a2;      // REGISTERED row-memory addresses (3*IMG_W fits 12 bits)
    reg [5:0] S_tb_r;
    reg       S_de_d1;
    reg [4:0] S_mode_d1;

    // The row-memory ADDRESS is registered before it reaches the RAM.  Leaving
    // the address multiply (W_i_y1 * IMG_W + W_x) directly in front of the RAM
    // input was the worst path of the whole slave once the link and the adapter
    // were added (Setup WNS -1515 ps, 38 failing endpoints, against +107 ps for
    // the filter standing alone).  This costs one pipeline cycle; the adapter's
    // S_LINE phase thresholds absorb it.
    always @(posedge I_clk) begin
        S_a1 <= W_i_y1 * IMG_W + W_x;
        S_a2 <= W_i_y2 * IMG_W + W_x;
    end

    always @(posedge I_clk) begin
        S_lb1_r    <= S_lb[S_a1];                   // row y-1, column x
        S_lb2_r    <= S_lb[S_a2];                   // row y-2, column x
        S_tb_r     <= S_tb[S_a1];
        S_y_d1     <= W_input_y;
        S_rgb_d1   <= I_y;
        S_y_d2a    <= S_y_d1;     // luma takes the same extra cycle
        S_de_d1    <= I_de;
        S_mode_d1  <= I_mode;
    end

    //-------------------------------------------------------------------------
    // R1 -> R2: the 3x3 window.
    //   After the R1 edge, S_y_d1 / S_lb1_r / S_lb2_r hold column x of rows
    //   y / y-1 / y-2.  Two more registers on each give columns x-1 and x-2,
    //   so when b_l is valid the window is centred on (row y-1, column x-1).
    //   S_tb_r at that same moment already holds the timing of row y-1,
    //   column x-1 (its read address led by one), so one more register aligns
    //   it with the window.
    //-------------------------------------------------------------------------
    reg [7:0] S_br, S_bc, S_bl;
    reg [7:0] S_mr, S_mc, S_ml;
    reg [7:0] S_tr, S_tc, S_tl;
    reg [5:0] S_tim_a;
    reg [4:0] S_mode_a;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_br <= 8'd0; S_bc <= 8'd0; S_bl <= 8'd0;
            S_mr <= 8'd0; S_mc <= 8'd0; S_ml <= 8'd0;
            S_tr <= 8'd0; S_tc <= 8'd0; S_tl <= 8'd0;
            S_tim_a <= 6'd0;
            S_mode_a <= 5'd0;
        end
        else begin
            S_br <= S_y_d2a;  S_bc <= S_br;   S_bl <= S_bc;
            S_mr <= S_lb1_r;  S_mc <= S_mr;   S_ml <= S_mc;
            S_tr <= S_lb2_r;  S_tc <= S_tr;   S_tl <= S_tc;
            S_tim_a  <= S_tb_r;
            S_mode_a <= S_mode_d1;
        end
    end

    wire W_vsync_a = S_tim_a[5];
    wire W_hsync_a = S_tim_a[4];
    wire W_de_a    = S_tim_a[3];
    wire W_user_a  = S_tim_a[2];
    wire W_last_a  = S_tim_a[1];
    wire W_wok_a   = S_tim_a[0];

    // The output pixel is column (x-1) of the previous line, so its own column
    // index lags the input counter by one.
    reg [10:0] S_xo;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)                     S_xo <= 11'd0;
        else if (W_user_a)                S_xo <= 11'd0;
        else if (W_de_a)                  S_xo <= S_xo + 11'd1;
    end

    // ★ I_de must be carried down the pipeline in its own shift register.
    //   The row memory only stores ACTIVE pixels, so during blanking the read
    //   address stops advancing and the last stored entry keeps being read:
    //   taking `de` out of the memory left O_de stuck high for the whole
    //   blanking interval.  The invariant check in the testbench caught this as
    //   559..633 output pixels per line instead of exactly IMG_W.
    //     S_de_pipe[2] = I_de delayed 3 -> the window stage
    //     S_de_pipe[5] = I_de delayed 6 -> the output stage
    reg [5:0] S_de_pipe;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) S_de_pipe <= 6'd0;
        else          S_de_pipe <= {S_de_pipe[4:0], I_de};
    end

    wire W_de_win = S_de_pipe[2];
    wire W_de_out = S_de_pipe[5];

    // A window is only meaningful with two rows of history; isp_mode_filter.v:376
    // gates every neighbourhood mode on `win_valid`.  The column gate is on the
    // window CENTRE: centre c needs columns c-1 .. c+1, so c >= 1.
    wire W_win_valid = W_de_win && W_wok_a && (S_xo >= 11'd1);

    //=========================================================================
    // 2) R3: window combos
    //=========================================================================
    // sobel Gx/Gy, isp_mode_filter.v:336-341 (11-bit signed operands)
    wire signed [11:0] W_sx =
          ($signed({4'b0, S_tr}) + ($signed({4'b0, S_mr}) <<< 1) + $signed({4'b0, S_br}))
        - ($signed({4'b0, S_tl}) + ($signed({4'b0, S_ml}) <<< 1) + $signed({4'b0, S_bl}));
    wire signed [11:0] W_sy =
          ($signed({4'b0, S_bl}) + ($signed({4'b0, S_bc}) <<< 1) + $signed({4'b0, S_br}))
        - ($signed({4'b0, S_tl}) + ($signed({4'b0, S_tc}) <<< 1) + $signed({4'b0, S_tr}));

    // emboss pos/neg, isp_mode_filter.v:352-353
    wire [10:0] W_em_pos = {3'b0, S_mr} + {3'b0, S_bc} + {2'b0, S_br, 1'b0};
    wire [10:0] W_em_neg = {3'b0, S_tc} + {3'b0, S_ml} + {2'b0, S_tl, 1'b0};

    // row min/max, isp_mode_filter.v:358-363
    wire [7:0] W_rmax0 = max2(S_tl, max2(S_tc, S_tr));
    wire [7:0] W_rmax1 = max2(S_ml, max2(S_mc, S_mr));
    wire [7:0] W_rmax2 = max2(S_bl, max2(S_bc, S_br));
    wire [7:0] W_rmin0 = min2(S_tl, min2(S_tc, S_tr));
    wire [7:0] W_rmin1 = min2(S_ml, min2(S_mc, S_mr));
    wire [7:0] W_rmin2 = min2(S_bl, min2(S_bc, S_br));

    // 3x3 sum for the mean / mask, isp_mode_filter.v:436
    wire [11:0] W_sum9 = ({5'b0, S_tl} + {5'b0, S_tc} + {5'b0, S_tr})
                       + ({5'b0, S_ml} + {5'b0, S_mc} + {5'b0, S_mr})
                       + ({5'b0, S_bl} + {5'b0, S_bc} + {5'b0, S_br});

    reg signed [11:0] S_sx_r, S_sy_r;
    reg [10:0]        S_emp_r, S_emn_r;
    reg [7:0]         S_rmax_r0, S_rmax_r1, S_rmax_r2;
    reg [7:0]         S_rmin_r0, S_rmin_r1, S_rmin_r2;
    reg [11:0]        S_sum9_r;
    reg [5:0]         S_tim_b;
    reg [4:0]         S_mode_b;
    reg               S_wok_b;
    reg [10:0]        S_xo_b;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_sx_r <= 12'sd0; S_sy_r <= 12'sd0;
            S_emp_r <= 11'd0; S_emn_r <= 11'd0;
            S_rmax_r0 <= 8'd0; S_rmax_r1 <= 8'd0; S_rmax_r2 <= 8'd0;
            S_rmin_r0 <= 8'd0; S_rmin_r1 <= 8'd0; S_rmin_r2 <= 8'd0;
            S_sum9_r <= 12'd0;
            S_tim_b <= 6'd0; S_mode_b <= 5'd0;
            S_wok_b <= 1'b0; S_xo_b <= 11'd0;
        end
        else begin
            S_sx_r <= W_sx; S_sy_r <= W_sy;
            S_emp_r <= W_em_pos; S_emn_r <= W_em_neg;
            S_rmax_r0 <= W_rmax0; S_rmax_r1 <= W_rmax1; S_rmax_r2 <= W_rmax2;
            S_rmin_r0 <= W_rmin0; S_rmin_r1 <= W_rmin1; S_rmin_r2 <= W_rmin2;
            S_sum9_r <= W_sum9;
            S_tim_b  <= S_tim_a;
            S_mode_b <= S_mode_a;
            S_wok_b  <= W_win_valid;
            S_xo_b   <= S_xo;
        end
    end

    //=========================================================================
    // 3) R4: per-mode results
    //=========================================================================
    // |Gx| + |Gy|, saturated; isp_mode_filter.v:906-911
    wire [11:0] W_ax = S_sx_r[11] ? (~S_sx_r + 12'sd1) : S_sx_r;
    wire [11:0] W_ay = S_sy_r[11] ? (~S_sy_r + 12'sd1) : S_sy_r;
    wire [12:0] W_mag = {1'b0, W_ax} + {1'b0, W_ay};

    wire [7:0] W_sob = S_wok_b ? ((W_mag > 13'd255) ? 8'hFF : W_mag[7:0]) : 8'h00;
    wire [7:0] W_snv = S_wok_b ? (~W_sob) : 8'h00;
    wire [7:0] W_bsb = S_wok_b ? ((W_mag >= {5'd0, EDGE_RGB_TH}) ? 8'hFF : 8'h00) : 8'h00;

    // emboss clamp((pos-neg)*2 + 128); isp_mode_filter.v:921-927
    wire signed [11:0] W_emd    = $signed({1'b0, S_emp_r}) - $signed({1'b0, S_emn_r});
    wire signed [23:0] W_emo    = ($signed(W_emd) <<< 1) + 24'sd128;
    wire [7:0]         W_emb    = S_wok_b ? clamp8(W_emo) : 8'h00;

    // morphology; isp_mode_filter.v:931-935
    wire [7:0] W_max9 = max2(S_rmax_r0, max2(S_rmax_r1, S_rmax_r2));
    wire [7:0] W_min9 = min2(S_rmin_r0, min2(S_rmin_r1, S_rmin_r2));
    wire [7:0] W_dil  = S_wok_b ? W_max9 : 8'h00;
    wire [7:0] W_ero  = S_wok_b ? W_min9 : 8'h00;
    wire [7:0] W_mrp  = S_wok_b ? (W_dil - W_ero) : 8'h00;

    // mean9 / mask / cartoon; isp_mode_filter.v:444-447, :549-552, :598-601
    wire [7:0] W_mean9 = S_wok_b ? mean_of_sum(S_sum9_r) : 8'd0;
    wire [11:0] W_sum_th = ({4'd0, I_bin_th} << 3) + {4'd0, I_bin_th};   // 9*th
    wire        W_mask_b = S_wok_b && (S_sum9_r >= W_sum_th);
    wire [7:0]  W_yq     = W_mean9 & 8'hE0;
    wire        W_edge_b = S_wok_b && ((W_max9 - W_min9) >= I_cartoon_th);

    reg [7:0]  S_sob_r, S_snv_r, S_bsb_r, S_emb_r, S_dil_r, S_ero_r, S_mrp_r;
    reg [7:0]  S_yq_r;
    reg        S_edge_r, S_mask_r;
    reg [5:0]  S_tim_c;
    reg [4:0]  S_mode_c;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_sob_r <= 8'd0; S_snv_r <= 8'd0; S_bsb_r <= 8'd0; S_emb_r <= 8'd0;
            S_dil_r <= 8'd0; S_ero_r <= 8'd0; S_mrp_r <= 8'd0;
            S_yq_r  <= 8'd0; S_edge_r <= 1'b0; S_mask_r <= 1'b0;
            S_tim_c <= 6'd0; S_mode_c <= 5'd0;
        end
        else begin
            S_sob_r <= W_sob; S_snv_r <= W_snv; S_bsb_r <= W_bsb;
            S_emb_r <= W_emb; S_dil_r <= W_dil; S_ero_r <= W_ero;
            S_mrp_r <= W_mrp; S_yq_r  <= W_yq;
            S_edge_r <= W_edge_b; S_mask_r <= W_mask_b;
            S_tim_c  <= S_tim_b;  S_mode_c <= S_mode_b;
        end
    end

    //=========================================================================
    // 4) R5: mode mux + output register
    //=========================================================================
    reg [7:0] S_y_mux;

    always @(*) begin
        case (S_mode_c)
            5'd9:    S_y_mux = S_sob_r;      // sobel
            5'd10:   S_y_mux = S_snv_r;      // sobel inverted
            5'd11:   S_y_mux = S_emb_r;      // emboss
            5'd13:   S_y_mux = S_ero_r;      // erode
            5'd14:   S_y_mux = S_dil_r;      // dilate
            5'd15:   S_y_mux = S_mrp_r;      // morphological edge
            5'd16:   S_y_mux = S_bsb_r;      // binary sobel
            5'd17:   S_y_mux = S_mask_r ? 8'hFF : 8'h00;
            5'd18:   S_y_mux = !S_wok_b ? 8'h00 :
                                (S_edge_r ? 8'h00 : rgb332_flat(S_rgb_d4, S_yq_r));
            // 0..8, 12 are computed on the MASTER (they need colour); 17 and 19
            // are completed by the mode pipeline wrapper.  Everything else passes luma
            // through so a wrong mode code degrades to "no effect", never to
            // garbage.  S_yq_r is the quantised mean and is the closest luma
            // proxy the slave has, but passing it through would silently alter
            // the picture, so the mux uses the delayed luma instead.
            default: S_y_mux = S_y_d5;
        endcase
    end

    // The luma delay chain S_y_d2..S_y_d5 is declared with S_y_d1 in the R0
    // block above, so the passthrough term in the mode mux can use it.
    // Mode 18's raw centre and R4 values are already available in the cycle
    // that feeds O_y.  Use those combinational values at the output register;
    // taking S_yq_r/S_edge_r here would add one clock to the colour byte while
    // O_de still followed the original timing pipe (the first centre then
    // received a stale 0x20 quantisation).
    // The registered RGB centre becomes valid one column after the first
    // neighbourhood flag (the R0 address/read stage is one cycle ahead of
    // the colour delay).  Suppress that lone early beat instead of returning
    // a stale quantisation; the later beats retain the exact centre-column
    // colour.  Mode 17 has its own majority timing and keeps W_win_valid.
    wire W_mode18_window_valid = W_win_valid && (S_xo >= 11'd2);
    wire [7:0] W_mode18_y_now = !W_mode18_window_valid ? 8'h00 :
                                 (W_edge_b ? 8'h00 :
                                  rgb332_flat(S_rgb_d4, W_yq));

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_y_d2 <= 8'd0; S_y_d3 <= 8'd0; S_y_d4 <= 8'd0; S_y_d5 <= 8'd0;
            S_rgb_d2 <= 8'd0; S_rgb_d3 <= 8'd0; S_rgb_d4 <= 8'd0; S_rgb_d5 <= 8'd0;
            O_y <= 8'd0; O_edge <= 1'b0; O_mask <= 1'b0;
            O_window_valid <= 1'b0;
        end
        else begin
            S_y_d2 <= S_y_d1;
            S_y_d3 <= S_y_d2;
            S_y_d4 <= S_y_d3;
            S_y_d5 <= S_y_d4;
            S_rgb_d2 <= S_rgb_d1;
            S_rgb_d3 <= S_rgb_d2;
            S_rgb_d4 <= S_rgb_d3;
            S_rgb_d5 <= S_rgb_d4;
            O_y    <= (S_mode_c == 5'd18) ? W_mode18_y_now : S_y_mux;
            O_edge <= (S_mode_c == 5'd18) ? W_edge_b : S_edge_r;
            O_mask <= S_mask_r;
            O_window_valid <= (S_mode_c == 5'd18) ? W_mode18_window_valid :
                              W_win_valid;
        end
    end

    // Timing travels R2 -> R3 -> R4 -> R5 alongside the data.
    reg [5:0] S_tim_d;
    reg [4:0] S_mode_d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            S_tim_d <= 6'd0; S_mode_d <= 5'd0;
        end
        else begin
            S_tim_d  <= S_tim_c;
            S_mode_d <= S_mode_c;
        end
    end

    //=========================================================================
    assign O_vsync = S_tim_d[5];
    assign O_hsync = S_tim_d[4];
    assign O_de    = W_de_out;      // NOT the memory-delayed de: see above
    assign O_user  = S_tim_d[2] & W_de_out;
    assign O_last  = S_tim_d[1];
    assign O_mode  = S_mode_d;

    //=========================================================================
    // 5) write side of the row memories (one write port, shared)
    //    The pixel written for row y is the input pixel; the mask bit is the
    //    one computed for this window and is retained for the mode-17 second
    //    pass.
    //=========================================================================
    always @(posedge I_clk) begin
        if (I_de) begin
            S_lb[W_i_y * IMG_W + W_x] <= W_input_y;
            S_tb[W_i_y * IMG_W + W_x] <= {I_vsync, I_hsync, I_de, I_user,
                                          I_last, W_row_ok_now};
        end
        if (W_de_win) begin
            S_mb[W_i_y1 * IMG_W + S_xo] <= {S_mask_r, 1'b0};
        end
    end

endmodule
