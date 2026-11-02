//=============================================================================
//  dsp_camera_neighbourhood -- 摄像头历史邻域算法，四像素分组接口
//-----------------------------------------------------------------------------
//  所属域   : dsp
//  时钟域   : I_clk_csi，接显示域像素时钟，每四个有效像素输入一组
//  复位     : I_rst_n，异步低有效；行缓存内容不复位
//  参数     : IMG_WIDTH, IMG_HEIGHT, EDGE_RGB_TH
//  来源     : contest/v1.13/rtl/isp/isp_mode_filter.v
//=============================================================================
// 保留原有窗口、算术、边界与五级流水。输出选择只保留邻域模式；
// 点运算仍由串行 dsp_mode_filter 处理，未引用的四路点运算由综合剪除。
//=============================================================================
//  isp_mode_filter  --  20 种摄像头画面模式算法（原 17 模式 + 区域填充 + 卡通 + 自动对比度）
//-----------------------------------------------------------------------------
//  所属域   : isp
//  时钟域   : I_clk_csi
//  复位     : I_rst_n
//  参数     : IMG_WIDTH, IMG_HEIGHT, EDGE_RGB_TH
//  验证状态 : 本轮证据见 docs/optimization-results-20261006.md；上板未做
//=============================================================================
//======================================================================
//  isp_mode_filter  --  v1.10 multi-mode image-processing pipeline
//
//  v1.6/v1.7 : 17 modes (original RGB .. binary Sobel).
//  v1.10     : adds the competition extension "edge extraction and
//              region filling" as two more modes (17, 18) -> 19 modes.
//              Nothing in modes 0..16 was changed; see SECTION v1.10
//              below for the exact data path of the new modes.
//
//  Ported from the 2025 national-first-prize "camera mode" image
//  processor (udp_cam_ctrl.v + sobel_process / emboss_process /
//  morphology_process / line_buffer) onto this project's ISP stream.
//
//  Source project: FPGA-ANLU-National-First-Prize (Anlogic HX4S20C,
//                  OV5640 640x480 RGB888, 1 pixel per beat).
//  This project   : PH1P35MDG324, SC520CS 1280x720, 96-bit beat
//                  (4 pixels per beat), MIPI_RX_BYTE_CLK = 103.5 MHz.
//
//  STREAM FORMAT
//  ------------
//  I_tdata[95:72] = pixel 0 (earliest)
//  I_tdata[71:48] = pixel 1
//  I_tdata[47:24] = pixel 2
//  I_tdata[23: 0] = pixel 3 (latest)
//  each pixel = {R[23:16], G[15:8], B[7:0]}
//
//  MODE MAP (I_mode[4:0]) -- the reference NUM_MODES=16 table plus one
//  compatibility mode kept from v1.1-v1.5:
//
//    5'd0  original RGB            5'd9  Sobel edge magnitude
//    5'd1  grayscale               5'd10 Sobel inverted
//    5'd2  binarize (gray >= 128)  5'd11 emboss
//    5'd3  negative                5'd12 heat map
//    5'd4  red channel only        5'd13 erode  (3x3 min)
//    5'd5  posterize (v & 0xE0)    5'd14 dilate (3x3 max)
//    5'd6  sepia                   5'd15 morphological edge (dilate-erode)
//    5'd7  brightness (+KEY2)      5'd16 v1.5 example-2 binary Sobel (>=80)
//    5'd8  saturation (+KEY2)      5'd17 REGION FILL (v1.10) smoothed-mask
//                                       region fill / hole fill
//                                  5'd18 REGION FILL (v1.10) edge-constrained
//                                       flat-LUMA colour fill (cartoon)
//
//  MATHS (taken verbatim from the reference)
//  -----------------------------------------
//  gray  = (77*R + 150*G + 29*B) >> 8                     (sum == 256)
//  sepia = R*{101,89,70} + G*{197,176,137} + B*{48,43,34},
//          saturate > 255*256 then take [15:8]
//  Y     = (( 66*R + 129*G +  25*B + 128) >> 8) + 16
//  U     = ((-38*R -  74*G + 112*B + 128) >> 8) + 128
//  V     = ((112*R -  94*G -  18*B + 128) >> 8) + 128
//  saturation: U' = clamp((U-128)*factor >> 7), V' likewise,
//          R' = clamp((298*(Y-16) + 409*V' + 128) >> 8)
//          G' = clamp((298*(Y-16) - 100*U' - 208*V' + 128) >> 8)
//          B' = clamp((298*(Y-16) + 516*U' + 128) >> 8)
//  sobel : Gx = (p13 + 2*p23 + p33) - (p11 + 2*p21 + p31)
//          Gy = (p31 + 2*p32 + p33) - (p11 + 2*p12 + p13)
//          G  = saturate(|Gx| + |Gy|)  -> 8 bit
//  emboss: kernel [-2 -1 0; -1 0 1; 0 1 2]  (win_rc, r=row c=col)
//          E  = clamp(((w12+w21+2*w22) - (w01+w10+2*w00)) * 2 + 128)
//  morph : dilate = max3x3, erode = min3x3, edge = dilate - erode
//  heat  : g<64 -> {0, g<<2, 255}        ; g<128 -> {0, 255, 255-((g-64)<<2)}
//          g<192 -> {(g-128)<<2, 255, 0}; else   -> {255, 255-((g-192)<<2), 0}
//
//  DOCUMENTED DEVIATIONS FROM THE REFERENCE
//  ----------------------------------------
//  1. Neighbourhood aperture.  The reference computes the 3x3
//     neighbourhood on every single pixel (640x480, one pixel per beat,
//     two 640x8 BRAMs).  This board has 42 432 registers but only
//     21 216 slices and the v1.5 baseline already used 73.9 % of them,
//     so the neighbourhood modes reuse the two-samples-per-beat
//     structure that v1.5's edge_detect_96 already ships: each beat
//     contributes two line samples, each the horizontal average of an
//     adjacent pixel pair.  Neighbourhood modes (9,10,11,13,14,15)
//     therefore run at IMG_WIDTH/2 horizontal resolution (2-pixel
//     horizontal smoothing); the vertical aperture and the 3x3 window
//     are exact.  Every per-pixel colour mode (0-8, 12, 16) runs at
//     full 1280-pixel resolution.
//  2. Mode 16 is not in the reference: it keeps the v1.5 example-2
//     binary Sobel (threshold 80) so the previous board behaviour is
//     still available.
//  3. Latency.  Five register stages, 5 beats.  The reference uses a
//     different (and mode-dependent) latency per effect; here every mode
//     has an identical latency and an identical position in the frame.
//  4. The reference's line_buffer.v cannot build a valid 3x3 window
//     (its read and write addresses are always equal and rd_addr stalls
//     at 0 for two cycles per line), so this file uses its own
//     two-line-buffer window instead of porting that module.
//
//  SECTION v1.10 -- "REGION FILL" (modes 17 and 18)
//  -------------------------------------------------
//  The competition extension asks for "video image stylisation or visual
//  enhancement, for example edge extraction and region filling".  Two
//  streaming-feasible readings of "region filling" are implemented; a
//  literal flood fill / connected-component fill is NOT (it needs a
//  frame buffer plus several passes, see the v1.10 report).
//
//  MODE 17 -- region fill on a smoothed binary mask (hole fill)
//    m      = (mean9(window) >= BIN_TH)   -- mask from the 3x3 MEAN, not
//                                            from a single raw sample
//    region = OR over 3x3 of m            -- effective 5x5 fill
//    Output is monochrome (white = filled region).
//    Why the mean: the first cut thresholded raw sensor luma and the
//    board test (2026-10-05) showed the result was still a ragged, noisy
//    binarisation.  Averaging the window first removes the
//    salt-and-pepper flicker of the threshold, and the OR pass then
//    closes every dark hole narrower than the 5x5 structuring element.
//    Only two 320x2-bit line memories + window shift registers are added,
//    written with exactly the same "previous group" scheme as the two
//    grey line memories, which keeps the beat alignment identical to
//    every other neighbourhood mode.  The mask is forced to 0 wherever
//    the window is invalid, so the second pass never reads
//    un-initialised line memory and never fills the frame border.
//
//  MODE 18 -- edge-constrained flat-colour fill  (cartoon / colour blocks)
//    Yq   = mean9(window) & 0xE0        -- smoothed luma, 8 flat levels
//    d_c  = p_c - gray(pixel)           -- the pixel's own chroma offset
//    out_c= clamp(Yq + (d_c rounded to 16))
//    out  = edge ? black : out_c
//    edge = (max9 - min9 of the window) >= CARTOON_TH  -- 3x3 local contrast,
//           NOT the binary Sobel: the board test showed the Sobel of a
//           real sensor fires on noise and paints most of the frame black.
//    Why luma-only quantisation: the first cut quantised R, G and B
//    independently to four levels.  A numeric sweep on a noisy frame
//    (docs/variant_sweep*.py) shows that, once the outline is fixed, the
//    independent per-channel quantisation is what leaves the loud colour
//    mottling, while luma-only quantisation keeps the hue of every pixel
//    and collapses noise into a few large flat levels.
//    Known limitation, shared with modes 9..16: the neighbourhood
//    aperture in this architecture lags the beat by about 4 pixels
//    horizontally (deviation 1), so the flat luma sits about 4 pixels to
//    the left of the chroma it belongs to.  Chroma is low frequency, so
//    this only reads as very slight colour fringing right at an edge,
//    where the black outline already is.
//
//  PIPELINE (5 register stages == 5 beats of latency)
//    R1  : gray, pair averages, RGB copy, side-bands, line-buffer writes
//    R2  : 3x3 window -> signed Sobel Gx/Gy, emboss pos/neg, row min/max,
//          YUV, window-valid, control
//    R3a : chroma scaling + pass-through of the R2 results
//    R3b : |Gx|+|Gy|, emboss output, dilate/erode/morph, sepia, saturation
//    R4  : per-pixel colour modes + mode mux + output register
//  All AXI4-Stream side-bands travel through the same registers, so every
//  marker stays aligned with its pixel.
//======================================================================
module dsp_camera_neighbourhood #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter EDGE_RGB_TH    = 8'd80      // mode 16 binary Sobel threshold
)(
    input                I_clk_csi,
    input                I_rst_n,
    input      [4:0]     I_mode,
    input signed [8:0]   I_brightness,   // -255 .. +255, step 32
    input      [7:0]     I_sat_factor,   // 64 / 128 / 192
    input      [7:0]     I_bin_th,       // v1.10 rev-D: BIN_TH  (SW4)
    input      [7:0]     I_cartoon_th,   // v1.10 rev-D: CARTOON_TH (SW3)
    input                I_tlast,
    input                I_tuser,
    input      [95:0]    I_tdata,
    input                I_tvalid,
    input                I_tready,
    output reg           O_tlast,
    output reg           O_tuser,
    output reg [95:0]    O_tdata,
    output reg           O_tvalid,
    output               O_tready
);

    assign O_tready = I_tready;

    //------------------------------------------------------------------
    // mode encoding
    //------------------------------------------------------------------
    localparam [4:0] M_ORIG      = 5'd0,
                     M_GRAY      = 5'd1,
                     M_BIN       = 5'd2,
                     M_NEG       = 5'd3,
                     M_RED       = 5'd4,
                     M_POST      = 5'd5,
                     M_SEPIA     = 5'd6,
                     M_BRIGHT    = 5'd7,
                     M_SAT       = 5'd8,
                     M_SOBEL     = 5'd9,
                     M_SOBEL_INV = 5'd10,
                     M_EMBOSS    = 5'd11,
                     M_HEAT      = 5'd12,
                     M_ERODE     = 5'd13,
                     M_DILATE    = 5'd14,
                     M_MORPH     = 5'd15,
                     M_BINSOBEL  = 5'd16,
                     M_CLOSE     = 5'd17,   // v1.10 region fill: closing
                     M_CARTOON   = 5'd18;   // v1.10 region fill: cartoon

    //------------------------------------------------------------------
    // small combin. helpers
    //------------------------------------------------------------------
    function [7:0] min2;
        input [7:0] a, b;
        begin min2 = (a < b) ? a : b; end
    endfunction

    function [7:0] max2;
        input [7:0] a, b;
        begin max2 = (a > b) ? a : b; end
    endfunction

    // clamp a wide signed value into an 8-bit unsigned pixel
    function [7:0] clamp8;
        input signed [23:0] v;
        begin
            if (v > 24'sd255)    clamp8 = 8'hFF;
            else if (v < 24'sd0) clamp8 = 8'h00;
            else                 clamp8 = v[7:0];
        end
    endfunction

    // reference grayscale: (77*R + 150*G + 29*B) >> 8
    //   77 = 64+ 8+ 4+1 ; 150 = 128+16+4+2 ; 29 = 16+8+4+1
    //   all partial products fit in 16 bits (max total = 65 280)
    //
    // v1.10 rev-C timing: this used to be written as one twelve-term
    // left-associative chain (eleven cascaded adders) and was the worst
    // setup path at 103.5 MHz.  Building the three per-channel products
    // independently and summing them as (pr + pg) + pb is bit-for-bit
    // identical (every partial product is exact in 16 bits) but only five
    // adders deep.
    function [7:0] gray_of;
        input [7:0] r, g, b;
        reg [15:0] pr, pg, pb, s;
        begin
            pr = ({8'd0, r} << 6) + ({8'd0, r} << 3) +
                 ({8'd0, r} << 2) + {8'd0, r};
            pg = ({8'd0, g} << 7) + ({8'd0, g} << 4) +
                 ({8'd0, g} << 2) + ({8'd0, g} << 1);
            pb = ({8'd0, b} << 4) + ({8'd0, b} << 3) +
                 ({8'd0, b} << 2) + {8'd0, b};
            s  = (pr + pg) + pb;
            gray_of = s[15:8];
        end
    endfunction

    //------------------------------------------------------------------
    // input pixels
    //------------------------------------------------------------------
    wire [23:0] px0 = I_tdata[95:72];
    wire [23:0] px1 = I_tdata[71:48];
    wire [23:0] px2 = I_tdata[47:24];
    wire [23:0] px3 = I_tdata[23: 0];

    wire [7:0] g_in0 = gray_of(px0[23:16], px0[15:8], px0[7:0]);
    wire [7:0] g_in1 = gray_of(px1[23:16], px1[15:8], px1[7:0]);
    wire [7:0] g_in2 = gray_of(px2[23:16], px2[15:8], px2[7:0]);
    wire [7:0] g_in3 = gray_of(px3[23:16], px3[15:8], px3[7:0]);

    // two averaged line samples per beat (see deviation 1 above)
    wire [8:0] pair_sum0 = {1'b0, g_in0} + {1'b0, g_in1};
    wire [8:0] pair_sum1 = {1'b0, g_in2} + {1'b0, g_in3};
    wire [7:0] pair_in0  = pair_sum0[8:1];
    wire [7:0] pair_in1  = pair_sum1[8:1];

    localparam integer IMG_WIDTH_GRP = (IMG_WIDTH >> 2);   // 320 groups

    //==================================================================
    // Stage R1 : gray / pair / RGB / side-band registers + line writes
    //==================================================================
    reg [7:0]  pair0_r, pair1_r;
    reg [7:0]  gray_r1_0, gray_r1_1, gray_r1_2, gray_r1_3;
    reg [95:0] rgb_r1;
    reg        tl_r1, tu_r1, tv_r1;

    (* ram_style = "block" *) reg [15:0] line1_mem [0:IMG_WIDTH_GRP-1];
    (* ram_style = "block" *) reg [15:0] line2_mem [0:IMG_WIDTH_GRP-1];

    reg [9:0]  x_grp;      // line-buffer address of the current write
    reg [9:0]  g_pos;      // group index of the beat on the input
    reg [9:0]  y_pos;      // row index of the beat on the input

    wire [15:0] top_word = line2_mem[x_grp];   // row y-2, current group
    wire [15:0] mid_word = line1_mem[x_grp];   // row y-1, current group
    wire [15:0] bot_word = {pair0_r, pair1_r}; // row y  , previous group

    wire [7:0] top0 = top_word[15:8];
    wire [7:0] top1 = top_word[7:0];
    wire [7:0] mid0 = mid_word[15:8];
    wire [7:0] mid1 = mid_word[7:0];

    // The two samples of the *previous* group are the horizontal
    // neighbours of this group's first sample, so the window columns are
    // three adjacent samples:
    //   w0 = S(2g-2), S(2g-1), S(2g)
    //   w1 = S(2g-1), S(2g)  , S(2g+1)
    // (S(2g) = pair0 of group g, S(2g+1) = pair1 of group g)
    reg [7:0] top_p2, top_p1;
    reg [7:0] mid_p2, mid_p1;
    reg [7:0] bot_p2, bot_p1;

    always @(posedge I_clk_csi or negedge I_rst_n) begin
        if (!I_rst_n) begin
            pair0_r <= 8'd0;  pair1_r <= 8'd0;
            gray_r1_0 <= 8'd0; gray_r1_1 <= 8'd0;
            gray_r1_2 <= 8'd0; gray_r1_3 <= 8'd0;
            rgb_r1 <= 96'd0;
            tl_r1 <= 1'b0; tu_r1 <= 1'b0; tv_r1 <= 1'b0;
        end else begin
            pair0_r <= pair_in0;
            pair1_r <= pair_in1;
            gray_r1_0 <= g_in0;  gray_r1_1 <= g_in1;
            gray_r1_2 <= g_in2;  gray_r1_3 <= g_in3;
            rgb_r1 <= I_tdata;
            tl_r1 <= I_tlast; tu_r1 <= I_tuser; tv_r1 <= I_tvalid;
        end
    end

    //==================================================================
    // 3x3 window (combinational, read side of the line memories)
    //   line samples per beat: S(2g), S(2g+1)   (two averaged pairs)
    //   w0 columns S(2g-2),S(2g-1),S(2g)
    //   w1 columns S(2g-1),S(2g)  ,S(2g+1)
    //==================================================================
    wire [7:0] t_l = top_p2, t_c = top_p1, t_r = top0,  t_r2 = top1;
    wire [7:0] m_l = mid_p2, m_c = mid_p1, m_r = mid0,  m_r2 = mid1;
    wire [7:0] b_l = bot_p2, b_c = bot_p1, b_r = pair0_r, b_r2 = pair1_r;

    // ---- Sobel (reference orientation) -------------------------------
    // Gx = (right column) - (left column), Gy = (bottom row) - (top row)
    //   11-bit signed operands: max column total 1020, max |grad| 1020
    wire signed [11:0] sx0 =
          ($signed({3'b0, t_r})  + ($signed({3'b0, m_r})  <<< 1) + $signed({3'b0, b_r}))
        - ($signed({3'b0, t_l})  + ($signed({3'b0, m_l})  <<< 1) + $signed({3'b0, b_l}));
    wire signed [11:0] sy0 =
          ($signed({3'b0, b_l})  + ($signed({3'b0, b_c})  <<< 1) + $signed({3'b0, b_r}))
        - ($signed({3'b0, t_l})  + ($signed({3'b0, t_c})  <<< 1) + $signed({3'b0, t_r}));
    wire signed [11:0] sx1 =
          ($signed({3'b0, t_r2}) + ($signed({3'b0, m_r2}) <<< 1) + $signed({3'b0, b_r2}))
        - ($signed({3'b0, t_c})  + ($signed({3'b0, m_c})  <<< 1) + $signed({3'b0, b_c}));
    wire signed [11:0] sy1 =
          ($signed({3'b0, b_c})  + ($signed({3'b0, b_r})  <<< 1) + $signed({3'b0, b_r2}))
        - ($signed({3'b0, t_c})  + ($signed({3'b0, t_r})  <<< 1) + $signed({3'b0, t_r2}));

    // ---- Emboss : pos - neg, kernel [(0,0)=-2 (0,1)=-1 (1,0)=-1
    //              (1,2)=+1 (2,1)=+1 (2,2)=+2] -------------------------
    //  10-bit operands, max row term 510, max partial sum 1020
    wire [10:0] em_pos0 = {2'b0, m_r} + {2'b0, b_c} + {1'b0, b_r, 1'b0};
    wire [10:0] em_neg0 = {2'b0, t_c} + {2'b0, m_l} + {1'b0, t_l, 1'b0};
    wire [10:0] em_pos1 = {2'b0, m_r2} + {2'b0, b_r} + {1'b0, b_r2, 1'b0};
    wire [10:0] em_neg1 = {2'b0, t_r}  + {2'b0, m_c} + {1'b0, t_c, 1'b0};

    // ---- 3x3 min / max per row (2 windows) --------------------------
    wire [7:0] rmax0_0 = max2(t_l, max2(t_c, t_r));
    wire [7:0] rmax0_1 = max2(m_l, max2(m_c, m_r));
    wire [7:0] rmax0_2 = max2(b_l, max2(b_c, b_r));
    wire [7:0] rmin0_0 = min2(t_l, min2(t_c, t_r));
    wire [7:0] rmin0_1 = min2(m_l, min2(m_c, m_r));
    wire [7:0] rmin0_2 = min2(b_l, min2(b_c, b_r));

    wire [7:0] rmax1_0 = max2(t_c, max2(t_r, t_r2));
    wire [7:0] rmax1_1 = max2(m_c, max2(m_r, m_r2));
    wire [7:0] rmax1_2 = max2(b_c, max2(b_r, b_r2));
    wire [7:0] rmin1_0 = min2(t_c, min2(t_r, t_r2));
    wire [7:0] rmin1_1 = min2(m_c, min2(m_r, m_r2));
    wire [7:0] rmin1_2 = min2(b_c, min2(b_r, b_r2));

    // A window is only coherent once two full rows and two groups of the
    // current row have been written, and never on the frame-start beat.
    // NOTE: g_pos (the beat's group index) must be used, not x_grp - the
    // line-buffer write address is one beat behind the pixel stream.
    wire win_valid = (y_pos >= 10'd2) && (g_pos >= 10'd2) && !tu_r1;

    // g_pos/y_pos track the row and group index of the beat currently
    // presented on the input, which is what win_valid has to test.
    always @(posedge I_clk_csi or negedge I_rst_n) begin
        if (!I_rst_n) begin
            g_pos <= 10'd0;
            y_pos <= 10'd0;
        end else if (I_tvalid) begin
            if (I_tuser) begin
                g_pos <= 10'd1;                 // this beat is group 0
                y_pos <= 10'd0;
            end else if (g_pos == IMG_WIDTH_GRP - 1) begin
                g_pos <= 10'd0;
                if (y_pos == IMG_HEIGHT - 1)
                    y_pos <= 10'd0;
                else
                    y_pos <= y_pos + 10'd1;
            end else begin
                g_pos <= g_pos + 10'd1;
            end
        end
    end

    //==================================================================
    // v1.10 rev-B MODE 17 : region fill on a SMOOTHED binary mask
    //
    //   The first cut (3x3 close of a raw-luma threshold) was tested on
    //   the board on 2026-10-05 and still looked like a noisy
    //   binarisation: the mask itself was ragged because it came straight
    //   from raw sensor luma.  rev-B takes the mask from the 3x3 MEAN of
    //   the same window (a six-pixel-effective low pass in this
    //   two-samples-per-beat domain) and then runs one more 3x3 OR pass
    //   over that mask stream:
    //
    //     m      = win_valid && (mean9(window) >= BIN_TH)   // smooth mask
    //     region = OR over 3x3 of m                         // 5x5 fill
    //
    //   The averaging kills the salt-and-pepper flicker of the threshold,
    //   and the OR pass closes every dark hole narrower than the 5x5
    //   structuring element, so the white regions come out solid.
    //==================================================================
    function [11:0] sum3;
        input [7:0] a, b, c;
        begin sum3 = {4'b0, a} + {4'b0, b} + {4'b0, c}; end
    endfunction

    // 3x3 mean of the nine window samples;  57/512 = 0.11133 ~= 1/9.
    // The *57 is written as shifts and adds on purpose: TD maps a
    // constant `s * 57` onto a DSP, and the device only has 40 of them.
    // A shift/add tree keeps it in LUTs.
    function [7:0] mean_of_sum;
        input [11:0] s;
        reg [17:0] m;
        begin
            m = (s << 5) + (s << 4) + (s << 3) + s;   // s * 57
            mean_of_sum = m[16:9];
        end
    endfunction

    wire [11:0] sum_w0 = sum3(t_l,t_c,t_r) + sum3(m_l,m_c,m_r) + sum3(b_l,b_c,b_r);
    wire [11:0] sum_w1 = sum3(t_c,t_r,t_r2) + sum3(m_c,m_r,m_r2) + sum3(b_c,b_r,b_r2);

    // mean9 >= I_bin_th  <=>  sum9 >= ~9*I_bin_th, so the MASK is taken
    // from a plain sum compare and never pays for the *57 shift/add chain
    // (that chain was on the critical path when it fed the mask as well).
    // v1.10 rev-D: I_bin_th is now a port driven by SW4, so the *9 is a
    // tiny shift/add off a register and stays off the sum path.
    wire [11:0] rf_sum_th = ({4'd0, I_bin_th} << 3) + {4'd0, I_bin_th};

    wire       mA = win_valid && (sum_w0 >= rf_sum_th);
    wire       mB = win_valid && (sum_w1 >= rf_sum_th);

    // Gated: on an incoherent window the line memories still hold
    // undefined data, so the smoothed luma is forced to a defined value
    // instead of leaking that into mode 18.  sum9 is registered first and
    // the *57 chain runs in the R2 combinational region (see below), so
    // the R1 -> R2 path stays at sum3 + row sums only.
    reg [11:0] sum0_r2, sum1_r2;

    //------------------------------------------------------------------
    // v1.10 rev-B MODE 18 outline source
    //
    //   rev-A used the binary Sobel of this window, which on a real
    //   sensor fires on noise: a numeric sweep on the exact aperture of
    //   this file (docs/variant_sweep*.py) shows EDGE_RGB_TH=80 paints 36 %
    //   of the frame black, and even 160 paints 27 % at high gain.
    //   The 3x3 LOCAL CONTRAST (max9 - min9) is far more robust to
    //   isolated noise and needs no new tree: the row max/min trees are
    //   already built for the morphology modes and their results are
    //   already registered as rm*_*_r / rn*_*_r at R2.
    //
    //   Timing: the combine + subtract runs purely on those R2 registers
    //   (see the R2 comb region below).  Doing it combinationally at R1
    //   failed timing at 103.5 MHz (WNS -2.48 ns) because it hung two
    //   more levels on the already critical DRAM -> row min/max path.
    //------------------------------------------------------------------

    reg        dilA_r, dilB_r;

    always @(posedge I_clk_csi or negedge I_rst_n) begin
        if (!I_rst_n) begin
            dilA_r <= 1'b0;
            dilB_r <= 1'b0;
        end else begin
            dilA_r <= mA;
            dilB_r <= mB;
        end
    end

    // second pass line memories, 2 bits per group (mask_a, mask_b)
    (* ram_style = "block" *) reg [1:0] dline1_mem [0:IMG_WIDTH_GRP-1];
    (* ram_style = "block" *) reg [1:0] dline2_mem [0:IMG_WIDTH_GRP-1];

    wire [1:0] d_top_word = dline2_mem[x_grp];   // row y-2
    wire [1:0] d_mid_word = dline1_mem[x_grp];   // row y-1
    wire [1:0] d_bot_word = {dilA_r, dilB_r};    // row y  , previous group

    wire d_top0 = d_top_word[1], d_top1 = d_top_word[0];
    wire d_mid0 = d_mid_word[1], d_mid1 = d_mid_word[0];

    reg  d_top_p2, d_top_p1, d_mid_p2, d_mid_p1, d_bot_p2, d_bot_p1;

    wire d_t_l = d_top_p2, d_t_c = d_top_p1, d_t_r = d_top0,  d_t_r2 = d_top1;
    wire d_m_l = d_mid_p2, d_m_c = d_mid_p1, d_m_r = d_mid0,  d_m_r2 = d_mid1;
    wire d_b_l = d_bot_p2, d_b_c = d_bot_p1, d_b_r = dilA_r, d_b_r2 = dilB_r;

    // rev-B: the second pass is a 3x3 MAJORITY (median) of the smoothed
    // mask, not an OR (dilate) and not an AND (erode):
    //   * a dark hole inside a bright region has 8 white neighbours, so
    //     the majority keeps it white  -> holes are filled;
    //   * an isolated white speck has 8 black neighbours, so the majority
    //     removes it  -> salt-and-pepper is gone;
    //   * solid regions and their boundaries are preserved.
    // A numeric check on a noisy frame (docs/mode17_compare.py) shows the
    // OR version leaves white speckle in mid-grey areas while the
    // majority version comes out clean.
    wire [3:0] cl0_cnt = {3'b0, d_t_l} + {3'b0, d_t_c} + {3'b0, d_t_r} +
                         {3'b0, d_m_l} + {3'b0, d_m_c} + {3'b0, d_m_r} +
                         {3'b0, d_b_l} + {3'b0, d_b_c} + {3'b0, d_b_r};
    wire [3:0] cl1_cnt = {3'b0, d_t_c} + {3'b0, d_t_r} + {3'b0, d_t_r2} +
                         {3'b0, d_m_c} + {3'b0, d_m_r} + {3'b0, d_m_r2} +
                         {3'b0, d_b_c} + {3'b0, d_b_r} + {3'b0, d_b_r2};
    wire cl0 = win_valid && (cl0_cnt >= 4'd5);
    wire cl1 = win_valid && (cl1_cnt >= 4'd5);

    // the region-fill comb is evaluated one stage earlier than the
    // morphological-edge comb (R1 here, R3b there), so it is pushed
    // through exactly one more register to land in the same beat as
    // mrp0_s / mrp1_s at the mode mux.
    reg cl0_r2, cl1_r2, cl0_r3a, cl1_r3a, cl0_r3, cl1_r3, cl0_s, cl1_s;

    // the smoothed luma travels the same four register stages so that it
    // reaches the mode mux in the same beat as the region-fill bits and
    // the mode-18 outline.
    reg [7:0] mean0_r3a, mean1_r3a;
    reg [7:0] mean0_r3, mean1_r3, mean0_s,  mean1_s;

    // mode-18 outline bits: max9/min9 are registered at R2, the subtract
    // and compare run in the R2 comb region, and the result then takes the
    // same three remaining stages as cl0/cl1 so everything lands together.
    reg ce0_r3a, ce1_r3a, ce0_r3, ce1_r3, ce0_s, ce1_s;

    //==================================================================
    // v1.10 rev-B MODE 18 : flat-colour fill, now in the LUMA domain
    //
    //   Board test (2026-10-05) showed saturated red/green speckle: the
    //   first cut quantised R, G and B INDEPENDENTLY to four levels, so
    //   a few LSB of sensor noise near a bin boundary flipped one channel
    //   by 64 and turned plain noise into loud colour patches.
    //
    //   rev-B quantises only the LUMA and keeps the chroma the pixel had:
    //
    //     Yq      = mean9(window) & 0xE0          // smoothed, 8 luma levels
    //     d_c     = p_c - gray(pixel)             // per-pixel chroma offset
    //     dq_c    = d_c rounded down to 16        // coarse chroma, 16 levels
    //     out_c   = clamp(Yq + dq_c)              // flat blocks, true hue
    //
    //   Luma noise therefore lands in a few big flat levels instead of
    //   flipping channels, and the hue of every pixel is preserved.
    //==================================================================
    function [23:0] flat_luma;
        input [23:0]       p;
        input [7:0]        gy;
        input [7:0]        yq;
        reg signed [9:0]   dr, dg, db;
        reg signed [23:0]  sr, sg, sb;
        begin
            dr = $signed({1'b0, p[23:16]}) - $signed({1'b0, gy});
            dg = $signed({1'b0, p[15: 8]}) - $signed({1'b0, gy});
            db = $signed({1'b0, p[7:  0]}) - $signed({1'b0, gy});
            dr = {dr[9], dr[8:4], 4'b0000};   // chroma -> 16 levels
            dg = {dg[9], dg[8:4], 4'b0000};
            db = {db[9], db[8:4], 4'b0000};
            sr = $signed({16'd0, yq}) + dr;
            sg = $signed({16'd0, yq}) + dg;
            sb = $signed({16'd0, yq}) + db;
            flat_luma = { clamp8(sr), clamp8(sg), clamp8(sb) };
        end
    endfunction

    //==================================================================
    // Stage R2 : register window results, RGB, gray, YUV, side-bands
    //==================================================================
    reg signed [11:0] sx0_r, sx1_r, sy0_r, sy1_r;
    reg [10:0]        em_pos0_r, em_neg0_r, em_pos1_r, em_neg1_r;
    reg [7:0] rm0_0_r, rm0_1_r, rm0_2_r, rm1_0_r, rm1_1_r, rm1_2_r;
    reg [7:0] rn0_0_r, rn0_1_r, rn0_2_r, rn1_0_r, rn1_1_r, rn1_2_r;
    reg               win_valid_r;

    // R2 combinational region: smoothed luma of mode 18 and the local
    // contrast used as its outline.  Both are built only from values that
    // are already registered at this point (sum0/sum1_r2 and rm*/rn*_*_r),
    // so nothing new hangs off the DRAM -> window path.
    wire [7:0] mean0_r2c = win_valid_r ? mean_of_sum(sum0_r2) : 8'd0;
    wire [7:0] mean1_r2c = win_valid_r ? mean_of_sum(sum1_r2) : 8'd0;

    wire [7:0] mx0 = max2(rm0_0_r, max2(rm0_1_r, rm0_2_r));
    wire [7:0] mn0 = min2(rn0_0_r, min2(rn0_1_r, rn0_2_r));
    wire [7:0] mx1 = max2(rm1_0_r, max2(rm1_1_r, rm1_2_r));
    wire [7:0] mn1 = min2(rn1_0_r, min2(rn1_1_r, rn1_2_r));

    wire [7:0] rng0 = mx0 - mn0;
    wire [7:0] rng1 = mx1 - mn1;
    wire       ce0  = win_valid_r && (rng0 >= I_cartoon_th);
    wire       ce1  = win_valid_r && (rng1 >= I_cartoon_th);
    reg [95:0] rgb_r2;
    reg [7:0]  gray_r2_0, gray_r2_1, gray_r2_2, gray_r2_3;
    reg        tl_r2, tu_r2, tv_r2;

    // ---- YUV, computed from rgb_r1 (reference coefficients) ----------
    // Each colour difference is built as a balanced adder tree instead of
    // a left-associative chain of nine terms.  The chain form used eight
    // cascaded 18-bit adders and was the critical path of the design.
    //   66 = 64+2 ; 129 = 128+1 ; 25 = 16+8+1
    function [7:0] yuv_y;
        input [7:0] r, g, b;
        reg [17:0] er, eg, eb, yr, yg, yb, s;
        begin
            er = {10'd0, r}; eg = {10'd0, g}; eb = {10'd0, b};
            yr = (er << 6) + (er << 1);                       //  66*r
            yg = (eg << 7) + eg;                              // 129*g
            yb = (eb << 4) + (eb << 3) + eb;                  //  25*b
            s  = (yr + yg) + (yb + 18'd128);
            yuv_y = s[15:8] + 8'd16;
        end
    endfunction

    //   38 = 32+4+2 ; 74 = 64+8+2 ; 112 = 64+32+16
    function signed [8:0] yuv_u;
        input [7:0] r, g, b;
        reg signed [17:0] er, eg, eb, ur, ug, ub, s;
        begin
            er = {10'd0, r}; eg = {10'd0, g}; eb = {10'd0, b};
            ur = (er <<< 5) + (er <<< 2) + (er <<< 1);        //  38*r
            ug = (eg <<< 6) + (eg <<< 3) + (eg <<< 1);        //  74*g
            ub = (eb <<< 6) + (eb <<< 5) + (eb <<< 4);        // 112*b
            s  = (ub - (ur + ug)) + 18'sd128;   // -38*r - 74*g +112*b
            s  = s >>> 8;
            yuv_u = s[8:0] + 9'sd128;
        end
    endfunction

    //   112 = 64+32+16 ; 94 = 64+16+8+4+2 ; 18 = 16+2
    function signed [8:0] yuv_v;
        input [7:0] r, g, b;
        reg signed [17:0] er, eg, eb, vr, vg, vb, s;
        begin
            er = {10'd0, r}; eg = {10'd0, g}; eb = {10'd0, b};
            vr = (er <<< 6) + (er <<< 5) + (er <<< 4);        // 112*r
            vg = (eg <<< 6) + (eg <<< 4) + (eg <<< 3)
               + (eg <<< 2) + (eg <<< 1);                     //  94*g
            vb = (eb <<< 4) + (eb <<< 1);                     //  18*b
            s  = (vr - (vg + vb)) + 18'sd128;
            s  = s >>> 8;
            yuv_v = s[8:0] + 9'sd128;
        end
    endfunction

    wire [7:0]        y_r1_0 = yuv_y(rgb_r1[95:88], rgb_r1[87:80], rgb_r1[79:72]);
    wire [7:0]        y_r1_1 = yuv_y(rgb_r1[71:64], rgb_r1[63:56], rgb_r1[55:48]);
    wire [7:0]        y_r1_2 = yuv_y(rgb_r1[47:40], rgb_r1[39:32], rgb_r1[31:24]);
    wire [7:0]        y_r1_3 = yuv_y(rgb_r1[23:16], rgb_r1[15:8],  rgb_r1[7:0]);
    wire signed [8:0] u_r1_0 = yuv_u(rgb_r1[95:88], rgb_r1[87:80], rgb_r1[79:72]);
    wire signed [8:0] u_r1_1 = yuv_u(rgb_r1[71:64], rgb_r1[63:56], rgb_r1[55:48]);
    wire signed [8:0] u_r1_2 = yuv_u(rgb_r1[47:40], rgb_r1[39:32], rgb_r1[31:24]);
    wire signed [8:0] u_r1_3 = yuv_u(rgb_r1[23:16], rgb_r1[15:8],  rgb_r1[7:0]);
    wire signed [8:0] v_r1_0 = yuv_v(rgb_r1[95:88], rgb_r1[87:80], rgb_r1[79:72]);
    wire signed [8:0] v_r1_1 = yuv_v(rgb_r1[71:64], rgb_r1[63:56], rgb_r1[55:48]);
    wire signed [8:0] v_r1_2 = yuv_v(rgb_r1[47:40], rgb_r1[39:32], rgb_r1[31:24]);
    wire signed [8:0] v_r1_3 = yuv_v(rgb_r1[23:16], rgb_r1[15:8],  rgb_r1[7:0]);

    reg [7:0]        y_r2_0, y_r2_1, y_r2_2, y_r2_3;
    reg signed [8:0] u_r2_0, u_r2_1, u_r2_2, u_r2_3;
    reg signed [8:0] v_r2_0, v_r2_1, v_r2_2, v_r2_3;

    always @(posedge I_clk_csi or negedge I_rst_n) begin
        if (!I_rst_n) begin
            sx0_r <= 12'sd0; sx1_r <= 12'sd0;
            sy0_r <= 12'sd0; sy1_r <= 12'sd0;
            em_pos0_r <= 11'd0; em_neg0_r <= 11'd0;
            em_pos1_r <= 11'd0; em_neg1_r <= 11'd0;
            rm0_0_r <= 8'd0; rm0_1_r <= 8'd0; rm0_2_r <= 8'd0;
            rm1_0_r <= 8'd0; rm1_1_r <= 8'd0; rm1_2_r <= 8'd0;
            rn0_0_r <= 8'd0; rn0_1_r <= 8'd0; rn0_2_r <= 8'd0;
            rn1_0_r <= 8'd0; rn1_1_r <= 8'd0; rn1_2_r <= 8'd0;
            win_valid_r <= 1'b0;
            rgb_r2 <= 96'd0;
            gray_r2_0 <= 8'd0; gray_r2_1 <= 8'd0;
            gray_r2_2 <= 8'd0; gray_r2_3 <= 8'd0;
            y_r2_0 <= 8'd0; y_r2_1 <= 8'd0; y_r2_2 <= 8'd0; y_r2_3 <= 8'd0;
            u_r2_0 <= 9'sd0; u_r2_1 <= 9'sd0; u_r2_2 <= 9'sd0; u_r2_3 <= 9'sd0;
            v_r2_0 <= 9'sd0; v_r2_1 <= 9'sd0; v_r2_2 <= 9'sd0; v_r2_3 <= 9'sd0;
            tl_r2 <= 1'b0; tu_r2 <= 1'b0; tv_r2 <= 1'b0;
            cl0_r2 <= 1'b0; cl1_r2 <= 1'b0;
            sum0_r2 <= 12'd0; sum1_r2 <= 12'd0;
        end else begin
            sx0_r <= sx0; sx1_r <= sx1; sy0_r <= sy0; sy1_r <= sy1;
            em_pos0_r <= em_pos0; em_neg0_r <= em_neg0;
            em_pos1_r <= em_pos1; em_neg1_r <= em_neg1;
            rm0_0_r <= rmax0_0; rm0_1_r <= rmax0_1; rm0_2_r <= rmax0_2;
            rm1_0_r <= rmax1_0; rm1_1_r <= rmax1_1; rm1_2_r <= rmax1_2;
            rn0_0_r <= rmin0_0; rn0_1_r <= rmin0_1; rn0_2_r <= rmin0_2;
            rn1_0_r <= rmin1_0; rn1_1_r <= rmin1_1; rn1_2_r <= rmin1_2;
            win_valid_r <= win_valid;
            rgb_r2 <= rgb_r1;
            gray_r2_0 <= gray_r1_0; gray_r2_1 <= gray_r1_1;
            gray_r2_2 <= gray_r1_2; gray_r2_3 <= gray_r1_3;
            y_r2_0 <= y_r1_0; y_r2_1 <= y_r1_1; y_r2_2 <= y_r1_2; y_r2_3 <= y_r1_3;
            u_r2_0 <= u_r1_0; u_r2_1 <= u_r1_1; u_r2_2 <= u_r1_2; u_r2_3 <= u_r1_3;
            v_r2_0 <= v_r1_0; v_r2_1 <= v_r1_1; v_r2_2 <= v_r1_2; v_r2_3 <= v_r1_3;
            tl_r2 <= tl_r1; tu_r2 <= tu_r1; tv_r2 <= tv_r1;
            cl0_r2 <= cl0; cl1_r2 <= cl1;
            sum0_r2 <= sum_w0; sum1_r2 <= sum_w1;
        end
    end

    //==================================================================
    // saturation helpers
    //==================================================================
    // The reference splits its chroma chain over several register stages.
    // This implementation splits it as well: "scale chroma" and
    // "reconstruct RGB" in one 9.66 ns stage does not close timing on
    // this device (multiply -> shift -> clamp -> 409/100/208/516 multiply
    // -> add -> shift -> clamp measured 11.4 ns).
    function signed [8:0] sat_scale;
        input signed [8:0] off;
        input [7:0]        factor;
        reg signed [17:0]  o, f, m;
        begin
            o = off;
            f = {10'd0, factor};       // 0..255, 18-bit signed
            m = o * f;                 // |max| = 112*192 = 21 504
            m = m >>> 7;
            if (m > 18'sd127)       sat_scale = 9'sd127;
            else if (m < -18'sd128) sat_scale = -9'sd128;
            else                    sat_scale = m[8:0];
        end
    endfunction

    //   298 = 256+32+8+2 ; 409 = 256+128+16+8+1 ; 100 = 64+32+4
    //   208 = 128+64+16  ; 516 = 512+4
    function [23:0] sat_recon;
        input signed [17:0] yterm_in;
        input signed [8:0] us;
        input signed [8:0] vs;
        reg signed [23:0]  yterm, usw, vsw, uterm, vterm, bterm, rs, gs, bs;
        begin
            // The luma product is registered with chroma scaling one stage earlier.
            yterm = yterm_in;
            usw = us;
            vsw = vs;
            // Constant products retain signed precision and do not need DSP blocks.
            uterm = ((usw <<< 6) + (usw <<< 5)) + (usw <<< 2);
            vterm = ((vsw <<< 7) + (vsw <<< 6)) + (vsw <<< 4);
            bterm = (usw <<< 9) + (usw <<< 2);
            rs = yterm + (vs * 24'sd409) + 24'sd128;
            gs = (yterm + 24'sd128) - (uterm + vterm);
            bs = (yterm + 24'sd128) + bterm;
            sat_recon = { clamp8(rs >>> 8), clamp8(gs >>> 8), clamp8(bs >>> 8) };
        end
    endfunction

    // ---- sepia (from rgb_r3a) ----------------------------------------
    // Same rebalancing as gray_of: build the three per-channel products
    // independently, then sum them.  Bit-for-bit identical, five adders
    // deep instead of eleven.
    //   101 = 64+32+4+1 ; 197 = 128+64+4+1 ; 48 = 32+16
    function [16:0] sepia_r;
        input [7:0] r, g, b;
        reg [16:0] pr, pg, pb;
        begin
            pr = (({9'd0,r} << 6) + ({9'd0,r} << 5)) + (({9'd0,r} << 2) + {9'd0,r});
            pg = (({9'd0,g} << 7) + ({9'd0,g} << 6)) + (({9'd0,g} << 2) + {9'd0,g});
            pb = ({9'd0,b} << 5) + ({9'd0,b} << 4);
            sepia_r = (pr + pg) + pb;
        end
    endfunction
    //   89 = 64+16+8+1 ; 176 = 128+32+16 ; 43 = 32+8+2+1
    function [16:0] sepia_g;
        input [7:0] r, g, b;
        reg [16:0] pr, pg, pb;
        begin
            pr = (({9'd0,r} << 6) + ({9'd0,r} << 4)) + (({9'd0,r} << 3) + {9'd0,r});
            pg = ({9'd0,g} << 7) + ({9'd0,g} << 5) + ({9'd0,g} << 4);
            pb = (({9'd0,b} << 5) + ({9'd0,b} << 3)) + (({9'd0,b} << 1) + {9'd0,b});
            sepia_g = (pr + pg) + pb;
        end
    endfunction
    //   70 = 64+4+2 ; 137 = 128+8+1 ; 34 = 32+2
    function [16:0] sepia_b;
        input [7:0] r, g, b;
        reg [16:0] pr, pg, pb;
        begin
            pr = ({9'd0,r} << 6) + ({9'd0,r} << 2) + ({9'd0,r} << 1);
            pg = ({9'd0,g} << 7) + ({9'd0,g} << 3) + {9'd0,g};
            pb = ({9'd0,b} << 5) + ({9'd0,b} << 1);
            sepia_b = (pr + pg) + pb;
        end
    endfunction

    function [7:0] sepia_sat;
        input [16:0] v;
        begin
            sepia_sat = (v > 17'd65280) ? 8'hFF : v[15:8];
        end
    endfunction

    function [23:0] sepia_px;
        input [23:0] p;
        begin
            sepia_px = { sepia_sat(sepia_r(p[23:16], p[15:8], p[7:0])),
                         sepia_sat(sepia_g(p[23:16], p[15:8], p[7:0])),
                         sepia_sat(sepia_b(p[23:16], p[15:8], p[7:0])) };
        end
    endfunction

    //==================================================================
    // Stage R3a : chroma scaling + pass-through of the R2 results
    //   Chroma scaling and luma/sepia partial products are evaluated here, so
    //   the final colour sums and clamping are left to
    //   R3b.  Everything else simply moves one stage further so that all
    //   data paths have the same depth.
    //==================================================================
    reg [95:0]        rgb_r3a;
    reg [31:0]        gray_r3a;
    // Registered partial products preserve the existing colour-path latency.
    reg signed [17:0] S_sat_yterm_r3a [0:3];
    reg [16:0] S_sepia_rg_r3a [0:11];
    reg [16:0] S_sepia_blue_r3a [0:11];
    integer S_sepia_lane;
    reg signed [8:0]  us_r3a_0, us_r3a_1, us_r3a_2, us_r3a_3;
    reg signed [8:0]  vs_r3a_0, vs_r3a_1, vs_r3a_2, vs_r3a_3;
    reg signed [11:0] sx0_a, sx1_a, sy0_a, sy1_a;
    reg [10:0]        em_pos0_a, em_neg0_a, em_pos1_a, em_neg1_a;
    reg [7:0] rm0_0_a, rm0_1_a, rm0_2_a, rm1_0_a, rm1_1_a, rm1_2_a;
    reg [7:0] rn0_0_a, rn0_1_a, rn0_2_a, rn1_0_a, rn1_1_a, rn1_2_a;
    reg       win_valid_a;
    reg       tl_r3a, tu_r3a, tv_r3a;

    always @(posedge I_clk_csi or negedge I_rst_n) begin
        if (!I_rst_n) begin
            rgb_r3a  <= 96'd0;
            gray_r3a <= 32'd0;
            for (S_sepia_lane = 0; S_sepia_lane < 4; S_sepia_lane = S_sepia_lane + 1) begin
                S_sat_yterm_r3a[S_sepia_lane] <= -18'sd4768;
                S_sepia_rg_r3a[3*S_sepia_lane] <= 17'd0;
                S_sepia_rg_r3a[3*S_sepia_lane+1] <= 17'd0;
                S_sepia_rg_r3a[3*S_sepia_lane+2] <= 17'd0;
                S_sepia_blue_r3a[3*S_sepia_lane] <= 17'd0;
                S_sepia_blue_r3a[3*S_sepia_lane+1] <= 17'd0;
                S_sepia_blue_r3a[3*S_sepia_lane+2] <= 17'd0;
            end
            us_r3a_0 <= 9'sd0; us_r3a_1 <= 9'sd0; us_r3a_2 <= 9'sd0; us_r3a_3 <= 9'sd0;
            vs_r3a_0 <= 9'sd0; vs_r3a_1 <= 9'sd0; vs_r3a_2 <= 9'sd0; vs_r3a_3 <= 9'sd0;
            sx0_a <= 12'sd0; sx1_a <= 12'sd0; sy0_a <= 12'sd0; sy1_a <= 12'sd0;
            em_pos0_a <= 11'd0; em_neg0_a <= 11'd0;
            em_pos1_a <= 11'd0; em_neg1_a <= 11'd0;
            rm0_0_a <= 8'd0; rm0_1_a <= 8'd0; rm0_2_a <= 8'd0;
            rm1_0_a <= 8'd0; rm1_1_a <= 8'd0; rm1_2_a <= 8'd0;
            rn0_0_a <= 8'd0; rn0_1_a <= 8'd0; rn0_2_a <= 8'd0;
            rn1_0_a <= 8'd0; rn1_1_a <= 8'd0; rn1_2_a <= 8'd0;
            win_valid_a <= 1'b0;
            tl_r3a <= 1'b0; tu_r3a <= 1'b0; tv_r3a <= 1'b0;
            cl0_r3a <= 1'b0; cl1_r3a <= 1'b0;
            mean0_r3a <= 8'd0; mean1_r3a <= 8'd0;
            ce0_r3a <= 1'b0; ce1_r3a <= 1'b0;
        end else begin            rgb_r3a  <= rgb_r2;
            gray_r3a <= {gray_r2_0, gray_r2_1, gray_r2_2, gray_r2_3};
            S_sat_yterm_r3a[0] <= ($signed({16'd0, y_r2_0}) - 24'sd16) * 24'sd298;
            S_sat_yterm_r3a[1] <= ($signed({16'd0, y_r2_1}) - 24'sd16) * 24'sd298;
            S_sat_yterm_r3a[2] <= ($signed({16'd0, y_r2_2}) - 24'sd16) * 24'sd298;
            S_sat_yterm_r3a[3] <= ($signed({16'd0, y_r2_3}) - 24'sd16) * 24'sd298;
            for (S_sepia_lane = 0; S_sepia_lane < 4; S_sepia_lane = S_sepia_lane + 1) begin
                S_sepia_rg_r3a[3*S_sepia_lane] <= sepia_r(rgb_r2[95-24*S_sepia_lane -: 8], rgb_r2[87-24*S_sepia_lane -: 8], 8'd0);
                S_sepia_rg_r3a[3*S_sepia_lane+1] <= sepia_g(rgb_r2[95-24*S_sepia_lane -: 8], rgb_r2[87-24*S_sepia_lane -: 8], 8'd0);
                S_sepia_rg_r3a[3*S_sepia_lane+2] <= sepia_b(rgb_r2[95-24*S_sepia_lane -: 8], rgb_r2[87-24*S_sepia_lane -: 8], 8'd0);
                S_sepia_blue_r3a[3*S_sepia_lane] <= ({9'd0,rgb_r2[79-24*S_sepia_lane -: 8]} << 5) + ({9'd0,rgb_r2[79-24*S_sepia_lane -: 8]} << 4);
                S_sepia_blue_r3a[3*S_sepia_lane+1] <= (({9'd0,rgb_r2[79-24*S_sepia_lane -: 8]} << 5) + ({9'd0,rgb_r2[79-24*S_sepia_lane -: 8]} << 3)) + (({9'd0,rgb_r2[79-24*S_sepia_lane -: 8]} << 1) + {9'd0,rgb_r2[79-24*S_sepia_lane -: 8]});
                S_sepia_blue_r3a[3*S_sepia_lane+2] <= ({9'd0,rgb_r2[79-24*S_sepia_lane -: 8]} << 5) + ({9'd0,rgb_r2[79-24*S_sepia_lane -: 8]} << 1);
            end
            us_r3a_0 <= sat_scale(u_r2_0 - 9'sd128, I_sat_factor);
            us_r3a_1 <= sat_scale(u_r2_1 - 9'sd128, I_sat_factor);
            us_r3a_2 <= sat_scale(u_r2_2 - 9'sd128, I_sat_factor);
            us_r3a_3 <= sat_scale(u_r2_3 - 9'sd128, I_sat_factor);
            vs_r3a_0 <= sat_scale(v_r2_0 - 9'sd128, I_sat_factor);
            vs_r3a_1 <= sat_scale(v_r2_1 - 9'sd128, I_sat_factor);
            vs_r3a_2 <= sat_scale(v_r2_2 - 9'sd128, I_sat_factor);
            vs_r3a_3 <= sat_scale(v_r2_3 - 9'sd128, I_sat_factor);
            sx0_a <= sx0_r; sx1_a <= sx1_r; sy0_a <= sy0_r; sy1_a <= sy1_r;
            em_pos0_a <= em_pos0_r; em_neg0_a <= em_neg0_r;
            em_pos1_a <= em_pos1_r; em_neg1_a <= em_neg1_r;
            rm0_0_a <= rm0_0_r; rm0_1_a <= rm0_1_r; rm0_2_a <= rm0_2_r;
            rm1_0_a <= rm1_0_r; rm1_1_a <= rm1_1_r; rm1_2_a <= rm1_2_r;
            rn0_0_a <= rn0_0_r; rn0_1_a <= rn0_1_r; rn0_2_a <= rn0_2_r;
            rn1_0_a <= rn1_0_r; rn1_1_a <= rn1_1_r; rn1_2_a <= rn1_2_r;
            win_valid_a <= win_valid_r;
            tl_r3a <= tl_r2; tu_r3a <= tu_r2; tv_r3a <= tv_r2;
            cl0_r3a <= cl0_r2; cl1_r3a <= cl1_r2;
            mean0_r3a <= mean0_r2c; mean1_r3a <= mean1_r2c;
            ce0_r3a <= ce0; ce1_r3a <= ce1;
        end
    end

    //==================================================================
    // Stage R3b : magnitude / emboss / morphology / sepia / saturation
    //==================================================================
    // |Gx| + |Gy|, saturate to 8 bit
    wire [11:0] ax0 = sx0_a[11] ? (~sx0_a + 12'sd1) : sx0_a;
    wire [11:0] ay0 = sy0_a[11] ? (~sy0_a + 12'sd1) : sy0_a;
    wire [11:0] ax1 = sx1_a[11] ? (~sx1_a + 12'sd1) : sx1_a;
    wire [11:0] ay1 = sy1_a[11] ? (~sy1_a + 12'sd1) : sy1_a;
    wire [12:0] mag0_w = {1'b0, ax0} + {1'b0, ay0};
    wire [12:0] mag1_w = {1'b0, ax1} + {1'b0, ay1};

    wire [7:0] sob0 = win_valid_a ? ((mag0_w > 13'd255) ? 8'hFF : mag0_w[7:0]) : 8'h00;
    wire [7:0] sob1 = win_valid_a ? ((mag1_w > 13'd255) ? 8'hFF : mag1_w[7:0]) : 8'h00;
    // the inverted form is gated as well, so an invalid window stays black
    // in every neighbourhood mode instead of turning white only in mode 10
    wire [7:0] snv0 = win_valid_a ? ~sob0 : 8'h00;
    wire [7:0] snv1 = win_valid_a ? ~sob1 : 8'h00;
    wire [7:0] bsb0 = win_valid_a ? ((mag0_w >= {5'd0, EDGE_RGB_TH}) ? 8'hFF : 8'h00) : 8'h00;
    wire [7:0] bsb1 = win_valid_a ? ((mag1_w >= {5'd0, EDGE_RGB_TH}) ? 8'hFF : 8'h00) : 8'h00;

    // emboss: clamp((pos - neg)*2 + 128)
    wire signed [11:0]  emd0 = $signed({1'b0, em_pos0_a}) - $signed({1'b0, em_neg0_a});
    wire signed [11:0]  emd1 = $signed({1'b0, em_pos1_a}) - $signed({1'b0, em_neg1_a});
    wire signed [23:0]  emd0_w = emd0;
    wire signed [23:0]  emd1_w = emd1;
    wire signed [23:0]  emo0 = (emd0_w <<< 1) + 24'sd128;
    wire signed [23:0]  emo1 = (emd1_w <<< 1) + 24'sd128;
    wire [7:0] emb0 = win_valid_a ? clamp8(emo0) : 8'h00;
    wire [7:0] emb1 = win_valid_a ? clamp8(emo1) : 8'h00;

    // morphology
    wire [7:0] dil0 = win_valid_a ? max2(rm0_0_a, max2(rm0_1_a, rm0_2_a)) : 8'h00;
    wire [7:0] dil1 = win_valid_a ? max2(rm1_0_a, max2(rm1_1_a, rm1_2_a)) : 8'h00;
    wire [7:0] ero0 = win_valid_a ? min2(rn0_0_a, min2(rn0_1_a, rn0_2_a)) : 8'h00;
    wire [7:0] ero1 = win_valid_a ? min2(rn1_0_a, min2(rn1_1_a, rn1_2_a)) : 8'h00;
    wire [7:0] mrp0 = win_valid_a ? (dil0 - ero0) : 8'h00;
    wire [7:0] mrp1 = win_valid_a ? (dil1 - ero1) : 8'h00;

    // saturation reconstruction
    wire [23:0] sat0 = sat_recon(S_sat_yterm_r3a[0], us_r3a_0, vs_r3a_0);
    wire [23:0] sat1 = sat_recon(S_sat_yterm_r3a[1], us_r3a_1, vs_r3a_1);
    wire [23:0] sat2 = sat_recon(S_sat_yterm_r3a[2], us_r3a_2, vs_r3a_2);
    wire [23:0] sat3 = sat_recon(S_sat_yterm_r3a[3], us_r3a_3, vs_r3a_3);

    // sepia
    wire [23:0] sep0 = { sepia_sat(S_sepia_rg_r3a[0] + S_sepia_blue_r3a[0]), sepia_sat(S_sepia_rg_r3a[1] + S_sepia_blue_r3a[1]), sepia_sat(S_sepia_rg_r3a[2] + S_sepia_blue_r3a[2]) };
    wire [23:0] sep1 = { sepia_sat(S_sepia_rg_r3a[3] + S_sepia_blue_r3a[3]), sepia_sat(S_sepia_rg_r3a[4] + S_sepia_blue_r3a[4]), sepia_sat(S_sepia_rg_r3a[5] + S_sepia_blue_r3a[5]) };
    wire [23:0] sep2 = { sepia_sat(S_sepia_rg_r3a[6] + S_sepia_blue_r3a[6]), sepia_sat(S_sepia_rg_r3a[7] + S_sepia_blue_r3a[7]), sepia_sat(S_sepia_rg_r3a[8] + S_sepia_blue_r3a[8]) };
    wire [23:0] sep3 = { sepia_sat(S_sepia_rg_r3a[9] + S_sepia_blue_r3a[9]), sepia_sat(S_sepia_rg_r3a[10] + S_sepia_blue_r3a[10]), sepia_sat(S_sepia_rg_r3a[11] + S_sepia_blue_r3a[11]) };

    reg [95:0] rgb_r3, sepia_r3, sat_r3;
    reg [31:0] gray_r3;                 // 4 x 8-bit per-pixel gray
    reg [7:0]  sob0_r, sob1_r, snv0_r, snv1_r, bsb0_r, bsb1_r;
    reg [7:0]  emb0_r, emb1_r, dil0_r, dil1_r, ero0_r, ero1_r, mrp0_r, mrp1_r;
    reg        tl_r3, tu_r3, tv_r3;

    // The neighbourhood path is one register shorter than the colour path:
    //   colour       : I_tdata -> rgb_r1 -> rgb_r2 -> rgb_r3a -> rgb_r3
    //   neighbourhood: window(comb) -> sx0_r -> sx0_a -> sob0_r
    // so the final neighbourhood values need one more stage before they
    // can share the mode mux with the colour pixels of the same beat.
    reg [7:0]  sob0_s, sob1_s, snv0_s, snv1_s, bsb0_s, bsb1_s;
    reg [7:0]  emb0_s, emb1_s, dil0_s, dil1_s, ero0_s, ero1_s, mrp0_s, mrp1_s;

    always @(posedge I_clk_csi or negedge I_rst_n) begin
        if (!I_rst_n) begin
            rgb_r3 <= 96'd0;
            gray_r3 <= 32'd0;
            sepia_r3 <= 96'd0;
            sat_r3 <= 96'd0;
            sob0_r <= 8'd0; sob1_r <= 8'd0;
            snv0_r <= 8'd0; snv1_r <= 8'd0;
            bsb0_r <= 8'd0; bsb1_r <= 8'd0;
            emb0_r <= 8'd0; emb1_r <= 8'd0;
            dil0_r <= 8'd0; dil1_r <= 8'd0;
            ero0_r <= 8'd0; ero1_r <= 8'd0;
            mrp0_r <= 8'd0; mrp1_r <= 8'd0;
            tl_r3 <= 1'b0; tu_r3 <= 1'b0; tv_r3 <= 1'b0;
            cl0_r3 <= 1'b0; cl1_r3 <= 1'b0;
            mean0_r3 <= 8'd0; mean1_r3 <= 8'd0;
            ce0_r3 <= 1'b0; ce1_r3 <= 1'b0;
        end else begin
            rgb_r3   <= rgb_r3a;
            gray_r3  <= gray_r3a;
            sepia_r3 <= {sep0, sep1, sep2, sep3};
            sat_r3   <= {sat0, sat1, sat2, sat3};
            sob0_r <= sob0; sob1_r <= sob1;
            snv0_r <= snv0; snv1_r <= snv1;
            bsb0_r <= bsb0; bsb1_r <= bsb1;
            emb0_r <= emb0; emb1_r <= emb1;
            dil0_r <= dil0; dil1_r <= dil1;
            ero0_r <= ero0; ero1_r <= ero1;
            mrp0_r <= mrp0; mrp1_r <= mrp1;
            tl_r3 <= tl_r3a; tu_r3 <= tu_r3a; tv_r3 <= tv_r3a;
            cl0_r3 <= cl0_r3a; cl1_r3 <= cl1_r3a;
            mean0_r3 <= mean0_r3a; mean1_r3 <= mean1_r3a;
            ce0_r3 <= ce0_r3a; ce1_r3 <= ce1_r3a;
        end
    end

    always @(posedge I_clk_csi or negedge I_rst_n) begin
        if (!I_rst_n) begin
            sob0_s <= 8'd0; sob1_s <= 8'd0;
            snv0_s <= 8'd0; snv1_s <= 8'd0;
            bsb0_s <= 8'd0; bsb1_s <= 8'd0;
            emb0_s <= 8'd0; emb1_s <= 8'd0;
            dil0_s <= 8'd0; dil1_s <= 8'd0;
            ero0_s <= 8'd0; ero1_s <= 8'd0;
            mrp0_s <= 8'd0; mrp1_s <= 8'd0;
            cl0_s <= 1'b0; cl1_s <= 1'b0;
            mean0_s <= 8'd0; mean1_s <= 8'd0;
            ce0_s <= 1'b0; ce1_s <= 1'b0;
        end else begin
            sob0_s <= sob0_r; sob1_s <= sob1_r;
            snv0_s <= snv0_r; snv1_s <= snv1_r;
            bsb0_s <= bsb0_r; bsb1_s <= bsb1_r;
            emb0_s <= emb0_r; emb1_s <= emb1_r;
            dil0_s <= dil0_r; dil1_s <= dil1_r;
            ero0_s <= ero0_r; ero1_s <= ero1_r;
            mrp0_s <= mrp0_r; mrp1_s <= mrp1_r;
            cl0_s <= cl0_r3; cl1_s <= cl1_r3;
            mean0_s <= mean0_r3; mean1_s <= mean1_r3;
            ce0_s <= ce0_r3; ce1_s <= ce1_r3;
        end
    end

    //==================================================================
    // Stage R4 : per-pixel colour modes + mode mux + output register
    //==================================================================
    wire [23:0] r3_p0 = rgb_r3[95:72];
    wire [23:0] r3_p1 = rgb_r3[71:48];
    wire [23:0] r3_p2 = rgb_r3[47:24];
    wire [23:0] r3_p3 = rgb_r3[23: 0];

    wire [7:0] gr3_0 = gray_r3[31:24];
    wire [7:0] gr3_1 = gray_r3[23:16];
    wire [7:0] gr3_2 = gray_r3[15: 8];
    wire [7:0] gr3_3 = gray_r3[ 7: 0];

    function [23:0] px_gray;
        input [7:0] g;
        begin px_gray = {g, g, g}; end
    endfunction

    function [23:0] px_bin;
        input [7:0] g;
        begin px_bin = (g >= I_bin_th) ? 24'hFFFFFF : 24'h000000; end
    endfunction

    function [23:0] px_post;
        input [23:0] p;
        begin px_post = p & 24'hE0E0E0; end
    endfunction

    function [23:0] px_red;
        input [23:0] p;
        begin px_red = {p[23:16], 8'h00, 8'h00}; end
    endfunction

    function [23:0] px_bright;
        input [23:0]       p;
        input signed [8:0] lvl;
        reg signed [23:0]  rr, gg, bb, ll;
        begin
            ll = lvl;
            rr = $signed({16'd0, p[23:16]}) + ll;
            gg = $signed({16'd0, p[15: 8]}) + ll;
            bb = $signed({16'd0, p[7:  0]}) + ll;
            px_bright = { clamp8(rr), clamp8(gg), clamp8(bb) };
        end
    endfunction

    // heat map (reference LUT)
    function [23:0] px_heat;
        input [7:0] g;
        reg [7:0] rr, gg, bb;
        begin
            if (g < 8'd64) begin
                rr = 8'd0;               gg = g << 2;                        bb = 8'd255;
            end else if (g < 8'd128) begin
                rr = 8'd0;               gg = 8'd255;  bb = 8'd255 - ((g - 8'd64) << 2);
            end else if (g < 8'd192) begin
                rr = (g - 8'd128) << 2;  gg = 8'd255;  bb = 8'd0;
            end else begin
                rr = 8'd255;             gg = 8'd255 - ((g - 8'd192) << 2); bb = 8'd0;
            end
            px_heat = {rr, gg, bb};
        end
    endfunction

    function [23:0] mono3;
        input [7:0] v;
        begin mono3 = {v, v, v}; end
    endfunction

    //------------------------------------------------------------------
    // v1.10 rev-B MODE 17 / MODE 18 helpers
    //------------------------------------------------------------------
    wire [7:0] cl0_px = cl0_s ? 8'hFF : 8'h00;
    wire [7:0] cl1_px = cl1_s ? 8'hFF : 8'h00;

    // smoothed luma of the two window positions, quantised to eight
    // levels; the pixel keeps its own chroma (see flat_luma above).
    wire [7:0] yq0 = mean0_s & 8'hE0;
    wire [7:0] yq1 = mean1_s & 8'hE0;

    wire [23:0] cart0 = ce0_s ? 24'h000000 : flat_luma(r3_p0, gr3_0, yq0);
    wire [23:0] cart1 = ce0_s ? 24'h000000 : flat_luma(r3_p1, gr3_1, yq0);
    wire [23:0] cart2 = ce1_s ? 24'h000000 : flat_luma(r3_p2, gr3_2, yq1);
    wire [23:0] cart3 = ce1_s ? 24'h000000 : flat_luma(r3_p3, gr3_3, yq1);

    reg [95:0] sel_data;

    always @(*) begin
        case (I_mode)
            M_SOBEL  : sel_data = { mono3(sob0_s), mono3(sob0_s),
                                    mono3(sob1_s), mono3(sob1_s) };
            M_SOBEL_INV : sel_data = { mono3(snv0_s), mono3(snv0_s),
                                       mono3(snv1_s), mono3(snv1_s) };
            M_EMBOSS : sel_data = { mono3(emb0_s), mono3(emb0_s),
                                    mono3(emb1_s), mono3(emb1_s) };
            M_ERODE  : sel_data = { mono3(ero0_s), mono3(ero0_s),
                                    mono3(ero1_s), mono3(ero1_s) };
            M_DILATE : sel_data = { mono3(dil0_s), mono3(dil0_s),
                                    mono3(dil1_s), mono3(dil1_s) };
            M_MORPH  : sel_data = { mono3(mrp0_s), mono3(mrp0_s),
                                    mono3(mrp1_s), mono3(mrp1_s) };
            M_BINSOBEL : sel_data = { mono3(bsb0_s), mono3(bsb0_s),
                                      mono3(bsb1_s), mono3(bsb1_s) };
            M_CLOSE  : sel_data = { mono3(cl0_px), mono3(cl0_px),
                                    mono3(cl1_px), mono3(cl1_px) };
            M_CARTOON : sel_data = { cart0, cart1, cart2, cart3 };
            default  : sel_data = rgb_r3;
        endcase
    end

    always @(posedge I_clk_csi or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tvalid <= 1'b0;
            O_tdata  <= 96'd0;
        end else begin
            O_tlast  <= tl_r3;
            O_tuser  <= tu_r3;
            O_tvalid <= tv_r3;
            O_tdata  <= tv_r3 ? sel_data : 96'd0;
        end
    end

    //==================================================================
    // line-buffer write side
    //==================================================================
    always @(posedge I_clk_csi or negedge I_rst_n) begin
        if (!I_rst_n) begin
            x_grp  <= 10'd0;
            top_p2 <= 8'd0; top_p1 <= 8'd0;
            mid_p2 <= 8'd0; mid_p1 <= 8'd0;
            bot_p2 <= 8'd0; bot_p1 <= 8'd0;
            d_top_p2 <= 1'b0; d_top_p1 <= 1'b0;
            d_mid_p2 <= 1'b0; d_mid_p1 <= 1'b0;
            d_bot_p2 <= 1'b0; d_bot_p1 <= 1'b0;
            // line memories are intentionally NOT cleared: a reset loop
            // maps the buffers into slice registers on this device.
            // The first two rows are masked by win_valid anyway; the
            // v1.10 dilate memories are additionally protected because
            // the value written into them is already win_valid gated.
        end else begin
            if (tv_r1) begin
                line2_mem[x_grp] <= mid_word;
                line1_mem[x_grp] <= bot_word;
                dline2_mem[x_grp] <= d_mid_word;
                dline1_mem[x_grp] <= d_bot_word;

                top_p2 <= top0;  top_p1 <= top1;
                mid_p2 <= mid0;  mid_p1 <= mid1;
                bot_p2 <= pair0_r; bot_p1 <= pair1_r;

                d_top_p2 <= d_top0; d_top_p1 <= d_top1;
                d_mid_p2 <= d_mid0; d_mid_p1 <= d_mid1;
                d_bot_p2 <= dilA_r; d_bot_p1 <= dilB_r;

                if (tu_r1) begin
                    // first beat of a row writes group 0 at address 0
                    x_grp <= 10'd1;
                end else if (x_grp == IMG_WIDTH_GRP - 1) begin
                    x_grp <= 10'd0;
                    top_p2 <= 8'd0; top_p1 <= 8'd0;
                    mid_p2 <= 8'd0; mid_p1 <= 8'd0;
                    bot_p2 <= 8'd0; bot_p1 <= 8'd0;
                    d_top_p2 <= 1'b0; d_top_p1 <= 1'b0;
                    d_mid_p2 <= 1'b0; d_mid_p1 <= 1'b0;
                    d_bot_p2 <= 1'b0; d_bot_p1 <= 1'b0;
                end else begin
                    x_grp <= x_grp + 10'd1;
                end
            end
        end
    end

endmodule
