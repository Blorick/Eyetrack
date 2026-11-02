create_clock -name {sys_clk_50m} -period 20.000 -waveform {0.000 10.000} [get_ports {I_sys_clk}]
create_clock -name {mipi_rx_ck_pad} -period 2.415 -waveform {0.000 1.208} [get_nets {IO_rx_clk_pad_p}]
create_clock -name {eth_rxc} -period 8.000 -waveform {0.000 4.000} [get_ports {I_eth_rxc}]
# Virtual data-launch reference: transitions are centered 2 ns before each
# physical RXC capture edge.  Its 2/6 ns edges represent high/low nibbles
# respectively; they do not change the physical RXC or IDDR nibble mapping.
create_clock -name {eth_rx_data_ref} -period 8.000 -waveform {2.000 6.000}

derive_clocks

rename_clock -name {pll_clk_100m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {pll_clk_24m} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[1]}]
rename_clock -name {HDMI_PIXEL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[4]}]
rename_clock -name {HDMI_SERIAL_CLK} [get_clocks {u_PLL/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[5]}]
rename_clock -name {eth_rx_pll_c0} [get_clocks {u_ph1p_eth_shell/u_eth_rx_pll/ph1p_phy_pll_wrapper_0cd802a517b1_Inst/u_PH1P_PHY_PLL.clkc[0]}]
rename_clock -name {eth_rx_pll_c1} [get_clocks {u_ph1p_eth_shell/u_eth_rx_pll/ph1p_phy_pll_wrapper_0cd802a517b1_Inst/u_PH1P_PHY_PLL.clkc[1]}]

rename_clock -name {MIPI_RX_BYTE_CLK} [get_clocks {u_mipi_dphy_rx_ph1p_mipiio_wrapper/u_ph1p_mipiio_rx_wrapper/u_PH1P_LOGIC_DPHY_MIPI_RX.o_fabric_div4_8_clk}]


set_clock_groups -asynchronous \
    -group [get_clocks {mipi_rx_ck_pad MIPI_RX_BYTE_CLK}] \
    -group [get_clocks {sys_clk_50m pll_clk_100m pll_clk_24m HDMI_PIXEL_CLK HDMI_SERIAL_CLK}] \
    -group [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/ddr_clk u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}]


#---------------------------------------------------------------------------
# AE / AG 数值：pll_clk_24m -> HDMI_PIXEL_CLK（2026-10-02 v1.5 新增 OSD 显示时引入）
#
# ae_set 在 pll_clk_24m 域输出当前曝光/增益，hdmi_mixer 在 HDMI_PIXEL_CLK 域
# 用它们显示 "AE:nnnn AG:nnn"。这是准静态多比特值（AE 最快 4.2 ms 变一次，
# AG 134 ms 变一次），接收侧是标准两级同步器（S_ae_s1/S_ae_s2）。
# 两个时钟同属 PLL 输出、边沿几乎重合，按同步路径检查时预算只有 0.653 ns，
# 物理上无法满足。按 CDC 常规做法用 set_max_delay -datapath_only 约束：
# 只保证各数据位在一个目的时钟周期内稳定到达，不做建立/保持检查。
# 目的时钟周期 = 13.333 ns（75 MHz），这里取 10 ns 作为位间偏斜上限。
#---------------------------------------------------------------------------
set_max_delay -from [get_clocks {pll_clk_24m}] -to [get_clocks {HDMI_PIXEL_CLK}] \
    -datapath_only 10.000

#---------------------------------------------------------------------------
# W4 自动曝光闭环：显示域亮度统计 S_y_avg 75M -> 24M（准静态，每帧一次）。
#
# dsp_luma_stat 在 HDMI_PIXEL_CLK 域输出 O_y_avg，经 top 两级同步器
# S_y_avg_meta/S_y_avg_24m 进入 ae_auto_loop（pll_clk_24m 域）。与上面的
# AE/AG 反向，同属 PLL 输出、边沿几乎重合的同步路径（预算 0.653 ns 无法满足），
# 按 CDC 常规做法用 set_max_delay -datapath_only：只保证各数据位在一个目的时钟
# 周期内稳定到达，不做建立/保持检查。目的时钟周期 = 41.666 ns（24 MHz）。
#---------------------------------------------------------------------------
set_max_delay -from [get_clocks {HDMI_PIXEL_CLK}] -to [get_clocks {pll_clk_24m}] \
    -datapath_only 41.666

#---------------------------------------------------------------------------
# Board POR distribution (2026-10-11, two-board master).
#
# ctl_ph1p_power_on_reset shifts the board reset out of the 50 MHz domain and
# the top ANDs it with PLL lock before feeding the asynchronous reset pins of
# every derived domain (S_rst_n / S_hdmi_rst_n, fanout 8478).  The release of
# that reset has no synchronous relationship to pll_clk_24m or HDMI_SERIAL_CLK,
# so TD's recovery/removal checks on it are not meaningful: at the gate stage
# they produced 228 + 44 failing endpoints (SWNS -1.661 ns / STNS -258.856 ns)
# that no placement can fix.  The only paths between these clock pairs are that
# reset distribution; every functional crossing has its own synchronizer, so the
# pair is cut instead of the check being left failing.
#---------------------------------------------------------------------------
set_false_path -from [get_clocks {sys_clk_50m}] \
               -to   [get_clocks {pll_clk_24m HDMI_SERIAL_CLK}]



# PH1P master Ethernet clock boundary.  The PLL IP derives 125 MHz TX
# clocks from I_sys_clk/C4; I_eth_rxc is the independent recovered PHY clock.
set_clock_groups -asynchronous -group [get_clocks {sys_clk_50m pll_clk_100m pll_clk_24m HDMI_PIXEL_CLK HDMI_SERIAL_CLK}] -group [get_clocks {eth_rxc eth_rx_pll_c0 eth_rx_pll_c1 eth_rx_data_ref}]
# +/-1.5 ns about virtual 2/6 ns gives transitions at 0.5..3.5/4.5..7.5 ns,
# exactly -3.5..-0.5 ns before the corresponding physical 4/8 ns RXC edges.
# This keeps setup/hold associated with adjacent data transitions even when
# the RX PLL rising edge is after 0 ns and falling edge is before 4 ns.
#
# Hold (min) is -1.4 ns, not -1.5 ns (2026-10-24).  Routed STA in v3.1 reported
# 5 hold endpoints at HWNS -0.011 ns on the RX data/ctl capture (eth_rx_data_ref
# -> eth_rx_pll_c1): the IDDR capture clock reaches the pad through a PLL+BUFG+
# SCLK path (~1.58 ns slow-corner insertion) while the data only crosses the
# 0.79 ns input pad, leaving ~0.79 ns skew against a 2.276 ns hold window.
# Setup on the same pins is +0.063 ns and is untouched by -min.  Shaving 0.1 ns
# off the assumed PCB skew (0.5 -> 0.4 ns) clears the hold to ~+0.09 ns without
# relaxing the setup budget.  No RTL change; the capture edge and nibble
# mapping are unchanged.
set_input_delay -clock [get_clocks {eth_rx_data_ref}] -max 1.500 [get_ports {I_eth_rxd[*] I_eth_rx_ctl}]
set_input_delay -clock [get_clocks {eth_rx_data_ref}] -min -1.400 [get_ports {I_eth_rxd[*] I_eth_rx_ctl}]
set_input_delay -clock [get_clocks {eth_rx_data_ref}] -max -clock_fall -add_delay 1.500 [get_ports {I_eth_rxd[*] I_eth_rx_ctl}]
set_input_delay -clock [get_clocks {eth_rx_data_ref}] -min -clock_fall -add_delay -1.400 [get_ports {I_eth_rxd[*] I_eth_rx_ctl}]

#---------------------------------------------------------------------------
# RGMII external timing contract (2026-10-18).
#
# HX1P35A uses YT8511C/H.  The board figure pulls LED_10_100 high, which is
# the YT8511 RXC-delay strap; at 1 Gb/s the matching EXT0C[0] contract is the
# approximately 1.8 ns internal RXC delay.  TXD/TX_CTL are launched by the
# Ethernet PLL C0 (0 degrees) and TXC by C1 (90 degrees), as in the shell RTL.
# The values are the common unmeasured-board budget documented in
# docs/rgmii-timing-budget-20261018.md, not a claim of PCB measurement or PHY
# maximum characterization.
#
# The native RX PLL derives C0/C1 from the actual recovered-clock port above;
# C1's generated BUFG clocks the HDIO DDR capture and GMII hand-off.  Keep
# RXC, both derived clocks, and the virtual data reference remain related.
# Both external input edges retain the same 3 ns-wide physical envelope
# -3.5..-0.5 ns before the same-named RXC edge.  The virtual reference follows
# TD's DDR Center Aligned input-delay template and preserves the 1 ns PHY
# setup/hold minima and assumed +/-0.5 ns PCB skew of the earlier budget.
# The
# remaining internal clock/CDC exceptions below do not replace this interface
# budget.
#
# TD 6.2 derives C1 at the following stable clock identity.  The historical
# ODDR-port errors were caused by source objects that did not exist after
# synthesis.  Use this actual C1 pin as both source and master for the
# generated clock on O_eth_txc, preserving clock-to-pad timing explicitly.
# Output max/min use +1.5/-1.5 ns from 1 ns PHY setup/hold minima and an
# assumed 0.5 ns clock/data PCB skew; the 150 ps TX setting is not credited.
create_generated_clock -name {ph1p_eth_txc} -source [get_pins {u_ph1p_eth_shell/u_eth_pll/ph1p_phy_pll_wrapper_65b93976a4bd_Inst/u_PH1P_PHY_PLL.clkc[1]}] -master_clock {u_ph1p_eth_shell/u_eth_pll/ph1p_phy_pll_wrapper_65b93976a4bd_Inst/u_PH1P_PHY_PLL.clkc[1]} -multiply_by 1 [get_ports {O_eth_txc}]
set_output_delay -clock [get_clocks {ph1p_eth_txc}] -max 1.500 [get_ports {O_eth_txd[*] O_eth_tx_ctl}]
set_output_delay -clock [get_clocks {ph1p_eth_txc}] -min -1.500 [get_ports {O_eth_txd[*] O_eth_tx_ctl}]
set_output_delay -clock [get_clocks {ph1p_eth_txc}] -clock_fall -max -add_delay 1.500 [get_ports {O_eth_txd[*] O_eth_tx_ctl}]
set_output_delay -clock [get_clocks {ph1p_eth_txc}] -clock_fall -min -1.500 [get_ports {O_eth_txd[*] O_eth_tx_ctl}] -add_delay

# No set_min_delay, set_false_path, or global clock cut is used for the RGMII
# ports.  The existing clock/CDC exceptions below remain scoped to their
# documented reset or asynchronous mailbox paths.
#---------------------------------------------------------------------------

#---------------------------------------------------------------------------
# Ethernet-shell ready flag used as an asynchronous reset (2026-10-11).
#
# ctl_ph1p_rgmii_shell outputs O_reset_ready from a two-stage synchronizer
# running on the TX 90-degree PLL clock; the top turns it into
# S_eth_app_reset_async_n, which is the asynchronous reset of the pixel-domain
# application reset synchronizer (top_ph1p_master.v:403-440).  The only paths
# between the Ethernet PLL clocks and HDMI_PIXEL_CLK are that reset
# distribution, so TD's recovery check on it (SWNS -1.661 ns, 3 endpoints) is
# not a real timing requirement.  Functional crossings stay on their own
# mailbox RAM and two-stage synchronizers inside ctl_lpr_packet_link.
#---------------------------------------------------------------------------
set_false_path \
    -from [get_clocks {u_ph1p_eth_shell/u_eth_pll/ph1p_phy_pll_wrapper_65b93976a4bd_Inst/u_PH1P_PHY_PLL.clkc[0] \
                       u_ph1p_eth_shell/u_eth_pll/ph1p_phy_pll_wrapper_65b93976a4bd_Inst/u_PH1P_PHY_PLL.clkc[1]}] \
    -to   [get_clocks {HDMI_PIXEL_CLK}]

# The TX reset released in the 0-degree Ethernet clock domain drives the
# asynchronous reset of the 90-degree TX reset synchronizer; this pair carries
# no data path at all, so its recovery check (gate SWNS -0.002 ns, post-route
# -0.890 ns on O_eth_txc_syn_2.rst) is not a real requirement.
set_false_path \
    -from [get_clocks {u_ph1p_eth_shell/u_eth_pll/ph1p_phy_pll_wrapper_65b93976a4bd_Inst/u_PH1P_PHY_PLL.clkc[0]}] \
    -to   [get_clocks {u_ph1p_eth_shell/u_eth_pll/ph1p_phy_pll_wrapper_65b93976a4bd_Inst/u_PH1P_PHY_PLL.clkc[1]}]

# S_eth_app_reset_async_n is built in the 50 MHz domain and drives the
# asynchronous reset of the Ethernet TX-domain structures (post-route recovery
# SWNS -1.040 ns / -0.154 ns on two endpoints).  That distribution is the only
# path between sys_clk_50m and the Ethernet PLL clocks, so the pair is cut
# rather than the check being left failing.
set_false_path -from [get_clocks {sys_clk_50m}] \
               -to   [get_clocks {u_ph1p_eth_shell/u_eth_pll/ph1p_phy_pll_wrapper_65b93976a4bd_Inst/u_PH1P_PHY_PLL.clkc[0] \
                                  u_ph1p_eth_shell/u_eth_pll/ph1p_phy_pll_wrapper_65b93976a4bd_Inst/u_PH1P_PHY_PLL.clkc[1]}]

# TD accepts the generated clock and both-edge delays.  Every RTL revision
# still requires a fresh routed STA with active TX and RX endpoint coverage.
