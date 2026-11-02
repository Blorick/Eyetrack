# EG4D C13/C14 compute-side shell timing.  The transport is raw GMII with
# independent TX and recovered RX clocks; the shared client owns CDC/FIFO.
create_clock -name {eg4d_clk_25m} -period 40.000 -waveform {0.000 20.000} [get_ports {I_clk_25m}]
create_clock -name {eg4d_eth_rxc} -period 8.000 -waveform {0.000 4.000} [get_ports {I_eth_rxc}]
# Local mode/raster PLL output.  Keep the 75 MHz application clock visible to
# TD so the line adapter and mode17/mode18 pipeline are timed in their own
# domain rather than silently inheriting the 25 MHz reference period.
create_generated_clock -name {eg4d_mode_75m} -source [get_ports {I_clk_25m}] -multiply_by 3 [get_pins {u_mode_pll/pll_inst.clkc[0]}]
create_generated_clock -name {eg4d_mode_75m_90} -source [get_ports {I_clk_25m}] -multiply_by 3 -phase 90 [get_pins {u_mode_pll/pll_inst.clkc[1]}]
derive_clocks

#---------------------------------------------------------------------------
# RGMII external timing contract (2026-10-18).
#
# The EG4 shell launches TXD/TX_CTL with the Ethernet PLL C0 (0 degrees) and
# launches TXC with C1 (90 degrees, nominally 2 ns at 125 MHz).  The RX clock
# is the independent PHY clock at I_eth_rxc.  MLK-S200 ETHA is YT8531DH-CA;
# its A001[8] contract is RXC internal delay enabled and the MDIO repair sets
# the A003 delay-select fields to the agreed software values.  The constraint
# values below use an explicit unmeasured-board budget documented in
# docs/rgmii-timing-budget-20261018.md, not a claim of PCB measurement or PHY
# maximum characterization.
#
# The input and output ports are DDR.  Keep the rising-edge delay and add a
# separate falling-edge delay; omitting -add_delay would replace the first
# edge constraint in TD.
#---------------------------------------------------------------------------
set_input_delay -clock [get_clocks {eg4d_eth_rxc}] -max 3.500 [get_ports {I_eth_rxd[*] I_eth_rx_ctl}]
set_input_delay -clock [get_clocks {eg4d_eth_rxc}] -min 0.500 [get_ports {I_eth_rxd[*] I_eth_rx_ctl}]
set_input_delay -clock [get_clocks {eg4d_eth_rxc}] -max -clock_fall -add_delay 3.500 [get_ports {I_eth_rxd[*] I_eth_rx_ctl}]
set_input_delay -clock [get_clocks {eg4d_eth_rxc}] -min -clock_fall -add_delay 0.500 [get_ports {I_eth_rxd[*] I_eth_rx_ctl}]

# TD 6.2 derives the two Ethernet PLL clocks as
# u_pll/pll_inst.clkc[0] and [1].  The historical ODDR-port errors were caused
# by source objects that did not exist after synthesis.  Use the actual C1
# pin as both source and master for the generated clock on the TXC port; this
# keeps clock-to-pad timing explicit instead of hiding it behind an internal
# reference clock.
create_generated_clock -name {eg4d_eth_txc} -source [get_pins {u_pll/pll_inst.clkc[1]}] -master_clock {u_pll/pll_inst.clkc[1]} -multiply_by 1 [get_ports {O_eth_txc}]
# For output delay, max is PHY setup plus the assumed 0.5 ns clock/data PCB
# skew; min is -PHY hold plus the worst-direction skew.  With 1 ns setup/hold minima this
# is +1.5/-1.5 ns.  The PHY GE TX delay is disabled; FPGA C1 supplies the skew.
set_output_delay -clock [get_clocks {eg4d_eth_txc}] -max 1.500 [get_ports {O_eth_txd[*] O_eth_tx_ctl}]
set_output_delay -clock [get_clocks {eg4d_eth_txc}] -min -1.500 [get_ports {O_eth_txd[*] O_eth_tx_ctl}]
set_output_delay -clock [get_clocks {eg4d_eth_txc}] -clock_fall -max -add_delay 1.500 [get_ports {O_eth_txd[*] O_eth_tx_ctl}]
set_output_delay -clock [get_clocks {eg4d_eth_txc}] -clock_fall -min -1.500 [get_ports {O_eth_txd[*] O_eth_tx_ctl}] -add_delay

set_clock_groups -asynchronous \
    -group [get_clocks {eg4d_clk_25m}] \
    -group [get_clocks {eg4d_eth_rxc u_rx_pll/pll_inst.clkc[0] u_rx_pll/pll_inst.clkc[1]}] \
    -group [get_clocks {u_pll/pll_inst.clkc[0] u_pll/pll_inst.clkc[1]}]

# Recovered Ethernet RX is unrelated to the local mode PLL.  The mode/GMII
# mailbox and line adapter use explicit CDC handshakes; bound those paths by
# a destination-cycle datapath budget while leaving the 25->75 PLL relation
# available to STA.
set_clock_groups -asynchronous \
    -group [get_clocks {eg4d_eth_rxc u_rx_pll/pll_inst.clkc[0] u_rx_pll/pll_inst.clkc[1]}] \
    -group [get_clocks {eg4d_mode_75m eg4d_mode_75m_90}]
set_max_delay -datapath_only -from [get_clocks {eg4d_mode_75m}] \
    -to [get_clocks {u_pll/pll_inst.clkc[0]}] 8.000
set_max_delay -datapath_only -from [get_clocks {u_pll/pll_inst.clkc[0]}] \
    -to [get_clocks {eg4d_mode_75m}] 13.333
set_max_delay -datapath_only -from [get_clocks {eg4d_eth_rxc u_rx_pll/pll_inst.clkc[0] u_rx_pll/pll_inst.clkc[1]}] \
    -to [get_clocks {eg4d_mode_75m}] 13.333

# The packet link crosses between the 25 MHz application domain and the
# 125 MHz GMII TX domain only through its mailbox RAM and two-stage
# synchronizers, so the two clock groups above are asynchronous.  Without this
# grouping TD timed the TX-to-application synchronizer as a same-edge path and
# reported a hold violation on it (post-route HWNS -1.330 ns).

#---------------------------------------------------------------------------
# Asynchronous reset distribution (2026-10-11).
#
# The 25 MHz POR (S_por_25_reg) and the TX reset (S_tx_rst_reg) drive the
# asynchronous reset pins of the TX/TX90 reset synchronizers.  TD's
# recovery/removal checks on an asynchronous reset release are not a real
# timing requirement, and these pairs carry nothing else: the
# eg4d_clk_25m -> clkc[1] group has 3 endpoints and the clkc[0] -> clkc[1]
# group has 1, all of them reset pins (gate stage SWNS -2.992 ns / -0.593 ns).
#---------------------------------------------------------------------------
set_false_path -from [get_clocks {eg4d_clk_25m}] \
               -to   [get_clocks {u_pll/pll_inst.clkc[1]}]
set_false_path -from [get_clocks {u_pll/pll_inst.clkc[0]}] \
               -to   [get_clocks {u_pll/pll_inst.clkc[1]}]
