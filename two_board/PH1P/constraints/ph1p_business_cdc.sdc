# Generated literal commands; no Tcl loops/procs in read_sdc.
# Telemetry req/ack: only synchronizer first stages cross clock domains.
# The 32-bit cfg hold bus stays frozen through the returning ACK and the
# complete packet write; this is a bundled-data handshake, not per-bit 2FF.
# The common PHY/video reset asserts asynchronously in the configuration
# domain. The first release-synchronizer stage has constant D=1. Exempt its
# recovery check only; removal and stage 0 -> stage 1 remain normally timed.
# Use a literal read_sdc command so TD tracks the register through mapping.
set_false_path -setup -to [get_regs {S_telemetry_cfg_reset_sync[0]}]
set_max_delay -datapath_only -from [get_regs {u_telemetry_tx/S_request_toggle_app}] -to [get_regs {u_telemetry_tx/S_request_meta_cfg}] 42.000
set_max_delay -datapath_only -from [get_regs {u_telemetry_tx/S_ack_toggle_cfg}] -to [get_regs {u_telemetry_tx/S_ack_meta_app}] 13.333
set_max_delay -datapath_only -from [get_regs {u_telemetry_tx/S_cfg_hold*}] -to [get_regs {u_telemetry_tx/S_snapshot_ag* u_telemetry_tx/S_snapshot_ae*}] 13.333

# Generated literal commands; no Tcl loops/procs in read_sdc.
# Protocol proof: v2.5/docs/mailbox-timing-contract.md.
# Exact endpoint preflight: luna_*_exact_register_map.tsv.
# Stage2 and all ordinary local paths retain normal STA checks.
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_hold_24m[0]}] -to [get_regs {u_vehicle_controls/S_snapshot_pending_pix[0]}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_hold_24m[1]}] -to [get_regs {u_vehicle_controls/S_snapshot_pending_pix[1]}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_hold_24m[2]}] -to [get_regs {u_vehicle_controls/S_snapshot_pending_pix[2]}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_hold_24m[3]}] -to [get_regs {u_vehicle_controls/S_snapshot_pending_pix[3]}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_hold_24m[4]}] -to [get_regs {u_vehicle_controls/S_snapshot_pending_pix[4]}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_hold_24m[5]}] -to [get_regs {u_vehicle_controls/S_snapshot_pending_pix[5]}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_hold_24m[6]}] -to [get_regs {u_vehicle_controls/S_snapshot_pending_pix[6]}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_hold_24m[8]}] -to [get_regs {u_vehicle_controls/S_snapshot_pending_pix[8]}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_hold_24m[9]}] -to [get_regs {u_vehicle_controls/S_snapshot_pending_pix[9]}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_req_24m}] -to [get_regs {u_vehicle_controls/S_snapshot_req_pix_1}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_snapshot_ack_pix}] -to [get_regs {u_vehicle_controls/S_snapshot_ack_meta_24m}] 41.666
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_enter_toggle_24m}] -to [get_regs {u_vehicle_controls/S_enter_toggle_pix_1}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_exit_toggle_24m}] -to [get_regs {u_vehicle_controls/S_exit_toggle_pix_1}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_step_toggle_24m}] -to [get_regs {u_vehicle_controls/S_step_toggle_pix_1}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_clear_toggle_24m}] -to [get_regs {u_vehicle_controls/S_clear_toggle_pix_1}] 13.333
set_max_delay -datapath_only -from [get_regs {u_vehicle_controls/S_home_toggle_24m}] -to [get_regs {u_vehicle_controls/S_home_toggle_pix_1}] 13.333

# Display tuple mailbox ACK returns through S_ack_meta_24m -> S_ack_sync_24m.
# Source holds the tuple until synchronized ACK. Bound only the first-stage
# asynchronous ACK net to one 24m period; stage1 -> stage2 retains normal STA.
# Root cause evidence: tier1 first route, HDMI_PIXEL_CLK -> pll_clk_24m,
# u_display_controls/S_ack_pix -> S_ack_meta_24m, nominal edge budget 0.674 ns.
set_max_delay -datapath_only -from [get_regs {u_display_controls/S_ack_pix}] -to [get_regs {u_display_controls/S_ack_meta_24m}] 41.666

