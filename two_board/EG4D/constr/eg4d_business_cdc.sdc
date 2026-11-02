# EG4D business CDC constraints -- v2.6 plate-only build.
#
# The v2.5 ink/character-recognition chain (ctl_slave_ink_application and its
# mailbox/tile/snapshot/result CDC instances) was removed in v2.6, so every
# constraint that targeted those registers is gone with the hierarchy.  The
# remaining board-level asynchronous inputs are the four key-matrix column
# pins, which are pulled up and sampled by a debounced scanner in the 25 MHz
# application domain.  A key press is stable for milliseconds, so these paths
# are false paths; the scanner's own synchronizer/debounce handles them.
set_false_path -from [get_ports {IO_key_col[0]}]
set_false_path -from [get_ports {IO_key_col[1]}]
set_false_path -from [get_ports {IO_key_col[2]}]
set_false_path -from [get_ports {IO_key_col[3]}]
set_false_path -from [get_ports {IO_key_row[0]}]
set_false_path -from [get_ports {IO_key_row[1]}]
set_false_path -from [get_ports {IO_key_row[2]}]
set_false_path -from [get_ports {IO_key_row[3]}]

# The key-state packet TX/receive CDC lives inside ctl_lpr_packet_link, which
# carries its own internal two-flop synchronizers and is unchanged from v2.5.