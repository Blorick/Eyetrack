# Validation: compilation/elaboration and 2 ms startup run only; full run-all completion was not verified.
# Questa/ModelSim configuration. Run from a separate simulation output directory.
# Example: vsim -c -do "source {<Eyetrack>/two_board/sim/tb_ink_integrated_recovery.do}"
onerror {quit -code 1}
onbreak {quit -code 1}
set source_root [file normalize [file join [file dirname [info script]] ..]]
set work_dir [file normalize [file join [pwd] work_tb_ink_integrated_recovery]]
vlib $work_dir
set source_files [list \
    [file join $source_root {common/ctl_lpr_packet_link.v}] \
    [file join $source_root {common/ctl_stream_link.v}] \
    [file join $source_root {common/rtl/ctl/ctl_gmii_tx_scheduler3.v}] \
    [file join $source_root {common/rtl/ctl/ctl_ink_packet_rx.v}] \
    [file join $source_root {common/rtl/ctl/ctl_mailbox_cdc.v}] \
    [file join $source_root {EG4D/rtl/candidate13/char_shape_feature_engine.v}] \
    [file join $source_root {EG4D/rtl/candidate14/char_match.v}] \
    [file join $source_root {EG4D/rtl/ctl/ctl_slave_ink_compute.v}] \
    [file join $source_root {EG4D/rtl/ctl/ctl_slave_ink_decoder.v}] \
    [file join $source_root {EG4D/rtl/ctl/ctl_slave_ink_reply.v}] \
    [file join $source_root {EG4D/rtl/ctl/ctl_slave_ink_session.v}] \
    [file join $source_root {EG4D/rtl/ctl/eg4d_line_adapter.v}] \
    [file join $source_root {EG4D/rtl/dsp/eg4d_mode17_majority.v}] \
    [file join $source_root {EG4D/rtl/dsp/eg4d_mode_filter.v}] \
    [file join $source_root {EG4D/rtl/dsp/eg4d_mode_pipeline.v}] \
    [file join $source_root {PH1P/rtl/ctl/ctl_master_ink_application.v}] \
    [file join $source_root {PH1P/rtl/ctl/ctl_master_ink_capture.v}] \
    [file join $source_root {PH1P/rtl/ctl/ctl_master_ink_response.v}] \
    [file join $source_root {PH1P/rtl/ctl/ctl_master_ink_session.v}] \
    [file join $source_root {PH1P/rtl/ctl/ph1p_line_bridge.v}] \
    [file join $source_root {PH1P/rtl/dsp/alg_contrast_stretch.v}] \
    [file join $source_root {PH1P/rtl/dsp/dsp_camera_modes.v}] \
    [file join $source_root {PH1P/rtl/dsp/dsp_camera_neighbourhood.v}] \
    [file join $source_root {PH1P/rtl/dsp/dsp_mode_filter.v}] \
    [file join $source_root {tests/ink_recovery/rtl/ctl_ink_packet_tx_metadata_variant.v}] \
    [file join $source_root {tests/ink_recovery/rtl/ctl_master_ink_watchdog_variant.v}] \
    [file join $source_root {tests/ink_recovery/rtl/ctl_slave_ink_application_variant.v}] \
    [file join $source_root {tests/ink_recovery/rtl/eg4d_cap_master_tile_capture_variant.v}] \
    [file join $source_root {tests/ink_recovery/tb_ink_integrated_recovery.sv}] \
]
eval vlog -sv -work [list $work_dir] [list "+incdir+$source_root/EG4D/rtl/candidate14"] $source_files
vsim -onfinish stop -lib $work_dir tb_ink_integrated_recovery 
run -all
quit -code 0

