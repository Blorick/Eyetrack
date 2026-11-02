# Questa/ModelSim configuration. Run from a separate simulation output directory.
# Example: vsim -c -do "source {<Eyetrack>/two_board/sim/tb_buf_display_latency_probe.do}"
onerror {quit -code 1}
onbreak {quit -code 1}
set source_root [file normalize [file join [file dirname [info script]] ..]]
set work_dir [file normalize [file join [pwd] work_tb_buf_display_latency_probe]]
vlib $work_dir
set source_files [list \
    [file join $source_root {PH1P/rtl/buf/buf_display_latency_probe.v}] \
    [file join $source_root {PH1P/tb/tb_buf_display_latency_probe.sv}] \
]
eval vlog -sv -work [list $work_dir] [list "+incdir+$source_root/EG4D/rtl/candidate14"] $source_files
vsim -onfinish stop -voptargs=+acc -lib $work_dir tb_buf_display_latency_probe 
run -all
quit -code 0
