# Questa/ModelSim configuration. Run from a separate simulation output directory.
# Example: vsim -c -do "source {<Eyetrack>/two_board/sim/tb_ctl_display_control_snapshot.do}"
onerror {quit -code 1}
onbreak {quit -code 1}
set source_root [file normalize [file join [file dirname [info script]] ..]]
set work_dir [file normalize [file join [pwd] work_tb_ctl_display_control_snapshot]]
vlib $work_dir
set source_files [list \
    [file join $source_root {PH1P/rtl/ctl/ctl_display_control_snapshot.v}] \
    [file join $source_root {PH1P/tb/tb_ctl_display_control_snapshot.sv}] \
]
eval vlog -sv -work [list $work_dir] [list "+incdir+$source_root/EG4D/rtl/candidate14"] $source_files
vsim -onfinish stop -voptargs=+acc -lib $work_dir tb_ctl_display_control_snapshot 
run -all
quit -code 0
