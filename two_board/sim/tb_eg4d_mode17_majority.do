# Questa/ModelSim configuration. Run from a separate simulation output directory.
# Example: vsim -c -do "source {<Eyetrack>/two_board/sim/tb_eg4d_mode17_majority.do}"
onerror {quit -code 1}
onbreak {quit -code 1}
set source_root [file normalize [file join [file dirname [info script]] ..]]
set work_dir [file normalize [file join [pwd] work_tb_eg4d_mode17_majority]]
vlib $work_dir
set source_files [list \
    [file join $source_root {EG4D/rtl/dsp/eg4d_mode17_majority.v}] \
    [file join $source_root {EG4D/tb/tb_eg4d_mode17_majority.sv}] \
]
eval vlog -sv -work [list $work_dir] [list "+incdir+$source_root/EG4D/rtl/candidate14"] $source_files
vsim -onfinish stop -voptargs=+acc -lib $work_dir tb_eg4d_mode17_majority 
run -all
quit -code 0
