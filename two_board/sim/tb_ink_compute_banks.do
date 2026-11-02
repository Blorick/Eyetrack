# Questa/ModelSim configuration. Run from a separate simulation output directory.
# Example: vsim -c -do "source {<Eyetrack>/two_board/sim/tb_ink_compute_banks.do}"
onerror {quit -code 1}
onbreak {quit -code 1}
set source_root [file normalize [file join [file dirname [info script]] ..]]
set work_dir [file normalize [file join [pwd] work_tb_ink_compute_banks]]
vlib $work_dir
set source_files [list \
    [file join $source_root {EG4D/rtl/candidate13/char_shape_feature_engine.v}] \
    [file join $source_root {EG4D/rtl/candidate14/char_match.v}] \
    [file join $source_root {EG4D/rtl/ctl/ctl_slave_ink_compute.v}] \
    [file join $source_root {EG4D/tb/tb_ink_compute_banks.sv}] \
]
eval vlog -sv -work [list $work_dir] [list "+incdir+$source_root/EG4D/rtl/candidate14"] $source_files
vsim -onfinish stop -voptargs=+acc -lib $work_dir tb_ink_compute_banks 
run -all
quit -code 0
