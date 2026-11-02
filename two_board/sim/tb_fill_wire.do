# Questa/ModelSim configuration. Run from a separate simulation output directory.
# Example: vsim -c -do "source {<Eyetrack>/two_board/sim/tb_fill_wire.do}"
onerror {quit -code 1}
onbreak {quit -code 1}
set source_root [file normalize [file join [file dirname [info script]] ..]]
set work_dir [file normalize [file join [pwd] work_tb_fill_wire]]
vlib $work_dir
set source_files [list \
    [file join $source_root {PH1P/rtl/dsp/dsp_fill_light.v}] \
    [file join $source_root {PH1P/tb/tb_fill_wire.sv}] \
]
eval vlog -sv -work [list $work_dir] [list "+incdir+$source_root/EG4D/rtl/candidate14"] $source_files
vsim -onfinish stop -voptargs=+acc -lib $work_dir tb_fill_wire 
run -all
quit -code 0
