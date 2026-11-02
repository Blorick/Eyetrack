# Questa/ModelSim configuration. Run from a separate simulation output directory.
# Example: vsim -c -do "source {<Eyetrack>/two_board/sim/tb_dsp_status_overlay.do}"
onerror {quit -code 1}
onbreak {quit -code 1}
set source_root [file normalize [file join [file dirname [info script]] ..]]
set work_dir [file normalize [file join [pwd] work_tb_dsp_status_overlay]]
vlib $work_dir
set source_files [list \
    [file join $source_root {PH1P/rtl/dsp/dsp_status_overlay.v}] \
    [file join $source_root {PH1P/tb/tb_dsp_status_overlay.sv}] \
]
eval vlog -sv -work [list $work_dir] [list "+incdir+$source_root/EG4D/rtl/candidate14"] $source_files
vsim -onfinish stop -voptargs=+acc -lib $work_dir tb_dsp_status_overlay 
run -all
quit -code 0
