# Questa/ModelSim configuration. Run from a separate simulation output directory.
# Example: vsim -c -do "source {<Eyetrack>/two_board/sim/tb_alg_vehicle_services.do}"
onerror {quit -code 1}
onbreak {quit -code 1}
set source_root [file normalize [file join [file dirname [info script]] ..]]
set work_dir [file normalize [file join [pwd] work_tb_alg_vehicle_services]]
vlib $work_dir
set source_files [list \
    [file join $source_root {PH1P/rtl/alg/alg_vehicle_services.v}] \
    [file join $source_root {PH1P/rtl/alg/gimbal_ctrl.v}] \
    [file join $source_root {PH1P/rtl/alg/plate_metrics.v}] \
    [file join $source_root {PH1P/rtl/ctl/ctl_plate_card.v}] \
    [file join $source_root {PH1P/rtl/ctl/tf_card_ctrl.v}] \
    [file join $source_root {PH1P/rtl/ctl/tf_sector_buf.v}] \
    [file join $source_root {PH1P/rtl/ctl/tf_spi_master.v}] \
    [file join $source_root {PH1P/tb/tb_alg_vehicle_services.sv}] \
]
eval vlog -sv -work [list $work_dir] [list "+incdir+$source_root/EG4D/rtl/candidate14"] $source_files
vsim -onfinish stop -voptargs=+acc -lib $work_dir tb_alg_vehicle_services 
run -all
quit -code 0
