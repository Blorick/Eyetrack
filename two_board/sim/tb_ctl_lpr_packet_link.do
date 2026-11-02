# Questa/ModelSim configuration. Run from a separate simulation output directory.
# Example: vsim -c -do "source {<Eyetrack>/two_board/sim/tb_ctl_lpr_packet_link.do}"
onerror {quit -code 1}
onbreak {quit -code 1}
set source_root [file normalize [file join [file dirname [info script]] ..]]
set work_dir [file normalize [file join [pwd] work_tb_ctl_lpr_packet_link]]
vlib $work_dir
set source_files [list \
    [file join $source_root {common/ctl_lpr_packet_link.v}] \
    [file join $source_root {common/tb_ctl_lpr_packet_link.sv}] \
]
eval vlog -sv -work [list $work_dir] [list "+incdir+$source_root/EG4D/rtl/candidate14"] $source_files
file mkdir [file join [pwd] golden-vectors]
foreach fixture [glob [file join $source_root tests fixtures lpr golden-vectors *.hex]] {
    file copy -force $fixture [file join [pwd] golden-vectors [file tail $fixture]]
}
vsim -onfinish stop -voptargs=+acc -lib $work_dir tb_ctl_lpr_packet_link 
run -all
quit -code 0
