# TD 6.2.168116: td_commands_prompt.exe build.tcl
# Tcl callers may set argv to {output_directory syn|full} before sourcing.
# With no arguments, rebuild through bitstream generation in ./build.
if {[catch {
# TD sources this file without defining Tcl argv on a direct CLI invocation.
if {![info exists argv]} {set argv {}}
set project_dir [file normalize [file dirname [info script]]]
set source_root [file dirname $project_dir]
set output_dir [file join $project_dir build]
if {[llength $argv] > 0} {set output_dir [file normalize [lindex $argv 0]]}
set build_stage full
if {[llength $argv] > 1} {set build_stage [lindex $argv 1]}
if {$build_stage ni {syn full}} {error "Build stage must be syn or full"}
set prj_name ph1p_master_core_business
file mkdir $output_dir
set channel [open [file join $project_dir ${prj_name}.al] r]
fconfigure $channel -encoding utf-8
set project_text [read $channel]
close $channel
regsub {(<Project[^>]* Path=")[^"]*"} $project_text [format {\1%s"} $project_dir] project_text
set staged_project [file join $output_dir ${prj_name}.al]
set channel [open $staged_project w]
fconfigure $channel -encoding utf-8 -translation lf
puts $channel $project_text
close $channel
cd $output_dir
set alRun phy_1
if {$build_stage eq "syn"} {set alRun syn_1}
set device_name ph1_35p.db
set package_name PH1P35MDG324
set top_model_name top_ph1p_master
set speed 3
set run_type phy
set start_step read_design
set end_step bitgen
if {$build_stage eq "syn"} {set end_step opt_gate}
set ADCList [list [file join $source_root PH1P/constraints/ph1p_master.adc]]
set SDCList [list [file join $source_root PH1P/constraints/ph1p_master.sdc] [file join $source_root PH1P/constraints/ph1p_business_cdc.sdc]]
set IpADCList [list ddr2 [file join $source_root PH1P/vendor/legacy_user_sources/hdl_sources/ph1p35_ddr/ddr2/src/adc/ddr2.adc]]
set IpSDCList [list ddr2 [file join $source_root PH1P/vendor/legacy_user_sources/hdl_sources/ph1p35_ddr/ddr2/src/sdc/ddr2.sdc] w128_d512_fifo [file join $source_root PH1P/vendor/legacy_user_sources/ip_sources/w128_d512_fifo/w128_d512_fifo.tcl]]
set area_option "-packarea"
set arr_filter false
set drHoldFix off
source [file join $source_root PH1P/flow/ph1p_native_core_business_flow.tcl]
} build_error]} {
    puts stderr "BUILD_ERROR: $build_error"
    exit 1
}
exit 0
