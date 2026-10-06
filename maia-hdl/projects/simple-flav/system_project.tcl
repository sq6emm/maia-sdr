set IGNORE_VERSION_CHECK 1
source ../../adi-hdl/scripts/adi_env.tcl
source $ad_hdl_dir/projects/scripts/adi_project_xilinx.tcl
source $ad_hdl_dir/projects/scripts/adi_board.tcl
source ../tezuka_env.tcl

if { [info exists ::env(PROJECT_NAME)] } {
  set project_name $::env(PROJECT_NAME)
} else {
  set project_name "unknown"
}

switch -glob -- $project_name {
    "plutoskyr2" {
        set p_device "xc7z020clg484-2"
    }
    "libre" {
        set p_device "xc7z020clg400-2"
    }
    "pluto" {
        # ADALM-Pluto: the transceiver only (trx mode without the CW-RS
        # network and the refmeter: tezuka_fw_simple docs/FLAVOURS.md).
        set p_device "xc7z010clg225-1"
    }
    default {
        puts "CRITICAL WARNING: Project name '$project_name' not recognized."
        exit 1
    }
}


# FPGA_MODE (tezuka_fw_simple docs/FPGA-MODES.md): which parts go in.
#   all   everything (the boot default)
#   trx   radio + wide scope + CW-RS network front end (no DATV)
#   datv  radio + wide scope + DVB-S2/T2 receive and transmit + LDPC
if { [info exists ::env(FPGA_MODE)] } {
  set fpga_mode $::env(FPGA_MODE)
} else {
  set fpga_mode "all"
}
if {[lsearch -exact {all trx datv} $fpga_mode] < 0} {
  puts "CRITICAL WARNING: FPGA_MODE '$fpga_mode' not recognized."
  exit 1
}
set ::fpga_mode $fpga_mode
adi_project $project_name

if {$project_name eq "plutoskyr2"} {
  # Local top: adds the EXT_IO0 1PPS input and the refmeter ports.
  adi_project_files $project_name [list \
    "plutoskyr2_system_top.v" \
    "$::tezuka_hdl_dir/boards/$project_name/system_constr.xdc" \
    "plutoskyr2_simple.xdc" \
    "bitstream.xdc" \
    "$ad_hdl_dir/library/common/ad_iobuf.v"]
} else {
  adi_project_files $project_name [list \
    "$::tezuka_hdl_dir/boards/$project_name/system_top.v" \
    "$::tezuka_hdl_dir/boards/$project_name/system_constr.xdc" \
    "bitstream.xdc" \
    "$ad_hdl_dir/library/common/ad_iobuf.v"]
}
if {$fpga_mode ne "trx"} {
  adi_project_files $project_name [list "datv.xdc"]
}
if {$fpga_mode eq "datv"} {
  adi_project_files $project_name [list "t2router.xdc"]
}

set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.TCL.PRE \
  [file normalize [file join [file dirname [info script]] hold_fix.tcl]] \
  [get_runs impl_1]
set_property is_enabled false [get_files  *system_sys_ps7_0.xdc]
adi_project_run $project_name
source $ad_hdl_dir/library/axi_ad9361/axi_ad9361_delay.tcl
