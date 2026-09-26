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
    default {
        puts "CRITICAL WARNING: Project name '$project_name' not recognized."
        exit 1
    }
}


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

set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.TCL.PRE \
  [file normalize [file join [file dirname [info script]] hold_fix.tcl]] \
  [get_runs impl_1]
set_property is_enabled false [get_files  *system_sys_ps7_0.xdc]
adi_project_run $project_name
source $ad_hdl_dir/library/axi_ad9361/axi_ad9361_delay.tcl
