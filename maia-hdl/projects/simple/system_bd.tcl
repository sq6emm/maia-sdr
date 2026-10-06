# tezuka_fw_simple block design: the bare AD9361 transceiver path.
#
#   RX: axi_ad9361 -> adc_fifo -> [libre: XO NCO] -> x8 FIR decimator -> cpack -> ADC DMA
#   TX: DAC DMA -> upack -> x8 FIR interpolator -> [libre: XO NCO] -> dac_fifo -> axi_ad9361
#   (LibreSDR trx: x64 both ways, the AD9361 at 24.576 MS/s; rate64.tcl)
#
# Both filters are bypassed after reset and switched in by bit 0 of the
# axi_ad9361 ADC / DAC GPIO-out registers (0x790200BC / 0x790240BC), the same
# control ADI's own Pluto design uses. No Maia-SDR, no DVB-S2, no sweeper, no
# CS8/CS12 packing.
#
# Address map
#   0x79020000 axi_ad9361
#   0x7C400000 ADC DMA
#   0x7C420000 DAC DMA
#   0x43C00000 vctcxo_lock (libre only)
#   0x7C460000 maia_sdr (spectrometer only: web UI wide scope)
#   0x43C10000 refmeter: reference-oscillator counter vs GPS 1PPS / software
#   0x43C20000 datv_tx (DATV pulse shaping), 0x43C30000 DVB-S2 encoder
#   0x43C40000 DVB-S2 LDPC decoder (normal frames)
#   0x43C60000 DVB-T2 cell router (datv bitstream)

switch -glob -- $project_name {
    "plutoskyr2" {
        set lvds "lvds"
    }
    "libre" {
        set lvds "lvds"
        set vctcxo "vctcxo"
        set xo_corrector "xo_corrector"
    }
    "pluto" {
        # CMOS interface to the AD9363; xc7z010: no CW-RS network front end,
        # no refmeter (no reference clock into the fabric); the wide scope
        # unless SIMPLE_NO_SCOPE=1.
        set pluto_slim 1
    }
    default {
        puts "CRITICAL WARNING: Project name '$project_name' not recognized."
        exit 1
    }
}

# Detach only the given sink pin from its net. ADI's ad_disconnect can delete
# the whole net (and every other load on it) for some pin kinds.
proc simple_disconnect_sink {pin_name} {
    set pin [get_bd_pins -quiet $pin_name]
    if {$pin eq ""} {
        error "simple: block-design sink pin '$pin_name' not found"
    }
    set net [get_bd_nets -quiet -of_objects $pin]
    if {$net ne ""} {
        disconnect_bd_net $net $pin
    }
}

source $::tezuka_hdl_dir/common/xilinx_init.tcl
source $::tezuka_hdl_dir/boards/$project_name/ps7.tcl
source $::tezuka_hdl_dir/common/xilinx_ad9361.tcl
source $::tezuka_hdl_dir/boards/$project_name/ports.tcl

# DMA masters to DDR through HP2 (the ADC and DAC DMAs share it).
ad_mem_hp2_interconnect sys_cpu_clk sys_ps7/S_AXI_HP2
ad_mem_hp2_interconnect sys_cpu_clk axi_ad9361_adc_dma/m_dest_axi
ad_mem_hp2_interconnect sys_cpu_clk axi_ad9361_dac_dma/m_src_axi

if {[info exists vctcxo]} { source $::tezuka_hdl_dir/boards/$project_name/vcxo_ctrl.tcl }
source rxfir.tcl
if {!([info exists pluto_slim] && [info exists ::env(SIMPLE_NO_SCOPE)])} { source maia_scope.tcl }
source $::tezuka_hdl_dir/common/txfir.tcl
if {[info exists xo_corrector]} { source xo_corrector.tcl }
# LibreSDR trx: the AD9361 at 24.576 MS/s, x64 to and from the DMAs, so
# Maia's spectrometer sees ~24 MHz (rate64.tcl)
if {$::fpga_mode eq "trx" && $project_name eq "libre"} { source rate64.tcl }
# FPGA_MODE (system_project.tcl): the DATV parts
if {$::fpga_mode ne "trx"} {
    source datv_tx.tcl
    source ldpc.tcl
}
if {$::fpga_mode eq "datv" || $::fpga_mode eq "t2"} {
    source t2router.tcl
}

# Reference oscillator meter (see refmeter.v). PlutoSky R2 brings the 40 MHz
# VCTCXO and the EXT_IO0 1PPS in through new top-level ports; Libre taps the
# ports its vctcxo_lock already has.
if {![info exists pluto_slim]} {
add_files -norecurse [file normalize refmeter.v]
update_compile_order -fileset sources_1
create_bd_cell -type module -reference refmeter refmeter_0
if {$project_name eq "plutoskyr2"} {
    create_bd_port -dir I -type clk -freq_hz 40000000 ref_clk
    create_bd_port -dir I pps
    ad_connect ref_clk refmeter_0/ref_clk
    ad_connect pps refmeter_0/pps
} else {
    ad_ip_instance util_vector_logic pps_or [list C_OPERATION {or} C_SIZE 1]
    ad_connect PPS_IN pps_or/Op1
    ad_connect PPS_GPS pps_or/Op2
    ad_connect pps_or/Res refmeter_0/pps
    ad_connect CLK_40MHz_FPGA refmeter_0/ref_clk
}
ad_cpu_interconnect 0x43C10000 refmeter_0
}
