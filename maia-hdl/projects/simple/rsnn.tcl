# CW-RS keying detector in the FPGA (trxd src/rsnn.rs).
#   trx mode: front end and temporal layers, maia_hdl/rsnn_axi.py (generated
#     rsnn_axi.v): the temporal weights come from DDR through HP0 (its own
#     port: 200 MB/s while a frame runs).
#   all mode: the front end only (rsnn_front.py, rsnn_front_axi.v): no room.
# CPU clock; 64 KiB window at 0x43C50000 (control/status/id at 0xFF00).
if {$::fpga_mode eq "trx"} {
    add_files -norecurse [file normalize rsnn_axi.v]
    update_compile_order -fileset sources_1
    create_bd_cell -type module -reference rsnn_axi rsnn_0
    ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP0 {1}
    ad_connect sys_cpu_clk sys_ps7/S_AXI_HP0_ACLK
    ad_connect rsnn_0/m_axi sys_ps7/S_AXI_HP0
    create_bd_addr_seg -range 0x20000000 -offset 0x00000000 \
                        [get_bd_addr_spaces rsnn_0/m_axi] \
                        [get_bd_addr_segs sys_ps7/S_AXI_HP0/HP0_DDR_LOWOCM] \
                        SEG_sys_ps7_HP0_DDR_LOWOCM
} else {
    add_files -norecurse [file normalize rsnn_front_axi.v]
    update_compile_order -fileset sources_1
    create_bd_cell -type module -reference rsnn_front_axi rsnn_0
}
ad_connect sys_cpu_clk rsnn_0/clk
ad_connect sys_cpu_reset rsnn_0/rst
ad_cpu_interconnect 0x43C50000 rsnn_0
