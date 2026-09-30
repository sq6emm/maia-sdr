# DVB-T2 receive: the equalizer's cells straight to their FEC blocks in DDR
# (maia_hdl/t2router.py, generated t2router.v; trxd builds the destination
# table). datv bitstream only. CPU clock for its registers (0x43C60000, id
# "T2R1") and AXI3 master (HP3); the words come from the Maia core in its
# own clock (eq_clk) through the router's async FIFO.
add_files -norecurse [file normalize t2router.v]
update_compile_order -fileset sources_1
create_bd_cell -type module -reference t2router t2router_0
ad_connect sys_cpu_clk t2router_0/clk
ad_connect sys_cpu_reset t2router_0/rst
ad_connect maia_sdr_clk/clk_out1 t2router_0/eq_clk
ad_connect GND t2router_0/eq_rst
ad_connect maia_sdr/t2_eq_data t2router_0/eq_data
ad_connect maia_sdr/t2_eq_valid t2router_0/eq_valid
ad_cpu_interconnect 0x43C60000 t2router_0
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP3 {1}
ad_connect sys_cpu_clk sys_ps7/S_AXI_HP3_ACLK
ad_connect t2router_0/m_axi sys_ps7/S_AXI_HP3
create_bd_addr_seg -range 0x20000000 -offset 0x00000000 \
                    [get_bd_addr_spaces t2router_0/m_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_HP3/HP3_DDR_LOWOCM] \
                    SEG_sys_ps7_HP3_DDR_LOWOCM_t2router
