# DVB-S2/T2 LDPC decoder (normal frames, rates 1/2 and 3/4) for trxd's DATV
# receivers: maia_hdl/ldpc_dec4.py behind maia_hdl/ldpc_axi.py with the DDR
# engine ldpc_dma.py (generated ldpc_axi.v, --lanes 4 --dma, id "LDP5").
# CPU clock; 64 KiB window at 0x43C40000: posterior RAM, control/status/id
# and the DDR addresses at 0xFF00. Its AXI3 master (LLRs in, decisions out)
# on HP0 (the trx bitstream gives HP0 to the network engine; it has no
# decoder).
add_files -norecurse [file normalize ldpc_axi.v]
update_compile_order -fileset sources_1
create_bd_cell -type module -reference ldpc_axi ldpc_0
ad_connect sys_cpu_clk ldpc_0/clk
ad_connect sys_cpu_reset ldpc_0/rst
ad_cpu_interconnect 0x43C40000 ldpc_0
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP0 {1}
ad_connect sys_cpu_clk sys_ps7/S_AXI_HP0_ACLK
ad_connect ldpc_0/m_axi sys_ps7/S_AXI_HP0
create_bd_addr_seg -range 0x20000000 -offset 0x00000000 \
                    [get_bd_addr_spaces ldpc_0/m_axi] \
                    [get_bd_addr_segs sys_ps7/S_AXI_HP0/HP0_DDR_LOWOCM] \
                    SEG_sys_ps7_HP0_DDR_LOWOCM_ldpc
