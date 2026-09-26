# DVB-S2 LDPC decoder (normal frames, rates 1/2 and 3/4) for trxd's DATV
# receiver: maia_hdl/ldpc_dec.py behind maia_hdl/ldpc_axi.py (generated
# ldpc_axi.v). CPU clock; 64 KiB window at 0x43C40000: posterior RAM
# (LLRs in, decisions out) and control/status/id at 0xFF00.
add_files -norecurse [file normalize ldpc_axi.v]
update_compile_order -fileset sources_1
create_bd_cell -type module -reference ldpc_axi ldpc_0
ad_connect sys_cpu_clk ldpc_0/clk
ad_connect sys_cpu_reset ldpc_0/rst
ad_cpu_interconnect 0x43C40000 ldpc_0
