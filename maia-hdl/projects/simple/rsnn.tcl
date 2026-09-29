# CW-RS keying detector's front end (trxd src/rsnn.rs Net::front_q):
# maia_hdl/rsnn_front.py (generated rsnn_front_axi.v). CPU clock; 64 KiB
# window at 0x43C50000: weights, biases, the feature row, the outputs and
# control/status/id at 0xFF00.
add_files -norecurse [file normalize rsnn_front_axi.v]
update_compile_order -fileset sources_1
create_bd_cell -type module -reference rsnn_front_axi rsnn_0
ad_connect sys_cpu_clk rsnn_0/clk
ad_connect sys_cpu_reset rsnn_0/rst
ad_cpu_interconnect 0x43C50000 rsnn_0
