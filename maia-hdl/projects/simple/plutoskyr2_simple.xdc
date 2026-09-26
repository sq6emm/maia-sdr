# tezuka_fw_simple additions for PlutoSky R2.
#
# EXT_IO0 (JP5) carries a GPS 1PPS. It sits in bank 33, whose VCCO is 3.3 V
# (the ADF4001 lines and CLK_40M_FPGA share it), so LVCMOS33 - the board
# xdc's LVCMOS25 for this pin predates anything being connected to it.
set_property IOSTANDARD LVCMOS33 [get_ports ext_io0]
set_property PULLDOWN true [get_ports ext_io0]

# refmeter crosses clk_40m <-> the AXI clock only through toggle handshakes
# whose payload is stable for many cycles when sampled.
set_false_path -to [get_cells -hier -filter {NAME =~ *refmeter_0/inst/*_sync_reg[0]}]
set_false_path -from [get_cells -hier -filter {NAME =~ *refmeter_0/inst/pps_cnt_reg[*]}]    -to [get_cells -hier -filter {NAME =~ *refmeter_0/inst/pps_cnt_axi_reg[*]}]
set_false_path -from [get_cells -hier -filter {NAME =~ *refmeter_0/inst/pps_period_reg[*]}] -to [get_cells -hier -filter {NAME =~ *refmeter_0/inst/pps_period_axi_reg[*]}]
set_false_path -from [get_cells -hier -filter {NAME =~ *refmeter_0/inst/pps_seq_r_reg[*]}]  -to [get_cells -hier -filter {NAME =~ *refmeter_0/inst/pps_seq_axi_reg[*]}]
set_false_path -from [get_cells -hier -filter {NAME =~ *refmeter_0/inst/snap_cnt_reg[*]}]   -to [get_cells -hier -filter {NAME =~ *refmeter_0/inst/snap_axi_reg[*]}]
