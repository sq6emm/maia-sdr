# t2router (datv bitstream): the async FIFO's Gray-code pointers and the
# overflow toggle cross between the Maia clock and the CPU clock; the
# clocks come from one MMCM input, so Vivado times these paths as related:
# bound them by a datapath delay instead (a synchronizer stage follows).
set_max_delay -datapath_only 8.0 \
    -from [get_cells -hierarchical -filter {NAME =~ *t2router_0*gry_reg*}] \
    -to [get_cells -hierarchical -filter {NAME =~ *t2router_0*_cdc/stage0_reg*}]
set_max_delay -datapath_only 8.0 \
    -from [get_cells -hierarchical -filter {NAME =~ *t2router_0*tog_eq_reg*}] \
    -to [get_cells -hierarchical -filter {NAME =~ *t2router_0*tog_s_reg*}]
