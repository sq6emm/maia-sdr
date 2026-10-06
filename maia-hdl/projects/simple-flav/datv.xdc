# DATV select (DAC GPIO bit 1, CPU clock) into the DAC clock: a quasi-static
# mode bit, synchronised by two flops in datv_split / datv_merge.
set_false_path -to [get_cells -hierarchical -filter {NAME =~ *datv_split_0*sel_m_reg}]
set_false_path -to [get_cells -hierarchical -filter {NAME =~ *datv_merge_0*sel_m_reg}]
# datv_merge's underflow counter, Gray-coded (DAC clock), into datv_tx's
# synchronizer (CPU clock): one bit changes at a time.
set_max_delay -datapath_only 5.0 \
    -from [get_cells -hierarchical -filter {NAME =~ *datv_merge_0*underflows_gray_reg*}] \
    -to [get_cells -hierarchical -filter {NAME =~ *datv_tx_0*und_cdc*stage0_reg*}]
