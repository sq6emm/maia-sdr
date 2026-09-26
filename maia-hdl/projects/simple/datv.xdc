# DATV select (DAC GPIO bit 1, CPU clock) into the DAC clock: a quasi-static
# mode bit, synchronised by two flops in datv_split / datv_merge.
set_false_path -to [get_cells -hierarchical -filter {NAME =~ *datv_split_0*sel_m_reg}]
set_false_path -to [get_cells -hierarchical -filter {NAME =~ *datv_merge_0*sel_m_reg}]
