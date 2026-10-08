# The x8 images (no rate64.tcl: ADALM-Pluto): the one TX interpolation stage
# at unity gain, as rate64.tcl does for its two (2026-10-08). The ADI x8
# FIR's output is the top 16 bits of its sum, a gain of about 1/4: the IQ
# path (SSB, CW, TUNE) reached the DAC 12 dB below its full scale. 18 output
# bits, saturated to 16 (rate64_bits.v rate64_tx_bits), put that back.
add_files -norecurse [file normalize rate64_bits.v]
update_compile_order -fileset sources_1
foreach i {0 1} {
    set f tx_fir_interpolator/fir_interpolation_$i
    ad_disconnect $f/m_axis_data_tdata tx_fir_interpolator/out_mux_$i/data_in_1
    set_property CONFIG.Output_Width 18 [get_bd_cells $f]
    create_bd_cell -type module -reference rate64_tx_bits tx_x8_bits_$i
    ad_connect $f/m_axis_data_tdata tx_x8_bits_$i/y
    ad_connect tx_x8_bits_$i/q tx_fir_interpolator/out_mux_$i/data_in_1
}
