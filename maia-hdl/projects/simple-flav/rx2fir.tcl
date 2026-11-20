# RX2 through the same two x8 decimation stages as RX1 (rxfir.tcl +
# rate64.tcl), trx bitstream: with the AD9361 in 2R2T both receivers reach
# the DMA at 0.384 MS/s, filtered alike, for trxd's noise canceller (RX1 the
# antenna, RX2 the noise reference). Before this RX2 went to the packer
# unfiltered on the decimated strobe (aliased).
#
# Lanes 2/3 (RX2 I/Q) take whatever drove the packer's inputs (the XO
# corrector on the LibreSDR, the ADC FIFO elsewhere) and the RX1 chain's
# input valid, so the two chains run in lockstep: identical FIRs on one
# clock and one strobe, the packer's write enable (RX1's second-stage valid)
# serves both.

proc rx2_net_of {pin} {
    set net [get_bd_nets -quiet -of_objects [get_bd_pins $pin]]
    if {$net eq ""} { error "rx2fir: $pin is not connected" }
    return $net
}

ad_add_decimation_filter "rx2_fir_decimator_pre" 8 2 1 {61.44} {61.44} \
                         "$ad_hdl_dir/library/util_fir_int/coefile_int.coe"
ad_add_decimation_filter "rx2_fir_decimator" 8 2 1 {61.44} {61.44} \
                         "$ad_hdl_dir/library/util_fir_int/coefile_int.coe"
foreach h {rx2_fir_decimator_pre rx2_fir_decimator} {
    ad_connect util_ad9361_divclk/clk_out $h/aclk
    ad_connect rx_decim_slice/Dout $h/active
}

set rx2_valid [rx2_net_of rx_fir_decimator_pre/valid_in_0]
foreach i {0 1} {
    set lane [expr {$i + 2}]
    # data: the packer's old source moves to the first stage
    rate64_move_sink util_ad9361_adc_pack/fifo_wr_data_$lane rx2_fir_decimator_pre/data_in_$i
    # enable: the packer's old source, or the ADC FIFO's
    if {[get_bd_nets -quiet -of_objects [get_bd_pins util_ad9361_adc_pack/enable_$lane]] ne ""} {
        rate64_move_sink util_ad9361_adc_pack/enable_$lane rx2_fir_decimator_pre/enable_in_$i
    } else {
        ad_connect util_ad9361_adc_fifo/dout_enable_$lane rx2_fir_decimator_pre/enable_in_$i
    }
    connect_bd_net -net $rx2_valid [get_bd_pins rx2_fir_decimator_pre/valid_in_$i]

    # first stage 18 bits, rounded to 16 for the second (as rate64.tcl)
    set f rx2_fir_decimator_pre/fir_decimation_$i
    ad_disconnect $f/m_axis_data_tdata rx2_fir_decimator_pre/out_mux_$i/data_in_1
    set_property CONFIG.Output_Width 18 [get_bd_cells $f]
    create_bd_cell -type module -reference rate64_rx_bits rx2_rate64_bits_$i
    ad_connect $f/m_axis_data_tdata rx2_rate64_bits_$i/y
    ad_connect rx2_rate64_bits_$i/q rx2_fir_decimator_pre/out_mux_$i/data_in_1

    ad_connect rx2_fir_decimator_pre/valid_out_$i  rx2_fir_decimator/valid_in_$i
    ad_connect rx2_fir_decimator_pre/enable_out_$i rx2_fir_decimator/enable_in_$i
    ad_connect rx2_fir_decimator_pre/data_out_$i   rx2_fir_decimator/data_in_$i
    # one sample in 8 clocks: multipliers shared, as RX1's second stage
    set_property CONFIG.Sample_Frequency 7.68 [get_bd_cells rx2_fir_decimator/fir_decimation_$i]

    ad_connect rx2_fir_decimator/data_out_$i   util_ad9361_adc_pack/fifo_wr_data_$lane
    ad_connect rx2_fir_decimator/enable_out_$i util_ad9361_adc_pack/enable_$lane
}
