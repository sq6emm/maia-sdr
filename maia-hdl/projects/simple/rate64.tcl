# x64 instead of x8 between the AD9361 and the DMAs (tezuka_fw_simple
# "wide trx", 2026-10-06): the AD9361 runs at 24.576 MS/s, so Maia's
# spectrometer (fed from the ADC FIFO, before any decimation) sees ~24 MHz in
# one look, while the DMAs keep the 384 kS/s stream trxd has always had.
#
# RX: a second ADI x8 decimation cell in front of rx_fir_decimator (after
#     the XO corrector where there is one): 24.576 -> 3.072 -> 0.384 MS/s.
#     The decimation cells pass the FIR's own output valid, so they chain.
# TX: the existing tx_fir_interpolator (txfir.tcl) stays at the DAC and
#     interpolates x8 from 3.072 MS/s; a new x8 stage (tx_fir_interp_pre)
#     in front of it reads the unpacker every 64 clocks and fills a small
#     AXI-stream FIFO per channel, which the DAC-side stage pulls every 8
#     clocks (its FIR wants one output per clock, so it cannot take the
#     pre-stage's 8-sample bursts directly). Both are clocked by the sample
#     clock; the rates match exactly, so the FIFOs neither fill nor empty.
#     The unpacker's read latency fix (delay_tvalid, txfir.tcl) moves to the
#     pre-stage, the stage that reads it; the FIFO presents data with its
#     valid, so the DAC-side stage takes it undelayed.
#
# Both new stages follow the same enable bits as the old ones (bit 0 of
# 0x790200BC / 0x790240BC): on, the path is x64; trxd always turns them on.

# Move the sink `pin` of whatever drives it onto `new_sink` instead.
proc rate64_move_sink {pin new_sink} {
    set p [get_bd_pins $pin]
    set net [get_bd_nets -of_objects $p]
    if {$net eq ""} { error "rate64: $pin is not driven" }
    disconnect_bd_net $net $p
    connect_bd_net -net $net [get_bd_pins $new_sink]
}

# ---- RX ---------------------------------------------------------------------
ad_add_decimation_filter "rx_fir_decimator_pre" 8 2 1 {61.44} {61.44} \
                         "$ad_hdl_dir/library/util_fir_int/coefile_int.coe"
ad_connect util_ad9361_divclk/clk_out rx_fir_decimator_pre/aclk
ad_connect rx_decim_slice/Dout rx_fir_decimator_pre/active
foreach i {0 1} {
    foreach s {valid_in enable_in data_in} {
        rate64_move_sink rx_fir_decimator/${s}_$i rx_fir_decimator_pre/${s}_$i
    }
    ad_connect rx_fir_decimator_pre/valid_out_$i  rx_fir_decimator/valid_in_$i
    ad_connect rx_fir_decimator_pre/enable_out_$i rx_fir_decimator/enable_in_$i
    ad_connect rx_fir_decimator_pre/data_out_$i   rx_fir_decimator/data_in_$i
}

# ---- TX ---------------------------------------------------------------------
add_files -norecurse [file normalize sat_shl2.v]
update_compile_order -fileset sources_1
ad_add_interpolation_filter "tx_fir_interp_pre" 8 2 1 {61.44} {7.68} \
                    "$::tezuka_hdl_dir/common/interpolator_x8.coe"
ad_connect util_ad9361_divclk/clk_out tx_fir_interp_pre/aclk
ad_connect interp8_slice/Dout tx_fir_interp_pre/active
# one input every 64 clocks (0.384 MS/s at 24.576 MHz)
set_property CONFIG.PULSE_PERIOD 63 [get_bd_cells tx_fir_interp_pre/rate_gen]

# the unpacker now feeds the pre-stage
foreach i {0 1} {
    rate64_move_sink tx_fir_interpolator/data_in_$i tx_fir_interp_pre/data_in_$i
    ad_connect tx_fir_interp_pre/dac_valid_$i VCC
}
simple_disconnect_sink util_ad9361_dac_upack/fifo_rd_en
ad_connect tx_fir_interp_pre/valid_out_0 util_ad9361_dac_upack/fifo_rd_en

foreach i {0 1} {
    # the read latency fix on the pre-stage (as txfir.tcl does it) ...
    ad_ip_instance c_shift_ram tx_fir_interp_pre/delay_tvalid_$i
    set_property -dict [list CONFIG.Width {1} CONFIG.Depth {2}] [get_bd_cells tx_fir_interp_pre/delay_tvalid_$i]
    ad_connect util_ad9361_divclk/clk_out tx_fir_interp_pre/delay_tvalid_$i/CLK
    ad_disconnect tx_fir_interp_pre/logic_and_$i/Res tx_fir_interp_pre/fir_interpolation_$i/s_axis_data_tvalid
    ad_connect tx_fir_interp_pre/logic_and_$i/Res tx_fir_interp_pre/delay_tvalid_$i/D
    ad_connect tx_fir_interp_pre/delay_tvalid_$i/Q tx_fir_interp_pre/fir_interpolation_$i/s_axis_data_tvalid
    # ... and off the DAC-side stage, which now reads a FIFO
    ad_disconnect tx_fir_interpolator/delay_tvalid_$i/Q tx_fir_interpolator/fir_interpolation_$i/s_axis_data_tvalid
    ad_disconnect tx_fir_interpolator/logic_and_$i/Res tx_fir_interpolator/delay_tvalid_$i/D
    delete_bd_objs [get_bd_cells tx_fir_interpolator/delay_tvalid_$i]
    ad_connect tx_fir_interpolator/logic_and_$i/Res tx_fir_interpolator/fir_interpolation_$i/s_axis_data_tvalid

    # pre-stage FIR output -> FIFO -> DAC-side stage, pulled by its request
    ad_ip_instance axis_data_fifo tx_rate64_fifo_$i [list \
        TDATA_NUM_BYTES 2 \
        FIFO_DEPTH 16 \
        HAS_TREADY 1]
    ad_connect util_ad9361_divclk/clk_out tx_rate64_fifo_$i/s_axis_aclk
    ad_connect util_ad9361_divclk_reset/peripheral_aresetn tx_rate64_fifo_$i/s_axis_aresetn
    ad_connect tx_fir_interp_pre/fir_interpolation_$i/m_axis_data_tvalid tx_rate64_fifo_$i/s_axis_tvalid
    ad_connect tx_fir_interp_pre/fir_interpolation_$i/m_axis_data_tdata  tx_rate64_fifo_$i/s_axis_tdata
    # x4 (saturating): the pre-stage's 1/4 gain back (sat_shl2.v)
    create_bd_cell -type module -reference sat_shl2 tx_rate64_gain_$i
    ad_connect tx_rate64_fifo_$i/m_axis_tdata tx_rate64_gain_$i/d
    ad_connect tx_rate64_gain_$i/q tx_fir_interpolator/data_in_$i
    ad_connect tx_fir_interpolator/valid_out_$i tx_rate64_fifo_$i/m_axis_tready
}
