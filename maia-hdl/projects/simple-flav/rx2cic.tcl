# RX2 x8 decimation for the Pluto trx bitstream (xc7z010), in fabric logic:
# rxfir.tcl's FIR for RX1 has left no DSP slice for a second one, so RX2
# goes through a four-stage CIC (rx2_cic.v, gain and delay matched to that
# FIR). With the AD9363 in 2R2T, trxd's noise canceller then gets RX2 at
# the decimated rate and band-limited rather than raw on RX1's strobe.
#
# The packer's lanes 2/3 (RX2 I/Q) take the CIC outputs; its write enable
# stays RX1's FIR output valid, the CIC counting the same input valids.

proc rx2cic_move_sink {pin new_sink} {
    set p [get_bd_pins $pin]
    set net [get_bd_nets -of_objects $p]
    if {$net eq ""} { error "rx2cic: $pin is not driven" }
    disconnect_bd_net $net $p
    connect_bd_net -net $net [get_bd_pins $new_sink]
}

add_files -norecurse [file normalize rx2_cic.v]
update_compile_order -fileset sources_1
create_bd_cell -type module -reference rx2_cic rx2_cic_0

ad_connect util_ad9361_divclk/clk_out rx2_cic_0/clk
ad_connect rx_decim_slice/Dout rx2_cic_0/active
set rx2_valid [get_bd_nets -of_objects [get_bd_pins rx_fir_decimator/valid_in_0]]
if {$rx2_valid eq ""} { error "rx2cic: rx_fir_decimator/valid_in_0 is not driven" }
connect_bd_net -net $rx2_valid [get_bd_pins rx2_cic_0/valid]

rx2cic_move_sink util_ad9361_adc_pack/fifo_wr_data_2 rx2_cic_0/in_i
rx2cic_move_sink util_ad9361_adc_pack/fifo_wr_data_3 rx2_cic_0/in_q
ad_connect rx2_cic_0/out_i util_ad9361_adc_pack/fifo_wr_data_2
ad_connect rx2_cic_0/out_q util_ad9361_adc_pack/fifo_wr_data_3
