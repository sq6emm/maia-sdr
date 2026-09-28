# DATV transmit in the fabric (DVB-S2 long frames, pilots; any symbol rate):
#
#   DAC DMA -> datv_split --(sel=0)--> upack -> x8 interpolator -> ... (IQ, as always)
#                  \--(sel=1)--> async FIFO -> DVB-S2 encoder (ORI dvb_fpga, as
#                        in F5OEO's tezuka) -> datv_tx (arbitrary-rate RRC,
#                        maia_hdl/arb_interp.py) -> async FIFO -> datv_merge
#                        -> (in place of the x8 interpolator's output) -> XO NCO -> DAC
#
# sel is bit 1 of the axi_ad9361 DAC GPIO-out register (0x790240BC); it also
# releases the encoder's reset. trxd writes framed BBFRAMEs into the IIO TX
# buffer (0xB8, config byte: bit 6 = 0 long, bit 5 pilots, 4:0 MODCOD).
#
#   0x43C20000 datv_tx registers (step, coefficient table, id "DTX1")
#   0x43C30000 encoder registers (dvb_fpga)

set dvb_fpga_vivado_dir [file normalize \
    [file join $::tezuka_hdl_dir .. dvb_fpga build vivado]]
source [file join $dvb_fpga_vivado_dir add_dvbs2_files.tcl]
add_files -norecurse [file join $dvb_fpga_vivado_dir dvbs2_encoder_wrapper.vhd]
add_files -norecurse [list [file normalize datv_split.v] \
                          [file normalize datv_raw.v] \
                          [file normalize datv_merge.v] \
                          [file normalize datv_tx.v] \
                          [file normalize t2ifft.v]]
update_compile_order -fileset sources_1

ad_ip_instance xlslice datv_slice
ad_ip_parameter datv_slice CONFIG.DIN_FROM 1
ad_ip_parameter datv_slice CONFIG.DIN_TO 1
ad_connect axi_ad9361/up_dac_gpio_out datv_slice/Din

# DMA -> split (divclk domain, like the DMA's stream side)
create_bd_cell -type module -reference datv_split datv_split_0
ad_connect util_ad9361_divclk/clk_out datv_split_0/clk
ad_connect datv_slice/Dout datv_split_0/sel_async
set net [get_bd_intf_nets -quiet -of_objects [get_bd_intf_pins axi_ad9361_dac_dma/m_axis]]
if {$net ne ""} { delete_bd_objs $net }
ad_connect axi_ad9361_dac_dma/m_axis datv_split_0/s_axis
ad_connect datv_split_0/m_norm_axis util_ad9361_dac_upack/s_axis

# -> CPU clock
ad_ip_instance axis_data_fifo datv_fifo_in
ad_ip_parameter datv_fifo_in CONFIG.FIFO_DEPTH 512
ad_ip_parameter datv_fifo_in CONFIG.IS_ACLK_ASYNC 1
ad_connect util_ad9361_divclk/clk_out datv_fifo_in/s_axis_aclk
ad_connect util_ad9361_divclk_reset/peripheral_aresetn datv_fifo_in/s_axis_aresetn
ad_connect sys_cpu_clk datv_fifo_in/m_axis_aclk
ad_connect datv_split_0/m_datv_axis datv_fifo_in/S_AXIS

# Encoder (in reset unless DATV is on)
create_bd_cell -type module -reference dvbs2_encoder_wrapper datv_encoder
ad_ip_parameter datv_encoder CONFIG.INPUT_DATA_WIDTH 64
ad_connect sys_cpu_clk datv_encoder/clk
ad_connect datv_slice/Dout datv_encoder/rst_n
ad_cpu_interconnect 0x43C30000 datv_encoder
# Raw IQ (DAC GPIO bit 2, DVB-T2): the DMA words skip the encoder.
ad_ip_instance xlslice datv_raw_slice
ad_ip_parameter datv_raw_slice CONFIG.DIN_FROM 2
ad_ip_parameter datv_raw_slice CONFIG.DIN_TO 2
ad_connect axi_ad9361/up_dac_gpio_out datv_raw_slice/Din
create_bd_cell -type module -reference datv_raw datv_raw_0
ad_connect sys_cpu_clk datv_raw_0/clk
ad_connect datv_raw_slice/Dout datv_raw_0/raw
ad_ip_instance xlslice datv_raw8_slice
ad_ip_parameter datv_raw8_slice CONFIG.DIN_FROM 3
ad_ip_parameter datv_raw8_slice CONFIG.DIN_TO 3
ad_connect axi_ad9361/up_dac_gpio_out datv_raw8_slice/Din
ad_connect datv_raw8_slice/Dout datv_raw_0/mode8
ad_connect datv_fifo_in/M_AXIS datv_raw_0/s_dma_axis
ad_connect datv_raw_0/m_enc_axis datv_encoder/s_axis

# Pulse shaping and resampling to the DAC rate
create_bd_cell -type module -reference datv_tx datv_tx_0
ad_connect sys_cpu_clk datv_tx_0/clk
ad_connect sys_cpu_reset datv_tx_0/rst
ad_cpu_interconnect 0x43C20000 datv_tx_0
ad_connect datv_encoder/m_axis datv_raw_0/s_sym_axis
# DVB-T2 transmit IFFT (maia_hdl/t2ifft.py; DAC GPIO bit 4): the raw words
# are P1 samples and each symbol's carriers, the IFFT and guard interval
# happen here. Off: the raw words pass straight through.
ad_ip_instance xlslice t2ifft_slice
ad_ip_parameter t2ifft_slice CONFIG.DIN_FROM 4
ad_ip_parameter t2ifft_slice CONFIG.DIN_TO 4
ad_connect axi_ad9361/up_dac_gpio_out t2ifft_slice/Din
create_bd_cell -type module -reference t2ifft t2ifft_0
ad_connect sys_cpu_clk t2ifft_0/clk
ad_connect sys_cpu_reset t2ifft_0/rst
ad_connect t2ifft_slice/Dout t2ifft_0/enable
ad_connect datv_raw_0/m_tx_axis t2ifft_0/s_axis
ad_connect t2ifft_0/m_axis datv_tx_0/s_axis

# -> DAC clock
ad_ip_instance axis_data_fifo datv_fifo_out
ad_ip_parameter datv_fifo_out CONFIG.FIFO_DEPTH 512
ad_ip_parameter datv_fifo_out CONFIG.IS_ACLK_ASYNC 1
ad_connect sys_cpu_clk datv_fifo_out/s_axis_aclk
ad_connect sys_cpu_resetn datv_fifo_out/s_axis_aresetn
ad_connect util_ad9361_divclk/clk_out datv_fifo_out/m_axis_aclk
ad_connect datv_tx_0/m_axis datv_fifo_out/S_AXIS

# Merge in front of the XO corrector (Libre) or the DAC FIFO (PlutoSky R2)
create_bd_cell -type module -reference datv_merge datv_merge_0
ad_connect util_ad9361_divclk/clk_out datv_merge_0/clk
ad_connect datv_slice/Dout datv_merge_0/sel_async
ad_connect axi_ad9361_dac_fifo/din_valid_0 datv_merge_0/req
ad_connect datv_fifo_out/M_AXIS datv_merge_0/s_axis
if {[info exists xo_corrector]} {
    simple_disconnect_sink iq_xo_corrector/tx_i0_in
    simple_disconnect_sink iq_xo_corrector/tx_q0_in
    simple_disconnect_sink iq_xo_corrector/tx_valid0_in
    ad_connect tx_fir_interpolator/data_out_0 datv_merge_0/norm_i
    ad_connect tx_fir_interpolator/data_out_1 datv_merge_0/norm_q
    ad_connect dac_fifo_valid_or/Res          datv_merge_0/norm_valid
    ad_connect datv_merge_0/out_i     iq_xo_corrector/tx_i0_in
    ad_connect datv_merge_0/out_q     iq_xo_corrector/tx_q0_in
    ad_connect datv_merge_0/out_valid iq_xo_corrector/tx_valid0_in
} else {
    simple_disconnect_sink axi_ad9361_dac_fifo/din_data_0
    simple_disconnect_sink axi_ad9361_dac_fifo/din_data_1
    simple_disconnect_sink axi_ad9361_dac_fifo/din_valid_in_0
    simple_disconnect_sink axi_ad9361_dac_fifo/din_valid_in_1
    ad_connect tx_fir_interpolator/data_out_0 datv_merge_0/norm_i
    ad_connect tx_fir_interpolator/data_out_1 datv_merge_0/norm_q
    ad_connect dac_fifo_valid_or/Res          datv_merge_0/norm_valid
    ad_connect datv_merge_0/out_i     axi_ad9361_dac_fifo/din_data_0
    ad_connect datv_merge_0/out_q     axi_ad9361_dac_fifo/din_data_1
    ad_connect datv_merge_0/out_valid axi_ad9361_dac_fifo/din_valid_in_0
    ad_connect datv_merge_0/out_valid axi_ad9361_dac_fifo/din_valid_in_1
}
