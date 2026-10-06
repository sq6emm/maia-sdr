# x8 RX FIR decimator for channel 0 (I/Q), in the divided sample-clock domain
# between the ADC FIFO and the DMA packer - the RX twin of common/txfir.tcl.
#
# Bypassed after reset; bit 0 of axi_ad9361 up_adc_gpio_out (0x790200BC)
# switches it in. Channel 1 (RX2) keeps flowing straight to the packer and is
# written on the same (decimated) valid strobe.

simple_disconnect_sink util_ad9361_adc_pack/fifo_wr_data_0
simple_disconnect_sink util_ad9361_adc_pack/fifo_wr_data_1
simple_disconnect_sink util_ad9361_adc_pack/enable_0
simple_disconnect_sink util_ad9361_adc_pack/enable_1
simple_disconnect_sink util_ad9361_adc_pack/fifo_wr_en

ad_add_decimation_filter "rx_fir_decimator" 8 2 1 {61.44} {61.44} \
                         "$ad_hdl_dir/library/util_fir_int/coefile_int.coe"

ad_connect util_ad9361_divclk/clk_out rx_fir_decimator/aclk

ad_ip_instance xlslice rx_decim_slice
ad_ip_parameter rx_decim_slice CONFIG.DIN_WIDTH 32
ad_ip_parameter rx_decim_slice CONFIG.DIN_FROM 0
ad_ip_parameter rx_decim_slice CONFIG.DIN_TO 0
ad_connect axi_ad9361/up_adc_gpio_out rx_decim_slice/Din
ad_connect rx_decim_slice/Dout rx_fir_decimator/active

ad_connect util_ad9361_adc_fifo/dout_valid_0  rx_fir_decimator/valid_in_0
ad_connect util_ad9361_adc_fifo/dout_enable_0 rx_fir_decimator/enable_in_0
ad_connect util_ad9361_adc_fifo/dout_data_0   rx_fir_decimator/data_in_0
ad_connect util_ad9361_adc_fifo/dout_valid_1  rx_fir_decimator/valid_in_1
ad_connect util_ad9361_adc_fifo/dout_enable_1 rx_fir_decimator/enable_in_1
ad_connect util_ad9361_adc_fifo/dout_data_1   rx_fir_decimator/data_in_1

ad_connect rx_fir_decimator/data_out_0   util_ad9361_adc_pack/fifo_wr_data_0
ad_connect rx_fir_decimator/data_out_1   util_ad9361_adc_pack/fifo_wr_data_1
ad_connect rx_fir_decimator/enable_out_0 util_ad9361_adc_pack/enable_0
ad_connect rx_fir_decimator/enable_out_1 util_ad9361_adc_pack/enable_1
ad_connect rx_fir_decimator/valid_out_0  util_ad9361_adc_pack/fifo_wr_en
