# LibreSDR fixed-TCXO frequency correction (iq_xo_corrector NCO), inserted at
# the one point of each path that every sample crosses:
#
#   RX: adc_fifo -> NCO -> rx_fir_decimator / packer (RX2)
#   TX: tx_fir_interpolator / upack (TX2) -> NCO -> dac_fifo
#
# Exact latency-matched bypass after reset; software enables it through the
# vctcxo_lock NCO registers (0x43C00020..0x43C00028), see
# board/tezuka/libre/README-tx-nco.md in tezuka_fw.

create_bd_cell -type module -reference iq_xo_corrector iq_xo_corrector

ad_connect util_ad9361_divclk/clk_out iq_xo_corrector/clk
ad_connect util_ad9361_divclk_reset/peripheral_aresetn iq_xo_corrector/rst_n
ad_connect vctcxo_lock/NCO_RX_FTW iq_xo_corrector/cfg_rx_ftw
ad_connect vctcxo_lock/NCO_TX_FTW iq_xo_corrector/cfg_tx_ftw
ad_connect vctcxo_lock/NCO_CONTROL iq_xo_corrector/cfg_control

# RX
simple_disconnect_sink rx_fir_decimator/data_in_0
simple_disconnect_sink rx_fir_decimator/data_in_1
simple_disconnect_sink rx_fir_decimator/valid_in_0
simple_disconnect_sink rx_fir_decimator/valid_in_1
simple_disconnect_sink util_ad9361_adc_pack/fifo_wr_data_2
simple_disconnect_sink util_ad9361_adc_pack/fifo_wr_data_3

ad_connect util_ad9361_adc_fifo/dout_valid_0 iq_xo_corrector/rx_valid_in
ad_connect util_ad9361_adc_fifo/dout_data_0  iq_xo_corrector/rx_i0_in
ad_connect util_ad9361_adc_fifo/dout_data_1  iq_xo_corrector/rx_q0_in
ad_connect util_ad9361_adc_fifo/dout_data_2  iq_xo_corrector/rx_i1_in
ad_connect util_ad9361_adc_fifo/dout_data_3  iq_xo_corrector/rx_q1_in
ad_connect iq_xo_corrector/rx_valid_out rx_fir_decimator/valid_in_0
ad_connect iq_xo_corrector/rx_valid_out rx_fir_decimator/valid_in_1
ad_connect iq_xo_corrector/rx_i0_out    rx_fir_decimator/data_in_0
ad_connect iq_xo_corrector/rx_q0_out    rx_fir_decimator/data_in_1
ad_connect iq_xo_corrector/rx_i1_out    util_ad9361_adc_pack/fifo_wr_data_2
ad_connect iq_xo_corrector/rx_q1_out    util_ad9361_adc_pack/fifo_wr_data_3

# TX
simple_disconnect_sink axi_ad9361_dac_fifo/din_data_0
simple_disconnect_sink axi_ad9361_dac_fifo/din_data_1
simple_disconnect_sink axi_ad9361_dac_fifo/din_data_2
simple_disconnect_sink axi_ad9361_dac_fifo/din_data_3
simple_disconnect_sink axi_ad9361_dac_fifo/din_valid_in_0
simple_disconnect_sink axi_ad9361_dac_fifo/din_valid_in_1
simple_disconnect_sink axi_ad9361_dac_fifo/din_valid_in_2
simple_disconnect_sink axi_ad9361_dac_fifo/din_valid_in_3

ad_connect tx_fir_interpolator/data_out_0       iq_xo_corrector/tx_i0_in
ad_connect tx_fir_interpolator/data_out_1       iq_xo_corrector/tx_q0_in
ad_connect util_ad9361_dac_upack/fifo_rd_data_2 iq_xo_corrector/tx_i1_in
ad_connect util_ad9361_dac_upack/fifo_rd_data_3 iq_xo_corrector/tx_q1_in
ad_connect dac_fifo_valid_or/Res                iq_xo_corrector/tx_valid0_in
ad_connect util_ad9361_dac_upack/fifo_rd_valid  iq_xo_corrector/tx_valid1_in

ad_connect iq_xo_corrector/tx_i0_out     axi_ad9361_dac_fifo/din_data_0
ad_connect iq_xo_corrector/tx_q0_out     axi_ad9361_dac_fifo/din_data_1
ad_connect iq_xo_corrector/tx_i1_out     axi_ad9361_dac_fifo/din_data_2
ad_connect iq_xo_corrector/tx_q1_out     axi_ad9361_dac_fifo/din_data_3
ad_connect iq_xo_corrector/tx_valid0_out axi_ad9361_dac_fifo/din_valid_in_0
ad_connect iq_xo_corrector/tx_valid0_out axi_ad9361_dac_fifo/din_valid_in_1
ad_connect iq_xo_corrector/tx_valid1_out axi_ad9361_dac_fifo/din_valid_in_2
ad_connect iq_xo_corrector/tx_valid1_out axi_ad9361_dac_fifo/din_valid_in_3
