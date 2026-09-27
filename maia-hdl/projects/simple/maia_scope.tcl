# Maia SDR's spectrometer as the web UI's wide scope: window, FFT (4096),
# power and averaging in the fabric, spectra DMA'd into a ring of 8 buffers at
# 0x16000000 (reserved in the device tree), one interrupt per buffer.
#
# Taken from common/maia.tcl, minus everything that rerouted the IIO DMA path
# through Maia's DDC: here the spectrometer only listens to the full-rate ADC
# stream.
#
# DATV (maia_iio_lite_datv): the DDC is the DVB-S2 receive front end (channel
# filter + RRC, set up by trxd) and the recorder writes its output, in ring
# mode, into 1 MiB at 0x16100000 (reserved in the device tree next to the
# spectra); trxd follows the ring's committed_address register.

create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz:6.0 maia_sdr_clk
set_property -dict [list CONFIG.USE_PHASE_ALIGNMENT {false} CONFIG.ENABLE_CLOCK_MONITOR {false} CONFIG.PRIM_SOURCE {Global_buffer} \
                        CONFIG.CLKOUT2_USED {true} CONFIG.CLKOUT3_USED {true} CONFIG.NUM_OUT_CLKS {3} \
                        CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {62.500} CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {125.000} \
                        CONFIG.CLKOUT3_REQUESTED_OUT_FREQ {187.5} \
                        CONFIG.PRIMITIVE {MMCM} CONFIG.MMCM_DIVCLK_DIVIDE {1} CONFIG.MMCM_CLKFBOUT_MULT_F {11.250} \
                        CONFIG.MMCM_CLKOUT0_DIVIDE_F {18.000} CONFIG.MMCM_CLKOUT1_DIVIDE {9} \
                        CONFIG.MMCM_CLKOUT3_DIVIDE {6} \
                        CONFIG.CLKOUT1_JITTER {133.663} CONFIG.CLKOUT1_PHASE_ERROR {91.100} \
                        CONFIG.CLKOUT2_JITTER {116.571} CONFIG.CLKOUT2_PHASE_ERROR {91.100} \
                        CONFIG.CLKOUT3_JITTER {108.217} CONFIG.CLKOUT3_PHASE_ERROR {91.100}] [get_bd_cells maia_sdr_clk]

ad_ip_instance maia_sdr_maia_iio_lite_datv maia_sdr

# 12-bit I/Q straight from the ADC FIFO (before the x8 decimator).
ad_ip_instance xlslice adc_i_slice
ad_ip_parameter adc_i_slice CONFIG.DIN_WIDTH 16
ad_ip_parameter adc_i_slice CONFIG.DOUT_WIDTH 12
ad_ip_parameter adc_i_slice CONFIG.DIN_FROM 11
ad_ip_instance xlslice adc_q_slice
ad_ip_parameter adc_q_slice CONFIG.DIN_TO 0
ad_ip_parameter adc_q_slice CONFIG.DIN_WIDTH 16
ad_ip_parameter adc_q_slice CONFIG.DOUT_WIDTH 12
ad_ip_parameter adc_q_slice CONFIG.DIN_FROM 11
ad_connect util_ad9361_adc_fifo/dout_data_0 adc_i_slice/Din
ad_connect util_ad9361_adc_fifo/dout_data_1 adc_q_slice/Din
ad_connect adc_i_slice/Dout maia_sdr/re_in
ad_connect adc_q_slice/Dout maia_sdr/im_in
# Only cycles with a sample (the sampling clock runs faster than the samples
# come: without this about 1 % of them went in twice).
ad_connect util_ad9361_adc_fifo/dout_valid_0 maia_sdr/valid_in

ad_connect maia_sdr/sampling_clk util_ad9361_divclk/clk_out
ad_connect sys_cpu_clk maia_sdr/s_axi_lite_clk
ad_connect sys_cpu_reset maia_sdr/s_axi_lite_rst
ad_connect maia_sdr_clk/clk_out1 maia_sdr/clk
ad_connect maia_sdr_clk/clk_out2 maia_sdr/clk2x_clk
ad_connect maia_sdr_clk/clk_out3 maia_sdr/clk3x_clk
ad_connect sys_cpu_clk maia_sdr_clk/clk_in1
ad_connect sys_cpu_reset maia_sdr_clk/reset

ad_cpu_interconnect 0x7C460000 maia_sdr
ad_cpu_interrupt ps-11 mb-11 maia_sdr/interrupt_out

# Spectra to DDR through HP1; the DATV ring recorder (2 MB/s) shares HP2 with
# the IIO DMAs.
ad_ip_parameter sys_ps7 CONFIG.PCW_USE_S_AXI_HP1 {1}
ad_connect maia_sdr_clk/clk_out1 sys_ps7/S_AXI_HP1_ACLK
ad_connect maia_sdr/m_axi_spectrometer sys_ps7/S_AXI_HP1
ad_mem_hp2_interconnect sys_cpu_clk maia_sdr/m_axi_recorder
create_bd_addr_seg -range 0x20000000 -offset 0x00000000 \
                    [get_bd_addr_spaces maia_sdr/m_axi_spectrometer] \
                    [get_bd_addr_segs sys_ps7/S_AXI_HP1/HP1_DDR_LOWOCM] \
                    SEG_sys_ps7_HP1_DDR_LOWOCM
