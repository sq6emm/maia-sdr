# Compressed bitstream: the Zynq configuration logic decompresses it itself,
# so FSBL/U-Boot `fpga load` need nothing new. Shrinks the FPGA image in each
# 12.25 MB flash slot from ~2 MB to a few hundred KB (tezuka_fw_simple
# docs/FLASH.md).
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
