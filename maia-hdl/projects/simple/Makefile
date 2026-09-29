####################################################################################
# tezuka_fw_simple bitstreams: AD9361 + DMA + x8 RX decimator / TX interpolator.
#   PROJECT_NAME=plutoskyr2 make all
#   PROJECT_NAME=libre make all
# `all` builds the IP cores, the project, and copies the .xsa into
# $(TEZUKA_SIMPLE)/board/tezuka/<board>/bitstream/simple.xsa
####################################################################################

PROJECT_NAME ?= plutoskyr2

export PROJECT_NAME

M_DEPS += ../../adi-hdl/library/common/ad_iobuf.v
M_DEPS += ../../adi-hdl/library/axi_ad9361/axi_ad9361_delay.tcl
M_DEPS += rxfir.tcl
M_DEPS += xo_corrector.tcl
M_DEPS += refmeter.v
M_DEPS += datv_tx.tcl
M_DEPS += datv_tx.v
M_DEPS += t2ifft.v
M_DEPS += datv_split.v
M_DEPS += datv_raw.v
M_DEPS += datv_merge.v
M_DEPS += ldpc.tcl
M_DEPS += ldpc_axi.v
M_DEPS += rsnn.tcl
M_DEPS += rsnn_front_axi.v
M_DEPS += plutoskyr2_system_top.v
M_DEPS += plutoskyr2_simple.xdc
M_DEPS += bitstream.xdc
M_DEPS += datv.xdc
M_DEPS += maia_scope.tcl
M_DEPS += ../common/txfir.tcl
M_DEPS += ip_cores
LIB_DEPS += xilinx/util_clkdiv
LIB_DEPS += axi_ad9361
LIB_DEPS += util_rfifo
LIB_DEPS += util_wfifo
LIB_DEPS += axi_dmac
LIB_DEPS += util_pack/util_cpack2
LIB_DEPS += util_pack/util_upack2

TEZUKA_SIMPLE ?= $(abspath ../../../../tezuka_fw_simple)
DEST = $(TEZUKA_SIMPLE)/board/tezuka/$(PROJECT_NAME)/bitstream
include ../../adi-hdl/projects/scripts/project-xilinx.mk

build_project: $(PROJECT_NAME).xpr

ip_cores:
	$(MAKE) -C ../../ip

clean:
	bash -c "rm -rf build $(PROJECT_NAME).*"
	bash -c "rm -f *.log *.jou *.str *.xpr *.xsa"

# FPGA_MODE=trx|datv: $(DEST)/simple-<mode>.xsa (all: simple.xsa, the boot one)
FPGA_MODE ?= all
export FPGA_MODE
XSA_NAME = $(if $(filter all,$(FPGA_MODE)),simple.xsa,simple-$(FPGA_MODE).xsa)

install:
	bash -c "mkdir -p $(DEST) && cp -f $(PROJECT_NAME).sdk/system_top.xsa $(DEST)/$(XSA_NAME)"

all: ip_cores build_project install

.PHONY: ip_cores build_project install all clean
