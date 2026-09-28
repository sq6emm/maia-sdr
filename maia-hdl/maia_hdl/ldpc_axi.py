#
# SPDX-License-Identifier: MIT
#

"""LdpcDecoder behind an AXI4-Lite slave (64 KiB window, CPU clock).

  0x0000-0xFD1F  posterior RAM, 4 bytes a word (byte v % 4 of word v // 4
                 is variable v): write the LLRs (6-bit, sign-extended),
                 read the decisions (sign bits). Only while not busy.
  0xFF00  control  W: bit 0 start; bit 1 rate (0 = 1/2, 1 = 3/4);
                   bits 13:8 maximum iterations
  0xFF04  status   R: bit 0 busy, bit 1 converged, bits 13:8 iterations
  0xFF08  id       "LDP1" (lanes 4: "LDP4", ldpc_dec4.py, whose parity
                   words are laid out in banks: see there)
"""

import argparse

from amaranth import *
import amaranth.back.verilog

from .dvbs2_tables import TABLES
from .ldpc_dec import LdpcDecoder
from .ldpc_dec4 import LdpcDecoder4

ID = 0x3150444C  # "LDP1"
ID4 = 0x3450444C  # "LDP4"


class LdpcAxi(Elaboratable):
    def __init__(self, lanes=1):
        self.dec = LdpcDecoder4(TABLES) if lanes == 4 else LdpcDecoder(TABLES)
        self.id = ID4 if lanes == 4 else ID
        aw = 16
        self.s_axi_awaddr = Signal(aw)
        self.s_axi_awvalid = Signal()
        self.s_axi_awready = Signal()
        self.s_axi_wdata = Signal(32)
        self.s_axi_wstrb = Signal(4)
        self.s_axi_wvalid = Signal()
        self.s_axi_wready = Signal()
        self.s_axi_bresp = Signal(2)
        self.s_axi_bvalid = Signal()
        self.s_axi_bready = Signal()
        self.s_axi_araddr = Signal(aw)
        self.s_axi_arvalid = Signal()
        self.s_axi_arready = Signal()
        self.s_axi_rdata = Signal(32)
        self.s_axi_rresp = Signal(2)
        self.s_axi_rvalid = Signal()
        self.s_axi_rready = Signal()

    def ports(self):
        return [self.s_axi_awaddr, self.s_axi_awvalid, self.s_axi_awready,
                self.s_axi_wdata, self.s_axi_wstrb, self.s_axi_wvalid,
                self.s_axi_wready, self.s_axi_bresp, self.s_axi_bvalid,
                self.s_axi_bready, self.s_axi_araddr, self.s_axi_arvalid,
                self.s_axi_arready, self.s_axi_rdata, self.s_axi_rresp,
                self.s_axi_rvalid, self.s_axi_rready]

    def elaborate(self, platform):
        m = Module()
        m.submodules.dec = dec = self.dec
        rate = Signal()
        max_iter = Signal(6, init=50)
        m.d.comb += [dec.rate.eq(rate), dec.max_iter.eq(max_iter)]

        is_reg_w = self.s_axi_awaddr[8:] == 0xFF
        is_reg_r = self.s_axi_araddr[8:] == 0xFF
        m.d.comb += [self.s_axi_bresp.eq(0), self.s_axi_rresp.eq(0)]  # OKAY
        rd_reg = Signal()
        rd_regval = Signal(32)
        with m.FSM():
            with m.State('IDLE'):
                with m.If(self.s_axi_awvalid & self.s_axi_wvalid):
                    m.d.comb += [self.s_axi_awready.eq(1), self.s_axi_wready.eq(1)]
                    with m.If(is_reg_w):
                        with m.If(self.s_axi_awaddr[:8] == 0x00):
                            m.d.sync += [rate.eq(self.s_axi_wdata[1]),
                                         max_iter.eq(self.s_axi_wdata[8:14])]
                            m.d.comb += dec.start.eq(self.s_axi_wdata[0] & ~dec.busy)
                    with m.Elif(~dec.busy):
                        m.d.comb += [dec.cpu_addr.eq(self.s_axi_awaddr[2:]),
                                     dec.cpu_wdata.eq(self.s_axi_wdata),
                                     dec.cpu_we.eq(self.s_axi_wstrb)]
                    m.next = 'BRESP'
                with m.Elif(self.s_axi_arvalid):
                    m.d.comb += self.s_axi_arready.eq(1)
                    m.d.sync += rd_reg.eq(is_reg_r)
                    with m.Switch(self.s_axi_araddr[:8]):
                        with m.Case(0x00):
                            m.d.sync += rd_regval.eq(Cat(C(0, 1), rate, C(0, 6), max_iter))
                        with m.Case(0x04):
                            m.d.sync += rd_regval.eq(Cat(dec.busy, dec.converged,
                                                         C(0, 6), dec.iterations))
                        with m.Case(0x08):
                            m.d.sync += rd_regval.eq(self.id)
                        with m.Default():
                            m.d.sync += rd_regval.eq(0)
                    m.d.comb += [dec.cpu_addr.eq(self.s_axi_araddr[2:]),
                                 dec.cpu_re.eq(~is_reg_r)]
                    m.next = 'RDATA'
            with m.State('BRESP'):
                m.d.comb += self.s_axi_bvalid.eq(1)
                with m.If(self.s_axi_bready):
                    m.next = 'IDLE'
            with m.State('RDATA'):
                # the RAM word read in IDLE is on cpu_rdata now; hold it
                rdata = Signal(32)
                first = Signal(init=1)
                with m.If(first):
                    m.d.sync += [rdata.eq(Mux(rd_reg, rd_regval, dec.cpu_rdata)),
                                 first.eq(0)]
                m.d.comb += [self.s_axi_rvalid.eq(~first), self.s_axi_rdata.eq(rdata)]
                with m.If(~first & self.s_axi_rready):
                    m.d.sync += first.eq(1)
                    m.next = 'IDLE'
        return m


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output_file')
    parser.add_argument('--lanes', type=int, default=1, choices=[1, 4])
    args = parser.parse_args()
    top = LdpcAxi(lanes=args.lanes)
    with open(args.output_file, 'w') as f:
        f.write(amaranth.back.verilog.convert(
            top, name='ldpc_axi', ports=top.ports(), emit_src=False))
