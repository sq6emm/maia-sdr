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
                   words are laid out in banks: see there; with the DDR
                   engine, ldpc_dma.py: "LDP6")
  with dma=True: control bit 2 (with bit 0): take the LLR words from DDR,
  decode, write the decisions back packed (bit j of word k: the sign of RAM
  variable 32 k + j); 0xFF10 their DDR address in, 0xFF14 out (bytes,
  128-aligned), 0xFF18 words in (<= 16200), 0xFF1C words out (multiple of
  32, <= 2048); 0xFF20 bit 0: the words are DVB-T2 QPSK cells (the engine
  makes the LLRs, ldpc_dma.py), bit 1 rotated, bit 2 load only (tests),
  bit 3 16QAM (four LLRs a cell and the bit deinterleaver, id "LDP6");
  0xFF24 kq; 0xFF28 c14 (15:0), s14 (31:16); 0xFF2C a14 (16QAM level).
"""

import argparse

from amaranth import *
import amaranth.back.verilog

from .dvbs2_tables import TABLES
from .ldpc_dec import LdpcDecoder
from .ldpc_dec4 import LdpcDecoder4
from .ldpc_dma import LdpcDma

ID = 0x3150444C  # "LDP1"
ID4 = 0x3450444C  # "LDP4"
ID5 = 0x3650444C  # "LDP6": four lanes and the DDR engine (QPSK + 16QAM cells)


class LdpcAxi(Elaboratable):
    def __init__(self, lanes=1, dma=False):
        self.dec = LdpcDecoder4(TABLES) if lanes == 4 else LdpcDecoder(TABLES)
        self.id = ID4 if lanes == 4 else ID
        self.dma = LdpcDma() if dma else None
        if dma:
            self.id = ID5
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
                self.s_axi_rvalid, self.s_axi_rready] + (self.dma.axi.ports() if self.dma else [])

    def elaborate(self, platform):
        m = Module()
        # The decoder and the DDR engine on a registered copy of the reset
        # (max_fanout: replicated near them): straight from the system reset
        # it reached thousands of registers across the chip (10 ns of route).
        m.domains.dec = cd_dec = ClockDomain('dec', local=True)
        rst_q = Signal(2, reset_less=True)
        rst_dec = Signal(reset_less=True, attrs={'max_fanout': '64'})
        m.d.sync += [rst_q.eq(Cat(ResetSignal('sync'), rst_q[0])), rst_dec.eq(rst_q[1])]
        m.d.comb += [cd_dec.clk.eq(ClockSignal('sync')), cd_dec.rst.eq(rst_dec)]
        dec = self.dec
        m.submodules.dec = DomainRenamer('dec')(dec)
        rate = Signal()
        max_iter = Signal(6, init=50)
        m.d.comb += [dec.rate.eq(rate), dec.max_iter.eq(max_iter)]
        dma = self.dma
        busy = Signal()
        m.d.comb += busy.eq(dec.busy | (dma.busy if dma else 0))
        if dma:
            m.submodules.dma = DomainRenamer('dec')(dma)

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
                            go = self.s_axi_wdata[0] & ~busy
                            if dma:
                                m.d.comb += [dec.start.eq(go & ~self.s_axi_wdata[2]),
                                             dma.go.eq(go & self.s_axi_wdata[2])]
                            else:
                                m.d.comb += dec.start.eq(go)
                        if dma:
                            with m.Switch(self.s_axi_awaddr[:8]):
                                with m.Case(0x10):
                                    m.d.sync += dma.in_addr.eq(self.s_axi_wdata)
                                with m.Case(0x14):
                                    m.d.sync += dma.out_addr.eq(self.s_axi_wdata)
                                with m.Case(0x18):
                                    m.d.sync += dma.in_words.eq(self.s_axi_wdata)
                                with m.Case(0x1C):
                                    m.d.sync += dma.out_words.eq(self.s_axi_wdata)
                                with m.Case(0x20):
                                    m.d.sync += [dma.cells.eq(self.s_axi_wdata[0]),
                                                 dma.rot.eq(self.s_axi_wdata[1]),
                                                 dma.load_only.eq(self.s_axi_wdata[2]),
                                                 dma.qam16.eq(self.s_axi_wdata[3])]
                                with m.Case(0x24):
                                    m.d.sync += dma.kq.eq(self.s_axi_wdata)
                                with m.Case(0x28):
                                    m.d.sync += [dma.c14.eq(self.s_axi_wdata[:16]),
                                                 dma.s14.eq(self.s_axi_wdata[16:])]
                                with m.Case(0x2C):
                                    m.d.sync += dma.a14.eq(self.s_axi_wdata[:20])
                    with m.Elif(~busy):
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
                            m.d.sync += rd_regval.eq(Cat(busy, dec.converged,
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
        if dma:
            # the DDR engine has the decoder's port while it runs (go needs
            # the AXI FSM in IDLE, so no CPU access is in flight)
            m.d.comb += [dma.dec_rdata.eq(dec.cpu_rdata), dma.dec_busy.eq(dec.busy),
                         dma.dec_rate.eq(rate)]
            with m.If(dma.busy):
                m.d.comb += [dec.cpu_addr.eq(dma.dec_addr), dec.cpu_wdata.eq(dma.dec_wdata),
                             dec.cpu_we.eq(dma.dec_we), dec.cpu_re.eq(dma.dec_re),
                             dec.start.eq(dma.dec_start)]
        return m


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output_file')
    parser.add_argument('--lanes', type=int, default=1, choices=[1, 4])
    parser.add_argument('--dma', action='store_true')
    args = parser.parse_args()
    top = LdpcAxi(lanes=args.lanes, dma=args.dma)
    with open(args.output_file, 'w') as f:
        f.write(amaranth.back.verilog.convert(
            top, name='ldpc_axi', ports=top.ports(), emit_src=False))
