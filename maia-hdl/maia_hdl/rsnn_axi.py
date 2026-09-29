#
# SPDX-License-Identifier: MIT
#

"""The CW-RS keying detector in the FPGA: the front end (rsnn_front.py)
and the temporal layers (rsnn_temporal.py, weights from DDR) behind one
AXI4-Lite slave (64 KiB window, CPU clock) and an AXI3 read master.

    0x0000-0x87FF  front weights, two a word (low 16 bits: the even one)
    0x9000-0x93FF  front biases
    0x9400-0x947F  the next feature row, two bins a word (low: the even)
    0x9500-0x95FF  the front's outputs (frame n - 3)
    0x9600-0x96FF  the last temporal layer's outputs (when bit 2 of status)
    0x9800-0x9FFF  temporal biases ([layer][out], 64 a layer)
    0xFF00  control  W: bit 0 go (the row is in), bit 1 reset (zero the
                     rings); 12:8 c2, 22:16 c1 (kept)
    0xFF04  status   R: bit 0 busy, bit 1 the front's outputs are there,
                     bit 2 the temporal layers gave a frame
    0xFF08  id       "RSF2"
    0xFF0C  frames   R: temporal output frames since the reset
    0xFF10  layers   temporal layers (0: the front alone)
    0xFF14  weights  the temporal weights' DDR address (128-byte aligned)
    0xFF18  words    64-bit words of them
    0xFF20+4l layer l (0..7): 6:0 dilation, 16:7 first ring frame, 26:17
                     ring frames (4 dilation + 1); reads back
"""

import argparse

from amaranth import *
import amaranth.back.verilog

from .rsnn_front import RsnnFront
from .rsnn_temporal import RsnnTemporal

ID = 0x32465352       # "RSF2"


class RsnnAxi(Elaboratable):
    def __init__(self):
        self.front = RsnnFront()
        self.temporal = RsnnTemporal()
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
                self.s_axi_rvalid, self.s_axi_rready] + self.temporal.axi.ports()

    def elaborate(self, platform):
        m = Module()
        m.submodules.front = front = self.front
        m.submodules.temporal = tmp = self.temporal
        busy = Signal()
        m.d.comb += [busy.eq(front.busy | tmp.busy),
                     tmp.c1.eq(front.c1),
                     # the front's outputs into the temporal layers, which
                     # start when the front is done with a frame
                     tmp.in_we.eq(front.h_we), tmp.in_waddr.eq(front.h_waddr),
                     tmp.in_wdata.eq(front.h_wdata), tmp.go.eq(front.done),
                     self.s_axi_bresp.eq(0), self.s_axi_rresp.eq(0)]
        aw = self.s_axi_awaddr
        ar = self.s_axi_araddr
        rd_sel = Signal(2)             # 0 register, 1 front outputs, 2 temporal outputs
        rd_regval = Signal(32)
        with m.FSM():
            with m.State('IDLE'):
                with m.If(self.s_axi_awvalid & self.s_axi_wvalid):
                    m.d.comb += [self.s_axi_awready.eq(1), self.s_axi_wready.eq(1)]
                    d = self.s_axi_wdata
                    with m.If(aw[8:] == 0xFF):
                        with m.Switch(aw[:8]):
                            with m.Case(0x00):
                                m.d.sync += [front.c2.eq(d[8:13]), front.c1.eq(d[16:23])]
                                m.d.comb += [front.go.eq(d[0] & ~busy),
                                             front.reset.eq(d[1] & ~busy),
                                             tmp.reset.eq(d[1] & ~busy)]
                            with m.Case(0x10):
                                m.d.sync += tmp.n_layers.eq(d[:4])
                            with m.Case(0x14):
                                m.d.sync += tmp.w_base.eq(d)
                            with m.Case(0x18):
                                m.d.sync += tmp.w_words.eq(d[:16])
                            with m.Case('001-----'):
                                m.d.comb += [tmp.l_waddr.eq(aw[2:5]), tmp.l_dil.eq(d[:7]),
                                             tmp.l_base.eq(d[7:17]), tmp.l_len.eq(d[17:27]),
                                             tmp.l_we.eq(1)]
                    with m.Elif(~busy):
                        with m.If(aw < 0x8800):
                            m.d.comb += [front.w_waddr.eq(aw[2:]), front.w_wdata.eq(d), front.w_we.eq(1)]
                        with m.Elif((aw >= 0x9000) & (aw < 0x9400)):
                            m.d.comb += [front.b_waddr.eq(aw[2:10]), front.b_wdata.eq(d), front.b_we.eq(1)]
                        with m.Elif((aw >= 0x9400) & (aw < 0x9480)):
                            m.d.comb += [front.x_waddr.eq(aw[2:7]), front.x_wdata.eq(d), front.x_we.eq(1)]
                        with m.Elif((aw >= 0x9800) & (aw < 0xA000)):
                            m.d.comb += [tmp.b_waddr.eq(aw[2:11]), tmp.b_wdata.eq(d), tmp.b_we.eq(1)]
                    m.next = 'BRESP'
                with m.Elif(self.s_axi_arvalid):
                    m.d.comb += self.s_axi_arready.eq(1)
                    with m.If(ar[8:] == 0xFF):
                        m.d.sync += rd_sel.eq(0)
                    with m.Elif(ar[8:] == 0x96):
                        m.d.sync += rd_sel.eq(2)
                    with m.Else():
                        m.d.sync += rd_sel.eq(1)
                    with m.Switch(ar[:8]):
                        with m.Case(0x04):
                            m.d.sync += rd_regval.eq(Cat(busy, front.valid, tmp.valid))
                        with m.Case(0x08):
                            m.d.sync += rd_regval.eq(ID)
                        with m.Case(0x0C):
                            m.d.sync += rd_regval.eq(tmp.frames)
                        with m.Case(0x10):
                            m.d.sync += rd_regval.eq(tmp.n_layers)
                        with m.Case(0x14):
                            m.d.sync += rd_regval.eq(tmp.w_base)
                        with m.Case(0x18):
                            m.d.sync += rd_regval.eq(tmp.w_words)
                        with m.Case('001-----'):
                            m.d.sync += rd_regval.eq(tmp.l_rdata)
                        with m.Default():
                            m.d.sync += rd_regval.eq(0)
                    m.d.comb += [front.h_raddr.eq(ar[2:8]), tmp.o_raddr.eq(ar[2:8]),
                                 tmp.l_raddr.eq(ar[2:5])]
                    m.next = 'RDATA'
            with m.State('BRESP'):
                m.d.comb += self.s_axi_bvalid.eq(1)
                with m.If(self.s_axi_bready):
                    m.next = 'IDLE'
            with m.State('RDATA'):
                rdata = Signal(32)
                first = Signal(init=1)
                with m.If(first):
                    m.d.sync += [rdata.eq(Mux(rd_sel == 0, rd_regval,
                                              Mux(rd_sel == 2, tmp.o_rdata, front.h_rdata))),
                                 first.eq(0)]
                m.d.comb += [self.s_axi_rvalid.eq(~first), self.s_axi_rdata.eq(rdata)]
                with m.If(~first & self.s_axi_rready):
                    m.d.sync += first.eq(1)
                    m.next = 'IDLE'
        return m


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output_file')
    args = parser.parse_args()
    top = RsnnAxi()
    with open(args.output_file, 'w') as f:
        f.write(amaranth.back.verilog.convert(
            top, name='rsnn_axi', ports=top.ports(), emit_src=False))
