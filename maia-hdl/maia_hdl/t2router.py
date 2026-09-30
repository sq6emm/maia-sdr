#
# SPDX-License-Identifier: MIT
#

"""The DVB-T2 receiver's cells to DDR (tezuka_fw_simple trxd
dvbt2/stream.rs): the equalizer's words (t2eq.py, tapped out of the Maia
core) cross into the CPU clock, and every data cell of an equalized symbol
goes straight to its place in a FEC block of a frame buffer in DDR: the
time and cell deinterleavers as one table the CPU makes once (entry
(symbol j, carrier k): the cell's index in the frame's FEC blocks, r n + q,
or all ones: no data cell). The LDPC decoder's DDR engine (ldpc_dma.py,
cells) then takes each block from there; the A9 no longer touches a data
cell.

Words (trxd dvbt2/fe.rs): a header has bit 0 set (bit 16: the carrier
stream), payload bits 15:1 and 31:17 (symbol j 7:0, frame start F 28:8,
equalized 29); an equalized symbol's 853 words carry two cells each (7-bit
I, Q at bits 7:1, 14:8, 23:17, 30:24). A cell goes out as trxd keeps it:
the 7-bit values doubled (I = w & 0xFE ...), I the low byte.

Table: 32-bit entries, symbol j at table + 8192 j, carrier k at + 4 k (the
AXI3 read master takes a symbol's in 16-beat bursts, two entries a beat,
one beat a word). Frame buffers: fb + b stride, b = 0..3, a new one at each
new F; a buffer is complete when every data symbol (j0 .. nsym - 1) of its
frame went out and every write was answered.

AXI4-Lite (CPU clock, 64 KiB): 0x00 control (bit 0 enable), 0x04 table,
0x08 fb, 0x0C stride, 0x10 nsym, 0x14 j0; 0x20 + 4 b buffer b (bit 31
complete, 20:0 its F); 0x30 frames started, 0x34 symbols written, 0x38
input overflows (words lost), 0x3C id "T2R1".
"""

import argparse

from amaranth import *
from amaranth.lib.fifo import AsyncFIFO, SyncFIFOBuffered
import amaranth.back.verilog

from . import axi

ID = 0x31523254       # "T2R1"
WORDS = 853           # carrier words a symbol
BURST = 16
SKIP = 0xFFFF_FFFF


class T2Router(Elaboratable):
    def __init__(self):
        # the equalizer's output, in the 'eq' clock domain
        self.eq_data = Signal(32)
        self.eq_valid = Signal()
        # the equalizer's clock (the Maia core's): ports eq_clk, eq_rst
        self.eq = ClockDomain('eq')
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
        self.axi = axi.AxiInterface(
            axi.AxiDevice.MANAGER,
            [axi.AxiChannel(axi.AxiDirection.READ, 32, 64, id_bits=6),
             axi.AxiChannel(axi.AxiDirection.WRITE, 32, 64, id_bits=6)],
            axi.AxiVersion.AXI3, name='m_axi')

    def ports(self):
        return [self.eq.clk, self.eq.rst, self.eq_data, self.eq_valid,
                self.s_axi_awaddr, self.s_axi_awvalid, self.s_axi_awready,
                self.s_axi_wdata, self.s_axi_wstrb, self.s_axi_wvalid,
                self.s_axi_wready, self.s_axi_bresp, self.s_axi_bvalid,
                self.s_axi_bready, self.s_axi_araddr, self.s_axi_arvalid,
                self.s_axi_arready, self.s_axi_rdata, self.s_axi_rresp,
                self.s_axi_rvalid, self.s_axi_rready] + self.axi.ports()

    def elaborate(self, platform):
        m = Module()
        m.domains.eq = self.eq
        a = self.axi
        m.d.comb += [a.arid.eq(0), a.arsize.eq(3), a.arlock.eq(0), a.arcache.eq(0b0011),
                     a.arburst.eq(0b01), a.arlen.eq(BURST - 1), a.arprot.eq(0),
                     a.awid.eq(0), a.awsize.eq(3), a.awlock.eq(0), a.awcache.eq(0b0011),
                     a.awburst.eq(0b01), a.awlen.eq(0), a.awprot.eq(0),
                     a.wid.eq(0), a.bready.eq(1)]

        # ---- registers
        enable = Signal()
        table = Signal(32)
        fb = Signal(32)
        stride = Signal(32)
        nsym = Signal(8, init=198)
        j0 = Signal(8, init=8)
        buf_f = Array(Signal(21, name=f'buf_f{i}') for i in range(4))
        buf_ok = Array(Signal(name=f'buf_ok{i}') for i in range(4))
        started = Signal(32)
        symbols = Signal(32)
        overflows = Signal(32)

        # ---- the words into the CPU clock (the eq side never waits)
        m.submodules.cdc = cdc = AsyncFIFO(width=32, depth=1024, r_domain='sync', w_domain='eq')
        m.d.comb += [cdc.w_data.eq(self.eq_data), cdc.w_en.eq(self.eq_valid)]
        ovf_eq = Signal()
        m.d.eq += ovf_eq.eq(self.eq_valid & ~cdc.w_rdy)
        # (a pulse a lost word, counted in the CPU clock through a toggle)
        tog_eq = Signal()
        with m.If(ovf_eq):
            m.d.eq += tog_eq.eq(~tog_eq)
        tog_s = Signal(3)
        m.d.sync += tog_s.eq(Cat(tog_eq, tog_s[:2]))
        with m.If(tog_s[1] ^ tog_s[2]):
            m.d.sync += overflows.eq(overflows + 1)

        # ---- the table: a symbol's entries as 64-bit beats into a FIFO
        m.submodules.tfifo = tfifo = SyncFIFOBuffered(width=64, depth=64)
        r_addr = Signal(32)
        r_left = Signal(10)
        outstanding = Signal(8)
        ar_pending = Signal()
        room = Signal()
        m.d.comb += room.eq(tfifo.level + outstanding + BURST <= 64)
        issue = Signal()
        m.d.comb += [issue.eq(~ar_pending & room & (r_left != 0)),
                     a.araddr.eq(r_addr), a.arvalid.eq(ar_pending),
                     a.rready.eq(tfifo.w_rdy), tfifo.w_data.eq(a.rdata), tfifo.w_en.eq(a.rvalid)]
        got = Signal()
        m.d.comb += got.eq(a.rvalid & a.rready)
        m.d.sync += outstanding.eq(outstanding + Mux(issue, BURST, 0) - got)
        with m.If(issue):
            m.d.sync += ar_pending.eq(1)
        with m.If(ar_pending & a.arready):
            m.d.sync += [ar_pending.eq(0), r_addr.eq(r_addr + BURST * 8),
                         r_left.eq(Mux(r_left > BURST, r_left - BURST, 0))]

        # ---- the cell writes: (address, 16-bit cell) in a FIFO, one a beat
        m.submodules.wfifo = wfifo = SyncFIFOBuffered(width=48, depth=16)
        aw_pend = Signal()
        w_pend = Signal()
        w_addr = Signal(32)
        w_cell = Signal(16)
        in_flight = Signal(6)          # writes not yet answered
        m.d.comb += [a.awaddr.eq(Cat(C(0, 3), w_addr[3:])), a.awvalid.eq(aw_pend),
                     a.wdata.eq(Cat(w_cell, w_cell, w_cell, w_cell)),
                     a.wstrb.eq(C(0b11, 8) << w_addr[:3]), a.wvalid.eq(w_pend), a.wlast.eq(1)]
        take = Signal()
        m.d.comb += take.eq(wfifo.r_rdy & ~aw_pend & ~w_pend & (in_flight < 31))
        m.d.comb += wfifo.r_en.eq(take)
        with m.If(take):
            m.d.sync += [aw_pend.eq(1), w_pend.eq(1), w_addr.eq(wfifo.r_data[:32]),
                         w_cell.eq(wfifo.r_data[32:])]
        with m.If(aw_pend & a.awready):
            m.d.sync += aw_pend.eq(0)
        with m.If(w_pend & a.wready):
            m.d.sync += w_pend.eq(0)
        m.d.sync += in_flight.eq(in_flight + take - a.bvalid)
        writes_idle = Signal()
        m.d.comb += writes_idle.eq(~wfifo.r_rdy & ~aw_pend & ~w_pend & (in_flight == 0))

        # ---- the parser
        cur_f = Signal(21)
        have_f = Signal()
        buf = Signal(2)
        base = Signal(32)
        j = Signal(8)
        widx = Signal(10)
        count = Signal(8)
        w = Signal(32)
        e = Signal(64)
        half = Signal()
        is_last = Signal()
        m.d.comb += is_last.eq(j == nsym - 1)

        hdr = cdc.r_data
        payload = Signal(30)
        m.d.comb += payload.eq(Cat(hdr[1:16], hdr[17:32]))
        with m.FSM():
            # skip to a carrier header
            with m.State('HEADER'):
                m.d.comb += cdc.r_en.eq(cdc.r_rdy)
                with m.If(cdc.r_rdy & hdr[0] & hdr[16] & payload[29] & enable
                          & (payload[:8] >= j0) & (payload[:8] < nsym)):
                    f = payload[8:29]
                    with m.If(~have_f | (f != cur_f)):
                        # a new frame: the next buffer
                        m.d.sync += [cur_f.eq(f), have_f.eq(1), buf.eq(buf + 1), count.eq(0),
                                     buf_ok[(buf + 1)[:2]].eq(0), buf_f[(buf + 1)[:2]].eq(f),
                                     started.eq(started + 1),
                                     base.eq(fb + stride * (buf + 1)[:2])]
                    m.d.sync += [j.eq(payload[:8]), widx.eq(0),
                                 r_addr.eq(table + Cat(C(0, 13), payload[:8])),
                                 r_left.eq(54 * BURST)]
                    m.next = 'WORD'
            # a word and its table beat: two cells
            with m.State('WORD'):
                with m.If(cdc.r_rdy & tfifo.r_rdy):
                    with m.If(cdc.r_data[0]):
                        # a header too early: words were lost; drop the symbol
                        m.next = 'DROP'
                    with m.Else():
                        m.d.comb += [cdc.r_en.eq(1), tfifo.r_en.eq(1)]
                        m.d.sync += [w.eq(cdc.r_data), e.eq(tfifo.r_data), half.eq(0)]
                        m.next = 'CELLS'
            with m.State('CELLS'):
                ent = Signal(32)
                cell = Signal(16)
                m.d.comb += ent.eq(Mux(half, e[32:], e[:32]))
                with m.If(half):
                    m.d.comb += cell.eq(Cat(w[16:24] & 0xFE, (w[24:31] << 1)[:8]))
                with m.Else():
                    m.d.comb += cell.eq(Cat(w[:8] & 0xFE, (w[8:15] << 1)[:8]))
                ready = Signal()
                m.d.comb += ready.eq((ent == SKIP) | wfifo.w_rdy)
                with m.If(ready):
                    with m.If(ent != SKIP):
                        m.d.comb += [wfifo.w_data.eq(Cat((base + Cat(C(0, 1), ent[:31]))[:32], cell)),
                                     wfifo.w_en.eq(1)]
                    m.d.sync += half.eq(1)
                    with m.If(half):
                        m.d.sync += widx.eq(widx + 1)
                        with m.If(widx == WORDS - 1):
                            m.next = 'SYM_END'
                        with m.Else():
                            m.next = 'WORD'
            # the rest of the symbol's table (bursts past 853 beats) goes
            with m.State('SYM_END'):
                m.d.comb += tfifo.r_en.eq(tfifo.r_rdy)
                with m.If((r_left == 0) & ~ar_pending & (outstanding == 0) & ~tfifo.r_rdy):
                    m.d.sync += [count.eq(count + 1), symbols.eq(symbols + 1)]
                    with m.If(is_last):
                        m.next = 'FRAME_END'
                    with m.Else():
                        m.next = 'HEADER'
            with m.State('FRAME_END'):
                with m.If(writes_idle):
                    with m.If(count == nsym - j0):
                        m.d.sync += buf_ok[buf].eq(1)
                    m.next = 'HEADER'
            # words lost mid-symbol: its table read ends, the frame is not complete
            with m.State('DROP'):
                m.d.comb += tfifo.r_en.eq(tfifo.r_rdy)
                with m.If((r_left == 0) & ~ar_pending & (outstanding == 0) & ~tfifo.r_rdy):
                    m.next = 'HEADER'

        # ---- AXI4-Lite
        m.d.comb += [self.s_axi_bresp.eq(0), self.s_axi_rresp.eq(0)]
        rd = Signal(32)
        with m.FSM():
            with m.State('IDLE'):
                with m.If(self.s_axi_awvalid & self.s_axi_wvalid):
                    m.d.comb += [self.s_axi_awready.eq(1), self.s_axi_wready.eq(1)]
                    d = self.s_axi_wdata
                    with m.Switch(self.s_axi_awaddr[:8]):
                        with m.Case(0x00):
                            m.d.sync += enable.eq(d[0])
                        with m.Case(0x04):
                            m.d.sync += table.eq(d)
                        with m.Case(0x08):
                            m.d.sync += fb.eq(d)
                        with m.Case(0x0C):
                            m.d.sync += stride.eq(d)
                        with m.Case(0x10):
                            m.d.sync += nsym.eq(d)
                        with m.Case(0x14):
                            m.d.sync += j0.eq(d)
                    m.next = 'B'
                with m.Elif(self.s_axi_arvalid):
                    m.d.comb += self.s_axi_arready.eq(1)
                    ar = self.s_axi_araddr
                    with m.Switch(ar[:8]):
                        with m.Case(0x00):
                            m.d.sync += rd.eq(enable)
                        with m.Case(0x04):
                            m.d.sync += rd.eq(table)
                        with m.Case(0x08):
                            m.d.sync += rd.eq(fb)
                        with m.Case(0x0C):
                            m.d.sync += rd.eq(stride)
                        with m.Case(0x10):
                            m.d.sync += rd.eq(nsym)
                        with m.Case(0x14):
                            m.d.sync += rd.eq(j0)
                        with m.Case(0x20, 0x24, 0x28, 0x2C):
                            m.d.sync += rd.eq(Cat(buf_f[ar[2:4]], C(0, 10), buf_ok[ar[2:4]]))
                        with m.Case(0x30):
                            m.d.sync += rd.eq(started)
                        with m.Case(0x34):
                            m.d.sync += rd.eq(symbols)
                        with m.Case(0x38):
                            m.d.sync += rd.eq(overflows)
                        with m.Case(0x3C):
                            m.d.sync += rd.eq(ID)
                        with m.Default():
                            m.d.sync += rd.eq(0)
                    m.next = 'R'
            with m.State('B'):
                m.d.comb += self.s_axi_bvalid.eq(1)
                with m.If(self.s_axi_bready):
                    m.next = 'IDLE'
            with m.State('R'):
                m.d.comb += [self.s_axi_rvalid.eq(1), self.s_axi_rdata.eq(rd)]
                with m.If(self.s_axi_rready):
                    m.next = 'IDLE'
        return m


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output_file')
    args = parser.parse_args()
    top = T2Router()
    with open(args.output_file, 'w') as f:
        f.write(amaranth.back.verilog.convert(
            top, name='t2router', ports=top.ports(), emit_src=False))
