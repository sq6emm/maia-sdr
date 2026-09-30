#
# SPDX-License-Identifier: MIT
#

"""DVB-T2 transmit OFDM: the inverse FFT of each symbol in the fabric.

trxd sends each T2 frame as a stream of 16-bit I/Q samples (the DAC DMA's
raw words, through ``datv_raw``):

- a sync word (I = 0x7FFF, Q = -0x7FFF: ``SYNC``), taken out: anything before
  it is dropped (IQ blocks still queued when the transmitter switched to
  T2, or a glitch), so each frame lines up by itself;
- P1: 2048 time-domain samples, passed through as they are;
- then up to ``NSYM`` symbols, each as its 1705 active carriers in FFT bin
  order; a sync word where the next symbol would start ends the frame early
  (frames of any length: the 1.35 MHz profile keeps within T2's 250 ms
  with 152 symbols)
  (the 853 at bins 0..852, then the 852 at bins 1196..2047; the 343 bins
  between are the unused ones, inserted here as zeros).

Each symbol goes through the FFT (maia's pipelined radix-2 core, the one the
T2 receiver uses) with I and Q swapped on the way in and out, which makes it
an inverse FFT; the result, times 8 and saturated to 16 bits, leaves as the
guard interval (the last 256 samples) followed by the 2048 samples. The
output stream goes on to ``datv_tx`` (resampling to the DAC rate). Scale:
out = sum(bins) / 8 (the FFT's truncation divides by 64); trxd scales the
carriers to match.

One symbol at a time: the FFT is restarted, fed the 2048 bins and then zeros
until the symbol's last output has left (2048 + the FFT's delay clocks, some
42 us of a 1.25 ms symbol at 100 MHz). Two buffers of 2048 samples hold what
goes out: one is read while the next item (P1 or a symbol) is written.

With ``enable`` low the input passes straight to the output (the raw IQ
path as before) and the frame position restarts.
"""

import argparse

import numpy as np
from amaranth import *
import amaranth.back.verilog
from amaranth.lib.memory import Memory

from .t2ofdm import new_fft, bitrev, N, ORDER

P1 = 2048
NSYM = 198          # the most symbols a frame (P2 and data) may have
GI = 256
NLOW = 853          # bins 0..852 from the input
NZERO = 343         # bins 853..1195 zero
NHIGH = 852         # bins 1196..2047 from the input
NCAR = NLOW + NHIGH
OUT_SHIFT = 3       # output = FFT output << 3 (x8), saturated
SYNC = 0x8001_7FFF  # the word before each frame (Q in 31:16, I in 15:0)


def sat16(x):
    return max(-32768, min(32767, int(x)))


class Model:
    """Bit-exact model: input samples [(i, q)] -> output samples [(i, q)]
    (whole items only)."""
    def __init__(self, nsym=NSYM):
        self.nsym = nsym
        self.fft = new_fft()

    def run(self, samples):
        out = []
        at = 0
        while True:
            # the sync word; what comes before it is dropped
            while at < len(samples) and ((samples[at][0] & 0xFFFF) | (samples[at][1] & 0xFFFF) << 16) != SYNC:
                at += 1
            at += 1
            # P1
            if at + P1 > len(samples):
                return out
            out += [tuple(s) for s in samples[at:at + P1]]
            at += P1
            for _ in range(self.nsym):
                if at < len(samples) and ((samples[at][0] & 0xFFFF) | (samples[at][1] & 0xFFFF) << 16) == SYNC:
                    break           # a shorter frame: the next one starts
                if at + NCAR > len(samples):
                    return out
                car = samples[at:at + NCAR]
                at += NCAR
                bins = list(car[:NLOW]) + [(0, 0)] * NZERO + list(car[NLOW:])
                # I/Q swapped in (re <- Q, im <- I) and out: an inverse FFT
                re = np.array([b[1] for b in bins])
                im = np.array([b[0] for b in bins])
                ore, oim = self.fft.model(re, im)
                y = [None] * N
                for i in range(N):
                    y[bitrev(i)] = (sat16(int(oim[i]) << OUT_SHIFT),
                                    sat16(int(ore[i]) << OUT_SHIFT))
                out += y[N - GI:] + y


class T2Ifft(Elaboratable):
    def __init__(self, nsym=NSYM):
        self.nsym = nsym
        self.fft = new_fft()
        self.enable = Signal()
        # input: I in 15:0, Q in 31:16
        self.s_tdata = Signal(32)
        self.s_tvalid = Signal()
        self.s_tready = Signal()
        self.m_tdata = Signal(32)
        self.m_tvalid = Signal()
        self.m_tready = Signal()

    def elaborate(self, platform):
        m = Module()
        delay = self.fft.delay
        self.fft_rst = Signal()
        fft = ResetInserter(self.fft_rst)(self.fft)
        m.submodules.fft = fft
        m.submodules.buf = buf = Memory(shape=32, depth=2 * N, init=[])
        wr = buf.write_port()
        rd = buf.read_port()

        # ---- buffers: full flags, which kind (1: symbol, with guard) ----
        full = Signal(2)
        kind = Signal(2)
        wb = Signal()           # buffer the write side fills next
        rb = Signal()           # buffer the read side empties next
        set_full = Signal()
        clr_full = Signal()

        # ---- write side ----
        item = Signal(range(self.nsym + 1))     # 0: P1, 1..nsym: symbols
        wcnt = Signal(range(N + delay + 2))     # P1 samples / FFT clkens
        bin_ = Signal(ORDER + 1)                # bin being fed (0..2047)
        in_sym = Signal()                       # symbol: feeding / flushing
        feeding = Signal()
        ocnt = Signal(range(N + delay + 2))     # clkens whose output is out
        clken = Signal()
        clken_d = Signal()

        # input bins: from the stream, or zero between the two carrier runs
        from_input = Signal()
        m.d.comb += from_input.eq((bin_ < NLOW) | (bin_ >= NLOW + NZERO))

        m.d.comb += [self.s_tready.eq(0), clken.eq(0), wr.en.eq(0),
                     set_full.eq(0), fft.re_in.eq(0), fft.im_in.eq(0)]
        free = ~full.bit_select(wb, 1)
        synced = Signal()
        with m.If(self.enable):
            with m.If((item == 0) & ~synced):
                # waiting for the sync word; anything else is dropped
                m.d.comb += self.s_tready.eq(1)
                with m.If(self.s_tvalid & (self.s_tdata == SYNC)):
                    m.d.sync += synced.eq(1)
            with m.Elif(item == 0):
                # P1: straight into the buffer, natural order
                m.d.comb += self.s_tready.eq(free)
                with m.If(free & self.s_tvalid):
                    m.d.comb += [wr.addr.eq(Cat(wcnt[:ORDER], wb)),
                                 wr.data.eq(self.s_tdata), wr.en.eq(1)]
                    m.d.sync += wcnt.eq(wcnt + 1)
                    with m.If(wcnt == P1 - 1):
                        m.d.comb += set_full.eq(1)
                        m.d.sync += [wcnt.eq(0), item.eq(1), synced.eq(0)]
            with m.Else():
                with m.If(~in_sym):
                    with m.If(self.s_tvalid & (self.s_tdata == SYNC)):
                        # a sync word where a symbol would start: the frame
                        # was shorter than nsym; the next one's P1 follows
                        m.d.comb += self.s_tready.eq(1)
                        m.d.sync += [item.eq(0), synced.eq(1)]
                    with m.Elif(free):
                        # a free buffer: start the symbol (FFT restarted)
                        m.d.sync += [in_sym.eq(1), feeding.eq(1), bin_.eq(0),
                                     wcnt.eq(0), ocnt.eq(0)]
                with m.Elif(feeding):
                    with m.If(from_input):
                        m.d.comb += [self.s_tready.eq(1),
                                     fft.re_in.eq(self.s_tdata[16:32]),
                                     fft.im_in.eq(self.s_tdata[0:16]),
                                     clken.eq(self.s_tvalid)]
                    with m.Else():
                        m.d.comb += clken.eq(1)
                    with m.If(clken):
                        m.d.sync += [bin_.eq(bin_ + 1), wcnt.eq(wcnt + 1)]
                        with m.If(bin_ == N - 1):
                            m.d.sync += feeding.eq(0)
                with m.Else():
                    # flushing: zeros until the last output has left
                    m.d.comb += clken.eq(wcnt < N + delay - 1)
                    with m.If(clken):
                        m.d.sync += wcnt.eq(wcnt + 1)
        m.d.comb += [fft.clken.eq(clken),
                     self.fft_rst.eq(~self.enable | (item == 0) | ~in_sym)]
        m.d.sync += clken_d.eq(clken)

        # FFT outputs: the one of clken c is there after it, for c >= delay-1
        oidx = Signal(ORDER + 1)
        m.d.comb += oidx.eq(ocnt - (delay - 1))
        o_re = Signal(signed(16))
        o_im = Signal(signed(16))

        def sat(x):
            return Mux(x > 32767, 32767, Mux(x < -32768, -32768, x))
        m.d.comb += [o_re.eq(sat(fft.re_out << OUT_SHIFT)),
                     o_im.eq(sat(fft.im_out << OUT_SHIFT))]
        with m.If(clken_d & in_sym):
            m.d.sync += ocnt.eq(ocnt + 1)
            with m.If(ocnt >= delay - 1):
                # I/Q swapped back: I <- im, Q <- re; natural order
                rev = Cat(*[oidx[ORDER - 1 - k] for k in range(ORDER)])
                m.d.comb += [wr.addr.eq(Cat(rev, wb)),
                             wr.data.eq(Cat(o_im, o_re)), wr.en.eq(1)]
                with m.If(oidx == N - 1):
                    m.d.comb += set_full.eq(1)
                    m.d.sync += [in_sym.eq(0), wcnt.eq(0)]
                    with m.If(item == self.nsym):
                        m.d.sync += item.eq(0)
                    with m.Else():
                        m.d.sync += item.eq(item + 1)

        # (for tests)
        self._item, self._in_sym, self._feeding, self._full = item, in_sym, feeding, full
        self._wb, self._rb, self._wcnt, self._ocnt, self._bin = wb, rb, wcnt, ocnt, bin_
        with m.If(set_full):
            m.d.sync += [wb.eq(~wb),
                         kind.bit_select(wb, 1).eq(item != 0)]

        # ---- read side: a registered output stage ----
        rcnt = Signal(range(N + GI + 1))
        rlen = Signal(range(N + GI + 1))
        m.d.comb += rlen.eq(Mux(kind.bit_select(rb, 1), N + GI, P1))
        # sample index in the buffer: guard first for a symbol
        ridx = Signal(ORDER + 1)
        m.d.comb += ridx.eq(Mux(kind.bit_select(rb, 1),
                                Mux(rcnt < GI, rcnt + (N - GI), rcnt - GI),
                                rcnt))
        out_valid = Signal()
        out_data = Signal(32)
        take = Signal()
        # the read port answers a cycle later: issue when the output
        # register will be free
        pend = Signal()
        m.d.comb += [rd.addr.eq(Cat(ridx[:ORDER], rb)), rd.en.eq(take),
                     clr_full.eq(0), take.eq(0)]
        room = ~out_valid | self.m_tready
        with m.If(self.enable):
            with m.If(pend & room):
                m.d.sync += [out_valid.eq(1), out_data.eq(rd.data), pend.eq(0)]
            with m.Elif(room):
                m.d.sync += out_valid.eq(0)
            can = full.bit_select(rb, 1) & (~pend | room)
            with m.If(can & room):
                m.d.comb += take.eq(1)
                m.d.sync += pend.eq(1)
                with m.If(rcnt == rlen - 1):
                    m.d.sync += [rcnt.eq(0), rb.eq(~rb)]
                    m.d.comb += clr_full.eq(1)
                with m.Else():
                    m.d.sync += rcnt.eq(rcnt + 1)
        with m.Else():
            m.d.sync += [out_valid.eq(0), pend.eq(0), rcnt.eq(0)]

        # full flags (set by the write side, cleared by the read side)
        nxt = Signal(2)
        m.d.comb += nxt.eq(full)
        with m.If(set_full):
            m.d.comb += nxt.bit_select(wb, 1).eq(1)
        with m.If(clr_full):
            m.d.comb += nxt.bit_select(rb, 1).eq(0)
        m.d.sync += full.eq(nxt)

        with m.If(~self.enable):
            m.d.sync += [full.eq(0), wb.eq(0), rb.eq(0), item.eq(0),
                         wcnt.eq(0), in_sym.eq(0), feeding.eq(0),
                         synced.eq(0)]

        # ---- output: this block's stream, or the input straight through ----
        with m.If(self.enable):
            m.d.comb += [self.m_tdata.eq(out_data),
                         self.m_tvalid.eq(out_valid)]
        with m.Else():
            m.d.comb += [self.m_tdata.eq(self.s_tdata),
                         self.m_tvalid.eq(self.s_tvalid),
                         self.s_tready.eq(self.m_tready)]
        return m


class T2IfftTop(Elaboratable):
    """The block with AXI-stream port names, for the Vivado block design."""
    def __init__(self):
        self.ifft = T2Ifft()
        self.enable = Signal()
        self.s_axis_tdata = Signal(32)
        self.s_axis_tvalid = Signal()
        self.s_axis_tready = Signal()
        self.m_axis_tdata = Signal(32)
        self.m_axis_tvalid = Signal()
        self.m_axis_tready = Signal()

    def ports(self):
        return [self.enable, self.s_axis_tdata, self.s_axis_tvalid,
                self.s_axis_tready, self.m_axis_tdata, self.m_axis_tvalid,
                self.m_axis_tready]

    def elaborate(self, platform):
        m = Module()
        m.submodules.ifft = f = self.ifft
        m.d.comb += [f.enable.eq(self.enable),
                     f.s_tdata.eq(self.s_axis_tdata),
                     f.s_tvalid.eq(self.s_axis_tvalid),
                     self.s_axis_tready.eq(f.s_tready),
                     self.m_axis_tdata.eq(f.m_tdata),
                     self.m_axis_tvalid.eq(f.m_tvalid),
                     f.m_tready.eq(self.m_axis_tready)]
        return m


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output_file')
    args = parser.parse_args()
    top = T2IfftTop()
    with open(args.output_file, 'w') as f:
        f.write(amaranth.back.verilog.convert(
            top, name='t2ifft', ports=top.ports(), emit_src=False))
