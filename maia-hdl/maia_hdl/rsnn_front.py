#
# SPDX-License-Identifier: MIT
#

"""The CW-RS keying detector's front end in the FPGA (tezuka_fw_simple
trxd src/rsnn.rs `Net::front_q`, bit for bit): per feature row (61 bins,
16 bits, x 128) the three 2-D convolutions over (time, frequency), the
attention and max pooling over frequency and the 1x1 into the temporal
layers; the ARM keeps the features and the temporal layers.

Streaming: row n in (frame n), the front's output for frame n - 3 out (a
layer's time kernel is 3 frames: each adds a frame of delay). The rings
are zeroed at reset and never written for frames before 0: the batch's
zero padding.

Fixed point (as rsnn.rs): activations i16 (x AQ = 128), weights i16
(x WQ = 2048), sums in 32 bits for the convolutions (wrapping, as i32) and
48 for the rest; a convolution's output min(max(sum, 0) >> 11, 32767);
attention logits x AQ WQ, softmax weights exp(-d) from a table (Q15, d in
steps of 1/128, 2048 entries), their sum's reciprocal r = 2^31 / sum; the
pooled value ((sum a e) >> 16) r >> 15; the 1x1 max(sum, 0) >> 11.

Parameters (the ARM loads them, rsnn.rs `Net::fpga_image`): weights c1
(c2 x 1 x 3 x 5), c2 (c2 x c2 x 3 x 5), c3 (c2 x c2 x 3 x 3), attention
(c2), 1x1 (c1 x 2 c2); biases c1, c2, c3 (c2 each), attention (1), 1x1
(c1). c2 <= 24, c1 <= 64 (registers).
"""

import argparse

from amaranth import *
from amaranth.lib.memory import Memory
import amaranth.back.verilog

import numpy as np

NB = 61
NB2 = 31
C2MAX = 24
C1MAX = 64
WWORDS = 8704         # 17408 weights, two a word
BDEPTH = 256
EXP_N = 2048
EXP_SHIFT = 11
ID = 0x31465352       # "RSF1"


def exp_table():
    return [int(round(32767 * np.exp(-i / 128.0))) for i in range(EXP_N)]


class SplitMem(Elaboratable):
    """A synchronous-read RAM of power-of-two pieces (Vivado maps a
    non-power-of-two depth to the next power of two of block RAM): one
    write port, one read port, the read data a cycle later."""
    def __init__(self, width, pieces):
        self.pieces = pieces
        self.depth = sum(pieces)
        abits = (self.depth - 1).bit_length()
        self.waddr = Signal(abits)
        self.wdata = Signal(width)
        self.we = Signal()
        self.raddr = Signal(abits)
        self.rdata = Signal(width)
        self.width = width

    def elaborate(self, platform):
        m = Module()
        base = 0
        sel = Signal(range(len(self.pieces)))
        sel_q = Signal.like(sel)
        m.d.sync += sel_q.eq(sel)
        datas = []
        for n, d in enumerate(self.pieces):
            mem = Memory(shape=self.width, depth=d, init=[])
            m.submodules[f'm{n}'] = mem
            rd = mem.read_port()
            wr = mem.write_port()
            lo, hi = base, base + d
            m.d.comb += [wr.addr.eq(self.waddr - lo), wr.data.eq(self.wdata),
                         wr.en.eq(self.we & (self.waddr >= lo) & (self.waddr < hi)),
                         rd.addr.eq(self.raddr - lo)]
            with m.If((self.raddr >= lo) & (self.raddr < hi)):
                m.d.comb += sel.eq(n)
            datas.append(rd.data)
            base = hi
        with m.Switch(sel_q):
            for n, dt in enumerate(datas):
                with m.Case(n):
                    m.d.comb += self.rdata.eq(dt)
        return m


def a1_addr(slot, c, f):
    """a1: 4 frame slots x 24 channels x 64 bins (6144 words, BRAM)."""
    return Cat(f[:6], c[:5]) + (slot * 1536)[:13]


def a2_addr(slot, c, f):
    """a2: 4 frame slots x 24 channels x 32 bins (3072 words, BRAM)."""
    return Cat(f[:5], c[:5]) + (slot * 768)[:12]


class RsnnFront(Elaboratable):
    """The engine. Ports (one clock domain):

    c2, c1 (in): the channels; go (in, pulse): the next row is in; reset
    (in, pulse): zero the rings (6144 cycles); busy, valid (out: the
    outputs are those of frame n - 3, n the last row's).
    w_waddr, w_wdata, w_we: two weights a word (low 16 bits: the even one);
    b_waddr, b_wdata, b_we; x_waddr, x_wdata, x_we: the next row, two bins
    a word (low: the even); h_raddr, h_rdata (a cycle later).
    """
    def __init__(self):
        self.c2 = Signal(5, init=C2MAX)
        self.c1 = Signal(7, init=C1MAX)
        self.go = Signal()
        self.reset = Signal()
        self.busy = Signal()
        self.valid = Signal()
        self.w_waddr = Signal(14)
        self.w_wdata = Signal(32)
        self.w_we = Signal()
        self.b_waddr = Signal(8)
        self.b_wdata = Signal(32)
        self.b_we = Signal()
        self.x_waddr = Signal(5)
        self.x_wdata = Signal(32)
        self.x_we = Signal()
        self.h_raddr = Signal(6)
        self.h_rdata = Signal(32)

    def elaborate(self, platform):
        m = Module()
        # BRAM (synchronous reads): weights, a1, a2, the exp table
        m.submodules.wmem = wmem = SplitMem(32, [8192, WWORDS - 8192])
        m.submodules.a1 = a1 = SplitMem(16, [4096, 2048])
        m.submodules.a2 = a2 = SplitMem(16, [2048, 1024])
        m.submodules.expm = expm = Memory(shape=16, depth=EXP_N, init=exp_table())
        e_rd = expm.read_port()
        # distributed RAM (small: read asynchronously, then registered)
        m.submodules.hm = hm = Memory(shape=32, depth=64, init=[])
        h_rd = hm.read_port(domain='comb')
        h_wr = hm.write_port()
        m.submodules.bmem = bmem = Memory(shape=32, depth=BDEPTH, init=[])
        b_rd = bmem.read_port(domain='comb')
        b_wr = bmem.write_port()
        b_q = Signal(32)
        m.d.sync += b_q.eq(b_rd.data)
        h_q = Signal(32)
        m.d.sync += h_q.eq(h_rd.data)
        m.submodules.xmem = xmem = Memory(shape=32, depth=4 * 32, init=[])
        x_rd = xmem.read_port(domain='comb')
        x_wr = xmem.write_port()
        m.submodules.a3 = a3 = Memory(shape=16, depth=1024, init=[])
        a3_rd = a3.read_port(domain='comb')
        a3_wr = a3.write_port()
        m.submodules.zm = zm = Memory(shape=18, depth=64, init=[])
        z_rd = zm.read_port(domain='comb')
        z_wr = zm.write_port()
        m.submodules.lgm = lgm = Memory(shape=32, depth=32, init=[])
        lg_rd = lgm.read_port(domain='comb')
        lg_wr = lgm.write_port()
        m.submodules.em = em = Memory(shape=16, depth=32, init=[])
        ev_rd = em.read_port(domain='comb')
        ev_wr = em.write_port()

        c2 = self.c2
        c1 = self.c1
        # n2: the newest row's slot; cnt: rows since reset (to 4)
        n2 = Signal(2)
        cnt = Signal(3)

        def slot(back):
            return (n2 - back)[:2]

        m.d.comb += [wmem.waddr.eq(self.w_waddr), wmem.wdata.eq(self.w_wdata), wmem.we.eq(self.w_we),
                     b_wr.addr.eq(self.b_waddr), b_wr.data.eq(self.b_wdata), b_wr.en.eq(self.b_we),
                     x_wr.addr.eq(Cat(self.x_waddr, (n2 + 1)[:2])), x_wr.data.eq(self.x_wdata),
                     x_wr.en.eq(self.x_we),
                     h_rd.addr.eq(self.h_raddr), self.h_rdata.eq(h_q)]

        # parameter offsets (fpga_image's order)
        w2 = Signal(15)
        w3 = Signal(15)
        wa = Signal(15)
        wi = Signal(15)
        c2sq = Signal(10)
        m.d.sync += [c2sq.eq(c2 * c2),
                     w2.eq(c2 * 15), w3.eq(c2 * 15 + c2sq * 15),
                     wa.eq(c2 * 15 + c2sq * 24), wi.eq(c2 * 16 + c2sq * 24)]
        b2 = Signal(8)
        b3 = Signal(8)
        ba = Signal(8)
        bi = Signal(8)
        m.d.sync += [b2.eq(c2), b3.eq(2 * c2), ba.eq(3 * c2), bi.eq(3 * c2 + 1)]

        # ---- the MAC pipeline
        # stage 0 (the sequencer's op): addresses, an asynchronously read
        # activation or the source to take it from in stage 1, flags
        op = Signal()
        op_w = Signal(15)
        op_b = Signal(8)
        op_a = Signal(signed(18))
        op_src = Signal(2)        # 0 op_a, 1 a1, 2 a2
        op_pv = Signal()          # inside the row (else zero padding)
        op_e = Signal()           # the second operand: op_ev, not a weight
        op_ev = Signal(16)
        op_first = Signal()
        op_last = Signal()
        op_mode = Signal(2)       # 0 conv, 1 logit, 2 pooled, 3 1x1
        op_dst = Signal(13)
        op_layer = Signal(2)
        m.d.comb += [wmem.raddr.eq(op_w[1:]), b_rd.addr.eq(op_b)]
        # stage 1: weight, bias, a1/a2 read out
        s1 = Signal()
        s1_half = Signal()
        s1_a = Signal(signed(18))
        s1_src = Signal(2)
        s1_pv = Signal()
        s1_e = Signal()
        s1_ev = Signal(16)
        s1_first = Signal()
        s1_last = Signal()
        s1_mode = Signal(2)
        s1_dst = Signal(13)
        s1_layer = Signal(2)
        m.d.sync += [s1.eq(op), s1_half.eq(op_w[0]), s1_a.eq(op_a), s1_src.eq(op_src),
                     s1_pv.eq(op_pv), s1_e.eq(op_e), s1_ev.eq(op_ev),
                     s1_first.eq(op_first), s1_last.eq(op_last), s1_mode.eq(op_mode),
                     s1_dst.eq(op_dst), s1_layer.eq(op_layer)]
        wv = Signal(signed(16))
        m.d.comb += wv.eq(Mux(s1_half, wmem.rdata[16:], wmem.rdata[:16]))
        opnd = Signal(signed(17))
        m.d.comb += opnd.eq(Mux(s1_e, Cat(s1_ev, C(0, 1)), wv))
        av = Signal(signed(18))
        with m.Switch(s1_src):
            with m.Case(1):
                m.d.comb += av.eq(a1.rdata)
            with m.Case(2):
                m.d.comb += av.eq(a2.rdata)
            with m.Default():
                m.d.comb += av.eq(s1_a)
        avm = Signal(signed(18))
        m.d.comb += avm.eq(Mux(s1_pv, av, 0))
        # stage 2: the product
        s2 = Signal()
        s2_p = Signal(signed(36))
        s2_a = Signal(signed(18))
        s2_bias = Signal(signed(32))
        s2_first = Signal()
        s2_last = Signal()
        s2_mode = Signal(2)
        s2_dst = Signal(13)
        s2_layer = Signal(2)
        m.d.sync += [s2.eq(s1), s2_p.eq(avm * opnd), s2_a.eq(avm), s2_bias.eq(b_q),
                     s2_first.eq(s1_first), s2_last.eq(s1_last), s2_mode.eq(s1_mode),
                     s2_dst.eq(s1_dst), s2_layer.eq(s1_layer)]
        # stage 3: the sum (and the running maximum, for pooling)
        acc = Signal(signed(48))
        amax = Signal(signed(18))
        nacc = Signal(signed(48))
        m.d.comb += nacc.eq(Mux(s2_first, Mux(s2_mode == 2, 0, s2_bias), acc) + s2_p)
        nmax = Signal(signed(18))
        m.d.comb += nmax.eq(Mux(s2_first | (s2_a > amax), s2_a, amax))
        with m.If(s2):
            m.d.sync += [acc.eq(nacc), amax.eq(nmax)]
        s3 = Signal()
        s3_v = Signal(signed(48))
        s3_max = Signal(signed(18))
        s3_mode = Signal(2)
        s3_dst = Signal(13)
        s3_layer = Signal(2)
        m.d.sync += [s3.eq(s2 & s2_last), s3_v.eq(nacc), s3_max.eq(nmax), s3_mode.eq(s2_mode),
                     s3_dst.eq(s2_dst), s3_layer.eq(s2_layer)]
        # stage 4: a result out
        v32 = Signal(signed(32))
        m.d.comb += v32.eq(s3_v[:32])
        relu = Signal(31)
        m.d.comb += relu.eq(Mux(v32[31], 0, v32[:31]))
        conv_out = Signal(16)
        m.d.comb += conv_out.eq(Mux(relu[11:] > 32767, 32767, relu[11:]))
        lgmax = Signal(signed(32))
        r = Signal(17)
        zq = Signal(signed(48))
        m.d.comb += zq.eq((s3_v[16:40].as_signed() * Cat(r, C(0, 1)).as_signed()) >> 15)
        hout = Signal(32)
        m.d.comb += hout.eq(Mux(s3_v[47], 0, s3_v[11:43]))
        zmax_pending = Signal()
        zmax_addr = Signal(6)
        zmax_val = Signal(18)
        m.d.sync += zmax_pending.eq(0)
        with m.If(s3):
            with m.Switch(s3_mode):
                with m.Case(0):
                    with m.Switch(s3_layer):
                        with m.Case(1):
                            m.d.comb += [a1.waddr.eq(s3_dst), a1.wdata.eq(conv_out), a1.we.eq(1)]
                        with m.Case(2):
                            m.d.comb += [a2.waddr.eq(s3_dst[:12]), a2.wdata.eq(conv_out), a2.we.eq(1)]
                        with m.Default():
                            m.d.comb += [a3_wr.addr.eq(s3_dst[:10]), a3_wr.data.eq(conv_out), a3_wr.en.eq(1)]
                with m.Case(1):
                    m.d.comb += [lg_wr.addr.eq(s3_dst[:5]), lg_wr.data.eq(v32), lg_wr.en.eq(1)]
                    with m.If((s3_dst[:5] == 0) | (v32 > lgmax)):
                        m.d.sync += lgmax.eq(v32)
                with m.Case(2):
                    m.d.comb += [z_wr.addr.eq(s3_dst[:6]), z_wr.data.eq(zq[:18]), z_wr.en.eq(1)]
                    m.d.sync += [zmax_pending.eq(1), zmax_addr.eq(s3_dst[:6] + c2), zmax_val.eq(s3_max)]
                with m.Case(3):
                    m.d.comb += [h_wr.addr.eq(s3_dst[:6]), h_wr.data.eq(hout), h_wr.en.eq(1)]
        with m.If(zmax_pending):
            m.d.comb += [z_wr.addr.eq(zmax_addr), z_wr.data.eq(zmax_val), z_wr.en.eq(1)]
        drained = Signal()
        m.d.comb += drained.eq(~op & ~s1 & ~s2 & ~s3 & ~zmax_pending)

        # ---- the sequencer
        o = Signal(7)
        f = Signal(6)
        i = Signal(6)
        kt = Signal(2)
        kf = Signal(3)
        wptr = Signal(15)
        wbase = Signal(15)
        layer = Signal(2)
        clr = Signal(13)
        den = Signal(21)
        div_q = Signal(32)
        div_r = Signal(33)
        div_n = Signal(5)
        valid = Signal()
        m.d.comb += self.valid.eq(valid)

        def conv_start(lay, w):
            return [layer.eq(lay), o.eq(0), f.eq(0), i.eq(0), kt.eq(0), kf.eq(0),
                    wptr.eq(w), wbase.eq(w)]

        cin = Signal(6)
        kf_max = Signal(3)
        fout = Signal(6)
        last = Signal()

        with m.FSM(name='seq'):
            with m.State('IDLE'):
                with m.If(self.reset):
                    m.d.sync += [clr.eq(0), cnt.eq(0), n2.eq(0), valid.eq(0)]
                    m.next = 'CLEAR'
                with m.Elif(self.go):
                    # the row just written (slot n2 + 1) is the newest
                    m.d.sync += [n2.eq(n2 + 1), valid.eq(0)]
                    with m.If(cnt != 4):
                        m.d.sync += cnt.eq(cnt + 1)
                    with m.If(cnt >= 1):
                        m.d.sync += conv_start(1, 0)
                        m.next = 'CONV'
            with m.State('CLEAR'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += [a1.waddr.eq(clr), a1.wdata.eq(0), a1.we.eq(1),
                             a2.waddr.eq(clr[:12]), a2.wdata.eq(0), a2.we.eq(clr < 3072),
                             x_wr.addr.eq(clr[:7]), x_wr.data.eq(0), x_wr.en.eq(clr < 128)]
                m.d.sync += clr.eq(clr + 1)
                with m.If(clr == 6143):
                    m.next = 'IDLE'
            with m.State('CONV'):
                m.d.comb += self.busy.eq(1)
                with m.Switch(layer):
                    with m.Case(1):
                        # a1[n-1] from x[n-2 .. n]: 5 bins, stride 1
                        q = Signal(7)
                        m.d.comb += [q.eq(f + kf), cin.eq(1), kf_max.eq(4), fout.eq(NB)]
                        pp = (q - 2)[:6]
                        m.d.comb += x_rd.addr.eq(Cat(pp[1:], (n2 + 2 + kt)[:2]))
                        m.d.comb += [op_a.eq(Mux(pp[0], x_rd.data[16:32], x_rd.data[:16]).as_signed()),
                                     op_src.eq(0), op_pv.eq((q >= 2) & (q < NB + 2)),
                                     op_dst.eq(a1_addr(slot(1), o, f)), op_b.eq(o)]
                    with m.Case(2):
                        # a2[n-2] from a1[n-3 .. n-1]: 5 bins, stride 2
                        q = Signal(7)
                        m.d.comb += [q.eq(Cat(C(0, 1), f) + kf), cin.eq(c2), kf_max.eq(4),
                                     fout.eq(NB2)]
                        m.d.comb += a1.raddr.eq(a1_addr((n2 + 1 + kt)[:2], i, (q - 2)[:6]))
                        m.d.comb += [op_src.eq(1), op_pv.eq((q >= 2) & (q < NB + 2)),
                                     op_dst.eq(a2_addr(slot(2), o, f)), op_b.eq(b2 + o)]
                    with m.Default():
                        # a3[n-3] from a2[n-4 .. n-2]: 3 bins, stride 1
                        q = Signal(7)
                        m.d.comb += [q.eq(f + kf), cin.eq(c2), kf_max.eq(2), fout.eq(NB2)]
                        m.d.comb += a2.raddr.eq(a2_addr((n2 + kt)[:2], i, (q - 1)[:5]))
                        m.d.comb += [op_src.eq(2), op_pv.eq((q >= 1) & (q < NB2 + 1)),
                                     op_dst.eq(Cat(f[:5], o[:5])), op_b.eq(b3 + o)]
                m.d.comb += last.eq((i == cin - 1) & (kt == 2) & (kf == kf_max))
                m.d.comb += [op.eq(1), op_w.eq(wptr), op_first.eq((i == 0) & (kt == 0) & (kf == 0)),
                             op_last.eq(last), op_mode.eq(0), op_layer.eq(layer)]
                m.d.sync += wptr.eq(wptr + 1)
                with m.If(kf != kf_max):
                    m.d.sync += kf.eq(kf + 1)
                with m.Else():
                    m.d.sync += kf.eq(0)
                    with m.If(kt != 2):
                        m.d.sync += kt.eq(kt + 1)
                    with m.Else():
                        m.d.sync += kt.eq(0)
                        with m.If(i != cin - 1):
                            m.d.sync += i.eq(i + 1)
                        with m.Else():
                            m.d.sync += i.eq(0)
                            with m.If(f != fout - 1):
                                m.d.sync += [f.eq(f + 1), wptr.eq(wbase)]
                            with m.Else():
                                m.d.sync += [f.eq(0), o.eq(o + 1), wbase.eq(wptr + 1)]
                                with m.If(o == c2 - 1):
                                    m.next = 'CONV_DRAIN'
            with m.State('CONV_DRAIN'):
                m.d.comb += self.busy.eq(1)
                with m.If(drained):
                    with m.If((layer == 1) & (cnt >= 3)):
                        m.d.sync += conv_start(2, w2)
                        m.next = 'CONV'
                    with m.Elif((layer == 2) & (cnt == 4)):
                        m.d.sync += conv_start(3, w3)
                        m.next = 'CONV'
                    with m.Elif(layer == 3):
                        m.d.sync += [f.eq(0), o.eq(0)]
                        m.next = 'LG'
                    with m.Else():
                        m.next = 'IDLE'
            # attention logits: for each bin, the sum over channels
            with m.State('LG'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += a3_rd.addr.eq(Cat(f[:5], o[:5]))
                m.d.comb += [op.eq(1), op_w.eq(wa + o), op_b.eq(ba), op_a.eq(a3_rd.data),
                             op_pv.eq(1), op_first.eq(o == 0), op_last.eq(o == c2 - 1),
                             op_mode.eq(1), op_dst.eq(f)]
                with m.If(o != c2 - 1):
                    m.d.sync += o.eq(o + 1)
                with m.Else():
                    m.d.sync += [o.eq(0), f.eq(f + 1)]
                    with m.If(f == NB2 - 1):
                        m.d.sync += [f.eq(0), den.eq(0)]
                        m.next = 'LG_DRAIN'
            with m.State('LG_DRAIN'):
                m.d.comb += self.busy.eq(1)
                with m.If(drained):
                    m.next = 'EXP'
            # softmax weights: e = EXP[min((max - lg) >> 11, 2047)]
            with m.State('EXP'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += lg_rd.addr.eq(f[:5])
                dd = Signal(32)
                m.d.comb += dd.eq(lgmax - lg_rd.data.as_signed())
                m.d.comb += e_rd.addr.eq(Mux(dd[EXP_SHIFT:] > EXP_N - 1, EXP_N - 1,
                                             dd[EXP_SHIFT:]))
                m.next = 'EXP_W'
            with m.State('EXP_W'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += [ev_wr.addr.eq(f[:5]), ev_wr.data.eq(e_rd.data), ev_wr.en.eq(1)]
                m.d.sync += [den.eq(den + e_rd.data), f.eq(f + 1)]
                m.next = 'EXP'
                with m.If(f == NB2 - 1):
                    m.d.sync += [f.eq(0), div_q.eq(0), div_r.eq(0), div_n.eq(0)]
                    m.next = 'DIV'
            # r = 2^31 / den (restoring division, a bit a cycle)
            with m.State('DIV'):
                m.d.comb += self.busy.eq(1)
                rr = Signal(33)
                m.d.comb += rr.eq(Cat(div_n == 0, div_r[:32]))
                with m.If(rr >= den):
                    m.d.sync += [div_r.eq(rr - den), div_q.eq(Cat(C(1, 1), div_q[:31]))]
                with m.Else():
                    m.d.sync += [div_r.eq(rr), div_q.eq(Cat(C(0, 1), div_q[:31]))]
                m.d.sync += div_n.eq(div_n + 1)
                with m.If(div_n == 31):
                    m.next = 'DIV_DONE'
            with m.State('DIV_DONE'):
                m.d.comb += self.busy.eq(1)
                m.d.sync += [r.eq(div_q[:17]), o.eq(0), f.eq(0)]
                m.next = 'NUM'
            # pooled: for each channel, the sum over bins of a3 e (and the max)
            with m.State('NUM'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += [a3_rd.addr.eq(Cat(f[:5], o[:5])), ev_rd.addr.eq(f[:5])]
                m.d.comb += [op.eq(1), op_e.eq(1), op_ev.eq(ev_rd.data), op_a.eq(a3_rd.data),
                             op_pv.eq(1), op_first.eq(f == 0), op_last.eq(f == NB2 - 1),
                             op_mode.eq(2), op_dst.eq(o)]
                with m.If(f != NB2 - 1):
                    m.d.sync += f.eq(f + 1)
                with m.Else():
                    m.d.sync += [f.eq(0), o.eq(o + 1)]
                    with m.If(o == c2 - 1):
                        m.d.sync += [o.eq(0), i.eq(0), wptr.eq(wi)]
                        m.next = 'NUM_DRAIN'
            with m.State('NUM_DRAIN'):
                m.d.comb += self.busy.eq(1)
                with m.If(drained):
                    m.next = 'INP'
            # the 1x1: for each output, the sum over the 2 c2 pooled values
            with m.State('INP'):
                m.d.comb += self.busy.eq(1)
                m.d.comb += z_rd.addr.eq(i)
                m.d.comb += [op.eq(1), op_w.eq(wptr), op_b.eq(bi + o), op_a.eq(z_rd.data),
                             op_pv.eq(1), op_first.eq(i == 0), op_last.eq(i == 2 * c2 - 1),
                             op_mode.eq(3), op_dst.eq(o)]
                m.d.sync += wptr.eq(wptr + 1)
                with m.If(i != 2 * c2 - 1):
                    m.d.sync += i.eq(i + 1)
                with m.Else():
                    m.d.sync += [i.eq(0), o.eq(o + 1)]
                    with m.If(o == c1 - 1):
                        m.next = 'INP_DRAIN'
            with m.State('INP_DRAIN'):
                m.d.comb += self.busy.eq(1)
                with m.If(drained):
                    m.d.sync += valid.eq(1)
                    m.next = 'IDLE'
        return m


class RsnnFrontAxi(Elaboratable):
    """RsnnFront behind an AXI4-Lite slave (64 KiB window, CPU clock):

    0x0000-0x87FF  weights, two a word (low 16 bits: the even one)
    0x9000-0x93FF  biases (32 bits)
    0x9400-0x947F  the next feature row, two bins a word (low: the even)
    0x9500-0x95FF  the outputs (32 bits, 64), read while not busy
    0xFF00  control  W: bit 0 go (the row is in), bit 1 reset (zero the
                     rings); bits 12:8 c2, 22:16 c1 (kept)
    0xFF04  status   R: bit 0 busy, bit 1 valid (the outputs are frame n-3)
    0xFF08  id       "RSF1"
    """
    def __init__(self):
        self.core = RsnnFront()
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
        m.submodules.core = core = self.core
        m.d.comb += [self.s_axi_bresp.eq(0), self.s_axi_rresp.eq(0)]
        aw = self.s_axi_awaddr
        ar = self.s_axi_araddr
        rd_reg = Signal()
        rd_regval = Signal(32)
        with m.FSM():
            with m.State('IDLE'):
                with m.If(self.s_axi_awvalid & self.s_axi_wvalid):
                    m.d.comb += [self.s_axi_awready.eq(1), self.s_axi_wready.eq(1)]
                    d = self.s_axi_wdata
                    with m.If(aw[8:] == 0xFF):
                        with m.If(aw[:8] == 0x00):
                            m.d.sync += [core.c2.eq(d[8:13]), core.c1.eq(d[16:23])]
                            m.d.comb += [core.go.eq(d[0] & ~core.busy), core.reset.eq(d[1] & ~core.busy)]
                    with m.Elif(~core.busy):
                        with m.If(aw < 0x8800):
                            m.d.comb += [core.w_waddr.eq(aw[2:]), core.w_wdata.eq(d), core.w_we.eq(1)]
                        with m.Elif((aw >= 0x9000) & (aw < 0x9400)):
                            m.d.comb += [core.b_waddr.eq(aw[2:10]), core.b_wdata.eq(d), core.b_we.eq(1)]
                        with m.Elif((aw >= 0x9400) & (aw < 0x9480)):
                            m.d.comb += [core.x_waddr.eq(aw[2:7]), core.x_wdata.eq(d), core.x_we.eq(1)]
                    m.next = 'BRESP'
                with m.Elif(self.s_axi_arvalid):
                    m.d.comb += self.s_axi_arready.eq(1)
                    m.d.sync += rd_reg.eq(ar[8:] == 0xFF)
                    with m.Switch(ar[:8]):
                        with m.Case(0x04):
                            m.d.sync += rd_regval.eq(Cat(core.busy, core.valid))
                        with m.Case(0x08):
                            m.d.sync += rd_regval.eq(ID)
                        with m.Default():
                            m.d.sync += rd_regval.eq(0)
                    m.d.comb += core.h_raddr.eq(ar[2:8])
                    m.next = 'RDATA'
            with m.State('BRESP'):
                m.d.comb += self.s_axi_bvalid.eq(1)
                with m.If(self.s_axi_bready):
                    m.next = 'IDLE'
            with m.State('RDATA'):
                rdata = Signal(32)
                first = Signal(init=1)
                with m.If(first):
                    m.d.sync += [rdata.eq(Mux(rd_reg, rd_regval, core.h_rdata)), first.eq(0)]
                m.d.comb += [self.s_axi_rvalid.eq(~first), self.s_axi_rdata.eq(rdata)]
                with m.If(~first & self.s_axi_rready):
                    m.d.sync += first.eq(1)
                    m.next = 'IDLE'
        return m


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output_file')
    args = parser.parse_args()
    top = RsnnFrontAxi()
    with open(args.output_file, 'w') as f:
        f.write(amaranth.back.verilog.convert(
            top, name='rsnn_front_axi', ports=top.ports(), emit_src=False))
