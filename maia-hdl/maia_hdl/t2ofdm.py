#
# SPDX-License-Identifier: MIT
#

"""DVB-T2 OFDM front end (tezuka_fw_simple): after the T2 resampler, before
the DATV ring. The ARM keeps channel estimation, equalization,
deinterleaving and LLRs; this does the per-sample work it cannot keep up
with.

Samples at the T2 elementary rate are counted (``counter``, 32 bits). With
``scheduled`` off every sample goes to the ring (acquisition: the ARM finds
P1 in them). With it on, frames are laid out from a start ``F`` (counter
units) the ARM gives (``next_start`` + ``load``; taken at once when no frame
is running, else at the end of the current frame; without a new one each
frame follows the last, ``F += frame_len``). Relative to ``F``, ``r``:

A start given too late (already more than a frame past) moves on a frame
a sample until it is ahead; the frame it lands in is left out (only the
next P1's raw window).

- raw samples (as received) where the ARM needs them: P1 and ``track``
  either side (``r < 2048 + track``, ``r >= frame_len - track``), and each
  symbol's guard interval and the tail it copies (``q < gi``,
  ``q >= 2048``, ``q`` the position in the symbol, ``u = r - 2048``,
  ``j = u // (2048 + gi)``);
- an FFT (2048, radix 2, maia-hdl ``FFT``) of each symbol's window
  ``gi - early <= q < gi - early + 2048``, after an NCO (``freq``, a
  32-bit phase step a sample, 1024-entry cos/sin table); of its output
  (bit-reversed order) only the active carriers' bins (``active``), shifted
  right by ``shift`` and saturated to 16 bits.

Ring words (Recorder16IQ: re in 15:0, im in 31:16): bit 0 of re is set on
headers, bit 0 of im (bit 16) on the carrier stream; the other bits carry
the value (its LSB lost). Raw stream: a header (payload: the counter of the
next sample) at every run start and every 65536 samples of a run, then
samples. Carrier stream: per FFT a header (payload: symbol j in bits 7:0,
the low 22 bits of its frame's start ``F`` in 29:8) then the active
carriers in bit-reversed bin order.
Header payload = re[15:1] | im[15:1] << 15.
"""

import numpy as np
from amaranth import *
from amaranth.lib.fifo import SyncFIFOBuffered
from amaranth.lib.memory import Memory

from .fft import FFT
from .t2eq import T2Eq

N = 2048
ORDER = 11
NCO_BITS = 10
TRUNCATES = [1] * 6 + [0] * 5
RUN_HEADER_EVERY = 65536


def bitrev(i, bits=ORDER):
    return int(format(i, f'0{bits}b')[::-1], 2)


def t2_active_bins(carriers=1705):
    """Natural FFT bins of the T2 2K carriers (trxd ofdm.rs bin())."""
    left = (N - carriers) // 2 + 1
    bins = []
    for k in range(carriers):
        b = left + k
        bins.append(b - N // 2 if b >= N // 2 else b + N // 2)
    return bins


def nco_tables():
    ph = 2 * np.pi * np.arange(2**NCO_BITS) / 2**NCO_BITS
    return (np.round(32767 * np.cos(ph)).astype(int),
            np.round(32767 * np.sin(ph)).astype(int))


def new_fft():
    # Plain complex multipliers (3 DSPs a twiddle): the single-multiplier
    # 3x-clock version (cmult3x) gave wrong FFTs on the board with a sample
    # only every ~34 clocks (the spectrometer feeds one every clock).
    return FFT(16, ORDER, 2, width_twiddle=16, truncates=TRUNCATES,
               use_bram_reg=True)


def pack(re, im, header, carrier):
    re = (re & 0xFFFE) | header
    im = (im & 0xFFFE) | carrier
    return (re & 0xFFFF) | (im & 0xFFFF) << 16


def pack_header(payload, carrier):
    re = ((payload & 0x7FFF) << 1) | 1
    im = (((payload >> 15) & 0x7FFF) << 1) | carrier
    return re | im << 16


def sat16(x):
    return max(-32768, min(32767, int(x)))


class Model:
    """Bit-exact model: samples (sequence of (re, im)) and register writes
    at sample indices -> the two word streams (raw, carrier), each a list of
    32-bit words in order (the merge between them is not modelled).

    ``events``: {sample_index: dict of register values to apply before that
    sample}; keys: scheduled, frame_len, nsym, gi, early, track, freq,
    shift, next_start (with load)."""
    def __init__(self, active):
        self.active = set(active)
        self.fft = new_fft()
        self.cos, self.sin = nco_tables()

    def run(self, samples, events):
        reg = dict(scheduled=0, frame_len=0, nsym=0, gi=256, early=64,
                   track=64, freq=0, shift=0)
        raw, car = [], []
        counter = 0
        running = False
        F = 0
        pending = None
        run_len = 0        # length of the raw run in progress (0: none)
        phase = 0
        frame_no = 0
        win = []           # samples of the FFT window being collected
        win_j = None
        # The FFT's latency: window v's output word i leaves on window sample
        # (since the last restart) 2048 v + i + delay - 1.
        delay = self.fft.delay
        clk = 0
        vecs = {}          # window number -> (tag, output re, im)
        tags = []
        skip = False       # the rest of a frame caught up with
        for n, (x_re, x_im) in enumerate(samples):
            if n in events:
                ev = dict(events[n])
                load = ev.pop('load', False)
                start = ev.pop('next_start', None)
                reg.update(ev)
                if load:
                    pending = start
                if not reg['scheduled']:
                    running = False
                    pending = pending if load else None
            if reg['scheduled'] and not running and pending is not None:
                F, pending, running = pending, None, True
                frame_no = 0
                # a start already past: the rest of that frame left out
                skip = (counter - F + 2**31) % 2**32 - 2**31 > 0
            if not (reg['scheduled'] and running):
                # no schedule: the FFT and its labels restart
                clk, vecs, tags, win = 0, {}, [], []
            # the NCO turns every sample
            idx = (phase >> (32 - NCO_BITS)) & (2**NCO_BITS - 1)
            c, s = self.cos[idx], self.sin[idx]
            y_re = sat16((x_re * c - x_im * s + (1 << 14)) >> 15)
            y_im = sat16((x_re * s + x_im * c + (1 << 14)) >> 15)
            phase = (phase + reg['freq']) & 0xFFFFFFFF
            is_raw, in_win, j = True, False, 0
            if reg['scheduled'] and running:
                r = (counter - F + 2**31) % 2**32 - 2**31
                fl, gi = reg['frame_len'], reg['gi']
                sl = N + gi
                is_raw = False
                if r < 0:
                    is_raw = r >= -reg['track']
                elif r < fl:
                    if r >= fl - reg['track'] or (
                            r < N + reg['track'] and not skip):
                        is_raw = True
                    if r >= N and not skip:
                        u = r - N
                        j, q = divmod(u, sl)
                        if j < reg['nsym']:
                            if q < gi or q >= N:
                                is_raw = True
                            lo = gi - reg['early']
                            if lo <= q < lo + N:
                                in_win = True
                if r == fl - 1:
                    if pending is not None:
                        F, pending = pending, None
                    else:
                        F = (F + fl) % 2**32
                    frame_no += 1
                    skip = False
                elif r >= fl:
                    # A start already past (given late): catch up, and
                    # leave out the frame that lands in.
                    F = (F + fl) % 2**32
                    skip = True
            elif reg['scheduled']:
                is_raw = False
            if is_raw:
                if run_len == 0 or run_len % RUN_HEADER_EVERY == 0:
                    raw.append(pack_header(counter & 0x3FFFFFFF, 0))
                raw.append(pack(x_re, x_im, 0, 0))
                run_len += 1
            else:
                run_len = 0
            if in_win:
                if not win:
                    # the frame start's low 21 bits (bit 29: equalized, t2eq)
                    tags.append(j | (F & 0x1FFFFF) << 8)
                win.append((y_re, y_im))
                if len(win) == N:
                    wr = np.array([v[0] for v in win])
                    wi = np.array([v[1] for v in win])
                    ore, oim = self.fft.model(wr, wi)
                    v = clk // N
                    vecs[v] = (tags[v], ore, oim)
                    win = []
                c = clk - (delay - 1)
                if c >= 0:
                    v, i = divmod(c, N)
                    tag, ore, oim = vecs[v]
                    if i == 0:
                        car.append(pack_header(tag, 1))
                    if bitrev(i) in self.active:
                        vr = sat16(int(ore[i]) >> reg['shift'])
                        vi = sat16(int(oim[i]) >> reg['shift'])
                        car.append(pack(vr, vi, 0, 1))
                    if i == N - 1:
                        del vecs[v]
                clk += 1
            counter = (counter + 1) & 0xFFFFFFFF
        return raw, car


class T2Ofdm(Elaboratable):
    """See the module docstring. Clock domain ``sync``; the FFT's complex
    multipliers run in ``clk3x`` (``common_edge_3x`` as for Maia's other
    cmult3x users)."""
    def __init__(self, active=None):
        self.active = set(t2_active_bins() if active is None else active)
        self.fft = new_fft()
        # the equalizer on the carrier stream (its registers are its own)
        self.eq = T2Eq()
        # registers (sync domain)
        self.enable = Signal()       # front end on (else nothing out)
        self.scheduled = Signal()
        self.load = Signal()         # pulse: next_start pending
        self.next_start = Signal(32)
        self.frame_len = Signal(20)
        self.nsym = Signal(8)
        self.gi = Signal(10)
        self.early = Signal(8)
        self.track = Signal(8)
        self.freq = Signal(32)
        self.shift = Signal(3)
        # debug: every sample raw, schedule or not (FFTs checkable against
        # the raw samples)
        self.raw_always = Signal()
        self.counter = Signal(32)    # out
        self.frames = Signal(22)     # out
        self.overflow = Signal()     # out, sticky until disabled
        self.common_edge_3x = Signal()
        # samples in
        self.strobe_in = Signal()
        self.re_in = Signal(signed(16))
        self.im_in = Signal(signed(16))
        # words out (to the recorder: re 15:0, im 31:16)
        self.strobe_out = Signal()
        self.re_out = Signal(16)
        self.im_out = Signal(16)

    def elaborate(self, platform):
        m = Module()
        self.fft_rst = Signal()
        fft = ResetInserter(self.fft_rst)(self.fft)
        m.submodules.fft = fft

        # NCO tables
        cos_t, sin_t = nco_tables()
        m.submodules.cos = cos_mem = Memory(shape=signed(16), depth=2**NCO_BITS, init=[int(v) for v in cos_t])
        m.submodules.sin = sin_mem = Memory(shape=signed(16), depth=2**NCO_BITS, init=[int(v) for v in sin_t])
        cos_rd = cos_mem.read_port()
        sin_rd = sin_mem.read_port()

        # Active-bin mask in output (bit-reversed) order.
        mask = [1 if bitrev(i) in self.active else 0 for i in range(N)]
        m.submodules.mask = mask_mem = Memory(shape=1, depth=N, init=mask)
        mask_rd = mask_mem.read_port()

        # ---- stage 0: sample in, schedule decided ----
        phase = Signal(32)
        running = Signal()
        pending = Signal()
        pend_start = Signal(32)
        F = Signal(32)
        frame_no = Signal(22)
        r = Signal(signed(33))
        q = Signal(12)
        j = Signal(9)
        run_len = Signal(32)
        skip = Signal()        # the rest of a frame caught up with

        x_re = Signal(signed(16))
        x_im = Signal(signed(16))
        s1 = Signal()          # sample in stage 1 (table read done next)
        s1_raw = Signal()
        s1_win = Signal()
        s1_j = Signal(8)
        s1_frame = Signal(22)
        s1_counter = Signal(32)
        s1_head = Signal()     # raw run header before this sample

        with m.If(self.load):
            m.d.sync += [pending.eq(1), pend_start.eq(self.next_start)]

        m.d.comb += [cos_rd.addr.eq(phase[32 - NCO_BITS:]),
                     sin_rd.addr.eq(phase[32 - NCO_BITS:])]

        fl = self.frame_len
        sl = Signal(12)
        m.d.comb += sl.eq(N + self.gi)
        m.d.comb += r.eq((self.counter - F)[:32].as_signed())
        # A start already past: the symbol counters would start from 0 in
        # the middle of that frame (windows and labels out of step, and the
        # FFT's framing with them): its rest is left out.
        start_past = Signal()
        m.d.comb += start_past.eq((self.counter - pend_start)[:32].as_signed() > 0)
        in_frame = Signal()
        m.d.comb += in_frame.eq((r >= 0) & (r < fl))

        m.d.sync += s1.eq(0)
        with m.If(~self.enable):
            m.d.sync += [running.eq(0), pending.eq(0), self.counter.eq(0),
                         phase.eq(0), run_len.eq(0), frame_no.eq(0),
                         self.frames.eq(0)]
        with m.Elif(self.strobe_in):
            m.d.sync += [x_re.eq(self.re_in), x_im.eq(self.im_in), s1.eq(1),
                         s1_counter.eq(self.counter), s1_frame.eq(F[:22]),
                         s1_j.eq(j[:8]),
                         self.counter.eq(self.counter + 1),
                         phase.eq(phase + self.freq)]
            is_raw = Signal()
            in_win = Signal()
            sched = Signal()
            m.d.comb += sched.eq(self.scheduled & running)
            with m.If(~self.scheduled):
                m.d.comb += is_raw.eq(1)
            with m.Elif(~running):
                m.d.comb += is_raw.eq(0)
            with m.Elif(r < 0):
                m.d.comb += is_raw.eq(r >= -self.track.as_unsigned())
            with m.Elif(in_frame):
                with m.If(((r < N + self.track) & ~skip)
                          | (r >= fl - self.track)):
                    m.d.comb += is_raw.eq(1)
                with m.If((r >= N) & (j < self.nsym) & ~skip):
                    with m.If((q < self.gi) | (q >= N)):
                        m.d.comb += is_raw.eq(1)
                    lo = self.gi - self.early
                    with m.If((q >= lo) & (q < lo + N)):
                        m.d.comb += in_win.eq(1)
            m.d.sync += [s1_raw.eq(is_raw | self.raw_always), s1_win.eq(in_win)]
            with m.If(is_raw | self.raw_always):
                m.d.sync += [s1_head.eq(run_len[:16] == 0),
                             run_len.eq(run_len + 1)]
            with m.Else():
                m.d.sync += [s1_head.eq(0), run_len.eq(0)]
            # symbol position counters (valid inside the frame after P1)
            with m.If(sched & in_frame & (r >= N)):
                with m.If(q == sl - 1):
                    m.d.sync += [q.eq(0), j.eq(j + 1)]
                with m.Else():
                    m.d.sync += q.eq(q + 1)
            with m.Else():
                m.d.sync += [q.eq(0), j.eq(0)]
            # frame advance
            with m.If(self.scheduled & ~running & pending):
                m.d.sync += [F.eq(pend_start), pending.eq(0),
                             running.eq(1), frame_no.eq(0),
                             skip.eq(start_past)]
            with m.Elif(sched & (r == fl - 1)):
                with m.If(pending):
                    m.d.sync += [F.eq(pend_start), pending.eq(0)]
                with m.Else():
                    m.d.sync += F.eq(F + fl)
                m.d.sync += [frame_no.eq(frame_no + 1),
                             self.frames.eq(self.frames + 1), skip.eq(0)]
            with m.Elif(sched & (r >= fl)):
                # A start already past (given late): catch up a frame a
                # sample, and leave out the frame that lands in.
                m.d.sync += [F.eq(F + fl), skip.eq(1)]
            with m.If(~self.scheduled):
                m.d.sync += running.eq(0)
        with m.Else():
            with m.If(~self.scheduled):
                m.d.sync += running.eq(0)
            with m.If(self.scheduled & ~running & pending):
                m.d.sync += [F.eq(pend_start), pending.eq(0),
                             running.eq(1), frame_no.eq(0),
                             skip.eq(start_past)]

        # ---- stage 1 -> 2: NCO multiply (tables read) ----
        s2 = Signal()
        s2_raw = Signal()
        s2_win = Signal()
        s2_head = Signal()
        s2_j = Signal(8)
        s2_frame = Signal(22)
        s2_counter = Signal(32)
        s2_xre = Signal(signed(16))
        s2_xim = Signal(signed(16))
        p_rr = Signal(signed(33))
        p_ii = Signal(signed(33))
        p_ri = Signal(signed(33))
        p_ir = Signal(signed(33))
        m.d.sync += s2.eq(s1)
        with m.If(s1):
            m.d.sync += [
                s2_raw.eq(s1_raw), s2_win.eq(s1_win), s2_head.eq(s1_head),
                s2_j.eq(s1_j), s2_frame.eq(s1_frame),
                s2_counter.eq(s1_counter), s2_xre.eq(x_re), s2_xim.eq(x_im),
                p_rr.eq(x_re * cos_rd.data), p_ii.eq(x_im * sin_rd.data),
                p_ri.eq(x_re * sin_rd.data), p_ir.eq(x_im * cos_rd.data),
            ]
        y_re = Signal(signed(35))
        y_im = Signal(signed(35))
        m.d.comb += [y_re.eq((p_rr - p_ii + (1 << 14)) >> 15),
                     y_im.eq((p_ri + p_ir + (1 << 14)) >> 15)]

        def sat(x):
            return Mux(x > 32767, 32767, Mux(x < -32768, -32768, x))

        # ---- stage 2: raw words, FFT input ----
        raw_fifo = SyncFIFOBuffered(width=32, depth=32)
        car_fifo = SyncFIFOBuffered(width=32, depth=32)
        m.submodules.raw_fifo = raw_fifo
        m.submodules.car_fifo = car_fifo

        # Raw: header (if due) and sample, one word a cycle.
        raw_pend = Signal()
        raw_word = Signal(32)
        m.d.comb += raw_fifo.w_en.eq(0)
        with m.If(s2 & s2_raw):
            with m.If(s2_head):
                p = s2_counter[:30]
                m.d.comb += [raw_fifo.w_data.eq(Cat(C(1, 1), p[:15], C(0, 1), p[15:30])),
                             raw_fifo.w_en.eq(1)]
                m.d.sync += [raw_pend.eq(1),
                             raw_word.eq(Cat(C(0, 1), s2_xre[1:], C(0, 1), s2_xim[1:]))]
            with m.Else():
                m.d.comb += [raw_fifo.w_data.eq(Cat(C(0, 1), s2_xre[1:], C(0, 1), s2_xim[1:])),
                             raw_fifo.w_en.eq(1)]
        with m.Elif(raw_pend):
            m.d.comb += [raw_fifo.w_data.eq(raw_word), raw_fifo.w_en.eq(1)]
            m.d.sync += raw_pend.eq(0)

        # FFT input: clken on each window sample.
        win_cnt = Signal(ORDER + 1)
        # The FFT, its windows' labels and the counters restart whenever no
        # schedule runs: windows cut short (raw-everything asked for, the
        # front end off) would otherwise leave labels behind and the FFT's
        # framing out of step with the windows for good.
        tag_fifo = ResetInserter(self.fft_rst)(SyncFIFOBuffered(width=30, depth=4))
        m.submodules.tag_fifo = tag_fifo
        m.d.comb += [fft.re_in.eq(sat(y_re)), fft.im_in.eq(sat(y_im)),
                     fft.clken.eq(s2 & s2_win),
                     tag_fifo.w_data.eq(Cat(s2_j, s2_frame)),
                     tag_fifo.w_en.eq(s2 & s2_win & (win_cnt == 0))]
        with m.If(s2 & s2_win):
            m.d.sync += win_cnt.eq(Mux(win_cnt == N - 1, 0, win_cnt + 1))

        # ---- FFT output: one output a clken, valid after `delay` clkens ----
        delay = self.fft.delay
        ocnt = Signal(range(delay + N + 1))
        primed = Signal()
        oidx = Signal(ORDER)
        clken_d = Signal()
        m.d.sync += clken_d.eq(s2 & s2_win)
        m.d.comb += self.fft_rst.eq(~self.enable | ~self.scheduled | ~running)

        # The output belonging to clken number c (from 0) is presented after
        # clken c, for c >= delay - 1 (see test_t2ofdm: alignment checked
        # against the model).
        out_valid = Signal()
        with m.If(clken_d):
            with m.If(~primed):
                m.d.sync += ocnt.eq(ocnt + 1)
                with m.If(ocnt == delay - 2):
                    m.d.sync += primed.eq(1)
            with m.Else():
                m.d.sync += oidx.eq(oidx + 1)
            m.d.comb += out_valid.eq(primed)
        m.d.comb += mask_rd.addr.eq(oidx)
        # mask_rd.data is for oidx (address set a cycle before use: oidx
        # changes on clken_d only, clken_d cycles are far apart).
        o_re = Signal(signed(16))
        o_im = Signal(signed(16))
        m.d.comb += [o_re.eq(sat(fft.re_out >> self.shift)),
                     o_im.eq(sat(fft.im_out >> self.shift))]
        car_pend = Signal()
        car_word = Signal(32)
        m.d.comb += [car_fifo.w_en.eq(0), tag_fifo.r_en.eq(0)]
        with m.If(out_valid):
            data = Cat(C(0, 1), o_re[1:], C(1, 1), o_im[1:])
            with m.If(oidx == 0):
                tag = tag_fifo.r_data
                m.d.comb += [car_fifo.w_data.eq(Cat(C(1, 1), tag[:15], C(1, 1), tag[15:30])),
                             car_fifo.w_en.eq(1), tag_fifo.r_en.eq(1)]
                with m.If(mask_rd.data):
                    m.d.sync += [car_pend.eq(1), car_word.eq(data)]
            with m.Elif(mask_rd.data):
                m.d.comb += [car_fifo.w_data.eq(data), car_fifo.w_en.eq(1)]
        with m.Elif(car_pend):
            m.d.comb += [car_fifo.w_data.eq(car_word), car_fifo.w_en.eq(1)]
            m.d.sync += car_pend.eq(0)

        # Restart (after the updates above: the last assignment wins).
        with m.If(self.fft_rst):
            m.d.sync += [ocnt.eq(0), primed.eq(0), oidx.eq(0), win_cnt.eq(0),
                         car_pend.eq(0)]

        # ---- equalizer: carrier FIFO -> t2eq -> its FIFO ----
        m.submodules.eq = eq = self.eq
        eq_fifo = SyncFIFOBuffered(width=32, depth=32)
        m.submodules.eq_fifo = eq_fifo
        m.d.comb += [eq.i_data.eq(car_fifo.r_data), eq.i_rdy.eq(car_fifo.r_rdy),
                     car_fifo.r_en.eq(eq.i_en),
                     eq.o_rdy.eq(eq_fifo.w_rdy),
                     eq_fifo.w_data.eq(eq.o_data), eq_fifo.w_en.eq(eq.o_en)]

        # ---- merge: a word every other cycle, carriers first ----
        turn = Signal()
        m.d.sync += [self.strobe_out.eq(0), turn.eq(~turn)]
        m.d.comb += [raw_fifo.r_en.eq(0), eq_fifo.r_en.eq(0)]
        with m.If(turn):
            with m.If(eq_fifo.r_rdy):
                m.d.comb += eq_fifo.r_en.eq(1)
                m.d.sync += [self.strobe_out.eq(1),
                             self.re_out.eq(eq_fifo.r_data[:16]),
                             self.im_out.eq(eq_fifo.r_data[16:])]
            with m.Elif(raw_fifo.r_rdy):
                m.d.comb += raw_fifo.r_en.eq(1)
                m.d.sync += [self.strobe_out.eq(1),
                             self.re_out.eq(raw_fifo.r_data[:16]),
                             self.im_out.eq(raw_fifo.r_data[16:])]
        with m.If(~self.enable):
            m.d.sync += self.overflow.eq(0)
        with m.Elif((raw_fifo.w_en & ~raw_fifo.w_rdy) | (car_fifo.w_en & ~car_fifo.w_rdy)):
            m.d.sync += self.overflow.eq(1)
        return m
