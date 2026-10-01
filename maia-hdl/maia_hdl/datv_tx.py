#
# SPDX-License-Identifier: MIT
#

"""DATV transmit back end for tezuka_fw_simple: the DVB-S2 encoder's
symbols (AXI-Stream as dvb_fpga packs them: Q in bits 31:16, I in 15:0, see
its PL scrambler's m_tdata <= im & re) through ArbInterpolator, samples out
towards the DAC with I in bits 31:16 and Q in 15:0 (datv_merge.v). One clock (the CPU clock) for
the AXI4-Lite registers and the datapath.

Registers (word offsets):
  0x0 step         symbol rate / DAC rate * 2**32
  0x4 coeff_addr   coefficient table write address (k * 256 + phase)
  0x8 coeff        bit 0 write strobe, bits 18:1 coefficient
  0xC id           "DTX3" (0x33585444, little-endian ASCII): "DTX2" before
                   the underflow register, "DTX1" before the DVB-T2
                   transmit IFFT (t2ifft) sat in front of it
  0x10 underflows  bits 15:0: DAC requests datv_merge.v found no sample for
                   (it sent zeros) since DATV was selected (DAC GPIO bit 1);
                   the counter comes Gray-coded from the DAC clock
"""

import argparse

from amaranth import *
import amaranth.back.verilog
from amaranth.lib.cdc import FFSynchronizer

from .arb_interp import ArbInterpolator
from .axi4_lite import Axi4LiteRegisterBridge
from .register import Access, Field, Registers, Register


class DatvTx(Elaboratable):
    def __init__(self):
        self.interp = ArbInterpolator()
        self.axi4lite = Axi4LiteRegisterBridge(3, name='s_axi_lite')
        self.registers = Registers(
            'datv_tx',
            {
                0b00: Register('step', [
                    Field('step', Access.RW, 32, 0)]),
                0b01: Register('coeff_addr', [
                    Field('coeff_waddr', Access.RW, self.interp.aw, 0)]),
                0b10: Register('coeff', [
                    Field('coeff_wren', Access.Wpulse, 1, 0),
                    Field('coeff_wdata', Access.RW, self.interp.cw, 0)]),
                0b11: Register('id', [
                    Field('id', Access.R, 32, 0x33585444)]),
                0b100: Register('underflows', [
                    Field('underflows', Access.R, 16, 0)]),
            },
            3)
        # datv_merge.v's counter, Gray-coded in the DAC clock
        self.underflows_gray = Signal(16)
        self.s_axis_tdata = Signal(32)
        self.s_axis_tvalid = Signal()
        self.s_axis_tready = Signal()
        self.m_axis_tdata = Signal(32)
        self.m_axis_tvalid = Signal()
        self.m_axis_tready = Signal()

    def ports(self):
        return self.axi4lite.axi.ports() + [
            self.s_axis_tdata, self.s_axis_tvalid, self.s_axis_tready,
            self.m_axis_tdata, self.m_axis_tvalid, self.m_axis_tready,
            self.underflows_gray]

    def elaborate(self, platform):
        m = Module()
        m.submodules.axi4lite = self.axi4lite
        m.submodules.registers = regs = self.registers
        m.submodules.interp = ip = self.interp
        for s in ['ren', 'wstrobe', 'address', 'wdata']:
            m.d.comb += getattr(regs, s).eq(getattr(self.axi4lite, s))
        for s in ['rdata', 'rdone', 'wdone']:
            m.d.comb += getattr(self.axi4lite, s).eq(getattr(regs, s))
        # into this clock (the first flop's input: datv.xdc max delay), back
        # to binary
        und_sync = Signal(16, name='und_sync')
        m.submodules.und_cdc = FFSynchronizer(self.underflows_gray, und_sync)
        und = Signal(16)
        for i in range(16):
            m.d.comb += und[i].eq(Cat(*[und_sync[j] for j in range(i, 16)]).xor())
        m.d.sync += regs['underflows']['underflows'].eq(und)
        m.d.comb += [
            ip.step.eq(regs['step']['step']),
            ip.coeff_waddr.eq(regs['coeff_addr']['coeff_waddr']),
            ip.coeff_wdata.eq(regs['coeff']['coeff_wdata']),
            ip.coeff_wren.eq(regs['coeff']['coeff_wren']),
            # dvb_fpga: Q high, I low. (I high here sent I and Q swapped:
            # every second pi/2-BPSK header symbol inverted, all data wrong.)
            ip.in_re.eq(self.s_axis_tdata[:16]),
            ip.in_im.eq(self.s_axis_tdata[16:]),
            ip.in_valid.eq(self.s_axis_tvalid),
            self.s_axis_tready.eq(ip.in_ready),
            self.m_axis_tdata.eq(Cat(ip.out_im, ip.out_re)),
            self.m_axis_tvalid.eq(ip.out_valid),
            ip.out_ready.eq(self.m_axis_tready),
        ]
        return m


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('output_file')
    args = parser.parse_args()
    top = DatvTx()
    with open(args.output_file, 'w') as f:
        f.write(amaranth.back.verilog.convert(
            top, name='datv_tx', ports=top.ports(), emit_src=False))
