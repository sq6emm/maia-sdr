#
# Copyright (C) 2022-2024 Daniel Estevez <daniel@destevez.net>
#
# This file is part of maia-sdr
#
# SPDX-License-Identifier: MIT
#

import argparse

from amaranth import *
from amaranth.lib.cdc import FFSynchronizer, PulseSynchronizer
import amaranth.back.verilog

from .axi4_lite import Axi4LiteRegisterBridge
from .cdc import RegisterCDC, RxIQCDC
from .clknx import ClkNxCommonEdge
from .config import MaiaSDRConfig
from . import configs
from .ddc import DDC
from .pulse import PulseStretcher
from .pluto_platform import PlutoPlatform
from .register import Access, Field, Registers, Register, RegisterMap
from .recorder import Recorder16IQ, RecorderMode
from .spectrometer import Spectrometer
from .symsync import SymSync
from .hdrdet import HdrDet
from .t2resamp import T2Resampler
from .t2ofdm import T2Ofdm
from .topfft import TopFFT

# IP core version
_version = '0.6.1'


class MaiaSDR(Elaboratable):
    """Maia SDR top level

    This elaboratable is the top-level Maia SDR IP core.
    """
    def __init__(self, config=MaiaSDRConfig()):
        config.validate()
        self.config = config
        # The DATV cores add a second 16-word window (0x40..0x7F): the
        # DVB-T2 front end's registers.
        self.axi4_awidth = 5 if config.datv_symsync else 4
        self.s_axi_lite = ClockDomain()
        self.sampling = ClockDomain()
        # A clock domain called 'sync' is added to override the default
        # behaviour, since we drive the reset internally.
        #
        # See https://github.com/amaranth-lang/amaranth/issues/1506
        self.sync = ClockDomain()
        self.clk2x = ClockDomain()
        self.clk3x = ClockDomain()

        self.axi4lite = Axi4LiteRegisterBridge(
            self.axi4_awidth, name='s_axi_lite')
        self.control_registers = Registers(
            'control',
            {
                0b00: Register(
                    'product_id', [
                        Field('product_id', Access.R, 32, 0x6169616d)
                    ]),
                0b01: Register('version', [
                    Field('bugfix', Access.R, 8,
                          int(_version.split('.')[2])),
                    Field('minor', Access.R, 8,
                          int(_version.split('.')[1])),
                    Field('major', Access.R, 8,
                          int(_version.split('.')[0])),
                    Field('platform', Access.R, 8, config.platform),
                ]),
                0b10: Register('control', [
                    Field('sdr_reset', Access.RW, 1, 1),
                ]),
                0b11: Register('interrupts', [
                    Field('spectrometer', Access.Rsticky, 1, 0),
                    Field('recorder', Access.Rsticky, 1, 0),
                ], interrupt=True),
            },
            2)
        self.recorder_registers = Registers(
            'recorder',
            {
                0b0: Register('recorder_control', [
                    Field('start', Access.Wpulse, 1, 0),
                    Field('stop', Access.Wpulse, 1, 0),
                    Field('mode', Access.RW,
                          Shape.cast(RecorderMode).width, 0),
                    Field('dropped_samples', Access.R, 1, 0),
                ]),
                0b01: Register('recorder_next_address', [
                    Field('next_address', Access.R, 32, 0),
                ]),
                0b10: Register('recorder_committed_address', [
                    Field('committed_address', Access.R, 32, 0),
                ]),
            },
            2)
        self.spectrometer = Spectrometer(
            config.spectrometer_address,
            config.spectrometer_buffers.bit_length() - 1,
            dma_name='m_axi_spectrometer')
        self.recorder = Recorder16IQ(
            config.recorder_address_range[0],
            config.recorder_address_range[1],
            dma_name='m_axi_recorder', domain_in='sync',
            domain_dma='s_axi_lite', ring=config.recorder_ring)
        self.ddc = DDC('clk3x')
        if config.Enable_RawFFT:
            self.raw_fft = TopFFT()
        self.sdr_registers = Registers(
            'sdr', {
                0b000: Register(
                    'spectrometer',
                    [
                        Field('use_ddc_out',
                              Access.RW,
                              1,
                              0),
                        Field('num_integrations',
                              Access.RW,
                              self.spectrometer.nint_width,
                              -1),
                        Field('abort', Access.Wpulse, 1, 0),
                        Field('last_buffer',
                              Access.R,
                              len(self.spectrometer.last_buffer),
                              0),
                        Field('peak_detect',
                              Access.RW,
                              1,
                              0),
                    ]),
                0b001: Register(
                    'ddc_coeff_addr',
                    [
                        # 10 bits for the DDC, 12 for the T2 resampler
                        Field('coeff_waddr',
                              Access.RW,
                              12,
                              0),
                    ]),
                0b010: Register(
                    'ddc_coeff',
                    [
                        Field('coeff_wren',
                              Access.Wpulse,
                              1,
                              0),
                        Field('coeff_wdata',
                              Access.RW,
                              18,
                              0),
                    ]),
                0b011: Register(
                    'ddc_decimation',
                    [
                        Field('decimation1',
                              Access.RW,
                              7,
                              0),
                        Field('decimation2',
                              Access.RW,
                              6,
                              0),
                        Field('decimation3',
                              Access.RW,
                              7,
                              0),
                    ]),
                0b100: Register(
                    'ddc_frequency',
                    [
                        Field('frequency',
                              Access.RW,
                              28,
                              0),
                    ]),
                0b101: Register(
                    'ddc_control',
                    [
                        Field('operations_minus_one1',
                              Access.RW,
                              7,
                              0),
                        Field('operations_minus_one2',
                              Access.RW,
                              6,
                              0),
                        Field('operations_minus_one3',
                              Access.RW,
                              7,
                              0),
                        Field('odd_operations1',
                              Access.RW,
                              1,
                              0),
                        Field('odd_operations3',
                              Access.RW,
                              1,
                              0),
                        Field('bypass2',
                              Access.RW,
                              1,
                              0),
                        Field('bypass3',
                              Access.RW,
                              1,
                              0),
                        Field('enable_input',
                              Access.RW,
                              1,
                              0),
                    ]),
                **({
                    0b110: Register(
                        'datv_symsync',
                        [
                            Field('enable', Access.RW, 1, 0),
                            Field('kp_shift', Access.RW, 5, 7),
                            Field('ki_shift', Access.RW, 5, 13),
                            Field('hdrdet', Access.RW, 1, 0),
                            # DVB-T2 receive: the recorder takes the T2
                            # resampler (ADC samples to the T2 rate, step
                            # in datv_omega); t2_coeff sends the DDC
                            # coefficient writes to its table instead.
                            Field('t2', Access.RW, 1, 0),
                            Field('t2_coeff', Access.RW, 1, 0),
                        ]),
                    0b111: Register(
                        'datv_omega',
                        [
                            Field('omega', Access.RW, 32, 0),
                        ]),
                } if config.datv_symsync else {}),
            }, 3)
        if config.datv_symsync:
            self.symsync = SymSync()
            self.hdrdet = HdrDet()
            self.t2resamp = T2Resampler()
            self.t2ofdm = T2Ofdm()
            self.t2_registers = Registers(
                't2', {
                    0b000: Register('t2_control', [
                        Field('enable', Access.RW, 1, 0),
                        Field('scheduled', Access.RW, 1, 0),
                        Field('load', Access.Wpulse, 1, 0),
                        Field('shift', Access.RW, 3, 0),
                        Field('raw_always', Access.RW, 1, 0),
                    ]),
                    0b001: Register('t2_frame_len', [
                        Field('frame_len', Access.RW, 20, 0),
                    ]),
                    0b010: Register('t2_layout', [
                        Field('nsym', Access.RW, 8, 0),
                        Field('gi', Access.RW, 10, 256),
                        Field('early', Access.RW, 8, 64),
                    ]),
                    0b011: Register('t2_track', [
                        Field('track', Access.RW, 8, 64),
                    ]),
                    0b100: Register('t2_freq', [
                        Field('freq', Access.RW, 32, 0),
                    ]),
                    0b101: Register('t2_next_start', [
                        Field('next_start', Access.RW, 32, 0),
                    ]),
                    0b110: Register('t2_counter', [
                        Field('counter', Access.R, 32, 0),
                    ]),
                    0b111: Register('t2_status', [
                        Field('frames', Access.R, 22, 0),
                        Field('overflow', Access.R, 1, 0),
                        Field('resamp_overflow', Access.R, 1, 0),
                    ]),
                }, 3)
        metadata = {
            'vendor': 'Daniel Estevez',
            'vendorID': 'destevez.net',
            'name': 'Maia SDR',
            'series': 'Maia SDR',
            'version': _version,
            'description': f'Maia SDR IP core (platform {config.platform})',
            'licenseText': ('SPDX-License-Identifier: MIT '
                            'Copyright (C) Daniel Estevez 2022-2024'),
        }
        self.register_map = RegisterMap({
            0x0: self.control_registers,
            0x10: self.recorder_registers,
            0x20: self.sdr_registers,
            **({0x40: self.t2_registers} if config.datv_symsync else {}),
        }, metadata)

        self.iq_in_width = 12
        self.re_in = Signal(self.iq_in_width)
        self.im_in = Signal(self.iq_in_width)
        # The ADC FIFO's valid (DATV cores): the sampling clock does not have
        # a new sample every cycle; without it about 1 % of the samples came
        # in twice (fatal to DVB-T2's OFDM).
        self.valid_in = Signal(init=1)
        self.interrupt_out = Signal()
        self.clk_fastlock_out = Signal()
        self.fastlock_profile_in = Signal(3)

        self.decim_re_out = Signal(signed(16))
        self.decim_im_out = Signal(signed(16))
        self.decim_strobe_out = Signal()
        if config.Enable_RawFFT:
            self.fft_re_out = Signal(signed(16))
            self.fft_im_out = Signal(signed(16))
            self.fft_strobe_out = Signal()
            
    def ports(self):
    # Start with the standard bus and clock ports
        p = (
            self.axi4lite.axi.ports()
            + self.spectrometer.dma.axi.ports()
            + self.recorder.dma.axi.ports()
            + [
                self.re_in,
                self.im_in,
            ]
            + ([self.valid_in] if self.config.datv_symsync else [])
            + [
                self.interrupt_out,
                self.s_axi_lite.clk,
                self.s_axi_lite.rst,
                self.sampling.clk,
                self.sync.clk,
                self.sync.rst,
                self.clk2x.clk,
                self.clk3x.clk,
                self.clk_fastlock_out,
                self.fastlock_profile_in,
                self.decim_re_out,
                self.decim_im_out,
                self.decim_strobe_out,
            ]
        )

        # Conditionally add the FFT ports
        if self.config.Enable_RawFFT:
            p += [
                self.fft_re_out,
                self.fft_im_out,
                self.fft_strobe_out,
            ]
            
        return p

    def svd(self):
        return self.register_map.svd()

    def elaborate(self, platform):
        m = Module()
        m.domains += [
            self.s_axi_lite,
            self.sampling,
            self.sync,
            self.clk2x,
            self.clk3x,
        ]
        
        s_axi_lite_renamer = DomainRenamer({'sync': 's_axi_lite'})
        m.submodules.axi4lite = s_axi_lite_renamer(self.axi4lite)
        m.submodules.control_registers = s_axi_lite_renamer(
            self.control_registers)
        m.submodules.recorder_registers = s_axi_lite_renamer(
            self.recorder_registers)
        m.submodules.spectrometer = self.spectrometer
        m.submodules.sync_spectrometer_interrupt = \
            sync_spectrometer_interrupt = PulseSynchronizer(
                i_domain='sync', o_domain='s_axi_lite')
       
        m.submodules.recorder = self.recorder
        m.submodules.ddc = self.ddc
        if self.config.Enable_RawFFT:
            m.submodules.raw_fft = raw_fft = self.raw_fft

        m.submodules.sdr_registers = self.sdr_registers
        m.submodules.sdr_registers_cdc = sdr_registers_cdc = RegisterCDC(
            's_axi_lite', 'sync', self.sdr_registers.aw)
        if self.config.datv_symsync:
            m.submodules.t2_registers = self.t2_registers
            m.submodules.t2_registers_cdc = t2_registers_cdc = RegisterCDC(
                's_axi_lite', 'sync', self.t2_registers.aw)

        m.submodules.common_edge_2x = common_edge_2x = ClkNxCommonEdge(
            'sync', 'clk2x', 2)
        m.submodules.common_edge_3x = common_edge_3x = ClkNxCommonEdge(
            'sync', 'clk3x', 3)

        # RX IQ CDC
        m.submodules.rxiq_cdc = rxiq_cdc = RxIQCDC(
            'sampling', 'sync', self.iq_in_width)
        m.d.comb += [rxiq_cdc.re_in.eq(self.re_in),
                     rxiq_cdc.im_in.eq(self.im_in)]
        if self.config.datv_symsync:
            m.d.comb += rxiq_cdc.valid_in.eq(self.valid_in)

        #CDC ddc out
        ddc_re_out = Signal(signed(16))
        ddc_im_out = Signal(signed(16))
        ddc_strobe_out = Signal()

        m.submodules.ddciq_cdc = ddciq_cdc = RxIQCDC(
             'sync','sampling', 16)
        m.d.comb += [ddciq_cdc.re_in.eq(self.ddc.re_out),
                     ddciq_cdc.im_in.eq(self.ddc.im_out)]

        #m.d.sync += [
        #        ddc_re_out.eq(ddciq_cdc.re_out),
        #        ddc_im_out.eq(ddciq_cdc.im_out),
        #        ddc_strobe_out.eq(ddciq_cdc.strobe_out),
        #]

        # Spectrometer (sync domain)
        spectrometer_re_in = Signal(
            self.spectrometer.width_in, reset_less=True)
        spectrometer_im_in = Signal(
            self.spectrometer.width_in, reset_less=True)
        assert len(spectrometer_re_in) == len(self.ddc.re_out)
        assert len(spectrometer_im_in) == len(self.ddc.im_out)
        spectrometer_strobe_in = Signal()
        with m.If(self.sdr_registers['spectrometer']['use_ddc_out']):
            m.d.sync += [
                spectrometer_re_in.eq(self.ddc.re_out),
                spectrometer_im_in.eq(self.ddc.im_out),
                spectrometer_strobe_in.eq(self.ddc.strobe_out),                
            ]
        with m.Else():
            shift = self.spectrometer.width_in - self.iq_in_width
            m.d.sync += [
                # The RX IQ samples have 12 bits, but the spectrometer input
                # has 16 bits. Push the 12 bits to the MSBs.
                spectrometer_re_in.eq(rxiq_cdc.re_out << shift),
                spectrometer_im_in.eq(rxiq_cdc.im_out << shift),
                spectrometer_strobe_in.eq(rxiq_cdc.strobe_out),
            ]
        m.d.sync += [
                self.decim_re_out.eq(self.ddc.re_out >> 4),
                self.decim_im_out.eq(self.ddc.im_out >> 4),
                self.decim_strobe_out.eq(self.ddc.strobe_out),                
            ]    
        m.d.comb += [
            self.spectrometer.strobe_in.eq(spectrometer_strobe_in),
            self.spectrometer.common_edge_2x.eq(common_edge_2x.common_edge),
            self.spectrometer.common_edge_3x.eq(common_edge_3x.common_edge),
            self.spectrometer.re_in.eq(spectrometer_re_in),
            self.spectrometer.im_in.eq(spectrometer_im_in),
            sync_spectrometer_interrupt.i.eq(self.spectrometer.interrupt_out),
            self.spectrometer.number_integrations.eq(
                self.sdr_registers['spectrometer']['num_integrations']),
            self.spectrometer.abort.eq(
                self.sdr_registers['spectrometer']['abort']),
            self.spectrometer.peak_detect.eq(
                self.sdr_registers['spectrometer']['peak_detect']),
            self.sdr_registers['spectrometer']['last_buffer'].eq(
                self.spectrometer.last_buffer),
            self.clk_fastlock_out.eq(self.spectrometer.end_fft),
            self.spectrometer.fastlock_profile.eq(self.fastlock_profile_in),
              
              

        ]
        if self.config.Enable_RawFFT:
            m.d.comb += [
                raw_fft.strobe_in.eq(spectrometer_strobe_in), # Keep FFT enabled
                raw_fft.common_edge_2x.eq(common_edge_2x.common_edge),
                
            ]

            # Logic to select source for the raw_fft (ADC or DDC)
            with m.If(self.sdr_registers['spectrometer']['use_ddc_out']):
                # If DDC is used (DDC output is 16-bit, TopFFT expects 12-bit)
                # We truncate the 4 LSBs to fit the 12-bit input of TopFFT
                m.d.comb += [
                    raw_fft.re_in.eq(self.ddc.re_out),
                    raw_fft.im_in.eq(self.ddc.im_out),
                ]
            with m.Else():
                # Use raw ADC samples (already 12-bit)
                m.d.comb += [
                    raw_fft.re_in.eq(spectrometer_re_in >> 4),
                    raw_fft.im_in.eq(spectrometer_im_in >> 4),
                ]

            # Bit-reversal reorder buffer for spectrometer FFT output.
            # The DIF FFT outputs bins in bit-reversed order; this
            # double-buffered memory reorders them to natural order.
            from amaranth.lib.memory import Memory

            spec_fft_order = self.spectrometer.fft_order_log2
            spec_fft_size = 2**spec_fft_order

            # FFT output strobe: 1-cycle delay of spectrometer_strobe_in
            # to align with spectrometer.re_out/im_out/out_last.
            fft_out_strobe = Signal()
            m.d.sync += fft_out_strobe.eq(spectrometer_strobe_in)

            mem_depth = 2 * spec_fft_size
            m.submodules.fft_buf_re = fft_buf_re = Memory(
                shape=16, depth=mem_depth, init=[])
            m.submodules.fft_buf_im = fft_buf_im = Memory(
                shape=16, depth=mem_depth, init=[])

            wr_fft_re = fft_buf_re.write_port()
            wr_fft_im = fft_buf_im.write_port()
            rd_fft_re = fft_buf_re.read_port()
            rd_fft_im = fft_buf_im.read_port()

            fft_wr_bank = Signal()
            fft_frame_avail = Signal()
            fft_wr_ctr = Signal(spec_fft_order)
            fft_rd_ctr = Signal(spec_fft_order)
            fft_reading = Signal()
            fft_rd_valid = Signal()
            fft_last_out = Signal()
            fft_marker = Signal()

            # Bit-reverse the read counter for natural-order output
            fft_rd_rev = Signal(spec_fft_order)
            for i in range(spec_fft_order):
                m.d.comb += fft_rd_rev[i].eq(
                    fft_rd_ctr[spec_fft_order - 1 - i])

            fft_wr_addr = Cat(fft_wr_ctr, fft_wr_bank)
            # XOR LSB of bit-reversed address to apply fftshift
            # (swap first and second halves for DC-centered output)
            fft_rd_addr = Cat(fft_rd_rev ^ 1, ~fft_wr_bank)

            # Write spectrometer FFT output to sequential addresses
            with m.If(fft_out_strobe):
                m.d.sync += fft_wr_ctr.eq(fft_wr_ctr + 1)

            m.d.comb += [
                wr_fft_re.en.eq(fft_out_strobe),
                wr_fft_re.addr.eq(fft_wr_addr),
                wr_fft_re.data.eq(self.spectrometer.re_out >> 2),
                wr_fft_im.en.eq(fft_out_strobe),
                wr_fft_im.addr.eq(fft_wr_addr),
                wr_fft_im.data.eq(self.spectrometer.im_out >> 2),
            ]

            # read_valid tracks reading with 1-strobe delay (memory latency)
            with m.If(fft_out_strobe):
                m.d.sync += fft_rd_valid.eq(fft_reading)

            # Read state machine (gated by fft_out_strobe)
            with m.If(self.spectrometer.out_last & fft_out_strobe):
                m.d.sync += [
                    fft_wr_bank.eq(~fft_wr_bank),
                    fft_frame_avail.eq(1),
                    fft_reading.eq(1),
                    fft_rd_ctr.eq(0),
                    fft_last_out.eq(fft_reading),
                    fft_marker.eq(1),
                ]
            with m.Elif(fft_out_strobe):
                m.d.sync += fft_last_out.eq(0)
                with m.If(fft_reading):
                    m.d.sync += fft_rd_ctr.eq(fft_rd_ctr + 1)
                    with m.If(fft_rd_ctr == spec_fft_size - 1):
                        m.d.sync += fft_reading.eq(0)

            # Read from bit-reversed addresses in opposite bank
            m.d.comb += [
                rd_fft_re.en.eq(1),
                rd_fft_re.addr.eq(fft_rd_addr),
                rd_fft_im.en.eq(1),
                rd_fft_im.addr.eq(fft_rd_addr),
            ]

            # Output
            fft_strobe = (
                (fft_rd_valid | fft_last_out)
                & fft_frame_avail & fft_out_strobe)
            m.d.comb += self.fft_strobe_out.eq(fft_strobe)

            # Replace bin 0 of each frame with 0xFF47 marker.
            # Use fft_rd_valid & ~fft_last_out to target bin 0 of the
            # NEW frame, not the last_out tail of the previous frame.
            fft_marker_active = (
                fft_marker & fft_rd_valid & ~fft_last_out
                & fft_frame_avail & fft_out_strobe)
            with m.If(fft_marker_active):
                m.d.comb += [
                    self.fft_re_out.eq(0x7F47),
                    self.fft_im_out.eq(0x7F47),
                ]
                m.d.sync += fft_marker.eq(0)
            with m.Else():
                m.d.comb += [
                    self.fft_re_out.eq(rd_fft_re.data),
                    self.fft_im_out.eq(rd_fft_im.data),
                ]

        # Recorder
        m.d.comb += [
            # s_axi_lite domain
            self.recorder.mode.eq(
                self.recorder_registers['recorder_control']['mode']),
            self.recorder.start.eq(
                self.recorder_registers['recorder_control']['start']),
            self.recorder.stop.eq(
                self.recorder_registers['recorder_control']['stop']),
            self.recorder_registers['recorder_control']['dropped_samples'].eq(
                self.recorder.dropped_samples),
            (self.recorder_registers['recorder_next_address']
             ['next_address'].eq(self.recorder.next_address)),
            (self.recorder_registers['recorder_committed_address']
             ['committed_address'].eq(self.recorder.committed_address)),
        ]
        # sync domain
        if self.config.recorder_from_ddc and self.config.datv_symsync:
            m.submodules.symsync = symsync = self.symsync
            m.submodules.hdrdet = hdrdet = self.hdrdet
            m.submodules.t2resamp = t2resamp = self.t2resamp
            t2 = self.sdr_registers['datv_symsync']['t2']
            t2_coeff = self.sdr_registers['datv_symsync']['t2_coeff']
            m.d.comb += [
                t2resamp.enable.eq(t2),
                t2resamp.step.eq(self.sdr_registers['datv_omega']['omega']),
                t2resamp.coeff_waddr.eq(
                    self.sdr_registers['ddc_coeff_addr']['coeff_waddr']),
                t2resamp.coeff_wdata.eq(
                    self.sdr_registers['ddc_coeff']['coeff_wdata']),
                t2resamp.coeff_wren.eq(
                    self.sdr_registers['ddc_coeff']['coeff_wren'] & t2_coeff),
                t2resamp.strobe_in.eq(rxiq_cdc.strobe_out),
                t2resamp.re_in.eq(rxiq_cdc.re_out.as_signed()),
                t2resamp.im_in.eq(rxiq_cdc.im_out.as_signed()),
            ]
            m.d.comb += [
                symsync.enable.eq(
                    self.sdr_registers['datv_symsync']['enable']),
                symsync.kp_shift.eq(
                    self.sdr_registers['datv_symsync']['kp_shift']),
                symsync.ki_shift.eq(
                    self.sdr_registers['datv_symsync']['ki_shift']),
                symsync.omega_nom.eq(
                    self.sdr_registers['datv_omega']['omega']),
                symsync.strobe_in.eq(self.ddc.strobe_out),
                symsync.re_in.eq(self.ddc.re_out),
                symsync.im_in.eq(self.ddc.im_out),
                hdrdet.enable.eq(
                    self.sdr_registers['datv_symsync']['enable']
                    & self.sdr_registers['datv_symsync']['hdrdet']),
                hdrdet.strobe_in.eq(symsync.strobe_out),
                hdrdet.re_in.eq(symsync.re_out),
                hdrdet.im_in.eq(symsync.im_out),
            ]
            # The T2 OFDM front end after the resampler (when enabled; else
            # the resampler's samples straight to the ring).
            m.submodules.t2ofdm = t2ofdm = self.t2ofdm
            t2r = self.t2_registers
            m.d.comb += [
                t2ofdm.enable.eq(t2 & t2r['t2_control']['enable']),
                t2ofdm.scheduled.eq(t2r['t2_control']['scheduled']),
                t2ofdm.load.eq(t2r['t2_control']['load']),
                t2ofdm.shift.eq(t2r['t2_control']['shift']),
                t2ofdm.raw_always.eq(t2r['t2_control']['raw_always']),
                t2ofdm.frame_len.eq(t2r['t2_frame_len']['frame_len']),
                t2ofdm.nsym.eq(t2r['t2_layout']['nsym']),
                t2ofdm.gi.eq(t2r['t2_layout']['gi']),
                t2ofdm.early.eq(t2r['t2_layout']['early']),
                t2ofdm.track.eq(t2r['t2_track']['track']),
                t2ofdm.freq.eq(t2r['t2_freq']['freq']),
                t2ofdm.next_start.eq(t2r['t2_next_start']['next_start']),
                t2r['t2_counter']['counter'].eq(t2ofdm.counter),
                t2r['t2_status']['frames'].eq(t2ofdm.frames),
                t2r['t2_status']['overflow'].eq(t2ofdm.overflow),
                t2r['t2_status']['resamp_overflow'].eq(t2resamp.overflow),
                t2ofdm.common_edge_3x.eq(common_edge_3x.common_edge),
                t2ofdm.strobe_in.eq(t2resamp.strobe_out),
                t2ofdm.re_in.eq(t2resamp.re_out),
                t2ofdm.im_in.eq(t2resamp.im_out),
            ]
            with m.If(t2 & t2r['t2_control']['enable']):
                m.d.comb += [
                    self.recorder.strobe_in.eq(t2ofdm.strobe_out),
                    self.recorder.re_in.eq(t2ofdm.re_out),
                    self.recorder.im_in.eq(t2ofdm.im_out),
                ]
            with m.Elif(t2):
                m.d.comb += [
                    self.recorder.strobe_in.eq(t2resamp.strobe_out),
                    self.recorder.re_in.eq(t2resamp.re_out),
                    self.recorder.im_in.eq(t2resamp.im_out),
                ]
            with m.Else():
                m.d.comb += [
                    self.recorder.strobe_in.eq(hdrdet.strobe_out),
                    self.recorder.re_in.eq(hdrdet.re_out),
                    self.recorder.im_in.eq(hdrdet.im_out),
                ]
        elif self.config.recorder_from_ddc:
            m.d.comb += [
                self.recorder.strobe_in.eq(self.ddc.strobe_out),
                self.recorder.re_in.eq(self.ddc.re_out),
                self.recorder.im_in.eq(self.ddc.im_out),
            ]
        else:
            m.d.comb += [
                self.recorder.strobe_in.eq(spectrometer_strobe_in),
                self.recorder.re_in.eq(spectrometer_re_in),
                self.recorder.im_in.eq(spectrometer_im_in),
            ]

        # DDC
        m.d.comb += [
            self.ddc.common_edge.eq(common_edge_3x.common_edge),
            self.ddc.enable_input.eq(
                self.sdr_registers['ddc_control']['enable_input']),
            self.ddc.frequency.eq(
                self.sdr_registers['ddc_frequency']['frequency']),
            self.ddc.coeff_waddr.eq(
                self.sdr_registers['ddc_coeff_addr']['coeff_waddr']),
            self.ddc.coeff_wren.eq(
                self.sdr_registers['ddc_coeff']['coeff_wren']
                & ~(self.sdr_registers['datv_symsync']['t2_coeff']
                    if self.config.datv_symsync else 0)),
            self.ddc.coeff_wdata.eq(
                self.sdr_registers['ddc_coeff']['coeff_wdata']),
            self.ddc.decimation1.eq(
                self.sdr_registers['ddc_decimation']['decimation1']),
            self.ddc.decimation2.eq(
                self.sdr_registers['ddc_decimation']['decimation2']),
            self.ddc.decimation3.eq(
                self.sdr_registers['ddc_decimation']['decimation3']),
            self.ddc.bypass2.eq(
                self.sdr_registers['ddc_control']['bypass2']),
            self.ddc.bypass3.eq(
                self.sdr_registers['ddc_control']['bypass3']),
            self.ddc.operations_minus_one1.eq(
                self.sdr_registers['ddc_control']['operations_minus_one1']),
            self.ddc.operations_minus_one2.eq(
                self.sdr_registers['ddc_control']['operations_minus_one2']),
            self.ddc.operations_minus_one3.eq(
                self.sdr_registers['ddc_control']['operations_minus_one3']),
            self.ddc.odd_operations1.eq(
                self.sdr_registers['ddc_control']['odd_operations1']),
            self.ddc.odd_operations3.eq(
                self.sdr_registers['ddc_control']['odd_operations3']),
            self.ddc.strobe_in.eq(rxiq_cdc.strobe_out),
            self.ddc.re_in.eq(rxiq_cdc.re_out),
            self.ddc.im_in.eq(rxiq_cdc.im_out),
        ]

        # Registers s_axi_lite domain
        # TODO: convert all of this into a RegisterCrossbar module
        address = Signal(self.axi4_awidth, reset_less=True)
        wdata = Signal(32, reset_less=True)
        if self.config.datv_symsync:
            high = self.axi4lite.address[4]
            t2_regs_select = high
        else:
            high = C(0, 1)
        sdr_regs_select = ~high & (self.axi4lite.address[3] == 1)
        recorder_regs_select = (
            ~high & ~sdr_regs_select & (self.axi4lite.address[2] == 1))
        control_regs_select = (
            ~high & ~sdr_regs_select & (self.axi4lite.address[2] == 0))
        m.d.s_axi_lite += [
            self.axi4lite.rdata.eq(self.control_registers.rdata
                                   | self.recorder_registers.rdata
                                   | sdr_registers_cdc.i_rdata
                                   | (t2_registers_cdc.i_rdata
                                      if self.config.datv_symsync else 0)),
            self.axi4lite.rdone.eq(self.control_registers.rdone
                                   | self.recorder_registers.rdone
                                   | sdr_registers_cdc.i_rdone
                                   | (t2_registers_cdc.i_rdone
                                      if self.config.datv_symsync else 0)),
            self.axi4lite.wdone.eq(self.control_registers.wdone
                                   | self.recorder_registers.wdone
                                   | sdr_registers_cdc.i_wdone
                                   | (t2_registers_cdc.i_wdone
                                      if self.config.datv_symsync else 0)),
            self.control_registers.ren.eq(
                self.axi4lite.ren & control_regs_select),
            self.control_registers.wstrobe.eq(
                Mux(control_regs_select, self.axi4lite.wstrobe, 0)),
            self.recorder_registers.ren.eq(
                self.axi4lite.ren & recorder_regs_select),
            self.recorder_registers.wstrobe.eq(
                Mux(recorder_regs_select, self.axi4lite.wstrobe, 0)),
            sdr_registers_cdc.i_ren.eq(
                self.axi4lite.ren & sdr_regs_select),
            sdr_registers_cdc.i_wstrobe.eq(
                Mux(sdr_regs_select, self.axi4lite.wstrobe, 0)),
            address.eq(self.axi4lite.address),
            wdata.eq(self.axi4lite.wdata),
        ]
        if self.config.datv_symsync:
            m.d.s_axi_lite += [
                t2_registers_cdc.i_ren.eq(self.axi4lite.ren & t2_regs_select),
                t2_registers_cdc.i_wstrobe.eq(
                    Mux(t2_regs_select, self.axi4lite.wstrobe, 0)),
            ]
            m.d.comb += [
                t2_registers_cdc.i_address.eq(address),
                t2_registers_cdc.i_wdata.eq(wdata),
                self.t2_registers.ren.eq(t2_registers_cdc.o_ren),
                self.t2_registers.wstrobe.eq(t2_registers_cdc.o_wstrobe),
                self.t2_registers.address.eq(t2_registers_cdc.o_address),
                self.t2_registers.wdata.eq(t2_registers_cdc.o_wdata),
                t2_registers_cdc.o_rdone.eq(self.t2_registers.rdone),
                t2_registers_cdc.o_wdone.eq(self.t2_registers.wdone),
                t2_registers_cdc.o_rdata.eq(self.t2_registers.rdata),
            ]
        m.d.comb += [
            self.control_registers.address.eq(address),
            self.control_registers.wdata.eq(wdata),
            self.recorder_registers.address.eq(address),
            self.recorder_registers.wdata.eq(wdata),
            sdr_registers_cdc.i_address.eq(address),
            sdr_registers_cdc.i_wdata.eq(wdata),
        ]

        # Registers sync domain
        m.d.comb += [
            self.sdr_registers.ren.eq(sdr_registers_cdc.o_ren),
            self.sdr_registers.wstrobe.eq(sdr_registers_cdc.o_wstrobe),
            self.sdr_registers.address.eq(sdr_registers_cdc.o_address),
            self.sdr_registers.wdata.eq(sdr_registers_cdc.o_wdata),
            sdr_registers_cdc.o_rdone.eq(self.sdr_registers.rdone),
            sdr_registers_cdc.o_wdone.eq(self.sdr_registers.wdone),
            sdr_registers_cdc.o_rdata.eq(self.sdr_registers.rdata),
        ]
        # internal resets
        # We use FFSynchronizer rather than ResetSynchronizer because of
        # https://github.com/amaranth-lang/amaranth/issues/721
        for internal in ['sync', 'clk2x', 'clk3x', 'sampling']:
            setattr(m.submodules, f'{internal}_rst', FFSynchronizer(
                self.control_registers['control']['sdr_reset'],
                ResetSignal(internal), o_domain=internal,
                init=1))
        m.d.comb += rxiq_cdc.reset.eq(
            self.control_registers['control']['sdr_reset'])

        m.d.comb += ddciq_cdc.reset.eq(
            self.control_registers['control']['sdr_reset'])    

        # Interrupts (s_axi_lite domain)
        interrupts_reg = self.control_registers['interrupts']
        m.d.comb += [
            self.interrupt_out.eq(interrupts_reg.interrupt),
            interrupts_reg['spectrometer'].eq(sync_spectrometer_interrupt.o),
            interrupts_reg['recorder'].eq(self.recorder.finished),
        ]

        return m


def write_svd(path):
    top = MaiaSDR()
    with open(path, 'wb') as f:
        f.write(top.svd())


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--config', default='default',
        help='Maia SDR configuration name [default=%(default)r]')
    parser.add_argument(
        'output_file', help='Output verilog file')
    return parser.parse_args()


def main():
    args = parse_args()
    config = getattr(configs, args.config)()
    top = MaiaSDR(config)
    platform = PlutoPlatform()
    with open(args.output_file, 'w') as f:
        f.write(amaranth.back.verilog.convert(
            top, platform=platform, ports=top.ports()))


if __name__ == '__main__':
    main()
