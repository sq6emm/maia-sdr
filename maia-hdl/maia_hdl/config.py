#
# Copyright (C) 2024 Daniel Estevez <daniel@destevez.net>
#
# This file is part of maia-sdr
#
# SPDX-License-Identifier: MIT
#

class MaiaSDRConfig:
    """Maia SDR configuration

    This class defines configuration parameters for the Maia SDR top-level.
    """
    def __init__(self):
        # create default configuration

        # general
        self.platform = 0

        # spectrometer
        self.spectrometer_address = 0x1a00_0000
        self.spectrometer_buffers = 8

        # IQ recorder
        self.recorder_address_range = (0x0100_0000, 0x1a00_0000)
        # Ring buffer instead of one-shot recording (see DmaStreamWrite).
        self.recorder_ring = False
        # Record the DDC output always, not whatever the spectrometer sees.
        self.recorder_from_ddc = False
        # DATV: symbol timing recovery between the DDC and the recorder
        # (symsync.py; registers sdr 0b110 / 0b111).
        self.datv_symsync = False
        # tezuka_fw_simple wide trx image: a second spectrometer input (the
        # first x8 decimation stage, 3.072 MS/s at 24.576), selected by the
        # spectrometer register's use_zoom bit: 750 Hz bins for the views
        # between the ARM's stream and the whole band.
        self.spectrometer_zoom = False
        # The ADC FIFO's valid as the input strobe without the DATV front end
        # (the trx core): one sample per valid, none repeated or lost.
        self.valid_in = False
        # The DDC and the DVB-T2 resampler take the zoom input (the first
        # decimation stage, 3.072 MS/s with its own valid) instead of the
        # ADC samples; needs spectrometer_zoom.
        self.datv_from_zoom = False
        # With datv_symsync: the DVB-T2 front end too (resampler, OFDM,
        # equalizer; register window 0x40..). False: DVB-S2 only.
        self.datv_t2 = True
        # With datv_symsync: the DVB-S2 receive front end (symbol timing
        # recovery, header detector). False with datv_t2: the DVB-T2 mode
        # bitstream (tezuka_fw_simple docs/FPGA-MODES.md "t2").
        self.datv_s2 = True
        # DVB-S2 known-symbol accumulator (s2trk.py) in the register window
        # T2 uses: only without the T2 front end (the s2 mode bitstream).
        self.datv_s2trk = True
        self.Enable_RawFFT = False


    def validate(self):
        assert self.platform >= 0 and self.platform < 256
        assert self.spectrometer_buffers > 0
        assert self.spectrometer_buffers.bit_count() == 1
        assert self.recorder_address_range[0] < self.recorder_address_range[1]
        # TODO: check that spectrometer and recorder buffers do not overlap
