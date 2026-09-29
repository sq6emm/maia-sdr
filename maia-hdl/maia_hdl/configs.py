#
# Copyright (C) 2024 Daniel Estevez <daniel@destevez.net>
#
# This file is part of maia-sdr
#
# SPDX-License-Identifier: MIT
#

from .config import MaiaSDRConfig


def default():
    """Default Maia SDR configuration"""
    return MaiaSDRConfig()


def maia_iio():
    """Configuration for Maia SDR + IIO"""
    config = MaiaSDRConfig()
    config.spectrometer_address = 0x1600_0000
    config.recorder_address_range = (0x0600_0000, 0x1600_0000)
    config.Enable_RawFFT=False
    return config

def maia_iio_lite():
    """Configuration for Maia SDR + IIO with reduced memory"""
    config = MaiaSDRConfig()
    config.spectrometer_address = 0x1600_0000
    #size only 0x8000000
    config.recorder_address_range = (0x0600_0000, 0xe00_0000)
    config.Enable_RawFFT=False
    return config

def maia_iio_lite_datv():
    """tezuka_fw_simple: the spectrometer on the ADC (wide scope) and the
    DDC as the DATV receive front end, its output recorded into a 1 MiB ring
    next to the spectrometer buffers (both reserved in the device tree)"""
    config = MaiaSDRConfig()
    config.spectrometer_address = 0x1600_0000
    config.recorder_address_range = (0x1610_0000, 0x1620_0000)
    config.recorder_ring = True
    config.recorder_from_ddc = True
    config.datv_symsync = True
    # Tells software (version register bits 31:24) that the recorder is this
    # ring: with any other core, starting the recorder writes outside the
    # reserved memory.
    config.platform = 0xD5
    config.Enable_RawFFT = False
    return config

def maia_iio_lite_s2():
    """tezuka_fw_simple mode bitstream "s2": maia_iio_lite_datv without the
    DVB-T2 front end (DVB-S2 receive front end only)"""
    config = maia_iio_lite_datv()
    config.datv_t2 = False
    return config

def maia_iio_lite_trx():
    """tezuka_fw_simple mode bitstream "trx": the spectrometer (wide scope)
    only. Platform 0xD6: trxd leaves the recorder alone (0xD5 only)."""
    config = MaiaSDRConfig()
    config.spectrometer_address = 0x1600_0000
    config.recorder_address_range = (0x1610_0000, 0x1620_0000)
    config.recorder_ring = True
    config.platform = 0xD6
    config.Enable_RawFFT = False
    return config

def maia_iio_lite_fft():
    """Configuration for Maia SDR + IIO with reduced memory + FFT"""
    config = MaiaSDRConfig()
    config.spectrometer_address = 0x1600_0000
    #size only 0x8000000
    config.recorder_address_range = (0x0600_0000, 0xe00_0000)
    config.Enable_RawFFT =True
    return config
