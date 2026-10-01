#
# Copyright (C) 2022-2023 Daniel Estevez <daniel@destevez.net>
#
# This file is part of maia-sdr
#
# SPDX-License-Identifier: MIT
#

import numpy as np


def clamp_nbits(x, nbits):
    offset = 2**(nbits - 1)
    return ((x + offset) % 2**nbits) - offset


def saturate_nbits(x, nbits):
    """x limited to a signed nbits range (what the saturating HDL does;
    clamp_nbits above wraps, as plain truncation does)"""
    top = 2**(nbits - 1)
    return np.clip(x, -top, top - 1) if isinstance(x, np.ndarray) \
        else max(-top, min(top - 1, x))


def saturate(v, nbits):
    """Amaranth: the signed value v limited to nbits (signed(nbits)):
    overflow when the bits above the top one are not all copies of the sign"""
    from amaranth import Mux, Const, signed
    v = v.as_signed()
    if len(v) <= nbits:
        return v
    upper = v[nbits - 1:]
    ok = (upper == 0) | (upper == (1 << len(upper)) - 1)
    top = Const(2**(nbits - 1) - 1, signed(nbits))
    bot = Const(-2**(nbits - 1), signed(nbits))
    return Mux(ok, v[:nbits].as_signed(), Mux(v[-1], bot, top))


def bit_invert(n, nbits, radix_log2):
    bits = ('0'*nbits + bin(n)[2:])[-nbits:]
    bits_arr = np.array([a for a in bits])
    inverted = bits_arr.reshape(-1, radix_log2)[::-1].ravel()
    inverted_str = ''.join(list(inverted))
    return int(inverted_str, 2)
