// Bit handling around the ADI x8 FIR stages of the x64 path (rate64.tcl).
//
// Each ADI x8 stage's FIR output is the top 16 bits of its full-precision
// sum, which leaves a gain of about 1/4 (TX, interpolation) or 1/2 (RX,
// decimation). rate64.tcl sets the two stages next to the 3.072 MS/s point
// to 18 output bits (the same MSBs, two more LSBs) and these modules make
// the 16-bit samples out of them.

// RX, first decimation stage (24.576 -> 3.072 MS/s): y = 4x the old output.
//   q    : the old 16-bit output (rounded), for the second stage (the stream)
//   q2x  : twice it, saturated: the converter's own level, with one more bit
//          of precision than the old output had: the 3.072 MS/s input of
//          Maia's zoom spectrometer and DATV receivers.
module rate64_rx_bits (
    input  wire [23:0] y,     // FIR m_axis_data_tdata (18 bits, sign-extended)
    output wire [15:0] q,
    output wire [15:0] q2x
);
    wire signed [17:0] s = y[17:0];
    wire signed [18:0] r4 = s + 19'sd2;          // round half up, >> 2
    wire signed [16:0] q_full = r4[18:2];
    assign q = (q_full > 17'sd32767) ? 16'h7FFF :
               (q_full < -17'sd32768) ? 16'h8000 : q_full[15:0];
    wire signed [18:0] r2 = s + 19'sd1;          // round half up, >> 1
    wire signed [17:0] h = r2[18:1];
    assign q2x = (h > 18'sd32767) ? 16'h7FFF :
                 (h < -18'sd32768) ? 16'h8000 : h[15:0];
endmodule

// TX, DAC-side interpolation stage (3.072 -> 24.576 MS/s): y = 4x the old
// output; saturated to 16 bits it is the unity-gain x8 interpolator, so a
// full-scale 3.072 MS/s sample (DATV) reaches the DAC at full scale, and the
// IQ path's 1/4 is the pre-stage's alone (the level trxd is calibrated for,
// as with the x8 images).
module rate64_tx_bits (
    input  wire [23:0] y,
    output wire [15:0] q
);
    wire signed [17:0] s = y[17:0];
    assign q = (s > 18'sd32767) ? 16'h7FFF :
               (s < -18'sd32768) ? 16'h8000 : s[15:0];
endmodule
