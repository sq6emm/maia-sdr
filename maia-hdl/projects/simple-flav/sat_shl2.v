// x4 with saturation on a signed 16-bit sample (tezuka_fw_simple rate64.tcl):
// each ADI x8 interpolation stage has a gain of about 1/4 (its FIR output is
// the top 16 bits of the full-precision sum), so the second stage of the x64
// TX path made the output 12 dB weaker than the x8 path trxd is calibrated
// for. This puts the 12 dB back between the stages.
module sat_shl2 (
    input  wire [15:0] d,
    output wire [15:0] q
);
    wire signed [17:0] x = {{2{d[15]}}, d} <<< 2;
    assign q = (x > 18'sd32767) ? 16'h7FFF : (x < -18'sd32768) ? 16'h8000 : x[15:0];
endmodule
