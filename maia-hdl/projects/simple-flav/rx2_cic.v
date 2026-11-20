// RX2 x8 decimator without multipliers, for the Pluto trx bitstream
// (xc7z010: rxfir.tcl's FIR has taken every DSP slice). Four-stage CIC
// (R=8, M=1) on RX2 I/Q in the ADC sample-clock domain, so that in 2R2T
// trxd's noise canceller gets RX2 band-limited and at the same rate as RX1
// (rx2cic.tcl). Before this RX2 went to the packer raw on RX1's decimated
// strobe (aliased).
//
// Matched to rxfir.tcl's FIR where the canceller cares:
//  - gain: that FIR has a DC gain of 2^17 and keeps the top 16 of 34 bits
//    (x0.5); the CIC's 8^4 = 2^12 shifted down 13 is x0.5 as well;
//  - delay: the FIR's group delay is 64 input samples, the CIC's 14 plus 3
//    of integrator pipeline: RX2 goes through a 47-sample delay line first,
//    so the residual offset is well inside the canceller's taps.
// The CIC droop (~ -1.5 dB at the 0.384 MS/s output's band edge) is a linear
// filter on the reference, which the adaptive weights take up.
//
// The output is held between decimated samples; the packer reads it on
// RX1's FIR output strobe (the two count the same input valids). With the
// decimator bypassed (active low) RX2 passes straight through, as RX1 does.

`default_nettype none

module rx2_cic (
    input  wire        clk,
    input  wire        active,   // async (AXI domain): synchronised here
    input  wire        valid,    // RX1's FIR input valid (ADC FIFO)
    input  wire [15:0] in_i,
    input  wire [15:0] in_q,
    output wire [15:0] out_i,
    output wire [15:0] out_q
);
    localparam D = 47;

    (* ASYNC_REG = "TRUE" *) reg [1:0] act_s = 2'b00;
    always @(posedge clk) act_s <= {act_s[0], active};
    wire act = act_s[1];

    // delay line (SRLs: no reset, zero INIT)
    reg [31:0] dl [0:D-1];
    integer k;
    initial for (k = 0; k < D; k = k + 1) dl[k] = 32'd0;
    always @(posedge clk)
        if (valid) begin
            dl[0] <= {in_q, in_i};
            for (k = 1; k < D; k = k + 1)
                dl[k] <= dl[k-1];
        end
    wire [31:0] d = dl[D-1];

    reg [2:0] cnt = 3'd0;
    always @(posedge clk)
        if (valid) cnt <= cnt + 3'd1;
    wire tick = valid && cnt == 3'd7;

    wire [15:0] yi, yq;
    rx2_cic4 ci (.clk(clk), .valid(valid), .tick(tick), .x(d[15:0]),  .y(yi));
    rx2_cic4 cq (.clk(clk), .valid(valid), .tick(tick), .x(d[31:16]), .y(yq));

    assign out_i = act ? yi : in_i;
    assign out_q = act ? yq : in_q;
endmodule

module rx2_cic4 (
    input  wire        clk,
    input  wire        valid,
    input  wire        tick,
    input  wire [15:0] x,
    output reg  [15:0] y = 16'd0
);
    // |x| * 8^4 <= 2^27: 28 bits (modulo arithmetic is exact in a CIC)
    reg signed [27:0] i1 = 0, i2 = 0, i3 = 0, i4 = 0;
    reg signed [27:0] d1 = 0, d2 = 0, d3 = 0, d4 = 0;
    wire signed [27:0] xs = {{12{x[15]}}, x};
    always @(posedge clk)
        if (valid) begin
            i1 <= i1 + xs;
            i2 <= i2 + i1;
            i3 <= i3 + i2;
            i4 <= i4 + i3;
        end

    wire signed [27:0] c1 = i4 - d1;
    wire signed [27:0] c2 = c1 - d2;
    wire signed [27:0] c3 = c2 - d3;
    wire signed [27:0] c4 = c3 - d4;
    // x0.5: drop 13 bits, rounded (29 bits: +2^12 must not wrap)
    wire signed [28:0] r = {c4[27], c4} + 29'sd4096;
    always @(posedge clk)
        if (tick) begin
            d1 <= i4;
            d2 <= c1;
            d3 <= c2;
            d4 <= c3;
            y  <= r[28:13];
        end
endmodule

`default_nettype wire
