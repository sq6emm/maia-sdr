// DATV transmit: with sel (DAC GPIO bit 1) set, the samples of the DVB-S2
// interpolator (AXI-Stream, I in 31:16, Q in 15:0, already at the DAC rate)
// replace the x8 interpolator's output, one per DAC request (req, the DAC
// FIFO's din_valid). An empty FIFO sends zeros and counts an underflow.
`timescale 1ns / 1ps
module datv_merge (
    input  wire        clk,
    input  wire        sel_async,
    input  wire        req,
    input  wire [15:0] norm_i,
    input  wire [15:0] norm_q,
    input  wire        norm_valid,
    input  wire [31:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    output wire [15:0] out_i,
    output wire [15:0] out_q,
    output wire        out_valid,
    output reg  [15:0] underflows = 16'd0
);
    // sel_async comes from the AD9361 core's DAC GPIO register (CPU clock):
    // two flops into this clock (false path into the first, datv.xdc).
    (* ASYNC_REG = "TRUE" *) reg sel_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg sel = 1'b0;
    always @(posedge clk) begin
        sel_m <= sel_async;
        sel   <= sel_m;
    end
    reg [15:0] di = 16'd0, dq = 16'd0;
    assign s_axis_tready = sel & req;
    always @(posedge clk) begin
        if (sel & req) begin
            if (s_axis_tvalid) begin
                di <= s_axis_tdata[31:16];
                dq <= s_axis_tdata[15:0];
            end else begin
                di <= 16'd0;
                dq <= 16'd0;
                underflows <= underflows + 16'd1;
            end
        end
        if (~sel)
            underflows <= 16'd0;
    end
    assign out_i     = sel ? di : norm_i;
    assign out_q     = sel ? dq : norm_q;
    assign out_valid = sel ? 1'b1 : norm_valid;
endmodule
