// DATV transmit: the DAC DMA stream goes either to the IQ path (upack, as
// always) or, with sel (DAC GPIO bit 1) set, to the DVB-S2 encoder: trxd
// then writes framed BBFRAMEs (0xB8, MODCOD byte, Kbch/8 bytes) into the
// IIO TX buffer instead of IQ samples.
`timescale 1ns / 1ps
module datv_split (
    input  wire        clk,
    input  wire        sel_async,
    input  wire [63:0] s_axis_tdata,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    input  wire        s_axis_tlast,
    output wire [63:0] m_norm_axis_tdata,
    output wire        m_norm_axis_tvalid,
    input  wire        m_norm_axis_tready,
    output wire        m_norm_axis_tlast,
    output wire [63:0] m_datv_axis_tdata,
    output wire [7:0]  m_datv_axis_tkeep,
    output wire        m_datv_axis_tvalid,
    input  wire        m_datv_axis_tready,
    output wire        m_datv_axis_tlast
);
    // sel_async comes from the AD9361 core's DAC GPIO register (CPU clock):
    // two flops into this clock (false path into the first, datv.xdc).
    (* ASYNC_REG = "TRUE" *) reg sel_m = 1'b0;
    (* ASYNC_REG = "TRUE" *) reg sel = 1'b0;
    always @(posedge clk) begin
        sel_m <= sel_async;
        sel   <= sel_m;
    end
    assign m_norm_axis_tdata  = s_axis_tdata;
    assign m_norm_axis_tlast  = s_axis_tlast;
    assign m_norm_axis_tvalid = s_axis_tvalid & ~sel;
    assign m_datv_axis_tdata  = s_axis_tdata;
    assign m_datv_axis_tkeep  = 8'hff;
    assign m_datv_axis_tlast  = s_axis_tlast;
    assign m_datv_axis_tvalid = s_axis_tvalid & sel;
    assign s_axis_tready = sel ? m_datv_axis_tready : m_norm_axis_tready;
endmodule
