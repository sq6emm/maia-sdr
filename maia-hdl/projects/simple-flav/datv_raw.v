// DATV transmit, raw IQ mode (DVB-T2 from trxd): with raw (DAC GPIO bit 2,
// CPU clock like this module) set, the DMA words skip the DVB-S2 encoder
// and go to datv_tx as samples, lowest first: two 16-bit I/Q pairs per
// 64-bit word (Q in 31:16, I in 15:0 as the encoder's symbols), or with
// mode8 (DAC GPIO bit 3) four 8-bit pairs (I in the low byte, Q in the
// high byte of each 16 bits), scaled up by 256: half the DMA traffic.
// datv_tx then resamples them to the DAC rate with whatever pulse trxd
// loaded (a low-pass for T2).
`timescale 1ns / 1ps
module datv_raw (
    input  wire        clk,
    input  wire        raw,
    input  wire        mode8,
    // from the input FIFO (DMA words)
    input  wire [63:0] s_dma_axis_tdata,
    input  wire [7:0]  s_dma_axis_tkeep,
    input  wire        s_dma_axis_tlast,
    input  wire        s_dma_axis_tvalid,
    output wire        s_dma_axis_tready,
    // to the encoder
    output wire [63:0] m_enc_axis_tdata,
    output wire [7:0]  m_enc_axis_tkeep,
    output wire        m_enc_axis_tlast,
    output wire        m_enc_axis_tvalid,
    input  wire        m_enc_axis_tready,
    // from the encoder (symbols)
    input  wire [31:0] s_sym_axis_tdata,
    input  wire        s_sym_axis_tlast,
    input  wire        s_sym_axis_tvalid,
    output wire        s_sym_axis_tready,
    // to datv_tx
    output wire [31:0] m_tx_axis_tdata,
    output wire        m_tx_axis_tvalid,
    input  wire        m_tx_axis_tready
);
    reg [1:0] idx = 2'd0;   // raw: which sample of the word is next
    wire last = mode8 ? (idx == 2'd3) : (idx[0] == 1'b1);
    always @(posedge clk) begin
        if (!raw)
            idx <= 2'd0;
        else if (s_dma_axis_tvalid && m_tx_axis_tready)
            idx <= last ? 2'd0 : idx + 2'd1;
    end
    wire [15:0] pair8 = s_dma_axis_tdata[16 * idx +: 16];
    wire [31:0] raw16 = idx[0] ? s_dma_axis_tdata[63:32] : s_dma_axis_tdata[31:0];
    wire [31:0] raw8  = {pair8[15:8], 8'd0, pair8[7:0], 8'd0};
    assign m_enc_axis_tdata  = s_dma_axis_tdata;
    assign m_enc_axis_tkeep  = s_dma_axis_tkeep;
    assign m_enc_axis_tlast  = s_dma_axis_tlast;
    assign m_enc_axis_tvalid = s_dma_axis_tvalid & ~raw;
    assign s_dma_axis_tready = raw ? (m_tx_axis_tready & last) : m_enc_axis_tready;
    assign m_tx_axis_tdata   = raw ? (mode8 ? raw8 : raw16) : s_sym_axis_tdata;
    assign m_tx_axis_tvalid  = raw ? s_dma_axis_tvalid : s_sym_axis_tvalid;
    assign s_sym_axis_tready = raw ? 1'b1 : m_tx_axis_tready;
endmodule
