// In place of the DVB-S2 encoder (dvbs2_encoder_wrapper) in the DVB-T2 mode
// bitstream (tezuka_fw_simple docs/FPGA-MODES.md "t2"): there DVB-T2 sends
// raw words (DAC GPIO bit 2) that never reach the encoder; whatever would
// is taken and dropped, and no symbol ever comes out.
module datv_noenc (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [63:0] s_axis_tdata,
    input  wire [7:0]  s_axis_tkeep,
    input  wire        s_axis_tlast,
    input  wire        s_axis_tvalid,
    output wire        s_axis_tready,
    output wire [31:0] m_axis_tdata,
    output wire        m_axis_tlast,
    output wire        m_axis_tvalid,
    input  wire        m_axis_tready
);
    assign s_axis_tready = 1'b1;
    assign m_axis_tdata  = 32'd0;
    assign m_axis_tlast  = 1'b0;
    assign m_axis_tvalid = 1'b0;
endmodule
