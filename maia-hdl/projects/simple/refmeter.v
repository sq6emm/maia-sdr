// refmeter: measures the board's reference oscillator (the AD936x's 40 MHz
// XO/VCTCXO, fed to the fabric) against a GPS 1PPS, or against software time.
//
// A free-running 64-bit counter runs on ref_clk. Two ways to read it:
//
//  * PPS capture: on every rising PPS edge the counter value and the count
//    since the previous edge (the reference frequency in Hz, to +-1 count)
//    are latched, and pps_seq increments. Software averages the periods.
//  * Snapshot: software writes CONTROL[0]=1; the counter is captured a few
//    ref_clk cycles later and snap_seq increments. Pairing snapshots with
//    chrony-disciplined system time gives the frequency without a PPS wire.
//
// Register map (AXI4-Lite, 32-bit words):
//   0x00 ID          RO  0x5246_4D31 ("RFM1")
//   0x04 CONTROL     WO  [0] write 1: take a snapshot
//   0x08 SNAP_LO     RO  snapshot counter [31:0]
//   0x0C SNAP_HI     RO  snapshot counter [63:32]
//   0x10 SNAP_SEQ    RO  increments when SNAP_LO/HI are updated
//   0x14 PPS_LO      RO  counter at the last PPS edge [31:0]
//   0x18 PPS_HI      RO  counter at the last PPS edge [63:32]
//   0x1C PPS_SEQ     RO  PPS edges seen
//   0x20 PPS_PERIOD  RO  ref_clk cycles between the last two PPS edges
//   0x24 STATUS      RO  [0] PPS present (edge within the last ~2 s)
//
// Clock crossing: both directions use a toggle handshake, and the payload
// registers are only ever sampled after the toggle has crossed, by which time
// they have been stable for several cycles (PPS values for a whole second).

`timescale 1ns/1ps

module refmeter #(
  parameter integer REF_HZ = 40_000_000
) (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ref_clk CLK" *)
  (* X_INTERFACE_PARAMETER = "FREQ_HZ 40000000" *)
  input  wire        ref_clk,
  input  wire        pps,

  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn" *)
  input  wire        s_axi_aclk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  wire        s_axi_aresetn,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi AWADDR" *)
  input  wire [7:0]  s_axi_awaddr,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi AWVALID" *)
  input  wire        s_axi_awvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi AWREADY" *)
  output reg         s_axi_awready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WDATA" *)
  input  wire [31:0] s_axi_wdata,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WSTRB" *)
  input  wire [3:0]  s_axi_wstrb,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WVALID" *)
  input  wire        s_axi_wvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi WREADY" *)
  output reg         s_axi_wready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi BRESP" *)
  output wire [1:0]  s_axi_bresp,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi BVALID" *)
  output reg         s_axi_bvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi BREADY" *)
  input  wire        s_axi_bready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi ARADDR" *)
  input  wire [7:0]  s_axi_araddr,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi ARVALID" *)
  input  wire        s_axi_arvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi ARREADY" *)
  output reg         s_axi_arready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RDATA" *)
  output reg  [31:0] s_axi_rdata,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RRESP" *)
  output wire [1:0]  s_axi_rresp,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RVALID" *)
  output reg         s_axi_rvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi RREADY" *)
  input  wire        s_axi_rready
);

  assign s_axi_bresp = 2'b00;
  assign s_axi_rresp = 2'b00;

  // ---------------------------------------------------------- ref_clk domain
  reg  [63:0] cnt = 64'd0;
  (* ASYNC_REG = "TRUE" *) reg [2:0] pps_sync = 3'b000;
  reg  [63:0] pps_cnt = 64'd0;
  reg  [31:0] pps_period = 32'd0;
  reg  [31:0] pps_seq_r = 32'd0;
  reg         pps_tog = 1'b0;
  reg  [31:0] pps_age = 32'hFFFF_FFFF;
  reg         pps_present_r = 1'b0;

  reg         snap_req = 1'b0;   // s_axi_aclk domain, toggled per request
  (* ASYNC_REG = "TRUE" *) reg [2:0] snap_req_sync = 3'b000;
  reg  [63:0] snap_cnt = 64'd0;
  reg         snap_ack = 1'b0;

  always @(posedge ref_clk) begin
    cnt <= cnt + 64'd1;

    pps_sync <= {pps_sync[1:0], pps};
    if (pps_sync[2:1] == 2'b01) begin
      pps_period <= cnt[31:0] - pps_cnt[31:0];
      pps_cnt    <= cnt;
      pps_seq_r  <= pps_seq_r + 32'd1;
      pps_tog    <= ~pps_tog;
      pps_age    <= 32'd0;
    end else if (pps_age != 32'hFFFF_FFFF) begin
      pps_age <= pps_age + 32'd1;
    end
    pps_present_r <= pps_age < 2 * REF_HZ;

    snap_req_sync <= {snap_req_sync[1:0], snap_req};
    if (snap_req_sync[2] != snap_req_sync[1]) begin
      snap_cnt <= cnt;
      snap_ack <= ~snap_ack;
    end
  end

  // ------------------------------------------------------- s_axi_aclk domain
  (* ASYNC_REG = "TRUE" *) reg [2:0] snap_ack_sync = 3'b000;
  (* ASYNC_REG = "TRUE" *) reg [2:0] pps_tog_sync = 3'b000;
  (* ASYNC_REG = "TRUE" *) reg [1:0] present_sync = 2'b00;
  reg  [63:0] snap_axi = 64'd0;
  reg  [31:0] snap_seq = 32'd0;
  reg  [63:0] pps_cnt_axi = 64'd0;
  reg  [31:0] pps_period_axi = 32'd0;
  reg  [31:0] pps_seq_axi = 32'd0;

  always @(posedge s_axi_aclk) begin
    snap_ack_sync <= {snap_ack_sync[1:0], snap_ack};
    pps_tog_sync  <= {pps_tog_sync[1:0], pps_tog};
    present_sync  <= {present_sync[0], pps_present_r};
    if (snap_ack_sync[2] != snap_ack_sync[1]) begin
      snap_axi <= snap_cnt;
      snap_seq <= snap_seq + 32'd1;
    end
    if (pps_tog_sync[2] != pps_tog_sync[1]) begin
      pps_cnt_axi    <= pps_cnt;
      pps_period_axi <= pps_period;
      pps_seq_axi    <= pps_seq_r;
    end
  end

  // AXI4-Lite write: accept address and data together, one beat at a time.
  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
      s_axi_awready <= 1'b0;
      s_axi_wready  <= 1'b0;
      s_axi_bvalid  <= 1'b0;
    end else begin
      s_axi_awready <= 1'b0;
      s_axi_wready  <= 1'b0;
      if (s_axi_awvalid && s_axi_wvalid && !s_axi_bvalid && !s_axi_awready) begin
        s_axi_awready <= 1'b1;
        s_axi_wready  <= 1'b1;
        s_axi_bvalid  <= 1'b1;
        if (s_axi_awaddr[7:2] == 6'h01 && s_axi_wstrb[0] && s_axi_wdata[0])
          snap_req <= ~snap_req;
      end else if (s_axi_bvalid && s_axi_bready) begin
        s_axi_bvalid <= 1'b0;
      end
    end
  end

  // AXI4-Lite read.
  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
      s_axi_arready <= 1'b0;
      s_axi_rvalid  <= 1'b0;
      s_axi_rdata   <= 32'd0;
    end else begin
      s_axi_arready <= 1'b0;
      if (s_axi_arvalid && !s_axi_rvalid && !s_axi_arready) begin
        s_axi_arready <= 1'b1;
        s_axi_rvalid  <= 1'b1;
        case (s_axi_araddr[7:2])
          6'h00: s_axi_rdata <= 32'h5246_4D31;
          6'h02: s_axi_rdata <= snap_axi[31:0];
          6'h03: s_axi_rdata <= snap_axi[63:32];
          6'h04: s_axi_rdata <= snap_seq;
          6'h05: s_axi_rdata <= pps_cnt_axi[31:0];
          6'h06: s_axi_rdata <= pps_cnt_axi[63:32];
          6'h07: s_axi_rdata <= pps_seq_axi;
          6'h08: s_axi_rdata <= pps_period_axi;
          6'h09: s_axi_rdata <= {31'd0, present_sync[1]};
          default: s_axi_rdata <= 32'd0;
        endcase
      end else if (s_axi_rvalid && s_axi_rready) begin
        s_axi_rvalid <= 1'b0;
      end
    end
  end

endmodule
