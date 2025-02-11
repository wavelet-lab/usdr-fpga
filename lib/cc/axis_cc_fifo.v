// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
// RX_VALID_PIPELINE_S will break AXI-S rule: valid should not wait for ready
// use this parameter with caution and only to optmize timing in hot places!
module axis_cc_fifo #(
    parameter         WIDTH = 32,
    parameter         DEEP = 32,
    parameter         DEEP_BITS = $clog2(DEEP),
    parameter         EXTRA_REG = 0,
    parameter         ASYNC_RESET = 0,
    parameter         ULTRA_SCALE = 0,
    parameter         DATA_PIPELINE = 1,
    parameter [0:0]   RX_READY_PIPELINE = 1,
    parameter [0:0]   RX_VALID_PIPELINE_S = 0
) (

  input                            rx_clk,
  input                            rx_rst,

  input [WIDTH-1:0]                s_rx_tdata,
  input                            s_rx_tvalid,
  output                           s_rx_tready,


  input                            tx_clk,
  input                            tx_rst,

  output [WIDTH-1:0]               m_tx_tdata,
  output                           m_tx_tvalid,
  input                            m_tx_tready
);

// All wires are prefixed with either rclk_ or tclk_ coresponding rx or tx clock domain
wire               tclk_valid;
wire               tclk_ready;
wire [WIDTH-1:0]   tclk_data;

wire               rclk_addr_inc = s_rx_tvalid && s_rx_tready;
wire [DEEP_BITS:0] rclk_rx_addr;
wire [DEEP_BITS:0] tclk_rx_addr;

wire               tclk_addr_inc = tclk_valid && tclk_ready;
wire [DEEP_BITS:0] tclk_tx_addr;
wire [DEEP_BITS:0] rclk_tx_addr;

cc_counter #(
   .WIDTH(DEEP_BITS + 1),
   .ASYNC_RESET(ASYNC_RESET)
) rx_addr_cc (
   .in_clk(rx_clk),
   .in_rst(rx_rst),
   .in_increment(rclk_addr_inc),
   .in_counter(rclk_rx_addr),

   .out_clk(tx_clk),
   .out_rst(tx_rst),
   .out_counter(tclk_rx_addr)
);

cc_counter #(
   .WIDTH(DEEP_BITS + 1),
   .ASYNC_RESET(ASYNC_RESET)
) tx_addr_cc (
   .in_clk(tx_clk),
   .in_rst(tx_rst),
   .in_increment(tclk_addr_inc),
   .in_counter(tclk_tx_addr),

   .out_clk(rx_clk),
   .out_rst(rx_rst),
   .out_counter(rclk_tx_addr)
);

ram_sxp #(.DATA_WIDTH(WIDTH), .ADDR_WIDTH(DEEP_BITS), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b1)) sdp_ram (
    .wclk(rx_clk),
    .we(rclk_addr_inc),
    .waddr(rclk_rx_addr[DEEP_BITS-1:0]),
    .wdata(s_rx_tdata),

    .raddr(tclk_tx_addr[DEEP_BITS-1:0]),
    .rdata(tclk_data)
);

axis_opt_pipeline #(.WIDTH(WIDTH), .PIPELINE(DATA_PIPELINE)) out (
  .clk(tx_clk),
  .rst(tx_rst),

  .s_rx_tdata(tclk_data),
  .s_rx_tvalid(tclk_valid),
  .s_rx_tready(tclk_ready),

  .m_tx_tdata(m_tx_tdata),
  .m_tx_tvalid(m_tx_tvalid),
  .m_tx_tready(m_tx_tready)
);

wire [DEEP_BITS:0] rclk_fifo_used   = rclk_rx_addr - rclk_tx_addr + RX_READY_PIPELINE;
wire [DEEP_BITS:0] tclk_fifo_used_n = tclk_rx_addr - tclk_tx_addr - 1'b1 - RX_VALID_PIPELINE_S;

generate
if (RX_READY_PIPELINE) begin
  reg rx_tready_p;
  always @(posedge rx_clk) begin
    if (rx_rst) begin
      rx_tready_p <= 1'b0;
    end else begin
      rx_tready_p <= ~rclk_fifo_used[DEEP_BITS];
    end
  end
  assign    s_rx_tready   = rx_tready_p;
end else begin
  assign    s_rx_tready   = ~rclk_fifo_used[DEEP_BITS];
end
endgenerate


generate
if (RX_VALID_PIPELINE_S) begin
  reg tclk_valid_p;
  always @(posedge tx_clk) begin
    if (tx_rst) begin
      tclk_valid_p <= 1'b0;
    end else begin
      tclk_valid_p <= ~tclk_fifo_used_n[DEEP_BITS];
    end
  end
  assign      tclk_valid = tclk_valid_p;
end else begin
  assign      tclk_valid = ~tclk_fifo_used_n[DEEP_BITS];
end
endgenerate

endmodule
