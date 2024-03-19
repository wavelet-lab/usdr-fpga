// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
// USDR PROJECT
// CLEAN
//
// Read 0xffffffff for non-valid address
//
// sn_readback_ready is non-standard ready AXI-S signal, it's like data request and SHOULD be asserted first
// not waiting for ready. To be complaint with AXI-S it's required to have 2 streams: request and data, but
// we combined in one for simplicity

module alrd_mux_axis #(
    parameter DATA_BITS     = 2,
    parameter DATA_WIDTH    = 8<<DATA_BITS,
    parameter ADDR_TOTAL    = 16,
    parameter ADDR_WIDTH    = $clog2(ADDR_TOTAL) + DATA_BITS,
    parameter PIPELINE_ADDR = 1,
    parameter PIPELINE_DATA = 1,
    parameter ID_WIDTH      = 1,
    parameter INVALID_DATA  = 32'hffff_ffff
)(
    input                                 clk,
    input                                 rst,

    input  [ADDR_WIDTH - 1:DATA_BITS]     s_al_araddr,
    input                                 s_al_arvalid,
    input  [ID_WIDTH - 1:0]               s_al_arid,
    output                                s_al_arready,

    output [DATA_WIDTH - 1:0]             s_al_rdata,
    output                                s_al_rvalid,
    output [ID_WIDTH - 1:0]               s_al_rid,
    input                                 s_al_rready,

    output [ADDR_TOTAL - 1:0]             sn_readback_ready,
    input  [ADDR_TOTAL - 1:0]             sn_readback_valid,
    input  [DATA_WIDTH*ADDR_TOTAL - 1:0]  sn_readback_data
);

// Stage 0: Address latch
wire [ADDR_WIDTH - 1:DATA_BITS] sel_addr;
wire                            sel_valid;
wire                            sel_ready;
wire [ID_WIDTH - 1:0]           sel_id;

wire                            rb_data_valid = ((sel_addr >= ADDR_TOTAL) ? 1'b1 : sn_readback_valid[sel_addr]);

axis_opt_pipeline #(.WIDTH(ADDR_WIDTH + ID_WIDTH - DATA_BITS), .PIPELINE(PIPELINE_ADDR > 0), .REG_READY(PIPELINE_ADDR > 1)) addr_op (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata({ s_al_arid, s_al_araddr }),
  .s_rx_tvalid(s_al_arvalid),
  .s_rx_tready(s_al_arready),

  .m_tx_tdata({ sel_id, sel_addr }),
  .m_tx_tvalid(sel_valid),
  .m_tx_tready(sel_ready && rb_data_valid)
);

// Stage 1: Data multiplexer
assign sn_readback_ready           = (sel_ready && sel_valid) ? 1 << sel_addr : 0;

wire [DATA_WIDTH - 1:0] rb_data;
genvar i;
generate
for (i = 0; i < DATA_WIDTH; i=i+1) begin: gen
  assign rb_data[i] = (sel_addr >= ADDR_TOTAL) ? INVALID_DATA[i] : sn_readback_data[DATA_WIDTH * sel_addr + i];
end
endgenerate


// Stage 2: Data latch
axis_opt_pipeline #(.WIDTH(DATA_WIDTH + ID_WIDTH), .PIPELINE(PIPELINE_DATA > 0), .REG_READY(PIPELINE_DATA > 1)) data_op (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata({ sel_id, rb_data }),
  .s_rx_tvalid(sel_valid && rb_data_valid),
  .s_rx_tready(sel_ready),

  .m_tx_tdata({ s_al_rid, s_al_rdata }),
  .m_tx_tvalid(s_al_rvalid),
  .m_tx_tready(s_al_rready)
);

endmodule

