// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module alrdwr_to_bram #(
    parameter ADDR_WIDTH = 12,
    parameter DATA_BITS = 2,
    parameter DATA_WIDTH = 8<<DATA_BITS,
    parameter BRAM_STAGES = 1,
    parameter ID_WIDTH = 1,
    parameter RD_PRIO = 1
)(
    input clk,
    input rst,

    // AL Write address channel
    input [ADDR_WIDTH - 1:DATA_BITS]  s_al_waddr,
    input                             s_al_wvalid,
    input [DATA_WIDTH-1:0]            s_al_wdata,
    output                            s_al_wready,

    // AL Read address channel
    input [ADDR_WIDTH - 1:DATA_BITS]  s_al_araddr,
    input                             s_al_arvalid,
    input [ID_WIDTH-1:0]              s_al_arid,
    output                            s_al_arready,

    // AL Read data channel signals
    output [DATA_WIDTH-1:0]           s_al_rdata,
    output                            s_al_rvalid,
    output [ID_WIDTH-1:0]             s_al_rid,
    input                             s_al_rready,

    // BRAM interface
    output [ADDR_WIDTH - 1:DATA_BITS] bram_addr,
    output                            bram_en,
    output                            bram_we,
    output                            bram_le,
    output [DATA_WIDTH-1:0]           bram_data_wr,
    input  [DATA_WIDTH-1:0]           bram_data_rd
);

wire read_ready_flag;
wire stall          = s_al_rvalid && !s_al_rready; //Data in pipeline

wire read_pending   = s_al_arvalid;
wire write_pending  = s_al_wvalid;

wire read_op        = (read_pending && write_pending) ? RD_PRIO : !write_pending;

wire read_valid     = s_al_arvalid && read_op;

assign s_al_arready = !stall && read_op;
assign s_al_wready  = !stall && !read_op;

assign bram_addr = (read_op) ? s_al_araddr : s_al_waddr;
assign bram_en   = s_al_arvalid && s_al_arready || s_al_wvalid && s_al_wready;
assign bram_we   = s_al_wvalid && s_al_wready;

assign bram_data_wr = s_al_wdata;

assign s_al_rdata      = bram_data_rd;



axis_pipeline_mover #(.WIDTH(ID_WIDTH), .DEEP(BRAM_STAGES)) axis_pipeline_mover (
  .clk(clk),
  .rst(rst),

  .s_in_valid(read_valid),
  .s_in_ready(read_ready_flag),
  .s_in_data(s_al_arid),

  .m_out_valid(s_al_rvalid),
  .m_out_ready(s_al_rready),
  .m_out_data(s_al_rid),

  .move_valid(bram_le)
);


endmodule
