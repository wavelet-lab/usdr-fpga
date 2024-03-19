// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
// USDR PROJECT
// CLEAN
//
// Port with lower index has higher priority. If none of port masks matches the address but arvalid is active
// MASTER_COUNT-1 port is selected to prevent deadlock

module alrdwr_demux_addr #(
    parameter DATA_BITS    = 2,
    parameter DATA_WIDTH   = 8 << DATA_BITS,
    parameter ADDR_WIDTH   = 4,
    parameter ID_WIDTH     = 1,
    parameter MASTER_COUNT = 1,
    parameter MASTER_COUNT_BITS = $clog2(MASTER_COUNT),
    parameter ADDR_ALIGN   = 32,
    parameter PORT_R_PIPELINE = 0,
    parameter [32*32 - 1 :0] ADDR_MASK = 0,
    parameter [32*32 - 1 :0] ADDR_COMP = 0
)(
    // AL clocks
    input                     clk,
    input                     rst,

    // AL Write address/data
    input  [ADDR_WIDTH - 1:DATA_BITS] s_al_waddr,
    input  [DATA_WIDTH - 1:0]         s_al_wdata,
    input                             s_al_wvalid,
    input  [ID_WIDTH - 1:0]           s_al_wid,
    output                            s_al_wready,

    // AL Read address
    input  [ADDR_WIDTH - 1:DATA_BITS] s_al_araddr,
    input                             s_al_arvalid,
    input  [ID_WIDTH - 1:0]           s_al_arid,
    output                            s_al_arready,
    // AL RB data
    output [DATA_WIDTH - 1:0]         s_al_rdata,
    output                            s_al_rvalid,
    output [ID_WIDTH - 1:0]           s_al_rid,
    input                             s_al_rready,

    // AL write address / data channel 0..n
    output [MASTER_COUNT * (ADDR_WIDTH - DATA_BITS) - 1:0]  mn_al_waddr,
    output [MASTER_COUNT * DATA_WIDTH - 1:0]                mn_al_wdata,
    output [MASTER_COUNT - 1:0]                             mn_al_wvalid,
    output [MASTER_COUNT * ID_WIDTH - 1:0]                  mn_al_wid,
    input  [MASTER_COUNT - 1:0]                             mn_al_wready,

    // AL Read address channel 0..n
    output [MASTER_COUNT * (ADDR_WIDTH - DATA_BITS) - 1:0]  mn_al_araddr,
    output [MASTER_COUNT - 1:0]                             mn_al_arvalid,
    output [MASTER_COUNT * ID_WIDTH - 1:0]                  mn_al_arid,
    input  [MASTER_COUNT - 1:0]                             mn_al_arready,

    // AL Write data channel 0..n
    input  [MASTER_COUNT * DATA_WIDTH - 1:0]                mn_al_rdata,
    input  [MASTER_COUNT - 1:0]                             mn_al_rvalid,
    input  [MASTER_COUNT * ID_WIDTH - 1:0]                  mn_al_rid,
    output [MASTER_COUNT - 1:0]                             mn_al_rready
);

// RD addr selector
wire [MASTER_COUNT_BITS - 1:0] mask_rd_sel_ridx;
address_selector #(
    .ADDR_WIDTH(ADDR_WIDTH), .ADDR_ALIGN(ADDR_ALIGN), .ADDR_MASK(ADDR_MASK), .ADDR_COMP(ADDR_COMP), .COUNT(MASTER_COUNT)
) al_araddr_sel (
    .addr({s_al_araddr, {DATA_BITS{1'b0}}}),
    .selector(mask_rd_sel_ridx),
    .nmatch()
);
assign s_al_arready  = mn_al_arready[mask_rd_sel_ridx];
assign mn_al_arvalid = {MASTER_COUNT{s_al_arvalid}} & (1 << mask_rd_sel_ridx);
assign mn_al_araddr  = {MASTER_COUNT{s_al_araddr}};
assign mn_al_arid    = {MASTER_COUNT{s_al_arid}};

// WR addr selector
wire [MASTER_COUNT_BITS - 1:0] mask_wr_sel_ridx;
address_selector #(
    .ADDR_WIDTH(ADDR_WIDTH), .ADDR_ALIGN(ADDR_ALIGN), .ADDR_MASK(ADDR_MASK), .ADDR_COMP(ADDR_COMP), .COUNT(MASTER_COUNT)
) al_waddr_sel (
    .addr({mn_al_waddr, {DATA_BITS{1'b0}}}),
    .selector(mask_wr_sel_ridx),
    .nmatch()
);
assign s_al_wready   = mn_al_wready[mask_wr_sel_ridx];
assign mn_al_wvalid  = {MASTER_COUNT{s_al_wvalid}} & (1 << mask_wr_sel_ridx);
assign mn_al_waddr   = {MASTER_COUNT{s_al_waddr}};
assign mn_al_wdata   = {MASTER_COUNT{s_al_wdata}};
assign mn_al_wid     = {MASTER_COUNT{s_al_wid}};

// RD readback selector
wire [MASTER_COUNT_BITS - 1:0] mn_al_ridx;
wire                           mn_al_ridx_nv;

ctz_clz #(.BITS(MASTER_COUNT)) clz_arddr_i (.data_i(mn_al_rvalid), .num_o(mn_al_ridx), .zero_o(mn_al_ridx_nv));

wire                    p_al_rvalid  = !mn_al_ridx_nv;
wire [DATA_WIDTH - 1:0] p_al_rdata   = mn_al_rdata >> (mn_al_ridx * DATA_WIDTH);
wire [ID_WIDTH - 1:0]   p_al_rid     = mn_al_rid   >> (mn_al_ridx * ID_WIDTH);
wire                    p_al_rready;

assign mn_al_rready = p_al_rready << mn_al_ridx;

axis_opt_pipeline #(.WIDTH(DATA_WIDTH + ID_WIDTH), .PIPELINE(PORT_R_PIPELINE > 0), .REG_READY(PORT_R_PIPELINE > 1)) r_pipe (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata({ p_al_rid, p_al_rdata }),
  .s_rx_tvalid(p_al_rvalid),
  .s_rx_tready(p_al_rready),

  .m_tx_tdata({ s_al_rid, s_al_rdata }),
  .m_tx_tvalid(s_al_rvalid),
  .m_tx_tready(s_al_rready)
);


endmodule
