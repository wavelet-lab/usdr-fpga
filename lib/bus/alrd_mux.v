// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module alrd_mux #(
    parameter DATA_BITS   = 2,
    parameter DATA_WIDTH  = 8 << DATA_BITS,
    parameter ADDR_WIDTH  = 4,
    parameter SLAVE_COUNT = 1,
    parameter SLAVE_COUNT_BITS = $clog2(SLAVE_COUNT),
    parameter SLAVE_COUNT_BITS_FIX = (SLAVE_COUNT_BITS == 0) ? 1 : SLAVE_COUNT_BITS,
    parameter SLAVE_LATCH_PIPELINED = 0
)(
    // AL clocks
    input                                 clk,
    input                                 rst,

    // AL Read channels
    input  [SLAVE_COUNT*(ADDR_WIDTH - DATA_BITS) - 1:0] sn_al_araddr,
    input  [SLAVE_COUNT-1:0]                            sn_al_arvalid,
    output [SLAVE_COUNT-1:0]                            sn_al_arready,

    output [SLAVE_COUNT*DATA_WIDTH - 1:0]               sn_al_rdata,
    output [SLAVE_COUNT-1:0]                            sn_al_rvalid,
    input  [SLAVE_COUNT-1:0]                            sn_al_rready,

    // Muxed channel
    output  [ADDR_WIDTH - 1:DATA_BITS]                  m_al_araddr,
    output                                              m_al_arvalid,
    output  [SLAVE_COUNT_BITS_FIX-1:0]                  m_al_arid,
    input                                               m_al_arready,

    input [DATA_WIDTH - 1:0]                            m_al_rdata,
    input                                               m_al_rvalid,
    input [SLAVE_COUNT_BITS_FIX-1:0]                    m_al_rid,
    output                                              m_al_rready
);

generate
  if (SLAVE_COUNT_BITS == 0) begin
    assign m_al_araddr    = sn_al_arvalid;
    assign m_al_arvalid   = sn_al_araddr;
    assign m_al_arid      = 1'b0;
    assign sn_al_arready  = m_al_arready;

    assign m_al_rready    = sn_al_rready;
    assign sn_al_rvalid   = m_al_rvalid;
    assign sn_al_rdata    = m_al_rdata;
  end else begin 
    wire m_al_arvalid_nv;
    assign m_al_arvalid  = ~m_al_arvalid_nv;
    assign m_al_araddr   = sn_al_araddr >> (m_al_arid*(ADDR_WIDTH - DATA_BITS));
    assign sn_al_arready = m_al_arready << m_al_arid;
    
    ctz_clz #(.BITS(SLAVE_COUNT)) clz_arddr_i (.data_i(sn_al_arvalid), .num_o(m_al_arid), .zero_o(m_al_arvalid_nv));

    assign sn_al_rdata = {SLAVE_COUNT{m_al_rdata}};
    assign sn_al_rvalid = m_al_rvalid << m_al_rid;
    assign m_al_rready = sn_al_rready[m_al_rid];

  end
endgenerate

endmodule
