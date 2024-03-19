// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module alwr_mux #(
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
    input  [SLAVE_COUNT*(ADDR_WIDTH - DATA_BITS) - 1:0] sn_al_waddr,
    input  [SLAVE_COUNT*DATA_WIDTH - 1:0]               sn_al_wdata,
    input  [SLAVE_COUNT-1:0]                            sn_al_wvalid,
    output [SLAVE_COUNT-1:0]                            sn_al_wready,

    // Muxed channel
    output  [ADDR_WIDTH - 1:DATA_BITS]                  m_al_waddr,
    output  [DATA_WIDTH - 1:0]                          m_al_wdata,
    output                                              m_al_wvalid,
    output  [SLAVE_COUNT_BITS_FIX-1:0]                  m_al_wid,
    input                                               m_al_wready
);

generate
  if (SLAVE_COUNT_BITS == 0) begin
    assign sn_al_wready[0] = m_al_wready;
    assign m_al_waddr      = sn_al_waddr;
    assign m_al_wdata      = sn_al_wdata;
    assign m_al_wvalid     = sn_al_wvalid;
    assign m_al_wid        = 1'b0;
  end else begin
    wire m_al_wvalid_nv;

    assign m_al_wvalid   = ~m_al_wvalid_nv;
    assign m_al_waddr    = sn_al_waddr >> (m_al_wid*(ADDR_WIDTH - DATA_BITS));
    assign m_al_wdata    = sn_al_wdata >> (m_al_wid*DATA_WIDTH);
    assign sn_al_wready  = m_al_wready << m_al_wid;

    ctz_clz #(.BITS(SLAVE_COUNT)) clz_arddr_i (.data_i(sn_al_wvalid), .num_o(m_al_wid), .zero_o(m_al_wvalid_nv));
  end
endgenerate

endmodule
