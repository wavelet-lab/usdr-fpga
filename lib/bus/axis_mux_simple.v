// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module axis_mux_simple #(
    parameter PORTS = 2,
    parameter PORT_BITS = $clog2(PORTS),
    parameter DATA_WIDTH = 64
)(
    input                             m_axis_tready,
    output [DATA_WIDTH-1:0]           m_axis_tdata,
    output                            m_axis_tvalid,

    output [PORTS-1:0]                sn_axis_tready,
    input  [PORTS * DATA_WIDTH-1:0]   sn_axis_tdata,
    input  [PORTS-1:0]                sn_axis_tvalid
);

wire [PORT_BITS - 1:0] sn_valid_idx;
wire                   sn_valid_idx_nv;

ctz_clz #(.BITS(PORTS)) clz_stream_i (.data_i(sn_axis_tvalid), .num_o(sn_valid_idx), .zero_o(sn_valid_idx_nv));

assign m_axis_tdata   = sn_axis_tdata  >> (DATA_WIDTH * sn_valid_idx);
assign m_axis_tvalid  = sn_axis_tvalid >> sn_valid_idx;
assign sn_axis_tready = m_axis_tready  << sn_valid_idx;

endmodule
