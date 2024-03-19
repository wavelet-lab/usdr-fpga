// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module alwr_demux_axis #(
    parameter DATA_BITS = 2,
    parameter DATA_WIDTH = 8 << DATA_BITS,
    parameter ADDR_TOTAL = 2,
    parameter ADDR_WIDTH = $clog2(ADDR_TOTAL) + DATA_BITS
)(
    // AL Write channel
    input [ADDR_WIDTH - 1:DATA_BITS]  s_al_waddr,
    input [DATA_WIDTH - 1:0]          s_al_wdata,
    input                             s_al_wvalid,
    output                            s_al_wready,

    // Demuxed Vector AXIS 
    output [DATA_WIDTH - 1:0]         mn_axis_data,
    output [ADDR_TOTAL - 1:0]         mn_axis_valid,
    input  [ADDR_TOTAL - 1:0]         mn_axis_ready
);

genvar i;
generate
  for (i = 0; i < ADDR_TOTAL; i = i + 1) begin: strobe_gen
    assign mn_axis_valid[i] = (s_al_wvalid && (s_al_waddr == i));
  end
endgenerate

assign s_al_wready = mn_axis_ready[s_al_waddr];
assign mn_axis_data = s_al_wdata;

endmodule
