// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module axis_mux #(
    parameter PORTS = 2,
    parameter PORT_BITS = $clog2(PORTS),
    parameter DATA_WIDTH = 64,
    parameter KEEP_WIDTH = DATA_WIDTH / 8,
    parameter USER_WIDTH = 1
)(
    input         clk,
    input         rst,

    input                             m_axis_tready,
    output [DATA_WIDTH-1:0]           m_axis_tdata,
    output [KEEP_WIDTH-1:0]           m_axis_tkeep,
    output                            m_axis_tlast,
    output                            m_axis_tvalid,
    output [USER_WIDTH-1:0]           m_axis_tuser,

    output [PORTS-1:0]                sn_axis_tready,
    input  [PORTS * DATA_WIDTH-1:0]   sn_axis_tdata,
    input  [PORTS * KEEP_WIDTH-1:0]   sn_axis_tkeep,
    input  [PORTS-1:0]                sn_axis_tlast,
    input  [PORTS-1:0]                sn_axis_tvalid,
    input  [PORTS * USER_WIDTH-1:0]   sn_axis_tuser
);

wire [PORT_BITS - 1:0] sn_valid_idx;
wire                   sn_valid_idx_nv;

ctz_clz #(.BITS(PORTS)) clz_stream_i (.data_i(sn_axis_tvalid), .num_o(sn_valid_idx), .zero_o(sn_valid_idx_nv));

reg [PORT_BITS-1:0] stream_sel;
reg                 in_transaction;
assign m_axis_tdata   = sn_axis_tdata  >> (DATA_WIDTH * stream_sel);
assign m_axis_tkeep   = sn_axis_tkeep  >> (KEEP_WIDTH * stream_sel);
assign m_axis_tvalid  = sn_axis_tvalid >> stream_sel;
assign m_axis_tlast   = sn_axis_tlast  >> stream_sel;
assign m_axis_tuser   = sn_axis_tuser  >> (USER_WIDTH * stream_sel);
assign sn_axis_tready = m_axis_tready  << stream_sel;


always @(posedge clk) begin
  if (rst) begin
    in_transaction <= 1'b0;
    stream_sel     <= 0;
  end else begin
    if (~in_transaction) begin
      if (~m_axis_tvalid) begin
        if (~sn_valid_idx_nv) begin
          stream_sel     <= sn_valid_idx;
        end
      end else begin
        in_transaction <= 1'b1;
      end
    end

    if (m_axis_tready && m_axis_tvalid && m_axis_tlast) begin
      stream_sel     <= sn_valid_idx;
      in_transaction <= (stream_sel != sn_valid_idx) && !sn_valid_idx_nv;
    end
  end
end


endmodule
