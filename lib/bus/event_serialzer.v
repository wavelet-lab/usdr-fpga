// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module event_serialzer #(
    parameter COUNT = 16,
    parameter COUNT_BITS = $clog2(COUNT),
    parameter PIPELINE = 1
) (
    input clk,
    input rst,

    // Event source
    input  [COUNT - 1:0]      sn_event_valid,
    output [COUNT - 1:0]      sn_event_ready,

    // Serialized event number
    output [COUNT_BITS - 1:0] m_evno_data,
    output                    m_evno_valid,
    input                     m_evno_ready
);

// Least bit has always higher priority
wire [COUNT_BITS - 1:0] ser_event_data;
wire                    ser_event_nvalid;
wire                    ser_event_ready;
assign sn_event_ready = (ser_event_nvalid) ? 0 : 1 << ser_event_data;

ctz_clz #(.BITS(COUNT)) selector (.data_i(sn_event_valid), .num_o(ser_event_data), .zero_o(ser_event_nvalid));

axis_opt_pipeline #(.WIDTH(COUNT_BITS), .PIPELINE(PIPELINE)) event_op (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata(ser_event_data),
  .s_rx_tvalid(~ser_event_nvalid),
  .s_rx_tready(ser_event_ready),

  .m_tx_tdata(m_evno_data),
  .m_tx_tvalid(m_evno_valid),
  .m_tx_tready(m_evno_ready)
);



endmodule

