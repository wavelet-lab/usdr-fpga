// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
// USDR PROJECT
//
module data_shuffle2 #(
    parameter DATA_WIDTH = 16,
    parameter CH_COUNT = 16,
    parameter TAG_WIDTH = 1,
    parameter _CFG_WIDTH = $clog2(CH_COUNT),
    parameter STAGE_PIPELINE = 0,
    parameter STAGE_SKIP = 0
) (

  input                                 rst,
  input                                 clk,

  input [CH_COUNT * _CFG_WIDTH - 1:0]   cfg,

  input [CH_COUNT * DATA_WIDTH - 1:0]   s_in_data,
  input [TAG_WIDTH - 1:0]               s_in_tag,
  input                                 s_in_valid,
  input                                 s_in_last,
  output                                s_in_ready,

  output [CH_COUNT * DATA_WIDTH - 1:0]  m_out_data,
  output [TAG_WIDTH - 1:0]              m_out_tag,
  output                                m_out_valid,
  output                                m_out_last,
  input                                 m_out_ready
);

// I/Q swap
// channel swap

localparam ST_DATA_WIDTH = CH_COUNT * DATA_WIDTH + TAG_WIDTH + 1;
localparam ST_STAGES     = $clog2(CH_COUNT);


// Stage 0: I/Q
wire [CH_COUNT * DATA_WIDTH - 1:0]   st0_data;
wire [CH_COUNT * DATA_WIDTH - 1:0]   st0_out_data;
wire [TAG_WIDTH - 1:0]               st0_out_tag;
wire                                 st0_out_last;
wire                                 st0_out_valid;
wire                                 st0_out_ready;

genvar i,j;
generate
for ( i = 0; i < CH_COUNT; i = i + 1) begin
    localparam [CH_COUNT - 1:0] k = i ^ 1;

    assign st0_data[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = (cfg[i]) ?
        s_in_data[DATA_WIDTH * (k + 1) - 1:DATA_WIDTH * k] :
        s_in_data[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i];
end
endgenerate

axis_opt_pipeline #(.WIDTH(ST_DATA_WIDTH), .PIPE_PASSTHROUGH(/*STAGE_PIPELINE[j]*/1'b1), .PIPELINE(1'b0)) fifo_stage0 (
    .clk(clk),
    .rst(rst),

    .s_rx_tdata({s_in_tag, s_in_last, st0_data }),
    .s_rx_tvalid(s_in_valid),
    .s_rx_tready(s_in_ready),

    .m_tx_tdata({st0_out_tag, st0_out_last, st0_out_data}),
    .m_tx_tvalid(st0_out_valid),
    .m_tx_tready(st0_out_ready)
);

// Stage FULL MUX: data
wire [CH_COUNT * DATA_WIDTH - 1:0] stN_data;
generate
for ( i = 0; i < CH_COUNT; i = i + 2) begin
    wire [_CFG_WIDTH - 1:1] chcfg;
    for ( j = 1; j < _CFG_WIDTH; j=j+1) begin
        assign chcfg[j] = cfg[CH_COUNT*j + i];
    end
    wire [_CFG_WIDTH - 1:1] chcfgc = chcfg ^ (i >> 1);

    assign stN_data[DATA_WIDTH * i +:DATA_WIDTH * 2] = st0_out_data[DATA_WIDTH * 2 * chcfgc +:DATA_WIDTH * 2];
end
endgenerate

axis_opt_pipeline #(.WIDTH(ST_DATA_WIDTH), .PIPE_PASSTHROUGH(/*STAGE_PIPELINE[j]*/1'b1), .PIPELINE(1'b0)) fifo_stageN (
    .clk(clk),
    .rst(rst),

    .s_rx_tdata({st0_out_tag, st0_out_last, stN_data}),
    .s_rx_tvalid(st0_out_valid),
    .s_rx_tready(st0_out_ready),

    .m_tx_tdata({m_out_tag, m_out_last, m_out_data}),
    .m_tx_tvalid(m_out_valid),
    .m_tx_tready(m_out_ready)
);



endmodule

