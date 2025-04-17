// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
// USDR PROJECT
//
module data_shuffle #(
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

// Mux config
// 1 -> 2         1 -> 4          1 -> 8
// 2 -> 2         2 -> 4          2 -> 8
//                3 -> 4          3 -> 8
//                4 -> 3          4 -> 8
// ... etc

// Stages:  REDUCED
// m3_cfg
// m2_cfg
// m1_cfg
// --------+------------------------------------------------------------------------------------------
// 0       0          0              0                     0
// 1       1          1 | 0          1 | 0                 1 | 0
// 2       2          2              2     ||  0           2     ||  0
// 3       3          3              3     ||  1 | 0       3     ||  1 | 0
// 4       4          4              4                     4                ||| 0
// 5       5          5              5                     5                ||| 1 | 0
// 6       6          6              6                     6                ||| 2     ||  0
// 7       7          7              7                     7                ||| 3     ||  1 | 0
//
//                                                        000 001  010  011    100  101  110  111
// --------+------------------------------------------------------------------------------------------
// 0       0          0 | 1          0 | 1 ||  2 | 3       0 | 1 ||  2 | 3  |||  4 | 5 ||  6 | 7
// 1       1          1 | 0          1 | 0 ||  3 | 2       1 | 0 ||  3 | 2  |||  5 | 4 ||  7 | 6
// 2       2          2 | 3          2 | 3 ||  0 | 1       2 | 3 ||  0 | 1  |||  6 | 7 ||  4 | 5
// 3       3          3 | 2          3 | 2 ||  1 | 0       3 | 2 ||  1 | 0  |||  7 | 6 ||  5 | 4
// 4       4          4 | 5          4 | 5 ||  6 | 7       4 | 5 ||  6 | 7  |||  0 | 1 ||  2 | 3
// 5       5          5 | 4          5 | 4 ||  7 | 6       5 | 4 ||  7 | 6  |||  1 | 0 ||  3 | 2
// 6       6          6 | 7          6 | 7 ||  4 | 5       6 | 7 ||  4 | 5  |||  2 | 3 ||  0 | 1
// 7       7          7 | 6          7 | 6 ||  5 | 4       7 | 6 ||  5 | 4  |||  3 | 2 ||  1 | 0


// I/Q swap
// channel swap

localparam ST_DATA_WIDTH = CH_COUNT * DATA_WIDTH + TAG_WIDTH + 1;
localparam ST_STAGES     = $clog2(CH_COUNT);

wire [ST_DATA_WIDTH * (ST_STAGES + 1) - 1:0] staged_data;
wire [ST_STAGES:0]                           staged_valid;
wire [ST_STAGES:0]                           staged_ready;

genvar j, i;
generate
for (j = 0; j < ST_STAGES; j = j + 1) begin: stages
    wire [CH_COUNT - 1 : 0] stage_cfg = cfg[CH_COUNT * (j + 1) - 1 :CH_COUNT * j];

    wire [TAG_WIDTH:0]                 ext_data = staged_data[ST_DATA_WIDTH * (j + 1) -1 : ST_DATA_WIDTH * (j + 1) - TAG_WIDTH - 1];
    wire [CH_COUNT * DATA_WIDTH - 1:0] in_data  = staged_data[CH_COUNT * DATA_WIDTH - 1 + ST_DATA_WIDTH * j : ST_DATA_WIDTH * j];
    wire [CH_COUNT * DATA_WIDTH - 1:0] out_data;


    if (STAGE_SKIP[j]) begin
        assign out_data = in_data;
    end else begin
        for ( i = 0; i < CH_COUNT; i = i + 1) begin
            localparam [CH_COUNT - 1:0] k = i ^ (1 << j);

            assign out_data[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = (stage_cfg[i]) ?
                in_data[DATA_WIDTH * (k + 1) - 1:DATA_WIDTH * k] :
                in_data[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i];
        end
    end

    axis_opt_pipeline #(.WIDTH(ST_DATA_WIDTH), .PIPE_PASSTHROUGH(STAGE_PIPELINE[j]), .PIPELINE(1'b0)) fifo_stage (
        .clk(clk),
        .rst(rst),

        .s_rx_tdata({ext_data, out_data}),
        .s_rx_tvalid(staged_valid[j]),
        .s_rx_tready(staged_ready[j]),

        .m_tx_tdata(staged_data[ST_DATA_WIDTH * (j + 2) - 1: ST_DATA_WIDTH * (j + 1)]),
        .m_tx_tvalid(staged_valid[j + 1]),
        .m_tx_tready(staged_ready[j + 1])
    );
end
endgenerate

//
assign staged_data[ST_DATA_WIDTH - 1:0] = {s_in_tag, s_in_last, s_in_data };
assign staged_valid[0]                  = s_in_valid;
assign s_in_ready                       = staged_ready[0];

//
wire [ST_DATA_WIDTH - 1:0] last_data = staged_data[ST_DATA_WIDTH * (ST_STAGES + 1) - 1:ST_DATA_WIDTH * ST_STAGES];
assign m_out_tag  = last_data[CH_COUNT * DATA_WIDTH + TAG_WIDTH:CH_COUNT * DATA_WIDTH + 1];
assign m_out_last = last_data[CH_COUNT * DATA_WIDTH];
assign m_out_data = last_data[CH_COUNT * DATA_WIDTH - 1:0];

assign m_out_valid             = staged_valid[ST_STAGES];
assign staged_ready[ST_STAGES] = m_out_ready;

endmodule

