// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module axis_pipeline_mover #(
  parameter WIDTH = 1,
  parameter DEEP  = 1,
  parameter DATA_INIT = 0,
  parameter DATA_DEF = 0
)(
  input clk,
  input rst,

  input              s_in_valid,
  output             s_in_ready,
  input  [WIDTH-1:0] s_in_data,

  output             m_out_valid,
  input              m_out_ready,
  output [WIDTH-1:0] m_out_data,

  output             move_valid
);

genvar i;
generate
if (DEEP == 0) begin
    assign m_out_data  = s_in_data;
    assign m_out_valid = s_in_valid;
    assign s_in_ready  = m_out_ready;
    assign move_valid  = m_out_valid && m_out_ready;
end else begin
    reg [DEEP-1:0]         pipeline_valid;
    reg [DEEP*WIDTH-1:0]   pipeline_data;

    assign s_in_ready = ~m_out_valid || m_out_ready;
    assign m_out_valid = pipeline_valid[DEEP-1];
    assign m_out_data  = pipeline_data[DEEP * WIDTH - 1:(DEEP - 1)*WIDTH];

    if (DEEP > 1) begin
        assign move_valid = (m_out_valid && m_out_ready) || (~m_out_valid && (s_in_valid || (pipeline_valid[DEEP-2:0] != 0)));
    end else begin
        assign move_valid = (m_out_valid && m_out_ready) || (~m_out_valid && s_in_valid);
    end

    for (i = 0; i < DEEP; i=i+1) begin: dpipeline
        always @(posedge clk) begin
            if (rst) begin
                pipeline_valid[i]                               <= 0;
                if (DATA_INIT) begin
                    pipeline_data[(i + 1) * WIDTH - 1:i * WIDTH]  <= DATA_DEF;
                end
            end else begin
                if (move_valid) begin
                    pipeline_valid[i]                             <= (i == 0) ? s_in_valid : pipeline_valid[i-1];
                    pipeline_data[(i + 1) * WIDTH - 1:i * WIDTH]  <= (i == 0) ? s_in_data  : pipeline_data[i * WIDTH - 1:(i - 1) * WIDTH];
                end
            end
        end
    end
end

endgenerate

endmodule
