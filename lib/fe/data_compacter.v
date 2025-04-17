// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
// USDR PROJECT
//
module data_compacter #(
    parameter DATA_WIDTH = 16,
    parameter CH_COUNT = 16,
    parameter TAG_WIDTH = 1,
    parameter _CFG_WIDTH = $clog2(CH_COUNT)
) (
  input                                 rst,
  input                                 clk,

  input [_CFG_WIDTH-1:0]                cfg_compact,

  input [CH_COUNT * DATA_WIDTH - 1:0]   s_in_data,
  input [TAG_WIDTH - 1:0]               s_in_tag,
  input                                 s_in_valid,
  input                                 s_in_last,
  output                                s_in_ready,

  output reg [CH_COUNT * DATA_WIDTH - 1:0]  m_out_data,
  output reg [TAG_WIDTH - 1:0]              m_out_tag,
  output reg                                m_out_valid,
  output reg                                m_out_last,
  output reg [CH_COUNT - 1:0]               m_out_keep,
  input                                     m_out_ready
);

localparam C_WIDTH = $clog2(CH_COUNT);

reg [C_WIDTH-1:0] stage;

// CH_COUNT   16
// C_WIDTH     4


// mode
// 0 -- 1
// 1 -- 1/2
// 2 -- 1/4
// 3 -- 1/8
// 4 -- 1/16

//            0       1     2     3
//          111     110   100   000
// bshift     7       6     4     0
//          001     010   100   000
// ishift     1       2     4     8
// 0   10000000    1000    10     1
// 1   01000000    1000    10     1
// 2   00100000    0100    10     1
// 3   00010000    0100    10     1
// 4   00001000    0010    01     1
// 5   00000100    0010    01     1
// 6   00000010    0001    01     1
// 7   00000001    0001    01     1

wire [C_WIDTH-1:0]   cfg_stages = (1 << (C_WIDTH - cfg_compact)) - 1'b1;

wire [C_WIDTH - 1:0]  msk_bshft = ((1 << C_WIDTH) - 1'b1) << cfg_compact;
wire [C_WIDTH - 1:0]  msk_ishft = (1'b1)                  << cfg_compact;

wire [CH_COUNT - 1:0] msk_raw   = ((1 << DATA_WIDTH) - 1'b1) >> msk_bshft;

wire [CH_COUNT - 1:0] ce        = msk_raw << (msk_ishft * stage);

assign s_in_ready = m_out_ready;


always @(posedge clk) begin
    if (rst) begin
        stage       <= 0;
        m_out_valid <= 1'b0;

    end else if (s_in_ready) begin

        m_out_valid <= 1'b0;

        if (s_in_valid) begin
            stage       <= stage + 1'b1;

            if (stage == cfg_stages || s_in_last) begin
                stage       <= 0;
                m_out_valid <= 1'b1;
            end

            m_out_last <= s_in_last;
            m_out_tag  <= s_in_tag;
        end
    end
end

genvar i;
generate

for (i = 0; i < CH_COUNT; i = i + 1) begin
    always @(posedge clk) begin
        if (rst) begin
            m_out_keep[i] <= 1'b0;
        end else if (m_out_ready) begin
            if (m_out_valid) begin
                m_out_keep[i] <= 1'b0;
            end

            if (ce[i]) begin
                m_out_data[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] <= s_in_data[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i];
                m_out_keep[i]                                       <= 1'b1;
            end
        end
    end
end

endgenerate

endmodule
