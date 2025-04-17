// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
// USDR PROJECT
//
module data_expander #(
    parameter DATA_WIDTH = 16,
    parameter CH_COUNT = 16,
    parameter TAG_WIDTH = 1,
    parameter _CFG_WIDTH = $clog2(CH_COUNT)
) (
  input                                 rst,
  input                                 clk,

  input [_CFG_WIDTH-1:0]                cfg_expand,

  input [CH_COUNT * DATA_WIDTH - 1:0]   s_in_data,
  input [TAG_WIDTH - 1:0]               s_in_tag,
  input                                 s_in_valid,
  input [_CFG_WIDTH - 1:0]              s_in_lccnt,
  input                                 s_in_last,
  output                                s_in_ready,

  // lookahead data (1 beat)
  input                                 s_in_nxt_valid,
  input [_CFG_WIDTH - 1:0]              s_in_nxt_lccnt,
  input                                 s_in_nxt_last,

  output reg [CH_COUNT * DATA_WIDTH - 1:0]  m_out_data,
  output reg [TAG_WIDTH - 1:0]              m_out_tag,
  output reg                                m_out_valid,
  output reg                                m_out_last,
  input                                     m_out_ready
);

localparam C_WIDTH = $clog2(CH_COUNT);

reg [C_WIDTH-1:0] stage;

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

wire [C_WIDTH-1:0]   cfg_stages = (1 << (C_WIDTH - cfg_expand)) - 1'b1;

wire [C_WIDTH - 1:0]  msk_bshft = ((1 << C_WIDTH) - 1'b1) << cfg_expand;
// wire [C_WIDTH - 1:0]  msk_ishft = 1                       << cfg_expand;
// wire [CH_COUNT - 1:0] msk_raw   = ((1 << DATA_WIDTH) - 1'b1) >> msk_bshft;


wire [CH_COUNT * DATA_WIDTH - 1:0] shifted_data = s_in_data >> ((DATA_WIDTH << cfg_expand) * stage);

//wire  stage_last = !(s_in_keep >> msk_ishft * (stage + 1))[0];


wire  next_transfer_last = s_in_nxt_valid && s_in_nxt_last && (s_in_nxt_lccnt <= ((1 << cfg_expand) - 1'b1));

reg   stall;

assign s_in_ready = m_out_ready && !stall;

wire [C_WIDTH-1:0] stage_last_num = (s_in_lccnt >> cfg_expand);

wire stage_last = (stage == stage_last_num);
wire stage_last_next = (stage_last_num != 0 && stage == stage_last_num - 1'b1);

always @(posedge clk) begin


    if (rst) begin
        m_out_valid <= 1'b0;
        stage       <= 0;
        stall       <= (cfg_stages != 0);

    end else begin
        if (m_out_ready) begin
            m_out_valid <= 1'b0;
            stall       <= (cfg_stages != 0);

            if (s_in_valid) begin
                stage       <= stage + 1'b1;
                m_out_last  <= 1'b0;
                stall       <= 1'b1;

                if (stage == cfg_stages) begin
                    stage       <= 0;
                end

                if (stage_last && s_in_last) begin
                    m_out_last  <= 1'b1;
                    stage       <= 0;

                    stall       <= !(next_transfer_last || (cfg_stages == 0));
                end

                // lookahead for the last transfer
                if (stage_last_next && s_in_last) begin
                    stall <= 1'b0;
                end else if (!s_in_last && stage == cfg_stages - 1) begin
                    stall <= 1'b0;
                end

                if (stage == cfg_stages && next_transfer_last) begin
                    stall <= 1'b0;
                end

                m_out_data  <= shifted_data;
                m_out_tag   <= s_in_tag;
                m_out_valid <= 1'b1;
            end else begin
                stall       <= !(next_transfer_last || (cfg_stages == 0));
            end
        end

        if (((s_in_valid == 0 && m_out_ready == 0) && next_transfer_last || (cfg_stages == 0))) begin
            stall       <= 1'b0;
        end
    end
end



endmodule
