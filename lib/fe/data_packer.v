// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
// USDR PROJECT
//
module data_packer #(
    parameter DATA_WIDTH = 16,
    parameter CH_COUNT = 16,
    parameter TAG_WIDTH = 1,
    parameter NO_BACKPRESSURE = 0
) (
  input                                 rst,
  input                                 clk,

  input                                 cfg_mode_12,
  input                                 cfg_last_12_extra,

  input [CH_COUNT * DATA_WIDTH - 1:0]   s_in_data,
  input [TAG_WIDTH - 1:0]               s_in_tag,
  input                                 s_in_valid,
  input                                 s_in_last,
  input [CH_COUNT-1:0]                  s_in_keep,
  output                                s_in_ready,

  output reg [CH_COUNT * 16 - 1:0]      m_out_data,
  output reg [TAG_WIDTH - 1:0]          m_out_tag,
  output reg                            m_out_valid,
  output reg                            m_out_last,
  output reg [CH_COUNT * 2 - 1:0]       m_out_keep,
  input                                 m_out_ready
);


localparam CH_COUNT_3_2 = 3  * ((CH_COUNT + 1) / 2);
localparam CH_COUNT_1_2 = 1  * ((CH_COUNT + 1) / 2);

wire [2  * CH_COUNT - 1:0]  packed_16_keep;
wire [CH_COUNT_3_2  - 1:0]  packed_12_keep;

wire [16 * CH_COUNT - 1:0]  packed_16;
wire [12 * CH_COUNT - 1:0]  packed_12;

genvar i;
generate
for (i = 0; i < CH_COUNT; i=i+1) begin
    assign packed_12[12 * i + 11:12 * i] = s_in_data[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * (i + 1) - 12];
    assign packed_16[16 * i + 15:16 * i] = s_in_data[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * (i + 1) - 16];

    assign packed_16_keep[2 * i + 1:2*i] = {2{s_in_keep[i]}};
end

for (i = 0; i < CH_COUNT; i=i+2) begin
    assign packed_12_keep[3 * i / 2 + 0] = s_in_keep[i + 0];
    assign packed_12_keep[3 * i / 2 + 1] = s_in_keep[i + 0];
    if (i + 1 < CH_COUNT) begin
        assign packed_12_keep[3 * i / 2 + 2] = s_in_keep[i + 1];
    end
end
endgenerate

reg [8 * CH_COUNT - 1:0]    res_12;
reg [1 * CH_COUNT - 1:0]    res_12_keep;
reg [1:0]                   state;

// Example for 256bit data bus
// 256 -> 192       | 0     | 0
// 256 -> 192 + 64  | 128   | 1
// 256 -> 128 + 128 | 64    | 1
// 256 -> 64  + 192 | 0     | 1

reg stall;

assign s_in_ready = m_out_ready && (NO_BACKPRESSURE || !stall);

wire mode_12_stage_1_ns = !packed_12_keep[CH_COUNT_1_2];
wire mode_12_stage_2_ns = !packed_12_keep[CH_COUNT];


always @(posedge clk) begin
    if (rst) begin
        state          <= 0;
        m_out_valid    <= 1'b0;
        stall          <= 1'b0;
    end else if (m_out_ready) begin

        if (m_out_valid && m_out_ready && stall) begin
            stall <= 1'b0;
        end

        if (s_in_valid) begin
            state <= state + 1'b1;

            if (cfg_mode_12) begin
                if (s_in_last && (NO_BACKPRESSURE || state == 3 || state == 0 || state == 1 && mode_12_stage_1_ns || state == 2 && mode_12_stage_2_ns )) begin
                    m_out_last     <= 1'b1;
                    state          <= 0;
                end else begin
                    m_out_last     <= 0;
                end
            end

            if (cfg_mode_12 && !NO_BACKPRESSURE) begin
                case (state)
                0: stall <= 0;
                1: begin stall <= s_in_last && !stall && !mode_12_stage_1_ns; end
                2: begin stall <= s_in_last && !stall && !mode_12_stage_2_ns; end
                3: stall <= 0;
                endcase
            end else begin
                stall <= 1'b0;
            end
        end

        if (!cfg_mode_12) begin
            m_out_data  <= packed_16;
            m_out_keep  <= packed_16_keep;
            m_out_valid <= s_in_valid;
            m_out_last  <= s_in_last;
            m_out_tag   <= s_in_tag;
        end else begin

            m_out_valid    <= (state == 2'b00) ? s_in_valid && s_in_last : s_in_valid;
            m_out_tag      <= s_in_tag;

            if (m_out_valid && stall) begin
                state       <= 0;
                m_out_valid <= 1'b1;
                m_out_last  <= 1'b1;
            end

            case (state)
            0: begin
                m_out_data[12 * CH_COUNT - 1:0]             <= packed_12;
                m_out_keep[CH_COUNT_3_2 - 1:0]              <= packed_12_keep;
                m_out_keep[ 2 * CH_COUNT - 1:CH_COUNT_3_2]  <= 0;
            end

            1: begin
                m_out_data[16 * CH_COUNT - 1:12 * CH_COUNT] <= packed_12[4 * CH_COUNT - 1:0];
                m_out_keep[ 2 * CH_COUNT - 1:CH_COUNT_3_2]  <= packed_12_keep[CH_COUNT_1_2 - 1:0];

                res_12                                      <= packed_12[12 * CH_COUNT - 1:4 * CH_COUNT];
                res_12_keep                                 <= packed_12_keep[CH_COUNT_3_2 - 1:CH_COUNT_1_2];
            end

            2: begin
                m_out_data[8 * CH_COUNT - 1:0]              <= res_12;
                m_out_data[16 * CH_COUNT - 1:8 * CH_COUNT]  <= packed_12[8 * CH_COUNT - 1:0];

                m_out_keep[1 * CH_COUNT - 1:0]              <= res_12_keep;
                m_out_keep[2 * CH_COUNT - 1: 1 * CH_COUNT]  <= s_in_valid ? packed_12_keep[1 * CH_COUNT - 1:0] : 0;

                if (s_in_valid) begin
                    //res_12[8 * CH_COUNT - 1:4 * CH_COUNT]       <= packed_12[12 * CH_COUNT - 1:8 * CH_COUNT];
                    //res_12_keep[1 * CH_COUNT - 1:CH_COUNT_1_2]  <= packed_12_keep[CH_COUNT_3_2 - 1:1 * CH_COUNT];

                    res_12[4 * CH_COUNT - 1:0]       <= packed_12[12 * CH_COUNT - 1:8 * CH_COUNT];
                    res_12_keep[CH_COUNT_1_2 - 1:0]  <= packed_12_keep[CH_COUNT_3_2 - 1:1 * CH_COUNT];
                end
            end

            3: begin
                //m_out_data[4 * CH_COUNT - 1:0]              <= res_12[8 * CH_COUNT - 1:4 * CH_COUNT];
                m_out_data[4 * CH_COUNT - 1:0]              <= res_12[4 * CH_COUNT - 1:0];
                m_out_data[16 * CH_COUNT - 1:4 * CH_COUNT]  <= packed_12;

                //m_out_keep[CH_COUNT_1_2 - 1:0]              <= res_12_keep[1 * CH_COUNT - 1:CH_COUNT_1_2];
                m_out_keep[CH_COUNT_1_2 - 1:0]              <= res_12_keep[CH_COUNT_1_2 - 1:0];
                m_out_keep[2 * CH_COUNT - 1:CH_COUNT_1_2]   <= s_in_valid ? packed_12_keep : 0;
            end
            endcase


        end
    end
end




endmodule

