// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
// USDR PROJECT
//
// s_in_lbcnt is valid only when s_in_last is asserted
//
module data_unpacker2 #(
    parameter DATA_WIDTH = 16,
    parameter CH_COUNT = 16,
    parameter TAG_WIDTH = 1,
    parameter _CH_COUNT_BITS = $clog2(CH_COUNT),
    parameter _CH_BCOUNT_BITS = _CH_COUNT_BITS + 1
) (
  input                                 rst,
  input                                 clk,

  input                                 cfg_mode_12,

  input [CH_COUNT * 16 - 1:0]           s_in_data,
  input [TAG_WIDTH - 1:0]               s_in_tag,
  input                                 s_in_valid,
  input                                 s_in_last,
  input [_CH_BCOUNT_BITS-1:0]           s_in_lbcnt,
  output                                s_in_ready,

  output reg [CH_COUNT * DATA_WIDTH - 1:0]  m_out_data,
  output reg [TAG_WIDTH - 1:0]              m_out_tag,
  output reg                                m_out_valid,
  output reg                                m_out_last,
  output reg [_CH_COUNT_BITS - 1:0]         m_out_lccnt,
  input                                     m_out_ready,

  // For lookahead analysis not to block pipeline in unpacker stage
  output [_CH_COUNT_BITS - 1:0]             m_out_nxt_lccnt,
  output                                    m_out_nxt_last,
  output                                    m_out_nxt_valid
);

// 16 ch * 16 bit = 256 bits / 32 bytes
// 24 bytes | 12bit lane
//
//    r |      res_12 + cur  | store
// 0: 1 | 256 ->        *192 | 64
// 1: 1 | 256 -> *64  + *128 | 128
// 2: 1 | 256 -> *128 +  *64 |
// 3: 0 |     ->        *192 |

reg [12 * CH_COUNT - 1:0]   res_12;
reg [_CH_COUNT_BITS  :0]    res_12_lbcnt;
reg [1:0]                   state;
reg                         stall;
reg                         stall_last;

wire [_CH_COUNT_BITS - 1:0]         unpacked_16_lccnt = s_in_lbcnt >> 1;

wire [DATA_WIDTH * CH_COUNT - 1:0]  unpacked_16;
wire [DATA_WIDTH * CH_COUNT - 1:0]  unpacked_12_0;
wire [DATA_WIDTH * CH_COUNT - 1:0]  unpacked_12_1;
wire [DATA_WIDTH * CH_COUNT - 1:0]  unpacked_12_2;
wire [DATA_WIDTH * CH_COUNT - 1:0]  unpacked_12_3;

wire [12 * CH_COUNT - 1:0]          packed_12_0 = { s_in_data[12 * CH_COUNT - 1:0] };
wire [12 * CH_COUNT - 1:0]          packed_12_1 = { s_in_data[8 * CH_COUNT - 1:0], res_12[4 * CH_COUNT - 1:0] };
wire [12 * CH_COUNT - 1:0]          packed_12_2 = { s_in_data[4 * CH_COUNT - 1:0], res_12[8 * CH_COUNT - 1:0] };
wire [12 * CH_COUNT - 1:0]          packed_12_3 = { res_12[12 * CH_COUNT - 1:0] };

// Requires extra cycle to flash remaining data at last transfer
localparam [_CH_COUNT_BITS:0]
    FULL_12_L0_BCNT = 12 * CH_COUNT / 8 - 1'b1,
    FULL_12_L1_BCNT =  8 * CH_COUNT / 8 - 1'b1,
    FULL_12_L2_BCNT =  4 * CH_COUNT / 8 - 1'b1;

wire st_0_12_extra = (s_in_lbcnt > FULL_12_L0_BCNT);
wire st_1_12_extra = (s_in_lbcnt > FULL_12_L1_BCNT);
wire st_2_12_extra = (s_in_lbcnt > FULL_12_L2_BCNT);


wire [_CH_COUNT_BITS + 1 - 1:0] unpacked_12_lccnt_0 = (st_0_12_extra) ? (CH_COUNT - 1'b1) : (((s_in_lbcnt + res_12_lbcnt + 1)<<1) / 3) - 1'b1;
wire [_CH_COUNT_BITS + 1 - 1:0] unpacked_12_lccnt_1 = (st_1_12_extra) ? (CH_COUNT - 1'b1) : (((s_in_lbcnt + res_12_lbcnt + 1)<<1) / 3) - 1'b1;
wire [_CH_COUNT_BITS + 1 - 1:0] unpacked_12_lccnt_2 = (st_2_12_extra) ? (CH_COUNT - 1'b1) : (((s_in_lbcnt + res_12_lbcnt + 1)<<1) / 3) - 1'b1;

wire [_CH_COUNT_BITS + 1 - 1:0] unpacked_12_rem_cnt =                                       (((0          + res_12_lbcnt)<<1) / 3) - 1'b1;

genvar i;
generate

for (i = 0; i < CH_COUNT; i=i+1) begin
    if (DATA_WIDTH <= 12) begin
        assign unpacked_12_0[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = packed_12_0[12 * (i + 1) - 1:12 * (i + 1) - DATA_WIDTH];
        assign unpacked_12_1[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = packed_12_1[12 * (i + 1) - 1:12 * (i + 1) - DATA_WIDTH];
        assign unpacked_12_2[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = packed_12_2[12 * (i + 1) - 1:12 * (i + 1) - DATA_WIDTH];
        assign unpacked_12_3[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = packed_12_3[12 * (i + 1) - 1:12 * (i + 1) - DATA_WIDTH];
    end else begin
        assign unpacked_12_0[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = { packed_12_0[12 * (i + 1) - 1:12 * (i + 1) - 12], {(DATA_WIDTH-12){1'b0}}};
        assign unpacked_12_1[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = { packed_12_1[12 * (i + 1) - 1:12 * (i + 1) - 12], {(DATA_WIDTH-12){1'b0}}};
        assign unpacked_12_2[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = { packed_12_2[12 * (i + 1) - 1:12 * (i + 1) - 12], {(DATA_WIDTH-12){1'b0}}};
        assign unpacked_12_3[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = { packed_12_3[12 * (i + 1) - 1:12 * (i + 1) - 12], {(DATA_WIDTH-12){1'b0}}};
    end

    if (DATA_WIDTH <= 16) begin
        assign unpacked_16[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = s_in_data[16 * (i + 1) - 1:16 * (i + 1) - DATA_WIDTH];
    end else begin
        assign unpacked_16[DATA_WIDTH * (i + 1) - 1:DATA_WIDTH * i] = { s_in_data[16 * (i + 1) - 1:16 * (i + 1) - 16], {(DATA_WIDTH-16){1'b0}}};
    end
end

endgenerate


assign s_in_ready = m_out_ready && (!stall) || !m_out_valid;

wire immediate_last_12 = (state == 3) ||
    (state == 0 && !st_0_12_extra) ||
    (state == 1 && !st_1_12_extra) ||
    (state == 2 && !st_2_12_extra);

assign m_out_nxt_lccnt = (!stall && !s_in_last) ? (CH_COUNT - 1'b1) :
    (!cfg_mode_12) ? unpacked_16_lccnt :
    (state == 0 && !stall) ? unpacked_12_lccnt_0 :
    (state == 1 && !stall) ? unpacked_12_lccnt_1 :
    (state == 2 && !stall) ? unpacked_12_lccnt_2 : unpacked_12_rem_cnt;

assign m_out_nxt_last = (!cfg_mode_12) ? s_in_last :
    (s_in_last && immediate_last_12) ? 1'b1 :
    (stall) ? stall_last : 1'b0;

assign m_out_nxt_valid = s_in_valid || stall;

always @(posedge clk) begin
    if (rst) begin
        state          <= 0;
        m_out_valid    <= 1'b0;
        stall          <= 1'b0;
        res_12_lbcnt   <= 0;
    end else if (m_out_ready || !m_out_valid) begin

        if (m_out_valid && m_out_ready && stall) begin
            stall <= 1'b0;
        end

        if (s_in_valid || (cfg_mode_12 && state == 3)) begin
            state <= state + 1'b1;

            if (cfg_mode_12) begin
                if (s_in_last && immediate_last_12) begin
                    m_out_last     <= 1'b1;
                    state          <= 0;
                end else begin
                    m_out_last     <= 0;
                end
            end

            if (cfg_mode_12) begin
                case (state)
                0: begin stall <= s_in_last && !stall && st_0_12_extra; end
                1: begin stall <= s_in_last && !stall && st_1_12_extra; end
                2: begin stall <= s_in_last && !stall && st_2_12_extra || (!s_in_last && !stall); end
                3: begin stall <= 0; end
                endcase

                stall_last <= s_in_last;
            end else begin
                stall <= 1'b0;
            end
        end

        m_out_valid <= m_out_nxt_valid;
        m_out_tag   <= s_in_tag;
        m_out_lccnt <= m_out_nxt_lccnt;

        if (!cfg_mode_12) begin
            m_out_data  <= unpacked_16;
            m_out_last  <= s_in_last;
        end else begin

            if (stall) begin
                state       <= 0;
                m_out_last  <= stall_last;
            end

            case (state)
            0: begin
                m_out_data <= unpacked_12_0;

                if (s_in_valid) begin
                    res_12[4 * CH_COUNT - 1:0]  <=  s_in_data[16 * CH_COUNT - 1:12 * CH_COUNT];
                    res_12_lbcnt                <=  (s_in_lbcnt > FULL_12_L0_BCNT) ? s_in_lbcnt - FULL_12_L0_BCNT : 0;
                end
            end

            1: begin
                m_out_data <= unpacked_12_1;

                if (s_in_valid) begin
                    res_12        <= s_in_data[16 * CH_COUNT - 1:8 * CH_COUNT];
                    res_12_lbcnt  <= (s_in_lbcnt > FULL_12_L1_BCNT) ? s_in_lbcnt - FULL_12_L1_BCNT : 0;
                end
            end

            2: begin
                m_out_data <= unpacked_12_2;
                if (s_in_valid) begin
                    res_12        <= s_in_data[16 * CH_COUNT - 1:4 * CH_COUNT];
                    res_12_lbcnt  <= (s_in_lbcnt > FULL_12_L2_BCNT) ? s_in_lbcnt - FULL_12_L2_BCNT : 0;
                end
            end

            3: begin
                m_out_data <= unpacked_12_3;
                if (s_in_valid) begin
                    res_12_lbcnt <= 0;
                end
            end
            endcase

        end
    end
end



endmodule
