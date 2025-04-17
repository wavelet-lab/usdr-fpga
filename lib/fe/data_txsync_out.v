// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
// USDR PROJECT
//
module data_txsync_out #(
    parameter TIMESTAMP_BITS = 48,
    parameter TAG_WIDTH = 1,
    parameter CH_COUNT = 4,
    parameter SAMP_WIDTH = 16,
    parameter ASYNC_CLK = 1,
    parameter FRAME_LENGTH = 32,
    parameter WITH_NOTS_BIT = 1
) (
  input clk,
  input rst,

  output                                    s_descr_ready,
  input                                     s_descr_valid,
  input  [TIMESTAMP_BITS-1:0]               s_descr_data,

  input [CH_COUNT * SAMP_WIDTH - 1:0]       s_in_data,
  input [TAG_WIDTH - 1:0]                   s_in_tag,
  input                                     s_in_valid,
  input                                     s_in_last,
  output                                    s_in_ready,

  output reg [TAG_WIDTH-1:0]                m_abort_id,
  output reg                                m_abort_valid,
  output reg                                sig_underrun,

  // Output to DAC
  output reg [CH_COUNT * SAMP_WIDTH - 1:0]  m_dac_data,
  output reg                                m_dac_valid,
  output                                    m_dac_frame,  // LFMC clock to resync after CCs
  output reg                                m_dac_active,
  input                                     m_dac_ready,
  input                                     m_dac_sync,  // crossing from 0->1 resets TX timer
  output [TIMESTAMP_BITS - 1:0]             m_dac_time,

    input                                   m_fedma_clk,
    input                                   m_fedma_rst,
    output [TIMESTAMP_BITS - 1:0]           m_fedma_ts   // Current TS for stats
);

wire  [TIMESTAMP_BITS-1:0]    tx_timer;
assign                        m_dac_time  = tx_timer;
assign                        m_dac_frame = tx_timer[$clog2(FRAME_LENGTH)];

generate
if (ASYNC_CLK) begin
    wire dac_nsync_fedma_clk;

    cc_counter #(
        .WIDTH(TIMESTAMP_BITS),
        .GRAY_BITS(6)
    ) cc_tx_timer (
        .in_clk(clk),
        .in_rst(!m_dac_sync),
        .in_increment( m_dac_valid && m_dac_ready ),
        .in_counter(tx_timer),

        .out_clk(m_fedma_clk),
        .out_rst(dac_nsync_fedma_clk),
        .out_counter(m_fedma_ts)
    );

    synchronizer #( .INIT(1) ) dac_sync_dma_cc (.clk(m_fedma_clk), .rst(1'b0), .a_in(!m_dac_sync), .s_out(dac_nsync_fedma_clk));
end else begin
    assign ram_addr_rst = m_fedma_rst;

    reg [TIMESTAMP_BITS-1:0] sync_tx_timer_reg;
    always @(posedge clk) begin
        if (!m_dac_sync) begin
            sync_tx_timer_reg <= 0;
        end else begin
            if (m_dac_valid && m_dac_ready) begin
                sync_tx_timer_reg <= sync_tx_timer_reg + 1'b1;
            end
        end
    end

    assign tx_timer   = sync_tx_timer_reg;
    assign m_fedma_ts = sync_tx_timer_reg;
end
endgenerate


wire [TIMESTAMP_BITS:0] timestamp_diff     = s_descr_data - tx_timer;
wire                    time_check         = timestamp_diff[TIMESTAMP_BITS];
wire                    time_notset        = WITH_NOTS_BIT && s_descr_data[TIMESTAMP_BITS-1];
wire                    time_late          = time_check && !time_notset;

reg                     tm_valid;
reg                     tm_late;

reg                     time_check_late;
reg                     time_check_trigger;


////////////////////////////////////////////////////////////////////////////////

localparam [1:0]
    ST_IDLE = 2'b00,
    ST_BURST = 2'b10,
    ST_SKIP = 2'b11;

reg [1:0] state;
reg       ready_flag;

assign s_in_ready = ready_flag && m_dac_ready;
assign s_descr_ready = m_dac_ready && (state == ST_IDLE) && ready_flag && s_in_valid;

wire time_valid_next = tm_valid && s_descr_valid && (time_late || time_notset && s_in_valid);

always @(posedge clk) begin
    if (rst) begin
        state         <= ST_IDLE;
        m_dac_valid   <= 1'b0;
        m_abort_valid <= 1'b0;

        ready_flag    <= 1'b0;
        tm_valid      <= 1'b0;

        sig_underrun  <= 1'b0;
    end else begin
        sig_underrun  <= 1'b0;

        if (m_dac_ready) begin

            tm_valid <= s_descr_valid;
            tm_late  <= time_late;

            time_check_trigger <= !tm_late && time_valid_next;
            time_check_late    <=  tm_late && time_valid_next;

            m_dac_valid <= 1'b1;
            m_dac_data  <= 0;

            case (state)
            ST_IDLE: begin
                ready_flag    <= time_valid_next;

                if (s_in_valid && time_check_trigger) begin
                    ready_flag    <= 1'b1;
                    m_dac_data    <= s_in_data;
                    state         <= ST_BURST;
                end else if (s_in_valid && time_check_late) begin
                    ready_flag    <= 1'b1;
                    m_abort_valid <= 1'b1;
                    m_abort_id    <= s_in_tag;
                    state         <= ST_SKIP;
                    sig_underrun  <= 1'b1;
                end
            end

            ST_BURST, ST_SKIP: begin
                m_dac_data <= (state == ST_BURST) ? s_in_data : 0;

                if (s_in_valid && s_in_last) begin
                    state      <= ST_IDLE;
                    // update flags
                    ready_flag <= time_valid_next;
                    m_abort_valid <= 1'b0;
                end
            end

            endcase

            if (s_descr_valid && s_descr_ready) begin
                tm_valid   <= 1'b0;
            end

        end

    end
end



endmodule
