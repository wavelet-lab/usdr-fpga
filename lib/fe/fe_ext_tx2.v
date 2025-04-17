// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2025 Wavelet Lab
//
// USDR PROJECT
//
module fe_ext_tx2 #(
    parameter TIMESTAMP_BITS = 48,
    parameter RAM_ADDR_WIDTH = 18,
    parameter RAW_CHANS = 4,
    parameter DATA_BITS = $clog2(RAW_CHANS) + 1,
    parameter DATA_WIDTH = 8 << DATA_BITS,
    parameter DESCR_DATA_BITS = $clog2(RAW_CHANS) + 1,
    parameter FE_IDX_BITS = 5,
    parameter SAMPLES_WIDTH = RAM_ADDR_WIDTH,
    parameter FE_DESCR_WIDTH = TIMESTAMP_BITS + SAMPLES_WIDTH + (RAM_ADDR_WIDTH - DESCR_DATA_BITS) + 2,
    parameter FRAME_LENGTH = 32, //Extra samples to flush beforehand to compensate distribution delay through extra pipeline stages
    parameter RAM_CHECK_BIT = 8,
    parameter SAMPLES_CHECK_BITS = (DATA_BITS - 1),
    //parameter ID_WIDTH = 1 + SAMPLES_CHECK_BITS,
    parameter EXACT_SAMPLES_CHECK = 1,
    parameter INITIAL_TS_COMP = FRAME_LENGTH,
    parameter ASYNC_CLK = 1,  // clk & m_fedma_clk are async to each other
    parameter WITH_NOTS_BIT = 1,
    parameter SUPPORT_8CH      = 1'b0,
    parameter SUPPORT_4CH      = 1'b0,
    parameter TX_OLD_FLAGS     = 1'b1,
    parameter DAC_CMD_WIDTH    = 1'b1,
    parameter ID_WIDTH = DAC_CMD_WIDTH + DATA_BITS + 1,
    parameter BUFFER = 1'b1,
    parameter ULTRA_SCALE = 1'b0,
    parameter EXTRA_DESC_BUFFER = 1'b1
)(
    input clk,
    input rst,

    input rst_fe,

    // Burst processed and is ready to be reused (clocked @m_fedma_clk)
    output                                  m_proc_idx_valid,
    input                                   m_proc_idx_ready,

    output                                  s_descr_ready,
    input                                   s_descr_valid,
    input  [FE_DESCR_WIDTH-1:0]             s_descr_data,

    // CC-stat
    input                                   m_fedma_clk,
    input                                   m_fedma_rst,
    output [TIMESTAMP_BITS - 1:0]           m_fedma_ts,         //Current TS to discard old buffer requests
    output [RAM_ADDR_WIDTH:RAM_CHECK_BIT]   m_fedma_ram_addr,   //Determines availability of RAM


    // Configurations
    input [1:0]                             cfg_mute,
    input                                   cfg_swap,
    input [1:0]                             cfg_format,

    output                                  sig_underrun,

    // FIFO RB interface
    output     [RAM_ADDR_WIDTH-1:DATA_BITS] m_fifo_araddr,
    output                                  m_fifo_arvalid,
    input                                   m_fifo_arready,
    output     [ID_WIDTH-1:0]               m_fifo_arid,

    input      [DATA_WIDTH-1:0]             m_fifo_rdata,
    input                                   m_fifo_rvalid,
    input      [ID_WIDTH-1:0]               m_fifo_rid,
    output                                  m_fifo_rready,

    // Clocked at m_fedma_clk
    input [31:0] s_fe_cmd_data,
    input        s_fe_cmd_valid,
    output       s_fe_cmd_ready,

    // DATA to DACs
    output [TIMESTAMP_BITS - 1:0]  dac_time,
    output [DATA_WIDTH-1:0]        dac_data,
    output                         dac_valid,
    output                         dac_frame,  // LFMC clock
    input                          dac_ready,
    input                          dac_sync    // crossing from 0->1 resets TX timer
);

`include "axi_helpers.vh"

localparam CHAN_BITS = $clog2(RAW_CHANS);
localparam DESCR_ADDR_WIDTH = RAM_ADDR_WIDTH + (RAM_ADDR_WIDTH - DESCR_DATA_BITS) + 2;

`ifdef SYM
wire [31:0] debug_CHAN_BITS = CHAN_BITS;
wire [31:0] debug_DATA_BITS = DATA_BITS;
wire [31:0] debug_DATA_WIDTH = DATA_WIDTH;
wire [31:0] debug_RAW_CHANS = RAW_CHANS;
wire [31:0] debug_DESCR_DATA_BITS = DESCR_DATA_BITS;
`endif


`DEFINE_AXIS_RVD_PORT(descr_time_n_, TIMESTAMP_BITS);
`DEFINE_AXIS_RVD_PORT(descr_addr_n_, DESCR_ADDR_WIDTH);

`DEFINE_AXIS_RVD_PORT(descr_time_, TIMESTAMP_BITS);
`DEFINE_AXIS_RVD_PORT(descr_addr_, DESCR_ADDR_WIDTH);

wire descr_time_pipe_ready;
wire descr_time_addr_pipe_ready;
assign s_descr_ready = descr_time_pipe_ready && descr_time_addr_pipe_ready;

axis_opt_pipeline #(
  .WIDTH(TIMESTAMP_BITS) //, .PIPELINE(!EXTRA_DESC_BUFFER)
) descr_time_pipe (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata(s_descr_data[FE_DESCR_WIDTH - 1:DESCR_ADDR_WIDTH]),
  .s_rx_tvalid(s_descr_valid && descr_time_addr_pipe_ready && !s_descr_data[0]),
  .s_rx_tready(descr_time_pipe_ready),

  `AXIS_RVD_PORT_CONN(m_tx_t, descr_time_n_)
);

axis_opt_pipeline #(
  .WIDTH(DESCR_ADDR_WIDTH) //, .PIPELINE(!EXTRA_DESC_BUFFER)
) descr_time_addr_pipe (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata(s_descr_data[DESCR_ADDR_WIDTH - 1:0]),
  .s_rx_tvalid(s_descr_valid && descr_time_pipe_ready),
  .s_rx_tready(descr_time_addr_pipe_ready),

  `AXIS_RVD_PORT_CONN(m_tx_t, descr_addr_n_)
);

generate
if (EXTRA_DESC_BUFFER) begin
// Elastic buffer
axis_fifo #(.WIDTH(DESCR_ADDR_WIDTH), .DEEP(16)) descr_addr_n_fifo (
  .clk(clk),
  .rst(rst),

  `AXIS_RVD_PORT_CONN(s_rx_t, descr_addr_n_),
  `AXIS_RVD_PORT_CONN(m_tx_t, descr_addr_)
);

// Elastic buffer
axis_fifo #(.WIDTH(TIMESTAMP_BITS), .DEEP(16)) descr_time_n_fifo (
  .clk(clk),
  .rst(rst),

  `AXIS_RVD_PORT_CONN(s_rx_t, descr_time_n_),
  `AXIS_RVD_PORT_CONN(m_tx_t, descr_time_)
);
end else begin
assign descr_time_valid   = descr_time_n_valid;
assign descr_time_data    = descr_time_n_data;
assign descr_time_n_ready = descr_time_ready;

assign descr_addr_valid   = descr_addr_n_valid;
assign descr_addr_data    = descr_addr_n_data;
assign descr_addr_n_ready = descr_addr_ready;
end
endgenerate



localparam SAMP_WIDTH = 16;

`DEFINE_AXIS_RVDLT_PORT(rbbuff_, SAMP_WIDTH * RAW_CHANS, DAC_CMD_WIDTH);

`DEFINE_AXIS_RVDLT_PORT(unpacker_, SAMP_WIDTH * RAW_CHANS, DAC_CMD_WIDTH);
`DEFINE_AXIS_RVDLT_PORT(expander_, SAMP_WIDTH * RAW_CHANS, DAC_CMD_WIDTH);
`DEFINE_AXIS_RVDLT_PORT(shuffle_, SAMP_WIDTH * RAW_CHANS, DAC_CMD_WIDTH);
`DEFINE_AXIS_RVDLT_PORT(muted_, SAMP_WIDTH * RAW_CHANS, DAC_CMD_WIDTH);

`DEFINE_AXIS_RVDLT_PORT(buffer_, SAMP_WIDTH * RAW_CHANS, DAC_CMD_WIDTH);

wire  [DAC_CMD_WIDTH-1:0]              abort_id;
wire                                   abort_valid;

localparam DESCR_ADDR_WIDTH_A = RAM_ADDR_WIDTH + 2 - DESCR_DATA_BITS;

wire [RAM_ADDR_WIDTH+1-DATA_BITS:1]    descr_addr = (DESCR_DATA_BITS == DATA_BITS) ?
    descr_addr_data[DESCR_ADDR_WIDTH_A - 1:1] : (DESCR_DATA_BITS > DATA_BITS) ?
    { descr_addr_data[DESCR_ADDR_WIDTH_A - 1:1], {(DESCR_DATA_BITS - DATA_BITS){1'b0}}} :
      descr_addr_data[DESCR_ADDR_WIDTH_A - 1:1 + DATA_BITS - DESCR_DATA_BITS];

wire [RAM_ADDR_WIDTH-1:0]              descr_data = descr_addr_data[DESCR_ADDR_WIDTH - 1:DESCR_ADDR_WIDTH_A];

data_packeter_rb #(
    .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH),
    .CHANS(RAW_CHANS),
    .FE_IDX_BITS(FE_IDX_BITS),
    .RAM_CHECK_BIT(RAM_CHECK_BIT),
    .ASYNC_CLK(ASYNC_CLK),
    .DAC_CMD_WIDTH(DAC_CMD_WIDTH),
    .ID_WIDTH(ID_WIDTH)
) data_packeter_rb (
    .clk(clk),
    .rst(rst),

    .s_descr_ready(descr_addr_ready),
    .s_descr_valid(descr_addr_valid),
    .s_descr_addr({descr_addr, descr_addr_data[0]}), //RAM addr + discard flag
    .s_descr_data(descr_data),

    // FIFO RB interface
    .m_fifo_araddr(m_fifo_araddr),
    .m_fifo_arvalid(m_fifo_arvalid),
    .m_fifo_arready(m_fifo_arready),
    .m_fifo_arid(m_fifo_arid),

    .s_abort_id(abort_id),
    .s_abort_valid(abort_valid),

    // CC-stat
    .m_fedma_clk(m_fedma_clk),
    .m_fedma_rst(m_fedma_rst),
    .m_fedma_ram_addr(m_fedma_ram_addr),   //Determines availability of RAM

    .m_fedma_idx_valid(m_proc_idx_valid),
    .m_fedma_idx_ready(m_proc_idx_ready)
);

wire [7:0]  fe_cmd_route_addr;
wire [23:0] fe_cmd_route_data;
wire        fe_cmd_route_valid;
wire        fe_cmd_route_ready = 1'b1;


wire rst_to_fedma_clk_rst_fe;

axis_opt_cc_fifo #(.CLK_CC(ASYNC_CLK), .WIDTH(32), .ULTRA_SCALE(ULTRA_SCALE)) cmd_queue (
  .rx_clk(m_fedma_clk),
  .rx_rst(rst_to_fedma_clk_rst_fe),

  .s_rx_tdata(s_fe_cmd_data),
  .s_rx_tvalid(s_fe_cmd_valid),
  .s_rx_tready(s_fe_cmd_ready),

  .tx_clk(clk),
  .tx_rst(rst_fe),

  .m_tx_tdata({ fe_cmd_route_addr, fe_cmd_route_data }),
  .m_tx_tvalid(fe_cmd_route_valid),
  .m_tx_tready(fe_cmd_route_ready)
);


generate
if (ASYNC_CLK == 0) begin
    assign rst_to_fedma_clk_rst_fe = rst_fe;
end else begin
    synchronizer #(.INIT(1'b1)) rst_to_fedma_sync (
        .clk(m_fedma_clk),
        .rst(1'b0),
        .a_in(rst_fe),
        .s_out(rst_to_fedma_clk_rst_fe)
    );
    end
endgenerate

localparam FE_CMD_EXPAND         = 8'h0d;
localparam FE_CMD_FMT12          = 8'h0e;
localparam FE_CMD_MUTE           = 8'h0f;

// 0x10 through 0x1f shuffle configuration
localparam FE_CMD_SHUFFLE_0      = 8'h10;
localparam FE_CMD_SHUFFLE_1      = 8'h11;
localparam FE_CMD_SHUFFLE_2      = 8'h12;
localparam FE_CMD_SHUFFLE_3      = 8'h13;
localparam FE_CMD_SHUFFLE_4      = 8'h14;
localparam FE_CMD_SHUFFLE_5      = 8'h15;

localparam CHAN_STAGES = $clog2(RAW_CHANS);
reg [RAW_CHANS * CHAN_STAGES - 1:0]   cfg_shuffle;
reg [RAW_CHANS - 1:0]                 cfg_emute;
reg                                   cfg_fmt12;
reg [CHAN_STAGES - 1:0]               cfg_expand;

always @(posedge clk) begin
    if (fe_cmd_route_valid && fe_cmd_route_ready) begin
        case (fe_cmd_route_addr)
        FE_CMD_EXPAND: begin
            cfg_expand <= fe_cmd_route_data;
        end

        FE_CMD_FMT12: begin
            cfg_fmt12 <= fe_cmd_route_data[0];
        end

        FE_CMD_MUTE: begin
            cfg_emute <= fe_cmd_route_data;
        end

        FE_CMD_SHUFFLE_0: if (RAW_CHANS > 1) begin
            cfg_shuffle[RAW_CHANS * (0 + 1) - 1: RAW_CHANS * 0] <= fe_cmd_route_data;
        end

        FE_CMD_SHUFFLE_1: if (RAW_CHANS > 2) begin
            cfg_shuffle[RAW_CHANS * (1 + 1) - 1: RAW_CHANS * 1] <= fe_cmd_route_data;
        end

        FE_CMD_SHUFFLE_2: if (RAW_CHANS > 4) begin
            cfg_shuffle[RAW_CHANS * (2 + 1) - 1: RAW_CHANS * 2] <= fe_cmd_route_data;
        end

        FE_CMD_SHUFFLE_3: if (RAW_CHANS > 8) begin
            cfg_shuffle[RAW_CHANS * (3 + 1) - 1: RAW_CHANS * 3] <= fe_cmd_route_data;
        end

        FE_CMD_SHUFFLE_4: if (RAW_CHANS > 16) begin
            cfg_shuffle[RAW_CHANS * (4 + 1) - 1: RAW_CHANS * 4] <= fe_cmd_route_data;
        end

        FE_CMD_SHUFFLE_5: if (RAW_CHANS > 32) begin
            cfg_shuffle[RAW_CHANS * (5 + 1) - 1: RAW_CHANS * 5] <= fe_cmd_route_data;
        end
        endcase
    end
end

///////////////////////////////////////////////////////////////////////////////
// data path

wire                       m_fifo_rlast  = m_fifo_rid[0];
wire [DATA_BITS - 1:0]     m_fifo_rlbcnt = m_fifo_rid[DATA_BITS:1];
wire [DAC_CMD_WIDTH - 1:0] m_fifo_rtag   = m_fifo_rid[ID_WIDTH - 1:ID_WIDTH - DAC_CMD_WIDTH];

wire [DATA_BITS - 1:0]     rbbuff_rlbcnt;

wire [CHAN_BITS -1:0]      unpacker_lccnt;
wire [CHAN_BITS -1:0]      unpacker_nxt_lccnt;
wire                       unpacker_nxt_last;
wire                       unpacker_nxt_valid;

axis_opt_pipeline #(
  .WIDTH(RAW_CHANS * SAMP_WIDTH + DATA_BITS + 1 + DAC_CMD_WIDTH), .PIPE_PASSTHROUGH(1'b1)
) rb_data_pipe (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata( { m_fifo_rlbcnt, m_fifo_rtag, m_fifo_rlast, m_fifo_rdata } ),
  .s_rx_tvalid(m_fifo_rvalid),
  .s_rx_tready(m_fifo_rready),

  .m_tx_tdata( { rbbuff_rlbcnt, rbbuff_tag, rbbuff_last, rbbuff_data } ),
  .m_tx_tvalid(rbbuff_valid),
  .m_tx_tready(rbbuff_ready)
);

data_unpacker2 #(
    .DATA_WIDTH(SAMP_WIDTH),
    .CH_COUNT(RAW_CHANS),
    .TAG_WIDTH(DAC_CMD_WIDTH)
) data_unpacker2 (
  .rst(rst),
  .clk(clk),

  .cfg_mode_12(cfg_fmt12),

  `AXIS_RVDLT_PORT_CONN(s_in_, rbbuff_),
   .s_in_lbcnt(rbbuff_rlbcnt),

  `AXIS_RVDLT_PORT_CONN(m_out_, unpacker_),
  .m_out_lccnt(unpacker_lccnt),

  .m_out_nxt_lccnt(unpacker_nxt_lccnt),
  .m_out_nxt_last(unpacker_nxt_last),
  .m_out_nxt_valid(unpacker_nxt_valid)
);

data_expander #(
    .DATA_WIDTH(SAMP_WIDTH),
    .CH_COUNT(RAW_CHANS),
    .TAG_WIDTH(DAC_CMD_WIDTH)
) data_expander (
  .rst(rst),
  .clk(clk),

  .cfg_expand(cfg_expand),

  `AXIS_RVDLT_PORT_CONN(s_in_, unpacker_),
  .s_in_lccnt(unpacker_lccnt),

  .s_in_nxt_lccnt(unpacker_nxt_lccnt),
  .s_in_nxt_last(unpacker_nxt_last),
  .s_in_nxt_valid(unpacker_nxt_valid),

  `AXIS_RVDLT_PORT_CONN(m_out_, expander_)
);


data_shuffle #(.DATA_WIDTH(SAMP_WIDTH), .CH_COUNT(RAW_CHANS), .STAGE_PIPELINE(8'b10101010), .TAG_WIDTH(DAC_CMD_WIDTH)) data_shuffle (
  .rst(rst),
  .clk(clk),

  .cfg(cfg_shuffle),

  `AXIS_RVDLT_PORT_CONN(s_in_,  expander_),
  `AXIS_RVDLT_PORT_CONN(m_out_, shuffle_)
);

// Mute stage
// Do not latch this stage
genvar i;
generate
for (i = 0; i < RAW_CHANS; i=i+1) begin
  assign muted_data[ SAMP_WIDTH * (i + 1) - 1 : SAMP_WIDTH * i ] = shuffle_data[ SAMP_WIDTH * (i + 1) - 1 : SAMP_WIDTH * i ];
end
endgenerate
assign muted_valid   = shuffle_valid;
assign muted_last    = shuffle_last;
assign muted_tag    = shuffle_tag;
assign shuffle_ready = muted_ready;

// Elastic buffer
axis_fifo #(.WIDTH(SAMP_WIDTH * RAW_CHANS + DAC_CMD_WIDTH + 1), .DEEP(4)) elastic_buffer (
  .clk(clk),
  .rst(rst),

  .s_rx_tdata({ muted_tag, muted_last, muted_data }),
  .s_rx_tvalid(muted_valid),
  .s_rx_tready(muted_ready),

  .m_tx_tdata({ buffer_tag, buffer_last, buffer_data }),
  .m_tx_tvalid(buffer_valid),
  .m_tx_tready(buffer_ready)
);


data_txsync_out #(
    .TIMESTAMP_BITS(TIMESTAMP_BITS),
    .TAG_WIDTH(DAC_CMD_WIDTH),
    .CH_COUNT(RAW_CHANS),
    .SAMP_WIDTH(SAMP_WIDTH),
    .ASYNC_CLK(ASYNC_CLK),
    .FRAME_LENGTH(FRAME_LENGTH),
    .WITH_NOTS_BIT(WITH_NOTS_BIT)
) data_txsync_out (
  .clk(clk),
  .rst(rst),

  `AXIS_RVD_PORT_CONN(s_descr_, descr_time_),
  `AXIS_RVDLT_PORT_CONN(s_in_,  buffer_),

  .m_abort_id(abort_id),
  .m_abort_valid(abort_valid),

  .sig_underrun(sig_underrun),

  // Output to DAC
  .m_dac_data(dac_data),
  .m_dac_valid(dac_valid),
  .m_dac_frame(dac_frame),  // LFMC clock to resync after CCs
  .m_dac_active(),          //////////////////////////////////////////////////
  .m_dac_ready(dac_ready),
  .m_dac_sync(dac_sync),    // crossing from 0->1 resets TX timer
  .m_dac_time(dac_time),

  .m_fedma_clk(m_fedma_clk),
  .m_fedma_rst(m_fedma_rst),
  .m_fedma_ts(m_fedma_ts)         //Current TS to discard old buffer requests
);


endmodule
