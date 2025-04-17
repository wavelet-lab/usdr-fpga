// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2025 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module fe_ext_rx #(
   parameter BUFFER_SIZE_ADDR  = 16,
   parameter DATA_WIDTH        = 16,
   parameter ASYNC_BURST_CLOCK = 0,
   parameter RAW_CHANS         = 4,
   parameter _DATA_BITS        = $clog2(RAW_CHANS) + 1,
   parameter _DATA_WIDTH       = 16 * RAW_CHANS,
   parameter PARAGRAPH_BITS    = 6,
   parameter NO_BACKPRESSURE_12 = 0,
   parameter NO_KEEP = 0
)(
  // High speed interface
  input                                rst,
  input                                clk,

  input [RAW_CHANS * DATA_WIDTH - 1:0] s_in_data,
  input                                s_in_valid,
  output                               s_in_ready,

  output                               o_enable,     // phy enable signal, e.g. for calibartion
  output                               o_tstp,       // phy test pattern gen, e.g. for calibration
  output                               o_running,    // Timer is active
  output reg [RAW_CHANS-1:0]           o_ch_enabled, // raw chans enabled flags

  // RAM FIFO Interface
  output                                   m_rxfe_tvalid,
  output [BUFFER_SIZE_ADDR-1:_DATA_BITS]   m_rxfe_taddr,
  output [_DATA_WIDTH-1:0]                 m_rxfe_tdata,
  output [_DATA_WIDTH/8-1:0]               m_rxfe_tkeep,
  output                                   m_rxfe_tlast,

  // Notification of partial block readiness
  // 0 -  128b
  // 1 -  256b
  // 2 -  512b
  // 3 - 1024b
  input [1:0]                              cfg_mlowmrk,

  // FIFO control for DMA
  input                                    fifo_burst_clk,
  input                                    fifo_burst_rst,

  // FE control
  input [31:0]                             s_fifo_burst_cmd_data,
  input                                    s_fifo_burst_cmd_valid,
  output                                   s_fifo_burst_cmd_ready,

  input                                    fifo_burst_timer_rst,

  output [BUFFER_SIZE_ADDR-PARAGRAPH_BITS:0] fifo_burst_avail_z,
  input                                      fifo_burst_release,

  output                                   fifo_burst_skip,
  output                                   fifo_burst_fill,

  output                                   fifo_burst_mlowmrk
);

`include "axi_helpers.vh"

////////////////////////////////////////////////////////////////////////
// RXTimer
wire rxtimer_rst_sync;
wire clk_burst_rst;

////////////////////////////////////////////////////////////////////////
// Burster / Streamer
localparam FE_CMD_BURST_SAMPLES  = 8'h00;
localparam FE_CMD_BURST_BYTES    = 8'h01;
localparam FE_CMD_BURST_CAPACITY = 8'h02;
localparam FE_CMD_COMPACTER      = 8'h03;
localparam FE_CMD_PACKER         = 8'h04;

localparam FE_CMD_ENABLE_EXT     = 8'h08;

// 0x10 through 0x1f shuffle configuration
localparam FE_CMD_SHUFFLE_0      = 8'h10;
localparam FE_CMD_SHUFFLE_1      = 8'h11;
localparam FE_CMD_SHUFFLE_2      = 8'h12;
localparam FE_CMD_SHUFFLE_3      = 8'h13;
localparam FE_CMD_SHUFFLE_4      = 8'h14;
localparam FE_CMD_SHUFFLE_5      = 8'h15;


localparam FE_CMD_BURST_THROTTLE = 8'h20;
localparam FE_CMD_RESET          = 8'h30;


localparam MAX_BURST = BUFFER_SIZE_ADDR;

reg [MAX_BURST-1:0]                  burst_samples_sz;
reg                                  burst_throttle;
reg [7:0]                            burst_num_send;
reg [7:0]                            burst_num_skip;

reg                                  rx_stream_active;
reg [2:0]                            rx_stream_cmd;

localparam RX_SCMD_IDLE      = 0;
localparam RX_SCMD_START_AT  = 1;
localparam RX_SCMD_START_IMM = 2;
localparam RX_SCMD_STOP_AT   = 3;
localparam RX_SCMD_STOP_IMM  = 4;

assign o_running          = rx_stream_active;


wire [7:0]  fe_cmd_route_addr;
wire [23:0] fe_cmd_route_data;
wire        fe_cmd_route_valid;
wire        fe_cmd_route_ready = 1'b1;

reg         burster_rst;
reg         enable;
reg         testpattern;

assign o_enable = ~enable;
assign o_tstp = testpattern;

// Total number of bursts fit in RAM
localparam MAX_BUSRTCNT_WIDTH = BUFFER_SIZE_ADDR - PARAGRAPH_BITS;
localparam MAX_BUSRTCNT_CNT   = (1 << MAX_BUSRTCNT_WIDTH) - 1'b1;


reg [MAX_BUSRTCNT_WIDTH - 1:0]            cfg_rxclk_total;
reg [BUFFER_SIZE_ADDR - 1:_DATA_BITS]     cfg_brst_words_z; //Burst size in bytes


localparam CHAN_STAGES = $clog2(RAW_CHANS);

reg [RAW_CHANS * CHAN_STAGES - 1:0]   cfg_shuffle;
reg [CHAN_STAGES - 1:0]               cfg_compact;
reg [1:0]                             cfg_packer;

always @(posedge clk) begin
  if (rst) begin
    burst_samples_sz <= 511;
    burst_throttle   <= 0;

    rx_stream_active <= 0;
    rx_stream_cmd    <= RX_SCMD_IDLE;

    burster_rst      <= 1'b1;
    enable           <= 1'b1;

    testpattern      <= 1'b0;

    o_ch_enabled     <= 0;
  end else begin

    if (~rxtimer_rst_sync && ~clk_burst_rst) begin
      if ((rx_stream_cmd == RX_SCMD_START_AT  || rx_stream_cmd == RX_SCMD_STOP_AT) ||
          (rx_stream_cmd == RX_SCMD_START_IMM || rx_stream_cmd == RX_SCMD_STOP_IMM)) begin
          rx_stream_active   <= (rx_stream_cmd == RX_SCMD_START_AT || rx_stream_cmd == RX_SCMD_START_IMM);
          rx_stream_cmd      <= RX_SCMD_IDLE;
      end
    end

    if (fe_cmd_route_valid && fe_cmd_route_ready) begin
      case (fe_cmd_route_addr)
        FE_CMD_BURST_SAMPLES: begin
          burst_samples_sz <= fe_cmd_route_data;
        end

        FE_CMD_BURST_BYTES: begin
          cfg_brst_words_z <= fe_cmd_route_data[BUFFER_SIZE_ADDR-1:_DATA_BITS];
        end

        FE_CMD_BURST_CAPACITY: begin
          cfg_rxclk_total  <= fe_cmd_route_data > MAX_BUSRTCNT_CNT ? MAX_BUSRTCNT_CNT : fe_cmd_route_data;
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

        FE_CMD_COMPACTER: begin
           cfg_compact <= fe_cmd_route_data;
        end

        FE_CMD_PACKER: begin
           cfg_packer  <= fe_cmd_route_data;
        end

        FE_CMD_ENABLE_EXT: begin
           o_ch_enabled  <= fe_cmd_route_data;
        end

        FE_CMD_BURST_THROTTLE: begin
          burst_num_skip   <= fe_cmd_route_data[7:0];
          burst_num_send   <= fe_cmd_route_data[15:8];
          burst_throttle   <= fe_cmd_route_data[16];
        end

        FE_CMD_RESET: begin
          burster_rst      <= fe_cmd_route_data[15];
          rx_stream_active <= rx_stream_active && ~fe_cmd_route_data[14] && ~clk_burst_rst; //Enable streaming only when DMA is active
          enable           <= fe_cmd_route_data[13];
          testpattern      <= fe_cmd_route_data[12];
          rx_stream_cmd    <= fe_cmd_route_data[2:0];
        end

      endcase
    end
  end
end

reg  [MAX_BURST-1:0]      fe_cur_sample;
wire [MAX_BURST:0]        fe_cur_sample_nxt = {1'b0, fe_cur_sample} - 1'b1;
reg  [7:0]                fe_cur_burst;
wire [8:0]                fe_cur_burst_nxt  = {1'b0, fe_cur_burst} - 1'b1;
reg                       fe_cur_burst_state;

always @(posedge clk) begin
  if (burster_rst) begin
    fe_cur_sample      <= burst_samples_sz;
    fe_cur_burst       <= 0;
    fe_cur_burst_state <= 0;
  end else begin
    if (s_in_ready && s_in_valid && rx_stream_active) begin
      fe_cur_sample <= fe_cur_sample_nxt;
      if (fe_cur_sample_nxt[MAX_BURST]) begin
        fe_cur_sample <= burst_samples_sz;

        if (burst_throttle) begin
          fe_cur_burst <= fe_cur_burst - 1;

          if (fe_cur_burst_nxt[8]) begin
            fe_cur_burst_state <= ~fe_cur_burst_state;
            if (fe_cur_burst_state == 0) begin
              fe_cur_burst <= burst_num_skip;
            end else begin
              fe_cur_burst <= burst_num_send;
            end
          end
        end
      end
    end
  end
end

localparam TOTAL_IN_WIDTH = RAW_CHANS * DATA_WIDTH;

wire [TOTAL_IN_WIDTH - 1:0] burster_data = s_in_data;
wire burster_valid = s_in_valid && (~burst_throttle || ~fe_cur_burst_state) && ~burster_rst && rx_stream_active;
wire burster_last  = (fe_cur_sample_nxt[MAX_BURST]);
wire burster_ready;

assign s_in_ready = (burster_ready) || burster_rst || !rx_stream_active;

////////////////////////////////////////////////////////////////////////
// Serializer

`DEFINE_AXIS_RVDL_PORT(shuffle_, TOTAL_IN_WIDTH);                  // shuffle output
`DEFINE_AXIS_RVDLK_PORT(compact_, TOTAL_IN_WIDTH, RAW_CHANS);      // compacter output
`DEFINE_AXIS_RVDLK_PORT(iq_bword_, TOTAL_IN_WIDTH, 2 * RAW_CHANS); // packer output

data_shuffle #(.DATA_WIDTH(DATA_WIDTH), .CH_COUNT(RAW_CHANS), .STAGE_PIPELINE(8'b10101010)) data_shuffle (
  .rst(burster_rst),
  .clk(clk),

  .cfg(cfg_shuffle),

  `AXIS_RVDL_PORT_CONN(s_in_,  burster_),
  `AXIS_RVDL_PORT_CONN(m_out_, shuffle_)
);

data_compacter #(.DATA_WIDTH(DATA_WIDTH), .CH_COUNT(RAW_CHANS)) data_compacter (
  .rst(burster_rst),
  .clk(clk),

  .cfg_compact(cfg_compact),

  `AXIS_RVDL_PORT_CONN(s_in_,  shuffle_),
  `AXIS_RVDLK_PORT_CONN(m_out_, compact_)
);


data_packer #(.DATA_WIDTH(DATA_WIDTH), .CH_COUNT(RAW_CHANS), .NO_BACKPRESSURE(NO_BACKPRESSURE_12)) data_packer (
  .rst(burster_rst),
  .clk(clk),

  .cfg_mode_12(cfg_packer[0]),
  .cfg_last_12_extra(cfg_packer[1]),

  `AXIS_RVDLK_PORT_CONN(s_in_,  compact_),
  `AXIS_RVDLK_PORT_CONN(m_out_, iq_bword_)
);


////////////////////////////////////////////////////////////////////////
// FIFO-RAM mapper
reg                           sig_brst_skip;
reg                           rts_mlowmrk;

wire [MAX_BUSRTCNT_WIDTH:0] rxclk_rptr;
wire [MAX_BUSRTCNT_WIDTH:0] rxclk_wptr;

wire [MAX_BUSRTCNT_WIDTH:0] fifo_burst_clk_rptr;
wire [MAX_BUSRTCNT_WIDTH:0] fifo_burst_clk_wptr;

wire [MAX_BUSRTCNT_WIDTH:0] fifo_burst_clk_ready_z = fifo_burst_clk_wptr - fifo_burst_clk_rptr - 1'b1;
wire [MAX_BUSRTCNT_WIDTH:0] rxclk_avail_z =
  { 1'b0, cfg_rxclk_total } + rxclk_rptr - rxclk_wptr - 1'b1;

wire rxclk_fifo_burst_full = rxclk_avail_z[MAX_BUSRTCNT_WIDTH];

assign fifo_burst_avail_z = fifo_burst_clk_ready_z;
assign fifo_burst_fill    = fifo_burst_clk_wptr[0];

wire rxclk_wrbuffer_done;
wire fifo_burst_clk_fe_rst;
wire rst_to_fifo_burst_clk;

axis_opt_cc_fifo #(.CLK_CC(ASYNC_BURST_CLOCK), .WIDTH(32)) cmd_queue (
  .rx_clk(fifo_burst_clk),
  .rx_rst(/*fifo_burst_rst*/ rst_to_fifo_burst_clk),

  .s_rx_tdata(s_fifo_burst_cmd_data),
  .s_rx_tvalid(s_fifo_burst_cmd_valid),
  .s_rx_tready(s_fifo_burst_cmd_ready),

  .tx_clk(clk),
  .tx_rst(/* clk_burst_rst */ rst),

  .m_tx_tdata({ fe_cmd_route_addr, fe_cmd_route_data }),
  .m_tx_tvalid(fe_cmd_route_valid),
  .m_tx_tready(fe_cmd_route_ready)
);


generate
if (ASYNC_BURST_CLOCK == 0) begin
    reg [MAX_BUSRTCNT_WIDTH:0] rptr;
    reg [MAX_BUSRTCNT_WIDTH:0] wptr;
    assign rxclk_rptr          = rptr;
    assign fifo_burst_clk_rptr = rptr;
    assign rxclk_wptr          = wptr;
    assign fifo_burst_clk_wptr = wptr;

    always @(posedge clk) begin
        if (clk_burst_rst) begin
            rptr <= 0;
            wptr <= 0;
        end else begin
            if (fifo_burst_release)
                rptr <= rptr + 1'b1;

            if (rxclk_wrbuffer_done)
                wptr <= wptr + 1'b1;
        end
    end

    assign fifo_burst_skip       = sig_brst_skip;
    assign fifo_burst_mlowmrk    = rts_mlowmrk;
    assign rxtimer_rst_sync      = fifo_burst_timer_rst;
    assign clk_burst_rst         = fifo_burst_rst;
    assign rst_to_fifo_burst_clk = rst;
end else begin

    // RPT
    cc_counter #(
       .WIDTH(MAX_BUSRTCNT_WIDTH + 1),
       .GRAY_BITS(2)
    ) ul_to_rx_addr (
       .in_clk(fifo_burst_clk),
       .in_rst(fifo_burst_rst),
       .in_increment(fifo_burst_release),
       .in_counter(fifo_burst_clk_rptr),

       .out_clk(clk),
       .out_rst(clk_burst_rst),
       .out_counter(rxclk_rptr)
    );

    // WPTR
    cc_counter #(
       .WIDTH(MAX_BUSRTCNT_WIDTH + 1),
       .GRAY_BITS(2)
    ) rx_to_ul_addr (
       .in_clk(clk),
       .in_rst(clk_burst_rst),
       .in_increment(rxclk_wrbuffer_done),
       .in_counter(rxclk_wptr),

       .out_clk(fifo_burst_clk),
       .out_rst(fifo_burst_rst),
       .out_counter(fifo_burst_clk_wptr)
    );

    synchronizer sig_skip(
        .clk(fifo_burst_clk),
        .rst(fifo_burst_rst),
        .a_in(sig_brst_skip),
        .s_out(fifo_burst_skip)
    );

    synchronizer sig_mlowmrk(
        .clk(fifo_burst_clk),
        .rst(fifo_burst_rst),
        .a_in(rts_mlowmrk),
        .s_out(fifo_burst_mlowmrk)
    );

    synchronizer #(.INIT(1'b1) /*, .ASYNC_RESET(1) */) farst_rxtimer (
        .clk(clk),
        .rst(rst),
        .a_in(fifo_burst_timer_rst),
        .s_out(rxtimer_rst_sync)
    );

    synchronizer #(.INIT(1'b1) /*, .ASYNC_RESET(1) */) farst_enable (
        .clk(clk),
        .rst(rst),
        .a_in(fifo_burst_rst),
        .s_out(clk_burst_rst)
    );

    synchronizer #(.INIT(1'b1) /*, .ASYNC_RESET(1) */) rst_to_fifo_burst_sync (
        .clk(fifo_burst_clk),
        .rst(1'b0),
        .a_in(rst),
        .s_out(rst_to_fifo_burst_clk)
    );
    end
endgenerate

// Number of QWORDs in each burst
reg [BUFFER_SIZE_ADDR-1:_DATA_BITS ] brst_cur_wcnt;
reg                                  brst_skip;
reg [BUFFER_SIZE_ADDR-1:_DATA_BITS]  ram_baddr;

reg wmrk_prev;
wire brst_cur_wcnt_rbit =
  (cfg_mlowmrk == 2'b00) ? brst_cur_wcnt[7] :
  (cfg_mlowmrk == 2'b01) ? brst_cur_wcnt[8] :
  (cfg_mlowmrk == 2'b10) ? brst_cur_wcnt[9] : brst_cur_wcnt[10];

wire s_mlowmrk = brst_cur_wcnt_rbit ^ wmrk_prev;



always @(posedge clk) begin
  if (clk_burst_rst) begin
    ram_baddr      <= 0;
    brst_skip      <= 0;
    brst_cur_wcnt  <= 0;
    sig_brst_skip  <= 0;
    wmrk_prev      <= 0;
    rts_mlowmrk    <= 0;
  end else begin

    rts_mlowmrk <= s_mlowmrk;

    if (iq_bword_valid) begin
      if (~brst_skip) begin
        brst_skip     <= rxclk_fifo_burst_full;
        brst_cur_wcnt <= brst_cur_wcnt + 1'b1;
      end

      if (iq_bword_last) begin
        brst_skip     <= rxclk_fifo_burst_full;
        sig_brst_skip <= sig_brst_skip ^ (brst_skip);

        brst_cur_wcnt <= 0;
        wmrk_prev     <= s_mlowmrk;
        if (~brst_skip) begin
          ram_baddr <= ram_baddr + cfg_brst_words_z + 1'b1;
        end
      end
    end
  end
end

////////////////////////////////////////////////////////////////////////
// Stream packer

wire symbol_valid = ~(brst_skip) && (iq_bword_valid);
wire symbol_last  = iq_bword_last;

assign rxclk_wrbuffer_done =  symbol_valid && symbol_last;

reg [BUFFER_SIZE_ADDR-1:_DATA_BITS] axis_tx_taddr;
reg [_DATA_WIDTH-1:0]               axis_tx_tdata;
reg [_DATA_WIDTH/8-1:0]             axis_tx_tkeep;
reg                                 axis_tx_tvalid;
reg                                 axis_tx_tlast;

assign iq_bword_ready = 1'b1;

always @(posedge clk) begin
  if (clk_burst_rst) begin
    axis_tx_tvalid <= 0;
  end else begin
    axis_tx_tvalid  <= ~rxclk_fifo_burst_full && symbol_valid;
    axis_tx_tlast   <= symbol_last;

    if (symbol_valid) begin
      axis_tx_taddr <= ram_baddr + brst_cur_wcnt;
    end

    axis_tx_tdata   <= iq_bword_data;
    axis_tx_tkeep   <= iq_bword_keep;
  end
end


assign m_rxfe_tvalid = axis_tx_tvalid;
assign m_rxfe_taddr  = axis_tx_taddr;
assign m_rxfe_tdata  = axis_tx_tdata;
assign m_rxfe_tkeep  = NO_KEEP ? {(_DATA_WIDTH/8){1'b1}} : axis_tx_tkeep;
assign m_rxfe_tlast  = axis_tx_tlast;

endmodule

