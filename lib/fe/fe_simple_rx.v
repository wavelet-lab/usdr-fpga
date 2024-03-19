// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
// USDR PROJECT
// CLEAN
//
module fe_simple_rx #(
   parameter BUFFER_SIZE_ADDR  = 16,
   parameter DATA_WIDTH        = 12,
   parameter ASYNC_BURST_CLOCK = 0,
   parameter RAW_CHANS         = 4
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
  output [3:0]                         o_ch_enabled, // raw chans enabled flags

  // RAM FIFO Interface
  output                               m_rxfe_tvalid,
  output [BUFFER_SIZE_ADDR-1:3]        m_rxfe_taddr,
  output [63:0]                        m_rxfe_tdata,
  output [7:0]                         m_rxfe_tkeep,
  output                               m_rxfe_tlast,

  // Notification of partial block readiness
  // 0 -  128b
  // 1 -  256b
  // 2 -  512b
  // 3 - 1024b
  input [1:0]                          cfg_mlowmrk,

  // FIFO control for DMA
  input                            fifo_burst_clk,
  input                            fifo_burst_rst,

  input [31:0]                     s_fifo_burst_cmd_data,
  input                            s_fifo_burst_cmd_valid,
  output                           s_fifo_burst_cmd_ready,

  input                            fifo_burst_timer_rst,

  output  [BUFFER_SIZE_ADDR - 6:0] fifo_burst_avail_z,
  input                            fifo_burst_release,

  output                           fifo_burst_skip,
  output                           fifo_burst_fill,

  output                           fifo_burst_mlowmrk
);

////////////////////////////////////////////////////////////////////////
// RXTimer
wire rxtimer_rst_sync;
wire clk_burst_rst;

////////////////////////////////////////////////////////////////////////
// Burster / Streamer
localparam FE_CMD_BURST_SAMPLES  = 4'h0;
localparam FE_CMD_BURST_FORMAT   = 4'h1;
localparam FE_CMD_BURST_THROTTLE = 4'h2;
localparam FE_CMD_RESET          = 4'h3;

localparam FMT_DSP   = 0;
localparam FMT_8BIT  = 1;
localparam FMT_12BIT = 2;
localparam FMT_16BIT = 3;


localparam MAX_BURST_BITS = BUFFER_SIZE_ADDR - 3;

reg [MAX_BURST_BITS-1:0]  burst_samples_sz;
reg                       burst_throttle;
reg [7:0]                 burst_num_send;
reg [7:0]                 burst_num_skip;

reg                       rx_stream_active;
reg [2:0]                 rx_stream_cmd;

localparam RX_SCMD_IDLE      = 0;
localparam RX_SCMD_START_AT  = 1;
localparam RX_SCMD_START_IMM = 2;
localparam RX_SCMD_STOP_AT   = 3;
localparam RX_SCMD_STOP_IMM  = 4;

assign o_running          = rx_stream_active;

wire [31:0] fe_cmd_route_data;
wire        fe_cmd_route_valid;
wire        fe_cmd_route_ready = 1'b1;

reg         burster_rst;
reg         enable;
reg         testpattern;

assign o_enable = ~enable;
assign o_tstp = testpattern;

reg [1:0]                        cfg_in_fmt;
reg [2:0]                        cfg_ch_fmt;

// Total number of bursts fit in RAM
reg [BUFFER_SIZE_ADDR - 6 - 1:0] cfg_rxclk_total;
// Number of QWORDs in each burst
reg [BUFFER_SIZE_ADDR-1:3]       cfg_brst_words_z;

always @(posedge clk) begin
  if (rst) begin
    burst_samples_sz <= 511;
    burst_throttle   <= 0;

    rx_stream_active <= 0;
    rx_stream_cmd    <= RX_SCMD_IDLE;

    burster_rst      <= 1'b1;
    enable           <= 1'b1;

    testpattern      <= 1'b0;
  end else begin

    if (~rxtimer_rst_sync) begin
      if ((rx_stream_cmd == RX_SCMD_START_AT  || rx_stream_cmd == RX_SCMD_STOP_AT) ||
          (rx_stream_cmd == RX_SCMD_START_IMM || rx_stream_cmd == RX_SCMD_STOP_IMM)) begin
          rx_stream_active   <= (rx_stream_cmd == RX_SCMD_START_AT || rx_stream_cmd == RX_SCMD_START_IMM);
          rx_stream_cmd      <= RX_SCMD_IDLE;
      end
    end

    if (fe_cmd_route_valid && fe_cmd_route_ready) begin
      case (fe_cmd_route_data[31:28])
        FE_CMD_BURST_SAMPLES: begin
          burst_samples_sz <= fe_cmd_route_data[MAX_BURST_BITS-1:0];
        end

        FE_CMD_BURST_FORMAT: begin
          cfg_in_fmt       <= fe_cmd_route_data[1:0];
          cfg_ch_fmt       <= fe_cmd_route_data[4:2];
          cfg_brst_words_z <= fe_cmd_route_data[MAX_BURST_BITS-1 + 5:5];
          cfg_rxclk_total  <= fe_cmd_route_data[BUFFER_SIZE_ADDR - 6 - 1 + MAX_BURST_BITS + 5:MAX_BURST_BITS + 5];
        end

        FE_CMD_BURST_THROTTLE: begin
          burst_num_skip   <= fe_cmd_route_data[7:0];
          burst_num_send   <= fe_cmd_route_data[15:8];
          burst_throttle   <= fe_cmd_route_data[16];
        end

        FE_CMD_RESET: begin
          burster_rst      <= fe_cmd_route_data[15];
          rx_stream_active <= rx_stream_active && ~fe_cmd_route_data[14];
          enable           <= fe_cmd_route_data[13];
          testpattern      <= fe_cmd_route_data[12];
          rx_stream_cmd    <= fe_cmd_route_data[2:0];
        end

      endcase
    end
  end
end

// Align to 16 bit
wire [DATA_WIDTH-1:0]     in_ai = s_in_data[DATA_WIDTH * (0 + 1) - 1:DATA_WIDTH * 0];
wire [DATA_WIDTH-1:0]     in_aq = s_in_data[DATA_WIDTH * (1 + 1) - 1:DATA_WIDTH * 1];
wire [DATA_WIDTH-1:0]     in_bi = s_in_data[DATA_WIDTH * (2 + 1) - 1:DATA_WIDTH * 2];
wire [DATA_WIDTH-1:0]     in_bq = s_in_data[DATA_WIDTH * (3 + 1) - 1:DATA_WIDTH * 3];
wire [15:0]               lin_ai = (DATA_WIDTH < 16) ? { in_ai, {(16-DATA_WIDTH){1'b0}} } : in_ai;
wire [15:0]               lin_aq = (DATA_WIDTH < 16) ? { in_aq, {(16-DATA_WIDTH){1'b0}} } : in_aq;
wire [15:0]               lin_bi = (DATA_WIDTH < 16) ? { in_bi, {(16-DATA_WIDTH){1'b0}} } : in_bi;
wire [15:0]               lin_bq = (DATA_WIDTH < 16) ? { in_bq, {(16-DATA_WIDTH){1'b0}} } : in_bq;
wire                      lin_valid = s_in_valid;
wire                      lin_ready;
assign                    s_in_ready = lin_ready;


reg [MAX_BURST_BITS-1:0]  fe_cur_sample;
wire [MAX_BURST_BITS:0]   fe_cur_sample_nxt = {1'b0, fe_cur_sample} - 1'b1;
reg [7:0]                 fe_cur_burst;
wire [8:0]                fe_cur_burst_nxt  = {1'b0, fe_cur_burst} - 1'b1;
reg                       fe_cur_burst_state;

always @(posedge clk) begin
  if (burster_rst) begin
    fe_cur_sample      <= burst_samples_sz;
    fe_cur_burst       <= 0;
    fe_cur_burst_state <= 0;
  end else begin
    if (lin_valid && rx_stream_active) begin
      fe_cur_sample <= fe_cur_sample_nxt;
      if (fe_cur_sample_nxt[MAX_BURST_BITS]) begin
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


wire burster_valid = lin_valid && (~burst_throttle || ~fe_cur_burst_state) && ~burster_rst && rx_stream_active;
wire burster_last  = (fe_cur_sample_nxt[MAX_BURST_BITS]);
wire burster_ready;

assign lin_ready = burster_ready || burster_rst || !rx_stream_active;

////////////////////////////////////////////////////////////////////////
// Serializer
wire [2:0] iq_mux_mode = cfg_ch_fmt;
reg        iq_mux_valid;
wire       iq_mux_ready;
reg        iq_mux_last;
reg [15:0] iq_mux_d0_i;
reg [15:0] iq_mux_d0_q;
reg [15:0] iq_mux_d1_i;
reg [15:0] iq_mux_d1_q;
reg [1:0]  pidx;

localparam [2:0]
    FMT_CH_3210 = 0,
    FMT_CH_xx10 = 1,
    FMT_CH_xxx0 = 2,
    FMT_CH_xx1x = 3,
    FMT_CH_x2x0 = 4,
    FMT_CH_32xx = 5,
    FMT_CH_x2xx = 6,
    FMT_CH_3xxx = 7;

assign o_ch_enabled =
    (iq_mux_mode == FMT_CH_3210) ? 4'b1111 :
    (iq_mux_mode == FMT_CH_xx10) ? 4'b0011 :
    (iq_mux_mode == FMT_CH_xxx0) ? 4'b0001 :
    (iq_mux_mode == FMT_CH_xx1x) ? 4'b0010 :
    (iq_mux_mode == FMT_CH_x2x0) ? 4'b0101 :
    (iq_mux_mode == FMT_CH_32xx) ? 4'b1100 :
    (iq_mux_mode == FMT_CH_x2xx) ? 4'b0100 : 4'b1000;


wire sample_burster_last_strobe =
    (iq_mux_mode == FMT_CH_3210) ? 1'b1 :
    (iq_mux_mode == FMT_CH_xx10 || iq_mux_mode == FMT_CH_32xx || iq_mux_mode == FMT_CH_x2x0) ? !pidx[0] : pidx != 3;
wire mux_valid_strobe =
    (iq_mux_mode == FMT_CH_3210) ? 1'b1 :
    (iq_mux_mode == FMT_CH_xx10 || iq_mux_mode == FMT_CH_32xx || iq_mux_mode == FMT_CH_x2x0) ? pidx[0] : pidx == 3;

assign burster_ready = iq_mux_ready || (~burster_last && sample_burster_last_strobe);


always @(posedge clk) begin
  if (clk_burst_rst) begin
    iq_mux_valid       <= 0;
    pidx               <= 0;
  end else if (burster_valid && burster_ready) begin
    if (burster_last) begin
      pidx <= 0;
    end else begin
      pidx <= pidx + 1'b1;
    end

    case (iq_mux_mode)
    FMT_CH_3210: begin
      iq_mux_d0_i <= lin_ai;
      iq_mux_d0_q <= lin_aq;
      iq_mux_d1_i <= lin_bi;
      iq_mux_d1_q <= lin_bq;
    end

    FMT_CH_xx10, FMT_CH_x2x0, FMT_CH_32xx: begin
      if (pidx[0] == 0) begin
        iq_mux_d0_i <= (iq_mux_mode == FMT_CH_32xx) ? lin_bi : lin_ai;
        iq_mux_d0_q <= (iq_mux_mode == FMT_CH_xx10) ? lin_aq : (iq_mux_mode == FMT_CH_x2x0) ? lin_bi : lin_bq;
      end else begin
        iq_mux_d1_i <= (iq_mux_mode == FMT_CH_32xx) ? lin_bi : lin_ai;
        iq_mux_d1_q <= (iq_mux_mode == FMT_CH_xx10) ? lin_aq : (iq_mux_mode == FMT_CH_x2x0) ? lin_bi : lin_bq;
      end
    end

    FMT_CH_xxx0, FMT_CH_xx1x, FMT_CH_x2xx, FMT_CH_3xxx: begin
      case (pidx)
      0: iq_mux_d0_i <= (iq_mux_mode == FMT_CH_xxx0) ? lin_ai : (iq_mux_mode == FMT_CH_xx1x) ? lin_aq : (iq_mux_mode == FMT_CH_x2xx) ? lin_bi : lin_bq;
      1: iq_mux_d1_i <= (iq_mux_mode == FMT_CH_xxx0) ? lin_ai : (iq_mux_mode == FMT_CH_xx1x) ? lin_aq : (iq_mux_mode == FMT_CH_x2xx) ? lin_bi : lin_bq;
      2: iq_mux_d0_q <= (iq_mux_mode == FMT_CH_xxx0) ? lin_ai : (iq_mux_mode == FMT_CH_xx1x) ? lin_aq : (iq_mux_mode == FMT_CH_x2xx) ? lin_bi : lin_bq;
      3: iq_mux_d1_q <= (iq_mux_mode == FMT_CH_xxx0) ? lin_ai : (iq_mux_mode == FMT_CH_xx1x) ? lin_aq : (iq_mux_mode == FMT_CH_x2xx) ? lin_bi : lin_bq;
      endcase
    end

    endcase
  end

  iq_mux_last  <= burster_last;
  iq_mux_valid <= burster_valid && (burster_last || mux_valid_strobe);
end


wire [15:0] mux_s0_8  = { iq_mux_d0_q[DATA_WIDTH-1:DATA_WIDTH-8],     iq_mux_d0_i[DATA_WIDTH-1:DATA_WIDTH-8] };
wire [15:0] mux_s1_8  = { iq_mux_d1_q[DATA_WIDTH-1:DATA_WIDTH-8],     iq_mux_d1_i[DATA_WIDTH-1:DATA_WIDTH-8] };

wire [23:0] mux_s0_12 = { iq_mux_d0_q[DATA_WIDTH-1:DATA_WIDTH-12],    iq_mux_d0_i[DATA_WIDTH-1:DATA_WIDTH-12] };
wire [23:0] mux_s1_12 = { iq_mux_d1_q[DATA_WIDTH-1:DATA_WIDTH-12],    iq_mux_d1_i[DATA_WIDTH-1:DATA_WIDTH-12] };

wire [31:0] mux_s0_16 = { iq_mux_d0_q,                                iq_mux_d0_i };
wire [31:0] mux_s1_16 = { iq_mux_d1_q,                                iq_mux_d1_i };


////////////////////////////////////////////////////////////////////////
// FIFO-RAM mapper
reg                           sig_brst_skip;
reg                           rts_mlowmrk;

wire [BUFFER_SIZE_ADDR - 6:0] rxclk_rptr;
wire [BUFFER_SIZE_ADDR - 6:0] rxclk_wptr;

wire [BUFFER_SIZE_ADDR - 6:0] fifo_burst_clk_rptr;
wire [BUFFER_SIZE_ADDR - 6:0] fifo_burst_clk_wptr;

wire [BUFFER_SIZE_ADDR - 6:0] fifo_burst_clk_ready_z = fifo_burst_clk_wptr - fifo_burst_clk_rptr - 1'b1;
wire [BUFFER_SIZE_ADDR - 6:0] rxclk_avail_z =
  { 1'b0, cfg_rxclk_total } + rxclk_rptr - rxclk_wptr - 1'b1;

wire rxclk_fifo_burst_full = rxclk_avail_z[BUFFER_SIZE_ADDR - 6];

assign fifo_burst_avail_z = fifo_burst_clk_ready_z;
assign fifo_burst_fill    = fifo_burst_clk_wptr[0];

wire rxclk_wrbuffer_done;
wire fifo_burst_clk_fe_rst;

axis_opt_cc_fifo #(.CLK_CC(ASYNC_BURST_CLOCK), .WIDTH(32)) cmd_queue (
  .rx_clk(fifo_burst_clk),
  .rx_rst(fifo_burst_rst),

  .s_rx_tdata(s_fifo_burst_cmd_data),
  .s_rx_tvalid(s_fifo_burst_cmd_valid),
  .s_rx_tready(s_fifo_burst_cmd_ready),

  .tx_clk(clk),
  .tx_rst(clk_burst_rst),

  .m_tx_tdata(fe_cmd_route_data),
  .m_tx_tvalid(fe_cmd_route_valid),
  .m_tx_tready(fe_cmd_route_ready)
);

generate
if (ASYNC_BURST_CLOCK == 0) begin
    reg [BUFFER_SIZE_ADDR - 6:0] rptr;
    reg [BUFFER_SIZE_ADDR - 6:0] wptr;
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
end else begin

    // RPT
    cc_counter #(
       .WIDTH(BUFFER_SIZE_ADDR - 6 + 1),
       .GRAY_BITS(2)
    ) ul_to_rx_addr (
       .clk(fifo_burst_clk),
       .in_rst(fifo_burst_rst),
       .in_increment(fifo_burst_release),
       .in_counter(fifo_burst_clk_rptr),

       .out_clk(clk),
       .out_rst(clk_burst_rst),
       .out_counter(rxclk_rptr)
    );

    // WPTR
    cc_counter #(
       .WIDTH(BUFFER_SIZE_ADDR - 6 + 1),
       .GRAY_BITS(2)
    ) rx_to_ul_addr (
       .clk(clk),
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
        .in(sig_brst_skip),
        .out(fifo_burst_skip)
    );

    synchronizer sig_mlowmrk(
        .clk(fifo_burst_clk),
        .rst(fifo_burst_rst),
        .in(rts_mlowmrk),
        .out(fifo_burst_mlowmrk)
    );

    synchronizer #(.INIT(1), .ASYNC_RESET(1)) farst_rxtimer (
        .clk(clk),
        .rst(rst),
        .a_in(fifo_burst_timer_rst),
        .s_out(rxtimer_rst_sync)
    );

    synchronizer #(.INIT(1'b1), .ASYNC_RESET(1)) farst_enable (
        .clk(clk),
        .rst(rst),
        .a_in(fifo_burst_rst),
        .s_out(clk_burst_rst)
    );
    end
endgenerate

// Number of QWORDs in each burst
reg [BUFFER_SIZE_ADDR-1:3] brst_cur_wcnt;
reg                        brst_skip;
reg [BUFFER_SIZE_ADDR-1:3] ram_baddr;

reg [1:0]  transmit_state;
wire [1:0] n_transmit_state = transmit_state + 1'b1;

wire iq_bword_valid =
  (cfg_in_fmt == FMT_8BIT)  ? iq_mux_valid && (transmit_state[0]   || iq_mux_last) :
  (cfg_in_fmt == FMT_12BIT) ? iq_mux_valid && (transmit_state != 0 || iq_mux_last) :
  /*(cfg_in_fmt == FMT_16BIT) ?*/ iq_mux_valid;

wire iq_bword_last = iq_mux_last;


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
    transmit_state <= 0;
    wmrk_prev      <= 0;
    rts_mlowmrk    <= 0;
  end else begin
    if (iq_mux_valid) begin
      transmit_state <= n_transmit_state;
    end

    rts_mlowmrk <= s_mlowmrk;

    if (iq_bword_valid) begin
      if (~brst_skip) begin
        brst_skip     <= rxclk_fifo_burst_full;
        brst_cur_wcnt <= brst_cur_wcnt + 1'b1;
      end

      if (iq_bword_last) begin
        transmit_state<= 0;
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

assign rxclk_wrbuffer_done = symbol_valid && symbol_last;

reg [BUFFER_SIZE_ADDR-1:3] axis_tx_taddr;
reg [63:0]                 axis_tx_tdata;
reg                        axis_tx_tvalid;
reg                        axis_tx_tlast;
reg [7:0]                  axis_tx_tkeep;

reg [31:0] tmp;
wire [1:0] nxt_transmit_state = transmit_state;

assign iq_mux_ready = ~clk_burst_rst;

always @(posedge clk) begin
  if (clk_burst_rst) begin
    axis_tx_tvalid <= 0;
  end else begin
    axis_tx_tvalid <= ~rxclk_fifo_burst_full && symbol_valid;
    axis_tx_tlast  <= symbol_last;

    if (symbol_valid) begin
      axis_tx_taddr <= ram_baddr + brst_cur_wcnt;
    end

    case (cfg_in_fmt)
      FMT_8BIT: begin
          if (nxt_transmit_state[0] == 0) begin
            axis_tx_tkeep        <= 8'h0f;
            axis_tx_tdata[15:0]  <= mux_s0_8;
            axis_tx_tdata[31:16] <= mux_s1_8;
          end else begin
            axis_tx_tkeep        <= 8'hff;
            axis_tx_tdata[47:32] <= mux_s0_8;
            axis_tx_tdata[63:48] <= mux_s1_8;
          end
      end

      FMT_12BIT: begin
        case (nxt_transmit_state)
          0: begin
            axis_tx_tkeep          <= 8'h3f;
            axis_tx_tdata[23:0]    <= mux_s0_12;
            axis_tx_tdata[47:24]   <= mux_s1_12;
          end

          1: begin
            axis_tx_tkeep          <= 8'hff;
            axis_tx_tdata[63:48]   <= mux_s0_12[15:0];
            tmp[31:0]              <= { mux_s1_12, mux_s0_12[23:16] };
          end

          2: begin
            axis_tx_tkeep          <= 8'hff;
            axis_tx_tdata[31:0]    <= tmp[31:0];
            axis_tx_tdata[55:32]   <= mux_s0_12;
            axis_tx_tdata[63:56]   <= mux_s1_12[7:0];
            if (iq_mux_valid)
              tmp[15:0]            <= mux_s1_12[23:8];
          end

          3: begin
            axis_tx_tkeep          <= 8'hff;
            axis_tx_tdata[15:0]    <= tmp[15:0];
            axis_tx_tdata[39:16]   <= mux_s0_12;
            axis_tx_tdata[63:40]   <= mux_s1_12;
          end
        endcase
      end

      FMT_DSP, FMT_16BIT: begin
        axis_tx_tkeep        <= 8'hff;
        axis_tx_tdata[31:0]  <= mux_s0_16;
        axis_tx_tdata[63:32] <= mux_s1_16;
      end
    endcase
  end
end


assign m_rxfe_tvalid = axis_tx_tvalid;
assign m_rxfe_taddr  = axis_tx_taddr;
assign m_rxfe_tdata  = axis_tx_tdata;
assign m_rxfe_tkeep  = axis_tx_tkeep;
assign m_rxfe_tlast  = axis_tx_tlast;

endmodule

