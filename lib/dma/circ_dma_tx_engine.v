// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module circ_dma_tx_engine #(
    parameter TIMESTAMP_BITS = 48,
    parameter TIMESTAMP_LATE_DISCARD = 1,
    parameter WITH_NOTS_BIT = 1,
    parameter RAM_ADDR_WIDTH = 18,
    parameter DATA_BITS = 3,
    parameter DTAA_WIDTH = 8 << DATA_BITS,
    // parameter FE_IDX_BITS = 5,
    parameter SAMPLES_WIDTH = RAM_ADDR_WIDTH - 1,
    parameter BURSTS_BITS = 5,
    parameter BUS_ADDR_WIDTH = 32,
    parameter USER_BUF_BITS = 6,
    parameter ULTRA_SCALE = 1,
    parameter CHECK_BUFFER_REQ_SIZE = 1'b0,
    parameter [31:0] DEFAULT_MAX_BUFFER_SIZE = 0,
    parameter [0:0]  TX_EX_CORE = 1'b0,
    parameter        TX_LATE_WIDTH = 16,
    parameter        FE_DESCR_WIDTH = TIMESTAMP_BITS + SAMPLES_WIDTH + (RAM_ADDR_WIDTH - DATA_BITS) + 2 * TX_EX_CORE,
    parameter _USER_BUF_CONFIG_WIDTH = TIMESTAMP_BITS + BURSTS_BITS + SAMPLES_WIDTH + RAM_ADDR_WIDTH
) (
    input clk,
    input rst,

    // DMA configuration
    input [USER_BUF_BITS:0]    s_dmacfg_waddr,
    input [31:0]               s_dmacfg_wdata,
    input                      s_dmacfg_wvalid,
    output                     s_dmacfg_wready,

    // Buffer post  TS | bursts_cnt | length_sps | length_bytes (per burst)
    input                                  s_bufcmd_valid,
    output                                 s_bufcmd_ready,
    input [_USER_BUF_CONFIG_WIDTH - 1 : 0] s_bufcmd_data,

    input        s_control_valid,
    output       s_control_ready,
    input  [0:0] s_control_data,

    // All initialization are done
    output       core_ready,
    output reg   stat_dmareq_ovf,

    // Stats & interrupts
    output [31:0]                        m_dma_stat_data,
    output [31:0]                        m_dma_tsl_data, // Current TS (for latency measurement)
    output reg [TX_LATE_WIDTH - 1:0]     m_dma_late,
    output [RAM_ADDR_WIDTH:8]            m_dma_fifo_avail,

    output reg        m_int_valid,
    input             m_int_ready,

    //////////////////////////////////////////////////
    // interface to dma_buff
    output     [RAM_ADDR_WIDTH:DATA_BITS]       m_rq_loc_addr,
    output reg [BUS_ADDR_WIDTH-1:DATA_BITS]     m_rq_bus_addr,
    output     [RAM_ADDR_WIDTH-1:DATA_BITS]     m_rq_length,
    output reg                                  m_rq_valid,
    output      [USER_BUF_BITS+BURSTS_BITS-1:0] m_rq_tag,
    input                                       m_rq_ready,

    // competion
    input  [USER_BUF_BITS+BURSTS_BITS-1:0]      m_rc_tag,
    input                                       m_rc_valid,
    output                                      m_rc_ready,

    //////////////////////////////////////////////
    // Interface to FE
    input  [TIMESTAMP_BITS - 1:0]        s_fedma_ts,         //Current TS to discard old buffer requests
    input  [RAM_ADDR_WIDTH:8]            s_fedma_ram_addr,   //Determines availability of RAM

    // Burst processed and is ready to be reused
    input                                s_proc_idx_valid,
    output                               s_proc_idx_ready,

    input                                m_descr_ready,
    output                               m_descr_valid,
    output [FE_DESCR_WIDTH-1:0]          m_descr_data
);
// TODO: add handling of late buffers


localparam BUFCMD_BYTES_OFF   = 0;
localparam BUFCMD_SAMPLES_OFF = BUFCMD_BYTES_OFF   + RAM_ADDR_WIDTH;
localparam BUFCMD_BURSTS_OFF  = BUFCMD_SAMPLES_OFF + SAMPLES_WIDTH;
localparam BUFCMD_TS_OFF      = BUFCMD_BURSTS_OFF  + BURSTS_BITS;
localparam BUFCMD_RB_OFF      = BUFCMD_TS_OFF      + TIMESTAMP_BITS;

wire [TIMESTAMP_BITS-1:0]         bufcmd_data_ts      = s_bufcmd_data[BUFCMD_RB_OFF - 1         :BUFCMD_TS_OFF];
wire [BURSTS_BITS-1:0]            bufcmd_data_bursts  = s_bufcmd_data[BUFCMD_TS_OFF - 1         :BUFCMD_BURSTS_OFF];
wire [SAMPLES_WIDTH-1:0]          bufcmd_data_samples = s_bufcmd_data[BUFCMD_BURSTS_OFF - 1     :BUFCMD_SAMPLES_OFF];
wire [RAM_ADDR_WIDTH-1:0]         bufcmd_data_bytes   = s_bufcmd_data[BUFCMD_SAMPLES_OFF - 1    :BUFCMD_BYTES_OFF];

// BufferNO posted
// BufferNO requested
// BufferNO completed
// BufferNO aired
localparam BUF_COUNT = 1 << USER_BUF_BITS;
reg [USER_BUF_BITS:0] usrbuf_posted;
reg [USER_BUF_BITS:0] usrbuf_requested;
reg [USER_BUF_BITS:0] usrbuf_completed;
reg [USER_BUF_BITS:0] usrbuf_aired;

reg                                            dma_rst;
wire [31:12]                                   dma_raddr;
reg  [BURSTS_BITS + RAM_ADDR_WIDTH:DATA_BITS]  dma_max_len; // Maximum request len per buffer

always @(posedge clk) begin
    if (rst) begin
        dma_max_len <= DEFAULT_MAX_BUFFER_SIZE[BURSTS_BITS + RAM_ADDR_WIDTH:DATA_BITS];
    end else begin
        if (s_dmacfg_wvalid && s_dmacfg_wready && s_dmacfg_waddr[USER_BUF_BITS] == 1'b1) begin
            if (s_dmacfg_waddr[USER_BUF_BITS-1:0] == 0) begin
                dma_max_len <= s_dmacfg_wdata[BURSTS_BITS + RAM_ADDR_WIDTH:DATA_BITS];
            end
        end
    end
end

assign s_dmacfg_wready = 1'b1;
ram_sxp #(.DATA_WIDTH(32-12), .ADDR_WIDTH(USER_BUF_BITS), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b1)) dma_addrs (
    .wclk(clk),
    .we(s_dmacfg_wvalid && (s_dmacfg_waddr[USER_BUF_BITS] == 0)),
    .waddr(s_dmacfg_waddr[USER_BUF_BITS-1:0]),
    .wdata(s_dmacfg_wdata[31:12]),
    .raddr( usrbuf_requested[USER_BUF_BITS-1:0] ),
    .rdata(dma_raddr)
);

wire [USER_BUF_BITS:0] can_post_diff = BUF_COUNT + usrbuf_aired - usrbuf_posted - 1'b1;
wire                   user_bufs_busy = can_post_diff[USER_BUF_BITS]; // Set to 1 when we can't accept more buffers

wire [SAMPLES_WIDTH-1:0]          nxtbuf_samples;
wire [TIMESTAMP_BITS-1:0]         nxtbuf_ts;
wire [BURSTS_BITS-1:0]            nxtbuf_bursts;
wire [RAM_ADDR_WIDTH-1:DATA_BITS] nxtbuf_bytes;
wire                              nxtbuf_valid;
wire                              nxtbuf_ready;

wire                             user_buffers_post_ready;
wire                             user_buffers_cpl_ready;
assign                           s_bufcmd_ready = user_buffers_post_ready && user_buffers_cpl_ready;

axis_fifo #(.WIDTH(BURSTS_BITS + (RAM_ADDR_WIDTH - DATA_BITS) + (TIMESTAMP_LATE_DISCARD ? SAMPLES_WIDTH + TIMESTAMP_BITS : 0)), .DEEP(BUF_COUNT), .EXTRA_REG(1'b0)) user_buffers_post (
  .clk(clk),
  .rst(dma_rst),

  .s_rx_tdata({ bufcmd_data_samples, bufcmd_data_ts, bufcmd_data_bursts, bufcmd_data_bytes[RAM_ADDR_WIDTH-1:DATA_BITS] }),
  .s_rx_tvalid(s_bufcmd_valid && user_buffers_cpl_ready),
  .s_rx_tready(user_buffers_post_ready),

  .m_tx_tdata({ nxtbuf_samples, nxtbuf_ts, nxtbuf_bursts, nxtbuf_bytes }),
  .m_tx_tvalid(nxtbuf_valid),
  .m_tx_tready(nxtbuf_ready)
);

localparam BLWABITS = (TX_EX_CORE) ? 0 : DATA_BITS;
wire [RAM_ADDR_WIDTH-1:BLWABITS]  cplbuf_bytes;
wire [BURSTS_BITS-1:0]            cplbuf_bursts;
wire [TIMESTAMP_BITS-1:0]         cplbuf_ts;
wire [SAMPLES_WIDTH-1:0]          cplbuf_samples;
wire                              cplbuf_valid;
wire                              cplbuf_ready;

axis_fifo #(.WIDTH((RAM_ADDR_WIDTH - BLWABITS) + BURSTS_BITS + TIMESTAMP_BITS + SAMPLES_WIDTH), .DEEP(BUF_COUNT)) user_buffers_cpl (
  .clk(clk),
  .rst(dma_rst),

  .s_rx_tdata({ bufcmd_data_bytes[RAM_ADDR_WIDTH-1:BLWABITS], bufcmd_data_bursts, bufcmd_data_ts, bufcmd_data_samples }),
  .s_rx_tvalid(s_bufcmd_valid && user_buffers_post_ready),
  .s_rx_tready(user_buffers_cpl_ready),

  .m_tx_tdata({ cplbuf_bytes, cplbuf_bursts, cplbuf_ts, cplbuf_samples }),
  .m_tx_tvalid(cplbuf_valid),
  .m_tx_tready(cplbuf_ready)
);

reg [BURSTS_BITS-1:0]  nxtbuf_cbrst_bursts;
reg [BURSTS_BITS-1:0]  cur_burst_idx;
wire                   last_busrt_req = (cur_burst_idx == nxtbuf_cbrst_bursts);
// Requester logic
// 1 circ_buff => up to N dma_tx_pcie reqs
reg skip_rq_buf_r;
wire skip_rq_buf = skip_rq_buf_r && TIMESTAMP_LATE_DISCARD;

reg [RAM_ADDR_WIDTH-1:DATA_BITS] nxtbuf_cbrst_bytes;

assign m_rq_length  = nxtbuf_cbrst_bytes; //nxtbuf_bytes;
assign m_rq_tag     = { usrbuf_requested[USER_BUF_BITS - 1:0], cur_burst_idx[BURSTS_BITS-1:0] };

reg  [RAM_ADDR_WIDTH:DATA_BITS] m_rq_loc_addr_e;
wire [RAM_ADDR_WIDTH:DATA_BITS] m_rq_loc_addr_e_nxt = m_rq_loc_addr_e + m_rq_length + 1'b1;

assign                         m_rq_loc_addr = m_rq_loc_addr_e[RAM_ADDR_WIDTH:DATA_BITS];

wire [RAM_ADDR_WIDTH:8]        loc_addr_available = s_fedma_ram_addr - m_rq_loc_addr_e_nxt[RAM_ADDR_WIDTH:8] + ((1<<(RAM_ADDR_WIDTH-8)) - 1'b1);
wire                           loc_addr_busy      = loc_addr_available[RAM_ADDR_WIDTH];
assign m_dma_fifo_avail = loc_addr_available;

reg                                             load_next_dma_addr;
reg  [BURSTS_BITS + RAM_ADDR_WIDTH-1:DATA_BITS] buffer_check_len;
wire [BURSTS_BITS + RAM_ADDR_WIDTH  :DATA_BITS] buffer_check_len_nxt = buffer_check_len - m_rq_length;

reg                           nxtbuf_cbrst_nts;
reg [TIMESTAMP_BITS-1:0]      nxtbuf_cbrst_ts;
reg [SAMPLES_WIDTH-1:0]       nxtbuf_cbrst_samples;
wire [TIMESTAMP_BITS:0]       nxtbuf_timestamp_diff  = nxtbuf_cbrst_ts - s_fedma_ts;
wire                          nxtbuf_timestamp_late  = nxtbuf_timestamp_diff[TIMESTAMP_BITS] && (!WITH_NOTS_BIT || !nxtbuf_cbrst_nts) && TIMESTAMP_LATE_DISCARD;

// Buffer reload cycle
reg                           buf_updated;  // Need a cycle to load new DMA buffer address
reg                           ovf_updated;  // Need a cycle to update m_rq_loc_addr_e and check loc_addr_busy (RAM availability)
assign nxtbuf_ready  = buf_updated;

wire                                 skip_rq_ready;
wire                                 skip_rc_valid_fifo;
wire                                 skip_rc_valid = skip_rc_valid_fifo && TIMESTAMP_LATE_DISCARD;
wire                                 skip_rc_ready;
wire [USER_BUF_BITS+BURSTS_BITS-1:0] skip_rc_tag;


axis_opt_pipeline #( .WIDTH(USER_BUF_BITS + BURSTS_BITS) ) skip_pipe (
  .clk(clk),
  .rst(dma_rst),

  .s_rx_tdata( m_rq_tag ),
  .s_rx_tvalid( skip_rq_buf_r ),
  .s_rx_tready( skip_rq_ready ),

  .m_tx_tdata( skip_rc_tag ),
  .m_tx_tvalid( skip_rc_valid_fifo ),
  .m_tx_tready( skip_rc_ready )
);

reg [39:0] stat_total_req;
reg [39:0] stat_total_dis;


always @(posedge clk) begin
    if (dma_rst) begin
        stat_total_dis <= 0;
        stat_total_req <= 0;

        usrbuf_posted    <= 0;
        usrbuf_requested <= 0;

        cur_burst_idx    <= 0;

        m_rq_loc_addr_e  <= 0;
        m_rq_valid       <= 1'b0;

        buf_updated      <= 1'b1;
        ovf_updated      <= 1'b1;

        stat_dmareq_ovf  <= 1'b0;
        skip_rq_buf_r    <= 1'b0;

        m_dma_late       <= 0;
    end else begin
        ovf_updated      <= 1'b0;
        skip_rq_buf_r    <= 1'b0;

        if (s_bufcmd_valid && s_bufcmd_ready) begin
            usrbuf_posted    <= usrbuf_posted + 1'b1;
        end

        if (m_rq_valid && m_rq_ready || skip_rq_buf) begin
            m_rq_valid         <= 1'b0;
            if (!skip_rq_buf) begin
                m_rq_loc_addr_e    <= m_rq_loc_addr_e_nxt;
            end
            m_rq_bus_addr      <= m_rq_bus_addr   + m_rq_length + 1'b1;
            cur_burst_idx      <= cur_burst_idx   + 1'b1;

            buffer_check_len   <= buffer_check_len_nxt;

            if (skip_rq_buf) begin
                m_dma_late     <= m_dma_late + 1'b1;
                stat_total_dis <= stat_total_dis + 1'b1;
            end

            stat_total_req  <= stat_total_req + 1'b1;
            nxtbuf_cbrst_ts <= nxtbuf_cbrst_ts + nxtbuf_cbrst_samples + 1'b1;

            if (last_busrt_req) begin
                usrbuf_requested <= usrbuf_requested + 1'b1;
                buf_updated      <= 1'b1;
            end
        end

        if (!buf_updated && !ovf_updated) begin
            if ((!CHECK_BUFFER_REQ_SIZE || !loc_addr_busy) && (!m_rq_valid)) begin
                if (nxtbuf_timestamp_late) begin
                    if (skip_rq_ready) begin
                        skip_rq_buf_r      <= 1'b1;
                        ovf_updated        <= 1'b1;
                    end
                end else begin
                    // Check DMA buffer has enough length
                    m_rq_valid         <= !buffer_check_len_nxt[BURSTS_BITS + RAM_ADDR_WIDTH];
                    ovf_updated        <= 1'b1;

                    stat_dmareq_ovf    <= buffer_check_len_nxt[BURSTS_BITS + RAM_ADDR_WIDTH];
                end
            end
        end

        // We're in buf_updated until we get a valid buffer
        if (buf_updated) begin
            buf_updated          <= !nxtbuf_valid;
            nxtbuf_cbrst_nts     <= nxtbuf_ts[TIMESTAMP_BITS - 1];
            nxtbuf_cbrst_ts      <= nxtbuf_ts;
            nxtbuf_cbrst_samples <= nxtbuf_samples;
            nxtbuf_cbrst_bytes   <= nxtbuf_bytes;
            nxtbuf_cbrst_bursts  <= nxtbuf_bursts;

            m_rq_bus_addr        <= { dma_raddr, {(12 - DATA_BITS){1'b0}} };
            buffer_check_len     <= dma_max_len;
            cur_burst_idx        <= 0;
        end

    end
end

localparam BURST_COUNT = 1 << BURSTS_BITS;

reg [USER_BUF_BITS:0] bursts_reset_idx;
wire                  bursts_reset_ready = bursts_reset_idx[USER_BUF_BITS];
always @(posedge clk) begin
    if (dma_rst) begin
        bursts_reset_idx <= 0;
    end else begin
        if (!bursts_reset_ready) begin
            bursts_reset_idx <= bursts_reset_idx + 1'b1;
        end
    end
end

reg                                   cpl_state_reset_buf;
wire                                  cpl_state_reset_buf_ready;

wire [USER_BUF_BITS-1:0] completion_buf_idx     = m_rc_tag[USER_BUF_BITS + BURSTS_BITS - 1:BURSTS_BITS];
wire [USER_BUF_BITS-1:0] skp_completion_buf_idx = skip_rc_tag[USER_BUF_BITS + BURSTS_BITS - 1:BURSTS_BITS];

wire [BURST_COUNT-1:0]   current_completion;
wire [BURST_COUNT-1:0]   current_discarded;

wire [BURST_COUNT-1:0]   buffer_fin             = 1 << m_rc_tag[BURSTS_BITS-1:0];
wire [BURST_COUNT-1:0]   skp_buffer_fin         = 1 << skip_rc_tag[BURSTS_BITS-1:0];

assign m_rc_ready                = 1'b1;
assign skip_rc_ready             = !m_rc_valid;
assign cpl_state_reset_buf_ready = !m_rc_valid && !skip_rc_valid;

wire                      rc_valid       = m_rc_valid || skip_rc_valid;
wire [USER_BUF_BITS-1:0]  rc_buf_idx     = m_rc_valid ? completion_buf_idx : skp_completion_buf_idx;
wire [BURST_COUNT-1:0]    rc_buffer_fin  = m_rc_valid ? buffer_fin         : skp_buffer_fin;


// Completion
ram_sxp #(.DATA_WIDTH(BURST_COUNT), .ADDR_WIDTH(USER_BUF_BITS), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b0)) completion_data (
    .wclk(clk),
    .we(    !bursts_reset_ready ? 1'b1                                : rc_valid || cpl_state_reset_buf),
    .waddr( !bursts_reset_ready ? bursts_reset_idx[USER_BUF_BITS-1:0] : rc_valid ? rc_buf_idx                         : usrbuf_completed[USER_BUF_BITS - 1:0] ),
    .wdata( !bursts_reset_ready ? 0                                   : rc_valid ? current_completion | rc_buffer_fin : 0 ),
    .raddr( ),
    .rdata( current_completion )
);
// Discarded buffer
ram_sxp #(.DATA_WIDTH(BURST_COUNT), .ADDR_WIDTH(USER_BUF_BITS), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b0)) discarded_data (
    .wclk(clk),
    .we(    !bursts_reset_ready ? 1'b1                                : skip_rc_valid || cpl_state_reset_buf),
    .waddr( !bursts_reset_ready ? bursts_reset_idx[USER_BUF_BITS-1:0] : skip_rc_valid ? skp_completion_buf_idx             : usrbuf_completed[USER_BUF_BITS - 1:0] ),
    .wdata( !bursts_reset_ready ? 0                                   : skip_rc_valid ? current_discarded | skp_buffer_fin : 0 ),
    .raddr( ),
    .rdata( current_discarded )
);


reg [BURSTS_BITS-1:0]                   cpl_cur_burst_idx;
wire                                    current_completion_burst = current_completion[cpl_cur_burst_idx];
wire                                    current_discarded_burst  = current_discarded[cpl_cur_burst_idx] && TIMESTAMP_LATE_DISCARD;

// With 1 extra high order bit for fast overflow detection
reg [RAM_ADDR_WIDTH:DATA_BITS]          current_burst_addr;
reg [BURSTS_BITS + SAMPLES_WIDTH - 1:0] current_burst_tsadd;
wire                                    current_burst_last   = (cpl_cur_burst_idx == cplbuf_bursts);

// NOTE: m_rc_valid cuses burst_finished_valid to deassert after being assert when burst_finished_ready is not set. This is
// violation of AXI-S but we keep it to simplify things since FIFO on the other end supports sudden deassert
// rewrite it with extra reg to be fully complaint if needed
wire                                  burst_finished_valid = !cpl_state_reset_buf && cplbuf_valid && current_completion_burst && !rc_valid && !cpl_state_reset_buf;
wire                                  burst_finished_ready;

reg [39:0] stat_total_cpl_req;
reg [39:0] stat_total_cpl_dis;

always @(posedge clk) begin
    if (dma_rst) begin
        cpl_state_reset_buf <= 1'b0;
        cpl_cur_burst_idx   <= 0;
        usrbuf_completed    <= 0;
        current_burst_tsadd <= 0;
        current_burst_addr  <= 0;

        stat_total_cpl_req <= 0;
        stat_total_cpl_dis <= 0;
    end else begin
        if (burst_finished_valid && burst_finished_ready) begin
            cpl_cur_burst_idx    <= cpl_cur_burst_idx + 1'b1;
            current_burst_tsadd  <= current_burst_tsadd + cplbuf_samples + 1'b1;
            if (!current_discarded_burst) begin
                current_burst_addr   <= current_burst_addr  + cplbuf_bytes[RAM_ADDR_WIDTH-1:DATA_BITS] + 1'b1;
            end

            if (current_discarded_burst) begin
                stat_total_cpl_dis <= stat_total_cpl_dis + 1'b1;
            end

            if (current_burst_last) begin
                cpl_cur_burst_idx   <= 0;
                current_burst_tsadd <= 0;
                cpl_state_reset_buf <= 1'b1;
            end

            stat_total_cpl_req <= stat_total_cpl_req + 1'b1;
        end

        if (cpl_state_reset_buf && cpl_state_reset_buf_ready) begin
            usrbuf_completed    <= usrbuf_completed + 1'b1;
            cpl_state_reset_buf <= 1'b0;
        end

    end
end

assign cplbuf_ready                          = cpl_state_reset_buf;
wire [TIMESTAMP_BITS - 1:0] current_burst_ts = cplbuf_ts + current_burst_tsadd;

wire internal_fifo_ready;
wire internal_fifo_valid = burst_finished_valid && m_descr_ready;
wire proc_idx_valid;

// Keep 64 as a good number for bursts to store
axis_fifo #(.WIDTH(1), .DEEP(64), .EXTRA_REG(1'b0)) bursts_fe_ready_to_air_map (
  .clk(clk),
  .rst(dma_rst),

  .s_rx_tdata(current_burst_last),
  .s_rx_tvalid(internal_fifo_valid),
  .s_rx_tready(internal_fifo_ready),

  .m_tx_tdata(proc_idx_last),
  .m_tx_tvalid(proc_idx_valid),
  .m_tx_tready(s_proc_idx_valid && s_proc_idx_ready)
);

assign m_descr_data         = (TX_EX_CORE) ?
    { current_burst_ts, cplbuf_bytes[SAMPLES_WIDTH-1:0], current_burst_addr, current_discarded_burst } :
    { current_burst_ts, cplbuf_samples, cplbuf_bytes };

assign m_descr_valid        = burst_finished_valid && internal_fifo_ready;
assign burst_finished_ready = m_descr_ready && internal_fifo_ready;

assign s_proc_idx_ready     = 1'b1;

always @(posedge clk) begin
    if (dma_rst) begin
        usrbuf_aired <= 0;

        m_int_valid  <= 0;
    end else begin
        if (m_int_valid && m_int_ready) begin
            m_int_valid <= 1'b0;
        end

        if (s_proc_idx_valid && s_proc_idx_ready && proc_idx_last) begin
            usrbuf_aired <= usrbuf_aired + 1'b1;

            // post interrupt
            m_int_valid  <= 1'b1;
        end
    end
end

// Control logic
// TODO: wait finilizing of all DMA in flight while reseting the core
assign s_control_ready = 1'b1;
always @(posedge clk) begin
    if (rst) begin
        dma_rst <= 1'b1;
    end else begin

        if (s_control_valid && s_control_ready) begin
            dma_rst <= !s_control_data[0];
        end

    end
end

assign m_dma_stat_data[7:0]   = usrbuf_posted;
assign m_dma_stat_data[15:8]  = usrbuf_requested;
assign m_dma_stat_data[23:16] = usrbuf_completed;
assign m_dma_stat_data[31:24] = usrbuf_aired;

assign m_dma_tsl_data         = s_fedma_ts[31:0];

assign core_ready             = bursts_reset_ready;


endmodule
