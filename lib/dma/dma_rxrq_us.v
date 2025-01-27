// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module dma_rxrq_us #(
    parameter BUFFER_SIZE_BITS = 16,
    parameter BUFFER_BURST_BITS = 6,
    parameter REMOTE_ADDR_WIDTH = 32,
    parameter MEM_TAG = 5,
    parameter DATA_BITS = 4,
    parameter ULTRA_SCALE = 0
)(
    input                     clk,
    input                     rst,

    input [7:0]               s_al_waddr,
    input [31:0]              s_al_wdata,
    input                     s_al_wvalid,
    output                    s_al_wready,

    // RXDMA control channel
    input [2:0]               axis_control_data,
    input                     axis_control_valid,
    output                    axis_control_ready,

    // Buffer confirmation channel
    input                     axis_confirm_valid,
    output                    axis_confirm_ready,

    // RXDMA debug stat output
    output [31:0]             axis_stat_data,
    output                    axis_stat_valid,
    input                     axis_stat_ready,

    // Bursts & Buffers status (effectively 64-bit register)
    output [31:0]             axis_fburts_data,
    output                    axis_fburts_valid,
    input                     axis_fburts_ready,
    output [31:0]             axis_fbuffs_data,
    output                    axis_fbuffs_valid,
    input                     axis_fbuffs_ready,

    // Interrupt Data (+OVF in mix mode)
    input                     int_ready,
    output reg                int_valid,

    output                    stat_cnf_ready,
    input                     stat_cnf_valid,

    input                     inplace_cnf_enabled,
    output reg                inplace_cnf_valid,
    input                     inplace_cnf_ready,

    output reg                rx_streamingready,
    output                    rx_dmaactive,

    input [1:0]               cfg_max_payload_sz,
    input [5:0]               fe_stat,

    output reg                                       m_tcq_valid,
    input                                            m_tcq_ready,
    output reg [BUFFER_SIZE_BITS - 1:DATA_BITS]      m_tcq_laddr,
    output reg [REMOTE_ADDR_WIDTH - 1:DATA_BITS]     m_tcq_raddr,
    output reg [BUFFER_BURST_BITS - 1 + 3:DATA_BITS] m_tcq_length,
    output     [MEM_TAG-1:0]                         m_tcq_tag,

    input                                            m_tcq_cvalid,
    output                                           m_tcq_cready,
    input [MEM_TAG-1:0]                              m_tcq_ctag,

    // Burst IF to FE
    output                          fifo_burst_release,
    input  [BUFFER_SIZE_BITS - 6:0] fifo_burst_avail_z,
    input                           fifo_burst_skip,
    input                           fifo_burst_fill,
    input                           fifo_burst_mlowmrk
);

// Number of DMA buffers
localparam DMA_BUF_BITS    = 5;

// Number of continous bursts in one DMA buffer
localparam MAX_BUSRTS_BITS = 5;
localparam MAX_BUSRTS      = 1 << MAX_BUSRTS_BITS;

// Number of full "zero" buffers
localparam MAX_BUFF_SKIP_BITS = 24;

// DMA buffer stages: inram => xfred => ntfysent => confirmed
reg [DMA_BUF_BITS:0] dbno_inram;     // BUF# filled in FIFO RAM
reg [DMA_BUF_BITS:0] dbno_xfred;     // BUF# transferred to host
reg [DMA_BUF_BITS:0] dbno_ntfysent;  // BUF# notification sent
reg [DMA_BUF_BITS:0] dbno_confirmed; // BUF# confirmed by user (can reuse again)

reg                  dma_en;
wire [31:12]         dma_raddr;

// DMA static configuration (isn't changing when DMA is active)
assign s_al_wready = 1'b1;
assign rx_dmaactive = dma_en;

ram_sxp #(.DATA_WIDTH(32-12), .ADDR_WIDTH(DMA_BUF_BITS), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b1)) dma_addrs (
    .wclk(clk),
    .we(s_al_wvalid && (s_al_waddr[7:5] == 0)),
    .waddr(s_al_waddr[4:0]),
    .wdata(s_al_wdata[31:12]),
    .raddr(dbno_xfred[DMA_BUF_BITS-1:0]),
    .rdata(dma_raddr)
);

reg [BUFFER_SIZE_BITS-1:DATA_BITS] cfg_brst_words_z;  // bytes in each bursts
reg [MAX_BUSRTS_BITS-1:0]          cfg_buff_brst_z;   // bursts in DMA buffer

always @(posedge clk) begin
  if (rst) begin
  end else begin
    if (s_al_wvalid && (s_al_waddr[7:5] == 1)) begin
      cfg_brst_words_z <= s_al_wdata[BUFFER_SIZE_BITS-3:DATA_BITS-3];
    end else if (s_al_wvalid && (s_al_waddr[7:5] == 2)) begin
      cfg_buff_brst_z  <= s_al_wdata;
    end
  end
end

wire                                     tbuffer_valid;
wire                                     tbuffer_last;
wire [BUFFER_SIZE_BITS:DATA_BITS]        tbuffer_data;

wire                                     tburst_valid;
wire                                     tburst_last;
wire [MAX_BUFF_SKIP_BITS+MAX_BUSRTS-1:0] tburst_data;

bsig_to_burstbuffer #(
  .MAX_BUSRTS_BITS(MAX_BUSRTS_BITS),
  .BUFFER_SIZE_BITS(BUFFER_SIZE_BITS),
  .MAX_BUFF_SKIP_BITS(MAX_BUFF_SKIP_BITS),
  .DATA_BITS(DATA_BITS)
) bsig_to_burstbuffer_i (
    .clk(clk),
    .dma_en(dma_en),

    .cfg_max_payload_sz(cfg_max_payload_sz),
    .cfg_dis_lowwmrk(1'b0),
    .cfg_buff_brst_z(cfg_buff_brst_z),
    .cfg_brst_words_z(cfg_brst_words_z),

    // BUSRT pulses
    .fifo_burst_skip(fifo_burst_skip),
    .fifo_burst_fill(fifo_burst_fill),
    .fifo_burst_mlowmrk(fifo_burst_mlowmrk),

    .tbuffer_valid(tbuffer_valid),     // Buffer update stats
    .tbuffer_last(tbuffer_last),       // Last update for this buffer
    .tbuffer_data(tbuffer_data),       // Number of bytes filled in current buffer

    .tburst_valid(tburst_valid),     //
    .tburst_last(tburst_last),       // Last burst in complete buffer
    .tburst_data(tburst_data)        // fillstatus & skipped count
);

reg                  full_buffer_filled_z_reset;

assign axis_confirm_ready = 1'b1;
assign stat_cnf_ready     = 1'b1;

wire confirm_read_buffer  = axis_confirm_valid && axis_confirm_ready;
wire dma_next_buf;

always @(posedge clk) begin
  if (~dma_en) begin
    dbno_confirmed    <= 6'b0;
    dbno_xfred        <= 6'b0;
    dbno_inram        <= 6'b0;
    dbno_ntfysent     <= 6'b0;

    full_buffer_filled_z_reset <= 1'b1;
  end else begin
    if (confirm_read_buffer) begin
      dbno_confirmed       <= dbno_confirmed + 1'b1;
    end

    if (dma_next_buf) begin
      dbno_xfred           <= dbno_xfred + 1'b1;
    end

    full_buffer_filled_z_reset <= 1'b0;
    if (tbuffer_last) begin
      dbno_inram                <= dbno_inram + 1'b1;
      full_buffer_filled_z_reset <= 1'b1;
    end

    if ((stat_cnf_ready && stat_cnf_valid || inplace_cnf_valid && inplace_cnf_ready)) begin
      dbno_ntfysent <= dbno_ntfysent + 1'b1;
    end
  end
end

wire [DMA_BUF_BITS:0] dma_buff_avail_cnt = dbno_confirmed - dbno_xfred - 1'b1;
wire                  dma_buff_avail     = dma_buff_avail_cnt[DMA_BUF_BITS] || inplace_cnf_enabled;
wire [DMA_BUF_BITS:0] dma_host_avail_cnf = dma_buff_avail_cnt; // not in userspace

wire                              full_buffer_finished;
wire [BUFFER_SIZE_BITS:DATA_BITS] full_buffer_filled_z; // Number of bytes in fully filled DMA buffer

// Buffer bursts status
ram_sxp #(.DATA_WIDTH(BUFFER_SIZE_BITS+2-DATA_BITS), .ADDR_WIDTH(DMA_BUF_BITS), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b1)) buf_stats (
    .wclk(clk),
    .we(tbuffer_valid || full_buffer_filled_z_reset),
    .waddr(dbno_inram[DMA_BUF_BITS-1:0]),
    .wdata({ tbuffer_last, full_buffer_filled_z_reset ? {(BUFFER_SIZE_BITS + 1 - DATA_BITS){1'b1}} : tbuffer_data}),
    .raddr(dbno_xfred[DMA_BUF_BITS-1:0]),
    .rdata({full_buffer_finished, full_buffer_filled_z})
);


reg  [BUFFER_SIZE_BITS:DATA_BITS] dma_buffer_req_sent;
wire [BUFFER_SIZE_BITS:DATA_BITS] dma_buffer_cts_z = full_buffer_filled_z - dma_buffer_req_sent;

wire [2:0]                     mlowmrk_size_ext_z = (cfg_max_payload_sz == 2'b00) ? 3'b000 :
                                                    (cfg_max_payload_sz == 2'b01) ? 3'b001 :
                                                    (cfg_max_payload_sz == 2'b10) ? 3'b011 : 3'b111;
wire [9:DATA_BITS]             mlowmrk_size_z     = { mlowmrk_size_ext_z, {(7-DATA_BITS){1'b1}} };

wire stat_cycle_valid                             = 1'b1;
wire [BUFFER_SIZE_BITS:DATA_BITS] pcie_buff_2     = dma_buffer_cts_z - mlowmrk_size_z;
wire                      pcie_maxpayload_avail   = ~pcie_buff_2[BUFFER_SIZE_BITS];
wire [BUFFER_BURST_BITS - 1 + DATA_BITS:DATA_BITS] pcie_reqsize_z = (pcie_maxpayload_avail) ? mlowmrk_size_z : dma_buffer_cts_z;

// Buffer is finished and we have ` <= mlowmrk_size_ext_z ` in buffer
wire                      pcie_last_buff_transfer = full_buffer_finished && (~pcie_maxpayload_avail || pcie_buff_2 == 0);
wire                      pcie_transfer_avail     = (pcie_maxpayload_avail || pcie_last_buff_transfer) && ~dma_buffer_cts_z[BUFFER_SIZE_BITS];
wire                      pcie_transfer_done      = full_buffer_finished && dma_buffer_cts_z[BUFFER_SIZE_BITS];

assign axis_fburts_valid = 1'b1;
assign axis_fbuffs_valid = 1'b1;

// Replace for in-place confirmation
wire st_buffer_full;
ram_sxp #(.DATA_WIDTH(1+MAX_BUSRTS+MAX_BUFF_SKIP_BITS), .ADDR_WIDTH(DMA_BUF_BITS+1), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b1)) bursts_b_stats (
    .wclk(clk),
    .we(tburst_valid),
    .waddr(dbno_inram),
    .wdata({tburst_last, tburst_data}),

    .raddr(dbno_ntfysent),
    .rdata({st_buffer_full, axis_fbuffs_data[MAX_BUFF_SKIP_BITS-1:0], axis_fburts_data})
);

// Report number of available buffers to read, so we can rely only on this register
assign axis_fbuffs_data[31:24] = { st_buffer_full, {(6-DMA_BUF_BITS){1'b1}},  dma_host_avail_cnf };


reg  req_tag_valid;
reg  req_trans_last;
wire tag_avaliable = !req_tag_valid;
reg [BUFFER_BURST_BITS - 1 + DATA_BITS:DATA_BITS] dma_req_len;

assign m_tcq_cready = 1'b1;
reg noop_send;

wire transfer_buf_end = m_tcq_cvalid && m_tcq_cready && req_trans_last || noop_send;
assign dma_next_buf   = m_tcq_valid && m_tcq_ready && req_trans_last || noop_send;

localparam INT_COUNTER_WIDTH = 3;
reg [INT_COUNTER_WIDTH - 1:0] outstanding_interrupts;


always @(posedge clk) begin
  if (~dma_en) begin
    int_valid <= 1'b0;
    outstanding_interrupts <= 0;
  end else begin
    if (outstanding_interrupts != 0) begin
      int_valid <= 1'b1;
    end
    if (int_valid && int_ready) begin
      int_valid <= 1'b0;
      outstanding_interrupts <= outstanding_interrupts - 1'b1;
    end

    if (transfer_buf_end && ~inplace_cnf_enabled) begin
      if (!(int_valid && int_ready)) begin
        outstanding_interrupts <= outstanding_interrupts + 1'b1;
      end else begin
        outstanding_interrupts <= outstanding_interrupts;
      end
    end
  end
end

/////////////////////////////////////////////////////////
// Reporting freeing up RING FIFO buffer

reg [BUFFER_SIZE_BITS:DATA_BITS]   dma_transferred;        // Number of bytes confirmed to be sent
reg [BUFFER_SIZE_BITS:DATA_BITS]   dma_burst_bytes_freed;  // Number of bytes reported to be freed

wire [BUFFER_SIZE_BITS:DATA_BITS]  dma_bytes_not_freed    = dma_transferred - dma_burst_bytes_freed;
wire [BUFFER_SIZE_BITS:DATA_BITS]  availbytes_free_burst  = dma_bytes_not_freed - cfg_brst_words_z - 1'b1;
wire                               can_free_burst         = ~availbytes_free_burst[BUFFER_SIZE_BITS];

reg signal_release;
assign fifo_burst_release = signal_release;

always @(posedge clk) begin
  if (~dma_en) begin
    dma_burst_bytes_freed <= 0;
    signal_release <= 0;
  end else begin
    signal_release                 <= 1'b0;
    if (can_free_burst) begin
      dma_burst_bytes_freed        <= dma_burst_bytes_freed + cfg_brst_words_z + 1'b1;
      signal_release               <= 1'b1;
    end
  end
end

assign axis_control_ready = 1'b1;
wire p_stream_ready         = axis_control_valid && axis_control_data[2];
wire start_rx               = axis_control_valid && axis_control_data[0] && ~axis_control_data[2];
wire stop_rx                = axis_control_valid && ~axis_control_data[0] && ~axis_control_data[2];

wire      dma_cycle_valid   = 1'b1;
reg       req_tag_dummy;
reg       m_tcq_dma_done;

wire      mem_wr_event = ~inplace_cnf_valid && ~m_tcq_valid && dma_buff_avail && (pcie_transfer_avail || pcie_transfer_done) && ~full_buffer_filled_z_reset && tag_avaliable && stat_cycle_valid && dma_cycle_valid;


// DMA control and PCIe data movement process
always @(posedge clk) begin
  if (rst) begin
    dma_en                       <= 1'b0;
    m_tcq_valid                  <= 1'b0;
    req_tag_valid                <= 1'b0;
    req_tag_dummy                <= 1'b0;
    rx_streamingready            <= 1'b0;
    noop_send                    <= 1'b0;

  end else begin

    // virtual DMA_UPDATE_COUNTERS state
    if (m_tcq_valid && m_tcq_ready) begin
      m_tcq_valid            <= 1'b0;
      if (!m_tcq_dma_done) begin
        m_tcq_laddr          <= m_tcq_laddr         + m_tcq_length + 1'b1;
        m_tcq_raddr          <= m_tcq_raddr         + m_tcq_length + 1'b1;
        dma_buffer_req_sent  <= dma_buffer_req_sent + m_tcq_length + 1'b1;
      end

      if (req_trans_last) begin
        dma_buffer_req_sent<= 0;
      end
    end

    if (m_tcq_cvalid && m_tcq_cready) begin
      req_tag_valid <= 1'b0;
      if (!req_tag_dummy) begin
        dma_transferred              <= dma_transferred + dma_req_len + 1'b1;
      end
    end

    if (stop_rx) begin
      dma_en        <= 1'b0;
    end

    if (p_stream_ready) begin
      rx_streamingready <= 1'b1;
    end

    if (~dma_en) begin
      m_tcq_laddr         <= 0;
      dma_buffer_req_sent <= 0;
      dma_transferred     <= 0;
      req_tag_valid       <= 2'b0;
      inplace_cnf_valid   <= 1'b0;

      if (start_rx) begin
        dma_en            <= 1'b1;
        req_trans_last    <= 2'b01;
      end

      rx_streamingready   <= 1'b0;
    end else begin
      if (inplace_cnf_valid && inplace_cnf_ready) begin
        inplace_cnf_valid <= 1'b0;
      end

      noop_send <= 1'b0;

      if (mem_wr_event) begin
        if (pcie_transfer_avail) begin
            dma_req_len        <= pcie_reqsize_z;
            req_tag_dummy      <= pcie_transfer_done;

            if (req_trans_last) begin
                m_tcq_raddr           <= { dma_raddr[REMOTE_ADDR_WIDTH - 1:12], {(12 - DATA_BITS){1'b0}} };
            end

            m_tcq_length            <= pcie_reqsize_z;
            m_tcq_valid             <= 1'b1;
            m_tcq_dma_done          <= pcie_transfer_done;

            req_tag_valid           <= 1'b1;
            req_trans_last          <= pcie_last_buff_transfer || pcie_transfer_done;
        end else begin
            noop_send               <= 1'b1;
            dma_buffer_req_sent     <= 0;
        end

        inplace_cnf_valid       <= (pcie_last_buff_transfer || pcie_transfer_done) && inplace_cnf_enabled;
      end
    end

  end
end

assign m_tcq_tag = 0;

assign axis_stat_valid                    = 1'b1;
assign axis_stat_data[DMA_BUF_BITS:0]     = dbno_confirmed;
assign axis_stat_data[7:DMA_BUF_BITS+1]   = fe_stat[1:0];
assign axis_stat_data[DMA_BUF_BITS+8:8]   = dbno_xfred;
assign axis_stat_data[15:DMA_BUF_BITS+9]  = fe_stat[3:2];
assign axis_stat_data[DMA_BUF_BITS+16:16] = dbno_inram;
assign axis_stat_data[23:DMA_BUF_BITS+17] = fe_stat[5:4];
assign axis_stat_data[DMA_BUF_BITS+24:24] = dbno_ntfysent;
assign axis_stat_data[31:DMA_BUF_BITS+25] = { rx_streamingready, dma_en };


endmodule
