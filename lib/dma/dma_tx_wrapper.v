// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module dma_tx_wrapper #(
    parameter TIMESTAMP_BITS   = 49,
    parameter TIMESTAMP_LATE_DISCARD = 1,
    parameter RAM_ADDR_WIDTH   = 17,
    parameter BUS_ADDR_WIDTH   = 32,
    parameter DATA_BITS        = 3,
    parameter REQUEST_LEN_BITS = 12,
    parameter PCIE_TAG_BITS    = 5,
    parameter SAMPLES_WIDTH    = RAM_ADDR_WIDTH - 1,
    parameter ULTRA_SCALE      = 0,
    parameter STAT_FE_WIDTH    = 20,
    parameter SUPPORT_8CH      = 1'b0,
    parameter SUPPORT_4CH      = 1'b0,
    parameter SUPPORT_2CH      = 1'b1,
    parameter STAT_CNTR_WIDTH  = 8,
    parameter [0:0] TX_EX_CORE = 1'b0,
    parameter FE_DESCR_WIDTH   = TIMESTAMP_BITS + SAMPLES_WIDTH + (RAM_ADDR_WIDTH - DATA_BITS) + 2 * TX_EX_CORE
)(
    input clk,
    input rst,

    ///////////////////////////////////////////////////////
    // Front end interface (compatibility with old software)
    output reg [1:0]                     fe_mute,
    output reg [1:0]                     fe_format,
    output reg                           fe_swap,

    input  [TIMESTAMP_BITS - 1:0]        s_fedma_ts,         //Current TS to discard old buffer requests
    input  [RAM_ADDR_WIDTH:8]            s_fedma_ram_addr,   //Determines availability of RAM

    input                                m_descr_ready,
    output                               m_descr_valid,
    output [FE_DESCR_WIDTH-1:0]          m_descr_data,

    // Burst processed and is ready to be reused
    input                                s_proc_idx_valid,
    output                               s_proc_idx_ready,

    input   [STAT_FE_WIDTH-1:0]          fe_late_bursts,
    ///////////////////////////////////////////////////////
    // AL DMA configuration channel
    input [7:0]               s_dmacfg_waddr,  //1 KB space
    input [31:0]              s_dmacfg_wdata,
    input                     s_dmacfg_wvalid,
    output                    s_dmacfg_wready,

    // PCIe mover interface
    output                                      m_tcq_valid,
    input                                       m_tcq_ready,
    output [RAM_ADDR_WIDTH - 1:DATA_BITS]       m_tcq_laddr,
    output [BUS_ADDR_WIDTH - 1:DATA_BITS]       m_tcq_raddr,
    output [REQUEST_LEN_BITS - 1:DATA_BITS]     m_tcq_length,
    output [PCIE_TAG_BITS-1:0]                  m_tcq_tag,

    // Request termination, s_tcq_ctag can be reused
    input                                       m_tcq_cvalid,
    output                                      m_tcq_cready,
    input  [PCIE_TAG_BITS-1:0]                  m_tcq_ctag,

    // Interrupt for buffer completion (aired)
    input                                       m_int_ready,
    output                                      m_int_valid,

    input [2:0]                                 cfg_max_req_sz,

    ///////////////////////////////////////////////////////
    // Control& status: Compatibility interface
    // Burst len (OLD)
    input [31:0]              axis_cnf_len_data,
    input                     axis_cnf_len_valid,
    output                    axis_cnf_len_ready,

    // Burst TS (OLD)
    input [31:0]              axis_tm_len_data,
    input                     axis_tm_len_valid,
    output                    axis_tm_len_ready,

    // New configuration interface
    input [31:0]              axis_ncfg0_data,
    input                     axis_ncfg0_valid,
    output                    axis_ncfg0_ready,

    input [31:0]              axis_ncfg1_data,
    input                     axis_ncfg1_valid,
    output                    axis_ncfg1_ready,

    input [31:0]              axis_ntsh_data,
    input                     axis_ntsh_valid,
    output                    axis_ntsh_ready,

    input [31:0]              axis_ntsl_data,
    input                     axis_ntsl_valid,
    output                    axis_ntsl_ready,

    // Control
    input [31:0]              axis_control_data,
    input                     axis_control_valid,
    output                    axis_control_ready,

    // Output data status stream
    output [31:0]             axis_stat_data,
    output                    axis_stat_valid,
    input                     axis_stat_ready,

    // Output data ext status stream
    output [31:0]             axis_stat_m_data,
    output                    axis_stat_m_valid,
    input                     axis_stat_m_ready,

    // Output data current TS stream
    output [31:0]             axis_stat_ts_data,
    output                    axis_stat_ts_valid,
    input                     axis_stat_ts_ready,

    // Output data ext status stream
    output [31:0]             axis_stat_cpl_data,
    output                    axis_stat_cpl_valid,
    input                     axis_stat_cpl_ready,

    // AUX status
    output reg                  tx_buffprecharged,
    output                      txdma_nactive,

    input [STAT_CNTR_WIDTH-1:0] stat_cpl_nodata,
    input                       stat_notlp,

    input                       tran_usb_active,
    input [63:0]                usb_fetx_stat
);

localparam [0:0] FINE_RAMFIFO_AVAIL_CHECK = 1'b1;
localparam       USER_BUF_BITS            = 5;  // Compatibility with old software

// Emulation API, OLD:
//   axis_cnf_len_data[31:0]   samples (in MIMO)  REG_WR_TXDMA_CNF_L
//   axis_tm_len_data[31:0]    TS                 REG_WR_TXDMA_CNF_T
//   axis_control_data[]                          REG_WR_TXDMA_COMB (before 27022025)
//       [1:0] -- 11 -- FMT_16BIT; 00 -- STOP
//       [2]      Repeat
//       [3]      Format; 0 - SISO; 1 - MIMO
//     [6:4]      <reserved>
//       [7]      <reserved> was: Reset
//       [8]      mute_b
//       [9]      mute_a
//      [10]      swap_ab
//      [11]      Update MuteAB/SwapAB during active mode
//
//   axis_control_data[]
//       [1:0] -- 11 -- Start; 00 -- STOP
//       [2]      Repeat
//
//       All swapping / muting is done directrly through FE registers
//
// +-------------- New Style -----------+
// | 31             24 | 23           0 |
// |                   | bytes[19:0]    |  axis_ncfg0_data
// |       bursts[1:0] | samples[19:0]  |  axis_ncfg1_data
// |  ts[63:32]                         |  axis_ntsh_data
// |  ts[31:0]                          |  axis_ntsl_data

// Old style (since 20240526)
// | 0 | nots | samples[14:0]  | ts[46:32]          |     REG_WR_TXDMA_CNF_L
// |  ts[31:0]                                      |     REG_WR_TXDMA_CNF_T

//  { TS, nb_bursts, nb_samples, nb_raw_bytes };
localparam BURSTS_BITS           = 3;
localparam USER_BUF_CONFIG_WIDTH = TIMESTAMP_BITS + BURSTS_BITS + SAMPLES_WIDTH + RAM_ADDR_WIDTH;

wire                              dma_core_ready;
wire                              circ_core_ready;

wire [BURSTS_BITS-1:0]            nb_bursts;
reg [SAMPLES_WIDTH-1:0]           nb_samples;
reg [RAM_ADDR_WIDTH-1:0]          nb_raw_bytes;
reg [TIMESTAMP_BITS-1:32]         nb_ts;

wire [14:0] oldstyle_samples       = axis_cnf_len_data[29:15]; // actual samples
wire [16:3] oldstyle_bytes         =
    (fe_format == 3 && SUPPORT_8CH) ? { oldstyle_samples, 2'b11 } :
    (fe_format == 2 && SUPPORT_4CH) ? { oldstyle_samples, 1'b1 }  :
    (fe_format == 1 && SUPPORT_2CH) ? oldstyle_samples : oldstyle_samples[14:1];


reg dma_tx_ready;
reg [BURSTS_BITS-1:0]             nb_rbrst;
assign                            nb_bursts = (TX_EX_CORE) ? nb_rbrst : 5'h0;

always @(posedge clk) begin
    if (TX_EX_CORE) begin
        if (axis_ncfg0_ready && axis_ncfg0_valid) begin
            nb_raw_bytes                <= axis_ncfg0_data[RAM_ADDR_WIDTH-1:0];
        end
        if (axis_ncfg1_ready && axis_ncfg1_valid) begin
            nb_samples                  <= axis_ncfg1_data[SAMPLES_WIDTH-1:0];
            nb_rbrst                    <= axis_ncfg1_data[24 + BURSTS_BITS -1:24];
        end
        if (axis_ntsh_ready && axis_ntsh_valid) begin
            nb_ts[TIMESTAMP_BITS-1:32]                <= axis_ntsh_data[TIMESTAMP_BITS-1-32:0];
        end
    end else begin
        if (axis_cnf_len_ready && axis_cnf_len_valid) begin
            nb_samples                                <= oldstyle_samples;
            nb_raw_bytes[RAM_ADDR_WIDTH-1:DATA_BITS]  <= oldstyle_bytes[16:DATA_BITS];
            nb_ts[47:32]                              <= { 1'b0, axis_cnf_len_data[14:0] };
            nb_ts[TIMESTAMP_BITS-1]                   <= axis_cnf_len_data[30];   //no-ts bit
        end
    end
end


wire [USER_BUF_CONFIG_WIDTH-1:0]  bufcmd_data  = { nb_ts, axis_tm_len_data, nb_bursts, nb_samples, nb_raw_bytes };
wire                              bufcmd_valid = TX_EX_CORE ? axis_ntsl_valid : axis_tm_len_valid;
wire                              bufcmd_ready;
assign                            axis_cnf_len_ready = TX_EX_CORE ? 1'b1 : dma_tx_ready; //1'b1;
assign                            axis_tm_len_ready  = TX_EX_CORE ? 1'b1 : bufcmd_ready;
assign                            axis_control_ready = 1'b1;

assign                            axis_ncfg0_ready   = 1'b1;
assign                            axis_ncfg1_ready   = 1'b1;
assign                            axis_ntsh_ready    = 1'b1;
assign                            axis_ntsl_ready    = TX_EX_CORE ? bufcmd_ready : 1'b1;

wire control_ready;
reg  control_valid;
reg  txdma_active;
assign                            txdma_nactive = !txdma_active;

always @(posedge clk) begin
    if (rst) begin
        txdma_active  <= 1'b0;
        fe_format     <= 2'b0;
        control_valid <= 1'b0;
        dma_tx_ready  <= 0;
    end else begin
        control_valid <= 1'b0;
        dma_tx_ready  <= !txdma_active || circ_core_ready && dma_core_ready /*|| tran_usb_active*/;

        if (axis_control_valid && axis_control_ready) begin
            if (txdma_active == 1'b0 && axis_control_data[1:0] == 2'b11) begin
                control_valid <= !tran_usb_active; //1'b1;
                txdma_active  <= 1'b1;
                fe_format       <= axis_control_data[4:3];

                // Legacy flags
                fe_mute       <= { axis_control_data[8], axis_control_data[9] };
                fe_swap       <= axis_control_data[10];
            end else if (txdma_active == 1'b1 && axis_control_data[1:0] == 2'b00) begin
                control_valid <= !tran_usb_active; //1'b1;
                txdma_active  <= 1'b0;
            end else if (axis_control_data[11]) begin // Legacy
                fe_mute       <= { axis_control_data[8], axis_control_data[9] };
                fe_swap       <= axis_control_data[10];
            end
        end
    end
end

wire control_data = txdma_active;
wire                               fifo_full;
wire [RAM_ADDR_WIDTH:8]            fifo_used;
wire [PCIE_TAG_BITS:0]             pcie_tags_avail;
////////////////////////////////////////////////////////////////////////////////
wire     [RAM_ADDR_WIDTH  :DATA_BITS]      rq_loc_addr;
wire     [BUS_ADDR_WIDTH-1:DATA_BITS]      rq_bus_addr;
wire     [RAM_ADDR_WIDTH-1:DATA_BITS]      rq_length;
wire                                       rq_valid;
wire     [USER_BUF_BITS+BURSTS_BITS-1:0]   rq_tag;
wire                                       rq_ready;
wire  [USER_BUF_BITS+BURSTS_BITS-1:0]      rc_tag;
wire                                       rc_valid;
wire                                       rc_ready;
wire  [RAM_ADDR_WIDTH:8]                   rc_fifomin;

dma_tx_pcie #(
    .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH),
    .BUS_ADDR_WIDTH(BUS_ADDR_WIDTH),
    .DATA_BITS(DATA_BITS),
    .PCIE_TAG_BITS(PCIE_TAG_BITS),
    .USE_DATA_CREDITS(FINE_RAMFIFO_AVAIL_CHECK),
    .USER_TAG_BITS(USER_BUF_BITS + BURSTS_BITS),
    .ULTRA_SCALE(ULTRA_SCALE)
) dma_tx_pcie (
    .clk(clk),
    .rst(!txdma_active),

    .core_ready(dma_core_ready),
    .s_consume_ram_addr(s_fedma_ram_addr),
    .fifo_full(fifo_full),
    .fifo_used(fifo_used),
    .pcie_tags_avail(pcie_tags_avail),

    // request
    .s_rq_loc_addr(rq_loc_addr),
    .s_rq_bus_addr(rq_bus_addr),
    .s_rq_length(rq_length),
    .s_rq_valid(rq_valid),
    .s_rq_tag(rq_tag),
    .s_rq_ready(rq_ready),

    // competion
    .m_rc_tag(rc_tag),
    .m_rc_valid(rc_valid),
    .m_rc_ready(rc_ready),

    .cfg_max_req_sz(cfg_max_req_sz),

    // memory op request
    .m_tcq_valid(m_tcq_valid),
    .m_tcq_ready(m_tcq_ready),
    .m_tcq_laddr(m_tcq_laddr),
    .m_tcq_raddr(m_tcq_raddr),
    .m_tcq_length(m_tcq_length),
    .m_tcq_tag(m_tcq_tag),

    // Request termination, s_tcq_ctag can be reused
    .m_tcq_cvalid(m_tcq_cvalid),
    .m_tcq_cready(m_tcq_cready),
    .m_tcq_ctag(m_tcq_ctag)
);

wire [31:0]  m_dma_stat_data;
wire [31:0]  m_dma_tsl_data;
wire [15:0]  m_dma_late;
wire [RAM_ADDR_WIDTH:8] m_dma_fifo_avail;
wire         stat_dmareq_ovf;

circ_dma_tx_engine #(
    .TIMESTAMP_BITS(TIMESTAMP_BITS),
    .TIMESTAMP_LATE_DISCARD(TIMESTAMP_LATE_DISCARD),
    .USER_BUF_BITS(USER_BUF_BITS),
    .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH),
    .DATA_BITS(DATA_BITS),
    .BURSTS_BITS(BURSTS_BITS),
    .BUS_ADDR_WIDTH(BUS_ADDR_WIDTH),
    .DEFAULT_MAX_BUFFER_SIZE(15'h7fff),   // Compatibility with old software (32kB max DMA buffer)
    .CHECK_BUFFER_REQ_SIZE(!FINE_RAMFIFO_AVAIL_CHECK),
    .SAMPLES_WIDTH(SAMPLES_WIDTH),
    .ULTRA_SCALE(ULTRA_SCALE),
    .TX_EX_CORE(TX_EX_CORE)
) circ_dma (
    .clk(clk),
    .rst(rst),

    // DMA configuration
    .s_dmacfg_waddr(s_dmacfg_waddr[USER_BUF_BITS:0]),
    .s_dmacfg_wdata(s_dmacfg_wdata),
    .s_dmacfg_wvalid(s_dmacfg_wvalid),
    .s_dmacfg_wready(s_dmacfg_wready),

    // Buffer post  TS | bursts_cnt | length_sps | length_bytes (per burst)
    .s_bufcmd_valid(bufcmd_valid),
    .s_bufcmd_ready(bufcmd_ready),
    .s_bufcmd_data(bufcmd_data),

    .s_control_valid(control_valid),
    .s_control_ready(control_ready),
    .s_control_data(control_data),

    .core_ready(circ_core_ready),
    .stat_dmareq_ovf(stat_dmareq_ovf),

    // Stats & interrupts
    .m_dma_stat_data(m_dma_stat_data),
    .m_dma_tsl_data(m_dma_tsl_data), // Current TS (for latency measurement)
    .m_dma_late(m_dma_late),
    .m_dma_fifo_avail(m_dma_fifo_avail),

    .m_int_valid(m_int_valid),
    .m_int_ready(m_int_ready),

    //////////////////////////////////////////////////
    // interface to dma_buff
    .m_rq_loc_addr(rq_loc_addr),
    .m_rq_bus_addr(rq_bus_addr),
    .m_rq_length(rq_length),
    .m_rq_valid(rq_valid),
    .m_rq_tag(rq_tag),
    .m_rq_ready(rq_ready),

    // competion
    .m_rc_tag(rc_tag),
    .m_rc_valid(rc_valid),
    .m_rc_ready(rc_ready),

    //////////////////////////////////////////////
    // Interface to FE
    .s_fedma_ts(s_fedma_ts),               //Current TS to discard old buffer requests
    .s_fedma_ram_addr(s_fedma_ram_addr),   //Determines availability of RAM

    // Burst processed and is ready to be reused
    .s_proc_idx_valid(s_proc_idx_valid),
    .s_proc_idx_ready(s_proc_idx_ready),

    // Burst decriptor data
    .m_descr_ready(m_descr_ready),
    .m_descr_valid(m_descr_valid),
    .m_descr_data(m_descr_data)
);


assign axis_stat_valid        = 1'b1;
assign axis_stat_m_valid      = 1'b1;
assign axis_stat_ts_valid     = 1'b1;
assign axis_stat_cpl_valid    = 1'b1;

wire [5:0] usrbuf_posted      = m_dma_stat_data[7:0];
wire [5:0] usrbuf_requested   = m_dma_stat_data[15:8];
wire [5:0] usrbuf_completed   = m_dma_stat_data[23:16];
wire [5:0] usrbuf_aired       = m_dma_stat_data[31:24];

wire [3:0] fifo_addr_full = { 2'b01, tx_buffprecharged, tran_usb_active };
wire [2:0] dma_state = {
    stat_dmareq_ovf,
    stat_notlp,
    circ_core_ready && dma_core_ready
};

reg [9:0] m_dma_fifo_avail_r;
always @(posedge clk) begin
    m_dma_fifo_avail_r <= m_dma_fifo_avail;
end

// New Sta
// DW0:stat_data     | Indecies
// DW1:stat_m_data   | [ fifo_min, fe_drop8, dma_drop8 ]
// DW2:stat_ts_data  | TS
// DW3:stat_cpl_data | [ OVF_ALARM ]

//                              1b            4b                  3b
wire [7:0]  dma_stat       = { !txdma_nactive, fifo_addr_full, dma_state };
wire [15:0] dropped_bursts = fe_late_bursts;
reg [15:0]  bursts_sent;
reg [RAM_ADDR_WIDTH:8] fifo_used_r;
reg [RAM_ADDR_WIDTH:8] fifo_used_p;

wire [7:0]  stat_fifo_used = fifo_used_r[RAM_ADDR_WIDTH:RAM_ADDR_WIDTH-7];
reg [PCIE_TAG_BITS:0]  pcie_tags_avail_r;
reg [PCIE_TAG_BITS:0]  pcie_tags_avail_p;
wire [7:0]  stat_pcie_tags_avail = pcie_tags_avail_r;
reg                    fifo_full_r;

always @(posedge clk) begin
    if (!txdma_active) begin
        bursts_sent       <= 0;
        fifo_used_r       <= ~0;
        pcie_tags_avail_r <= ~0;
        fifo_used_p       <= ~0;
        pcie_tags_avail_p <= ~0;
        fifo_full_r       <= 0;
    end else begin
        fifo_full_r       <= fifo_full;
        fifo_used_p       <= fifo_used;
        pcie_tags_avail_p <= pcie_tags_avail;

        if (axis_stat_m_ready) begin
            fifo_used_r       <= ~0;
            pcie_tags_avail_r <= ~0;
        end
        if (fifo_used_r > fifo_used_p) begin
            fifo_used_r <= fifo_used_p;
        end
        if (pcie_tags_avail_r > pcie_tags_avail_p) begin
            pcie_tags_avail_r <= pcie_tags_avail_p;
        end

        if (s_proc_idx_valid && s_proc_idx_ready) begin
            bursts_sent <= bursts_sent + 1'b1;
        end
    end
end

assign axis_stat_data = tran_usb_active ? { dropped_bursts[7:0], dma_stat } : {usrbuf_completed[5:4], usrbuf_posted,             // Written by user
                         usrbuf_completed[3:2], usrbuf_aired,              // Sent out to air
                         usrbuf_completed[1:0], usrbuf_requested,          // Requested by DMA engine
                         dma_stat};
assign axis_stat_m_data   = tran_usb_active ? usb_fetx_stat[31:0] : { stat_pcie_tags_avail[7:0], stat_fifo_used[7:0], dropped_bursts[7:0], m_dma_late[7:0] };
assign axis_stat_ts_data  = m_dma_tsl_data;
assign axis_stat_cpl_data = { bursts_sent, tran_usb_active ? usb_fetx_stat[47:32] : { stat_cpl_nodata[7:0], fifo_full_r, stat_dmareq_ovf, 6'h00 }};

// Control & AUX
always @(posedge clk) begin
    if (rst) begin
        tx_buffprecharged <= 1'b0;
    end else begin
        if (txdma_nactive) begin
            tx_buffprecharged <= 1'b0;
        end else if (!tran_usb_active && (m_descr_valid)) begin
            tx_buffprecharged <= 1'b1;
        end else if (tran_usb_active && (usb_fetx_stat[32])) begin
            tx_buffprecharged <= 1'b1;
        end
    end
end

endmodule
