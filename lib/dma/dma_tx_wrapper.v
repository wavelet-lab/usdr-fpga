// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module dma_tx_wrapper #(
    parameter TIMESTAMP_BITS   = 49,
    parameter RAM_ADDR_WIDTH   = 17,
    parameter BUS_ADDR_WIDTH   = 32,
    parameter DATA_BITS        = 3,
    parameter REQUEST_LEN_BITS = 12,
    parameter PCIE_TAG_BITS    = 5,
    parameter SAMPLES_WIDTH    = RAM_ADDR_WIDTH - 1,
    parameter FE_DESCR_WIDTH   = TIMESTAMP_BITS + SAMPLES_WIDTH + (RAM_ADDR_WIDTH - DATA_BITS),
    parameter BURSTS_BITS      = 5,
    parameter ULTRA_SCALE      = 0,
    parameter _USER_BUF_CONFIG_WIDTH =  BURSTS_BITS + FE_DESCR_WIDTH,
    parameter STAT_FE_WIDTH    = 20
)(
    input clk,
    input rst,

    ///////////////////////////////////////////////////////
    // Front end interface (compatibility with old software)
    output reg [1:0]                     fe_mute,
    output reg                           fe_format,
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
    // Burst len
    input [31:0]              axis_cnf_len_data,
    input                     axis_cnf_len_valid,
    output                    axis_cnf_len_ready,

    // Burst TS
    input [31:0]              axis_tm_len_data,
    input                     axis_tm_len_valid,
    output                    axis_tm_len_ready,

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

    ////////////////////////////////////////////////////////////
    // TODO: New burst style interface


    // AUX status
    output reg                tx_buffprecharged,
    output                    txdma_nactive,

    input [19:0]              stat_cpl_nodata,
    input                     stat_notlp,

    input                     tran_usb_active,
    input [63:0]              usb_fetx_stat
);

localparam [0:0] FINE_RAMFIFO_AVAIL_CHECK = 1'b1;
localparam       USER_BUF_BITS            = 5;  // Compatibility with old software

// Emulation API, OLD:
//   axis_cnf_len_data[31:0]   samples (in MIMO)  REG_WR_TXDMA_CNF_L
//   axis_tm_len_data[31:0]    TS                 REG_WR_TXDMA_CNF_T
//   axis_control_data[]                          REG_WR_TXDMA_COMB
//       [1:0] -- 11 -- FMT_16BIT; 00 -- STOP
//       [2]      Repeat
//       [3]      Format; 0 - SISO; 1 - MIMO
//     [6:4]      <reserved>
//       [7]      Reset
//       [8]      mute_b
//       [9]      mute_a
//      [10]      swap_ab
//      [11]      Update MuteAB/SwapAB during active mode
//
// New Style (TBD)
// |  bursts[4:0]  |  samples[12:0]  |  bytes[17:3] |
// |  ts[63:32]                                     |
// |  ts[31:0]                                      |
//
// Old style (deprecated)
// | 0 | nots | dw_samples[13:0]  | ts[47:32]       |     REG_WR_TXDMA_CNF_L
// |  ts[31:0]                                      |     REG_WR_TXDMA_CNF_T

// Old style (since 20240526)
// S bit tell odd or even number of samles in SISO mode
// | 0 | nots | samples[14:0]  | ts[46:32]          |     REG_WR_TXDMA_CNF_L
// |  ts[31:0]                                      |     REG_WR_TXDMA_CNF_T

wire                              dma_core_ready;
wire                              circ_core_ready;

wire [BURSTS_BITS-1:0]            nb_bursts;
reg [SAMPLES_WIDTH-1:0]           nb_samples;
reg [RAM_ADDR_WIDTH-1:DATA_BITS]  nb_bytes;
reg [TIMESTAMP_BITS-1:32]         nb_ts;

wire [14:0] oldstyle_samples       = axis_cnf_len_data[29:15]; // actual samples
wire [16:3] oldstyle_bytes         = (fe_format) ? oldstyle_samples : oldstyle_samples[14:1];

reg dma_tx_ready;

always @(posedge clk) begin
    if (rst) begin
        // nb_bursts <= 0;
        nb_samples <= 0;
        nb_bytes   <= 0;
        nb_ts      <= 0;
    end else begin
        if (axis_cnf_len_ready && axis_cnf_len_valid) begin
            nb_samples   <= oldstyle_samples;
            nb_bytes     <= oldstyle_bytes[16:DATA_BITS];
            nb_ts[47:32] <= { 1'b0, axis_cnf_len_data[14:0] };
            nb_ts[48]    <= axis_cnf_len_data[30];   //no-ts bit
        end
    end
end

wire [_USER_BUF_CONFIG_WIDTH-1:0] bufcmd_data  = { nb_ts, axis_tm_len_data, nb_bursts, nb_samples, nb_bytes };
wire                              bufcmd_valid = axis_tm_len_valid;
wire                              bufcmd_ready;
assign                            axis_cnf_len_ready = dma_tx_ready; //1'b1;
assign                            axis_tm_len_ready  = bufcmd_ready;
assign                            axis_control_ready = 1'b1;
assign                            nb_bursts = 5'h0;
wire control_ready;
reg  control_valid;
reg  txdma_active;
assign                            txdma_nactive = !txdma_active;

always @(posedge clk) begin
    if (rst) begin
        txdma_active  <= 1'b0;
        fe_format     <= 1'b0;
        control_valid <= 1'b0;
        dma_tx_ready  <= 0;
    end else begin
        control_valid <= 1'b0;
        dma_tx_ready  <= !txdma_active || circ_core_ready && dma_core_ready /*|| tran_usb_active*/;

        if (axis_control_valid && axis_control_ready) begin
            if (txdma_active == 1'b0 && axis_control_data[1:0] == 2'b11) begin
                control_valid <= !tran_usb_active; //1'b1;
                txdma_active  <= 1'b1;
                fe_format       <= axis_control_data[3];
                fe_mute       <= { axis_control_data[8], axis_control_data[9] };
                fe_swap       <= axis_control_data[10];
            end else if (txdma_active == 1'b1 && axis_control_data[1:0] == 2'b00) begin
                control_valid <= !tran_usb_active; //1'b1;
                txdma_active  <= 1'b0;
            end else if (axis_control_data[11]) begin
                fe_mute       <= { axis_control_data[8], axis_control_data[9] };
                fe_swap       <= axis_control_data[10];
            end
        end
    end
end

wire control_data = txdma_active;
wire fifo_full;

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

wire         stat_dmareq_ovf;

circ_dma_tx_engine #(
    .TIMESTAMP_BITS(TIMESTAMP_BITS),
    .USER_BUF_BITS(USER_BUF_BITS),
    .RAM_ADDR_WIDTH(RAM_ADDR_WIDTH),
    .DATA_BITS(DATA_BITS),
    .BURSTS_BITS(BURSTS_BITS),
    .BUS_ADDR_WIDTH(BUS_ADDR_WIDTH),
    .DEFAULT_MAX_BUFFER_SIZE(15'h7fff),   // Compatibility with old software (32kB max DMA buffer)
    .CHECK_BUFFER_REQ_SIZE(!FINE_RAMFIFO_AVAIL_CHECK),
    .ULTRA_SCALE(ULTRA_SCALE)
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

    //.m_new_idx_valid(m_new_idx_valid),
    //.m_new_idx_ready(m_new_idx_ready),

    // Burst processed and is ready to be reused
    .s_proc_idx_valid(s_proc_idx_valid),
    .s_proc_idx_ready(s_proc_idx_ready),

    // Burst decriptor data
    //.s_descr_araddr(s_descr_araddr),
    //.s_descr_rdata(s_descr_rdata)
    .m_descr_ready(m_descr_ready),
    .m_descr_valid(m_descr_valid),
    .m_descr_data(m_descr_data)
);


assign axis_stat_valid        = 1'b1;
assign axis_stat_m_valid      = 1'b1;
assign axis_stat_ts_valid     = 1'b1;
assign axis_stat_cpl_valid    = 1'b1;

assign axis_stat_ts_data      = m_dma_tsl_data;

wire [5:0] usrbuf_posted      = m_dma_stat_data[7:0];
wire [5:0] usrbuf_requested   = m_dma_stat_data[15:8];
wire [5:0] usrbuf_completed   = m_dma_stat_data[23:16];
wire [5:0] usrbuf_aired       = m_dma_stat_data[31:24];

reg [3:0]  fifo_addr_full;
wire [2:0] dma_state = {
    stat_dmareq_ovf,
    stat_notlp,
    circ_core_ready && dma_core_ready
};

//                    1b            4b                  3b
wire [7:0] dma_stat = { !txdma_nactive, fifo_addr_full, dma_state };
assign axis_stat_data = {usrbuf_completed[5:4], usrbuf_posted,             // Written by user
                         usrbuf_completed[3:2], usrbuf_aired,              // Sent out to air
                         usrbuf_completed[1:0], usrbuf_requested,          // Requested by DMA engine
                         dma_stat};

wire [15:0]   dropped_bursts    = fe_late_bursts;// stat_cpl_nodata;    // TODO: dropped frames due to TO
reg [3:0]     fifo_min_space;


reg [15:0]    bursts_sent;
always @(posedge clk) begin
    if (!txdma_active) begin
        bursts_sent <= 0;
    end else begin
        if (s_proc_idx_valid && s_proc_idx_ready) begin
            bursts_sent <= bursts_sent + 1'b1;
        end
    end
end


assign axis_stat_cpl_data = { bursts_sent, usb_fetx_stat[47:32] };
assign axis_stat_m_data   = tran_usb_active ? usb_fetx_stat[31:0] : { dropped_bursts, 16'h0000 };
                                        //2b                  //2b                      // 12b

always @(posedge clk) begin
    if (rst) begin
        fifo_addr_full <= 4'b0;
        fifo_min_space <= 4'b0;
    end else begin
        // TODO
    end
end

// Control & AUX
always @(posedge clk) begin
    if (rst) begin
        tx_buffprecharged <= 1'b0;
    end else begin
        if (txdma_nactive) begin
            tx_buffprecharged <= 1'b0;
        end else if (!tran_usb_active && (usrbuf_completed[0])) begin
            tx_buffprecharged <= 1'b1;
        end else if (tran_usb_active && (usb_fetx_stat[32])) begin
            tx_buffprecharged <= 1'b1;
        end
    end
end




endmodule
