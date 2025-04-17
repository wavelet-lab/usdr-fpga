// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module dma_tx_pcie #(
    parameter RAM_ADDR_WIDTH = 18,
    parameter BUS_ADDR_WIDTH = 32,
    parameter REQUEST_LEN_BITS = 12, //4KiB maximum request
    parameter DATA_BITS = 3,
    parameter USER_TAG_BITS = 6,
    parameter PCIE_TAG_BITS = 4,
    parameter ULTRA_SCALE = 0,
    parameter USE_DATA_CREDITS = 0
) (
    input clk,
    input rst,

    output core_ready,

    // for data credits calculations
    // TODO: add proper interface
    input   [RAM_ADDR_WIDTH:8]            s_consume_ram_addr,
    output                                fifo_full,
    output  [RAM_ADDR_WIDTH:8]            fifo_used,
    output  [PCIE_TAG_BITS:0]             pcie_tags_avail,
      
    // request
    input  [RAM_ADDR_WIDTH  :DATA_BITS]   s_rq_loc_addr,
    input  [BUS_ADDR_WIDTH-1:DATA_BITS]   s_rq_bus_addr,
    input  [RAM_ADDR_WIDTH-1:DATA_BITS]   s_rq_length,
    input                                 s_rq_valid,
    input  [USER_TAG_BITS-1:0]            s_rq_tag,
    output reg                            s_rq_ready,

    // competion
    output reg [USER_TAG_BITS-1:0]        m_rc_tag,
    output reg                            m_rc_valid,
    input                                 m_rc_ready,
    
    input  [2:0]                          cfg_max_req_sz,

    // memory op request
    output                                      m_tcq_valid,
    input                                       m_tcq_ready,
    output [RAM_ADDR_WIDTH - 1:DATA_BITS]       m_tcq_laddr,
    output [BUS_ADDR_WIDTH - 1:DATA_BITS]       m_tcq_raddr,
    output [REQUEST_LEN_BITS - 1:DATA_BITS]     m_tcq_length,
    output [PCIE_TAG_BITS-1:0]                  m_tcq_tag,

    // Request termination, s_tcq_ctag can be reused
    input                                       m_tcq_cvalid,
    output                                      m_tcq_cready,
    input  [PCIE_TAG_BITS-1:0]                  m_tcq_ctag
);

// cfg_max_req_sz
// 000 - 128
// 001 - 256
// 010 - 512
// 011 - 1024
// 100 - 2048
// 101 - 4096

wire [2:0]                         cfg_max_req_sz_d = (cfg_max_req_sz > 3'b101) ? 3'b101 : cfg_max_req_sz;
wire [RAM_ADDR_WIDTH-1:DATA_BITS]  pcie_max_req     = (1'b1 << (cfg_max_req_sz_d + 7 - DATA_BITS)) - 1'b1;

reg  [RAM_ADDR_WIDTH:DATA_BITS]    len_requested;
wire [RAM_ADDR_WIDTH:DATA_BITS]    len_requested_plus = len_requested + 1'b1;
wire [RAM_ADDR_WIDTH:DATA_BITS]    len_remaining = s_rq_length - len_requested_plus;
wire [RAM_ADDR_WIDTH-1:DATA_BITS]  len_pcie_req  = (len_remaining > pcie_max_req) ? pcie_max_req : len_remaining;
wire                               last_pcie_req = (len_remaining > pcie_max_req) ? 1'b0 : 1'b1;

reg [RAM_ADDR_WIDTH-1:7]           req_cnt;
wire                               tag_valid;

wire [RAM_ADDR_WIDTH:DATA_BITS]    m_tcq_laddr_e      = s_rq_loc_addr + len_requested_plus;
wire [RAM_ADDR_WIDTH-1:8]          cfg_max_req_sz_e   = (cfg_max_req_sz_d == 0) ? 1 : (1 << (cfg_max_req_sz_d - 1));
wire [RAM_ADDR_WIDTH:8]            loc_addr_available = s_consume_ram_addr - m_tcq_laddr_e[RAM_ADDR_WIDTH:8] - cfg_max_req_sz_e + ((1<<(RAM_ADDR_WIDTH-8)) - 1'b1);
assign                             fifo_full          = loc_addr_available[RAM_ADDR_WIDTH];
assign                             fifo_used          = m_tcq_laddr_e[RAM_ADDR_WIDTH:8] - s_consume_ram_addr;

always @(posedge clk) begin
    if (rst) begin
        s_rq_ready    <= 1'b0;
        len_requested <= ~0;
        req_cnt       <= 0;
    end else begin

        if (m_tcq_valid && m_tcq_ready) begin
            len_requested <= len_requested + len_pcie_req + 1'b1;
            req_cnt       <= req_cnt + 1'b1;
            if (last_pcie_req) begin
                s_rq_ready <= 1'b1;
            end
        end

        if (s_rq_ready) begin
            s_rq_ready    <= 1'b0;
            len_requested <= ~0;
            req_cnt       <= 0;
        end

    end
end

assign m_tcq_valid  = tag_valid && s_rq_valid && !s_rq_ready && (!USE_DATA_CREDITS || !fifo_full);
assign m_tcq_laddr  = m_tcq_laddr_e[RAM_ADDR_WIDTH-1:DATA_BITS];
assign m_tcq_raddr  = s_rq_bus_addr + len_requested_plus[RAM_ADDR_WIDTH-1:DATA_BITS];
assign m_tcq_length = len_pcie_req;

reg [PCIE_TAG_BITS-1:0] free_tag;

// No need to check ready, since we're pushing back allocated values from the pull
tag_allocator #(.PCIE_TAG_BITS(PCIE_TAG_BITS)) tag_alloc (
    .clk(clk),
    .rst(rst),

    .core_ready(core_ready),
    .notags(),
    .tag_fifo_used(pcie_tags_avail),

    .m_tag_alloc_data(m_tcq_tag),
    .m_tag_alloc_ready(s_rq_valid && s_rq_ready),
    .m_tag_alloc_valid(tag_valid),

    .s_tag_free_data(free_tag),
    .s_tag_free_valid(m_rc_valid && m_rc_ready),
    .s_tag_free_ready()
);

wire [RAM_ADDR_WIDTH-1:7] cpl_req_cnt;
wire [RAM_ADDR_WIDTH-1:7] cpl_prc_cnt;
wire                      cpl_req_fin;
wire [USER_TAG_BITS-1:0]  cpl_req_tag;

// Request progress:     f | cnt_req
ram_sxp #(
    .DATA_WIDTH( USER_TAG_BITS + 1 + (RAM_ADDR_WIDTH - 7) ),
    .ADDR_WIDTH(PCIE_TAG_BITS),
    .ULTRA_SCALE(ULTRA_SCALE),
    .MODE_SDP(1)
) req_progress (
   .wclk(clk),
   .we( m_tcq_valid && m_tcq_ready ),
   .waddr( m_tcq_tag ),
   .wdata( { s_rq_tag, last_pcie_req, req_cnt } ),

   .raddr( m_tcq_ctag ),
   .rdata( { cpl_req_tag, cpl_req_fin, cpl_req_cnt } )
);

assign m_tcq_cready = !m_rc_valid;

//Initial reset
reg [PCIE_TAG_BITS:0] reset_idx;
wire                  tag_reset_ready = reset_idx[PCIE_TAG_BITS];
always @(posedge clk) begin
    if (rst) begin
        reset_idx <= 0;
    end else begin
        if (!tag_reset_ready) begin
            reset_idx <= reset_idx + 1'b1;
        end
    end
end

wire completion_done = cpl_req_fin && (cpl_req_cnt == cpl_prc_cnt);

ram_sxp #(
    .DATA_WIDTH((RAM_ADDR_WIDTH - 7)),
    .ADDR_WIDTH(PCIE_TAG_BITS),
    .ULTRA_SCALE(ULTRA_SCALE),
    .MODE_SDP(1)
) req_completion (
   .wclk(clk),
   .we(    !tag_reset_ready || m_tcq_cvalid && m_tcq_cready ),
   .waddr( !tag_reset_ready ? reset_idx[PCIE_TAG_BITS-1:0] : m_tcq_ctag ),
   .wdata( !tag_reset_ready || completion_done ? 1'b0 : cpl_prc_cnt + 1'b1),

   .raddr( m_tcq_ctag ),
   .rdata( cpl_prc_cnt )
);

always @(posedge clk) begin
    if (rst) begin
        m_rc_valid   <= 1'b0;
    end else begin
        if (m_rc_valid && m_rc_ready) begin
            m_rc_valid   <= 1'b0;
        end

        if (m_tcq_cvalid && m_tcq_cready && completion_done) begin
            m_rc_valid <= 1'b1;
            m_rc_tag   <= cpl_req_tag;
            free_tag   <= m_tcq_ctag;
        end
    end
end



endmodule
