// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module pcie_rq_rc_al_mem #(
    parameter LOCAL_ADDR_WIDTH = 17,
    parameter REMOTE_ADDR_WIDTH = 32,
    parameter REQUEST_LEN_BITS = 6,
    parameter MEM_TAG = 1,
    parameter DATA_BITS = 4, // 3 - 64 bit, 4 - 128 bit, 5 - 256 bit
    parameter DATA_WIDTH_ = 8 << DATA_BITS,
    parameter ULTRA_SCALE = 0,
    parameter KEEP_WIDTH_ = DATA_WIDTH_/32,
    parameter USER_WIDTH_ = ULTRA_SCALE ? 62 : 1,
    parameter EN64BIT = 0, // for 7-Series,
    parameter STAT_CNTR_WIDTH = 20,
    parameter PCIE_TAG_BITS = 5  //32 TAGs in total by default
) (
    input                                      clk,
    input                                      rst,

    input                                      s_tcq_valid,
    output                                     s_tcq_ready,
    input [LOCAL_ADDR_WIDTH - 1:DATA_BITS]     s_tcq_laddr,
    input [REMOTE_ADDR_WIDTH - 1:DATA_BITS]    s_tcq_raddr,
    input [REQUEST_LEN_BITS - 1:DATA_BITS]     s_tcq_length,
    input [MEM_TAG-1:0]                        s_tcq_tag,

    // Request termination, s_tcq_ctag can be reused
    output reg                                 s_tcq_cvalid,
    input                                      s_tcq_cready,
    output reg [MEM_TAG-1:0]                   s_tcq_ctag,
    //output reg                                 s_tcq_cok,

    output                                     core_ready,
    input [15:0]                               cfg_pcie_reqid,
    input [1:0]                                cfg_pcie_attr,
    input [5:0]                                pcie7s_tx_buf_av,

    // AXIs PCIe RQ
    input                                      m_axis_rq_tready,
    output reg [DATA_WIDTH_-1:0]               m_axis_rq_tdata,
    output reg [KEEP_WIDTH_-1:0]               m_axis_rq_tkeep,
    output reg                                 m_axis_rq_tlast,
    output reg                                 m_axis_rq_tvalid,
    output     [USER_WIDTH_-1:0]               m_axis_rq_tuser,
    //input  [7:0] pcie_rq_tag0,
    //input        pcie_rq_tag_vld0,
    //input  [1:0] pcie_tfc_nph_av,
    //input  [1:0] pcie_tfc_npd_av,

    // AXIS PCIe RC
    output                                     s_axis_rc_tready,
    input      [DATA_WIDTH_-1:0]               s_axis_rc_tdata,
    input      [KEEP_WIDTH_-1:0]               s_axis_rc_tkeep,
    input                                      s_axis_rc_tlast,
    input                                      s_axis_rc_tvalid,
    input      [USER_WIDTH_-1:0]               s_axis_rc_tuser,

    //FIFO
    output reg [LOCAL_ADDR_WIDTH-1:DATA_BITS]  m_al_wraddr,
    output     [DATA_WIDTH_-1:0]               m_al_wdata,
    output                                     m_al_wvalid,
    output reg  [MEM_TAG-1:0]                  m_al_wtag,    // The same tag as in s_tcq_tag
    input                                      m_al_wready,

    output                                     stat_notlp, // There's no non-posted TLP in the fly
    output [STAT_CNTR_WIDTH-1:0]               stat_cpl_nodata,
    
    input [3:0]                                extra_data
);

// For UltraScale use 128/256bit Address-Align mode with External Tag Management
// For 7-Series use 64bit interface (with externel CPL/MEM stream separation)

localparam MEM_RD32_FMT_TYPE =      7'b00_00000;   // 3DW
localparam MEM_RD64_FMT_TYPE =      7'b01_00000;   // 4DW

localparam CPL_FMT_TYPE =           7'b00_01010;   // 3DW
localparam CPL_DATA_FMT_TYPE =      7'b10_01010;   // 3DW + data
//memrd32 // memrd64


wire [PCIE_TAG_BITS-1:0] tag_alloc_data;
wire                     tag_alloc_ready;
wire                     tag_alloc_valid;

wire [PCIE_TAG_BITS-1:0] tag_free_data;
wire                     tag_free_valid;
wire                     tag_free_ready;

tag_allocator #(.PCIE_TAG_BITS(PCIE_TAG_BITS)) tag_alloc (
    .clk(clk),
    .rst(rst),

    .core_ready(core_ready),
    .notags(stat_notlp),

    .m_tag_alloc_data(tag_alloc_data),
    .m_tag_alloc_ready(tag_alloc_ready),
    .m_tag_alloc_valid(tag_alloc_valid),

    .s_tag_free_data(tag_free_data),
    .s_tag_free_valid(tag_free_valid),
    .s_tag_free_ready(tag_free_ready)
);

// Allocation requests
wire [PCIE_TAG_BITS-1:0]            completion_pcie_tag;
wire [LOCAL_ADDR_WIDTH-1:DATA_BITS] completion_addr;
wire [MEM_TAG-1:0]                  completion_utag;

ram_sxp #(
    .DATA_WIDTH(LOCAL_ADDR_WIDTH - DATA_BITS + MEM_TAG),
    .ADDR_WIDTH(PCIE_TAG_BITS),
    .ULTRA_SCALE(ULTRA_SCALE),
    .MODE_SDP(1)
) req_progress (
   .wclk(clk),
   .we(tag_alloc_valid && tag_alloc_ready),
   .waddr(tag_alloc_data),
   .wdata({ s_tcq_laddr + s_tcq_length + 1'b1, s_tcq_tag }),

   .raddr(completion_pcie_tag),
   .rdata({ completion_addr, completion_utag })
);

// Stages for allocation
// 1. Obtain REQ tag / fill allocation data
// 2. Post request

localparam [1:0]
    REQ_TAG_ALLOC_DW0 = 0,
    REQ_POST_DW1      = 1;

reg [0:0]   req_stage;
wire [7:0]  pcie_tag    = tag_alloc_data;
wire [12:2] pcie_length;
wire [63:2] pcie_addr;

assign pcie_length[12:DATA_BITS]    = s_tcq_length + 1'b1;
assign pcie_length[DATA_BITS - 1:2] = 0;

assign pcie_addr[63:DATA_BITS]      = s_tcq_raddr;
assign pcie_addr[DATA_BITS - 1:2]   = 0;


//wire addr64bit7s = (EN64BIT && (REMOTE_ADDR_WIDTH > 32)) ? s_tcq_raddr[REMOTE_ADDR_WIDTH-1:32] != 0 : 1'b0;
wire addr64bit7s;
generate
if (EN64BIT && (REMOTE_ADDR_WIDTH > 32)) begin
    assign addr64bit7s = s_tcq_raddr[REMOTE_ADDR_WIDTH-1:32] != 0;
end else begin
    assign addr64bit7s = 1'b0;
end
endgenerate

assign s_tcq_ready = (ULTRA_SCALE ? tag_alloc_ready && tag_alloc_valid : req_stage == REQ_POST_DW1 && (!m_axis_rq_tvalid || m_axis_rq_tready));
assign m_axis_rq_tuser = { 1'b0, 3'b000, 4'hf, 4'hf }; // {discontinue, add_offset[2:0], last_be, first_be }


always @(posedge clk) begin
    if (rst) begin
        req_stage        <= REQ_TAG_ALLOC_DW0;
        m_axis_rq_tvalid <= 1'b0;


    end else begin
        if (m_axis_rq_tvalid && m_axis_rq_tready) begin
            m_axis_rq_tvalid <= 1'b0;
        end

        if ((req_stage == REQ_TAG_ALLOC_DW0) || ULTRA_SCALE) begin
            if (tag_alloc_ready && tag_alloc_valid) begin
                if (ULTRA_SCALE) begin
                    m_axis_rq_tdata[63:0]    <= { pcie_addr, 2'b00 };
                    m_axis_rq_tdata[74:64]   <= pcie_length;
                    m_axis_rq_tdata[78:75]   <= 4'b0000;        // MemRd
                    m_axis_rq_tdata[79]      <= 1'b0;           // Poisoned Request
                    m_axis_rq_tdata[95:80]   <= cfg_pcie_reqid; // Requester ID
                    m_axis_rq_tdata[103:96]  <= pcie_tag;       // Tag
                    m_axis_rq_tdata[119:104] <= 16'h0000;       // Completer ID
                    m_axis_rq_tdata[120]     <= 1'b0;           // Request ID Enable (must be 0 for endpoint)
                    m_axis_rq_tdata[123:121] <= 3'b000;         // TC
                    m_axis_rq_tdata[126:124] <= {1'b0, cfg_pcie_attr};   // Attr
                    m_axis_rq_tdata[127]     <= 1'b0;           // Force ECRC

                    m_axis_rq_tkeep          <= 4'b1111;
                    m_axis_rq_tlast          <= 1'b1;
                end else begin
                    m_axis_rq_tdata[63:32]   <= { cfg_pcie_reqid, pcie_tag, 8'hff};
                    m_axis_rq_tdata[31:0]    <= { 1'b0, addr64bit7s ? MEM_RD64_FMT_TYPE : MEM_RD32_FMT_TYPE, 8'h00,   2'b00, cfg_pcie_attr, 2'b00, pcie_length[11:2] };
                    m_axis_rq_tkeep          <= 2'b11;
                    m_axis_rq_tlast          <= 1'b0;
                    req_stage                <= REQ_POST_DW1;
                end

                m_axis_rq_tvalid <= 1'b1;
            end
        end else if (req_stage == REQ_POST_DW1) begin
            if (!m_axis_rq_tvalid || m_axis_rq_tready) begin
                if (!EN64BIT) begin
                    m_axis_rq_tdata[31:0]  <= {pcie_addr[31:2], 2'b00};
                end else begin
                    m_axis_rq_tdata[31:0]  <= addr64bit7s ? pcie_addr[63:32] : {pcie_addr[31:2], 2'b00};
                    m_axis_rq_tdata[63:32] <= {pcie_addr[31:2], 2'b00};
                end
                m_axis_rq_tkeep       <= addr64bit7s ? 2'b11 : 2'b01;
                m_axis_rq_tlast       <= 1'b1;
                m_axis_rq_tvalid      <= 1'b1;

                req_stage             <= REQ_TAG_ALLOC_DW0;
            end
        end
    end
end

wire tx_buffer_ready   = ULTRA_SCALE ? 1'b1 : (pcie7s_tx_buf_av > 2);
assign tag_alloc_ready = s_tcq_valid && (ULTRA_SCALE ? 1'b1 : req_stage == REQ_TAG_ALLOC_DW0) && !m_axis_rq_tvalid && tx_buffer_ready;


localparam CPL_UR  = 3'b001; // Unsupported Request
localparam CPL_CRS = 3'b010; // Configuration Request Retry Status
localparam CPL_CA  = 3'b100; // Completer Abort

// PCIe completion format 3DW
// 0: R:1 FMT_TYPE:7  |  R:1 TC:2 R:3 || DP:1 EP:1 ATTR:2 R:2 LENGTH:10
// 1: COMPLETION_ID:16                || STAT:3 BCM:1     BYTE_COUNT:12
// 2: REQ_ID:16                       || TAG:8         R:1 LOWER_ADDR:7

// DW0/1
wire [9:0]  tlp_length  = s_axis_rc_tdata[9:0];
wire        tlp_ep      = s_axis_rc_tdata[14];
wire        tlp_dp      = s_axis_rc_tdata[15];
wire [6:0]  tlp_type    = s_axis_rc_tdata[30:24];
wire [3:0]  tlp_ldwbe   = s_axis_rc_tdata[39:36];
wire [3:0]  tlp_fdwbe   = s_axis_rc_tdata[35:32];
wire [11:0] tlp_bytes   = s_axis_rc_tdata[43:32];
wire [2:0]  cpld_status = s_axis_rc_tdata[47:45];

wire [12:DATA_BITS] corr_bytes = { tlp_bytes[11:DATA_BITS] == 0 ? 1'b1 : 1'b0, tlp_bytes[11:DATA_BITS] };
wire                cpl_last   = (tlp_bytes[11:2] == tlp_length);

// DW2
wire [6:0] tlp_lower_addr = s_axis_rc_tdata[6:0];
wire [7:0] tlp_tag        = s_axis_rc_tdata[15:8];
// DW3 -- data0

///////////////////////////////////////////////////////////////////
// Ultrasacle 128/256 bit completion fields
wire [12:0] bytec_count_us= s_axis_rc_tdata[28:16]; //13 bit Byte Count
wire [11:0] lower_addr_us = s_axis_rc_tdata[11:0];
wire [3:0] error_code_us  = s_axis_rc_tdata[15:12]; //Error code
wire [2:0] cpld_status_us = s_axis_rc_tdata[45:43]; //Completion Status
wire       cpl_last_us    = s_axis_rc_tdata[30];    //Request Completed
//wire [7:0] tlp_tag_us     = s_axis_rc_tdata[71:64];

assign completion_pcie_tag = (ULTRA_SCALE) ? s_axis_rc_tdata[71:64] : tlp_tag;

// Competion
reg [12:DATA_BITS] remaining_bytes;

reg        [1:0] cpl_state;
localparam [1:0]
    ST_CPL_HDR = 0,
    ST_CPL_HDW1 = 1,
    ST_CPL_DATA = 2;

reg [31:0] data_cached;
reg        last_cpl_cached;
reg [2:0]  cpld_status_cached;

assign tag_free_data  = completion_pcie_tag;

// Assume assertation of s_axis_rc_tlast in HDW1 is an abnormal transaction finilization;
// however, this maight be a valid transaction with just a single payload DW
// that we ignore here as a minimal requested transfer is 64bit
//
assign tag_free_valid = s_axis_rc_tvalid && s_axis_rc_tready && (ULTRA_SCALE ?
    cpl_state == ST_CPL_HDR  && cpl_last_us :
    cpl_state == ST_CPL_HDW1 && (last_cpl_cached || s_axis_rc_tlast));

// TODO: cpld_status_us parse
reg [STAT_CNTR_WIDTH-1:0] comp_no_data;

always @(posedge clk) begin
    if (rst) begin
        cpl_state    <= ST_CPL_HDR;
        s_tcq_cvalid <= 1'b0;
        comp_no_data <= 0;
    end else begin

        if (s_tcq_cvalid && s_tcq_cready) begin
            s_tcq_cvalid <= 1'b0;
        end

        if (ULTRA_SCALE) begin
            if (cpl_state[1] == 1'b0 && s_axis_rc_tvalid && s_axis_rc_tready) begin
                last_cpl_cached   <= cpl_last_us;

                m_al_wraddr       <= completion_addr - bytec_count_us[12:DATA_BITS];
                m_al_wtag         <= completion_utag;

                cpl_state[1]      <= s_axis_rc_tlast ? 1'b0 : 1'b1;
            end else if (cpl_state[1] == 1'b1 && s_axis_rc_tvalid && s_axis_rc_tready) begin
                m_al_wraddr       <= m_al_wraddr + 1'b1;

                if (s_axis_rc_tlast) begin
                    s_tcq_cvalid  <= last_cpl_cached;
                    s_tcq_ctag    <= m_al_wtag;
                end

                cpl_state[1]      <= s_axis_rc_tlast ? 1'b0 : 1'b1;
            end
        end else begin
            case (cpl_state)

            ST_CPL_HDR: begin
                if (s_axis_rc_tvalid && s_axis_rc_tready) begin
                    last_cpl_cached    <= cpl_last;
                    remaining_bytes    <= corr_bytes;
                    cpld_status_cached <= cpld_status;
                    cpl_state          <= (s_axis_rc_tlast) ? ST_CPL_HDR : ST_CPL_HDW1;
                end
            end

            ST_CPL_HDW1: begin
                if (s_axis_rc_tvalid && s_axis_rc_tready) begin
                    m_al_wraddr <= completion_addr - remaining_bytes;
                    m_al_wtag   <= completion_utag;

                    data_cached <= s_axis_rc_tdata[63:32];

                    if (s_axis_rc_tlast) begin
                        comp_no_data <= comp_no_data + 1'b1;

                        // Completion without data; analyze cpld_status_cached
                        s_tcq_cvalid <= 1'b1;
                        s_tcq_ctag   <= completion_utag;

                        cpl_state    <= ST_CPL_HDR;
                    end else begin
                        cpl_state    <= ST_CPL_DATA;
                    end
                end
            end

            ST_CPL_DATA: begin
                if (s_axis_rc_tvalid && s_axis_rc_tready) begin
                    data_cached   <= s_axis_rc_tdata[63:32];

                    m_al_wraddr       <= m_al_wraddr + 1'b1;
                    cpl_state         <= s_axis_rc_tlast ? ST_CPL_HDR : ST_CPL_DATA;

                    if (s_axis_rc_tlast) begin
                        s_tcq_cvalid <= last_cpl_cached;
                        s_tcq_ctag   <= m_al_wtag;

                        cpl_state    <= ST_CPL_HDR;
                    end else begin
                        cpl_state    <= ST_CPL_DATA;
                    end
                end
            end

            endcase
        end

    end
end

wire [63:0] data_7s_swapped = {
    s_axis_rc_tdata[7:0], s_axis_rc_tdata[15:8], s_axis_rc_tdata[23:16], s_axis_rc_tdata[31:24],
    data_cached[7:0],     data_cached[15:8],     data_cached[23:16],     data_cached[31:24] };

assign m_al_wdata       = ULTRA_SCALE ? s_axis_rc_tdata : data_7s_swapped;
assign m_al_wvalid      = (cpl_state == ST_CPL_DATA) && s_axis_rc_tvalid;
assign s_axis_rc_tready = (cpl_state == ST_CPL_DATA) ? (!m_al_wvalid || m_al_wready) : !m_al_wvalid && (!s_tcq_cvalid || s_tcq_cready);

assign stat_cpl_nodata  = comp_no_data;

endmodule
