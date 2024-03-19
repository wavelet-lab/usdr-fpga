// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module al_ram_to_pcie_memwr #(
    parameter LOCAL_ADDR_WIDTH = 17,
    parameter REMOTE_ADDR_WIDTH = 32,
    parameter MEM_TAG = 1,
    parameter REQUEST_LEN_BITS = 6,
    parameter DATA_BITS = 4, // 3 - 64 bit, 4 - 128 bit, 5 - 256 bit
    parameter DATA_WIDTH_ = 8 << DATA_BITS,
    parameter BRAM_STAGES = 1,
    parameter ULTRA_SCALE = 0,
    parameter KEEP_WIDTH_ = DATA_WIDTH_/32,
    parameter USER_WIDTH_ = ULTRA_SCALE ? 62 : 1,
    parameter TX_BUF_CTRL = 0,  // for 7-Series
    parameter EN64BIT = 0       // for 7-Series
)(
    input                                      clk,
    input                                      rst,

    input                                      s_tcq_valid,
    output reg                                 s_tcq_ready,
    input [LOCAL_ADDR_WIDTH - 1:DATA_BITS]     s_tcq_laddr,
    input [REMOTE_ADDR_WIDTH - 1:DATA_BITS]    s_tcq_raddr,
    input [REQUEST_LEN_BITS - 1:0]             s_tcq_length,
    input [MEM_TAG-1:0]                        s_tcq_tag,

    // Bus data move confirmation
    output reg                                 s_tcq_cvalid,
    input                                      s_tcq_cready,
    output reg [MEM_TAG-1:0]                   s_tcq_ctag,

    input [15:0]                               cfg_pcie_reqid,
    input [1:0]                                cfg_pcie_attr,
    input [5:0]                                pcie7s_tx_buf_av,

    // AXIs PCIe TX/RQ
    input                                      m_axis_tx_tready,
    output reg [DATA_WIDTH_-1:0]               m_axis_tx_tdata,
    output reg [KEEP_WIDTH_-1:0]               m_axis_tx_tkeep,
    output reg                                 m_axis_tx_tlast,
    output reg                                 m_axis_tx_tvalid,
    output reg [USER_WIDTH_-1:0]               m_axis_tx_tuser,

    // RAM interface
    output reg [LOCAL_ADDR_WIDTH-1:DATA_BITS]  m_al_araddr,
    output reg                                 m_al_arvalid,
    output reg                                 m_al_arid,    // 1 -- last beat;
    input                                      m_al_arready,

    input [DATA_WIDTH_-1:0]                    m_al_rdata,
    input                                      m_al_rvalid,
    output                                     m_al_rready,
    input                                      m_al_rid
);

wire can_send_fc = (ULTRA_SCALE) ? 1'b1 : !TX_BUF_CTRL || (pcie7s_tx_buf_av > 3);

wire [DATA_WIDTH_:0] data_to_pcie;
generate
    if (!ULTRA_SCALE) begin
        assign data_to_pcie[31:0]  = { m_al_rdata[7:0],   m_al_rdata[15:8],  m_al_rdata[23:16], m_al_rdata[31:24] };
        assign data_to_pcie[63:32] = { m_al_rdata[39:32], m_al_rdata[47:40], m_al_rdata[55:48], m_al_rdata[63:56] };
    end else begin
        assign data_to_pcie = m_al_rdata;
    end
endgenerate

wire [6:0]                  cmd_32         = 7'b10_00000;  // MemWr32
wire [6:0]                  cmd_64         = 7'b11_00000;  // MemWr64

wire [10:0]                 pcie_lm_length = { s_tcq_length, {(DATA_BITS - 2){1'b1}} } + 1'b1;

reg   [REQUEST_LEN_BITS:0]  pcie_burst_counter;
wire                        pcie_burst_last        = pcie_burst_counter[REQUEST_LEN_BITS];
wire  [REQUEST_LEN_BITS:0]  pcie_burst_counter_nxt = pcie_burst_counter - 1'b1;
wire                        pcie_burst_last_nxt    = pcie_burst_counter_nxt[REQUEST_LEN_BITS];

wire  [REQUEST_LEN_BITS:0]  pcie_burst_counter_req     = { 1'b0, s_tcq_length };
wire  [REQUEST_LEN_BITS:0]  pcie_burst_counter_req_nxt = pcie_burst_counter_req - 1'b1;

reg [31:0]                  tmp_axis_data_wrap; //7s: DWORD wrap-around
reg                         pcie_64bit;         //7s: Active 64bit memory transaction
reg                         pcie_pkt_last;
reg [1:0]                   dma_state;
localparam [1:0]
    DMA_RAM_LOAD       = 0,
    DMA_FILL_PCIE_HDR  = 1, // Filling PCIe header  (+address on US)
    DMA_FILL_PCIE_ADDR = 2, // Filling PCIe address (data transfer on US)
    DMA_PCIE_TRANSFER  = 3; // Filling PCIe data

wire [63:0] s_tcq_raddr_aligned = { s_tcq_raddr, {DATA_BITS{1'b0}} };
assign m_al_rready = (dma_state == DMA_FILL_PCIE_ADDR || (dma_state == DMA_PCIE_TRANSFER && (ULTRA_SCALE || ~pcie_pkt_last))) && ((m_axis_tx_tready || ~m_axis_tx_tvalid)/*m_axis_tx_tready*/);

wire pcie_64bit_act = ULTRA_SCALE ? 1'b1 : EN64BIT && pcie_64bit;

always @(posedge clk) begin
    if (ULTRA_SCALE) begin
        m_axis_tx_tkeep          <= {(1<<(DATA_BITS-2)){1'b1}};
        pcie_64bit               <= 1'b1;
    end

  if (rst) begin
    dma_state            <= DMA_RAM_LOAD;
    m_axis_tx_tvalid     <= 1'b0;

    s_tcq_ready          <= 1'b0;
    s_tcq_cvalid         <= 1'b0;

    m_al_arvalid         <= 1'b0;
  end else begin

    if (m_axis_tx_tready && m_axis_tx_tvalid) begin
        m_axis_tx_tvalid     <= 1'b0;
    end

    if (m_al_arvalid && m_al_arready) begin
        pcie_burst_counter   <= pcie_burst_counter_nxt;
        m_al_arvalid         <= !pcie_burst_last;
        m_al_araddr          <= m_al_araddr + 1'b1;
        m_al_arid            <= pcie_burst_last_nxt;
    end

    if (s_tcq_ready && s_tcq_valid) begin
      s_tcq_ready <= 1'b0;
    end

    if (s_tcq_cvalid && s_tcq_cready) begin
      s_tcq_cvalid <= 1'b0;
    end

    case (dma_state)
     DMA_RAM_LOAD: begin
      if (s_tcq_valid && ~s_tcq_ready && can_send_fc) begin //do we need ~m_al_arvalid?
        m_al_arvalid         <= 1'b1;
        m_al_araddr          <= s_tcq_laddr;
        m_al_arid            <= pcie_burst_counter_req_nxt[REQUEST_LEN_BITS];
        pcie_burst_counter   <= pcie_burst_counter_req_nxt;
        dma_state            <= DMA_FILL_PCIE_HDR;

        pcie_64bit           <= (ULTRA_SCALE) ? 1'b1 : (s_tcq_raddr_aligned[63:32] != 32'h0000_0000);
        pcie_pkt_last        <= 1'b0;
      end
     end

     DMA_FILL_PCIE_HDR: begin
      if ((m_axis_tx_tready || ~m_axis_tx_tvalid) && (s_tcq_cready || ~s_tcq_cvalid)) begin
        m_axis_tx_tvalid <= 1'b1;
        m_axis_tx_tlast  <= 1'b0;
        dma_state        <= DMA_FILL_PCIE_ADDR;

        if (ULTRA_SCALE) begin
            m_axis_tx_tdata[63:0]    <= s_tcq_raddr_aligned;
            m_axis_tx_tdata[74:64]   <= pcie_lm_length;
            m_axis_tx_tdata[78:75]   <= 4'b0001;        // MemWR
            m_axis_tx_tdata[79]      <= 1'b0;           // Poisoned Request
            m_axis_tx_tdata[95:80]   <= cfg_pcie_reqid; // Requester ID
            m_axis_tx_tdata[103:96]  <= 8'h00;          // Tag
            m_axis_tx_tdata[119:104] <= 16'h0000;       // Completer ID
            m_axis_tx_tdata[120]     <= 1'b0;           // Request ID Enable (must be 0 for endpoint)
            m_axis_tx_tdata[123:121] <= 3'b000;         // TC
            m_axis_tx_tdata[126:124] <= {1'b0, cfg_pcie_attr};   // Attr
            m_axis_tx_tdata[127]     <= 1'b0;           // Force ECRC

            m_axis_tx_tuser[7:0]     <= 8'hff;
            m_axis_tx_tuser[10:8]    <= 0;
            m_axis_tx_tuser[USER_WIDTH_-1:11] <= 0;

            s_tcq_ready              <= 1'b1;
            s_tcq_ctag               <= s_tcq_tag;
        end else begin
            m_axis_tx_tkeep  <= 2'b11;
            m_axis_tx_tdata  <= {
                cfg_pcie_reqid,
                8'h00, 8'hff,
                1'b0, pcie_64bit_act ? cmd_64 : cmd_32, 8'h00,
                1'b0, 1'b0, cfg_pcie_attr, 2'b00, pcie_lm_length[9:0] };
        end

      end
     end

     DMA_FILL_PCIE_ADDR, DMA_PCIE_TRANSFER: begin
      if ((m_axis_tx_tready || ~m_axis_tx_tvalid)/*m_axis_tx_tready*/ && (m_al_rvalid || !ULTRA_SCALE && pcie_pkt_last)) begin
        if (!ULTRA_SCALE && dma_state == DMA_FILL_PCIE_ADDR) begin
            s_tcq_ready       <= 1'b1;
            s_tcq_ctag         <= s_tcq_tag;
            dma_state          <= DMA_PCIE_TRANSFER;
        end

        m_axis_tx_tvalid   <= 1'b1;
        if (pcie_64bit_act) begin
            m_axis_tx_tdata    <= (!ULTRA_SCALE && dma_state == DMA_PCIE_TRANSFER) ? { s_tcq_raddr_aligned[31:0], s_tcq_raddr_aligned[63:32] } : data_to_pcie;
            m_axis_tx_tlast    <= m_al_rid;
        end else begin
            m_axis_tx_tkeep    <= (pcie_pkt_last) ? 2'b01 : 2'b11;
            m_axis_tx_tdata    <= { data_to_pcie[31:0], dma_state == DMA_PCIE_TRANSFER ? tmp_axis_data_wrap : s_tcq_raddr_aligned[31:0] };
            m_axis_tx_tlast    <= pcie_pkt_last;

            tmp_axis_data_wrap <= data_to_pcie[63:32];
            pcie_pkt_last      <= m_al_rid;
        end

        if (m_al_rid && (pcie_64bit_act || ~pcie_pkt_last)) begin
            s_tcq_cvalid     <= 1'b1;
        end

        if (pcie_64bit_act && m_al_rid || (!pcie_64bit_act && pcie_pkt_last)) begin
          if (s_tcq_valid && ~s_tcq_ready && can_send_fc) begin
            m_al_arvalid         <= 1'b1;
            m_al_araddr          <= s_tcq_laddr;
            m_al_arid            <= pcie_burst_counter_req_nxt[REQUEST_LEN_BITS];
            pcie_burst_counter   <= pcie_burst_counter_req_nxt;
            dma_state            <= DMA_FILL_PCIE_HDR;

            pcie_64bit           <= (ULTRA_SCALE) ? 1'b1 : (s_tcq_raddr_aligned[63:32] != 32'h0000_0000);
            pcie_pkt_last        <= 1'b0;
          end else begin
            dma_state          <= DMA_RAM_LOAD;
          end
        end
      end
     end


    endcase

  end
end


endmodule
