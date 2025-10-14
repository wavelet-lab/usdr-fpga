// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module al_ram_to_rx_usbeps #(
    parameter LOCAL_ADDR_WIDTH = 17,
    parameter MEM_TAG = 1,
    parameter REQUEST_LEN_BITS = 6,
    parameter DATA_BITS = 3, // 3 - 64 bit, 4 - 128 bit, 5 - 256 bit
    parameter DATA_WIDTH_ = 8 << DATA_BITS,
    parameter BRAM_STAGES = 1,
    parameter ULTRA_SCALE = 0,
    parameter DUAL_NTFY_EN = 1
)(
    input                                      clk,
    input                                      rst,

    input                                      s_tcq_valid,
    output reg                                 s_tcq_ready,
    input [LOCAL_ADDR_WIDTH - 1:DATA_BITS]     s_tcq_laddr,
    input [REQUEST_LEN_BITS - 1:0]             s_tcq_length,
    input [MEM_TAG-1:0]                        s_tcq_tag,
    input                                      s_tcq_trailer,

    // Bus data move confirmation
    output reg                                 s_tcq_cvalid,
    input                                      s_tcq_cready,
    output reg [MEM_TAG-1:0]                   s_tcq_ctag,

    // USB EP-S
    input                                      m_axis_usbtx_tready,
    output reg [31:0]                          m_axis_usbtx_tdata,
    output reg                                 m_axis_usbtx_tlast,
    output reg                                 m_axis_usbtx_tvalid,
    output     [3:0]                           m_axis_usbtx_tkeep,

    // in-place USB notification
    input [127:0]                              s_usbtx_ntfy_data,
    input                                      s_usbtx_ntfy_valid,
    input                                      s_usbtx_ntfy_dual,
    output reg                                 s_usbtx_ntfy_ready,

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

assign m_axis_usbtx_tkeep = 4'b1111;

reg [2:0] state;
localparam [2:0]
    DMA_RAM_LOAD     = 0,
    DMA_USB_DATA_L   = 2,
    DMA_USB_DATA_H   = 3,
    DMA_USB_DATA_QL  = 4,
    DMA_USB_DATA_QH  = 5,
    DMA_USB_NTFY_W0  = 6,
    DMA_USB_NTFY_W1  = 7;

reg usb_ntfy_stage;
reg usb_ntfy;

reg   [REQUEST_LEN_BITS:0]  burst_counter;
wire                        burst_last            = burst_counter[REQUEST_LEN_BITS];
wire  [REQUEST_LEN_BITS:0]  burst_counter_nxt     = burst_counter - 1'b1;
wire                        burst_last_nxt        = burst_counter_nxt[REQUEST_LEN_BITS];
wire  [REQUEST_LEN_BITS:0]  burst_counter_req     = { 1'b0, s_tcq_length };
wire  [REQUEST_LEN_BITS:0]  burst_counter_req_nxt = burst_counter_req - 1'b1;

assign m_al_rready = (!m_axis_usbtx_tvalid || m_axis_usbtx_tready) && (state == ((DATA_BITS > 3) ? DMA_USB_DATA_QH : DMA_USB_DATA_H));

always @(posedge clk) begin
  if (rst) begin
    state                <= DMA_RAM_LOAD;

    s_tcq_ready          <= 1'b0;
    s_tcq_cvalid         <= 1'b0;

    m_al_arvalid         <= 1'b0;

    s_usbtx_ntfy_ready   <= 1'b0;

    m_axis_usbtx_tvalid  <= 1'b0;
  end else begin

    if (m_al_arvalid && m_al_arready) begin
        burst_counter        <= burst_counter_nxt;
        m_al_arvalid         <= !burst_last;
        m_al_araddr          <= m_al_araddr + 1'b1;
        m_al_arid            <= burst_last_nxt;
    end

    if (s_tcq_ready && s_tcq_valid) begin
      s_tcq_ready <= 1'b0;
    end

    if (s_tcq_cvalid && s_tcq_cready) begin
      s_tcq_cvalid <= 1'b0;
    end

    if (s_usbtx_ntfy_valid && s_usbtx_ntfy_ready) begin
      s_usbtx_ntfy_ready <= 1'b0;
    end

    if (m_axis_usbtx_tvalid && m_axis_usbtx_tready) begin
      m_axis_usbtx_tvalid <= 1'b0;
    end

    case (state)
     DMA_RAM_LOAD: begin
      if (s_tcq_valid && ~s_tcq_ready && (s_tcq_cready || ~s_tcq_cvalid) && ~s_usbtx_ntfy_ready /* && can_send_fc*/) begin
        m_al_arvalid         <= 1'b1;
        m_al_araddr          <= s_tcq_laddr;
        m_al_arid            <= burst_counter_req_nxt[REQUEST_LEN_BITS];
        burst_counter        <= burst_counter_req_nxt;
        s_tcq_ready          <= 1'b1;
        s_tcq_ctag           <= s_tcq_tag;
        usb_ntfy             <= s_tcq_trailer;
        usb_ntfy_stage       <= 1'b0;
        state                <= DMA_USB_DATA_L;
      end else if (~s_tcq_valid && (s_tcq_cready || ~s_tcq_cvalid) && ~s_usbtx_ntfy_ready && s_usbtx_ntfy_valid) begin
        // Burst finished but no data!
        usb_ntfy             <= 1'b1;
        usb_ntfy_stage       <= 1'b0;
        state                <= DMA_USB_NTFY_W0;
      end
     end

     DMA_USB_DATA_L, DMA_USB_DATA_H, DMA_USB_DATA_QL, DMA_USB_DATA_QH: begin
       if ((!m_axis_usbtx_tvalid || m_axis_usbtx_tready) && m_al_rvalid) begin
        state               <= state + 1'b1;
        m_axis_usbtx_tlast  <= 1'b0;
        m_axis_usbtx_tvalid <= 1'b1;
        m_axis_usbtx_tdata  <=
            ((DATA_BITS > 3) && state == DMA_USB_DATA_QH) ? m_al_rdata[127:96] :
            ((DATA_BITS > 3) && state == DMA_USB_DATA_QL) ? m_al_rdata[95:64] :
            (state == DMA_USB_DATA_H) ? m_al_rdata[63:32] : m_al_rdata[31:0];

        if (state == (DATA_BITS == 3 ? DMA_USB_DATA_H : DMA_USB_DATA_QH)) begin
          state <= DMA_USB_DATA_L;

          if (m_al_rid) begin
            m_axis_usbtx_tlast <= !usb_ntfy;
            s_tcq_cvalid       <= 1'b1;
            state              <= usb_ntfy ? DMA_USB_NTFY_W0 : DMA_RAM_LOAD;
          end
        end
      end
     end

     DMA_USB_NTFY_W0: begin
       if (~m_axis_usbtx_tvalid || m_axis_usbtx_tready) begin
        m_axis_usbtx_tvalid <= 1'b1;
        m_axis_usbtx_tdata  <= (DUAL_NTFY_EN && usb_ntfy_stage) ? s_usbtx_ntfy_data[95:64] : s_usbtx_ntfy_data[31:0];
        m_axis_usbtx_tlast  <= 1'b0;
        state               <= DMA_USB_NTFY_W1;
       end
     end

     DMA_USB_NTFY_W1: begin
       if (~m_axis_usbtx_tvalid || m_axis_usbtx_tready) begin
        m_axis_usbtx_tvalid <= 1'b1;
        m_axis_usbtx_tdata  <= (DUAL_NTFY_EN && usb_ntfy_stage)                       ? s_usbtx_ntfy_data[127:96] : s_usbtx_ntfy_data[63:32];
        m_axis_usbtx_tlast  <= (DUAL_NTFY_EN && s_usbtx_ntfy_dual && !usb_ntfy_stage) ? 1'b0            : 1'b1;
        state               <= (DUAL_NTFY_EN && s_usbtx_ntfy_dual && !usb_ntfy_stage) ? DMA_USB_NTFY_W0 : DMA_RAM_LOAD;
        s_usbtx_ntfy_ready  <= (DUAL_NTFY_EN && s_usbtx_ntfy_dual && !usb_ntfy_stage) ? 1'b0            : 1'b1;
        usb_ntfy_stage      <= 1'b1;
       end
     end

    endcase

  end
end

endmodule
