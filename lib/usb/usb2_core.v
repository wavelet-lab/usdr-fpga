module usb2_core #(
    parameter        AL_BUS_WIDTH   = 12,
    parameter [15:0] USDR_PID       = 0,
    parameter        ULTRA_SCALE    = 0,
    parameter        DATA_BITS      = 3,
    parameter        TX_RAM_ADDR_WIDTH = 17,
    parameter        _DATA_BYTES       = 1 << DATA_BITS,
    parameter        TX_TIMESTAMP_BITS = 48,
    parameter        TX_SAMPLES_WIDTH  = TX_RAM_ADDR_WIDTH - 1,
    parameter        TX_FE_DESCR_WIDTH = TX_TIMESTAMP_BITS + TX_SAMPLES_WIDTH + (TX_RAM_ADDR_WIDTH - DATA_BITS),
    parameter        NO_TX             = 0,
    parameter        RAM_CHECK_BIT     = 8
)(
    // ULPI interface
    input            phy_rst,
    input            phy_clk,

    output     [7:0] phy_do,
    input      [7:0] phy_di,
    output           phy_doe,

    input            phy_dir,
    input            phy_nxt,
    output           phy_stp,


    input            clk,
    input            rst,

    // AL interface
    output [AL_BUS_WIDTH-1:2]      m_al_waddr,
    output [31:0]                  m_al_wdata,
    output                         m_al_wvalid,
    input                          m_al_wready,

    output [AL_BUS_WIDTH-1:2]      m_al_araddr,
    output                         m_al_arvalid,
    input                          m_al_arready,

    input [31:0]                   m_al_rdata,
    input                          m_al_rvalid,
    output                         m_al_rready,


    // Notification/interrupr interface
    input [31:0]                   s_evd_data,
    input                          s_evd_last,
    input                          s_evd_valid,
    output                         s_evd_ready,

    // TX FIFO-RAM interface
    input                          tx_nactive,
    input                          fetx_mode_format,
    output [63:0]                  fetx_stat,

    input [TX_RAM_ADDR_WIDTH:RAM_CHECK_BIT] txfe_out_rd_addr,
    input [TX_TIMESTAMP_BITS-1:0]           txfe_out_rd_time,

    output                                  mem_tvalid,
    input                                   mem_tready,
    output  [TX_RAM_ADDR_WIDTH-1:DATA_BITS] mem_taddr,
    output  [_DATA_BYTES*8-1:0]             mem_tdata,
    output  [_DATA_BYTES-1:0]               mem_tkeep,

    output [TX_FE_DESCR_WIDTH-1:0]          m_usbs_burst_data,
    output                                  m_usbs_burst_valid,
    input                                   m_usbs_burst_ready,
    input                                   m_usbs_burst_busy,

    // RX FIFO-RAM interface
    output                                  s_axis_usbtx_tready,
    input [31:0]                            s_axis_usbtx_tdata,
    input                                   s_axis_usbtx_tlast,
    input                                   s_axis_usbtx_tvalid,
    input [3:0]                             s_axis_usbtx_tkeep,

    // AUX signal
    output          usb_bus_reset
);

`include "axi_helpers.vh"

localparam USB_TX_STREAMS = 3;
localparam USB_RX_STREAMS = 3;

// AXIS_M
wire [USB_RX_STREAMS - 1:0]     mn_axis_endpoint_rx_valid;
wire [USB_RX_STREAMS - 1:0]     mn_axis_endpoint_rx_ready;
wire [USB_RX_STREAMS - 1:0]     mn_axis_endpoint_rx_last;
wire [USB_RX_STREAMS * 32-1:0]  mn_axis_endpoint_rx_data;
wire [USB_RX_STREAMS * 4-1:0]   mn_axis_endpoint_rx_keep;
wire [USB_RX_STREAMS - 1:0]     mn_axis_endpoint_rx_stall;

// AXIS_S
wire [USB_TX_STREAMS - 1:0]     sn_axis_endpoint_tx_busy; //(busy flag -- do not try to push data)
wire [USB_TX_STREAMS - 1:0]     sn_axis_endpoint_tx_valid;
wire [USB_TX_STREAMS - 1:0]     sn_axis_endpoint_tx_ready;
wire                            s_axis_endpoint_tx_last;
wire [32-1:0]                   s_axis_endpoint_tx_data;
wire [4-1:0]                    s_axis_endpoint_tx_keep;

// TODO: STALL handling
assign mn_axis_endpoint_rx_stall[USB_RX_STREAMS - 1:0] = {USB_RX_STREAMS{1'b0}};


localparam USB_RX_CSR    = 0;
localparam USB_RX_NTFY   = 1;
localparam USB_RX_STREAM = 2;

localparam USB_TX_CSR    = 0;
localparam USB_TX_INV    = 1; // Invalid endpoint
localparam USB_TX_STREAM = 2;

assign sn_axis_endpoint_tx_busy[USB_TX_INV]  = 1'b1;
assign sn_axis_endpoint_tx_ready[USB_TX_INV] = 1'b1;

wire [31:0] csr_tx_data;
wire [3:0]  csr_tx_keep;
wire        csr_tx_last;
wire        csr_tx_valid;
wire        csr_tx_ready;
wire        fifo_empty;

assign sn_axis_endpoint_tx_busy[USB_TX_CSR] = !fifo_empty;

axis_fifo #(.WIDTH(32 + 1 + 4), .DEEP(64)) csr_fifo (
    .clk(clk),
    .rst(rst),
    
    .s_rx_tdata({ s_axis_endpoint_tx_data[USB_TX_CSR * 32 + 31:(USB_TX_CSR)*32], s_axis_endpoint_tx_keep[USB_TX_CSR * 4 + 3:(USB_TX_CSR)*4], s_axis_endpoint_tx_last  }),
    .s_rx_tvalid(sn_axis_endpoint_tx_valid[USB_TX_CSR]),
    .s_rx_tready(sn_axis_endpoint_tx_ready[USB_TX_CSR]),

    .m_tx_tdata({ csr_tx_data, csr_tx_keep, csr_tx_last }),
    .m_tx_tvalid(csr_tx_valid),
    .m_tx_tready(csr_tx_ready),

    .fifo_empty(fifo_empty)
);

usbeps_al_bridge #(.AL_BUS_WIDTH(AL_BUS_WIDTH)) usb_axis_bridge (
    .clk(clk),
    .rst(rst),

    .m_axis_endpoint_rx_valid(mn_axis_endpoint_rx_valid[USB_RX_CSR]),
    .m_axis_endpoint_rx_ready(mn_axis_endpoint_rx_ready[USB_RX_CSR]),
    .m_axis_endpoint_rx_last(mn_axis_endpoint_rx_last[USB_RX_CSR]),
    .m_axis_endpoint_rx_data(mn_axis_endpoint_rx_data[USB_RX_CSR * 32 + 31:(USB_RX_CSR)*32]),
    .m_axis_endpoint_rx_keep(mn_axis_endpoint_rx_keep[USB_RX_CSR * 4 + 3:(USB_RX_CSR)*4]),

    .s_axis_endpoint_tx_busy(),
    .s_axis_endpoint_tx_valid(csr_tx_valid),
    .s_axis_endpoint_tx_ready(csr_tx_ready),
    .s_axis_endpoint_tx_last(csr_tx_last),
    .s_axis_endpoint_tx_data(csr_tx_data),
    .s_axis_endpoint_tx_keep(csr_tx_keep),

    `AXIS_ALRDWR_CONNECT(m_al, m_al)
);

////////////////////////////////////////////////////////////////////////////////
// Notification
assign s_evd_ready                                                        = mn_axis_endpoint_rx_ready[USB_RX_NTFY];
assign mn_axis_endpoint_rx_valid[USB_RX_NTFY]                             = s_evd_valid;
assign mn_axis_endpoint_rx_last[USB_RX_NTFY]                              = s_evd_last;
assign mn_axis_endpoint_rx_data[USB_RX_NTFY * 32 + 31:(USB_RX_NTFY) * 32] = s_evd_data;
assign mn_axis_endpoint_rx_keep[USB_RX_NTFY * 4 + 3:(USB_RX_NTFY) * 4]    = 4'hf;


////////////////////////////////////////////////////////////////////////////////
// Stream RX
assign mn_axis_endpoint_rx_data[USB_RX_STREAM * 32 + 31:(USB_RX_STREAM) * 32]  = s_axis_usbtx_tdata;
assign mn_axis_endpoint_rx_last[USB_RX_STREAM]                                 = s_axis_usbtx_tlast;
assign mn_axis_endpoint_rx_valid[USB_RX_STREAM]                                = s_axis_usbtx_tvalid;
assign mn_axis_endpoint_rx_keep[USB_RX_STREAM * 4 + 3:(USB_RX_STREAM) * 4]     = s_axis_usbtx_tkeep;
assign s_axis_usbtx_tready                                                     = mn_axis_endpoint_rx_ready[USB_RX_STREAM];

////////////////////////////////////////////////////////////////////////////////
// Stream TX
generate
if (NO_TX) begin
    assign sn_axis_endpoint_tx_busy[USB_TX_STREAM] = 1'b1;
    assign sn_axis_endpoint_tx_ready[USB_TX_STREAM] = 1'b1;
    assign mem_tvalid         = 1'b0;
    assign mem_tkeep          = 0;
    assign mem_taddr          = 0;
    assign mem_tdata          = 0;
    assign m_usbs_burst_valid = 1'b0;
    assign m_usbs_burst_data  = 0;
    assign fetx_stat          = 0;
end else begin
usbeps_tx_deframer #(
    .TX_TIMESTAMP_BITS(TX_TIMESTAMP_BITS),
    .TX_RAM_ADDR_WIDTH(TX_RAM_ADDR_WIDTH),
    .TX_SAMPLES_WIDTH(TX_SAMPLES_WIDTH),
    .DATA_BITS(DATA_BITS),
    .RAM_CHECK_BIT(RAM_CHECK_BIT)
) usbeps_tx_deframer (
    .clk(clk),
    .rst(tx_nactive),

    .s_axis_endpoint_tx_busy(sn_axis_endpoint_tx_busy[USB_TX_STREAM]),
    .s_axis_endpoint_tx_valid(sn_axis_endpoint_tx_valid[USB_TX_STREAM]),
    .s_axis_endpoint_tx_ready(sn_axis_endpoint_tx_ready[USB_TX_STREAM]),
    .s_axis_endpoint_tx_last(s_axis_endpoint_tx_last /*[USB_TX_STREAM]*/),
    .s_axis_endpoint_tx_data(s_axis_endpoint_tx_data /* [USB_TX_STREAM * 32 + 31:(USB_TX_STREAM)*32] */ ),
    .s_axis_endpoint_tx_keep(s_axis_endpoint_tx_keep /* [USB_TX_STREAM * 4 + 3:(USB_TX_STREAM)*4] */ ),

    .mem_tvalid(mem_tvalid),
    .mem_tready(mem_tready),
    .mem_taddr(mem_taddr),
    .mem_tdata(mem_tdata),
    .mem_tkeep(mem_tkeep),

    .usbs_burst_data(m_usbs_burst_data),
    .usbs_burst_valid(m_usbs_burst_valid),
    .usbs_burst_ready(m_usbs_burst_ready),
    .usbs_burst_busy(m_usbs_burst_busy),

    .out_rd_addr(txfe_out_rd_addr),
    .out_rd_time(txfe_out_rd_time),

    .stat(fetx_stat),
    .fetx_mode_format(fetx_mode_format)
);
end
endgenerate

////////////////////////////////////////////////////////////////////////////////
// Streams to ULPI bridge

usb2_ulpi_usbeps  #(
    .AUX_BUS_TX(USB_TX_STREAMS),
    .AUX_BUS_RX(USB_RX_STREAMS),
    .USDR_PID(USDR_PID),
    .ULTRA_SCALE(ULTRA_SCALE)
) usb2_ulpi_usbeps (
    //ULPI ports
    .phy_rst(phy_rst),
    .phy_clk(phy_clk),

    .phy_do(phy_do),
    .phy_di(phy_di),
    .phy_doe(phy_doe),

    .phy_dir(phy_dir),
    .phy_nxt(phy_nxt),
    .phy_stp(phy_stp),

    .cfg_usb2_en(1'b1),

    .usb2_stat(),
    .usb2_stat2(),
    .usb_bus_reset(usb_bus_reset),

    .clk(clk),
    .rst(rst),

    .sn_axis_endpoint_rx_valid(mn_axis_endpoint_rx_valid),
    .sn_axis_endpoint_rx_ready(mn_axis_endpoint_rx_ready),
    .sn_axis_endpoint_rx_last(mn_axis_endpoint_rx_last),
    .sn_axis_endpoint_rx_data(mn_axis_endpoint_rx_data),
    .sn_axis_endpoint_rx_keep(mn_axis_endpoint_rx_keep),
    .sn_axis_endpoint_rx_stall(mn_axis_endpoint_rx_stall),

    .mn_axis_endpoint_tx_busy(sn_axis_endpoint_tx_busy),
    .mn_axis_endpoint_tx_valid(sn_axis_endpoint_tx_valid),
    .mn_axis_endpoint_tx_ready(sn_axis_endpoint_tx_ready),
    .m_axis_endpoint_tx_last(s_axis_endpoint_tx_last),
    .m_axis_endpoint_tx_data(s_axis_endpoint_tx_data),
    .m_axis_endpoint_tx_keep(s_axis_endpoint_tx_keep)

);

endmodule
