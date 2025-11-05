module usdr_app_generic_us #(
    parameter I2C_SPEED      = 400_000,
    parameter I2C_CLOCK_STRETCHING = 1'b1,
    parameter HUL_BUS_SPEED  = 125_000_000, // hclk / hrst
    parameter PUL_BUS_SPEED  = 125_000_000, // pclk / prst
    parameter SPI_WIDTH      = 32'h00_18_18_18,
    parameter SPI_DIV        = 32'h80_80_80_80,
    parameter SPI_COUNT      = 4,
    parameter I2C_COUNT      = 1,
    parameter SPI_EXT_PRESENT = 0,
    parameter IGPI_COUNT     = 1,
    parameter IGPO_COUNT     = 1,
    parameter IGPO_INIT      = 0,
    parameter CFG_PHY_PORTS  = 2,
    parameter NO_TX          = 1,
    parameter C_DATA_WIDTH   = 128,
    parameter AXI4_CQ_TUSER_WIDTH = 88,
    parameter AXI4_CC_TUSER_WIDTH = 33,
    parameter AXI4_RQ_TUSER_WIDTH = 62,
    parameter AXI4_RC_TUSER_WIDTH = 75,
    parameter KEEP_WIDTH          = C_DATA_WIDTH / 32,
    parameter DWISBE              = 1,
    parameter EGPIO_PRESENT       = 0,
    parameter EGPIO_WIDTH         = 15,
    parameter EGPIO_ALT_MODE1     = 0,
    parameter UART_PRESENT        = 0,
    parameter ULTRA_SCALE         = 1,
    parameter USB2_PRESENT        = 0,
    parameter USDR_PID            = 0,
    parameter TX_FRAME_LENGTH     = 32,
    parameter TX_INITIAL_TS_COMP  = TX_FRAME_LENGTH,
    parameter USE_EXT_ADC_CLK     = 0,
    parameter USE_EXT_DAC_CLK     = 0,
    parameter _I2C_PORT_COUNT     = (I2C_COUNT > 1) ? 4 : 2,
    parameter COLLAPSE_RQ_EVENT   = 1'b1,
    parameter COLLAPSE_CC_REPLY   = 1'b1,
    parameter ADC_CHANS           = 4,
    parameter ADC_WIDTH           = 16,
    parameter DAC_CHANS           = 4,
    parameter DAC_WIDTH           = 16,
    parameter ADC_FLAG_WIDTH      = 1,
    parameter TX_SUPPORT_8CH      = 1'b0,
    parameter TX_SUPPORT_4CH      = 1'b0,
    parameter [0:0] TX_EX_CORE    = 1'b0,
    parameter WIDE_DMA_TO_FE      = 1'b0
)(
    // high speed data interfaces
    input          hrst,
    input          hclk,

    output                                     s_axis_rq_tlast,
    output              [C_DATA_WIDTH-1:0]     s_axis_rq_tdata,
    output       [AXI4_RQ_TUSER_WIDTH-1:0]     s_axis_rq_tuser,
    output                [KEEP_WIDTH-1:0]     s_axis_rq_tkeep,
    input                                      s_axis_rq_tready,
    output                                     s_axis_rq_tvalid,

    input               [C_DATA_WIDTH-1:0]     m_axis_rc_tdata,
    input        [AXI4_RC_TUSER_WIDTH-1:0]     m_axis_rc_tuser,
    input                                      m_axis_rc_tlast,
    input                 [KEEP_WIDTH-1:0]     m_axis_rc_tkeep,
    input                                      m_axis_rc_tvalid,
    output                                     m_axis_rc_tready,

    input               [C_DATA_WIDTH-1:0]     m_axis_cq_tdata,
    input        [AXI4_CQ_TUSER_WIDTH-1:0]     m_axis_cq_tuser,
    input                                      m_axis_cq_tlast,
    input                 [KEEP_WIDTH-1:0]     m_axis_cq_tkeep,
    input                                      m_axis_cq_tvalid,
    output                                     m_axis_cq_tready,

    output              [C_DATA_WIDTH-1:0]     s_axis_cc_tdata,
    output       [AXI4_CC_TUSER_WIDTH-1:0]     s_axis_cc_tuser,
    output                                     s_axis_cc_tlast,
    output                [KEEP_WIDTH-1:0]     s_axis_cc_tkeep,
    output                                     s_axis_cc_tvalid,
    input                                      s_axis_cc_tready,


    input [5:0]    pcie7s_tx_buf_av,

    // US transmit ordering control
    input [5:0]    pcieus_rq_seq_num0,
    input          pcieus_rq_seq_num_vld0,
    input [5:0]    pcieus_rq_seq_num1,
    input          pcieus_rq_seq_num_vld1,

    input [7:0]    cfg_fc_ph,
    input [11:0]   cfg_fc_pd,

    // pci configuration
    input [15:0]   cfg_completer_id,
    input [2:0]    cfg_max_payload_size,
    input [2:0]    cfg_max_read_req_size,

    // pci interrupts (MSI only)
    input          cfg_interrupt_msienable,
    input          cfg_interrupt_rdy,
    output         cfg_interrupt,
    output [7:0]   cfg_interrupt_di,

    input [2:0]    cfg_interrupt_mmenable,
    output [4:0]   cfg_pciecap_interrupt_msgnum,

    // streaming ADC
    input                               adc_clk_ext, // External ADC clock when USE_EXT_ADC_CLK is activated
    input                               adc_rst_ext,
    input [ADC_CHANS * ADC_WIDTH - 1:0] adc_realigned,
    input                               adc_fifo_valid,
    input [ADC_FLAG_WIDTH - 1:0]        adc_flags,

    output                              rx_ready,
    output                              rx_streaming,
    output [ADC_CHANS-1:0]              adc_ch_enable,
    output                              adc_enable,

    // streaming DAC
    input                                dac_clk_ext,
    input                                dac_rst_ext,
    output [DAC_CHANS * DAC_WIDTH - 1:0] dac_realigned,
    output                               dac_fifo_valid,
    output                               dac_frame,
    input                                tx_ready,
    output                               tx_streaming,

    // USB2 ULPI interface
    input          phy_rst,
    input          phy_clk,
    output   [7:0] phy_do,
    input    [7:0] phy_di,
    output         phy_doe,
    input          phy_dir,
    input          phy_nxt,
    output         phy_stp,
    input          usb_bus_active,
    output         usb_bus_reset,

    // peripheral interfaces
    input          prst,
    input          pclk,

    input  [_I2C_PORT_COUNT-1:0]   sda_in,
    input  [_I2C_PORT_COUNT-1:0]   scl_in,
    output [_I2C_PORT_COUNT-1:0]   sda_out_eo,
    output [_I2C_PORT_COUNT-1:0]   scl_out_eo,

    output [3:0]   spi_mosi,
    input  [3:0]   spi_miso,
    output [3:0]   spi_sclk,
    output [3:0]   spi_sen,

    output         spi_ext_mosi,
    input          spi_ext_miso,
    output         spi_ext_sclk,
    output [7:0]   spi_ext_sen,

    output [3:0]   flash_dout,
    input  [3:0]   flash_din,
    output [3:0]   flash_ndrive,
    output         flash_ncs,
    output         flash_sck,

    input  [0:0]   uart_rxd,
    output [0:0]   uart_txd,

    // Internal GPI & GPO
    output reg [IGPO_COUNT * 8 - 1:0]  igpo_data,
    output reg [IGPO_COUNT - 1:0]      igpo_strobe,
    input [IGPI_COUNT * 8 - 1:0]       igpi_data,

    // Extrenal board GPIO with alternative functions
    output [EGPIO_WIDTH-1:0]           egpio_data_out,
    output [EGPIO_WIDTH-1:0]           egpio_data_oe,
    input  [EGPIO_WIDTH-1:0]           egpio_data_in,
    output [EGPIO_WIDTH-1:0]           egpio_altf0_active,
    output [EGPIO_WIDTH-1:0]           egpio_altf1_active,
    output [EGPIO_WIDTH-1:0]           egpio_alt0_in,
    input  [EGPIO_WIDTH-1:0]           egpio_alt0_out,
    input  [EGPIO_WIDTH-1:0]           egpio_alt0_out_oe,
    output [EGPIO_WIDTH-1:0]           egpio_alt1_in,
    input  [EGPIO_WIDTH-1:0]           egpio_alt1_out,
    input  [EGPIO_WIDTH-1:0]           egpio_alt1_out_oe,
    
    // DATA PHY configuration
    output                             cfg_port_clk,
    output                             cfg_port_rst,
    output [CFG_PHY_PORTS - 1:0]       cfg_port_wvalid,
    input  [CFG_PHY_PORTS - 1:0]       cfg_port_wready,
    output [31:0]                      cfg_port_wdata,
    input  [CFG_PHY_PORTS - 1:0]       cfg_port_rvalid,
    output [CFG_PHY_PORTS - 1:0]       cfg_port_rready,
    input  [32 * CFG_PHY_PORTS - 1:0]  cfg_port_rdata,

    // Sync
    input                              pps_on_in,
    output                             sysref_gen
);

localparam DATA_BITS = (C_DATA_WIDTH == 256) ? 5 : (C_DATA_WIDTH == 128) ? 4 : 3;
localparam FEDATA_TX_WIDTH = DAC_CHANS * 16;
localparam FEDATA_TX_BITS  = $clog2(FEDATA_TX_WIDTH / 8);

localparam BUCKET_WIDTH = (C_DATA_WIDTH == 256) ? 128 : C_DATA_WIDTH;
localparam BUCKET_BITS  = $clog2(BUCKET_WIDTH / 8);
localparam BUCKET_KEEP  = BUCKET_WIDTH / 32;

wire tran_usb_active = USB2_PRESENT && usb_bus_active;

// if hclk != pclk ADC automatically switches to ASYNC mode
wire adc_rst = (USE_EXT_ADC_CLK) ? adc_rst_ext : hrst;
wire adc_clk = (USE_EXT_ADC_CLK) ? adc_clk_ext : hclk;
localparam ADC_ASYNC_CLK = (HUL_BUS_SPEED != PUL_BUS_SPEED) || USE_EXT_ADC_CLK;

// pclk is default clock for FE ?  todo the same as ADC
wire dac_clk    = (USE_EXT_DAC_CLK) ? dac_clk_ext : hclk;
wire dac_rst_fe = (USE_EXT_DAC_CLK) ? dac_rst_ext : hclk;
localparam DAC_ASYNC_CLK = (HUL_BUS_SPEED != PUL_BUS_SPEED) || USE_EXT_DAC_CLK;


/////////////////////////////////////
// Register map

`include "usdr_app_generic_const.vh"
`include "axi_helpers.vh"

wire rx_streamingready;
wire rx_dmaactive;

genvar i;

localparam IGP_ADDR_BITS =
    (IGPO_COUNT <= 2) ? 1 :
    (IGPO_COUNT <= 4) ? 2 :
    (IGPO_COUNT <= 8) ? 3 :
    (IGPO_COUNT <= 16) ? 4 :
    (IGPO_COUNT <= 32) ? 5 :
    (IGPO_COUNT <= 64) ? 6 :
    (IGPO_COUNT <= 128) ? 7 :
    (IGPO_COUNT <= 256) ? 8 : -1;


///////////////////////////////////////
// RQ-interface
`DEFINE_AXIS_RVDLKU_PORT(hclk_axis_rq_rxdma_t, C_DATA_WIDTH, KEEP_WIDTH, AXI4_RQ_TUSER_WIDTH);     // MemWr
`DEFINE_AXIS_RVDLKU_PORT(hclk_axis_rq_txdma_t, C_DATA_WIDTH, KEEP_WIDTH, AXI4_RQ_TUSER_WIDTH);     // MemRd posted

`DEFINE_AXIS_RVDLKU_PORT(pclk_axis_rq_event_t,      BUCKET_WIDTH, BUCKET_KEEP, AXI4_RQ_TUSER_WIDTH);     // MemWr
`DEFINE_AXIS_RVDLKU_PORT(hclk_axis_rq_event_t,      BUCKET_WIDTH, BUCKET_KEEP, AXI4_RQ_TUSER_WIDTH);     // MemWr
`DEFINE_AXIS_RVDLKU_PORT(hclk_axis_rq_event_exp_t , BUCKET_WIDTH, BUCKET_KEEP, AXI4_RQ_TUSER_WIDTH);     // MemWr
`DEFINE_AXIS_RVDLKU_PORT(hclk_axis_rq_event_comb_t, C_DATA_WIDTH, KEEP_WIDTH,  AXI4_RQ_TUSER_WIDTH);     // MemWr

axis_opt_cc_fifo #(
  .WIDTH(BUCKET_WIDTH + BUCKET_KEEP + 1),
  .CLK_CC(PUL_BUS_SPEED != HUL_BUS_SPEED)
) axis_rq_event_p_to_h (
  .rx_clk(pclk),
  .rx_rst(prst),

  .s_rx_tdata({pclk_axis_rq_event_tlast, pclk_axis_rq_event_tkeep, pclk_axis_rq_event_tdata}),
  .s_rx_tvalid(pclk_axis_rq_event_tvalid),
  .s_rx_tready(pclk_axis_rq_event_tready),

  .tx_clk(hclk),
  .tx_rst(hrst),

  .m_tx_tdata({hclk_axis_rq_event_tlast, hclk_axis_rq_event_tkeep, hclk_axis_rq_event_tdata}),
  .m_tx_tvalid(hclk_axis_rq_event_tvalid),
  .m_tx_tready(hclk_axis_rq_event_tready)
);

axis_fifo_pkt #(
  .WIDTH(BUCKET_WIDTH + BUCKET_KEEP),
  .PASSTHROUGH(!COLLAPSE_RQ_EVENT),
  .DEEP(16)
) axis_rq_event_comb (
  .clk(hclk),
  .rst(hrst),

  .s_rx_tdata({hclk_axis_rq_event_tkeep, hclk_axis_rq_event_tdata}),
  .s_rx_tvalid(hclk_axis_rq_event_tvalid),
  .s_rx_tready(hclk_axis_rq_event_tready),
  .s_rx_tlast(hclk_axis_rq_event_tlast),

  .m_tx_tdata({hclk_axis_rq_event_exp_tkeep, hclk_axis_rq_event_exp_tdata}),
  .m_tx_tvalid(hclk_axis_rq_event_exp_tvalid),
  .m_tx_tready(hclk_axis_rq_event_exp_tready),
  .m_tx_tlast(hclk_axis_rq_event_exp_tlast)
);

// Fixup for natural aligment for 256 bit mode!
assign hclk_axis_rq_event_exp_tready  = hclk_axis_rq_event_comb_tready;
assign hclk_axis_rq_event_comb_tvalid = hclk_axis_rq_event_exp_tvalid;
assign hclk_axis_rq_event_comb_tlast  = hclk_axis_rq_event_exp_tlast;

`ifdef FULLY_COMPATIBLE_256
wire a_sw;
assign hclk_axis_rq_event_comb_tdata  = (C_DATA_WIDTH == 256) ? { hclk_axis_rq_event_exp_tdata, hclk_axis_rq_event_exp_tdata } : hclk_axis_rq_event_exp_tdata;
assign hclk_axis_rq_event_comb_tkeep  = (C_DATA_WIDTH == 256) ? ((hclk_axis_rq_event_exp_tlast && !a_sw) ? 8'h0f : 8'hff)      : hclk_axis_rq_event_exp_tkeep;
assign hclk_axis_rq_event_comb_tuser  = (C_DATA_WIDTH == 256) ? ({1'b0, hclk_axis_rq_event_exp_tdata[4], 10'h0ff }) : 12'h0ff; // Sampled at SOP

generate
if (C_DATA_WIDTH == 256) begin
    reg a_sw_r;
    always @(posedge hclk) begin
        if (hclk_axis_rq_event_exp_tvalid && hclk_axis_rq_event_exp_tready && !hclk_axis_rq_event_exp_tlast) begin
            a_sw_r <= hclk_axis_rq_event_exp_tdata[4]; //Addr[4]
        end
    end
    assign a_sw = a_sw_r;
end
endgenerate
`else
assign hclk_axis_rq_event_comb_tdata  = hclk_axis_rq_event_exp_tdata;
assign hclk_axis_rq_event_comb_tkeep  = (C_DATA_WIDTH == 256) ? (hclk_axis_rq_event_exp_tlast ? 8'h0f : 8'hff) : hclk_axis_rq_event_exp_tkeep;
assign hclk_axis_rq_event_comb_tuser  = 12'h0ff;
`endif


axis_mux #(.DATA_WIDTH(C_DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .PORTS(3), .USER_WIDTH(12)) axis_mux(
    .clk(hclk),
    .rst(hrst),

    `AXIS_RVDLK_PORT_CONN(m_axis_t, s_axis_rq_t),
    .m_axis_tuser(s_axis_rq_tuser[11:0]),

    .sn_axis_tready({ hclk_axis_rq_rxdma_tready, hclk_axis_rq_event_comb_tready, hclk_axis_rq_txdma_tready}),
    .sn_axis_tdata( { hclk_axis_rq_rxdma_tdata,  hclk_axis_rq_event_comb_tdata,  hclk_axis_rq_txdma_tdata}),
    .sn_axis_tkeep( { hclk_axis_rq_rxdma_tkeep,  hclk_axis_rq_event_comb_tkeep,  hclk_axis_rq_txdma_tkeep}),
    .sn_axis_tlast( { hclk_axis_rq_rxdma_tlast,  hclk_axis_rq_event_comb_tlast,  hclk_axis_rq_txdma_tlast}),
    .sn_axis_tvalid({ hclk_axis_rq_rxdma_tvalid, hclk_axis_rq_event_comb_tvalid, hclk_axis_rq_txdma_tvalid}),
    .sn_axis_tuser( { 12'h0ff,                   hclk_axis_rq_event_comb_tuser[11:0],  12'h0ff})
);


`ifdef SEQNUM_RQ
// in US
reg [6:0] axis_rq_tag;
reg [6:0] processed_tag;
reg       s_axis_rq_tsop;
reg       s_in_packet;
reg       s_rq_err;

always @(posedge hclk) begin
    if (hrst) begin
        axis_rq_tag   <= 0;
        processed_tag <= ~0;

        s_axis_rq_tsop <= 1'b1;
        s_in_packet <= 0;

        s_rq_err    <= 0;
    end else begin

        if (s_axis_rq_tvalid && s_axis_rq_tready && s_axis_rq_tsop) begin
            axis_rq_tag <= axis_rq_tag + 1'b1;
        end

        if (s_axis_rq_tvalid && s_axis_rq_tready) begin
            s_axis_rq_tsop <= s_axis_rq_tlast;
            s_in_packet    <= !s_axis_rq_tlast;
            if (s_axis_rq_tlast) begin
                s_rq_err    <= 1'b0;
            end
        end

        if (s_in_packet && !s_axis_rq_tvalid) begin
            s_rq_err <= 1'b1;
        end
    end

    if (pcieus_rq_seq_num_vld0) begin
        processed_tag[5:0] <= pcieus_rq_seq_num0;

        if (processed_tag[5] == 1'b1 && pcieus_rq_seq_num0[5] == 1'b0) begin
            processed_tag[6] <= ~processed_tag[6];
        end
    end
end
//    input [7:0]    cfg_fc_ph,
//    input [11:0]   cfg_fc_pd,
wire [6:0] axis_rq_tag_diff    = 1'b1 + processed_tag - axis_rq_tag;
wire       us_rq_busy          = axis_rq_tag_diff[6]; // || (cfg_fc_ph < 4) || (cfg_fc_pd < 32);

// The user application must communicate the offset of the first Dword of the payload on the datapath
// using the addr_offset[2:0] signal in s_axis_rq_tuser[10:8].
assign s_axis_rq_tuser[23:12] = 0; //12'h0ff;
assign s_axis_rq_tuser[27:24] = axis_rq_tag[3:0];
assign s_axis_rq_tuser[59:28] = 0;
assign s_axis_rq_tuser[61:60] = axis_rq_tag[5:4];
`else
wire       us_rq_busy         = 0;
assign s_axis_rq_tuser[61:12] = 0; 
`endif


localparam AL_BUS_WIDTH = 12;

/////////////////////////////////////
// PCIe master peripheral lowspeed bus (100/125 Mhz)
`DEFINE_ALRDWR_AXIS(pul, AL_BUS_WIDTH - 2, 32); // PCIe master
`DEFINE_ALRDWR_AXIS(uul, AL_BUS_WIDTH - 2, 32); // USB2 master
`DEFINE_ALRDWR_AXIS(cul, AL_BUS_WIDTH - 2, 32); // combined (pcie/usb2) master

`DEFINE_AXIS_RVDLKU_PORT(p_axis_tx_al_t, C_DATA_WIDTH, KEEP_WIDTH, AXI4_CQ_TUSER_WIDTH);
`DEFINE_AXIS_RVDLKU_PORT(p_axis_tx_al_comb_t, C_DATA_WIDTH, KEEP_WIDTH, AXI4_CQ_TUSER_WIDTH);
`DEFINE_AXIS_RVDLKU_PORT(p_axis_rx_al_t, C_DATA_WIDTH, KEEP_WIDTH, AXI4_CC_TUSER_WIDTH);

axis_opt_cc_fifo #(
  .WIDTH(C_DATA_WIDTH + KEEP_WIDTH + 1 + 4),
  .CLK_CC(PUL_BUS_SPEED != HUL_BUS_SPEED)
) axis_cq_h_to_p_al (
  .rx_clk(hclk),
  .rx_rst(hrst),

  .s_rx_tdata({m_axis_cq_tuser[3:0], m_axis_cq_tlast, m_axis_cq_tkeep[KEEP_WIDTH - 1:0], m_axis_cq_tdata }),
  .s_rx_tvalid(m_axis_cq_tvalid),
  .s_rx_tready(m_axis_cq_tready),

  .tx_clk(pclk),
  .tx_rst(prst),

  .m_tx_tdata({p_axis_rx_al_tuser[3:0], p_axis_rx_al_tlast, p_axis_rx_al_tkeep[KEEP_WIDTH - 1:0], p_axis_rx_al_tdata }),
  .m_tx_tvalid(p_axis_rx_al_tvalid),
  .m_tx_tready(p_axis_rx_al_tready)
);

// TODO: combine fifo_pkt to cc_fifo_pkt
axis_fifo_pkt #(
  .WIDTH(C_DATA_WIDTH + KEEP_WIDTH),
  .PASSTHROUGH(!COLLAPSE_CC_REPLY),
  .DEEP(16)
) axis_cc_event_p_comb (
  .clk(pclk),
  .rst(prst),

  .s_rx_tdata({p_axis_tx_al_tkeep, p_axis_tx_al_tdata}),
  .s_rx_tvalid(p_axis_tx_al_tvalid),
  .s_rx_tready(p_axis_tx_al_tready),
  .s_rx_tlast(p_axis_tx_al_tlast),

  .m_tx_tdata({p_axis_tx_al_comb_tkeep, p_axis_tx_al_comb_tdata}),
  .m_tx_tvalid(p_axis_tx_al_comb_tvalid),
  .m_tx_tready(p_axis_tx_al_comb_tready),
  .m_tx_tlast(p_axis_tx_al_comb_tlast)
);


axis_opt_cc_fifo #(
  .WIDTH(C_DATA_WIDTH + KEEP_WIDTH + 1),
  .CLK_CC(PUL_BUS_SPEED != HUL_BUS_SPEED)
) axis_cc_event_p_to_h_al (
  .rx_clk(pclk),
  .rx_rst(prst),

  .s_rx_tdata({p_axis_tx_al_comb_tlast, p_axis_tx_al_comb_tkeep[KEEP_WIDTH - 1:0], p_axis_tx_al_comb_tdata}),
  .s_rx_tvalid(p_axis_tx_al_comb_tvalid),
  .s_rx_tready(p_axis_tx_al_comb_tready),

  .tx_clk(hclk),
  .tx_rst(hrst),

  .m_tx_tdata({s_axis_cc_tlast, s_axis_cc_tkeep[KEEP_WIDTH-1:0], s_axis_cc_tdata}),
  .m_tx_tvalid(s_axis_cc_tvalid),
  .m_tx_tready(s_axis_cc_tready)
);
assign s_axis_cc_tuser = 0;

axis_pcie_to_al_us #(.BUS_WIDTH(C_DATA_WIDTH), .ULTRA_SCALE(ULTRA_SCALE), .ADDR_WIDTH(AL_BUS_WIDTH), .DWISBE(DWISBE)) axis_pcie_to_al_us (
    .clk(pclk),
    .rst_n(~prst),

    // Configuration
    .cfg_completer_id(cfg_completer_id),

    // AXIS PCIe RX & TX
    `AXIS_RVDLKU_PORT_CONN(s_axis_rx_t, p_axis_rx_al_t),
    `AXIS_RVDLKU_PORT_CONN(m_axis_tx_t, p_axis_tx_al_t),

    // AL Write / Readback channels
    `AXIS_ALRDWR_CONNECT(m_al, pul)
);

alwr_mux #(.ADDR_WIDTH(AL_BUS_WIDTH), .SLAVE_COUNT(USB2_PRESENT ? 2 : 1)) alwr_pul_uul_mux (
    .clk(pclk),
    .rst(prst),

    `AXIS_ALWR_VCONNECT_2(sn_al, uul, pul),
    `AXIS_ALWR_CONNECT(m_al, cul)
);

assign cul_araddr  = (tran_usb_active) ? uul_araddr  : pul_araddr;
assign cul_arvalid = (tran_usb_active) ? uul_arvalid : pul_arvalid;
assign pul_arready = !tran_usb_active  && cul_arready;
assign uul_arready = tran_usb_active   && cul_arready;

assign uul_rdata   = cul_rdata;
assign pul_rdata   = cul_rdata;
assign pul_rvalid  = !tran_usb_active  && cul_rvalid;
assign uul_rvalid  = tran_usb_active   && cul_rvalid;
assign cul_rready  = (tran_usb_active) ? uul_rready : pul_rready;

`DEFINE_ALRDWR_AXIS(csr_ul, AL_BUS_WIDTH - 2, 32);  // Readback registers
`DEFINE_ALRDWR_AXIS(mem_al, AL_BUS_WIDTH - 2, 32);  // Storage RAM for QSPI operations

localparam [2 * 32 - 1:0] AL_ADDR_MASK = { 32'h0800, 32'h0800 };
localparam [2 * 32 - 1:0] AL_ADDR_COMP = { 32'h0800, 32'h0000 };

alrdwr_demux_addr #(.ADDR_WIDTH(AL_BUS_WIDTH), .ID_WIDTH(1), .MASTER_COUNT(2), .ADDR_MASK(AL_ADDR_MASK), .ADDR_COMP(AL_ADDR_COMP)) bus_demux (
    // AL clocks
    .clk(pclk),
    .rst(prst),

    `AXIS_ALRDWR_CONNECT(s_al, cul),
    `AXIS_ALRDWR_VCONNECT_2(mn_al, mem_al, csr_ul)
);

////////////////////////////////////////////////////////////////////////////////

`define DEFINE_CSR32_PORT(name) `DEFINE_AXIS_RVD_PORT(axis_``name``_, 32)

`define DEFINE_CSR32_WR_PORT(name) `DEFINE_CSR32_PORT(wr_``name)
`define ASSIGN_CSR32_WR(addr, name) \
    assign axis_wr_``name``_data = axis_csrwr_data; \
    assign axis_wr_``name``_valid = axis_csrwr_valid[addr]; \
    assign axis_csrwr_ready[addr] = axis_wr_``name``_ready

`define CSR32_WR(addr, name) \
    `DEFINE_CSR32_WR_PORT(name); \
    `ASSIGN_CSR32_WR(addr, name)

`define DEFINE_CSR32_RD_PORT(name) `DEFINE_CSR32_PORT(rd_``name)
`define ASSIGN_CSR32_RD(addr, name) \
    assign axis_rd_``name``_ready = axis_csrrd_ready[addr]; \
    assign axis_csrrd_data[32*(addr + 1)-1:32*(addr)] = axis_rd_``name``_data; \
    assign axis_csrrd_valid[addr] = axis_rd_``name``_valid

`define CSR32_RD(addr, name) \
    `DEFINE_CSR32_RD_PORT(name); \
    `ASSIGN_CSR32_RD(addr, name)

`define CSR32_RDWR(addr, name) \
    `CSR32_RD(addr, name); \
    `CSR32_WR(addr, name)

`define CSR32_RD_CONST(addr, value) \
    `CSR32_RD(addr, addr); \
    assign axis_rd_``addr``_data = value; \
    assign axis_rd_``addr``_valid = 1'b1

`define CSR32_RD_NULL(addr) `CSR32_RD_CONST(addr, 32'h0000_0000)

`define CSR32_WR_NULL(addr) \
    `CSR32_WR(addr, addr); \
    assign axis_wr_``addr``_ready = 1'b1

`define CSR32_RDWR_NULL(addr) \
    `CSR32_RD_NULL(addr); \
    `CSR32_WR_NULL(addr)

`define DEFINE_CSR32_RDWR_PORT(name) \
    `DEFINE_CSR32_RD_PORT(name); \
    `DEFINE_CSR32_WR_PORT(name)

`define ASSIGN_CSR32_RDWR(addr, name) \
    `ASSIGN_CSR32_RD(addr, name); \
    `ASSIGN_CSR32_WR(addr, name)

//////////////////////////////////////////////////////////////////////////////////////////////

// Write ports
localparam USER_CSR_COUNT = 64;
localparam USER_CSR_BITS = $clog2(USER_CSR_COUNT);

localparam UCSR_WR_SIZE = USER_CSR_COUNT;
wire [31:0]              axis_csrwr_data;
wire [UCSR_WR_SIZE-1:0]  axis_csrwr_valid;
wire [UCSR_WR_SIZE-1:0]  axis_csrwr_ready;

alwr_demux_axis #(
    .ADDR_TOTAL(UCSR_WR_SIZE)
) alwr_demux_axis(
    `AXIS_ALWR_CONNECT(s_al, csr_ul),
    `AXIS_RVD_PORT_CONN(mn_axis_, axis_csrwr_)
);

// Read ports
localparam UCSR_RD_SIZE = USER_CSR_COUNT;
wire [32*UCSR_RD_SIZE - 1:0] axis_csrrd_data;
wire [UCSR_RD_SIZE - 1:0]    axis_csrrd_valid;
wire [UCSR_RD_SIZE - 1:0]    axis_csrrd_ready;


`DEFINE_ALRD_AXIS(egrb, AL_BUS_WIDTH - 2, 32);
`DEFINE_ALRDI_AXIS(master_rb, AL_BUS_WIDTH - 2, 32, 1);

alrd_mux #(.ADDR_WIDTH(AL_BUS_WIDTH), .SLAVE_COUNT(2)) bus_mux (
    // AL clocks
    .clk(pclk),
    .rst(prst),

    `AXIS_ALRD_VCONNECT_2(sn_al, egrb, csr_ul),
    `AXIS_ALRDI_CONNECT(m_al,  master_rb)
);

alrd_mux_axis #(
    .ADDR_TOTAL(UCSR_RD_SIZE),
    .PIPELINE_ADDR(0),
    .PIPELINE_DATA(0),
    .ID_WIDTH(1)
) readback (
    .clk(pclk),
    .rst(prst),
    `AXIS_ALRDI_CONNECT(s_al, master_rb),
    `AXIS_RVD_PORT_CONN(sn_readback_, axis_csrrd_)
);


//////////////////////////////////////////////////////////////////////////////////////////////
// CSR defines

// RD16 -- 28 .. GPI readback
`STATIC_ASSERT(IGPI_COUNT <= 12 * 4, igpi_count)
wire [12 * 32 - 1:0] igpi_data_ext = igpi_data;

// CSRs [0:15]
`CSR32_WR(REG_WR_CTRL, gpo); `CSR32_RD(REG_RD_IGPI_0, gpi);
`CSR32_RDWR(REG_I2C,  i2c);
`CSR32_RDWR(REG_SPI0, spi0);
`CSR32_RDWR(REG_GPIO_CTRL, egpio);

`CSR32_WR(REG_WR_RXDMA_CNF0, rxdma_confirm);
`CSR32_WR(REG_WR_RXDMA_CTRL0, rxdma_controlcomb);
`CSR32_WR(REG_WR_MBUS2_ADDR, mbus2_addr);
`CSR32_WR(REG_WR_MBUS2_DATA, mbus2_data);
`CSR32_RD(REG_RD_RXDMA_BST0, rxdma_buffs);
`CSR32_RD(REG_RD_RXDMA_BST1, rxdma_bursts);
`CSR32_RD(REG_RD_RXDMA_STAT, rxdma_stat);
`CSR32_RD(REG_RD_RXFE_PWR0,  rxfe0_pwr0);

`CSR32_WR(REG_WR_PNTFY_CFG,     pntfy_cfg);
`CSR32_WR(REG_WR_PNTFY_ACK,     pntfy_ack);
`CSR32_WR(REG_WR_FLASHSPI_CMD,  flashspi_cmd);
`CSR32_WR(REG_WR_FLASHSPI_ADDR, flashspi_addr);
`CSR32_RD(REG_RD_RXFE_PWR1,     rxfe0_pwr1);
`CSR32_RD(REG_RD_TIMEEVENT,     rxfe0_timeevent);
`CSR32_RD(REG_RD_FLASHSPI_STAT, flashspi_stat);
`CSR32_RD(REG_RD_FLASHSPI_DATA, flashspi_rb);

`CSR32_WR(REG_WR_TXDMA_CNF_L, txdma_cnf_len);     `CSR32_RD_NULL(REG_WR_TXDMA_CNF_L);
`CSR32_WR(REG_WR_TXDMA_CNF_T, txdma_cnf_tm);      `CSR32_RD_NULL(REG_WR_TXDMA_CNF_T);
`CSR32_WR(REG_WR_TXDMA_COMB, txdma_controlcomb);  `CSR32_RD_NULL(REG_WR_TXDMA_COMB);
`CSR32_WR(REG_INTS, interrupts);                  `CSR32_RD_NULL(REG_INTS);

// CSRs [16:31]
`CSR32_WR_NULL(16); `CSR32_RD_CONST(16, `SUBVECTOR(igpi_data_ext, 32, 0));
`CSR32_WR_NULL(17); `CSR32_RD_CONST(17, `SUBVECTOR(igpi_data_ext, 32, 1));
`CSR32_WR_NULL(18); `CSR32_RD_CONST(18, `SUBVECTOR(igpi_data_ext, 32, 2));
`CSR32_WR_NULL(19); `CSR32_RD_CONST(19, `SUBVECTOR(igpi_data_ext, 32, 3));

`CSR32_WR_NULL(20); `CSR32_RD_CONST(20, `SUBVECTOR(igpi_data_ext, 32, 4));
`CSR32_WR_NULL(21); `CSR32_RD_CONST(21, `SUBVECTOR(igpi_data_ext, 32, 5));
`CSR32_WR_NULL(22); `CSR32_RD_CONST(22, `SUBVECTOR(igpi_data_ext, 32, 6));
`CSR32_WR_NULL(23); `CSR32_RD_CONST(23, `SUBVECTOR(igpi_data_ext, 32, 7));

`CSR32_WR_NULL(24); `CSR32_RD_CONST(24, `SUBVECTOR(igpi_data_ext, 32, 8));
`CSR32_WR_NULL(25); `CSR32_RD_CONST(25, `SUBVECTOR(igpi_data_ext, 32, 9));
`CSR32_WR_NULL(26); `CSR32_RD_CONST(26, `SUBVECTOR(igpi_data_ext, 32, 10));
`CSR32_WR_NULL(27); `CSR32_RD_CONST(27, `SUBVECTOR(igpi_data_ext, 32, 11));

`CSR32_WR(REG_WR_TXDMA_CFG0, txdma_cfg0);  `CSR32_RD(REG_RD_TXDMA_STAT, txdma_stat);
`CSR32_WR(REG_WR_TXDMA_CFG1, txdma_cfg1);  `CSR32_RD(REG_RD_TXDMA_STATM, txdma_statm);
`CSR32_WR(REG_WR_TXDMA_TS_HI, txdma_ts_h); `CSR32_RD(REG_RD_TXDMA_STATTS, txdma_statts);
`CSR32_WR(REG_WR_TXDMA_TS_LO, txdma_ts_l); `CSR32_RD(REG_RD_TXDMA_STAT_CPL, txdma_stat_cpl);

// CSRs [32:47]
`CSR32_WR_NULL(32); `CSR32_RD_NULL(32);
`CSR32_WR_NULL(33); `CSR32_RD_NULL(33);
`CSR32_WR_NULL(34); `CSR32_RD_NULL(34);
`CSR32_WR_NULL(35); `CSR32_RD_NULL(35);

`CSR32_WR_NULL(36); `CSR32_RD_NULL(36);
`CSR32_WR_NULL(37); `CSR32_RD_NULL(37);
`CSR32_WR_NULL(38); `CSR32_RD_NULL(38);
`CSR32_WR_NULL(39); `CSR32_RD_NULL(39);

`CSR32_WR_NULL(40); `CSR32_RD_NULL(40);
`CSR32_WR_NULL(41); `CSR32_RD_NULL(41);
`CSR32_WR_NULL(42); `CSR32_RD_NULL(42);
`CSR32_WR_NULL(43); `CSR32_RD_NULL(43);

`CSR32_WR_NULL(44); `CSR32_RD_NULL(44);
`CSR32_WR_NULL(45); `CSR32_RD_NULL(45);
`CSR32_WR_NULL(46); `CSR32_RD_NULL(46);
`CSR32_WR_NULL(47); `CSR32_RD_NULL(47);

// CSRs [48:63]
`CSR32_RDWR(REG_SPI1, spi1);
`CSR32_RDWR(REG_SPI2, spi2);
`CSR32_RDWR(REG_SPI3, spi3);
`CSR32_WR_NULL(51); `CSR32_RD_NULL(51);

`CSR32_RDWR(REG_LBDSP, lbdsp);
`CSR32_WR_NULL(53); `CSR32_RD_NULL(53);
`CSR32_RDWR(REG_UART_TRX, uart_trx);
`CSR32_WR_NULL(55); `CSR32_RD_NULL(55);

`CSR32_RDWR(REG_CFG_PHY_0, cfg_phy_0);
`CSR32_RDWR(REG_CFG_PHY_1, cfg_phy_1);
`CSR32_WR(REG_SPI_EXT_CFG, spi_ext_cfg); `CSR32_RD_NULL(58);
`CSR32_RDWR(REG_SPI_EXT_DATA, spi_ext);

`CSR32_WR(REG_I2C2_CFG, i2c2_cfg);  `CSR32_RD_NULL(60);
`CSR32_RDWR(REG_I2C2,  i2c2);
`CSR32_WR_NULL(62); `CSR32_RD_NULL(62);
`CSR32_WR_NULL(63); `CSR32_RD_NULL(63);



assign axis_rd_gpi_data = igpi_data_ext[31:0];
assign axis_rd_gpi_valid = 1'b1;

/////////////////////////////////////////////////////////////////////////////////////////////
// Interrupts
`DEFINE_AXIS_RV_PORT(axis_int_i2c_);
`DEFINE_AXIS_RV_PORT(axis_int_spi0_);
`DEFINE_AXIS_RV_PORT(axis_int_spi1_);
`DEFINE_AXIS_RV_PORT(axis_int_spi2_);
`DEFINE_AXIS_RV_PORT(axis_int_spi3_);
`DEFINE_AXIS_RV_PORT(axis_int_spi_ext_);
`DEFINE_AXIS_RV_PORT(axis_int_i2c2_);

/////////////////////////////////////////////////////////////////////////////////////////////
// I2C
reg [31:0]  m2pcie_i2c_lut;

axis_i2c_dme_wrapper  #(
    .I2C_SPEED(I2C_SPEED),
    .BUS_SPEED(PUL_BUS_SPEED),
    .CLOCK_STRETCHING(I2C_CLOCK_STRETCHING)
) axis_i2c_dme_wrapper (
    .rst(prst),
    .clk(pclk),

    .addr_lut(m2pcie_i2c_lut),

    .sda_in(sda_in[1:0]),
    .scl_in(scl_in[1:0]),
    .sda_out_eo(sda_out_eo[1:0]),
    .scl_out_eo(scl_out_eo[1:0]),

    `AXIS_RVD_PORT_CONN(s_cmd_, axis_wr_i2c_),
    `AXIS_RVD_PORT_CONN(m_rb_, axis_rd_i2c_),
    `AXIS_RV_PORT_CONN(m_int_, axis_int_i2c_)
);

assign axis_wr_i2c2_cfg_ready = 1'b1;

generate
if (I2C_COUNT > 1) begin
    reg [31:0]  m2pcie_i2c2_lut;

    axis_i2c_dme_wrapper  #(
        .I2C_SPEED(I2C_SPEED),
        .BUS_SPEED(PUL_BUS_SPEED),
        .CLOCK_STRETCHING(I2C_CLOCK_STRETCHING)
    ) axis_i2c2_dme_wrapper (
        .rst(prst),
        .clk(pclk),

        .addr_lut(m2pcie_i2c2_lut),

        .sda_in(sda_in[3:2]),
        .scl_in(scl_in[3:2]),
        .sda_out_eo(sda_out_eo[3:2]),
        .scl_out_eo(scl_out_eo[3:2]),

        `AXIS_RVD_PORT_CONN(s_cmd_, axis_wr_i2c2_),
        `AXIS_RVD_PORT_CONN(m_rb_, axis_rd_i2c2_),
        `AXIS_RV_PORT_CONN(m_int_, axis_int_i2c2_)
    );

    always @(posedge pclk) begin
        if (axis_wr_i2c2_cfg_valid && axis_wr_i2c2_cfg_ready) begin
            m2pcie_i2c2_lut <= axis_wr_i2c2_cfg_data;
        end
    end

end else begin
    assign axis_wr_i2c2_ready = 1'b1;
    assign axis_rd_i2c2_valid = 1'b1;
    assign axis_int_i2c2_valid = 1'b0;
end
endgenerate

/////////////////////////////////////////////////////////////////////////////////////////////
// SPI

`define INSTANCE_SPI(i) \
        axis_spi_wrapper /*axis_spi_null2*/ #( \
            .FIXED_DIV(SPI_DIV[8*i+7:8*i]), \
            .DATA_WIDTH(SPI_WIDTH[8*i+7:8*i]) \
        ) spi_inst``i`` ( \
            .clk(pclk), \
            .rst(prst), \
            `AXIS_RVD_PORT_CONN(axis_tx_, axis_wr_spi``i``_), \
            `AXIS_RVD_PORT_CONN(axis_rx_, axis_rd_spi``i``_), \
            .spi_mosi(spi_mosi[i]), \
            .spi_miso(spi_miso[i]), \
            .spi_sclk(spi_sclk[i]), \
            .spi_sen(spi_sen[i]), \
            `AXIS_RV_PORT_CONN(spi_interrupt_, axis_int_spi``i``_))

`INSTANCE_SPI(0);
`INSTANCE_SPI(1);
`INSTANCE_SPI(2);
`INSTANCE_SPI(3);

/*
`define NO_SPI_BUS(i) \
    assign spi_sclk[i] = 1'b1; \
    assign spi_sen[i] = 1'b1; \
    assign spi_mosi[i] = 1'b1

generate
    `INSTANCE_SPI(0);

    if (EGPIO_PRESENT) begin
        `NO_SPI_BUS(1);
    end else begin
        `INSTANCE_SPI(1);
    end

    if (NO_TX) begin
        `INSTANCE_SPI(2);
        `INSTANCE_SPI(3);
    end else begin
        `NO_SPI_BUS(2);
        `NO_SPI_BUS(3);
    end
endgenerate
*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// AUX bus
localparam RXFE_ADDR_BITS = 3;
localparam TXFE_ADDR_BITS = 3;

reg [1:0]                  bus_selector;

// RX DMA configuration (should be isolated!)
reg [7:0]                  m2_al_waddr;
wire [31:0]                m2_al_wdata  = axis_wr_mbus2_data_data;
wire                       m2_al_wvalid = axis_wr_mbus2_data_valid && (bus_selector == 2'b00);
wire                       m2_al_wready;

// TX DMA configuration (should be isolated!)
reg [7:0]                  m3_al_waddr;
wire [31:0]                m3_al_wdata  = axis_wr_mbus2_data_data;
wire                       m3_al_wvalid = axis_wr_mbus2_data_valid && (bus_selector == 2'b10);
wire                       m3_al_wready;

// RX FE registers
reg [RXFE_ADDR_BITS-1:0]   rxfe0_waddr;
wire [31:0]                rxfe0_wdata  = axis_wr_mbus2_data_data;
wire                       rxfe0_wvalid = axis_wr_mbus2_data_valid && (bus_selector == 2'b01);
wire                       rxfe0_wready;

// RX FE registers
reg [TXFE_ADDR_BITS-1:0]   txfe0_waddr;
wire [31:0]                txfe0_wdata  = axis_wr_mbus2_data_data;
wire                       txfe0_wvalid = axis_wr_mbus2_data_valid && (bus_selector == 2'b11);
wire                       txfe0_wready;

assign axis_wr_mbus2_data_ready = (bus_selector == 2'b11) ? txfe0_wready : (bus_selector == 2'b10) ? m3_al_wready : (bus_selector == 2'b01) ? rxfe0_wready : m2_al_wready;
assign axis_wr_mbus2_addr_ready = 1'b1;

always @(posedge pclk) begin
  if (prst) begin
    m2_al_waddr <= 0;
    m3_al_waddr <= 0;
    bus_selector  <= 0;
  end else if (axis_wr_mbus2_addr_valid && ~axis_wr_mbus2_addr_data[31]) begin
    bus_selector <= axis_wr_mbus2_addr_data[9:8];

    if (axis_wr_mbus2_addr_data[9:8] == 2'b11)
      txfe0_waddr   <= axis_wr_mbus2_addr_data[TXFE_ADDR_BITS-1:0];
    else if (axis_wr_mbus2_addr_data[9:8] == 2'b01)
      rxfe0_waddr   <= axis_wr_mbus2_addr_data[RXFE_ADDR_BITS-1:0];
    else if (axis_wr_mbus2_addr_data[9:8] == 2'b10)
      m3_al_waddr <= axis_wr_mbus2_addr_data;
    else
      m2_al_waddr <= axis_wr_mbus2_addr_data;
  end
end


///////////////////////////////////////////////////////////////////////////////////////////////////////////////
assign axis_wr_gpo_ready = 1'b1;
always @(posedge pclk) begin
  if (prst) begin
    igpo_data   <= IGPO_INIT;
    igpo_strobe <= 0;
  end else begin
    igpo_strobe <= 0;

    if (axis_wr_gpo_valid) begin
      if (axis_wr_gpo_data[31]) begin
        m2pcie_i2c_lut <= axis_wr_gpo_data;
      end else if (axis_wr_gpo_data[30:24] < IGPO_COUNT) begin
        igpo_strobe[axis_wr_gpo_data[30:24]]       <= 1'b1;

        igpo_data[8 * axis_wr_gpo_data[30:24] + 0] <= axis_wr_gpo_data[0];
        igpo_data[8 * axis_wr_gpo_data[30:24] + 1] <= axis_wr_gpo_data[1];
        igpo_data[8 * axis_wr_gpo_data[30:24] + 2] <= axis_wr_gpo_data[2];
        igpo_data[8 * axis_wr_gpo_data[30:24] + 3] <= axis_wr_gpo_data[3];
        igpo_data[8 * axis_wr_gpo_data[30:24] + 4] <= axis_wr_gpo_data[4];
        igpo_data[8 * axis_wr_gpo_data[30:24] + 5] <= axis_wr_gpo_data[5];
        igpo_data[8 * axis_wr_gpo_data[30:24] + 6] <= axis_wr_gpo_data[6];
        igpo_data[8 * axis_wr_gpo_data[30:24] + 7] <= axis_wr_gpo_data[7];
      end
    end
  end
end

// interrupts
localparam INT_COUNT      = (I2C_COUNT > 1) ? 16 : 8;

localparam INT_COUNT_BITS = (I2C_COUNT > 1) ? 4 : 3;
localparam INT_PUSH_BITS  = 2;

wire [INT_COUNT-1:0] int_valid;
wire [INT_COUNT-1:0] int_ready;

wire [INT_COUNT-1:0] evntproc_valid;
wire [INT_COUNT-1:0] evntproc_ready;

//////////////////////////////////////////////////////
wire dmarx_interrupt_valid;
wire dmarx_interrupt_ready       = int_ready[0];

wire dmatx_interrupt_valid;
wire dmatx_interrupt_ready       = int_ready[1];

wire dmarx_stat_cnf_valid        = evntproc_valid[0];
wire dmarx_stat_cnf_ready;


assign int_valid[0]   = dmarx_interrupt_valid;
assign int_valid[1]   = dmatx_interrupt_valid;

assign axis_int_spi0_ready = int_ready[2];
assign axis_int_spi1_ready = int_ready[3];
assign axis_int_spi2_ready = int_ready[4];
assign axis_int_spi3_ready = int_ready[5];

assign int_valid[2] = axis_int_spi0_valid;
assign int_valid[3] = axis_int_spi1_valid;
assign int_valid[4] = axis_int_spi2_valid;
assign int_valid[5] = axis_int_spi3_valid;

assign int_valid[6] = axis_int_i2c_valid;
assign axis_int_i2c_ready = int_ready[6];

assign int_valid[7] = axis_int_spi_ext_valid;
assign axis_int_spi_ext_ready = int_ready[7];

generate
if (I2C_COUNT > 1) begin
assign int_valid[8] = axis_int_i2c2_valid;
assign axis_int_i2c2_ready = int_ready[8];

assign int_valid[15:9] = 0;

assign evntproc_ready[15:8] = ~0;
end
endgenerate

assign evntproc_ready[7:1] = ~0;
assign evntproc_ready[0]   =  dmarx_stat_cnf_ready;
///////////////////////////////////////////////////////

`DEFINE_AXIS_RVD_PORT(evno_, INT_COUNT_BITS);
event_serialzer #(.COUNT(INT_COUNT), .COUNT_BITS(INT_COUNT_BITS)) evnt_ser (
	.clk(pclk),
	.rst(prst),
    `AXIS_RV_PORT_CONN(sn_event_, int_),
    `AXIS_RVD_PORT_CONN(m_evno_, evno_)
);

`DEFINE_AXIS_RVD_PORT(evnocnf_, INT_COUNT_BITS);
event_deserialzer #(.COUNT(INT_COUNT), .COUNT_BITS(INT_COUNT_BITS)) evnt_deser (
	.clk(pclk),
	.rst(prst),
    `AXIS_RVD_PORT_CONN(s_evno_, evnocnf_),
    `AXIS_RV_PORT_CONN(mn_event_, evntproc_)
);


wire [INT_COUNT_BITS - 1:0]                 push_evno_data;
wire [INT_COUNT_BITS + INT_PUSH_BITS - 1:0] push_evno_user;
wire                                        push_evno_valid;
wire                                        push_evno_ready;

localparam EVENT_WIDTH = 32;

wire [INT_COUNT_BITS - 1:0]                 pcied_evno_data;
wire [INT_COUNT_BITS + INT_PUSH_BITS - 1:0] pcied_evno_user;
wire                                        pcied_evno_valid;
wire                                        pcied_evno_ready;

event_router #(
    .COUNT_BITS(INT_COUNT_BITS),
    .USER_BITS(INT_PUSH_BITS),
    .DISP_COUNT(2),     // 2 buses
    .DISP_COUNT_BITS(1),
    .ULTRA_SCALE(ULTRA_SCALE)
) erouter (
    .clk(pclk),
    .rst(prst),

    `AXIS_RVD_PORT_CONN(s_cer_, axis_wr_interrupts_),
    `AXIS_RVD_PORT_CONN(s_evno_, evno_),

    // Disptch
    .mn_evno_data({push_evno_data, pcied_evno_data}),       // mmaped event
    .mn_evno_user({push_evno_user, pcied_evno_user}),       // original event + user data
    .mn_evno_valid({push_evno_valid, pcied_evno_valid}),
    .mn_evno_ready({push_evno_ready, pcied_evno_ready})
);

`DEFINE_AXIS_RVDL_PORT(evd_, EVENT_WIDTH);
`DEFINE_AXIS_RVDL_PORT(evd_f_, EVENT_WIDTH);
wire evd_f_pcie_ready;
wire evd_f_usb2_ready;
assign evd_f_ready = (!tran_usb_active) ? evd_f_pcie_ready : evd_f_usb2_ready;

event_fifo_gen #(
    .EVENT_COUNT_BITS(INT_COUNT_BITS),
    .ADDR_WIDTH(USER_CSR_BITS + 2),
    .DATA_WIDTH(32),
    .AUX_WIDTH(INT_COUNT_BITS + INT_PUSH_BITS),
    //                     i2c2    spi_e      i2c     spi3     spi2     spi1     spi0     tx0     rx0
    .A_EVENT_ADDRS(288'h0000003D_0000003B_00000001_0000000E_0000000D_00000003_00000002_0000001C_00000004),
    .A_EVENT_LEN(  288'h00000000_00000000_00000000_00000000_00000000_00000000_00000000_00000002_00000002)
) event_fifo_gen (
    .clk(pclk),
    .rst(prst),

    .s_evno_user({push_evno_data, push_evno_user[INT_PUSH_BITS-1:0]}),                // mmaped evno | user_data
    .s_evno_data(push_evno_user[INT_COUNT_BITS + INT_PUSH_BITS - 1:INT_PUSH_BITS]),   // original evno
    .s_evno_valid(push_evno_valid),
    .s_evno_ready(push_evno_ready),

    `AXIS_ALRD_CONNECT(m_al, egrb),
    `AXIS_RVDL_PORT_CONN(m_evd_, evd_),
    `AXIS_RVD_PORT_CONN(m_cnfno_, evnocnf_)
);

// Serial interrupts with associated data (not to block AL- CSR bus)
axis_fifo #(.WIDTH(1+32), .DEEP(16), .EXTRA_REG(1'b1)) axis_evd_fifo (
  .clk(pclk),
  .rst(prst),

  .s_rx_tdata({evd_last, evd_data}),
  .s_rx_tvalid(evd_valid),
  .s_rx_tready(evd_ready),

  .m_tx_tdata({evd_f_last, evd_f_data}),
  .m_tx_tvalid(evd_f_valid),
  .m_tx_tready(evd_f_ready)
);

`DEFINE_AXIS_RVD_PORT(pcied_ntfy_, INT_COUNT_BITS);

pcie_notification_gen_us #(
    .EVENT_WIDTH(EVENT_WIDTH),
    .INT_NUM_BITS(INT_COUNT_BITS),
    .ORIG_NUM_BITS(INT_COUNT_BITS),
    .BUCKET_BITS(INT_PUSH_BITS),
    .ULTRA_SCALE(ULTRA_SCALE),
    .DATA_BITS(BUCKET_BITS)
) pcie_gen (
    .clk(pclk),
    .rst(prst),

    .cfg_pcie_attr(2'b00),
    .cfg_pcie_reqid(cfg_completer_id),

    `AXIS_RVD_PORT_CONN(s_cfg_, axis_wr_pntfy_cfg_),     // Configuration channel
    `AXIS_RVD_PORT_CONN(s_ack_, axis_wr_pntfy_ack_),     // ACK channel

    // Event info
    `AXIS_VDL_PORT_CONN(s_evd_, evd_f_),
    .s_evd_ready(evd_f_pcie_ready),

	`AXIS_RVDLKU_PORT_CONN(m_axis_tx_t, pclk_axis_rq_event_t), // PCIe TLP
    `AXIS_RVD_PORT_CONN(m_int_, pcied_ntfy_)                  // Interrupt logic generator
);

`DEFINE_AXIS_RVD_PORT(pcied_comb_, INT_COUNT_BITS);
assign pcied_comb_data = (pcied_ntfy_valid) ? pcied_ntfy_data : pcied_evno_data;
assign pcied_comb_valid = pcied_ntfy_valid || pcied_evno_valid;
assign pcied_ntfy_ready = pcied_comb_ready;
assign pcied_evno_ready = !pcied_ntfy_valid && pcied_comb_ready;

`DEFINE_AXIS_RVD_PORT(h_pcied_comb_, INT_COUNT_BITS);
axis_opt_cc_fifo #(
  .WIDTH(INT_COUNT_BITS),
  .CLK_CC(PUL_BUS_SPEED != HUL_BUS_SPEED)
) pcied_comb_p_to_h (
  .rx_clk(pclk),
  .rx_rst(prst),
  `AXIS_RVD_PORT_CONN(s_rx_t, pcied_comb_),
  .tx_clk(hclk),
  .tx_rst(hrst),
  `AXIS_RVD_PORT_CONN(m_tx_t, h_pcied_comb_)
);

pci_msi_intrrupt #(
    .COUNT(INT_COUNT),
    .COUNT_BITS(INT_COUNT_BITS)
) pci_msi_intrrupt_g (
    .clk(hclk),
    .rst(hrst),

    `AXIS_RVD_PORT_CONN(s_int_, h_pcied_comb_),

     // PCI-e MSI interface
    .interrupt_msi_enabled(cfg_interrupt_msienable),
    .interrupt_rdy(cfg_interrupt_rdy),
    .interrupt(cfg_interrupt),
    .interrupt_num(cfg_interrupt_di),

    .interrupt_mmenable(cfg_interrupt_mmenable),
    .cap_interrupt_msgnum(cfg_pciecap_interrupt_msgnum)
);

assign axis_rd_interrupts_valid = 1'b1;
assign axis_rd_interrupts_data  = 32'h0;

// ==========================================================================
// RX Streaming
// ==========================================================================

localparam RAM_RX_ADDR_W = 16;
localparam WIDTH_RX      = C_DATA_WIDTH;
localparam WIDTH_FE_RX   = ADC_CHANS * ADC_WIDTH;
localparam ADC_ADDR_BITS = $clog2(WIDTH_FE_RX / 8);

wire                                 fe_rxdma_ten;
wire [RAM_RX_ADDR_W-1:ADC_ADDR_BITS] fe_rxdma_taddr;
wire [WIDTH_FE_RX-1:0]               fe_rxdma_tdata_wr;
wire [WIDTH_FE_RX/8-1:0]             fe_rxdma_twbe;

wire [WIDTH_RX/8-1:0]              rxdma_bram_wbe = 0;
wire [WIDTH_RX-1:0]                rxdma_bram_data_wr = 0;
wire [WIDTH_RX-1:0]                rxdma_bram_data_rd;
wire [RAM_RX_ADDR_W-1:DATA_BITS]   rxdma_bram_addr;
wire                               rxdma_bram_en;

generate
if (ULTRA_SCALE) begin
    if (RAM_RX_ADDR_W == 16 && WIDTH_RX == 128 && WIDTH_FE_RX == 128) begin
        // 64kB, direct map
        blk_mem_gen_nrx_128_128 fifo_mem_rx (
            .clka(hclk),
            .rsta(hrst),
            .ena(rxdma_bram_en),
            .wea(rxdma_bram_wbe),
            .addra(rxdma_bram_addr),
            .dina(rxdma_bram_data_wr),
            .douta(rxdma_bram_data_rd),

            .clkb(adc_clk),
            .rstb(adc_rst),
            .enb(fe_rxdma_ten),
            .web(fe_rxdma_twbe),
            .addrb(fe_rxdma_taddr),
            .dinb(fe_rxdma_tdata_wr),
            .doutb()
        );
    end else if (RAM_RX_ADDR_W == 16 && WIDTH_RX == 256 && WIDTH_FE_RX == 128) begin
        // 64kB, direct map
        blk_mem_gen_nrx_256_128 fifo_mem_rx (
            .clka(hclk),
            .rsta(hrst),
            .ena(rxdma_bram_en),
            .wea(rxdma_bram_wbe),
            .addra(rxdma_bram_addr),
            .dina(rxdma_bram_data_wr),
            .douta(rxdma_bram_data_rd),

            .clkb(adc_clk),
            .rstb(adc_rst),
            .enb(fe_rxdma_ten),
            .web(fe_rxdma_twbe),
            .addrb(fe_rxdma_taddr),
            .dinb(fe_rxdma_tdata_wr),
            .doutb()
        );
    end
end else begin
    blk_mem_gen_nrx fifo_mem_rx (
        .clka(hclk),
        .rsta(hrst),
        .ena(rxdma_bram_en),
        .wea(rxdma_bram_wbe),
        .addra(rxdma_bram_addr),
        .dina(rxdma_bram_data_wr),
        .douta(rxdma_bram_data_rd),

        .clkb(adc_clk),
        .rstb(adc_rst),
        .enb(fe_rxdma_ten),
        .web(fe_rxdma_twbe),
        .addrb(fe_rxdma_taddr),
        .dinb(fe_rxdma_tdata_wr),
        .doutb()
    );
end
endgenerate

wire                                rxctr_tcq_valid;
wire                                rxctr_tcq_ready;
wire [RAM_RX_ADDR_W-1:DATA_BITS]    rxctr_tcq_laddr;
wire [31:DATA_BITS]                 rxctr_tcq_raddr;
wire [9:DATA_BITS]                  rxctr_tcq_length;
wire [0:0]                          rxctr_tcq_tag;

wire                                rxctr_tcq_cvalid;
wire                                rxctr_tcq_cready;
wire [0:0]                          rxctr_tcq_ctag;


wire                                rxdma_tcq_valid  = rxctr_tcq_valid && !tran_usb_active;
wire                                rxdma_tcq_ready;
wire [RAM_RX_ADDR_W-1:DATA_BITS]    rxdma_tcq_laddr  = rxctr_tcq_laddr;
wire [31:DATA_BITS]                 rxdma_tcq_raddr  = rxctr_tcq_raddr;
wire [9:DATA_BITS]                  rxdma_tcq_length = rxctr_tcq_length;
wire [0:0]                          rxdma_tcq_tag    = rxctr_tcq_tag;

wire                                rxdma_tcq_cvalid;
wire                                rxdma_tcq_cready = rxctr_tcq_cready && !tran_usb_active;
wire [0:0]                          rxdma_tcq_ctag;


wire                                rxusb_tcq_valid  = rxctr_tcq_valid && tran_usb_active;
wire                                rxusb_tcq_ready;
wire [RAM_RX_ADDR_W-1:DATA_BITS]    rxusb_tcq_laddr  = rxctr_tcq_laddr;
//wire [31:DATA_BITS]                 rxusb_tcq_raddr  = rxctr_tcq_raddr;
wire [9:DATA_BITS]                  rxusb_tcq_length = rxctr_tcq_length;
wire [0:0]                          rxusb_tcq_tag    = rxctr_tcq_tag;

wire                                rxusb_tcq_cvalid;
wire                                rxusb_tcq_cready = rxctr_tcq_cready && tran_usb_active;
wire [0:0]                          rxusb_tcq_ctag;

assign                              rxctr_tcq_ready  = tran_usb_active ? rxusb_tcq_ready  : rxdma_tcq_ready;
assign                              rxctr_tcq_cvalid = tran_usb_active ? rxusb_tcq_cvalid : rxdma_tcq_cvalid;
assign                              rxctr_tcq_ctag   = tran_usb_active ? rxusb_tcq_ctag   : rxdma_tcq_ctag;

`DEFINE_ALRDI_AXIS(al_rxbram, RAM_RX_ADDR_W - DATA_BITS, C_DATA_WIDTH, 1);
`DEFINE_ALRDI_AXIS(al_rxpcie, RAM_RX_ADDR_W - DATA_BITS, C_DATA_WIDTH, 1);
`DEFINE_ALRDI_AXIS(al_rxusbs, RAM_RX_ADDR_W - DATA_BITS, C_DATA_WIDTH, 1);

assign al_rxbram_araddr  = tran_usb_active ? al_rxusbs_araddr  : al_rxpcie_araddr;
assign al_rxbram_arvalid = tran_usb_active ? al_rxusbs_arvalid : al_rxpcie_arvalid;
assign al_rxbram_arid    = tran_usb_active ? al_rxusbs_arid    : al_rxpcie_arid;
assign al_rxusbs_arready = al_rxbram_arready;
assign al_rxpcie_arready = al_rxbram_arready;

assign al_rxusbs_rdata   = al_rxbram_rdata;
assign al_rxusbs_rvalid  = al_rxbram_rvalid;
assign al_rxusbs_rid     = al_rxbram_rid;
assign al_rxpcie_rdata   = al_rxbram_rdata;
assign al_rxpcie_rvalid  = al_rxbram_rvalid;
assign al_rxpcie_rid     = al_rxbram_rid;
assign al_rxbram_rready  = tran_usb_active ? al_rxusbs_rready : al_rxpcie_rready;

alrdwr_to_bram #(.ADDR_WIDTH(RAM_RX_ADDR_W), .DATA_BITS(DATA_BITS), .BRAM_STAGES(1)) alrdwr_to_bram (
    .clk(hclk),
    .rst(hrst),

    `AXIS_ALRDI_CONNECT(s_al, al_rxbram),
    .s_al_wvalid(1'b0),

    .bram_addr(rxdma_bram_addr),
    .bram_en(rxdma_bram_en),
    .bram_le(),
    .bram_we(),
    .bram_data_rd(rxdma_bram_data_rd),
    .bram_data_wr()
);

al_ram_to_pcie_memwr #(
    .LOCAL_ADDR_WIDTH(RAM_RX_ADDR_W),
    .DATA_BITS(DATA_BITS),
    .ULTRA_SCALE(ULTRA_SCALE),
    .TX_BUF_CTRL(1'b1),
    .EARLY_CNF(1)
) pcie_ram_to_rq_memwr (
    .clk(hclk),
    .rst(hrst),

    .s_tcq_valid(rxdma_tcq_valid),
    .s_tcq_ready(rxdma_tcq_ready),
    .s_tcq_laddr(rxdma_tcq_laddr),
    .s_tcq_raddr(rxdma_tcq_raddr),
    .s_tcq_length(rxdma_tcq_length),
    .s_tcq_tag(rxdma_tcq_tag),

    .s_tcq_cvalid(rxdma_tcq_cvalid),
    .s_tcq_cready(rxdma_tcq_cready),
    .s_tcq_ctag(rxdma_tcq_ctag),

    .cfg_pcie_attr(/*cfg_pcie_dma_rx_attr*/ 2'b00),
    .cfg_pcie_reqid(cfg_completer_id),
    .pcie7s_tx_buf_av(pcie7s_tx_buf_av),
    .pcieus_tx_busy(us_rq_busy),

    // AXIs PCIe RQ
    `AXIS_RVDLKU_PORT_CONN(m_axis_tx_t, hclk_axis_rq_rxdma_t),
    `AXIS_ALRDI_CONNECT(m_al, al_rxpcie)
);

wire                     rxfe0_fifo_burst_release;
wire                     rxfe0_fifo_burst_skip;
wire                     rxfe0_fifo_burst_fill;
wire                     rxfe0_fifo_burst_mlowmrk;
wire                     rx_timer_en;

assign axis_rd_rxfe0_timeevent_valid = 1'b1;
assign axis_rd_rxfe0_pwr0_valid = 1'b1;
assign axis_rd_rxfe0_pwr1_valid = 1'b1;
assign axis_rd_rxfe0_timeevent_data = 0;
assign axis_rd_rxfe0_pwr0_data = 0;
assign axis_rd_rxfe0_pwr1_data = 0;

reg [1:0] cfg_max_payload_sz_adc_clk;
always @(posedge adc_clk) begin
  //if (adc_rst) begin
    //cfg_max_payload_sz_adc_clk <= 2'b0;
  //end else begin
    cfg_max_payload_sz_adc_clk <= tran_usb_active ? 2'b10 : cfg_max_payload_size[1:0];
  //end
end
//

`ifndef FE_RX_IPCORE
`define FE_RX_IPCORE fe_simple_rx
`endif

`FE_RX_IPCORE #(
    .BUFFER_SIZE_ADDR(RAM_RX_ADDR_W),
    .DATA_WIDTH(ADC_WIDTH),
    .RAW_CHANS(ADC_CHANS),
    .ASYNC_BURST_CLOCK(ADC_ASYNC_CLK)
) fe_simple_rx (
    .clk(adc_clk),
    .rst(adc_rst),

    .s_in_data(adc_realigned),
    .s_in_valid(adc_fifo_valid),
    .s_in_ready(rx_ready),
    .o_enable(adc_enable),
    .o_tstp(),
    .o_ch_enabled(adc_ch_enable),
    .o_running(rx_streaming),

    //RAM
    .m_rxfe_tvalid(fe_rxdma_ten),
    .m_rxfe_taddr(fe_rxdma_taddr),
    .m_rxfe_tdata(fe_rxdma_tdata_wr),
    .m_rxfe_tkeep(fe_rxdma_twbe),
    .m_rxfe_tlast(),

    // For USB use 512b for BULK EP
    .cfg_mlowmrk(cfg_max_payload_sz_adc_clk),

    // FIFO control
    .fifo_burst_clk(pclk),
    .fifo_burst_rst(~rx_dmaactive),

    .s_fifo_burst_cmd_data(rxfe0_wdata),
    .s_fifo_burst_cmd_valid(rxfe0_wvalid && (rxfe0_waddr == 0)),
    .s_fifo_burst_cmd_ready(rxfe0_wready),

    .fifo_burst_timer_rst(!rx_timer_en),

    .fifo_burst_avail_z(),
    .fifo_burst_release(rxfe0_fifo_burst_release),
    .fifo_burst_skip(rxfe0_fifo_burst_skip),
    .fifo_burst_fill(rxfe0_fifo_burst_fill),
    .fifo_burst_mlowmrk(rxfe0_fifo_burst_mlowmrk)
);

wire usb_inplace_cnf_valid;
wire usb_inplace_cnf_ready;
wire usb_inplace_cnf_dual = 1'b1;

reg [1:0] cfg_max_payload_size_pclk;
always @(posedge pclk) begin
    if (prst) begin
        cfg_max_payload_size_pclk <= 2'b00;
    end else begin
        // use 512b data frames for USB
        cfg_max_payload_size_pclk <= tran_usb_active ? 2'b10 : cfg_max_payload_size[1:0];
    end
end

dma_rxrq_us  #(
    .BUFFER_SIZE_BITS(RAM_RX_ADDR_W),
    .DATA_BITS(DATA_BITS),
    .ULTRA_SCALE(ULTRA_SCALE)
) dma_rxrq (
    .clk(pclk),
    .rst(prst),

    `AXIS_ALWR_CONNECT(s_al, m2_al),

    `AXIS_RVD_PORT_CONN(axis_control_, axis_wr_rxdma_controlcomb_),
    `AXIS_RV_PORT_CONN(axis_confirm_,  axis_wr_rxdma_confirm_),

    `AXIS_RVD_PORT_CONN(axis_stat_,    axis_rd_rxdma_stat_),
    `AXIS_RVD_PORT_CONN(axis_fburts_,  axis_rd_rxdma_bursts_),
    `AXIS_RVD_PORT_CONN(axis_fbuffs_,  axis_rd_rxdma_buffs_),

    `AXIS_RV_PORT_CONN(int_,           dmarx_interrupt_),
    `AXIS_RV_PORT_CONN(stat_cnf_,      dmarx_stat_cnf_),

    .inplace_cnf_enabled(tran_usb_active),
    .inplace_cnf_valid(usb_inplace_cnf_valid),
    .inplace_cnf_ready(usb_inplace_cnf_ready),

    .rx_streamingready(rx_streamingready),
    .rx_dmaactive(rx_dmaactive),

    .cfg_max_payload_sz(cfg_max_payload_size_pclk),
    .fe_stat({ adc_ch_enable, 1'b0, rx_streaming }),

    .m_tcq_valid(rxctr_tcq_valid),
    .m_tcq_ready(rxctr_tcq_ready),
    .m_tcq_laddr(rxctr_tcq_laddr),
    .m_tcq_raddr(rxctr_tcq_raddr),
    .m_tcq_length(rxctr_tcq_length),
    .m_tcq_tag(rxctr_tcq_tag),

    .m_tcq_cvalid(rxctr_tcq_cvalid),
    .m_tcq_cready(rxctr_tcq_cready),
    .m_tcq_ctag(rxctr_tcq_ctag),

    .fifo_burst_avail_z(),
    .fifo_burst_release(rxfe0_fifo_burst_release),
    .fifo_burst_skip(rxfe0_fifo_burst_skip),
    .fifo_burst_fill(rxfe0_fifo_burst_fill),
    .fifo_burst_mlowmrk(rxfe0_fifo_burst_mlowmrk)
);

wire tx_streamingready;
wire [63:0] fetx_stat;

localparam RAM_TX_ADDR_W     = 17;

localparam TX_TIMESTAMP_BITS = TX_EX_CORE ? 64 : 49; // 48 bits + NOTS flag
localparam TX_RAM_ADDR_WIDTH = RAM_TX_ADDR_W;
localparam TX_SAMPLES_WIDTH  = (TX_EX_CORE) ? TX_RAM_ADDR_WIDTH : TX_RAM_ADDR_WIDTH - 1;
localparam TX_FE_DESCR_WIDTH = TX_TIMESTAMP_BITS + TX_SAMPLES_WIDTH + (TX_RAM_ADDR_WIDTH - DATA_BITS) + 2 * TX_EX_CORE;
localparam STAT_CNTR_WIDTH   = 8;

`DEFINE_AXIS_RVD_PORT(txusb_descr_, TX_FE_DESCR_WIDTH);

wire                              usb_bram_en;
wire [RAM_TX_ADDR_W-1:DATA_BITS]  usb_bram_addr;
wire [C_DATA_WIDTH-1:0]           usb_bram_wdata;
wire [C_DATA_WIDTH/8-1:0]         usb_bram_wbe;

wire  [TX_TIMESTAMP_BITS - 1:0]   fedma_ts;
wire  [TX_RAM_ADDR_WIDTH:8]       fedma_ram_addr;

wire                              txdma_nactive;
wire  [1:0]                       fetx_cfg_format;
generate
if (NO_TX) begin

assign dac_realigned = 0;
assign dac_fifo_valid = 0;
assign tx_streaming = 0;
assign tx_streamingready = 1'b1;
assign rx_streamingready = 1'b1;

assign rx_timer_en = 1'b1; //TODO

assign dmatx_interrupt_valid = 1'b0;

assign m_axis_rc_tready = 1'b1;

assign hclk_axis_rq_txdma_tdata = 0;
assign hclk_axis_rq_txdma_tkeep = 0;
assign hclk_axis_rq_txdma_tlast = 0;
assign hclk_axis_rq_txdma_tvalid = 1'b0;

assign txdma_nactive = 1'b1;

assign m_axis_rc_tready = 1'b1;

assign m3_al_wready = 1'b1;
assign txfe0_wready = 1'b1;

assign sysref_gen = 1'b0;

assign axis_wr_txdma_cnf_len_ready = 1'b1;
assign axis_wr_txdma_cnf_tm_ready = 1'b1;
assign axis_wr_txdma_controlcomb_ready = 1'b1;

assign axis_wr_txdma_cfg0_ready = 1'b1;
assign axis_wr_txdma_cfg1_ready = 1'b1;
assign axis_wr_txdma_ts_h_ready = 1'b1;
assign axis_wr_txdma_ts_l_ready = 1'b1;

assign axis_rd_txdma_stat_valid = 1'b1;
assign axis_rd_txdma_statm_valid = 1'b1;
assign axis_rd_txdma_statts_valid = 1'b1;
assign axis_rd_txdma_stat_cpl_valid = 1'b1;

assign axis_rd_txdma_stat_data = 0;
assign axis_rd_txdma_statm_data = 0;
assign axis_rd_txdma_statts_data = 0;
assign axis_rd_txdma_stat_cpl_data = 0;

end else begin

////////////////////////////////////////////////////////////////////////////////
//    TX STREAMING


wire                              txdma_bram_en;
wire [RAM_TX_ADDR_W-1:DATA_BITS]  txdma_bram_addr;
wire [C_DATA_WIDTH-1:0]           txdma_bram_wdata;
wire [C_DATA_WIDTH/8-1:0]         txdma_bram_wbe;

wire                              txs_bram_en    = tran_usb_active ? usb_bram_en    : txdma_bram_en;
wire [RAM_TX_ADDR_W-1:DATA_BITS]  txs_bram_addr  = tran_usb_active ? usb_bram_addr  : txdma_bram_addr;
wire [C_DATA_WIDTH-1:0]           txs_bram_wdata = tran_usb_active ? usb_bram_wdata : txdma_bram_wdata;
wire [C_DATA_WIDTH/8-1:0]         txs_bram_wbe   = tran_usb_active ? usb_bram_wbe   : txdma_bram_wbe;

wire                              fe_aux_tvalid; // = 1'b0;

wire                                   fe_txdma_ten;
wire [RAM_TX_ADDR_W-1:FEDATA_TX_BITS]  fe_txdma_taddr;
wire [FEDATA_TX_WIDTH-1:0]             fe_txdma_tdata_rd_ext;

if (ULTRA_SCALE) begin
    if (RAM_TX_ADDR_W == 17 && C_DATA_WIDTH == 128 && FEDATA_TX_WIDTH == 128) begin
        // 128kB of RAM
        blk_mem_gen_ntx_128_128_2 fifo_mem_tx (
            .clka(hclk),
            .ena(    txs_bram_en),
            .wea(    |txs_bram_wbe),
            .addra(  txs_bram_addr),
            .dina(   txs_bram_wdata),
            .douta(),

            .clkb(dac_clk),
            .enb(fe_txdma_ten),
            .web(0),
            .addrb(fe_txdma_taddr),
            .dinb(0),
            .doutb(fe_txdma_tdata_rd_ext)
        );
    end else if (RAM_TX_ADDR_W == 17 && C_DATA_WIDTH == 256 && FEDATA_TX_WIDTH == 128) begin
        blk_mem_gen_ntx_256_128_2 fifo_mem_tx (
            .clka(hclk),
            .ena(    txs_bram_en),
            .wea(    |txs_bram_wbe),
            .addra(  txs_bram_addr),
            .dina(   txs_bram_wdata),
            .douta(),

            .clkb(dac_clk),
            .enb(fe_txdma_ten),
            .web(0),
            .addrb(fe_txdma_taddr),
            .dinb(0),
            .doutb(fe_txdma_tdata_rd_ext)
        );
    end
end else begin

localparam WIDTH_TX = 48;

// 8b->6b mapping
wire [5:0]                txs_bram_wbe_mapped;
wire [WIDTH_TX-1:0]       txs_bram_wdata_mapped;

assign txs_bram_wdata_mapped = {
  txs_bram_wdata[63:52],
  txs_bram_wdata[47:36],
  txs_bram_wdata[31:20],
  txs_bram_wdata[15:4]
};

assign txs_bram_wbe_mapped = {
  txs_bram_wbe[7] | txs_bram_wbe[6],
  txs_bram_wbe[6] | txs_bram_wbe[5],
  txs_bram_wbe[5] | txs_bram_wbe[4],

  txs_bram_wbe[3] | txs_bram_wbe[2],
  txs_bram_wbe[2] | txs_bram_wbe[1],
  txs_bram_wbe[1] | txs_bram_wbe[0]
};

// DSP read channel
// wire                     fe_txdma_ten;
// wire [RAM_TX_ADDR_W-1:3] fe_txdma_taddr;
wire [WIDTH_TX-1:0]      fe_txdma_tdata_rd_r;
assign                   fe_txdma_tdata_rd_ext = {
    fe_txdma_tdata_rd_r[47:36], 4'h0,
    fe_txdma_tdata_rd_r[35:24], 4'h0,
    fe_txdma_tdata_rd_r[23:12], 4'h0,
    fe_txdma_tdata_rd_r[11:0],  4'h0
};

// DSP write channel
wire                             fe_aux_tvalid; // = 1'b0;
wire                             fe_aux_tready;
wire [RAM_TX_ADDR_W-1:3]         fe_aux_taddr;
wire [WIDTH_TX-1:0]              fe_aux_tdata;
wire [5:0]                       fe_aux_tkeep;

//Multiplexed access to TX ram
wire                              ntx_fifo_enb  = fe_txdma_ten || (fe_aux_tvalid && fe_aux_tready);
wire [5:0]                        ntx_fifo_web  = (fe_aux_tvalid && fe_aux_tready) ? fe_aux_tkeep : 6'h00;
wire [RAM_TX_ADDR_W-1:3]          ntx_fifo_addr = (fe_aux_tvalid && fe_aux_tready) ? fe_aux_taddr : fe_txdma_taddr;
assign                            fe_aux_tready = ~fe_txdma_ten;


// 96kB of RAM mapped to 128kB space, 8b -> 6b on the fly translation
blk_mem_gen_ntx fifo_mem_tx (
    .clka(hclk),
    .rsta(hrst),
    .ena(    txs_bram_en),
    .wea(    txs_bram_wbe_mapped),
    .addra(  txs_bram_addr),
    .dina(   txs_bram_wdata_mapped),
    .douta(),

    .clkb(dac_clk),
    .rstb(1'b0),
    .enb(ntx_fifo_enb),
    .web(ntx_fifo_web),
    .addrb(ntx_fifo_addr),
    .dinb(fe_aux_tdata),
    .doutb(fe_txdma_tdata_rd_r)
);

end



//
// fe_simple_tx          | Transmition front end      | TXCLK/PCLK
// dma_tx_wrapper        | Wrapper DMA                | PCLK
//  - circ_dma_tx_engine | Circular DMA engine        | PCLK
//  - dma_tx_pcie        | DMA buffer requester       | PCLK
// pcie_rq_rc_al_mem     | PCIe RQ/RC to RAM    logic | HCLK
//


localparam PCIE_TAG_BITS     = 5;
localparam TX_STAT_FE_WIDTH  = 20;
localparam TX_BUS_ADDR_WIDTH = 32;
localparam DAC_CMD_WIDTH     = 3;
localparam TX_FIFO_ID        = DAC_CMD_WIDTH + FEDATA_TX_BITS + 1;

`DEFINE_ALRDI_AXIS(tx_fifo, TX_RAM_ADDR_WIDTH - FEDATA_TX_BITS, FEDATA_TX_WIDTH, TX_FIFO_ID);

wire [TX_RAM_ADDR_WIDTH-1:DATA_BITS] tcq_laddr;
wire [TX_BUS_ADDR_WIDTH-1:DATA_BITS] tcq_raddr;
wire [11:DATA_BITS]                  tcq_length;
wire [PCIE_TAG_BITS-1:0]             tcq_tag;
wire                                 tcq_valid;
wire                                 tcq_ready;

wire                                 tcq_cvalid;
wire                                 tcq_cready;
wire [PCIE_TAG_BITS-1:0]             tcq_ctag;

wire                                 txdma_bram_tvalid;
assign                               txdma_bram_en  = txdma_bram_tvalid;
assign                               txdma_bram_wbe = {(C_DATA_WIDTH/8){txdma_bram_tvalid}};

wire [1:0]                           fetx_cfg_mute;
wire                                 fetx_cfg_swap;

wire                                 proc_idx_valid;
wire                                 proc_idx_ready;

`DEFINE_AXIS_RVD_PORT(txfe_descr_, TX_FE_DESCR_WIDTH);
`DEFINE_AXIS_RVD_PORT(txdma_descr_, TX_FE_DESCR_WIDTH);
`DEFINE_AXIS_RVD_PORT(txcomb_descr_, TX_FE_DESCR_WIDTH);

wire                                 dacclk_fe_underrun;
wire  [TX_STAT_FE_WIDTH-1:0]         txdma_fe_underruns;

//wire                                 dac_frame;
wire                                 dac_sync;
wire                                 dac_rst;
wire                                 tx_timer_en;

assign                               tx_streaming = !dac_rst;


if (DAC_ASYNC_CLK) begin
    synchronizer #(.INIT(1)) tx_dmaen_to_dacclk_rst (
      .clk(dac_clk),
      .rst(prst /*1'b0*/),
      .a_in(txdma_nactive),
      .s_out(dac_rst)
    );

    synchronizer #(.INIT(0)) tx_timer_to_dacclk_sync (
      .clk(dac_clk),
      .rst(dac_rst),
      .a_in(tx_timer_en),
      .s_out(dac_sync)
    );
end else begin
    assign dac_rst  = txdma_nactive;
    assign dac_sync = tx_timer_en;
end

axis_opt_pipeline #(.WIDTH(TX_FIFO_ID), .PIPE_PASSTHROUGH(TX_EX_CORE)) tx_ram_p (
  .clk(dac_clk),
  .rst(dac_rst),

  .s_rx_tdata(tx_fifo_arid),
  .s_rx_tvalid(tx_fifo_arvalid),
  .s_rx_tready(tx_fifo_arready),

  .m_tx_tdata(tx_fifo_rid),
  .m_tx_tvalid(tx_fifo_rvalid),
  .m_tx_tready(tx_fifo_rready)
);
assign fe_txdma_taddr = tx_fifo_araddr;
assign fe_txdma_ten   = tx_fifo_arvalid && tx_fifo_arready;
assign tx_fifo_rdata  = fe_txdma_tdata_rd_ext;

cc_counter #(
   .WIDTH(TX_STAT_FE_WIDTH),
   .GRAY_BITS(4)
) cc_tx_timer (
   .in_clk(dac_clk),
   .in_rst(dac_rst),
   .in_increment( dacclk_fe_underrun ),
   .in_counter(),

   .out_clk(pclk),
   .out_rst(txdma_nactive),
   .out_counter(txdma_fe_underruns)
);

`ifndef FE_TX_IPCORE
`define FE_TX_IPCORE fe_simple_tx
`endif

`FE_TX_IPCORE #(
    .DESCR_DATA_BITS(DATA_BITS),
    .RAM_ADDR_WIDTH(TX_RAM_ADDR_WIDTH),
    .TIMESTAMP_BITS(TX_TIMESTAMP_BITS),
    .DATA_BITS(FEDATA_TX_BITS),
    .SAMPLES_WIDTH(TX_SAMPLES_WIDTH),
    .FE_DESCR_WIDTH(TX_FE_DESCR_WIDTH),
    .FRAME_LENGTH(TX_FRAME_LENGTH), //Extra samples to flush beforehand to compensate distribution delay through extra pipeline stages
    .INITIAL_TS_COMP(TX_INITIAL_TS_COMP),
    .ASYNC_CLK(DAC_ASYNC_CLK),
    .SUPPORT_8CH(TX_SUPPORT_8CH),
    .SUPPORT_4CH(TX_SUPPORT_4CH),
    .RAW_CHANS(DAC_CHANS),
    .ULTRA_SCALE(ULTRA_SCALE),
    .DAC_CMD_WIDTH(DAC_CMD_WIDTH)
) fe_simple_tx(
    .clk(dac_clk),
    .rst(dac_rst),

    .rst_fe(dac_rst_fe),

    `AXIS_RVD_PORT_CONN(s_descr_, txfe_descr_),

    // Burst processed and is ready to be reused
    .m_proc_idx_valid(proc_idx_valid),
    .m_proc_idx_ready(proc_idx_ready),

    // CC-stat
    .m_fedma_clk(pclk),
    .m_fedma_rst(txdma_nactive),
    .m_fedma_ts(fedma_ts),               //Current TS to discard old buffer requests
    .m_fedma_ram_addr(fedma_ram_addr),   //Determines availability of RAM

    // Configurations
    .cfg_mute(fetx_cfg_mute),
    .cfg_swap(fetx_cfg_swap),
    .cfg_format(fetx_cfg_format),

    .sig_underrun(dacclk_fe_underrun),

    `AXIS_ALRDI_CONNECT(m_fifo, tx_fifo),

    .s_fe_cmd_data(txfe0_wdata),
    .s_fe_cmd_valid(txfe0_wvalid && (txfe0_waddr == 0)),
    .s_fe_cmd_ready(txfe0_wready),

    // DATA to DACs
    .dac_data(dac_realigned),
    .dac_valid(dac_fifo_valid),
    .dac_ready(tx_ready),
    .dac_frame(dac_frame),  // LFMC clock
    .dac_sync(dac_sync)     // crossing from 0->1 resets TX timer
);

assign txcomb_descr_data  = (tran_usb_active) ? txusb_descr_data  : txdma_descr_data;
assign txcomb_descr_valid = (tran_usb_active) ? txusb_descr_valid : txdma_descr_valid;
assign txusb_descr_ready  = tran_usb_active && txcomb_descr_ready;
assign txdma_descr_ready  = !tran_usb_active && txcomb_descr_ready;

// TODO: Assume FIFO is 32 deep, put proper constant
if (DAC_ASYNC_CLK) begin
    axis_opt_cc_fifo #(
      .WIDTH(TX_FE_DESCR_WIDTH),
      .CLK_CC(1'b1)
    ) axis_txfedescr_pclk_dacclk (
      .rx_clk(pclk),
      .rx_rst(txdma_nactive),
      `AXIS_RVD_PORT_CONN(s_rx_t, txcomb_descr_),

      .tx_clk(dac_clk),
      .tx_rst(dac_rst),
      `AXIS_RVD_PORT_CONN(m_tx_t, txfe_descr_)
    );
end else begin
    axis_fifo #(.WIDTH(TX_FE_DESCR_WIDTH), .DEEP(32)) axis_txfedescr_sync (
      .clk(dac_clk),
      .rst(dac_rst),
      `AXIS_RVD_PORT_CONN(s_rx_t, txcomb_descr_),
      `AXIS_RVD_PORT_CONN(m_tx_t, txfe_descr_)
    );
end

wire [STAT_CNTR_WIDTH-1:0] stat_cpl_nodata;
wire                       stat_notlp;

pcie_rq_rc_al_mem #(
    .LOCAL_ADDR_WIDTH(TX_RAM_ADDR_WIDTH),
    .REMOTE_ADDR_WIDTH(TX_BUS_ADDR_WIDTH),
    .REQUEST_LEN_BITS(12),
    .MEM_TAG(PCIE_TAG_BITS),
    .DATA_BITS(DATA_BITS), // 3 - 64 bit, 4 - 128 bit, 5 - 256 bit
    .ULTRA_SCALE(ULTRA_SCALE),
    .EN64BIT(0), // for 7-Series,
    .PCIE_TAG_BITS(5),
    .STAT_CNTR_WIDTH(STAT_CNTR_WIDTH)
) pcie_rq_rc_al_mem (
    .clk(hclk),
    .rst(hrst),

    .s_tcq_valid(tcq_valid),
    .s_tcq_ready(tcq_ready),
    .s_tcq_laddr(tcq_laddr),
    .s_tcq_raddr(tcq_raddr),
    .s_tcq_length(tcq_length),
    .s_tcq_tag(tcq_tag),

    // Request termination, s_tcq_ctag can be reused
    .s_tcq_cvalid(tcq_cvalid),
    .s_tcq_cready(tcq_cready),
    .s_tcq_ctag(tcq_ctag),

    .core_ready(),
    .cfg_pcie_reqid(cfg_completer_id),
    .cfg_pcie_attr(2'b00),
    .pcie7s_tx_buf_av(pcie7s_tx_buf_av),

    `AXIS_RVDLKU_PORT_CONN(m_axis_rq_t, hclk_axis_rq_txdma_t),
    `AXIS_RVDLKU_PORT_CONN(s_axis_rc_t, m_axis_rc_t),

    .m_al_wraddr(txdma_bram_addr),
    .m_al_wdata(txdma_bram_wdata),
    .m_al_wvalid(txdma_bram_tvalid),
    .m_al_wtag(),
    .m_al_wready(1'b1),

    .stat_notlp(stat_notlp), // There's no non-posted TLP in the fly
    .stat_cpl_nodata(stat_cpl_nodata),

    .extra_data( { m_axis_cq_tvalid, m_axis_cq_tready, axis_wr_txdma_cnf_tm_ready, axis_wr_txdma_cnf_tm_valid } )
);

dma_tx_wrapper #(
    .TIMESTAMP_BITS(TX_TIMESTAMP_BITS),
    .TIMESTAMP_LATE_DISCARD(TX_EX_CORE),
    .BUS_ADDR_WIDTH(TX_BUS_ADDR_WIDTH),
    .RAM_ADDR_WIDTH(TX_RAM_ADDR_WIDTH),
    .DATA_BITS(DATA_BITS),
    .PCIE_TAG_BITS(PCIE_TAG_BITS),
    .SAMPLES_WIDTH(TX_SAMPLES_WIDTH),
    .FE_DESCR_WIDTH(TX_FE_DESCR_WIDTH),
    .ULTRA_SCALE(ULTRA_SCALE),
    .STAT_FE_WIDTH(TX_STAT_FE_WIDTH),
    .SUPPORT_8CH(TX_SUPPORT_8CH),
    .SUPPORT_4CH(TX_SUPPORT_4CH),
    .TX_EX_CORE(TX_EX_CORE),
    .STAT_CNTR_WIDTH(STAT_CNTR_WIDTH)
) dma_tx_wrapper (
    .clk(pclk),
    .rst(prst),

    ///////////////////////////////////////////////////////
    .fe_mute(fetx_cfg_mute),
    .fe_swap(fetx_cfg_swap),
    .fe_format(fetx_cfg_format),

    .s_fedma_ts(fedma_ts),               //Current TS to discard old buffer requests
    .s_fedma_ram_addr(fedma_ram_addr),   //Determines availability of RAM

    .s_proc_idx_valid(proc_idx_valid),
    .s_proc_idx_ready(proc_idx_ready),

    `AXIS_RVD_PORT_CONN(m_descr_, txdma_descr_),

    ///////////////////////////////////////////////////////
    `AXIS_ALWR_CONNECT(s_dmacfg, m3_al),

    // PCIe mover interface
    .m_tcq_valid(tcq_valid),
    .m_tcq_ready(tcq_ready),
    .m_tcq_laddr(tcq_laddr),
    .m_tcq_raddr(tcq_raddr),
    .m_tcq_length(tcq_length),
    .m_tcq_tag(tcq_tag),

    // Request termination, s_tcq_ctag can be reused
    .m_tcq_cvalid(tcq_cvalid),
    .m_tcq_cready(tcq_cready),
    .m_tcq_ctag(tcq_ctag),

    // Interrupt for buffer completion (aired)
    `AXIS_RV_PORT_CONN(m_int_, dmatx_interrupt_),

    .cfg_max_req_sz(cfg_max_read_req_size),

    // Old Control
    `AXIS_RVD_PORT_CONN(axis_cnf_len_, axis_wr_txdma_cnf_len_),
    `AXIS_RVD_PORT_CONN(axis_tm_len_,  axis_wr_txdma_cnf_tm_),

    // New Control
    `AXIS_RVD_PORT_CONN(axis_ncfg0_, axis_wr_txdma_cfg0_),
    `AXIS_RVD_PORT_CONN(axis_ncfg1_, axis_wr_txdma_cfg1_),
    `AXIS_RVD_PORT_CONN(axis_ntsh_,  axis_wr_txdma_ts_h_),
    `AXIS_RVD_PORT_CONN(axis_ntsl_,  axis_wr_txdma_ts_l_),

    .axis_control_data(axis_wr_txdma_controlcomb_data),
    .axis_control_valid(axis_wr_txdma_controlcomb_valid && ~axis_wr_txdma_controlcomb_data[31]),
    .axis_control_ready(),

    // Output data status stream
    `AXIS_RVD_PORT_CONN(axis_stat_,     axis_rd_txdma_stat_),
    `AXIS_RVD_PORT_CONN(axis_stat_m_,   axis_rd_txdma_statm_),
    `AXIS_RVD_PORT_CONN(axis_stat_ts_,  axis_rd_txdma_statts_),
    `AXIS_RVD_PORT_CONN(axis_stat_cpl_, axis_rd_txdma_stat_cpl_),

    // AUX status
    .fe_late_bursts(txdma_fe_underruns),
    .tx_buffprecharged(tx_streamingready),
    .txdma_nactive(txdma_nactive),

    .stat_cpl_nodata(stat_cpl_nodata),
    .stat_notlp(stat_notlp),

    .tran_usb_active(tran_usb_active),
    .usb_fetx_stat(fetx_stat)
);

assign axis_wr_txdma_controlcomb_ready = 1'b1;

////////////////////////////////////////////////////////////////////////////////
// rx_streamingready
// tx_streamingready
// tx_timer_en
// rx_timer_en

time_sync time_sync (
    .rx_streamingready(rx_streamingready),
    .tx_streamingready(tx_streamingready),
    .tx_timer_en(tx_timer_en),
    .rx_timer_en(rx_timer_en),

    // sync events
    .onepps_raw(pps_on_in),
    .sysref_gen(sysref_gen),

    // control
    .igp_clk(pclk),
    .igp_reset(prst),
    .igp_timecmd_valid(axis_wr_txdma_controlcomb_valid && axis_wr_txdma_controlcomb_data[31]),
    .igp_timecmd_data(axis_wr_txdma_controlcomb_data[31:16]),
    .igp_timecmd_ready()
);

end
endgenerate


localparam FLASH_RAM_BUFF_WIDTH = 7; // 128-byte temp buffer for flash IO

`DEFINE_ALRDI_AXIS(al_flashram, FLASH_RAM_BUFF_WIDTH - 2, 32, 1);
`DEFINE_ALWR_AXIS(al_flashram, FLASH_RAM_BUFF_WIDTH - 2, 32);

`DEFINE_ALRDWR_AXIS(mem_flash, FLASH_RAM_BUFF_WIDTH - 2, 32);

wire [6:2]  spiflash_ram_addr;
wire        spiflash_ram_en;
wire        spiflash_ram_we;
wire [31:0] spiflash_ram_data_rd;
wire [31:0] spiflash_ram_data_wr;

alrdwr_to_bram #(.ADDR_WIDTH(FLASH_RAM_BUFF_WIDTH), .DATA_BITS(2), .BRAM_STAGES(0)) alrdwr_to_flash (
    .clk(pclk),
    .rst(prst),

    `AXIS_ALRDI_CONNECT(s_al, al_flashram),
    `AXIS_ALWR_CONNECT(s_al, al_flashram),

    .bram_addr(spiflash_ram_addr),
    .bram_en(spiflash_ram_en),
    .bram_le(),
    .bram_we(spiflash_ram_we),
    .bram_data_rd(spiflash_ram_data_rd),
    .bram_data_wr(spiflash_ram_data_wr)
);

ram_sxp #(.DATA_WIDTH(32), .ADDR_WIDTH(FLASH_RAM_BUFF_WIDTH - 2), .ULTRA_SCALE(ULTRA_SCALE), .MODE_SDP(1'b0)) ram_flash_buffer (
    .wclk(pclk),
    .we(spiflash_ram_we && spiflash_ram_en),
    .waddr(spiflash_ram_addr),
    .wdata(spiflash_ram_data_wr),

    .raddr(spiflash_ram_addr),
    .rdata(spiflash_ram_data_rd)
);

al_spi_memory_wrapper al_spi_memory_wrapper(
    .clk(pclk),
    .rst(prst),

    `AXIS_RVD_PORT_CONN(flashspi_addr_,  axis_wr_flashspi_addr_),
    `AXIS_RVD_PORT_CONN(flashspi_cmd_,   axis_wr_flashspi_cmd_),
    `AXIS_RVD_PORT_CONN(flashspi_rd_,    axis_rd_flashspi_rb_),
    `AXIS_RVD_PORT_CONN(flashspi_stat_,  axis_rd_flashspi_stat_),

    `AXIS_ALRDWR_CONNECT(m_mem, mem_flash),

    .qphy_clk(flash_sck),
    .qphy_di(flash_din),
    .qphy_do(flash_dout),
    .qphy_dt(flash_ndrive),
    .qphy_dncs(flash_ncs)
);

// mem_flash + mem_al => al_flashram
alrd_mux #(.ADDR_WIDTH(FLASH_RAM_BUFF_WIDTH), .SLAVE_COUNT(2)) flash_ram_mux (
    .clk(pclk),
    .rst(prst),

    `AXIS_ALRD_VCONNECT_2(sn_al, mem_al, mem_flash),
    `AXIS_ALRDI_CONNECT(m_al,  al_flashram)
);

assign al_flashram_wvalid = mem_al_wvalid || mem_flash_wvalid;
assign al_flashram_waddr  = (mem_flash_wvalid) ? mem_flash_waddr : mem_al_waddr;
assign al_flashram_wdata  = (mem_flash_wvalid) ? mem_flash_wdata : mem_al_wdata;
assign mem_flash_wready   = al_flashram_wready;
assign mem_al_wready      = !mem_flash_wvalid && al_flashram_wready;


generate
if (EGPIO_PRESENT) begin: egpio_present
    // GPIO & AUX logic
    axis_gpio #(.WIDTH(EGPIO_WIDTH), .ALT_MODE1(EGPIO_ALT_MODE1)) axis_gpio(
        .clk(pclk),
        .rst(prst),

        // UL Write channel
        .axis_wdata(axis_wr_egpio_data),
        .axis_wvalid(axis_wr_egpio_valid),
        .axis_wready(axis_wr_egpio_ready),

        // Output data readback stream
        .axis_rdata(axis_rd_egpio_data),
        .axis_rvalid(axis_rd_egpio_valid),
        .axis_rready(axis_rd_egpio_ready),

        .gpi_data(egpio_data_in),
        .gpo_data(egpio_data_out),
        .gpo_data_oe(egpio_data_oe),
        .gpio_altf0_valid(egpio_altf0_active),

        .alt0_in(egpio_alt0_in),
        .alt0_out(egpio_alt0_out),
        .alt0_out_oe(egpio_alt0_out_oe)
    );
end else begin
    // NO EGPIO
    assign axis_wr_egpio_ready = 1'b1;
    assign axis_rd_egpio_data = 0;
    assign axis_rd_egpio_valid = 1'b1;

    assign egpio_data_oe  = {EGPIO_WIDTH{1'b0}};
    assign egpio_data_out = {EGPIO_WIDTH{1'b0}};
    assign egpio_alt0_in  = {EGPIO_WIDTH{1'b1}};
    assign egpio_altf0_active = {EGPIO_WIDTH{1'b1}};
end
endgenerate


generate
if (USB2_PRESENT) begin: usb2_present
    `DEFINE_AXIS_RVDLK_PORT(axis_usbtx_t, 32, 4);

    usb2_core  #(
        .AL_BUS_WIDTH(AL_BUS_WIDTH),
        .USDR_PID(USDR_PID),
        .DATA_BITS(DATA_BITS),
        .NO_TX(NO_TX),
        .TX_TIMESTAMP_BITS(TX_TIMESTAMP_BITS),
        .TX_RAM_ADDR_WIDTH(TX_RAM_ADDR_WIDTH),
        .TX_SAMPLES_WIDTH(TX_SAMPLES_WIDTH)
    ) usb2_core(
        // ULPI interface
        .phy_rst(phy_rst),
        .phy_clk(phy_clk),

        .phy_do(phy_do),
        .phy_di(phy_di),
        .phy_doe(phy_doe),

        .phy_dir(phy_dir),
        .phy_nxt(phy_nxt),
        .phy_stp(phy_stp),

        .clk(pclk),
        .rst(prst),

        `AXIS_ALRDWR_CONNECT(m_al, uul),

        `AXIS_VDL_PORT_CONN(s_evd_, evd_f_),
        .s_evd_ready(evd_f_usb2_ready),

        // TX FIFO-RAM interface
        .tx_nactive(txdma_nactive || !tran_usb_active),  ///
        .fetx_mode_format(fetx_cfg_format),
        .fetx_stat(fetx_stat),

        .txfe_out_rd_addr(fedma_ram_addr),
        .txfe_out_rd_time(fedma_ts),

        .mem_tvalid(usb_bram_en),
        .mem_tready(1'b1),
        .mem_taddr(usb_bram_addr),
        .mem_tdata(usb_bram_wdata),
        .mem_tkeep(usb_bram_wbe),

        `AXIS_RVD_PORT_CONN(m_usbs_burst_, txusb_descr_),
        .m_usbs_burst_busy(!txusb_descr_ready),

        // RX USB-EP-S interface
        `AXIS_RVDLK_PORT_CONN(s_axis_usbtx_t, axis_usbtx_t),

        // AUX signal
        .usb_bus_reset(usb_bus_reset)
    );

    al_ram_to_rx_usbeps #(
        .LOCAL_ADDR_WIDTH(RAM_RX_ADDR_W),
        .DATA_BITS(DATA_BITS)
    ) al_ram_to_rx_usbeps (
        .clk(pclk),
        .rst(prst),

        .s_tcq_valid(rxusb_tcq_valid),
        .s_tcq_ready(rxusb_tcq_ready),
        .s_tcq_laddr(rxusb_tcq_laddr),
        .s_tcq_length(rxusb_tcq_length),
        .s_tcq_tag(rxusb_tcq_tag),
        .s_tcq_trailer(usb_inplace_cnf_valid),

        .s_tcq_cvalid(rxusb_tcq_cvalid),
        .s_tcq_cready(rxusb_tcq_cready),
        .s_tcq_ctag(rxusb_tcq_ctag),

        `AXIS_RVDLK_PORT_CONN(m_axis_usbtx_t, axis_usbtx_t),

        // In-place EOB notification
        .s_usbtx_ntfy_valid( usb_inplace_cnf_valid ),
        .s_usbtx_ntfy_data({ axis_rd_txdma_statm_data, usb_tx_dfrm_stat , axis_rd_rxdma_bursts_data, axis_rd_rxdma_buffs_data }),
        .s_usbtx_ntfy_dual( usb_inplace_cnf_dual ),
        .s_usbtx_ntfy_ready( usb_inplace_cnf_ready ),

        `AXIS_ALRDI_CONNECT(m_al, al_rxusbs)
    );

end else begin
    assign uul_wvalid = 1'b0;
    assign uul_waddr = 0;
    assign uul_wdata = 0;

    assign uul_arvalid = 1'b0;
    assign uul_rready = 1'b1;

    assign phy_do = 8'h00;
    assign phy_doe = 1'b0;
    assign phy_stp = 1'b0;
    assign usb_bus_reset = 1'b0;

    assign usb_bram_en = 1'b0;
    assign usb_bram_addr = 1'b0;
    assign usb_bram_wdata = 1'b0;
    assign usb_bram_wbe = 1'b0;

    assign txusb_descr_valid = 1'b1;
    assign txusb_descr_data = 1'b1;

    assign fetx_stat = 0;

    assign rxusb_tcq_ready  = 1'b1;
    assign rxusb_tcq_cvalid = 1'b1;
    assign rxusb_tcq_ctag   = 1'b1;

    assign usb_inplace_cnf_ready = 1'b0;

end
endgenerate

generate
if (UART_PRESENT) begin: gen_has_uart
    simple_uart_trx_axisfifo  #(
        .UART_SPEED(9600),
        .BUS_SPEED(PUL_BUS_SPEED),
        .BUS_ALT_SPEED((PUL_BUS_SPEED * 6) / 10)
    ) simple_uart_trx (
        .rst(prst),
        .clk(pclk),

        .rxd(uart_rxd),
        .txd(uart_txd),

        .cfg_alt_mode(tran_usb_active),

        // Output data readback stream
        .axis_rdata(axis_rd_uart_trx_data),
        .axis_rvalid(axis_rd_uart_trx_valid),
        .axis_rready(axis_rd_uart_trx_ready),

        .axis_wdata(axis_wr_uart_trx_data),
        .axis_wvalid(axis_wr_uart_trx_valid),
        .axis_wready(axis_wr_uart_trx_ready),

        // Interrupt
        .int_ready(1'b1),
        .int_valid()
    );
end else begin
    // NO UART
    assign axis_rd_uart_trx_data  = 32'h0;
    assign axis_rd_uart_trx_valid = 1'b1;
    assign axis_wr_uart_trx_ready = 1'b1;

    assign uart_txd = 1'b1; // IDLE state
end
endgenerate

generate
if (SPI_EXT_PRESENT) begin
    axis_spi_ext_wrapper  #(
        .DATA_WIDTH(32),
        .CS_COUNT(8)
    ) spi_ext_inst (
        .clk(pclk),
        .rst(prst),
        `AXIS_RVD_PORT_CONN(axis_cfg_, axis_wr_spi_ext_cfg_),
        `AXIS_RVD_PORT_CONN(axis_tx_, axis_wr_spi_ext_),
        `AXIS_RVD_PORT_CONN(axis_rx_, axis_rd_spi_ext_),
        .spi_mosi(spi_ext_mosi),
        .spi_miso(spi_ext_miso),
        .spi_sclk(spi_ext_sclk),
        .spi_sen(spi_ext_sen),
        `AXIS_RV_PORT_CONN(spi_interrupt_, axis_int_spi_ext_)
    );
end else begin
    assign spi_ext_mosi = 1'b1;
    assign spi_ext_sclk = 1'b1;
    assign spi_ext_sen = 8'hff;

    assign axis_wr_spi_ext_cfg_ready = 1'b1;
    assign axis_wr_spi_ext_ready = 1'b1;
    assign axis_rd_spi_ext_valid = 1'b1;
    assign axis_rd_spi_ext_data = 0;

    assign axis_int_spi_ext_valid = 1'b0;
end
endgenerate

// NO LB
assign fe_aux_tvalid = 1'b0;
assign axis_rd_lbdsp_data = 32'h0;
assign axis_rd_lbdsp_valid = 1'b1;
assign axis_wr_lbdsp_ready = 1'b1;

// CFG PHY
assign axis_wr_cfg_phy_0_ready = cfg_port_wready[0];
assign axis_wr_cfg_phy_1_ready = cfg_port_wready[1];
assign cfg_port_wvalid = { axis_wr_cfg_phy_1_valid, axis_wr_cfg_phy_0_valid };
assign cfg_port_wdata = axis_wr_cfg_phy_0_data;

assign cfg_port_rready = { axis_rd_cfg_phy_1_ready, axis_rd_cfg_phy_0_ready };
assign axis_rd_cfg_phy_0_valid = cfg_port_rvalid[0];
assign axis_rd_cfg_phy_1_valid = cfg_port_rvalid[1];

assign axis_rd_cfg_phy_0_data = cfg_port_rdata[31:0];
assign axis_rd_cfg_phy_1_data = cfg_port_rdata[63:32];

assign cfg_port_clk = pclk;
assign cfg_port_rst = prst;

endmodule



