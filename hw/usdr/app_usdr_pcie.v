// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module app_usdr_pcie #(
    parameter MASTER_BUS_SPEED = 125_000_000,
    parameter I2C_SPEED = 400_000,
    parameter I2C_CLOCK_STRETCHING = 1'b0,
    parameter USB2_PRESENT = 1'b1,
    parameter USDR_PID     = 16'h1001
)(
    // Global clock domains for the device
    input          gclk_dsp,
    input          gclk_master,

    ////////////////////////////////////////////////////////////////////////////
    // PCIe interface (to PCIe hard core)
    input          reset,

    input [63:0]   s_axis_rx_tdata,
    input [1:0]    s_axis_rx_tkeep,
    input          s_axis_rx_tlast,
    input          s_axis_rx_tvalid,
    output         s_axis_rx_tready,
    input  [6:0]   pcie_rx_bar_hit,

    output [63:0]  m_axis_tx_tdata,
    output [1:0]   m_axis_tx_tkeep,
    output         m_axis_tx_tlast,
    output         m_axis_tx_tvalid,
    input          m_axis_tx_tready,
    input [5:0]    pcie7s_tx_buf_av,

    // pci configuration
    input [15:0]   cfg_completer_id,
    input [2:0]    cfg_max_payload_size,
    input [2:0]    cfg_max_read_req_size,

    // pci interrupts (legacy + MSI)
    input          cfg_interrupt_msienable,
    input          cfg_interrupt_rdy,
    output         cfg_interrupt,
    output         cfg_interrupt_assert,
    output [7:0]   cfg_interrupt_di,
    input          legacy_interrupt_disabled,
    input [2:0]    cfg_interrupt_mmenable,
    output [4:0]   cfg_pciecap_interrupt_msgnum,
    output         cfg_interrupt_stat,


    ////////////////////////////////////////////////////////////////////////////
    // LMS6 input samples IQ demuxed
    input [31:0]   hwcfg_port,

    input          rxclk_f_clk,
    input [11:0]   rxd_iob_data,
    input          rxd_iob_iqsel,
    output         rxd_enable,

    // LMS6 output samples IQ demuxed
    input          txclk_m_clk,
    output [11:0]  txd_iob_data,
    output         txd_iob_iqsel,
    output         txd_enable,

    // I2C bus
    input  [0:0]   sda_in,
    input  [0:0]   scl_in,
    output [0:0]   sda_out_eo,
    output [0:0]   scl_out_eo,

    // SPI bus
    output [0:0]   spi_mosi,
    input  [0:0]   spi_miso,
    output [0:0]   spi_sclk,
    output [0:0]   spi_sen,

    // UART bus
    input  [0:0]   uart_rxd,
    output [0:0]   uart_txd,

    // QSPI bus
    output [3:0]   flash_dout,
    input  [3:0]   flash_din,
    output [3:0]   flash_ndrive,
    output         flash_ncs,
    output         flash_sck,
    input          flash_cclk,

    // GPIOs
    output         igpo_lms_rst_n,
    output         igpo_rxmix_en,
    output         igpo_txsw,
    output         igpo_rxsw,
    output         igpo_dcen,
    output         igpo_led,

    input [31:0]   igpi_usr_access2_data,

    output [17:0]  user_gpio_oe,
    output [17:0]  user_gpio_out,
    input  [17:0]  user_gpio_in,

    output         enable_osc,

    // USB2 ULPI interface
    input        usb2_altrst_valid,
    input        usb2_altrst_logicrst,
    input        usb2_altrst_phynrst,

    input        usb2_phy_clk,
    output       usb2_phy_nrst,
    input        usb2_phy_dir,
    input        usb2_phy_nxt,
    output       usb2_phy_stp,
    output [7:0] usb2_phy_do,
    input  [7:0] usb2_phy_di,
    output       usb2_phy_doe
);

wire   usb_bus_reset;

`include "../axi_helpers.vh"
////////////////////////////////////////////////////////////////////////////////
// BOARD CONFIGURATION

localparam IGPO_COUNT        = 18;
localparam IGPI_COUNT        = 28;

localparam IGPO_LMS_RST      = 0;
localparam IGPO_RXMIX_EN     = 1;
localparam IGPO_TXSW         = 2;
localparam IGPO_RXSW         = 3;
localparam IGPO_DSP_RX_CFG   = 4;
localparam IGPO_DSP_TX_CFG   = 5;
localparam IGPO_USB2_CFG     = 6;
localparam IGPO_BOOSTER      = 7;
localparam IGPO_LED          = 8;
localparam IGPO_DCCORR       = 9;
localparam IGPO_DSP_RX_CTRL  = 10;

localparam IGPO_CLKMEAS      = 16;
localparam IGPO_ENABLE_OSC   = 17;

localparam IGPI_USR_ACCESS2  = 0;
localparam IGPI_CORE_CONF1   = 4;
localparam IGPI_CORE_CONF2   = 8;
localparam IGPI_HWID         = 12;

localparam IGPI_DC_CORR_VAL  = 20;
localparam IGPI_CLK1PPS      = 24;


wire [IGPO_COUNT * 8 - 1:0] intgpo;
wire [IGPO_COUNT - 1:0]     igpo_s;
wire [IGPI_COUNT * 8 - 1:0] intgpi;


assign intgpi[IGPI_USR_ACCESS2 * 8 + 31 : IGPI_USR_ACCESS2 * 8 + 0] = igpi_usr_access2_data;
assign intgpi[IGPI_HWID * 8 + 31        : IGPI_HWID * 8 + 0]        = hwcfg_port;

wire      igp_rst;
wire      igp_clk = gclk_master;
synchronizer  #(.INIT(1), .ASYNC_RESET(1)) usb_bus_igp_reg  (
    .clk(igp_clk),
    .rst(USB2_PRESENT && usb2_altrst_valid ? usb_bus_reset : 1'b0),
    .a_in(reset),
    .s_out(igp_rst)
);

////////////////////////////////////////////////////////////////////////////////
wire      dsp_clk = gclk_dsp;
wire      dsp_prog_rst;

////////////////////////////////////////////////////////////////////////////////

assign igpo_lms_rst_n  = intgpo[IGPO_LMS_RST * 8 + 0];
assign igpo_rxmix_en   = intgpo[IGPO_RXMIX_EN * 8 + 0];
assign igpo_txsw       = intgpo[IGPO_TXSW * 8 + 0];
assign igpo_rxsw       = intgpo[IGPO_RXSW * 8 + 0];
assign igpo_dcen       = intgpo[IGPO_BOOSTER * 8 + 0];
assign igpo_led        = intgpo[IGPO_LED * 8 + 0];
assign enable_osc      = intgpo[IGPO_ENABLE_OSC * 8 + 0];

wire   igpdspcfg_rst   = intgpo[IGPO_DSP_RX_CTRL * 8 + 0];

////////////////////////////////////////////////////////////////////////////////
// LMS IF

wire       rxclk_stream_rst;
wire       dsp_strem_rst;
wire       igp_stream_rst;

reg [23:0] rxd_demux_data;
reg        rxd_demux_valid;

always @(posedge rxclk_f_clk) begin
  if (rxclk_stream_rst) begin
    rxd_demux_valid <= 1'b0;
  end else begin
    if (rxd_iob_iqsel) begin
      rxd_demux_data[23:12] <= rxd_iob_data;
      rxd_demux_valid       <= 1'b1;
    end else begin
      rxd_demux_data[11:0]  <= rxd_iob_data;
      rxd_demux_valid       <= 1'b0;
    end
  end
end

wire [31:0] dac_fifo_data;
wire        dac_fifo_valid;
wire        dac_fifo_ready;

reg [11:0]  txd_iob_data_s;
reg         txd_iob_iqsel_s;
assign txd_iob_data   = txd_iob_data_s;
assign txd_iob_iqsel  = txd_iob_iqsel_s;
assign dac_fifo_ready = txd_iob_iqsel_s;

always @(posedge txclk_m_clk) begin
    if (~txd_enable) begin
        txd_iob_data_s  <= 0;
        txd_iob_iqsel_s <= 0;
    end else begin
        if (dac_fifo_valid) begin
            txd_iob_iqsel_s <= ~txd_iob_iqsel_s;
            txd_iob_data_s  <= (txd_iob_iqsel_s) ? dac_fifo_data[31:20] : dac_fifo_data[15:4];
        end
    end
end

////////////////////////////////////////////////////////////////////////////////
// DSP RX
wire       cfg_dsprst = intgpo[IGPO_LMS_RST * 8 + 1];

wire [7:0] cfg_dspchain_prg_data  = intgpo[IGPO_DSP_RX_CFG*8 + 7 : IGPO_DSP_RX_CFG*8 + 0];
wire       cfg_dspchain_prg_valid = igpo_s[IGPO_DSP_RX_CFG];

synchronizer  #(.INIT(1), .ASYNC_RESET(1)) dsp_reset_reg  (.clk(dsp_clk), .rst(cfg_dsprst), .a_in(igp_rst), .s_out(dsp_prog_rst));

wire       dsp_dspchain_prg_valid;
wire [7:0] dsp_dspchain_prg_data;

axis_cc_fifo #(
    .WIDTH(8),
    .DEEP_BITS(3)
) igpdspcfg_fifo (
    .rx_clk(igp_clk),
    .rx_rst(igp_rst),

    .s_rx_tdata(cfg_dspchain_prg_data),
    .s_rx_tvalid(cfg_dspchain_prg_valid),
    .s_rx_tready(),

    .tx_clk(dsp_clk),
    .tx_rst(dsp_rst),

    .m_tx_tdata(dsp_dspchain_prg_data),
    .m_tx_tvalid(dsp_dspchain_prg_valid),
    .m_tx_tready(1'b1)
);

wire [31:0] dsp_data;
wire        dsp_valid;
wire        dsp_ready = 1'b1;


// RX cross clock chain  (reset -- source rx_streaming)
// rxclk_f_clk        =>       dsp_clk       =>        igp_clk
// rxclk_stream_rst   <=   dsp_strem_rst     <=        { igp_stream_rst | igpdspcfg_rst }

wire       rx_streaming;
reg        rx_str_rst;
assign igp_stream_rst = rx_str_rst;
always @(posedge igp_clk) begin
    if (igpdspcfg_rst/*igp_rst*/) begin
        rx_str_rst <= 1'b1;
    end else begin
        rx_str_rst <= !rx_streaming;
    end
end

synchronizer  #(.INIT(1), .ASYNC_RESET(0)) rxclk_stream_rst_sync  (.clk(rxclk_f_clk), .rst(0), .a_in(dsp_strem_rst),  .s_out(rxclk_stream_rst));
synchronizer  #(.INIT(1), .ASYNC_RESET(0)) dsp_strem_rst_sync     (.clk(dsp_clk),     .rst(0), .a_in(igp_stream_rst), .s_out(dsp_strem_rst));

rx_dsp_chain #(
    .ADC_WIDTH(12),
    .CFG_WIDTH(8),
    .STAGES(8),
    .DC_CORR(1),
    .CORDIC(0)
) dsp_chain (
    .adc_clk(rxclk_f_clk),
    .adc_rst(rxclk_stream_rst),
    .adc_data(rxd_demux_data),
    .adc_valid(rxd_demux_valid),
    .adc_dsp_cordic_phase(0),

    .adc_dc_corr_en(intgpo[IGPO_DCCORR * 8 + 0]),
    .adc_dc_corr_vals(intgpi[IGPI_DC_CORR_VAL * 8 + 31 : IGPI_DC_CORR_VAL * 8 + 0]),

    .dsp_clk(dsp_clk),
    .dsp_rst(dsp_strem_rst),
    .dsp_data(dsp_data),
    .dsp_valid(dsp_valid),
    .dsp_ready(dsp_ready),
    .dsp_cfg_valid(dsp_dspchain_prg_valid),
    .dsp_cfg_data(dsp_dspchain_prg_data[7:0])
);

wire [31:0] adc_fifo_data;
wire        adc_fifo_valid;
wire        adc_fifo_ready;

axis_cc_fifo #(
    .WIDTH(16+16),
    .DEEP_BITS(3)
) dsc_to_usrclk_fifo (
    .rx_clk(dsp_clk),
    .rx_rst(dsp_strem_rst),

    .s_rx_tdata( dsp_data ),
    .s_rx_tvalid( dsp_valid ),
    .s_rx_tready(),

    .tx_clk(igp_clk),
    .tx_rst(igp_stream_rst),

    .m_tx_tdata(adc_fifo_data),
    .m_tx_tvalid(adc_fifo_valid),
    .m_tx_tready(adc_fifo_ready)
);



wire [14:0]  egpio_data_out;
wire [14:0]  egpio_data_oe;
wire [14:0]  egpio_data_in;
wire [14:0]  egpio_altf0_active;
wire [14:0]  egpio_alt0_in;
wire [14:0]  egpio_alt0_out;
wire [14:0]  egpio_alt0_out_oe;

wire         alt_i2c_scl_out_oe;
wire         alt_i2c_sda_out_oe;
wire         alt_i2c_sda_in;
wire         alt_i2c_scl_in;

wire         sel_uart_rxd;
wire         sel_uart_txd;

wire         pps_on_in;
wire         sysref_gen;

wire [31:0]  cntr_txclk_m_clk;
wire [31:0]  cntr_rxclk_f_clk;

// 7-series RQ/CC RC/CQ multiplexing
localparam C_DATA_WIDTH = 64;
localparam KEEP_WIDTH   = C_DATA_WIDTH / 32;
`DEFINE_AXIS_RVDLKU_PORT(s_axis_cc_t, C_DATA_WIDTH, KEEP_WIDTH, 1);
`DEFINE_AXIS_RVDLKU_PORT(s_axis_rq_t, C_DATA_WIDTH, KEEP_WIDTH, 1);
`DEFINE_AXIS_RVDLKU_PORT(m_axis_cq_t, C_DATA_WIDTH, KEEP_WIDTH, 1);
`DEFINE_AXIS_RVDLKU_PORT(m_axis_rc_t, C_DATA_WIDTH, KEEP_WIDTH, 1);

axis_mux #(.DATA_WIDTH(C_DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .PORTS(2)) axis_cc_rq_mux(
    .clk(igp_clk),
    .rst(igp_rst),

    `AXIS_RVDLK_PORT_CONN(m_axis_t, m_axis_tx_t),

    .sn_axis_tready({ s_axis_rq_tready, s_axis_cc_tready }),
    .sn_axis_tdata( { s_axis_rq_tdata,  s_axis_cc_tdata }),
    .sn_axis_tkeep( { s_axis_rq_tkeep,  s_axis_cc_tkeep }),
    .sn_axis_tlast( { s_axis_rq_tlast,  s_axis_cc_tlast }),
    .sn_axis_tvalid({ s_axis_rq_tvalid, s_axis_cc_tvalid })
);

wire cq_sel = (pcie_rx_bar_hit[0]); // != 7'h00);

assign m_axis_cq_tdata  = s_axis_rx_tdata;
assign m_axis_cq_tkeep  = s_axis_rx_tkeep;
assign m_axis_cq_tlast  = s_axis_rx_tlast;
assign m_axis_cq_tvalid = s_axis_rx_tvalid && cq_sel;

assign m_axis_rc_tdata  = s_axis_rx_tdata;
assign m_axis_rc_tkeep  = s_axis_rx_tkeep;
assign m_axis_rc_tlast  = s_axis_rx_tlast;
assign m_axis_rc_tvalid = s_axis_rx_tvalid && !cq_sel;

assign s_axis_rx_tready = (cq_sel) ? m_axis_cq_tready : m_axis_rc_tready;


assign cfg_interrupt_assert = 1'b0;
assign cfg_interrupt_stat   = 1'b0;
assign cfg_pciecap_interrupt_msgnum = 5'h1;

usdr_app_generic_us #(
    .I2C_SPEED(I2C_SPEED),
    .I2C_CLOCK_STRETCHING(I2C_CLOCK_STRETCHING),
    .HUL_BUS_SPEED(MASTER_BUS_SPEED),
    .PUL_BUS_SPEED(MASTER_BUS_SPEED),
    .IGPI_COUNT(IGPI_COUNT),
    .IGPO_COUNT(IGPO_COUNT),
    .COLLAPSE_RQ_EVENT(1'b0),
    .COLLAPSE_CC_REPLY(1'b1),
    .SPI_WIDTH(32'h00_00_00_10),
    .SPI_DIV(32'h80_80_80_80),
    .SPI_COUNT(1),
    .EGPIO_PRESENT(1'b1),
    .EGPIO_WIDTH(14),
    .UART_PRESENT(1'b1),
    .ULTRA_SCALE(0),
    .C_DATA_WIDTH(C_DATA_WIDTH),
    .NO_TX(0),
    .USB2_PRESENT(USB2_PRESENT),
    .USDR_PID(USDR_PID),
    .USE_EXT_DAC_CLK(1'b1),
    .TX_INITIAL_TS_COMP(0) // No compensation yet TBD
) gen_app (
    .hrst(igp_rst),
    .hclk(igp_clk),

    .s_axis_cc_tdata(s_axis_cc_tdata),
    .s_axis_cc_tkeep(s_axis_cc_tkeep),
    .s_axis_cc_tlast(s_axis_cc_tlast),
    .s_axis_cc_tvalid(s_axis_cc_tvalid),
    .s_axis_cc_tuser(s_axis_cc_tuser),
    .s_axis_cc_tready(s_axis_cc_tready),

    .s_axis_rq_tdata(s_axis_rq_tdata),
    .s_axis_rq_tkeep(s_axis_rq_tkeep),
    .s_axis_rq_tlast(s_axis_rq_tlast),
    .s_axis_rq_tvalid(s_axis_rq_tvalid),
    .s_axis_rq_tuser(s_axis_rq_tuser),
    .s_axis_rq_tready(s_axis_rq_tready),

    .m_axis_cq_tdata(m_axis_cq_tdata),
    .m_axis_cq_tlast(m_axis_cq_tlast),
    .m_axis_cq_tvalid(m_axis_cq_tvalid),
    .m_axis_cq_tuser(m_axis_cq_tuser),
    .m_axis_cq_tkeep(m_axis_cq_tkeep),
    .m_axis_cq_tready(m_axis_cq_tready),

    .m_axis_rc_tdata(m_axis_rc_tdata),
    .m_axis_rc_tlast(m_axis_rc_tlast),
    .m_axis_rc_tvalid(m_axis_rc_tvalid),
    .m_axis_rc_tuser(m_axis_rc_tuser),
    .m_axis_rc_tkeep(m_axis_rc_tkeep),
    .m_axis_rc_tready(m_axis_rc_tready),

    // PCIe Flow control
    .pcie7s_tx_buf_av(pcie7s_tx_buf_av),

    // pci interrupts
    .cfg_completer_id(cfg_completer_id),
    .cfg_max_payload_size(cfg_max_payload_size),
    .cfg_max_read_req_size(cfg_max_read_req_size),

/*
    .cfg_interrupt_msienable(cfg_interrupt_msienable),
    .cfg_interrupt_rdy(cfg_interrupt_rdy),
    .cfg_interrupt(cfg_interrupt),
    .cfg_interrupt_assert(cfg_interrupt_assert),
    .cfg_interrupt_di(cfg_interrupt_di),
    .legacy_interrupt_disabled(legacy_interrupt_disabled),
    .cfg_interrupt_mmenable(cfg_interrupt_mmenable),
    .cfg_pciecap_interrupt_msgnum(cfg_pciecap_interrupt_msgnum),
    .cfg_interrupt_stat(cfg_interrupt_stat),
*/
    .cfg_interrupt_msienable(cfg_interrupt_msienable),
    .cfg_interrupt_rdy(cfg_interrupt_rdy),
    .cfg_interrupt(cfg_interrupt),
    .cfg_interrupt_di(cfg_interrupt_di),
    .cfg_interrupt_mmenable(cfg_interrupt_mmenable),

    // streaming
    .adc_realigned({32'h0, adc_fifo_data}),
    .adc_fifo_valid(adc_fifo_valid),
    .rx_ready(adc_fifo_ready),
    .rx_streaming(rx_streaming),
    .adc_ch_enable(),

    .dac_clk_ext(txclk_m_clk), //todo remove me!!!!
    .dac_realigned(dac_fifo_data),
    .dac_fifo_valid(dac_fifo_valid),
    .tx_ready(dac_fifo_ready),
    .tx_streaming(txd_enable),

    .cfg_port_wvalid(),
    .cfg_port_wready(2'b11),
    .cfg_port_wdata(),

    .cfg_port_rvalid(2'b11),
    .cfg_port_rready(),
    .cfg_port_rdata({ cntr_txclk_m_clk,  cntr_rxclk_f_clk }),

    .prst(igp_rst),
    .pclk(igp_clk),

    // per
    .sda_in({alt_i2c_sda_in, sda_in}),
    .scl_in({alt_i2c_scl_in, scl_in}),
    .sda_out_eo({alt_i2c_sda_out_oe, sda_out_eo}),
    .scl_out_eo({alt_i2c_scl_out_oe, scl_out_eo}),

    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_sclk(spi_sclk),
    .spi_sen(spi_sen),

    .uart_rxd(sel_uart_rxd),
    .uart_txd(sel_uart_txd),

    .flash_dout(flash_dout),
    .flash_din(flash_din),
    .flash_ndrive(flash_ndrive),
    .flash_ncs(flash_ncs),
    .flash_sck(flash_sck),

    .egpio_data_out(egpio_data_out),
    .egpio_data_oe(egpio_data_oe),
    .egpio_data_in(egpio_data_in),
    .egpio_altf0_active(egpio_altf0_active),
    .egpio_alt0_in(egpio_alt0_in),
    .egpio_alt0_out(egpio_alt0_out),
    .egpio_alt0_out_oe(egpio_alt0_out_oe),

    .igpo_data(intgpo),
    .igpo_strobe(igpo_s),
    .igpi_data(intgpi),

    .pps_on_in(pps_on_in),
    .sysref_gen(sysref_gen),

    //usb2 ulphi & control
    .phy_rst(usb2_wrapper_rst),
    .phy_clk(usb2_phy_clk),
    .phy_do(usb2_phy_do),
    .phy_di(usb2_phy_di),
    .phy_doe(usb2_phy_doe),
    .phy_dir(usb2_phy_dir),
    .phy_nxt(usb2_phy_nxt),
    .phy_stp(usb2_phy_stp),

    .usb_bus_active(USB2_PRESENT && usb2_altrst_valid),
    .usb_bus_reset(usb_bus_reset)
);

reg [27:0] cntr_div;
reg        cntr_latch = 0;
always @(posedge igp_clk) begin
    cntr_div <= cntr_div + 1'b1;
    if (cntr_div == 124_999_9) begin //10 ms update cycle
        cntr_div   <= 0;
        cntr_latch <= ~cntr_latch;
    end
end

clock_measurement #(.POSEDGE_ONLY(1'b1), .GEN_WIDTH(4), .CNT_WIDTH(28)) cntr_rxclk_f_meas(
    .meas_clk_i(rxclk_f_clk),
    .ref_pulse_i(cntr_latch),
    .ref_rst_i(igp_rst),
    .meas_data_o(cntr_rxclk_f_clk)
);

clock_measurement #(.POSEDGE_ONLY(1'b1), .GEN_WIDTH(4), .CNT_WIDTH(28)) cntr_txclk_m_meas(
    .meas_clk_i(txclk_m_clk),
    .ref_pulse_i(cntr_latch),
    .ref_rst_i(igp_rst),
    .meas_data_o(cntr_txclk_m_clk)
);

// SDA -              GPLED0 / GPIO0
// SCL -              GPLED1 / GPIO1
// PPS                         GPIO2
// device_uart_rxd -- UART_TX  GPIO3
// device_uart_txd -- UART_RX  GPIO4
// SYSREF_GEN                  GPIO7

localparam GPIO_FUNC_SDA        = 0;
localparam GPIO_FUNC_SCL        = 1;
localparam GPIO_FUNC_PPS        = 2;
localparam GPIO_FUNC_UART_TX    = 3;
localparam GPIO_FUNC_UART_RX    = 4;
localparam GPIO_FUNC_TDD_SW     = 5;
localparam GPIO_FUNC_SYSREF_GEN = 7;


wire alt_i2c_pins   = egpio_altf0_active[GPIO_FUNC_SDA] && egpio_altf0_active[GPIO_FUNC_SCL];
wire alt_uart_tx    = egpio_altf0_active[GPIO_FUNC_UART_TX];
wire alt_uart_rx    = egpio_altf0_active[GPIO_FUNC_UART_RX];
wire alt_sysref_gen = egpio_altf0_active[GPIO_FUNC_SYSREF_GEN];


wire [1:0] agpio_led_oe        = alt_i2c_pins ? 2'b11 : { alt_i2c_scl_out_oe, alt_i2c_sda_out_oe };
wire [1:0] agpio_led_out       = alt_i2c_pins ? { ~txd_enable, ~rxd_enable } : 2'b00;

wire [1:0] agpio_i2c_oe        = alt_i2c_pins ? { alt_i2c_scl_out_oe, alt_i2c_sda_out_oe } : egpio_data_oe[1:0];
wire [1:0] agpio_i2c_out       = alt_i2c_pins ? 2'b00                                      : egpio_data_out[1:0];

assign user_gpio_oe            = { agpio_led_oe,  1'b0, egpio_data_oe[14:2],  agpio_i2c_oe  };
assign user_gpio_out           = { agpio_led_out, 1'b0, egpio_data_out[14:2], agpio_i2c_out };
assign egpio_data_in           = user_gpio_in[14:0];

assign alt_i2c_sda_in          = alt_i2c_pins ? egpio_alt0_in[GPIO_FUNC_SDA] : user_gpio_in[16];
assign alt_i2c_scl_in          = alt_i2c_pins ? egpio_alt0_in[GPIO_FUNC_SCL] : user_gpio_in[17];

assign pps_on_in               = egpio_alt0_in[GPIO_FUNC_PPS];
assign sel_uart_rxd            = alt_uart_rx ? egpio_alt0_in[GPIO_FUNC_UART_RX] : uart_rxd;

assign egpio_alt0_out[7:0]     = { sysref_gen, 3'b000, sel_uart_txd, 3'b000 };
assign egpio_alt0_out_oe[7:0]  = { alt_sysref_gen, 3'b000,  alt_uart_tx, 1'b0, alt_i2c_scl_out_oe, alt_i2c_sda_out_oe };

assign egpio_alt0_out[14:8]    = 7'h0;
assign egpio_alt0_out_oe[14:8] = 7'h0;


wire [1:0] clk_tr_sel = intgpo[8 * IGPO_CLKMEAS + 1 : 8 * IGPO_CLKMEAS + 0];

reg [27:0] ref_clk_counter = 0;
reg        ref_clk_trgigger = 0;
always @(posedge igp_clk) begin
  if (igp_rst) begin
    ref_clk_counter  <= 0;
    ref_clk_trgigger <= 0;
  end else begin
    ref_clk_counter <= ref_clk_counter + 1'b1;
    if (ref_clk_counter == MASTER_BUS_SPEED / 2 - 1) begin
        ref_clk_trgigger <= ~ref_clk_trgigger;
        ref_clk_counter <= 0;
    end
  end
end

wire pps_sig = (clk_tr_sel == 2) ? 0 : (clk_tr_sel == 1) ? ref_clk_trgigger : pps_on_in;

clock_measurement #(.POSEDGE_ONLY(1'b1), .GEN_WIDTH(4), .CNT_WIDTH(28)) clk_1pps_meas(
    .meas_clk_i(rxclk_f_clk),
    .ref_pulse_i(pps_sig),
    .ref_rst_i(igp_rst),
    .meas_data_o(intgpi[IGPI_CLK1PPS * 8 + 31:IGPI_CLK1PPS * 8 + 0])
);

assign uart_txd     = alt_uart_tx ? 1'b0 : sel_uart_txd;


////////////////////////////////////////////////////////////////////////////////
// USB2 PHY
assign usb2_wrapper_rst = (USB2_PRESENT && usb2_altrst_valid) ? usb2_altrst_logicrst : intgpo[IGPO_USB2_CFG * 8 + 0];
assign usb2_phy_nrst    = (USB2_PRESENT && usb2_altrst_valid) ? usb2_altrst_phynrst  : intgpo[IGPO_USB2_CFG * 8 + 1];


endmodule

