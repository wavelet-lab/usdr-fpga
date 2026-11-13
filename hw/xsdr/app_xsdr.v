// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module app_xsdr #(
    parameter MASTER_BUS_SPEED     = 125_000_000,
    parameter I2C_SPEED            = 400_000,
    parameter I2C_CLOCK_STRETCHING = 1'b0,
    parameter USB2_PRESENT         = 1'b1,
    parameter USDR_PID             = 16'h1011,
    parameter HAS_LMS8             = 1'b0,
    parameter EGPIO_ALT_MODE1      = 1'b0,
    parameter USE_EXT_DAC_CLK      = 1'b0,
    parameter [0:0] TX_EX_CORE     = 1'b0,
    parameter ULTRA_SCALE          = 1,
    parameter ULTRA_SCALE_CLI_TAG  = 0,
    parameter HAS_DUC_DDC          = 0,
    parameter C_DATA_WIDTH         = 64,
    parameter KEEP_WIDTH           = C_DATA_WIDTH / 32,
    parameter AXI4_CQ_TUSER_WIDTH  = 88,
    parameter AXI4_CC_TUSER_WIDTH  = 33,
    parameter AXI4_RQ_TUSER_WIDTH  = 62,
    parameter AXI4_RC_TUSER_WIDTH  = 75
)(
    // Global clock domains for the device
    input          gclk_dsp,
    input          gclk_master,

    input          reset,

    ////////////////////////////////////////////////////////////////////////////
    // PCIe interface (to PCIe hard core)
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

    ////////////////////////////////////////////////////////////////////////////
    // PCIe interface common
    input          cfg_interrupt_msienable,
    input          cfg_interrupt_rdy,
    output         cfg_interrupt,
    output [7:0]   cfg_interrupt_di,
    input  [2:0]   cfg_interrupt_mmenable,

    input [15:0]   cfg_completer_id,
    input [2:0]    cfg_max_payload_size,
    input [2:0]    cfg_max_read_req_size,

    // US+ transmit ordering/control
    // US transmit ordering control
    input [5:0]    pcieus_rq_seq_num0,
    input          pcieus_rq_seq_num_vld0,
    input [5:0]    pcieus_rq_seq_num1,
    input          pcieus_rq_seq_num_vld1,
    input [3:0]    pcie_tfc_nph_av,
    input [3:0]    pcie_tfc_npd_av,
    input [7:0]    pcie_rq_tag0,
    input          pcie_rq_tag_vld0,
    input [7:0]    pcie_rq_tag1,
    input          pcie_rq_tag_vld1,
    input [3:0]    pcie_rq_tag_av,

    input [7:0]    cfg_fc_ph,
    input [11:0]   cfg_fc_pd,

    // 7-Series control
    input [5:0]    pcie7s_tx_buf_av,

    ////////////////////////////////////////////////////////////////////////////
    // LMS7 input samples IQ demuxed

    input [31:0]   hwcfg_port,

    input              rxclk_f_clk,
    input              rxd_rst_gclk,
    input [12*4-1:0]   rxd_demux_data,
    input              rxd_demux_valid,
    output             rxd_enable,

    // LMS7 output samples IQ demuxed
    input              txclk_m_clk,
    input              txd_rst_gclk,
    output [12*4-1:0]  txd_demux_data,
    output             txd_demux_valid,
    output             txd_enable,

    // PHY port configuration
    output             cfg_port_clk,
    output             cfg_port_rst,

    output [1:0]       cfg_port_wvalid,
    input  [1:0]       cfg_port_wready,
    output [31:0]      cfg_port_wdata,

    input  [1:0]       cfg_port_rvalid,
    output [1:0]       cfg_port_rready,
    input  [32*2-1:0]  cfg_port_rdata,

    // I2C bus
    input  [1:0]   sda_in,
    input  [1:0]   scl_in,
    output [1:0]   sda_out_eo,
    output [1:0]   scl_out_eo,
    input          second_i2c_sec,

    // SPI bus
    output [0:0]   spi_mosi,
    input  [0:0]   spi_miso,
    output [0:0]   spi_sclk,
    output [0:0]   spi_sen,

    // UART
    input          uart_rxd,
    output         uart_txd,

    // QSPI bus
    output [3:0]   flash_dout,
    input  [3:0]   flash_din,
    output [3:0]   flash_ndrive,
    output         flash_ncs,
    output         flash_sck,

    // GPIOs
    output         igpo_lms_core_ldo_en,
    output         igpo_lms_rxen,
    output         igpo_lms_txen,
    output         igpo_lms_reset,
    output         igpo_lms_einit,

    output         igpo_en_tcxo,
    output         igpo_ext_clk,
    output         igpo_en_gps,
    output         igpo_iovcc_sel,
    output         igpo_en_smsigio,
    output         igpo_txsw,
    output [1:0]   igpo_rxsw,

    output         igpo_usb_clk_en,
    output         igpo_ldoen,
    output [7:0]   igpo_phydly,

    output         igpo_delay_reset,

    output         led_front,

    input          igpi_delay_ready,

    input [31:0]   igpi_usr_access2_data,

    output [17:0]  user_gpio_oe,
    output [17:0]  user_gpio_out,
    input  [17:0]  user_gpio_in,

    // LMS8 control
    output        lms8_sen,
    output        lms8_reset_n,

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

localparam DSP_WIDTH      = 16;
localparam CFG_WIDTH      = 8;
localparam STAGES         = 8;
localparam L_CHANS        = 2; //TODO: Add 4ch support

localparam L_RX_CHANS     = L_CHANS;
localparam L_TX_CHANS     = L_CHANS;

localparam [0:0] HAS_RX_DDC = HAS_DUC_DDC;
localparam [0:0] HAS_TX_DUC = HAS_DUC_DDC;


// TODO DOUBLE_I2C

wire   usb_bus_reset;

`include "axi_helpers.vh"

////////////////////////////////////////////////////////////////////////////////
// BOARD CONFIGURATION

`include "app_xsdr_const.vh"

wire [IGPO_COUNT * 8 - 1:0] intgpo;
wire [IGPO_COUNT - 1:0]     igpo_s;
wire [IGPI_COUNT * 8 - 1:0] intgpi;

assign intgpi[IGPI_USR_ACCESS2 * 8 + 31 : IGPI_USR_ACCESS2 * 8 + 0] = igpi_usr_access2_data;
assign intgpi[IGPI_HWID * 8 + 31        : IGPI_HWID * 8 + 0]        = hwcfg_port;

////////////////////////////////////////////////////////////////////////////////
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

assign igpo_lms_core_ldo_en = intgpo[IGPO_LMS_PWR * 8 + 0];
assign igpo_lms_rxen        = intgpo[IGPO_LMS_PWR * 8 + 1];
assign igpo_lms_txen        = intgpo[IGPO_LMS_PWR * 8 + 2];
assign igpo_lms_reset       = intgpo[IGPO_LMS_PWR * 8 + 3];
assign igpo_lms_einit       = intgpo[IGPO_LMS_PWR * 8 + 4];

assign igpo_delay_reset     = intgpo[IGPO_LMS_PWR * 8 + 7]; // Should be called before LMS

assign igpo_en_tcxo     = intgpo[IGPO_CLK_CFG * 8 + 0];
assign igpo_ext_clk     = intgpo[IGPO_CLK_CFG * 8 + 1];

assign igpo_en_gps      = intgpo[IGPO_GPS * 8 + 0];
assign igpo_iovcc_sel   = intgpo[IGPO_IOVCCSEL * 8 + 0];
assign igpo_en_smsigio  = intgpo[IGPO_SMSIGIO * 8 + 0];

assign igpo_txsw        = intgpo[IGPO_TXSW * 8 + 0];
assign igpo_rxsw[0]     = intgpo[IGPO_RXSW * 8 + 0];
assign igpo_rxsw[1]     = intgpo[IGPO_RXSW * 8 + 1];

assign led_front        = intgpo[IGPO_LED * 8 + 0];

assign igpo_usb_clk_en  = intgpo[IGPO_USB_CLK_EN * 8 + 0];
assign igpo_ldoen       = intgpo[IGPO_LDOLMS_EN * 8 + 0];
assign igpo_phydly      = intgpo[IGPO_PHYCTRL * 8 + 7:IGPO_PHYCTRL * 8 + 0];


assign lms8_reset_n     = intgpo[IGPO_LMS8_CTRL * 8 + 7];
wire lms8_switch        = HAS_LMS8 && intgpo[IGPO_LMS8_CTRL * 8 + 0];
wire spi_sen_raw;

assign lms8_sen         = lms8_switch ? spi_sen_raw : 1'b1;
assign spi_sen          = lms8_switch ? 1'b1 : spi_sen_raw;

// wire   igpdspcfg_rst    = intgpo[IGPO_DSP_RX_CTRL * 8 + 0];

////////////////////////////////////////////////////////////////////////////////
// DSP RX
wire        dsp_cfg_rst = intgpo[IGPO_DSP_RST * 8 + 0];

wire [63:0] adc_fifo_data;
wire        adc_fifo_valid;
wire        adc_fifo_ready;
wire        rxclk_stream_rst;

localparam ADC_RESET_DELAY = 8;
reg [ADC_RESET_DELAY:0] adc_reset_delay;
always @(posedge rxclk_f_clk) begin
    adc_reset_delay <= { adc_reset_delay[ADC_RESET_DELAY-1:0], rxclk_stream_rst };
end
assign rxd_enable = !adc_reset_delay[ADC_RESET_DELAY];

// RX cross clock chain  (reset -- source rx_streaming)
// rxclk_f_clk        =>       dsp_clk       =>        igp_clk
// rxclk_stream_rst   <=   dsp_strem_rst     <=        { !rx_streaming | dsp_cfg_rst }

localparam RX_DELAY = 7;

wire       rx_rst_dsp;
wire       rx_streaming;
reg        rx_str_rst;
assign igp_stream_rst = rx_str_rst;
always @(posedge igp_clk) begin
    if (dsp_cfg_rst) begin
        rx_str_rst <= 1'b1;
    end else begin
        rx_str_rst <= !rx_streaming || rxd_rst_gclk || rx_rst_dsp;
    end
end

reg [RX_DELAY:0] rx_str_delay;
always @(posedge rxclk_f_clk) begin
    rx_str_delay <= { rx_str_delay[RX_DELAY - 1:0], !rxclk_stream_rst};
    if (rxclk_stream_rst) begin
        rx_str_delay <= 0;
    end
end

synchronizer  #(.INIT(1), .ASYNC_RESET(0)) rxclk_stream_rst_sync  (.clk(rxclk_f_clk), .rst(0), .a_in(igp_stream_rst),  .s_out(rxclk_stream_rst));


generate
if (HAS_RX_DDC) begin

wire fir_adc_clk = rxclk_f_clk;
wire fir_adc_rst = rxclk_stream_rst;

wire [STAGES-1:0]   dsp_dspchain_prg_data;
wire                dsp_dspchain_prg_valid;
wire                igp_prg_strobe = igpo_s[IGPO_DSPCHAIN_RX_PRG];
wire [STAGES-1: 0]  igp_prg_data   = { intgpo[8 * IGPO_DSPCHAIN_RX_PRG + 7 : 8 * IGPO_DSPCHAIN_RX_PRG + 0] };
reg                 igp_dsp_cfg_rst;

always @(posedge igp_clk) begin
    if (igp_rst) begin
        igp_dsp_cfg_rst <= 1'b1;
    end else begin
        igp_dsp_cfg_rst <= intgpo[8 * IGPO_DSPCHAIN_RX_RST + 0 : 8 * IGPO_DSPCHAIN_RX_RST + 0];
    end
end

assign rx_rst_dsp = intgpo[8 * IGPO_DSPCHAIN_RX_RST + 1];

synchronizer  #(.INIT(1)) dsp_uload_rst_sync (.clk(fir_adc_clk),   .rst(1'b0), .a_in(igp_dsp_cfg_rst), .s_out(fir_adc_clk_dsp_uload_rst));

axis_cc_fifo #(
    .WIDTH(STAGES)
) igpdsprxcfg_fifo (
    .rx_clk(igp_clk),
    .rx_rst(igp_dsp_cfg_rst),

    .s_rx_tdata(igp_prg_data),
    .s_rx_tvalid(igp_prg_strobe),
    .s_rx_tready(),

    .tx_clk(fir_adc_clk),
    .tx_rst(fir_adc_clk_dsp_uload_rst),

    .m_tx_tdata(dsp_dspchain_prg_data),
    .m_tx_tvalid(dsp_dspchain_prg_valid),
    .m_tx_tready(1'b1)
);

/*
wire [2 * L_RX_CHANS * DSP_WIDTH - 1:0] adc_demux_tdata = {
    rxd_demux_data[47:36], 4'hf,
    rxd_demux_data[35:24], 4'hf,
    rxd_demux_data[23:12], 4'hf,
    rxd_demux_data[11:0], 4'hf
};
*/

wire [2 * L_RX_CHANS * DSP_WIDTH - 1:0] adc_demux_tdata = {
    rxd_demux_data[47:36], 4'h8,
    rxd_demux_data[35:24], 4'h8,
    rxd_demux_data[23:12], 4'h8,
    rxd_demux_data[11:0], 4'h8
};


wire                                    adc_demux_tvalid = rxd_demux_valid && rx_str_delay[RX_DELAY];

wire [2 * L_RX_CHANS * DSP_WIDTH - 1:0] dsp_data;
wire                                    dsp_valid;
wire                                    dsp_ready;

// FIR gearbox
reconf_dsp_fir #(
    .IN_WIDTH(DSP_WIDTH),
    .CHANS(2 * L_RX_CHANS),
    .OUT_WIDTH(DSP_WIDTH),
    .CFG_WIDTH(CFG_WIDTH),
    .STAGES(STAGES),
    .ULTRA_SCALE(ULTRA_SCALE)
) rx_reconf (
    .rst(fir_adc_rst),
    .clk(fir_adc_clk),

    .in_valid(adc_demux_tvalid),
    .in_data(adc_demux_tdata),
    .in_ready(adc_demux_tready),

    .out_valid(dsp_valid),
    .out_data(dsp_data),
    .out_ready(dsp_ready),

    .cfg_valid(dsp_dspchain_prg_valid),
    .cfg_data(dsp_dspchain_prg_data)
);

axis_cc_fifo #(
    .WIDTH(2 * L_RX_CHANS * DSP_WIDTH),
    .DEEP_BITS(4)
) rxd_to_usrclk_fifo (
    .rx_clk(fir_adc_clk),
    .rx_rst(fir_adc_rst),

    .s_rx_tdata(dsp_data),
    .s_rx_tvalid(dsp_valid),
    .s_rx_tready(dsp_ready),

    .tx_clk(igp_clk),
    .tx_rst(igp_stream_rst),

    .m_tx_tdata(adc_fifo_data),
    .m_tx_tvalid(adc_fifo_valid),
    .m_tx_tready(adc_fifo_ready)
);

`ifdef DEBUG_VIO_RST
vio_8 vio_8(
    .clk(igp_clk),
    .probe_in0(1'b0| igp_stream_rst),
    .probe_in1(1'b0| fir_adc_rst),
    .probe_in2(1'b0| adc_fifo_valid),
    .probe_in3(1'b0| dsp_valid),
    .probe_in4(1'b0| adc_demux_tvalid),
    .probe_in5(1'b0| fir_adc_clk_dsp_uload_rst),
    .probe_in6(1'b0| igp_dsp_cfg_rst),
    .probe_in7(1'b0| rx_streaming),
    .probe_in8(1'b0| rx_rst_dsp),
    .probe_in9(1'b0| rxd_rst_gclk)
);
`endif


end else begin

axis_cc_fifo #(
    .WIDTH(12 * 4),
    .DEEP_BITS(4)
) rxd_to_usrclk_fifo (
    .rx_clk(rxclk_f_clk),
    .rx_rst(rxclk_stream_rst),

    .s_rx_tdata( rxd_demux_data ),
    .s_rx_tvalid( rxd_demux_valid && rx_str_delay[RX_DELAY] ),
    .s_rx_tready(),

    .tx_clk(igp_clk),
    .tx_rst(igp_stream_rst),

    .m_tx_tdata( { adc_fifo_data[63:52], adc_fifo_data[47:36], adc_fifo_data[31:20], adc_fifo_data[15:4]} ),
    .m_tx_tvalid(adc_fifo_valid),
    .m_tx_tready(adc_fifo_ready)
);

assign adc_fifo_data[51:48] = 4'h8;
assign adc_fifo_data[35:32] = 4'h8;
assign adc_fifo_data[19:16] = 4'h8;
assign adc_fifo_data[3:0]   = 4'h8;

end
endgenerate

// During TX calibration TX clock can violate hold/setup requirements, so don't use it for internal logic



wire [14:0]  egpio_data_out;
wire [14:0]  egpio_data_oe;
wire [14:0]  egpio_data_in;
wire [14:0]  egpio_altf0_active;
wire [14:0]  egpio_altf1_active;

wire [14:0]  egpio_alt0_in;
wire [14:0]  egpio_alt0_out;
wire [14:0]  egpio_alt0_out_oe;
wire [14:0]  egpio_alt1_in;
wire [14:0]  egpio_alt1_out;
wire [14:0]  egpio_alt1_out_oe;

localparam ALT_I2C_CNT = (EGPIO_ALT_MODE1) ? 3 : 1;
wire [ALT_I2C_CNT - 1:0]       alt_i2c_scl_out_oe;
wire [ALT_I2C_CNT - 1:0]       alt_i2c_sda_out_oe;
wire [ALT_I2C_CNT - 1:0]       alt_i2c_sda_in;
wire [ALT_I2C_CNT - 1:0]       alt_i2c_scl_in;

wire         sel_uart_rxd;
wire         sel_uart_txd;

wire         pps_on_in;
wire         sysref_gen;

wire [31:0]  cntr_txclk_m_clk;
wire [31:0]  cntr_rxclk_f_clk;

wire        dac_fifo_ready;
wire        dac_fifo_valid;
wire [63:0] dac_fifo_data;

generate
if (USE_EXT_DAC_CLK) begin
    assign dac_fifo_ready = 1'b1;
    assign txd_demux_data = {
        dac_fifo_data[63:52],
        dac_fifo_data[47:36],
        dac_fifo_data[31:20],
        dac_fifo_data[15:4] };
    assign txd_demux_valid = dac_fifo_valid;
end else begin

    // 1024 cycles for any freq
    localparam RST_DLY_CNT_RST = 10;
    localparam RST_DLY_VALID   = 5;

    reg [RST_DLY_CNT_RST:0] tx_rst_cntr;
    wire                    tx_str_rst_done = tx_rst_cntr[RST_DLY_CNT_RST];

    reg [RST_DLY_VALID:0]   tx_str_rst_dly;

    wire                    tx_rst_dsp;
    wire                    tx_str_rst_sig = !txd_enable || txd_rst_gclk || tx_rst_dsp;
    reg                     tx_str_rst_sig_r;
    always @(posedge igp_clk) begin
        //if (dsp_cfg_rst) begin
        //    rx_str_rst <= 1'b1;
        //end else begin
        //end

        if (!tx_str_rst_done) begin
            tx_rst_cntr <= tx_rst_cntr + 1'b1;
        end
        tx_str_rst_sig_r <= tx_str_rst_sig;
        tx_str_rst_dly   <= { tx_str_rst_dly[RST_DLY_VALID - 1:0], !tx_str_rst_done };
        if (tx_str_rst_sig_r) begin
            tx_rst_cntr    <= 0;
            tx_str_rst_dly <= {(RST_DLY_VALID + 1){1'b1}};
        end
    end

    wire txclk_m_clk_rst;
    synchronizer  #(.INIT(1)) tx_str_rst_TXCLK   (.clk(txclk_m_clk), .rst(0), .a_in(tx_str_rst_sig_r),  .s_out(txclk_m_clk_rst));


    if (HAS_TX_DUC) begin
    wire                clk_dsp_tx_fe_rst;
    wire                igp_prg_tx_strobe = igpo_s[IGPO_DSPCHAIN_TX_PRG];
    wire [STAGES-1: 0]  igp_prg_tx_data   = { intgpo[8 * IGPO_DSPCHAIN_TX_PRG + 7 : 8 * IGPO_DSPCHAIN_TX_PRG + 0] };
    reg                 igp_dsp_tx_cfg_rst;
    reg                 igp_dsp_tx_fe_rst;
    
    always @(posedge igp_clk) begin
        if (igp_rst) begin
            igp_dsp_tx_cfg_rst <= 1'b1;
            igp_dsp_tx_fe_rst  <= 1'b1;
        end else begin
            igp_dsp_tx_cfg_rst <= intgpo[8 * IGPO_DSPCHAIN_TX_RST + 0];
            igp_dsp_tx_fe_rst  <= intgpo[8 * IGPO_DSPCHAIN_TX_RST + 2];
        end
    end

    assign tx_rst_dsp = intgpo[8 * IGPO_DSPCHAIN_TX_RST + 1];

    wire fir_dac_clk_dsp_uload_rst;
    synchronizer  #(.INIT(1)) dsp_tx_uload_rst_sync (.clk(txclk_m_clk), .rst(1'b0), .a_in(igp_dsp_tx_cfg_rst), .s_out(fir_dac_clk_dsp_uload_rst));
    synchronizer  #(.INIT(1)) dsp_tx_fe_rst_sync    (.clk(txclk_m_clk), .rst(1'b0), .a_in(igp_dsp_tx_fe_rst),  .s_out(clk_dsp_tx_fe_rst));

    wire [STAGES-1:0]   dsp_tx_dspchain_prg_data;
    wire                dsp_tx_dspchain_prg_valid;

    axis_cc_fifo #(
        .WIDTH(STAGES)
    ) igpdsptxcfg_fifo (
        .rx_clk(igp_clk),
        .rx_rst(igp_dsp_tx_cfg_rst),

        .s_rx_tdata(igp_prg_tx_data),
        .s_rx_tvalid(igp_prg_tx_strobe),
        .s_rx_tready(),

        .tx_clk(txclk_m_clk),
        .tx_rst(fir_dac_clk_dsp_uload_rst),

        .m_tx_tdata(dsp_tx_dspchain_prg_data),
        .m_tx_tvalid(dsp_tx_dspchain_prg_valid),
        .m_tx_tready(1'b1)
    );

    wire [DSP_WIDTH * 2 * L_TX_CHANS - 1:0] dac_dsp_in_data;
    wire                                    dac_dsp_in_valid;
    wire                                    dac_dsp_in_ready;

    wire [DSP_WIDTH * 2 * L_TX_CHANS - 1:0] dac_dsp_data;
    wire                                    dac_dsp_valid;
    wire                                    dac_dsp_ready = 1'b1;

    axis_cc_fifo #(
        .WIDTH(DSP_WIDTH * 2 * L_TX_CHANS),
        .DEEP_BITS(4)
    ) rxd_to_usrclk_fifo (
        .rx_clk(igp_clk),
        .rx_rst(tx_str_rst_dly[0]),

        .s_rx_tdata( dac_fifo_data ),
        .s_rx_tvalid( dac_fifo_valid && !tx_str_rst_dly[RST_DLY] ),
        .s_rx_tready( dac_fifo_ready ),

        .tx_clk(txclk_m_clk),
        .tx_rst(txclk_m_clk_rst),

        .m_tx_tdata(dac_dsp_in_data),
        .m_tx_tvalid(dac_dsp_in_valid),
        .m_tx_tready(dac_dsp_in_ready)
    );

    reconf_dsp_fir #(
        .IN_WIDTH(DSP_WIDTH),
        .CHANS(2 * L_TX_CHANS),
        .OUT_WIDTH(DSP_WIDTH),
        .CFG_WIDTH(CFG_WIDTH),
        .STAGES(STAGES),
        .BACKPRESSURE(1'b1),
        .HAS_C_PORT(1'b0),
        .EXTERNAL_OMUX(1'b0)
    ) tx_reconf (
        .rst(txclk_m_clk_rst),
        .clk(txclk_m_clk),

        .in_valid(dac_dsp_in_valid),
        .in_ready(dac_dsp_in_ready),
        .in_data(dac_dsp_in_data),

        .out_valid(dac_dsp_valid),
        .out_ready(dac_dsp_ready),
        .out_data(dac_dsp_data),

        .cfg_valid(dsp_tx_dspchain_prg_valid),
        .cfg_data(dsp_tx_dspchain_prg_data)
    );

    assign txd_demux_data  = {dac_dsp_data[63:52], dac_dsp_data[47:36], dac_dsp_data[31:20], dac_dsp_data[15:4] };
    assign txd_demux_valid = dac_dsp_valid;

    end else begin

    axis_cc_fifo #(
        .WIDTH(12 * 4),
        .DEEP_BITS(4)
    ) rxd_to_usrclk_fifo (
        .rx_clk(igp_clk),
        .rx_rst(tx_str_rst_dly[0]),

        .s_rx_tdata( {dac_fifo_data[63:52], dac_fifo_data[47:36], dac_fifo_data[31:20], dac_fifo_data[15:4] } ),
        .s_rx_tvalid( dac_fifo_valid && !tx_str_rst_dly[RST_DLY] ),
        .s_rx_tready( dac_fifo_ready ),

        .tx_clk(txclk_m_clk),
        .tx_rst(txclk_m_clk_rst),

        .m_tx_tdata(txd_demux_data),
        .m_tx_tvalid(txd_demux_valid),
        .m_tx_tready(1'b1)
    );

    end

end
endgenerate


usdr_app_generic_us #(
    .I2C_SPEED(I2C_SPEED),
    .I2C_CLOCK_STRETCHING(I2C_CLOCK_STRETCHING),
    .HUL_BUS_SPEED(MASTER_BUS_SPEED),
    .PUL_BUS_SPEED(MASTER_BUS_SPEED),
    .IGPI_COUNT(IGPI_COUNT),
    .IGPO_COUNT(IGPO_COUNT),
    .SPI_WIDTH(32'h00_00_00_20),
    .SPI_DIV(32'h80_80_80_80),
    .SPI_COUNT(1),
    .I2C_COUNT(EGPIO_ALT_MODE1 ? 2 : 1),
    .EGPIO_PRESENT(1'b1),
    .EGPIO_WIDTH(14),
    .EGPIO_ALT_MODE1(EGPIO_ALT_MODE1),
    .UART_PRESENT(1'b1),
    .ULTRA_SCALE(ULTRA_SCALE),
    .ULTRA_SCALE_CLI_TAG(ULTRA_SCALE_CLI_TAG),
    .C_DATA_WIDTH(C_DATA_WIDTH),
    .AXI4_CQ_TUSER_WIDTH(AXI4_CQ_TUSER_WIDTH),
    .AXI4_CC_TUSER_WIDTH(AXI4_CC_TUSER_WIDTH),
    .AXI4_RQ_TUSER_WIDTH(AXI4_RQ_TUSER_WIDTH),
    .AXI4_RC_TUSER_WIDTH(AXI4_RC_TUSER_WIDTH),
    .NO_TX(0),
    .USB2_PRESENT(USB2_PRESENT),
    .USDR_PID(USDR_PID),
    .USE_EXT_DAC_CLK(USE_EXT_DAC_CLK),
    .TX_EX_CORE(TX_EX_CORE),
    .COLLAPSE_CC_REPLY(1'b1),
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

    // US Control
    .pcieus_rq_seq_num0(pcieus_rq_seq_num0),
    .pcieus_rq_seq_num_vld0(pcieus_rq_seq_num_vld0),
    .pcieus_rq_seq_num1(pcieus_rq_seq_num1),
    .pcieus_rq_seq_num_vld1(pcieus_rq_seq_num_vld1),
    .pcie_tfc_npd_av(pcie_tfc_npd_av),
    .pcie_tfc_nph_av(pcie_tfc_nph_av),
    .pcie_rq_tag0(pcie_rq_tag0),
    .pcie_rq_tag1(pcie_rq_tag1),
    .pcie_rq_tag_av(pcie_rq_tag_av),
    .pcie_rq_tag_vld0(pcie_rq_tag_vld0),
    .pcie_rq_tag_vld1(pcie_rq_tag_vld1),

    .cfg_fc_ph(cfg_fc_ph),
    .cfg_fc_pd(cfg_fc_pd),

    // pci interrupts
    .cfg_completer_id(cfg_completer_id),
    .cfg_max_payload_size(cfg_max_payload_size),
    .cfg_max_read_req_size(cfg_max_read_req_size),

    .cfg_interrupt_msienable(cfg_interrupt_msienable),
    .cfg_interrupt_rdy(cfg_interrupt_rdy),
    .cfg_interrupt(cfg_interrupt),
    .cfg_interrupt_di(cfg_interrupt_di),
    .cfg_interrupt_mmenable(cfg_interrupt_mmenable),

    // streaming
    .adc_realigned(adc_fifo_data),
    .adc_fifo_valid(adc_fifo_valid),
    .rx_ready(adc_fifo_ready),
    .rx_streaming(rx_streaming),
    .adc_ch_enable(),

    .dac_clk_ext(txclk_m_clk),
    .dac_rst_ext(clk_dsp_tx_fe_rst),
    .dac_realigned(dac_fifo_data),
    .dac_fifo_valid(dac_fifo_valid),
    .tx_ready(dac_fifo_ready),
    .tx_streaming(txd_enable),

    .cfg_port_clk(cfg_port_clk),
    .cfg_port_rst(cfg_port_rst),

    .cfg_port_wvalid(cfg_port_wvalid),
    .cfg_port_wready(cfg_port_wready),
    .cfg_port_wdata(cfg_port_wdata),

    .cfg_port_rvalid(cfg_port_rvalid),
    .cfg_port_rready(cfg_port_rready),
    .cfg_port_rdata(cfg_port_rdata),

    .prst(igp_rst),
    .pclk(igp_clk),

    // per
    .sda_in({alt_i2c_sda_in, sda_in[0]}),
    .scl_in({alt_i2c_scl_in, scl_in[0]}),
    .sda_out_eo({alt_i2c_sda_out_oe, sda_out_eo[0]}),
    .scl_out_eo({alt_i2c_scl_out_oe, scl_out_eo[0]}),

    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_sclk(spi_sclk),
    .spi_sen(spi_sen_raw),

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
    .egpio_altf1_active(egpio_altf1_active),
    .egpio_alt0_in(egpio_alt0_in),
    .egpio_alt0_out(egpio_alt0_out),
    .egpio_alt0_out_oe(egpio_alt0_out_oe),
    .egpio_alt1_in(egpio_alt1_in),
    .egpio_alt1_out(egpio_alt1_out),
    .egpio_alt1_out_oe(egpio_alt1_out_oe),

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
    .ref_rst_i(rxd_rst_gclk/*igp_rst*/),
    .meas_data_o(cntr_rxclk_f_clk)
);

clock_measurement #(.POSEDGE_ONLY(1'b1), .GEN_WIDTH(4), .CNT_WIDTH(28)) cntr_txclk_m_meas(
    .meas_clk_i(txclk_m_clk),
    .ref_pulse_i(cntr_latch),
    .ref_rst_i(txd_rst_gclk/*igp_rst*/),
    .meas_data_o(cntr_txclk_m_clk)
);



// GPIO Routing
//|------------------------------------------------------|---------------|
//|M2       |PIN |    USDR / XSDR  |     Alt function0   | Alt function1 |
//|------------------------------------------------------|---------------|
//|VD       | 38 |      GPIO0      |  SDA                |
//|VD       | 40 |      GPIO1      |  SCL                | SDA4
//|VD       | 42 |      GPIO2      | 1PPS_IN | SYSREF_IN |
//|COEX3    | 44 |      GPIO3      |  UART_TX            |                  SYSREF_OUT
//|COEX2    | 46 |      GPIO4      |  UART_RX            |
//|COEX1    | 48 |      GPIO5      |                     | SCL4             EN_OSC
//|W_DIS2#  | 54 |      GPIO6      |                     | SDA3             EN_GPS
//|W_DIS1#  | 56 |      GPIO7      | SYSREF_OUT          |                 | Regenerated sync event aka sysref |
//|------------------------------------------------------|
//|UART_RXD | 22 | GPIO11 / GPIO11 |                     |
//|UART_TXD | 32 | GPIO12 / GPIO8  |                     |
//|UART_RTS | 34 | GPIO13 / GPIO9  |                     |
//|UART_CTS | 36 | GPIO14 / GPIO10 |                     |
//|UIM_SWP  | 66 | GPIO8  / NA     |                     |
//|UIM_PSNK | 68 | GPIO9  / NA     |                     |
//|UIM_PSRC | 70 | GPIO10 / NA     |                     |
//|------------------------------------------------------|
//|LED1#    | 6  | RX_ACTIVE                             | SCL3
//|LED2#    | 16 | TX_ACTIVE                             |
//|------------------------------------------------------|

// Alt0 config
localparam GPIO_FUNC_SDA        = 0;
localparam GPIO_FUNC_SCL        = 1;
localparam GPIO_FUNC_PPS        = 2;
localparam GPIO_FUNC_UART_TX    = 3;
localparam GPIO_FUNC_UART_RX    = 4;
localparam GPIO_FUNC_TDD_SW     = 5;
localparam GPIO_FUNC_SYSREF_GEN = 7;
// Alt1 config
// localparam GPIO_FUNC1_SCL3       = 16;
localparam GPIO_FUNC1_SDA3       = 6;
localparam GPIO_FUNC1_SDA4       = 1;
localparam GPIO_FUNC1_SCL4       = 5;


wire alt_i2c3_pins   = EGPIO_ALT_MODE1 && /*egpio_altf1_active[GPIO_FUNC1_SDA3] &&*/ egpio_altf1_active[GPIO_FUNC1_SDA3];
wire alt_i2c4_pins   = EGPIO_ALT_MODE1 && egpio_altf1_active[GPIO_FUNC1_SDA4] && egpio_altf1_active[GPIO_FUNC1_SCL4];


wire alt_i2c_pins   = egpio_altf0_active[GPIO_FUNC_SDA] && egpio_altf0_active[GPIO_FUNC_SCL];
wire alt_uart_tx    = egpio_altf0_active[GPIO_FUNC_UART_TX];
wire alt_uart_rx    = egpio_altf0_active[GPIO_FUNC_UART_RX];
wire alt_uart       = alt_uart_tx && alt_uart_rx;

wire alt_sysref_gen = egpio_altf0_active[GPIO_FUNC_SYSREF_GEN];


wire [1:0] agpio_led_oe        = alt_i2c3_pins ? {1'b0, alt_i2c_scl_out_oe[1]} : alt_i2c_pins ? 2'b11                        : { alt_i2c_scl_out_oe[0], alt_i2c_sda_out_oe[0] };
wire [1:0] agpio_led_out       = alt_i2c3_pins ? 2'b00                         : alt_i2c_pins ? { ~txd_enable, ~rxd_enable } : 2'b00;

wire [1:0] agpio_i2c_oe        = alt_i2c_pins ? { alt_i2c_scl_out_oe[0], alt_i2c_sda_out_oe[0] } : egpio_data_oe[1:0];
wire [1:0] agpio_i2c_out       = alt_i2c_pins ? 2'b00                                            : egpio_data_out[1:0];

assign user_gpio_oe            = { agpio_led_oe,  1'b0, egpio_data_oe[14:2],  agpio_i2c_oe  };
assign user_gpio_out           = { agpio_led_out, 1'b0, egpio_data_out[14:2], agpio_i2c_out };

//assign user_gpio_out           = { 2'b00,                       1'b0, egpio_data_out[14:7],  2'b00,  3'b000, 1'b0,  agpio_i2c_out[0]  };
////assign user_gpio_oe            = { 1'b0, alt_i2c_sda_out_oe[1], 1'b0, egpio_data_oe[14:7],   alt_i2c_scl_out_oe[1], alt_i2c_scl_out_oe[2],  1'b0, 1'b0, 1'b0, alt_i2c_sda_out_oe[2],  agpio_i2c_oe[0] };
//assign user_gpio_oe            = { 1'b0, alt_i2c_scl_out_oe[1], 1'b0, egpio_data_oe[14:7],   alt_i2c_sda_out_oe[1], alt_i2c_scl_out_oe[2],  1'b0, 1'b0, 1'b0, alt_i2c_sda_out_oe[2],  agpio_i2c_oe[0] };

assign egpio_data_in           = user_gpio_in[14:0];

//assign alt_i2c_sda_in         = (DOUBLE_I2C) ? sda_in[1] : (alt_i2c_pins ? egpio_alt0_in[0] : user_gpio_in[16]);
assign alt_i2c_sda_in[0]       = second_i2c_sec ? sda_in[1] : alt_i2c_pins ? egpio_alt0_in[GPIO_FUNC_SDA] : user_gpio_in[16];
assign alt_i2c_scl_in[0]       = second_i2c_sec ? scl_in[1] : alt_i2c_pins ? egpio_alt0_in[GPIO_FUNC_SCL] : user_gpio_in[17];

generate
if (EGPIO_ALT_MODE1) begin
assign alt_i2c_sda_in[1]       = alt_i2c3_pins ? user_gpio_in[GPIO_FUNC1_SDA3]  : 1'b1;
assign alt_i2c_scl_in[1]       = alt_i2c3_pins ? egpio_alt1_in[16]              : 1'b1;
//
assign alt_i2c_sda_in[2]       = alt_i2c4_pins ? egpio_alt1_in[GPIO_FUNC1_SDA4] : 1'b1;
assign alt_i2c_scl_in[2]       = alt_i2c4_pins ? egpio_alt1_in[GPIO_FUNC1_SCL4] : 1'b1;
//

//assign alt_i2c_sda_in[1]       = user_gpio_in[6];//user_gpio_in[16];
//assign alt_i2c_scl_in[1]       = user_gpio_in[16];//user_gpio_in[6];

//assign alt_i2c_sda_in[2]       = user_gpio_in[1];
//assign alt_i2c_scl_in[2]       = user_gpio_in[5];


assign egpio_alt1_out[7:0]     = 8'h00;
assign egpio_alt1_out_oe[7:0]  = { 1'b0, alt_i2c_sda_out_oe[1], alt_i2c_scl_out_oe[2], 1'b0, 1'b0, 1'b0, alt_i2c_sda_out_oe[2], 1'b0 };

assign egpio_alt1_out[14:8]    = 7'h0;
assign egpio_alt1_out_oe[14:8] = 7'h0;
end
endgenerate


assign pps_on_in               = egpio_alt0_in[GPIO_FUNC_PPS];
assign sel_uart_rxd            = alt_uart_rx ? egpio_alt0_in[GPIO_FUNC_UART_RX] : uart_rxd;

assign egpio_alt0_out[7:0]     = { sysref_gen, 3'b000, alt_uart ? sel_uart_txd : sysref_gen, 3'b000 };
assign egpio_alt0_out_oe[7:0]  = { alt_sysref_gen, 3'b000,  alt_uart_tx, 1'b0, alt_i2c_scl_out_oe[0], alt_i2c_sda_out_oe[0] };

assign egpio_alt0_out[14:8]    = 7'h0;
assign egpio_alt0_out_oe[14:8] = 7'h0;


//generate
//if (DOUBLE_I2C) begin
assign sda_out_eo[1] = second_i2c_sec ? alt_i2c_sda_out_oe[0] : 1'b0;
assign scl_out_eo[1] = second_i2c_sec ? alt_i2c_scl_out_oe[0] : 1'b0;
//end
//endgenerate

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
// assign usb2_wrapper_rst = (USB2_PRESENT && usb2_altrst_valid) ? usb2_altrst_logicrst : intgpo[IGPO_USB2_CFG * 8 + 0];
wire usb2_wrapper_rst_async = (USB2_PRESENT && usb2_altrst_valid) ? usb2_altrst_logicrst : intgpo[IGPO_USB2_CFG * 8 + 0];
// assign usb2_wrapper_rst = usb2_wrapper_rst_async;
synchronizer  #(.INIT(1)) usb2_wrapper_rst_sync (.clk(usb2_phy_clk), .rst(1'b0), .a_in(usb2_wrapper_rst_async), .s_out(usb2_wrapper_rst));


assign usb2_phy_nrst    = (USB2_PRESENT && usb2_altrst_valid) ? usb2_altrst_phynrst  : intgpo[IGPO_USB2_CFG * 8 + 1];


assign intgpi[IGPI_MEAS_RXCLK * 8 + 31:IGPI_MEAS_RXCLK * 8 + 0] = cntr_rxclk_f_clk;
assign intgpi[IGPI_MEAS_TXCLK * 8 + 31:IGPI_MEAS_TXCLK * 8 + 0] = cntr_txclk_m_clk;


endmodule






