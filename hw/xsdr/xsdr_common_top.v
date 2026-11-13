`default_nettype none

module xsdr_common_top #(
    parameter [7:0] DEVICE_ID       = 0,
    parameter [7:0] DEVICE_REVISION = 0,
    parameter       EGPIO_COUNT     = 12,
    parameter       HAS_LMS8        = 1'b0,
    parameter       EGPIO_ALT_MODE1 = 1'b0,
    parameter       HAS_RESET       = 1'b0,
    parameter       ULTRA_SCALE         = 1,
    parameter       ULTRA_SCALE_CLI_TAG = 0
)(
  output wire [1:0]  pci_exp_txp,
  output wire [1:0]  pci_exp_txn,
  input  wire [1:0]  pci_exp_rxp,
  input  wire [1:0]  pci_exp_rxn,

  input  wire        sys_clk_p,
  input  wire        sys_clk_n,

  // LMS SPI
  output wire        lms_sdio,
  output wire        lms_sclk,
  input  wire        lms_sdo,
  output wire        lms_sen,

  // LMS generic
  output wire        lms_reset_n,
  output wire        lms_rxen,
  output wire        lms_txen,
  output wire        lms_ldo_en,

  // LML1 - TX
  output wire        lml1_txnrx,
  input  wire        lml1_clk_m,
  output wire        lml1_clk_f,
  inout  wire        lml1_iqsel,
  inout  wire [11:0] lml1_diq,

  // LML2 - RX
  output wire        lml2_txnrx,
  input  wire        lml2_clk_m,
  output wire        lml2_clk_f,
  inout  wire        lml2_iqsel,
  inout  wire [11:0] lml2_diq,

  // USB2 Phy
  inout  wire [7:0]  usb_d,
  input  wire        usb_clk,
  output wire        usb_nrst,
  input  wire        usb_dir,
  inout  wire        usb_stp,
  input  wire        usb_nxt,

  // FLASH
  inout  wire [3:0]  flash_d,
  output wire        flash_fcs_b,

  // EXTERNAL GPIO
  inout wire [EGPIO_COUNT - 1:0]  gpio,

  // I2C_BUS2 (LP8758)
  inout wire          sda2,
  inout wire          scl2,

  /////////////////////////////////////////
  // Commonn control (externally buffered)
  output wire         igpo_txsw,
  output wire [1:0]   igpo_rxsw,

  output wire         led_internal,

  input  wire         uart_txd,
  output wire         uart_rxd,

  output wire         usb_clk_en,
  output wire         igpo_ext_clk,

  // XSDR specific controls
  output wire         xsdr_ldoen,

  // XTRX specific controls
  output wire         xtrx_en_gps,
  output wire         xtrx_iovcc_sel,
  output wire         xtrx_en_smsigio,
  output wire         xtrx_i2c_sel,

  // Extra I2C bus
  input  wire         sdae_in,
  input  wire         scle_in,
  output wire         sdae_out_eo,
  output wire         scle_out_eo,
  inout  wire         second_i2c_sec,

  // Extra GPIOs
  output wire  [1:0]  gpled_gpio_out,
  output wire  [1:0]  gpled_gpio_oe,
  input  wire  [1:0]  gpled_gpio_in,

  // Extrenal clock (syncronization)
  input  wire         external_clock,
  input  wire         sys_rst_n_ext,

  // Extra bring up  logic
  output wire         excfg_clk,
  output wire         excfg_rst,
  input  wire         excfg_done,

  // LMS8 control
  output wire         lms8_sen,
  output wire         lms8_reset_n
);

generate
if (DEVICE_ID == 0) begin
    // Make compilation fail if DEVICE_ID is forgotten
    assign pci_exp_rxp = 0;
end
endgenerate


wire [7:0] hwcfg_devid = DEVICE_ID;
wire [7:0] hwcfg_revid = DEVICE_REVISION;

////////////////////////////////////////////////////////////////////////////////
// BOARD CONFIGURATION

localparam PCIE_BITS        = 2;
localparam I2C_SPEED        = 100_000;
localparam MASTER_BUS_SPEED = 125_000_000;

localparam [0:0] RX_AUTO_PS       = 1'b1;
localparam [0:0] SINGLE_MMCM      = 1'b1;
localparam [0:0] MODE_TX_MMCM     = 1'b1;
localparam [0:0] MODE_RX_MMCM     = 1'b0;
localparam [0:0] RX_LML2          = 1'b0; // 0: RX=LML1, TX=LML2; 1: RX=LML2, TX=LML1

localparam [3:0] PHY_CORE_VER     = { 3'h0, SINGLE_MMCM };
localparam [7:0] PHY_CONFIG_ID    = { 1'b1, RX_LML2, MODE_TX_MMCM, MODE_RX_MMCM, PHY_CORE_VER };

wire       idelay200mhz_clk;

wire      user_clk_pcie;
wire      sys_clk;

wire       igpo_lms_core_ldo_en;
wire       igpo_lms_rxen;
wire       igpo_lms_txen;
wire       igpo_lms_reset;

//////////////////////////////
wire       igpo_en_gps;
wire       igpo_iovcc_sel;
wire       igpo_en_smsigio;
//////////////////////////////
wire       igpo_usb_clk_en;
wire       igpo_ldoen;
//////////////////////////////


wire      spi_mosi;
wire      spi_sclk;
wire      spi_sen;
wire      spi_miso;

wire [1:0] sda_in;
wire [1:0] scl_in;

wire [1:0] sda_out_eo;
wire [1:0] scl_out_eo;


////////////////////////////////////////////////////////////////////////////
// Autonomous bring up USB2 wires
reg        brng_usb_clk_en_r = 1'b0;

OBUF lms_gpwrdwn_obuf( .O(lms_ldo_en),     .I(igpo_lms_core_ldo_en));
OBUF lms_rxen_obuf(    .O(lms_rxen),       .I(igpo_lms_rxen));
OBUF lms_txen_obuf(    .O(lms_txen),       .I(igpo_lms_txen));
OBUF lms_reset_obuf(   .O(lms_reset_n),    .I(igpo_lms_reset));


assign usb_clk_en      = igpo_usb_clk_en | brng_usb_clk_en_r;
assign xsdr_ldoen      = igpo_ldoen;
assign xtrx_en_gps     = igpo_en_gps;
assign xtrx_iovcc_sel  = igpo_iovcc_sel;
assign xtrx_en_smsigio = igpo_en_smsigio;

// LMS SPI BUS
OBUF lms_spi_mosi_obuf( .O(lms_sdio),     .I(spi_mosi));
OBUF lms_spi_sck_obuf(  .O(lms_sclk),     .I(spi_sclk));
OBUF lms_spi_sen_obuf(  .O(lms_sen),      .I(spi_sen));
IBUF lms_spi_miso_ibuf( .O(spi_miso),     .I(lms_sdo));

//I2C bus
IOBUF sda0_buf ( .IO(sda2), .I(1'b0), .T(~sda_out_eo[0]), .O(sda_in[0]));
IOBUF scl0_buf ( .IO(scl2), .I(1'b0), .T(~scl_out_eo[0]), .O(scl_in[0]));

assign sda_in[1]   = sdae_in;
assign scl_in[1]   = scle_in;
assign sdae_out_eo = sda_out_eo[1];
assign scle_out_eo = scl_out_eo[1];

IBUFDS_GTE2 refclk_ibuf (.O(sys_clk), .ODIV2(), .I(sys_clk_p), .CEB(1'b0), .IB(sys_clk_n));


////////////////////////////////////////////////////////////////////////////////
// FLASH QSPI
wire [3:0] flash_dout;
wire [3:0] flash_din;
wire [3:0] flash_ndrive;
wire       flash_ncs;

genvar i;
generate
for (i = 0; i < 4; i=i+1) begin: gen_d
  IOBUF flash_buf_d (
    .IO(flash_d[i]),
    .I(flash_dout[i]),
    .T(flash_ndrive[i]),
    .O(flash_din[i])
  );
end
endgenerate

OBUF flash_buf_cs ( .O(flash_fcs_b), .I(flash_ncs) );


///////////////////////////////////////////////////////////////////////////////
// LMS PORT1 -- in
// LMS PORT2 -- out

wire             cfg_port_clk;
wire             cfg_port_rst;

wire [1:0]       cfg_port_wvalid;
wire [1:0]       cfg_port_wready;
wire [31:0]      cfg_port_wdata;

wire [1:0]       cfg_port_rvalid;
wire [1:0]       cfg_port_rready;
wire [31:0]      cfg_port_rdata_p0;
wire [31:0]      cfg_port_rdata_p1;

wire        lmle_bufio;
wire        lmle_bufio_b;
wire        lmle_bufr;
wire        lmle_fclk;
wire        lmle_usrclk;

wire        lmle_psen;
wire        lmle_psclk;
wire        lmle_psdone;

assign lml1_txnrx = 1'b0;

wire        ddr_frame;
wire [11:0] ddr_data;

IBUF phy_frame_buf(
      .I(lml1_iqsel),
      .O(ddr_frame)
);

genvar k;
generate
  for (k = 0; k < 12; k = k + 1) begin: lml_bufs
    IBUF phy_data_buf_i(
      .I(lml1_diq[k]),
      .O(ddr_data[k])
    );
  end
endgenerate

//wire [11:0]      rx_data_s0_p0;
//wire [11:0]      rx_data_s1_p0;
//wire [11:0]      rx_data_s2_p0;
//wire [11:0]      rx_data_s3_p0;

wire             rxd_demux_clk;
wire             rxd_demux_valid;
wire [47:0]      rxd_demux_data;
// wire [47:0]      rxd_demux_data  = { rx_data_s3_p0, rx_data_s2_p0, rx_data_s1_p0, rx_data_s0_p0 };

ddr_reciver #(
    .DATA_WIDTH(12),
    .EXTERNAL_CLOCKS(SINGLE_MMCM),
    .DELAY_REFCLK(200),
    .WITH_FIFO(0),
    .NO_CLK_DELAY(1),
    .ALT_PS(RX_AUTO_PS)
) ddr_r (
    // DDR interface
    .ddr_clk_ref(lml1_clk_m),
    .ddr_clk_fwd(lml1_clk_f),

    .ddr_frame(ddr_frame),
    .ddr_data(ddr_data),

    // Out logic interface
    .l_clk(rxd_demux_clk),
    .l_valid(rxd_demux_valid),
    //.l_data({rx_data_s3_p0, rx_data_s2_p0, rx_data_s1_p0, rx_data_s0_p0 }),
    .l_data(rxd_demux_data),

    .extclk_buf_io(lmle_bufio),
    .extclk_buf_io_b(lmle_bufio_b),
    .extclk_bufr(lmle_bufr),
    .extclk_fwd(lmle_fclk),
    .extclk_usr(lmle_usrclk),
    
    .extclk_psen(lmle_psen),
    .extclk_psclk(lmle_psclk),
    .extclk_psdone(lmle_psdone),

    .cfg_clk(cfg_port_clk),
    .cfg_rst(cfg_port_rst),

    .cfg_wvalid(cfg_port_wvalid[0]),
    .cfg_wready(cfg_port_wready[0]),
    .cfg_wdata(cfg_port_wdata),

    .cfg_rvalid(cfg_port_rvalid[0]),
    .cfg_rready(cfg_port_rready[0]),
    .cfg_rdata(cfg_port_rdata_p0)
);


assign cfg_port_rvalid[1] = 1'b1;
assign cfg_port_wready[1] = 1'b1;

wire         lms7_tx_en;
wire  [47:0] txd_demux_data;
wire         txd_demux_valid;
wire         txd_demux_clk;

localparam         LML7_LFSR_POLY       = 15'h6000;
localparam         LML7_LFSR_WIDTH      = 15;
localparam         DATA_WIDTH           = 12;
localparam [14:0]  INIT_LFSR_A          = 15'h7a3;
localparam [14:0]  INIT_LFSR_B          = 15'h06e;

wire [DATA_WIDTH-1:0] gen_ai;
wire [DATA_WIDTH-1:0] gen_bi;
wire                  gen_lfsr_rst;
wire                  iqalgn_gen_rst;
wire                  lfsr_gen_en;
wire                  iqalgn_gen_en;
reg                   iqalgn_gen_phase;

synchronizer #(.INIT(1)) phy_tx_lfsr_gen_clkdiv_sync (.clk(txd_demux_clk), .rst(~lfsr_gen_en),   .a_in(1'b0), .s_out(gen_lfsr_rst));
synchronizer #(.INIT(1)) phy_iqalgn_gen_en_sync      (.clk(txd_demux_clk), .rst(~iqalgn_gen_en), .a_in(1'b0), .s_out(iqalgn_gen_rst));

lfsr_generator #(
    .DATA_WIDTH(DATA_WIDTH),
    .LFSR_POLY(LML7_LFSR_POLY),
    .LFSR_LEN(LML7_LFSR_WIDTH)
) gen_a (
    .clk(txd_demux_clk),
    .rst(gen_lfsr_rst),
    .init_i(INIT_LFSR_A),
    .data_o(gen_ai)
);
lfsr_generator #(
    .DATA_WIDTH(DATA_WIDTH),
    .LFSR_POLY(LML7_LFSR_POLY),
    .LFSR_LEN(LML7_LFSR_WIDTH)
) gen_b (
    .clk(txd_demux_clk),
    .rst(gen_lfsr_rst),
    .init_i(INIT_LFSR_B),
    .data_o(gen_bi)
);

localparam [47:0] PHASE_IQAB_ALIGN_0 = 48'h5A5_969_BBE_441; 
localparam [47:0] PHASE_IQAB_ALIGN_1 = 48'hF0F_356_C42_676;

reg [47:0] txd_demux_data_p;
reg        txd_demux_valid_p;
always @(posedge txd_demux_clk) begin
    if (iqalgn_gen_rst) begin
        iqalgn_gen_phase <= 0;
    end else begin
        iqalgn_gen_phase <= ~iqalgn_gen_phase;
    end

    txd_demux_data_p  <= gen_lfsr_rst ? txd_demux_data  : !iqalgn_gen_rst ? (iqalgn_gen_phase ? PHASE_IQAB_ALIGN_1 : PHASE_IQAB_ALIGN_0) :  { ~gen_bi, gen_bi, ~gen_ai, gen_ai };
    txd_demux_valid_p <= gen_lfsr_rst ? txd_demux_valid : 1'b1;
end

xlnx_lml_phy #(
    .MODE_MMCM(MODE_TX_MMCM),
    .LMLE_CLK(SINGLE_MMCM),
    .ALT_PS(RX_AUTO_PS)
) phy (
  // LMS7 LML port
  .lms_i_txnrx(lml2_txnrx),
  .lms_io_iqsel(lml2_iqsel),
  .lms_o_mclk(lml2_clk_m),
  .lms_i_fclk(lml2_clk_f),
  .lms_io_diq(lml2_diq),

  // Control
  .phy_cfg_clk(cfg_port_clk),
  .phy_cfg_rst(cfg_port_rst),
  .phy_cfg_addr(cfg_port_wdata[31:24]),
  .phy_cfg_data(cfg_port_wdata[23:0]),
  .phy_cfg_strobe(cfg_port_wvalid[1] && cfg_port_wready[1]),
  .phy_cfg_status(cfg_port_rdata_p1),
  .phy_cfg_busy(), // TODO 

  .lmle_lms_o_mclk(lml1_clk_m),
  .lmle_bufio(lmle_bufio),
  .lmle_bufio_b(lmle_bufio_b),
  .lmle_bufr(lmle_bufr),
  .lmle_fclk(lmle_fclk),
  .lmle_usrclk(lmle_usrclk),

  .lmle_psen(lmle_psen),
  .lmle_psclk(lmle_psclk),
  .lmle_psdone(lmle_psdone),
  
  .lfsr_gen_en(lfsr_gen_en),
  .iqalgn_gen_en(iqalgn_gen_en),

  // DATA Out
  .tx_data_clk(txd_demux_clk),
  .tx_data_valid(txd_demux_valid_p),
  .tx_data_ready(),
  .tx_data_s0(txd_demux_data_p[11:0]),
  .tx_data_s1(txd_demux_data_p[23:12]),
  .tx_data_s2(txd_demux_data_p[35:24]),
  .tx_data_s3(txd_demux_data_p[47:36])
);

wire [7:0] igp_phydly;
wire       rxd_enable;

////////////////////////////////////////////////////////////////////////////////
// USB2 PHY ULPI

wire        phy_clk;
IBUF   phy_clk_ibuf (.O(phy_clk), .I(usb_clk));

wire        phy_nrst;
wire        phy_dir;
wire        phy_nxt;
wire        phy_stp;

OBUF   phy_nrst_obuf (.O(usb_nrst), .I(phy_nrst));
IOBUF  phy_stpio_buf(
  .IO(usb_stp),
  .I(phy_stp),
  .T(~phy_nrst),
  .O()
);
IBUF   phy_dir_ibuf (.O(phy_dir), .I(usb_dir));
IBUF   phy_nxt_ibuf (.O(phy_nxt), .I(usb_nxt));

wire [7:0] phy_do;
wire [7:0] phy_di;
wire       phy_doe;

genvar j;
generate
begin
for (j = 0; j < 8; j = j + 1) begin: usb
  IOBUF phy_d_buf(
    .IO(usb_d[j]),
    .I(phy_do[j]),
    .T(~phy_doe),
    .O(phy_di[j])
  );
end
end
endgenerate

////////////////////////////////////////////////////////////////////
// GPIO
wire [17:0] user_gpio_oe;
wire [17:0] user_gpio_out;
wire [17:0] user_gpio_in;

generate
begin
for (j = 0; j < EGPIO_COUNT; j = j + 1) begin: gpiobufs
  IOBUF gpio_buf(
    .IO(gpio[j]),
    .I(user_gpio_out[j]),
    .T(~user_gpio_oe[j]),
    .O(user_gpio_in[j])
  );
end
for (j = EGPIO_COUNT; j < 16; j = j + 1) begin
    assign user_gpio_in[j] = 1'b0;
end

assign gpled_gpio_out[1:0] = user_gpio_out[17:16];
assign gpled_gpio_oe[1:0]  = user_gpio_oe[17:16];
assign user_gpio_in[17:16] = gpled_gpio_in[1:0];
end
endgenerate

////////////////////////////////////////////////////////////////////////////////
wire user_reset;
wire user_lnk_up;

reg user_reset_q;
reg user_lnk_up_q;

wire flash_sck;
wire cfg_clk;
wire cfg_mclk;

(* keep = "TRUE" *)
STARTUPE2 #(
    .PROG_USR("FALSE")
) STARTUPE2_inst (
    .CFGCLK( cfg_clk ),
    .CFGMCLK( cfg_mclk ),
    .EOS(/* NC */),
    .PREQ(/* NC */),
    .CLK(1'b0),
    .GSR(1'b0),
    .GTS(1'b0),
    .KEYCLEARB(1'b0),
    .PACK(1'b0),
    .USRCCLKO( flash_sck ),
    .USRCCLKTS(1'b0),
    .USRDONEO(1'b1),  // Set DONE to 1
    .USRDONETS(1'b0)
);

wire [31:0] usr_access2_data;
wire        usr_access2_valid;

(* keep = "TRUE" *)
USR_ACCESSE2 USR_ACCESSE2_inst(
   .CFGCLK(),
   .DATA(usr_access2_data),
   .DATAVALID(usr_access2_valid)
);


wire pipe_mmcm_rst_n;
wire startup_mode_pcie;
wire brng_usb_user_reset;
wire usb2_fsm_done;

wire brng_usb_logic_reset;
wire brng_phy_nrst;

wire init_usb_clk_done;
wire pipe_mmcm_lock_out;

wire cfg_reset;

synchronizer  #(.INIT(1), .ASYNC_RESET(1)) usb_bus_igp_reg  (
    .clk(cfg_mclk),
    .rst(!sys_rst_n_ext),
    .a_in(1'b0),
    .s_out(cfg_reset)
);

xlnx_startup_mmcm #(.HAS_RESET(HAS_RESET) /*, .PCIE_INIT_ONLY(1'b1)*/) sfsm (
    .cfg_reset(cfg_reset),
    .cfg_mclk(cfg_mclk),
    .startup_mode_pcie(startup_mode_pcie),

    .pipe_mmcm_rst_n(pipe_mmcm_rst_n),

    .brng_usb_logic_reset(brng_usb_logic_reset),
    .brng_phy_nrst(brng_phy_nrst),

    .brng_usb_user_reset(brng_usb_user_reset),
    .usb2_fsm_done(usb2_fsm_done),

    .usb2_clk_ready(init_usb_clk_done),
    .pipe_mmcm_lock_out(pipe_mmcm_lock_out),
    .debug_state()
);

wire pci_mmcm_sel = (startup_mode_pcie) ? 1'b0 : 1'b1;
wire sys_rst_n_c = (startup_mode_pcie)  ? sys_rst_n_ext : 1'b0;

always @(posedge user_clk_pcie) begin
    user_reset_q  <= (startup_mode_pcie) ? user_reset : brng_usb_user_reset;
    user_lnk_up_q <= user_lnk_up;
end

wire cfg_startup_mode_usb = (!startup_mode_pcie);

always @(posedge cfg_mclk) begin
  brng_usb_clk_en_r <= cfg_startup_mode_usb;
end

// External CFG logic
assign init_usb_clk_done = excfg_done;
assign excfg_clk         = cfg_mclk;
assign excfg_rst         = 0; //startup_mode_pcie;

// Tx
wire                                        s_axis_tx_tready;
wire [3:0]                                  s_axis_tx_tuser;
wire [63:0]                                 s_axis_tx_tdata;
wire [7:0]                                  s_axis_tx_tkeep_8;
wire                                        s_axis_tx_tlast;
wire                                        s_axis_tx_tvalid;

// Rx
wire [63:0]                                 m_axis_rx_tdata;
wire [7:0]                                  m_axis_rx_tkeep_8;
wire                                        m_axis_rx_tlast;
wire                                        m_axis_rx_tvalid;
wire                                        m_axis_rx_tready;
wire  [21:0]                                m_axis_rx_tuser;

wire                                        tx_cfg_gnt;
wire                                        rx_np_ok;
wire                                        rx_np_req;
wire                                        cfg_turnoff_ok;
wire                                        cfg_trn_pending;
wire                                        cfg_pm_halt_aspm_l0s;
wire                                        cfg_pm_halt_aspm_l1;
wire                                        cfg_pm_force_state_en;
wire   [1:0]                                cfg_pm_force_state;
wire                                        cfg_pm_wake;
wire  [63:0]                                cfg_dsn;

wire                                        cfg_interrupt;
wire                                        cfg_interrupt_assert;
wire   [7:0]                                cfg_interrupt_di;
wire                                        cfg_interrupt_stat;
wire   [4:0]                                cfg_pciecap_interrupt_msgnum;
wire   [2:0]                                cfg_interrupt_mmenable;
wire                                        cfg_interrupt_rdy;
wire                                        cfg_interrupt_msienable;

wire                                        cfg_to_turnoff;
wire   [7:0]                                cfg_bus_number;
wire   [4:0]                                cfg_device_number;
wire   [2:0]                                cfg_function_number;


assign tx_cfg_gnt = 1'b1;                        // Always allow transmission of Config traffic within block
assign rx_np_ok = 1'b1;                          // Allow Reception of Non-posted Traffic
assign rx_np_req = 1'b1;                         // Always request Non-posted Traffic if available
assign cfg_pm_wake = 1'b0;                       // Never direct the core to send a PM_PME Message
assign cfg_trn_pending = 1'b0;                   // Never set the transaction pending bit in the Device Status Register
assign cfg_pm_halt_aspm_l0s = 1'b0;              // Allow entry into L0s
assign cfg_pm_halt_aspm_l1 = 1'b0;               // Allow entry into L1
assign cfg_pm_force_state_en  = 1'b0;            // Do not qualify cfg_pm_force_state
assign cfg_pm_force_state  = 2'b00;              // Do not move force core into specific PM state
assign cfg_dsn = 64'h12345678;                   // Assign the input DSN (Device Serial Number)
assign s_axis_tx_tuser[0] = 1'b0;                // Unused for V6
assign s_axis_tx_tuser[1] = 1'b0;                // Error forward packet
assign s_axis_tx_tuser[2] = 1'b0;                // Stream packet
assign s_axis_tx_tuser[3] = 1'b0;                // Transmit source discontinue

assign cfg_turnoff_ok = cfg_to_turnoff;

wire [15:0] cfg_pcie_ctrl_reg; //Device Control Register (Offset 08h)

wire [2:0] cfg_max_read_req_size = cfg_pcie_ctrl_reg[14:12];
wire       cfg_no_snoop_enable   = cfg_pcie_ctrl_reg[11];
wire       cfg_ext_tag_enabled   = cfg_pcie_ctrl_reg[8];
wire [2:0] cfg_max_payload_size  = cfg_pcie_ctrl_reg[7:5];
wire       cfg_relax_ord_enabled = cfg_pcie_ctrl_reg[4];

wire [15:0] cfg_completer_id      = { cfg_bus_number, cfg_device_number, cfg_function_number };
wire [6:0]  rx_bar_hit            = m_axis_rx_tuser[8:2];
wire        rx_ecrc_err           = m_axis_rx_tuser[0]; // Receive ECRC Error: Indicates the current packet has an ECRC error. Asserted at the packet EOF.
wire        rx_err_fwd            = m_axis_rx_tuser[1]; // When asserted, marks the packet in progress as error-poisoned. Asserted by the core for the entire length of the packet.

wire [15:0] cfg_command;


wire                 pipe_pclk_out;
wire                 pipe_txoutclk_in;
wire [PCIE_BITS-1:0] pipe_pclk_sel_in;

wire                 pipe_userclk_out;
wire                 pipe_dclk_out;

wire                 dsp_clk;

xilinx_pci_mmcm #(
    .PCIE_LANE(PCIE_BITS),
    .PCIE_GEN1_MODE(0),
    .PCIE_USERCLK_FREQ(MASTER_BUS_SPEED == 250_000_000 ? 4 : 3),
    .PCIE_ALT_CLOCK(1'b1)
) xilinx_pci_mmcm (
    .rst_n_i(pipe_mmcm_rst_n),
    .refclk_i({ phy_clk, pipe_txoutclk_in }),
    .refclk_sel_i( pci_mmcm_sel ),
    .pclk_sel_i(pipe_pclk_sel_in),
    .pipeclk_en_i(~pci_mmcm_sel),

    .pclk_o(pipe_pclk_out),
    .dclk_o(pipe_dclk_out),
    .userclk_o(pipe_userclk_out),
    .mmcm_lock_o(pipe_mmcm_lock_out),

    .clk_50mhz_o(),
    .clk_200mhz_o(idelay200mhz_clk),
    .clk_333mhz_o(dsp_clk)
);

wire idelay_ready;
wire idelay_reset;

IDELAYCTRL idelayctrl_inst (
  .RDY   (idelay_ready),
  .REFCLK(idelay200mhz_clk),
  .RST   (idelay_reset)
);


/////////////////////////////////////////////////
// | Clok             |   USB     |    PCIe     |
// | sys_clk          |    NA     |  100.000    |
// | phy_clk          |  60.000   |   60.000
// | pipe_userclk_out |  75.000   |  125.000    |
// | idelay200mhz_clk | 120.000   |  200.000    |
// | dsp_clk          | 200.000   |  333.333    |
/////////////////////////////////////////////////


wire pipe_oobclk_out = pipe_pclk_out;
wire pipe_rxusrclk_out = pipe_pclk_out;
wire [5:0] tx_buf_av;

// user_clk_pcie is connected to pipe_userclk2_in
pcie_7x_0 pci (
    // Shared clock interface
    .pipe_pclk_in(pipe_pclk_out),
    .pipe_rxusrclk_in(pipe_rxusrclk_out),
    .pipe_rxoutclk_in(2'b00),
    .pipe_mmcm_rst_n(pipe_mmcm_rst_n),
    .pipe_dclk_in(pipe_dclk_out),
    .pipe_userclk1_in(pipe_userclk_out),
    .pipe_userclk2_in(pipe_userclk_out),
    .pipe_oobclk_in( pipe_oobclk_out ),
    .pipe_mmcm_lock_in(pipe_mmcm_lock_out),
    .pipe_txoutclk_out(pipe_txoutclk_in),
    .pipe_rxoutclk_out(),
    .pipe_pclk_sel_out(pipe_pclk_sel_in),
    .pipe_gen3_out(),


    //----------------------------------------------------------------------------------------------------------------//
    // PCI Express (pci_exp) Interface                                                                                //
    //----------------------------------------------------------------------------------------------------------------//
    // Tx
    .pci_exp_txn                               ( pci_exp_txn[1:0] ),
    .pci_exp_txp                               ( pci_exp_txp[1:0] ),

    // Rx
    .pci_exp_rxn                               ( pci_exp_rxn[1:0] ),
    .pci_exp_rxp                               ( pci_exp_rxp[1:0] ),

    //----------------------------------------------------------------------------------------------------------------//
    // AXI-S Interface                                                                                                //
    //----------------------------------------------------------------------------------------------------------------//
    // Common
    .user_clk_out                              ( user_clk_pcie ),
    .user_reset_out                            ( user_reset ),
    .user_lnk_up                               ( user_lnk_up ),
    .user_app_rdy                              ( ),

    // TX
    .s_axis_tx_tready                          ( s_axis_tx_tready ),
    .s_axis_tx_tdata                           ( s_axis_tx_tdata ),
    .s_axis_tx_tkeep                           ( s_axis_tx_tkeep_8 ),
    .s_axis_tx_tuser                           ( s_axis_tx_tuser ),
    .s_axis_tx_tlast                           ( s_axis_tx_tlast ),
    .s_axis_tx_tvalid                          ( s_axis_tx_tvalid ),

    // Rx
    .m_axis_rx_tdata                           ( m_axis_rx_tdata ),
    .m_axis_rx_tkeep                           ( m_axis_rx_tkeep_8 ),
    .m_axis_rx_tlast                           ( m_axis_rx_tlast ),
    .m_axis_rx_tvalid                          ( m_axis_rx_tvalid ),
    .m_axis_rx_tready                          ( m_axis_rx_tready ),
    .m_axis_rx_tuser                           ( m_axis_rx_tuser ),

    .tx_cfg_gnt                                ( tx_cfg_gnt ),
    .rx_np_ok                                  ( rx_np_ok ),
    .rx_np_req                                 ( rx_np_req ),
    .cfg_trn_pending                           ( cfg_trn_pending ),
    .cfg_pm_halt_aspm_l0s                      ( cfg_pm_halt_aspm_l0s ),
    .cfg_pm_halt_aspm_l1                       ( cfg_pm_halt_aspm_l1 ),
    .cfg_pm_force_state_en                     ( cfg_pm_force_state_en ),
    .cfg_pm_force_state                        ( cfg_pm_force_state ),
    .cfg_dsn                                   ( cfg_dsn ),
    .cfg_turnoff_ok                            ( cfg_turnoff_ok ),
    .cfg_pm_wake                               ( cfg_pm_wake ),
    .cfg_pm_send_pme_to                        ( 1'b0 ),
    .cfg_ds_bus_number                         ( 8'b0 ),
    .cfg_ds_device_number                      ( 5'b0 ),
    .cfg_ds_function_number                    ( 3'b0 ),

    //----------------------------------------------------------------------------------------------------------------//
    // Configuration (CFG) Interface                                                                                  //
    //----------------------------------------------------------------------------------------------------------------//
    .cfg_device_number                         ( cfg_device_number ),
    .cfg_dcommand2                             ( ),
    .cfg_pmcsr_pme_status                      ( ),
    .cfg_status                                ( ),
    .cfg_to_turnoff                            ( cfg_to_turnoff ),
    .cfg_received_func_lvl_rst                 ( ),
    .cfg_dcommand                              ( cfg_pcie_ctrl_reg ),
    .cfg_bus_number                            ( cfg_bus_number ),
    .cfg_function_number                       ( cfg_function_number ),
    .cfg_command                               ( cfg_command ),
    .cfg_dstatus                               ( ),
    .cfg_lstatus                               ( ),
    .cfg_pcie_link_state                       ( ),
    .cfg_lcommand                              ( ),
    .cfg_pmcsr_pme_en                          ( ),
    .cfg_pmcsr_powerstate                      ( ),
    .tx_buf_av                                 ( tx_buf_av ),
    .tx_err_drop                               ( ),
    .tx_cfg_req                                ( ),
    .cfg_vc_tcvc_map                           ( ),
    //------------------------------------------------//
    // RP Only                                        //
    //------------------------------------------------//
    .cfg_bridge_serr_en                        ( ),
    .cfg_slot_control_electromech_il_ctl_pulse ( ),
    .cfg_root_control_syserr_corr_err_en       ( ),
    .cfg_root_control_syserr_non_fatal_err_en  ( ),
    .cfg_root_control_syserr_fatal_err_en      ( ),
    .cfg_root_control_pme_int_en               ( ),
    .cfg_aer_rooterr_corr_err_reporting_en     ( ),
    .cfg_aer_rooterr_non_fatal_err_reporting_en( ),
    .cfg_aer_rooterr_fatal_err_reporting_en    ( ),
    .cfg_aer_rooterr_corr_err_received         ( ),
    .cfg_aer_rooterr_non_fatal_err_received    ( ),
    .cfg_aer_rooterr_fatal_err_received        ( ),

    //------------------------------------------------//
    // EP Only                                        //
    //------------------------------------------------//
    .cfg_interrupt                             ( cfg_interrupt ),
    .cfg_interrupt_rdy                         ( cfg_interrupt_rdy ),
    .cfg_interrupt_assert                      ( cfg_interrupt_assert ),
    .cfg_interrupt_di                          ( cfg_interrupt_di ),
    .cfg_interrupt_do                          ( ),
    .cfg_interrupt_mmenable                    ( cfg_interrupt_mmenable ),
    .cfg_interrupt_msienable                   ( cfg_interrupt_msienable ),
    .cfg_interrupt_msixenable                  ( ),
    .cfg_interrupt_msixfm                      ( ),
    .cfg_interrupt_stat                        ( cfg_interrupt_stat ),
    .cfg_pciecap_interrupt_msgnum              ( cfg_pciecap_interrupt_msgnum ),

    //----------------------------------------------------------------------------------------------------------------//
    // System  (SYS) Interface                                                                                        //
    //----------------------------------------------------------------------------------------------------------------//
    .sys_clk                                    ( sys_clk ),
    .sys_rst_n                                  ( sys_rst_n_c )
);



wire led_front;

reg [27:0] cntr2 = 28'hfff_ffff;
always @(posedge  user_clk_pcie) begin
  cntr2 <= cntr2 + 1;
end

wire clk_enumerating = cntr2[27] || cntr2[26];               // On-On-On-Off
wire clk_reset       = cntr2[27] && cntr2[26] && cntr2[25];  // Off-..-Off-On
wire clk_ext_reset   = cntr2[27] && cntr2[26] && cntr2[24];

wire uclk_reset      = cntr2[27] || cntr2[26] || cntr2[25];  // Off-..-Off-On


wire led_diagnostic_pcie = (!sys_rst_n_c)                    ? clk_ext_reset :
                      (user_reset_q)                    ? clk_reset :
                      (!user_reset_q && !user_lnk_up_q) ? clk_enumerating :
                           led_front;
wire led_diagnostic_usb2 = (!usb2_fsm_done) ? uclk_reset : led_front;

assign led_internal = (~cfg_startup_mode_usb) ? led_diagnostic_pcie : led_diagnostic_usb2;

wire legacy_interrupt_disabled = 1'b0;


wire [1:0] s_axis_tx_tkeep;
wire [1:0] m_axis_rx_tkeep = { m_axis_rx_tkeep_8[4], m_axis_rx_tkeep_8[0] };

assign s_axis_tx_tkeep_8 = {{4{s_axis_tx_tkeep[1]}}, {4{s_axis_tx_tkeep[0]}}};

// 7-series RQ/CC RC/CQ multiplexing
localparam C_DATA_WIDTH = 64;
localparam KEEP_WIDTH   = C_DATA_WIDTH / 32;
//`DEFINE_AXIS_RVDLKU_PORT(s_axis_ccb_t, C_DATA_WIDTH, KEEP_WIDTH, 1);
`DEFINE_AXIS_RVDLKU_PORT(s_axis_cc_t, C_DATA_WIDTH, KEEP_WIDTH, 1);
`DEFINE_AXIS_RVDLKU_PORT(s_axis_rq_t, C_DATA_WIDTH, KEEP_WIDTH, 1);
`DEFINE_AXIS_RVDLKU_PORT(m_axis_cq_t, C_DATA_WIDTH, KEEP_WIDTH, 1);
`DEFINE_AXIS_RVDLKU_PORT(m_axis_cqb_t, C_DATA_WIDTH, KEEP_WIDTH, 1);
`DEFINE_AXIS_RVDLKU_PORT(m_axis_rc_t, C_DATA_WIDTH, KEEP_WIDTH, 1);

axis_mux #(.DATA_WIDTH(C_DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .PORTS(2)) axis_cc_rq_mux(
    .clk(user_clk_pcie),
    .rst(user_reset_q),
    
    `AXIS_RVDLK_PORT_CONN(m_axis_t, s_axis_tx_t),
    
    .sn_axis_tready({ s_axis_rq_tready, s_axis_cc_tready }),
    .sn_axis_tdata( { s_axis_rq_tdata,  s_axis_cc_tdata }),
    .sn_axis_tkeep( { s_axis_rq_tkeep,  s_axis_cc_tkeep }),
    .sn_axis_tlast( { s_axis_rq_tlast,  s_axis_cc_tlast }),
    .sn_axis_tvalid({ s_axis_rq_tvalid, s_axis_cc_tvalid })
);


    
wire cq_sel = (rx_bar_hit[0]); // != 7'h00);

assign m_axis_cqb_tdata  = m_axis_rx_tdata;
assign m_axis_cqb_tkeep  = m_axis_rx_tkeep;
assign m_axis_cqb_tlast  = m_axis_rx_tlast;
assign m_axis_cqb_tvalid = m_axis_rx_tvalid && cq_sel;

assign m_axis_rc_tdata  = m_axis_rx_tdata;
assign m_axis_rc_tkeep  = m_axis_rx_tkeep;
assign m_axis_rc_tlast  = m_axis_rx_tlast;
assign m_axis_rc_tvalid = m_axis_rx_tvalid && !cq_sel;

assign m_axis_rx_tready = (cq_sel) ? m_axis_cqb_tready : m_axis_rc_tready;


axis_fifo_pkt #(
  .WIDTH(C_DATA_WIDTH + KEEP_WIDTH),
  .PASSTHROUGH(0),
  .DEEP(64)
) axis_cq_buffer (
  .clk(user_clk_pcie),
  .rst(user_reset_q),

  .s_rx_tdata({m_axis_cqb_tkeep, m_axis_cqb_tdata}),
  .s_rx_tvalid(m_axis_cqb_tvalid),
  .s_rx_tready(m_axis_cqb_tready),
  .s_rx_tlast(m_axis_cqb_tlast),

  .m_tx_tdata({m_axis_cq_tkeep, m_axis_cq_tdata}),
  .m_tx_tvalid(m_axis_cq_tvalid),
  .m_tx_tready(m_axis_cq_tready),
  .m_tx_tlast(m_axis_cq_tlast)
);


assign cfg_interrupt_assert = 1'b0;
assign cfg_interrupt_stat   = 1'b0;

app_xsdr #(
    .MASTER_BUS_SPEED(MASTER_BUS_SPEED),
    .I2C_SPEED(I2C_SPEED),
    .HAS_LMS8(HAS_LMS8),
    .EGPIO_ALT_MODE1(EGPIO_ALT_MODE1),
    .ULTRA_SCALE(ULTRA_SCALE),
    .ULTRA_SCALE_CLI_TAG(ULTRA_SCALE_CLI_TAG)
) app (
    .gclk_dsp(dsp_clk),
    .gclk_master(user_clk_pcie),
    .reset(user_reset_q),

/*
    .s_axis_rx_tdata(m_axis_rx_tdata),
    .s_axis_rx_tkeep({m_axis_rx_tkeep[4], m_axis_rx_tkeep[0]}),
    .s_axis_rx_tlast(m_axis_rx_tlast),
    .s_axis_rx_tvalid(m_axis_rx_tvalid),
    .s_axis_rx_tready(m_axis_rx_tready),
    .pcie_rx_bar_hit(rx_bar_hit),

    .m_axis_tx_tready(s_axis_tx_tready),
    .m_axis_tx_tdata(s_axis_tx_tdata),
    .m_axis_tx_tkeep(s_axis_tx_tkeep_32),
    .m_axis_tx_tlast(s_axis_tx_tlast),
    .m_axis_tx_tvalid(s_axis_tx_tvalid),
    */
    .s_axis_cc_tdata ( s_axis_cc_tdata ),
    .s_axis_cc_tkeep ( s_axis_cc_tkeep ),
    .s_axis_cc_tlast ( s_axis_cc_tlast ),
    .s_axis_cc_tvalid( s_axis_cc_tvalid ),
    .s_axis_cc_tuser ( s_axis_cc_tuser ),
    .s_axis_cc_tready( s_axis_cc_tready ),

    .s_axis_rq_tdata ( s_axis_rq_tdata ),
    .s_axis_rq_tkeep ( s_axis_rq_tkeep ),
    .s_axis_rq_tlast ( s_axis_rq_tlast ),
    .s_axis_rq_tvalid( s_axis_rq_tvalid ),
    .s_axis_rq_tuser ( s_axis_rq_tuser ),
    .s_axis_rq_tready( s_axis_rq_tready ),

    .m_axis_cq_tdata ( m_axis_cq_tdata ),
    .m_axis_cq_tlast ( m_axis_cq_tlast ),
    .m_axis_cq_tvalid( m_axis_cq_tvalid ),
    .m_axis_cq_tuser ( m_axis_cq_tuser ),
    .m_axis_cq_tkeep ( m_axis_cq_tkeep ),
    .m_axis_cq_tready( m_axis_cq_tready ),
    
    .m_axis_rc_tdata ( m_axis_rc_tdata ),
    .m_axis_rc_tlast ( m_axis_rc_tlast ),
    .m_axis_rc_tvalid( m_axis_rc_tvalid ),
    .m_axis_rc_tuser ( m_axis_rc_tuser ),
    .m_axis_rc_tkeep ( m_axis_rc_tkeep ),
    .m_axis_rc_tready( m_axis_rc_tready ),


    .pcie7s_tx_buf_av(tx_buf_av),

    // pci interrupts
    .cfg_completer_id(cfg_completer_id),
    .cfg_max_payload_size(cfg_max_payload_size),
    .cfg_max_read_req_size(cfg_max_read_req_size),

    .cfg_interrupt_msienable(cfg_interrupt_msienable),
    .cfg_interrupt_rdy(cfg_interrupt_rdy),
    .cfg_interrupt(cfg_interrupt),
    //.cfg_interrupt_assert(cfg_interrupt_assert),
    .cfg_interrupt_di(cfg_interrupt_di),
    //.legacy_interrupt_disabled(legacy_interrupt_disabled),
    .cfg_interrupt_mmenable(cfg_interrupt_mmenable),
    //.cfg_pciecap_interrupt_msgnum(cfg_pciecap_interrupt_msgnum),
    //.cfg_interrupt_stat(cfg_interrupt_stat),


    // streaming
    .hwcfg_port({ 1'b1, cfg_startup_mode_usb, 6'h0, hwcfg_devid, hwcfg_revid, PHY_CONFIG_ID }),
    .rxclk_f_clk(rxd_demux_clk),
    .rxd_demux_data(rxd_demux_data),
    .rxd_demux_valid(rxd_demux_valid),
    .rxd_enable(rxd_enable),

    // LMS6 output samples IQ demuxed
    .txclk_m_clk(txd_demux_clk),
    .txd_demux_data(txd_demux_data),
    .txd_demux_valid(txd_demux_valid),
    .txd_enable(lms7_tx_en),

    .cfg_port_clk(cfg_port_clk),
    .cfg_port_rst(cfg_port_rst),

    .cfg_port_wvalid(cfg_port_wvalid),
    .cfg_port_wready(cfg_port_wready),
    .cfg_port_wdata(cfg_port_wdata),

    .cfg_port_rvalid(cfg_port_rvalid),
    .cfg_port_rready(cfg_port_rready),
    .cfg_port_rdata({ cfg_port_rdata_p1, cfg_port_rdata_p0 }),

    // per
    .sda_in(sda_in),
    .scl_in(scl_in),
    .sda_out_eo(sda_out_eo),
    .scl_out_eo(scl_out_eo),
    .second_i2c_sec(second_i2c_sec),

    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_sclk(spi_sclk),
    .spi_sen(spi_sen),

    .uart_rxd(uart_rxd),
    .uart_txd(uart_txd),

    .flash_dout(flash_dout),
    .flash_din(flash_din),
    .flash_ndrive(flash_ndrive),
    .flash_ncs(flash_ncs),
    .flash_sck(flash_sck),

    .igpo_lms_core_ldo_en(igpo_lms_core_ldo_en),
    .igpo_lms_rxen(igpo_lms_rxen),
    .igpo_lms_txen(igpo_lms_txen),
    .igpo_lms_reset(igpo_lms_reset),
    .igpo_lms_einit(xtrx_i2c_sel),

 //   .igpo_en_tcxo(igpo_en_tcxo),
    .igpo_ext_clk(igpo_ext_clk),
    .igpo_en_gps(igpo_en_gps),
    .igpo_iovcc_sel(igpo_iovcc_sel),
    .igpo_en_smsigio(igpo_en_smsigio),

    .igpo_txsw(igpo_txsw),
    .igpo_rxsw(igpo_rxsw),

    .igpo_usb_clk_en(igpo_usb_clk_en),
    .igpo_ldoen(igpo_ldoen),
    .igpo_phydly(igp_phydly),

    .igpo_delay_reset(idelay_reset),
    .igpi_delay_ready(idelay_ready),

    .igpi_usr_access2_data(usr_access2_data),

    .usb2_altrst_valid(cfg_startup_mode_usb),
    .usb2_altrst_logicrst(brng_usb_logic_reset),
    .usb2_altrst_phynrst(brng_phy_nrst),

    .usb2_phy_clk(phy_clk),
    .usb2_phy_nrst(phy_nrst),
    .usb2_phy_dir(phy_dir),
    .usb2_phy_nxt(phy_nxt),
    .usb2_phy_stp(phy_stp),
    .usb2_phy_do(phy_do),
    .usb2_phy_di(phy_di),
    .usb2_phy_doe(phy_doe),

    .led_front(led_front),
    .user_gpio_oe(user_gpio_oe),
    .user_gpio_out(user_gpio_out),
    .user_gpio_in(user_gpio_in),

    .lms8_sen(lms8_sen),
    .lms8_reset_n(lms8_reset_n)
);


endmodule

`default_nettype wire
