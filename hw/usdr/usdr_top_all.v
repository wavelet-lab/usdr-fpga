// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2022-2024 Wavelet Lab
//
//
module usdr_top_all #(
    parameter [7:0] DEVICE_ID = 8'h2f,
    parameter [7:0] DEVICE_REVISION = 4,
    parameter       USB2_PRESENT = 1'b1,
    parameter       PCIE_INIT_ONLY = 1'b0
)(
  output  [1:0] pci_exp_txp,
  output  [1:0] pci_exp_txn,
  input   [1:0] pci_exp_rxp,
  input   [1:0] pci_exp_rxn,

  input         sys_clk_p,
  input         sys_clk_n,

  output        dcen,

  inout [3:0]   flash_d,
  output        flash_fcs_b,

  inout [14:0]  gpio,
  inout [1:0]   gpled,
  output        gpledint,

  inout         scl,
  inout         sda,

  output        lms_sen,
  output        lms_rst_n,
  output        lms_sck,
  output        lms_sdio,
  input         lms_sdo,

  output        rxen,
  output        rxsw,
  output        txen,
  output        txsw,
  output        rxmix_en,
  input         rxclk_f,
  input         txclk_ref,
  output        txclk,

  input         rxiqsel,
  input [11:0]  rxd,

  output        txiqsel,
  output [11:0] txd,

`ifndef USDR_REV2
  input         fpga_ref,
  output        osc_buf_en,
`endif

  output        usb_nrst,
  inout [7:0]   usb_d,
  input         usb_clk,
  inout         usb_stp,
  input         usb_dir,
  input         usb_nxt,

  input [1:0]   board_rev
);

wire [1:0] board_rev_b;

IBUF board_rev_0_ibuf( .O(board_rev_b[1]),     .I(board_rev[1]));
IBUF board_rev_1_ibuf( .O(board_rev_b[0]),     .I(board_rev[0]));

// This is universal top file for uSDR rev 1,2,3,4`
// XDC constains are different for revision 1 / 2 and 3 / 4
//
//  pin    | Rev 1        | Rev 2      |
//  K17    | i,PCI_RESET# | o,DC_EN    | PCI_RESET# is no use since it disable DC/DC completly
//  Switch | SKYA21024    | BGS12P2L6  |
//           TX_HI - 0    | TX_HI - 0  |
//           RX_HI - 1    | RX_HI - 1  |
//

generate
if (DEVICE_ID == 0) begin
    $error("Set DEVICE_ID to non-zero");
    assign pci_exp_rxp = 0;
end
endgenerate

wire [7:0] hwcfg_devid = DEVICE_ID;
wire [7:0] hwcfg_revid =
    board_rev_b == 2'b11 ? 8'h01 :
    board_rev_b == 2'b10 ? 8'h02 :
    board_rev_b == 2'b01 ? 8'h03 : DEVICE_REVISION;

wire dcen_ctrl;
`ifndef USDR_REV2
// PMIC should be enabled to prevent leakage current from the other rail
assign dcen = 1'b1;
`else
assign dcen = dcen_ctrl;
`endif


////////////////////////////////////////////////////////////////////////////////
// BOARD CONFIGURATION
localparam PCIE_BITS        = 2;
localparam I2C_SPEED        = 100_000;
localparam MASTER_BUS_SPEED = 125_000_000;

wire      user_clk_pcie;
wire      sys_rst_n_c = 1'b1;
wire      sys_clk;

wire      igp_clk;
wire      igp_rst;

wire      igpo_lms_rst_n;
wire      igpo_rxmix_en;
wire      igpo_txsw;
wire      igpo_rxsw;

wire      spi_mosi;
wire      spi_sclk;
wire      spi_sen;
wire      spi_miso;

wire      sda_in;
wire      scl_in;
wire      sda_out_eo;
wire      scl_out_eo;

wire      use_i2c_alt;
wire      alt_sda_out_eo;
wire      alt_scl_out_eo;

OBUF lms_rst_n_obuf( .O(lms_rst_n),     .I(igpo_lms_rst_n));
OBUF rxmix_en_obuf(  .O(rxmix_en),      .I(igpo_rxmix_en));
OBUF txsw_obuf(      .O(txsw),          .I(igpo_txsw));
OBUF rxsw_obuf(      .O(rxsw),          .I(igpo_rxsw));

assign rxen = 1'b1;
assign txen = 1'b1;

// LMS SPI BUS
OBUF lms_spi_mosi_obuf( .O(lms_sdio),     .I(spi_mosi));
OBUF lms_spi_sck_obuf(  .O(lms_sck),      .I(spi_sclk));
OBUF lms_spi_sen_obuf(  .O(lms_sen),      .I(spi_sen));
IBUF lms_spi_miso_ibuf( .O(spi_miso),     .I(lms_sdo));

IOBUF sda_buf ( .IO(sda), .I(1'b0), .T(use_i2c_alt ? ~alt_sda_out_eo : ~sda_out_eo), .O(sda_in));
IOBUF scl_buf ( .IO(scl), .I(1'b0), .T(use_i2c_alt ? ~alt_scl_out_eo : ~scl_out_eo), .O(scl_in));

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

// LMS HIGHSPEED interface
wire [11:0] lms_rxd;
wire        lms_rxiqsel;

(* IOB = "true" *) reg [11:0] lms_txd;
(* IOB = "true" *) reg        lms_txiqsel;
wire                          lms_txclk;

OBUF txiqsel_obuf(  .O(txiqsel),      .I(lms_txiqsel));
IBUF rxiqsel_ibuf(  .O(lms_rxiqsel),  .I(rxiqsel));
OBUF txclk_obuf(    .O(txclk),        .I(lms_txclk));

generate
for (i = 0; i < 12; i = i + 1) begin: lms_hs
	OBUF txd_obuf(  .O(txd[i]),      .I(lms_txd[i]));
	IBUF rxd_ibuf(  .O(lms_rxd[i]),  .I(rxd[i]));
end
endgenerate

wire rxclk_f_clk;
wire rxclk_f_clk_iob;

BUFG rxclk_buf(.I(rxclk_f), .O(rxclk_f_clk));
assign rxclk_f_clk_iob = rxclk_f_clk;


(* IOB = "true" *) reg [11:0] rxd_iob_data;
(* IOB = "true" *) reg        rxd_iob_iqsel;
always @(posedge rxclk_f_clk_iob) begin
    rxd_iob_data  <= lms_rxd;
    rxd_iob_iqsel <= lms_rxiqsel;
end

wire txclk_m_clk;
wire txclk_m_clk_iob;

BUFG txclk_buf(.I(txclk_ref), .O(txclk_m_clk));
assign txclk_m_clk_iob = txclk_m_clk;

wire [11:0] txd_iob_data;
wire        txd_iob_iqsel;
always @(posedge txclk_m_clk_iob) begin
    lms_txd     <= txd_iob_data;
    lms_txiqsel <= txd_iob_iqsel;
end

ODDR txclk_f_oddr( .Q(lms_txclk), .D1(1'b0), .D2(1'b1), .CE(1'b1), .C(txclk_m_clk_iob), .R(1'b0)  );

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
for (j = 0; j < 15; j = j + 1) begin: gpiobufs
  IOBUF gpio_buf(
    .IO(gpio[j]),
    .I(user_gpio_out[j]),
    .T(~user_gpio_oe[j]),
    .O(user_gpio_in[j])
  );
end
for (j = 0; j < 2; j = j + 1) begin: gpioleds
  IOBUF gpled_buf(
    .IO(gpled[j]),
    .I(user_gpio_out[16 + j]),
    .T(~user_gpio_oe[16 + j]),
    .O(user_gpio_in[16 + j])
  );
end
assign user_gpio_in[15] = 1'b0;
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
wire flash_cclk = cfg_mclk;

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
wire                 pipe_mmcm_lock_out;

xlnx_startup_mmcm #(
    .PCIE_INIT_ONLY(PCIE_INIT_ONLY)
) sfsm (
    .cfg_mclk(cfg_mclk),
    .startup_mode_pcie(startup_mode_pcie),

    .pipe_mmcm_rst_n(pipe_mmcm_rst_n),

    .brng_usb_logic_reset(brng_usb_logic_reset),
    .brng_phy_nrst(brng_phy_nrst),

    .brng_usb_user_reset(brng_usb_user_reset),
    .usb2_fsm_done(usb2_fsm_done),

    .usb2_clk_ready(init_usb_clk_done),
    .pipe_mmcm_lock_out(pipe_mmcm_lock_out)
);

wire pci_mmcm_sel = (startup_mode_pcie) ? 1'b0 : 1'b1;

always @(posedge user_clk_pcie) begin
    user_reset_q  <= (startup_mode_pcie) ? user_reset : brng_usb_user_reset;
    user_lnk_up_q <= user_lnk_up;
end

wire cfg_startup_mode_usb = (!startup_mode_pcie) ;
wire osc_buf_en_p;

`ifdef USDR_REV2
assign    use_i2c_alt = (startup_mode_pcie) ? 1'b0 : ~init_usb_clk_done;

init_usb_clk init_usb_clk(
    .clk(cfg_mclk),
    .rst(startup_mode_pcie),

    .sda_in(sda_in),
    .scl_in(scl_in),
    .sda_out_eo(alt_sda_out_eo),
    .scl_out_eo(alt_scl_out_eo),

    .done(init_usb_clk_done)
);
`else

assign use_i2c_alt = 1'b0;
assign alt_sda_out_eo = 1'b1;
assign alt_scl_out_eo = 1'b1;

// Wait some time (~1ms) to stabilize the OSC after puting EN
localparam DELAY_USB_CNT_BITS = 17;

reg [DELAY_USB_CNT_BITS-1:0] clk_delay = 0;
reg                          init_usb_clk_done_r = 0;

always @(posedge cfg_mclk) begin
    if (cfg_startup_mode_usb) begin
        clk_delay <= clk_delay + 1'b1;
        if (clk_delay[DELAY_USB_CNT_BITS - 1: DELAY_USB_CNT_BITS - 4] == 4'hf) begin
            init_usb_clk_done_r <= 1'b1;
        end
    end else begin
        clk_delay           <= 0;
        init_usb_clk_done_r <= 0;
    end
end
assign init_usb_clk_done = init_usb_clk_done_r;

// USB PHY needs 26Mhz clock, so activate buffer when entering USB mode
assign osc_buf_en = osc_buf_en_p | cfg_startup_mode_usb;
`endif


// Tx
wire                                        s_axis_tx_tready;
wire [3:0]                                  s_axis_tx_tuser;
wire [63:0]                                 s_axis_tx_tdata;
wire [7:0]                                  s_axis_tx_tkeep;
wire                                        s_axis_tx_tlast;
wire                                        s_axis_tx_tvalid;

// Rx
wire [63:0]                                 m_axis_rx_tdata;
wire [7:0]                                  m_axis_rx_tkeep;
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

assign cfg_turnoff_ok = cfg_to_turnoff; //

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
    .clk_200mhz_o(),
    .clk_333mhz_o(dsp_clk)
);


wire pipe_oobclk_out = pipe_pclk_out;
wire pipe_rxusrclk_out = pipe_pclk_out;
wire [5:0] tx_buf_av;

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
    .s_axis_tx_tkeep                           ( s_axis_tx_tkeep ),
    .s_axis_tx_tuser                           ( s_axis_tx_tuser ),
    .s_axis_tx_tlast                           ( s_axis_tx_tlast ),
    .s_axis_tx_tvalid                          ( s_axis_tx_tvalid ),

    // Rx
    .m_axis_rx_tdata                           ( m_axis_rx_tdata ),
    .m_axis_rx_tkeep                           ( m_axis_rx_tkeep ),
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


wire led_aux;

reg [27:0] cntr2 = 28'hfff_ffff;
always @(posedge cfg_mclk) begin
  cntr2 <= cntr2 + 1;
end

wire clk_enumerating = cntr2[27] || cntr2[26];               // On-On-On-Off
wire clk_reset       = cntr2[27] && cntr2[26] && cntr2[25];  // Off-..-Off-On
wire clk_ext_reset   = cntr2[27] && cntr2[26] && cntr2[24];

wire uclk_reset      = cntr2[27] || cntr2[26] || cntr2[25];  // Off-..-Off-On

wire led_diagnostic = (!sys_rst_n_c)                    ? clk_ext_reset :
                      (user_reset_q)                    ? clk_reset :
                      (!user_reset_q && !user_lnk_up_q) ? clk_enumerating :
                      led_aux;

wire led_diagnostic_usb2 = (!usb2_fsm_done) ? uclk_reset : led_aux;

assign gpledint = ! (startup_mode_pcie ? led_diagnostic : led_diagnostic_usb2);

wire legacy_interrupt_disabled = 1'b0;

wire [1:0] s_axis_tx_tkeep_32;
assign s_axis_tx_tkeep = {{4{s_axis_tx_tkeep_32[1]}}, {4{s_axis_tx_tkeep_32[0]}}};


app_usdr_pcie #(
    .MASTER_BUS_SPEED(MASTER_BUS_SPEED),
    .I2C_SPEED(I2C_SPEED),
    .USB2_PRESENT(USB2_PRESENT)
) app (
    .gclk_dsp(dsp_clk),
    .gclk_master(user_clk_pcie),
    .reset(user_reset_q),

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
    .pcie7s_tx_buf_av(tx_buf_av),

    // pci interrupts
    .cfg_completer_id(cfg_completer_id),
    .cfg_max_payload_size(cfg_max_payload_size),
    .cfg_max_read_req_size(cfg_max_read_req_size),

    .cfg_interrupt_msienable(cfg_interrupt_msienable),
    .cfg_interrupt_rdy(cfg_interrupt_rdy),
    .cfg_interrupt(cfg_interrupt),
    .cfg_interrupt_assert(cfg_interrupt_assert),
    .cfg_interrupt_di(cfg_interrupt_di),
    .legacy_interrupt_disabled(legacy_interrupt_disabled),
    .cfg_interrupt_mmenable(cfg_interrupt_mmenable),
    .cfg_pciecap_interrupt_msgnum(cfg_pciecap_interrupt_msgnum),
    .cfg_interrupt_stat(cfg_interrupt_stat),

    // streaming
    .hwcfg_port({ 1'b1, cfg_startup_mode_usb, 6'h0, hwcfg_devid, hwcfg_revid, 8'h00 }),

    .rxclk_f_clk(rxclk_f_clk_iob),
    .rxd_iob_data(rxd_iob_data),
    .rxd_iob_iqsel(rxd_iob_iqsel),

    // LMS6 output samples IQ demuxed
    .txclk_m_clk(txclk_m_clk_iob),
    .txd_iob_data(txd_iob_data),
    .txd_iob_iqsel(txd_iob_iqsel),

    // per
    .sda_in(sda_in),
    .scl_in(scl_in),
    .sda_out_eo(sda_out_eo),
    .scl_out_eo(scl_out_eo),

    .spi_mosi(spi_mosi),
    .spi_miso(spi_miso),
    .spi_sclk(spi_sclk),
    .spi_sen(spi_sen),

    .uart_rxd(1'b1),
    .uart_txd(),

    .flash_dout(flash_dout),
    .flash_din(flash_din),
    .flash_ndrive(flash_ndrive),
    .flash_ncs(flash_ncs),
    .flash_sck(flash_sck),
    .flash_cclk(flash_cclk),

    .igpo_lms_rst_n(igpo_lms_rst_n),
    .igpo_rxmix_en(igpo_rxmix_en),
    .igpo_txsw(igpo_txsw),
    .igpo_rxsw(igpo_rxsw),
    .igpo_dcen(dcen_ctrl),
    .igpo_led(led_aux),

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

    .igpi_usr_access2_data(usr_access2_data),

    .user_gpio_oe(user_gpio_oe),
    .user_gpio_out(user_gpio_out),
    .user_gpio_in(user_gpio_in),

    .enable_osc(osc_buf_en_p)
);


endmodule

