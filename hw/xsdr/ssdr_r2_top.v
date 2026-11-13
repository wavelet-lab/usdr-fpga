// SPDX-License-Identifier: CERN-OHL-P
//
// Copyright 2025 Wavelet Lab
//
//
module ssdr_r2_top #(
    parameter [7:0] DEVICE_ID = 8'h33,
    parameter [7:0] DEVICE_REVISION = 0
)(
  output  [3:0] pci_exp_txp,
  output  [3:0] pci_exp_txn,
  input   [3:0] pci_exp_rxp,
  input   [3:0] pci_exp_rxn,

  input         sys_clk_p,
  input         sys_clk_n,

  input         sys_rst_n,

  // LMS SPI
  output        lms_sdio,
  output        lms_sclk,
  input         lms_sdo,
  output        lms_sen,

  // LMS generic
  output        lms_reset_n,
  output        lms_rxen,
  output        lms_txen,
  output        lms_ldo_en,

  // LML1 - TX
  output        lml1_txnrx,
  input         lml1_clk_m,
  output        lml1_clk_f,
  inout         lml1_iqsel,
  inout [11:0]  lml1_diq,

  // LML2 - RX
  output        lml2_txnrx,
  input         lml2_clk_m,
  output        lml2_clk_f,
  inout         lml2_iqsel,
  inout [11:0]  lml2_diq,

  // USB2 Phy
  inout [7:0]   usb_d,
  input         usb_clk,
  output        usb_nrst,
  input         usb_dir,
  inout         usb_stp,
  input         usb_nxt,

  // EXTERNAL GPIO
  inout [9:0]   gpio,

  // I2C_BUS2 (LP8758)
  inout         sda2,
  inout         scl2,

  //////////////////////
  // XSDR/SSDR specific
  inout [1:0]   gpled,

  // INTERNAL CONTROL
  output        rxsw,
  output        txsw,

  output        gpledint,

  output        intref_sw,
  output        usb_clk_en,
  output        ldoen,

  // LMS8001 control
  input  [3:0]  lms8_gpio,
  output        lms8_sen,
  output        lms8_reset_n,

  // External sync clock
  input         fpga_ref
);

localparam ULTRA_SCALE_CLI_TAG = 0; //Use US+ TAG manager


wire sys_rst_n_ext = sys_rst_n;

wire led_internal;
wire igpo_usb_clk_en;
wire igpo_ldoen;
wire igpo_ext_clk;

wire       igpo_txsw;
wire [1:0] igpo_rxsw;

// intref_sw 0: EXTCLK -> LMS
// intref_sw 1: INT26M -> LMS
OBUF intref_sw_obuf(   .O(intref_sw),    .I(~igpo_ext_clk));
OBUF usb_clk_en_obuf(  .O(usb_clk_en),   .I(igpo_usb_clk_en));
OBUF ldoen_obuf(       .O(ldoen),        .I(igpo_ldoen));
OBUF gpledint_obuf(    .O(gpledint),     .I(~led_internal)); // LED is inverted

OBUF txsw_obuf(        .O(txsw),         .I(igpo_txsw));
OBUF rxsw_obuf(        .O(rxsw),         .I(igpo_rxsw[0]));

wire [1:0] gpled_gpio_out;
wire [1:0] gpled_gpio_oe;
wire [1:0] gpled_gpio_in;

genvar j;
generate
for (j = 0; j < 2; j = j + 1) begin: gpioleds
  IOBUF gpled_buf(
    .IO(gpled[j]),
    .I(gpled_gpio_out[j]),
    .T(~gpled_gpio_oe[j]),
    .O(gpled_gpio_in[j])
  );
end
endgenerate

ssdrpro_common_top #(
    .DEVICE_ID(DEVICE_ID),
    .DEVICE_REVISION(DEVICE_REVISION),
    .EGPIO_COUNT(10),
    .HAS_LMS8(1),
    .EGPIO_ALT_MODE1(1),
    .HAS_RESET(1),
    .PCIE_LANE_COUNT(4),
    .ULTRA_SCALE(1),
    .C_DATA_WIDTH(128),
    .AXI4_CQ_TUSER_WIDTH(88),
    .AXI4_CC_TUSER_WIDTH(33),
    .AXI4_RQ_TUSER_WIDTH(62),
    .AXI4_RC_TUSER_WIDTH(75),
    .ULTRA_SCALE_CLI_TAG(ULTRA_SCALE_CLI_TAG)
) common (
    .pci_exp_txp(pci_exp_txp),
    .pci_exp_txn(pci_exp_txn),
    .pci_exp_rxp(pci_exp_rxp),
    .pci_exp_rxn(pci_exp_rxn),

    .sys_clk_p(sys_clk_p),
    .sys_clk_n(sys_clk_n),

    // LMS SPI
    .lms_sdio(lms_sdio),
    .lms_sclk(lms_sclk),
    .lms_sdo(lms_sdo),
    .lms_sen(lms_sen),

    // LMS generic
    .lms_reset_n(lms_reset_n),
    .lms_rxen(lms_rxen),
    .lms_txen(lms_txen),
    .lms_ldo_en(lms_ldo_en),

    // LMS port1 - TX
    .lml1_txnrx(lml1_txnrx),
    .lml1_clk_m(lml1_clk_m),
    .lml1_clk_f(lml1_clk_f),
    .lml1_iqsel(lml1_iqsel),
    .lml1_diq(lml1_diq),

    // LMS port2 - RX
    .lml2_txnrx(lml2_txnrx),
    .lml2_clk_m(lml2_clk_m),
    .lml2_clk_f(lml2_clk_f),
    .lml2_iqsel(lml2_iqsel),
    .lml2_diq(lml2_diq),

    // USB2 Phy
    .usb_d(usb_d),
    .usb_clk(usb_clk),
    .usb_nrst(usb_nrst),
    .usb_dir(usb_dir),
    .usb_stp(usb_stp),
    .usb_nxt(usb_nxt),

    // FLASH
    .flash_d(flash_d),
    .flash_fcs_b(flash_fcs_b),

    // GPIO
    .gpio(gpio),

    // I2C_BUS2 (vio: LP8758 [LMS])
    .sda2(sda2),
    .scl2(scl2),

    ///////////////////////////////
    // common control
    .igpo_txsw(igpo_txsw),
    .igpo_rxsw(igpo_rxsw),

    .led_internal(led_internal),

    .uart_txd(1'b1),
    .uart_rxd(),

    .usb_clk_en(igpo_usb_clk_en),
    .igpo_ext_clk(igpo_ext_clk),

    .xsdr_ldoen(igpo_ldoen),

    .xtrx_en_gps(),
    .xtrx_iovcc_sel(),
    .xtrx_en_smsigio(),
    .xtrx_i2c_sel(),

    .sdae_in(1'b1),
    .scle_in(1'b1),
    .sdae_out_eo(),
    .scle_out_eo(),
    .second_i2c_sec(1'b0),

    .gpled_gpio_out(gpled_gpio_out),
    .gpled_gpio_oe(gpled_gpio_oe),
    .gpled_gpio_in(gpled_gpio_in),

    .lms8_sen(lms8_sen),
    .lms8_reset_n(lms8_reset_n),

    .external_clock(1'b0),
    .sys_rst_n_ext(sys_rst_n_ext), // TODO fsm reset

    .excfg_clk(),
    .excfg_rst(),
    .excfg_done(1'b1)
);

endmodule

