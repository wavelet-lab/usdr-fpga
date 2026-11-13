set_property BITSTREAM.CONFIG.UNUSEDPIN        Pullup    [current_design]
set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN Disable   [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE       66        [current_design]
set_property BITSTREAM.GENERAL.COMPRESS        TRUE      [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE    YES       [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH     1         [current_design]
set_property BITSTREAM.config.SPI_opcode       0x0B      [current_design]
set_property BITSTREAM.CONFIG.USR_ACCESS       timestamp [current_design]

set_property CFGBVS           GND [current_design]
set_property CONFIG_VOLTAGE   1.8 [current_design]


# set_property BITSTREAM.CONFIG.USERID           0x000001  [current_design]


# Bitstream configuration GOLDEN
set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR   0x1C0000   [current_design]
set_property BITSTREAM.CONFIG.NEXT_CONFIG_REBOOT ENABLE     [current_design]
set_property BITSTREAM.CONFIG.TIMER_CFG          0x002fbd0 [current_design]
set_property BITSTREAM.CONFIG.CONFIGFALLBACK     ENABLE     [current_design]

# operational
# set_property BITSTREAM.CONFIG.TIMER_CFG        0x4001fbd0  [current_design]
# set_property BITSTREAM.CONFIG.CONFIGFALLBACK   ENABLE      [current_design]



###########################################################
# IO types
###########################################################

set VIO_CMOS_TYPE        LVCMOS18
set VIO_CMOS_LML_DRIVE   12

set VIO_LML2_TYPE        LVCMOS18

# set VIO_LML1_TYPE        LVCMOS18
set VIO_LML1_TYPE        HSTL_II_18
set_property INTERNAL_VREF  0.9               [get_iobanks 14]
set_property IN_TERM        UNTUNED_SPLIT_50  [get_ports {lml1_diq[*] lml1_iqsel lml1_clk_m}]
set_property IBUF_LOW_PWR   TRUE              [get_ports {lml1_diq[*] lml1_iqsel}]
set_property IBUF_LOW_PWR   FALSE             [get_ports {lml1_clk_m}]

set_property IOSTANDARD     HSTL_II_18        [get_ports {lml2_clk_m}]
set_property IN_TERM        UNTUNED_SPLIT_50  [get_ports {lml2_clk_m}]
set_property IBUF_LOW_PWR   FALSE             [get_ports {lml2_clk_m}]

###########################################################
# PCIexpress
###########################################################

# PCI_REF_CLK
set_property PACKAGE_PIN A8 [get_ports sys_clk_n]
set_property PACKAGE_PIN B8 [get_ports sys_clk_p]


##########################################################
# USB PHY 1.8V (BANK 16)
##########################################################
set_property IOSTANDARD LVCMOS18 [get_ports {usb_d[*] usb_clk usb_dir usb_stp usb_nxt}]

set_property PACKAGE_PIN A14 [get_ports usb_d[7]]
set_property PACKAGE_PIN A15 [get_ports usb_d[6]]
set_property PACKAGE_PIN C15 [get_ports usb_d[5]]
set_property PACKAGE_PIN B15 [get_ports usb_d[3]]
set_property PACKAGE_PIN A16 [get_ports usb_d[4]]
set_property PACKAGE_PIN A17 [get_ports usb_d[2]]
set_property PACKAGE_PIN C16 [get_ports usb_clk]
set_property PACKAGE_PIN B16 [get_ports usb_d[1]]
set_property PACKAGE_PIN C17 [get_ports usb_stp]
set_property PACKAGE_PIN B17 [get_ports usb_nxt]
set_property PACKAGE_PIN B18 [get_ports usb_dir]
set_property PACKAGE_PIN A18 [get_ports usb_d[0]]

##########################################################
# LML1 & LML2 1.8V (BANK 14)
##########################################################

set_property PULLUP   true [get_ports usb_stp]

##  schematics  RXD = lml1
##  schematics  TXD = lml2

set_property IOSTANDARD $VIO_CMOS_TYPE [get_ports usb_nrst]
set_property IOSTANDARD $VIO_CMOS_TYPE [get_ports {flash_d[*]}]
set_property IOSTANDARD $VIO_CMOS_TYPE [get_ports flash_fcs_b]

set_property IOSTANDARD $VIO_LML1_TYPE [get_ports {lml1_clk_f}]
set_property IOSTANDARD $VIO_LML1_TYPE [get_ports {lml1_diq[*] lml1_iqsel lml1_clk_m}]
#set_property IN_TERM UNTUNED_SPLIT_50  [get_ports {lml1_diq[*] lml1_iqsel lml1_clk_m}]
#set_property IBUF_LOW_PWR false        [get_ports {lml1_diq[*] lml1_iqsel lml1_clk_m}]


set_property IOSTANDARD $VIO_LML2_TYPE [get_ports {lml2_clk_f}]
#set_property IOSTANDARD $VIO_CMOS_TYPE [get_ports {lml2_clk_f}]
set_property IOSTANDARD $VIO_LML2_TYPE [get_ports {lml2_diq[*] lml2_iqsel}]
#set_property IOSTANDARD $VIO_LML2_TYPE [get_ports {lml2_clk_m}]
#set_property IN_TERM UNTUNED_SPLIT_50 [get_ports {lml2_diq[*] lml2_iqsel lml2_clk_m}]

set_property IOSTANDARD $VIO_CMOS_TYPE [get_ports {rxsw intref_sw usb_clk_en}]
set_property IOSTANDARD $VIO_CMOS_TYPE [get_ports {lms_ldo_en lms_rxen lms_txen}]
set_property IOSTANDARD $VIO_CMOS_TYPE [get_ports {lms_sdio lms_sdo lms_sclk lms_sen}]
set_property IOSTANDARD $VIO_CMOS_TYPE [get_ports {sda2 scl2}]
set_property IOSTANDARD $VIO_CMOS_TYPE [get_ports {en_ext_buf}]

set_property PULLUP   true             [get_ports {lms_sdio lms_sdo lms_sen}]
set_property PULLUP   true             [get_ports {flash_fcs_b flash_d[0] flash_d[1] flash_d[2] flash_d[3]}]
set_property PULLUP   true             [get_ports {en_ext_buf}]

set_property SLEW  FAST                [get_ports {lml1_clk_f lml2_clk_f}]

set_property SLEW  FAST                [get_ports {lml2_diq[*] lml2_iqsel}]
set_property DRIVE 12                  [get_ports {lml2_clk_f lml2_diq[*] lml2_iqsel}]

# set_property DRIVE 24                  [get_ports {lml1_clk_f}]


set_property PACKAGE_PIN D17 [get_ports usb_nrst]
set_property PACKAGE_PIN D18 [get_ports flash_d[0]]
set_property PACKAGE_PIN D19 [get_ports flash_d[1]]
set_property PACKAGE_PIN G18 [get_ports flash_d[2]]
set_property PACKAGE_PIN F18 [get_ports flash_d[3]]
# E18 tied to GND, EN_EXT_BUF in r4
set_property PACKAGE_PIN E18 [get_ports en_ext_buf]
set_property PACKAGE_PIN E19 [get_ports usb_clk_en]
set_property PACKAGE_PIN H19 [get_ports lml1_diq[2]]
set_property PACKAGE_PIN G19 [get_ports rxsw]
   # was [get_ports intref_sw], now ...
set_property PACKAGE_PIN H17 [get_ports lml1_diq[0]]
   # was [get_ports usb_clk_en], now ...
set_property PACKAGE_PIN G17 [get_ports intref_sw]
set_property PACKAGE_PIN K19 [get_ports flash_fcs_b]
set_property PACKAGE_PIN J19 [get_ports lml1_diq[7]]
   # was [get_ports lml1_diq[0]], now ...
set_property PACKAGE_PIN J17 [get_ports lml1_diq[6]]
   # was [get_ports lml1_diq[6]], now ...
set_property PACKAGE_PIN J18 [get_ports lml1_diq[9]]
   # was [get_ports lml1_diq[1]], now ...
set_property PACKAGE_PIN L18 [get_ports extref_n]
   # was [get_ports lml1_diq[9]], now ...
set_property PACKAGE_PIN K18 [get_ports lml1_diq[1]]
set_property PACKAGE_PIN N18 [get_ports lml1_diq[4]]
set_property PACKAGE_PIN N19 [get_ports lml1_diq[5]]
set_property PACKAGE_PIN P19 [get_ports lml1_diq[11]]
set_property PACKAGE_PIN R19 [get_ports lms_ldo_en]
set_property PACKAGE_PIN M18 [get_ports lml1_diq[3]]
set_property PACKAGE_PIN M19 [get_ports lml1_iqsel]
set_property PACKAGE_PIN L17 [get_ports lml1_clk_m]
set_property PACKAGE_PIN K17 [get_ports lml1_clk_f]
set_property PACKAGE_PIN N17 [get_ports lml2_clk_m]
set_property PACKAGE_PIN P17 [get_ports lml2_clk_f]
set_property PACKAGE_PIN P18 [get_ports lml1_diq[8]]
set_property PACKAGE_PIN R18 [get_ports lms_rxen]
set_property PACKAGE_PIN U19 [get_ports lml2_diq[9]]
set_property PACKAGE_PIN V19 [get_ports lml2_diq[6]]
set_property PACKAGE_PIN W18 [get_ports lml2_diq[4]]
set_property PACKAGE_PIN W19 [get_ports lml2_diq[5]]
set_property PACKAGE_PIN T17 [get_ports lml1_diq[10]]
set_property PACKAGE_PIN T18 [get_ports lml2_iqsel]
set_property PACKAGE_PIN U17 [get_ports lml2_diq[3]]
set_property PACKAGE_PIN U18 [get_ports lml2_diq[7]]
set_property PACKAGE_PIN V16 [get_ports lml2_diq[2]]
set_property PACKAGE_PIN V17 [get_ports lml2_diq[8]]
set_property PACKAGE_PIN W16 [get_ports lml2_diq[0]]
set_property PACKAGE_PIN W17 [get_ports lml2_diq[1]]
set_property PACKAGE_PIN V15 [get_ports lms_sdio]
set_property PACKAGE_PIN W15 [get_ports lms_sdo]
set_property PACKAGE_PIN W13 [get_ports lms_sclk]
set_property PACKAGE_PIN W14 [get_ports sda2]
set_property PACKAGE_PIN U15 [get_ports lml2_diq[11]]
set_property PACKAGE_PIN U16 [get_ports lml2_diq[10]]
set_property PACKAGE_PIN V13 [get_ports lms_sen]
set_property PACKAGE_PIN V14 [get_ports scl2]
set_property PACKAGE_PIN U14 [get_ports lms_txen]


##########################################################
# GPIO & AUX 1.8V (BANK35)
##########################################################

set_property IOSTANDARD LVCMOS18 [get_ports {gpio[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpled[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {lml1_txnrx lml2_txnrx}]
set_property IOSTANDARD LVCMOS18 [get_ports {lms_reset_n ldoen txsw}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpledint}]


set_property PACKAGE_PIN G3 [get_ports lml2_txnrx]
set_property PACKAGE_PIN G2 [get_ports gpio[7]]
set_property PACKAGE_PIN H2 [get_ports gpio[6]]
set_property PACKAGE_PIN J2 [get_ports gpio[3]]
set_property PACKAGE_PIN H1 [get_ports gpio[5]]
set_property PACKAGE_PIN J1 [get_ports gpio[4]]
set_property PACKAGE_PIN K2 [get_ports gpio[2]]
set_property PACKAGE_PIN L2 [get_ports gpio[0]]
set_property PACKAGE_PIN L1 [get_ports gpio[1]]
set_property PACKAGE_PIN J3 [get_ports lms_reset_n]
set_property PACKAGE_PIN K3 [get_ports gpledint]
# PCIe_RESET
set_property PACKAGE_PIN L3 [get_ports lml1_txnrx]
set_property PACKAGE_PIN M3 [get_ports ldoen]
set_property PACKAGE_PIN M2 [get_ports gpio[9]]
set_property PACKAGE_PIN M1 [get_ports gpio[10]]
set_property PACKAGE_PIN N2 [get_ports gpled[1]]
set_property PACKAGE_PIN N1 [get_ports gpio[8]]
set_property PACKAGE_PIN N3 [get_ports gpled[0]]
set_property PACKAGE_PIN P3 [get_ports txsw]
set_property PACKAGE_PIN P1 [get_ports gpio[11]]

############################################################
# Sensing bank 34
############################################################

set_property PULLDOWN    true     [get_ports {board_id}]
set_property IOSTANDARD  LVCMOS12 [get_ports {board_id}]
set_property PACKAGE_PIN U1       [get_ports board_id]


#########################################################################################
# implementation

# set_property LOC IN_FIFO_X0Y1    [get_cells common/lml_tx/rx_fifo.in_fifo]
# set_property LOC OUT_FIFO_X0Y2   [get_cells common/lml_rx/tx_fifo.out_fifo]

