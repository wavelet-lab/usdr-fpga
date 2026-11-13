set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4           [current_design]
set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN Disable [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES        [current_design]
set_property BITSTREAM.CONFIG.SPI_OPCODE 8'h6B         [current_design]
set_property BITSTREAM.CONFIG.USR_ACCESS TIMESTAMP     [current_design]

set_property BITSTREAM.CONFIG.CONFIGRATE 102.0         [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE           [current_design]

set_property BITSTREAM.CONFIG.D00_MOSI PULLUP          [current_design]
set_property BITSTREAM.CONFIG.D01_DIN  PULLUP          [current_design]
set_property BITSTREAM.CONFIG.D02      PULLUP          [current_design]
set_property BITSTREAM.CONFIG.D03      PULLUP          [current_design]
set_property BITSTREAM.CONFIG.RDWR_B_FCS_B  PULLUP     [current_design]

set_property CFGBVS GND [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]

# Bitstream configuration GOLDEN
 set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR   0x7B0000   [current_design]
 set_property BITSTREAM.CONFIG.NEXT_CONFIG_REBOOT ENABLE     [current_design]
 set_property BITSTREAM.CONFIG.TIMER_CFG          0x002fbd0  [current_design]
 set_property BITSTREAM.CONFIG.CONFIGFALLBACK     ENABLE     [current_design]

# set_property BITSTREAM.CONFIG.TIMER_CFG        0x4001fbd0  [current_design]
# set_property BITSTREAM.CONFIG.CONFIGFALLBACK   ENABLE      [current_design]


create_clock -name cfg_mclk  -period 12      [get_pins common/STARTUPE3_inst/CFGMCLK]

create_clock -name usb_clk   -period 16      [get_ports usb_clk]

create_clock -name lms_rxclk -period 2.667   [get_ports lml1_clk_m]
create_clock -name lms_txclk -period 2.667   [get_ports lml2_clk_m]

###########################################################
# IO types
###########################################################

set VIO_CMOS_TYPE        LVCMOS18
set VIO_CMOS_LML_DRIVE   12

set VIO_LML1_TYPE        LVCMOS18
# set VIO_LML1_TYPE          SSTL_II_18

set VIO_LML2_TYPE          LVCMOS18





###########################################################
# PCIexpress
###########################################################

# PCI_REF_CLK
set_property PACKAGE_PIN R15 [get_ports sys_clk_n]
set_property PACKAGE_PIN R14 [get_ports sys_clk_p]


##########################################################
# CFG 1.8V (BANK 0)
##########################################################
# set_property IOSTANDARD LVCMOS18 [get_ports {flash_d[*] flash_fcs_b}]
#set_property PACKAGE_PIN K9  [get_ports flash_d[0]]
#set_property PACKAGE_PIN M9  [get_ports flash_d[1]]
#set_property PACKAGE_PIN J9  [get_ports flash_d[2]]
#set_property PACKAGE_PIN N9  [get_ports flash_d[3]]
#set_property PACKAGE_PIN L10 [get_ports flash_fcs_b]


##########################################################
# Not used / 1.8V (BANK 104)
##########################################################

##########################################################
# M.2 GPIOs + RESET 1.8V (BANK 84)
##########################################################
set_property IOSTANDARD LVCMOS18 [get_ports {sys_rst_n}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpio[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {gpled[*]}]

set_property PACKAGE_PIN U7  [get_ports sys_rst_n]

# rename GPIOs
# 0_P   => GPLED1
# 0_N   => GPIO0
# 1_P   => na        => 7
# 1_N   => GPIO2
# 2_P   => GPIO6
# 2_N   => GPLED0
# 3_P   => na        => 8 == HI_SPEED bank
# 3_N   => GPIO1          == HI_SPEED bank
# 5     => GPIO5
# 6     => na        => 9
# n7    => 3
# n8    => 4

set_property PACKAGE_PIN U8       [get_ports gpled[0]]
set_property PACKAGE_PIN T6       [get_ports gpled[1]]
set_property PACKAGE_PIN U6       [get_ports gpio[0]]
## set_property PACKAGE_PIN       [get_ports gpio[1]]
set_property PACKAGE_PIN U4       [get_ports gpio[2]]
# dummy 3 & 4 -- not routed just for comatibility
set_property PACKAGE_PIN R9      [get_ports gpio[3]]
set_property PACKAGE_PIN P9      [get_ports gpio[4]]

set_property PACKAGE_PIN N4       [get_ports gpio[5]]
set_property PACKAGE_PIN T8       [get_ports gpio[6]]
set_property PACKAGE_PIN T4       [get_ports gpio[7]]
## set_property PACKAGE_PIN       [get_ports gpio[8]]
set_property PACKAGE_PIN P4       [get_ports gpio[9]]


##########################################################
# USB PHY 1.8V (BANK 106) + AUX
##########################################################
set_property IOSTANDARD LVCMOS18 [get_ports {usb_d[*] usb_clk usb_dir usb_stp usb_nxt usb_nrst}]
set_property IOSTANDARD LVCMOS18 [get_ports {rxsw txsw intref_sw usb_clk_en lms8_reset_n}]

set_property PACKAGE_PIN G2  [get_ports usb_d[7]]
set_property PACKAGE_PIN F1  [get_ports usb_d[6]]
set_property PACKAGE_PIN G1  [get_ports usb_d[5]]
set_property PACKAGE_PIN E2  [get_ports usb_d[4]]
set_property PACKAGE_PIN E1  [get_ports usb_d[3]]
set_property PACKAGE_PIN D2  [get_ports usb_d[2]]
set_property PACKAGE_PIN D1  [get_ports usb_d[1]]
set_property PACKAGE_PIN C1  [get_ports usb_d[0]]
set_property PACKAGE_PIN D4  [get_ports usb_clk]
set_property PACKAGE_PIN B2  [get_ports usb_stp]
set_property PACKAGE_PIN B3  [get_ports usb_nxt]
set_property PACKAGE_PIN B1  [get_ports usb_dir]
set_property PACKAGE_PIN G3  [get_ports usb_nrst]

set_property PACKAGE_PIN A2 [get_ports usb_clk_en]
set_property PACKAGE_PIN A3 [get_ports intref_sw]
set_property PACKAGE_PIN B4 [get_ports rxsw]
set_property PACKAGE_PIN A4 [get_ports txsw]
set_property PACKAGE_PIN B5 [get_ports lms8_reset_n]


##########################################################
# EXTCLK 1.8V (BANK 66) + AUX
##########################################################
set_property IOSTANDARD LVCMOS18 [get_ports {lml1_txnrx lml2_txnrx fpga_ref}]

set_property PACKAGE_PIN B9 [get_ports lml1_txnrx]
set_property PACKAGE_PIN A9 [get_ports lml2_txnrx]
set_property PACKAGE_PIN B8 [get_ports fpga_ref]


##########################################################
# HIGH SPEED 1.8V (BANK 56)
##########################################################

set_property IOSTANDARD LVCMOS18 [get_ports {lms8_gpio[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {ldoen scl2 sda2 lms_reset_n}]
set_property IOSTANDARD LVCMOS18 [get_ports {lms_ldo_en lms_rxen lms_txen gpledint}]
set_property IOSTANDARD LVCMOS18 [get_ports {lms_sdio lms_sdo lms_sclk lms_sen lms8_sen}]

set_property PACKAGE_PIN J15 [get_ports lms_reset_n]
set_property PACKAGE_PIN H16 [get_ports scl2]
set_property PACKAGE_PIN H17 [get_ports sda2]
set_property PACKAGE_PIN H15 [get_ports ldoen]

set_property PACKAGE_PIN G14  [get_ports lms8_gpio[3]]
set_property PACKAGE_PIN H14  [get_ports lms8_gpio[2]]
set_property PACKAGE_PIN G17  [get_ports lms8_gpio[1]]
set_property PACKAGE_PIN D16  [get_ports lms8_gpio[0]]

set_property PACKAGE_PIN F15 [get_ports lms_rxen]
set_property PACKAGE_PIN D14 [get_ports lms_txen]
set_property PACKAGE_PIN B13 [get_ports lms_ldo_en]
set_property PACKAGE_PIN K15 [get_ports gpledint]

set_property PACKAGE_PIN D17 [get_ports lms_sdio]
set_property PACKAGE_PIN F16 [get_ports lms_sdo]
set_property PACKAGE_PIN E17 [get_ports lms_sclk]
set_property PACKAGE_PIN G15 [get_ports lms_sen]
set_property PACKAGE_PIN B12 [get_ports lms8_sen]


# set_property IOSTANDARD $VIO_LML1_TYPE [get_ports {lml1_clk_f}]
# set_property IOSTANDARD $VIO_LML1_TYPE [get_ports {lml1_diq[*] lml1_iqsel lml1_clk_m}]

# set_property IN_TERM UNTUNED_SPLIT_50  [get_ports {lml1_diq[*] lml1_iqsel lml1_clk_m}]
# set_property IBUF_LOW_PWR false        [get_ports {lml1_clk_m}]
# set_property IBUF_LOW_PWR true         [get_ports {lml1_diq[*] lml1_iqsel}]
# See UG571


set_property IOSTANDARD       HSTL_I_18  [get_ports {lml1_clk_f}]
set_property OUTPUT_IMPEDANCE RDRV_48_48 [get_ports {lml1_clk_f}]
set_property IOSTANDARD       HSTL_I_18  [get_ports {lml1_diq[*] lml1_iqsel lml1_clk_m}]
set_property ODT              RTT_48     [get_ports {lml1_diq[*] lml1_iqsel lml1_clk_m}]

set_property IBUF_LOW_PWR     false      [get_ports {lml1_clk_m}]
set_property INTERNAL_VREF    0.90       [get_iobanks 56]


set_property PACKAGE_PIN G11 [get_ports lml1_diq[0]]
set_property PACKAGE_PIN G12 [get_ports lml1_diq[1]]
set_property PACKAGE_PIN H11 [get_ports lml1_diq[2]]
set_property PACKAGE_PIN H12 [get_ports lml1_diq[3]]
set_property PACKAGE_PIN D11 [get_ports lml1_diq[4]]
set_property PACKAGE_PIN A10 [get_ports lml1_diq[5]]
set_property PACKAGE_PIN D10 [get_ports lml1_diq[6]]
set_property PACKAGE_PIN B10 [get_ports lml1_diq[7]]
set_property PACKAGE_PIN D12 [get_ports lml1_diq[8]]
set_property PACKAGE_PIN E13 [get_ports lml1_diq[9]]
set_property PACKAGE_PIN A11 [get_ports lml1_diq[10]]
set_property PACKAGE_PIN B11 [get_ports lml1_diq[11]]

set_property PACKAGE_PIN E12 [get_ports lml1_iqsel]
set_property PACKAGE_PIN E10 [get_ports lml1_clk_m]
set_property PACKAGE_PIN E14 [get_ports lml1_clk_f]



set_property IOSTANDARD       HSTL_I_18  [get_ports {lml2_clk_f lml2_diq[*] lml2_iqsel}]
set_property OUTPUT_IMPEDANCE RDRV_48_48 [get_ports {lml2_clk_f lml2_diq[*] lml2_iqsel}]
set_property IOSTANDARD       HSTL_I_18  [get_ports {lml2_clk_m}]
set_property ODT              RTT_48     [get_ports {lml2_clk_m}]

set_property IBUF_LOW_PWR     false      [get_ports {lml2_clk_m}]

#set_property IOSTANDARD $VIO_LML2_TYPE [get_ports {lml2_clk_f}]
#set_property IOSTANDARD $VIO_LML2_TYPE [get_ports {lml2_diq[*] lml2_iqsel lml2_clk_m}]
# set_property IOSTANDARD $VIO_LML1_TYPE [get_ports {lml2_clk_m}]
#set_property IN_TERM UNTUNED_SPLIT_50  [get_ports {lml2_clk_m}]
#set_property IBUF_LOW_PWR false        [get_ports {lml2_clk_m}]


set_property PACKAGE_PIN B17 [get_ports lml2_diq[0]]
set_property PACKAGE_PIN D15 [get_ports lml2_diq[1]]
set_property PACKAGE_PIN A15 [get_ports lml2_diq[2]]
set_property PACKAGE_PIN A16 [get_ports lml2_diq[3]]
set_property PACKAGE_PIN C15 [get_ports lml2_diq[4]]
set_property PACKAGE_PIN C16 [get_ports lml2_diq[5]]
set_property PACKAGE_PIN D13 [get_ports lml2_diq[6]]
set_property PACKAGE_PIN B16 [get_ports lml2_diq[7]]
set_property PACKAGE_PIN A14 [get_ports lml2_diq[8]]
set_property PACKAGE_PIN B15 [get_ports lml2_diq[9]]
set_property PACKAGE_PIN C13 [get_ports lml2_diq[10]]
set_property PACKAGE_PIN A13 [get_ports lml2_diq[11]]

set_property PACKAGE_PIN B14 [get_ports lml2_iqsel]
set_property PACKAGE_PIN E15 [get_ports lml2_clk_m]
set_property PACKAGE_PIN C12 [get_ports lml2_clk_f]




 set_property PACKAGE_PIN  K13     [get_ports gpio[1]]
 set_property PACKAGE_PIN  K12     [get_ports gpio[8]]




##################################################################################

set_false_path -from [get_ports sys_rst_n]
create_clock -period 10.000 -name sys_clk [get_ports sys_clk_p]

#create_generated_clock -name user_clk_pcie_core -divide_by 1 -master_clock sys_clk  [get_pins common/pcie4c_uscale_plus_0_i/user_clk]
#create_generated_clock -name user_clk_pcie_core -source [get_pins common/pcie4c_uscale_plus_0_i/sys_clk] [get_pins common/pcie4c_uscale_plus_0_i/user_clk]
set pcie_clk [get_clocks -of_objects [get_pins common/pcie4c_uscale_plus_0_i/user_clk]]


#set_clock_groups -name async_lml1     -asynchronous -group [get_clocks -include_generated_clocks lml1_clk_m]    -group [get_clocks {user_clk usb_clk}]
#set_clock_groups -name async_lml2     -asynchronous -group [get_clocks -include_generated_clocks lml2_clk_m]    -group [get_clocks {user_clk usb_clk}]

set_clock_groups -name async_usb_mclk -asynchronous -group [get_clocks { usb_clk } ]             -group [get_clocks cfg_mclk]
set_clock_groups -name async_aux2     -asynchronous -group [get_clocks { usb_clk cfg_mclk} ]     -group $pcie_clk

create_generated_clock -name uclk1mux -divide_by 1 -add -master_clock $pcie_clk          -source [get_pins common/clkmux_i/I0] [get_pins common/clkmux_i/O]
create_generated_clock -name uclk2mux -divide_by 1 -add -master_clock usb_clk            -source [get_pins common/clkmux_i/I1] [get_pins common/clkmux_i/O]

set_clock_groups -logically_exclusive -group [ get_clocks uclk1mux ] -group [ get_clocks uclk2mux ]

set_clock_groups -name async_muxclkc_mclk -asynchronous -group [get_clocks { uclk1mux uclk2mux } ]   -group [get_clocks cfg_mclk]
#set_clock_groups -name async_muxclkc_lml1 -asynchronous -group [get_clocks { uclk1mux uclk2mux } ]   -group [get_clocks -include_generated_clocks lml1_clk_m]
#set_clock_groups -name async_muxclkc_lml2 -asynchronous -group [get_clocks { uclk1mux uclk2mux } ]   -group [get_clocks -include_generated_clocks lml2_clk_m]

set_clock_groups -name async_aux3     -asynchronous -group [get_clocks { uclk2mux } ]     -group $pcie_clk
set_clock_groups -name async_aux4     -asynchronous -group [get_clocks { uclk1mux } ]     -group [get_clocks usb_clk]

set_clock_groups -name async_mux5     -asynchronous -group [get_clocks { uclk2mux } ]     -group [get_clocks  -include_generated_clocks  lms_rxclk]
set_clock_groups -name async_mux6     -asynchronous -group [get_clocks { uclk1mux } ]     -group [get_clocks  -include_generated_clocks  lms_rxclk]
set_clock_groups -name async_mux7     -asynchronous -group [get_clocks { uclk2mux } ]     -group [get_clocks  -include_generated_clocks  lms_txclk]
set_clock_groups -name async_mux8     -asynchronous -group [get_clocks { uclk1mux } ]     -group [get_clocks  -include_generated_clocks  lms_txclk]

# PCIe core
set_clock_groups -name async18 -asynchronous -group [get_clocks sys_clk] -group [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ *gen_channel_container[0].*gen_gt*e4_channel_inst[3].GT*E4_CHANNEL_PRIM_INST/TXOUTCLK}]]
set_clock_groups -name async19 -asynchronous -group [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ *gen_channel_container[0].*gen_gt*e4_channel_inst[3].GT*E4_CHANNEL_PRIM_INST/TXOUTCLK}]] -group [get_clocks sys_clk]
set_clock_groups -name async24 -asynchronous -group [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ common/pcie4*_uscale_plus_0_i/inst/pcie4*_uscale_plus_0*_gt_top_i/diablo_gt.diablo_gt_phy_wrapper/phy_clk_i/bufg_gt_intclk/O}]] -group [get_clocks sys_clk]

# sys_clk vs pclk
set_clock_groups -name async1 -asynchronous -group [get_clocks {sys_clk}] -group [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ common/pcie4*_uscale_plus_0_i/inst/pcie4*_uscale_plus_0*_gt_top_i/diablo_gt.diablo_gt_phy_wrapper/phy_clk_i/bufg_gt_pclk/O}]]
set_clock_groups -name async2 -asynchronous -group [get_clocks -of_objects [get_pins -hierarchical -filter {NAME =~ common/pcie4*_uscale_plus_0_i/inst/pcie4*_uscale_plus_0*_gt_top_i/diablo_gt.diablo_gt_phy_wrapper/phy_clk_i/bufg_gt_pclk/O}]] -group [get_clocks {sys_clk}]


set_clock_groups -name lms_rx_tx  -asynchronous -group [get_clocks  -include_generated_clocks  lms_rxclk]     -group [get_clocks  -include_generated_clocks  lms_txclk]

# We need a free running clock for proper debug hub initialization
connect_debug_port dbg_hub/clk [get_nets common/user_clk_pcie]

#set_clock_groups -name async_lml1p    -asynchronous -group [get_clocks -include_generated_clocks lml1_clk_m]    -group [get_clocks {user_clk_pcie_core usb_clk}]
#set_clock_groups -name async_lml2p    -asynchronous -group [get_clocks -include_generated_clocks lml2_clk_m]    -group [get_clocks {user_clk_pcie_core usb_clk}]


#set_clock_groups -name async_aux      -asynchronous -group [get_clocks { usb_clk cfg_mclk} ]     -group [get_clocks user_clk]
#set_clock_groups -name async_aux2     -asynchronous -group [get_clocks { usb_clk cfg_mclk} ]     -group [get_clocks user_clk_pcie_core]
#set_clock_groups -name async_usb_mclk -asynchronous -group [get_clocks { usb_clk } ]             -group [get_clocks cfg_mclk]





#set_property CLOCK_REGION X0Y0 [get_nets usb_clk]
#set_property CLOCK_REGION X0Y0 [get_cells common/usb_clk_bufg]

# set_property CLOCK_DEDICATED_ROUTE ANY_CMT_REGION [get_nets usb_clk]
#set_property CLOCK_DEDICATED_ROUTE ANY_CMT_REGION [get_nets lml1_clk_m]
#set_property CLOCK_DEDICATED_ROUTE ANY_CMT_REGION [get_nets lml2_clk_m]

# set_property CLOCK_REGION X0Y2 [get_nets lml2_clk_m]
# set_property LOC MMCM_X0Y2 [get_cells common/phy/mmcm_gen.mmcme2]
# set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_pins -hier -filter {NAME =~ */phy/mmcm_gen.mmcme2/CLKIN2}]

#[get_pins -hier -filter {NAME =~ */phy/mmcm_gen.mmcme2/CLKIN1}]
#set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_pins -hier -filter {NAME =~ */phy/mmcm_gen.mmcme2/CLKIN2}]
# FIXUPS!!!!
# set_property CLOCK_DEDICATED_ROUTE FALSE    [get_nets common/phy_clk_ibuf/O]
#set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets usb_clk_IBUF_inst/O]
#set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets common/phy/phy_lml_mclk_in2]
#set_property CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN [get_nets common/phy/phy_mclk_ibuf2/O]
#
#set_property CLOCK_DEDICATED_ROUTE ANY_CMT_COLUMN [get_nets common/phy/phy_lml_mclk_in]
