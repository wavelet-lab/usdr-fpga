set_property BITSTREAM.CONFIG.UNUSEDPIN Pulldown [current_design]
set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN Disable [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 66 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]

#set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
#set_property BITSTREAM.config.SPI_opcode 0x6B [current_design ]

## 4-wire mode doesn't work!!!
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 1 [current_design]
set_property BITSTREAM.config.SPI_opcode 0x0B [current_design ]

set_property CFGBVS GND [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]

set_property BITSTREAM.CONFIG.USR_ACCESS         timestamp [current_design]

# Bitstream configuration GOLDEN
 set_property BITSTREAM.CONFIG.NEXT_CONFIG_ADDR   0x1C0000   [current_design]
 set_property BITSTREAM.CONFIG.NEXT_CONFIG_REBOOT ENABLE     [current_design]
 set_property BITSTREAM.CONFIG.TIMER_CFG          0x002fbd0 [current_design]
 set_property BITSTREAM.CONFIG.CONFIGFALLBACK     ENABLE     [current_design]

# operational.
# set_property BITSTREAM.CONFIG.TIMER_CFG        0x4001fbd0  [current_design]
# set_property BITSTREAM.CONFIG.CONFIGFALLBACK   ENABLE      [current_design]

###########################################################
# PCIexpress (1.8V) Pinout and BANK14 (1.8V)
###########################################################

# PCI_REF_CLK
set_property PACKAGE_PIN A8             [get_ports sys_clk_n]
set_property PACKAGE_PIN B8             [get_ports sys_clk_p]


set_property IOSTANDARD LVCMOS18        [get_ports {flash_d[*] flash_fcs_b}]
set_property IOSTANDARD LVCMOS18        [get_ports {scl sda}]
set_property IOSTANDARD LVCMOS18        [get_ports {lms_rst_n lms_sdio lms_sck lms_sdo lms_sen}]
set_property IOSTANDARD LVCMOS18        [get_ports {txen txsw rxsw rxen rxmix_en rxclk_f txclk_ref txclk}]
set_property IOSTANDARD LVCMOS18        [get_ports dcen]
set_property IOSTANDARD LVCMOS18        [get_ports {rxd[*] txd[*] rxiqsel txiqsel}]
set_property IOSTANDARD LVCMOS18        [get_ports usb_nrst]

set_property PULLUP      true           [get_ports {flash_d[2] flash_d[3]}]
set_property PULLUP      true           [get_ports {lms_sdo}]
set_property PULLUP      true           [get_ports {board_rev[0]}]

set_property IOSTANDARD LVCMOS18        [get_ports board_rev[0]]
set_property IOSTANDARD LVCMOS18        [get_ports osc_buf_en]

set_property PACKAGE_PIN D17            [get_ports usb_nrst]
set_property PACKAGE_PIN D18            [get_ports flash_d[0]]
set_property PACKAGE_PIN D19            [get_ports flash_d[1]]
set_property PACKAGE_PIN G18            [get_ports flash_d[2]]
set_property PACKAGE_PIN F18            [get_ports flash_d[3]]
set_property PACKAGE_PIN E19            [get_ports osc_buf_en ]
# floating in rev1, tied to GND in rev2
set_property PACKAGE_PIN E18            [get_ports board_rev[0]]
set_property PACKAGE_PIN H19            [get_ports rxd[10]]
set_property PACKAGE_PIN G19            [get_ports rxd[11]]
set_property PACKAGE_PIN H17            [get_ports rxsw]
set_property PACKAGE_PIN G17            [get_ports rxmix_en]
set_property PACKAGE_PIN K19            [get_ports flash_fcs_b]
set_property PACKAGE_PIN J19            [get_ports rxd[8]]
set_property PACKAGE_PIN J17            [get_ports rxen]
set_property PACKAGE_PIN J18            [get_ports rxd[9]]
set_property PACKAGE_PIN L18            [get_ports rxd[5]]
set_property PACKAGE_PIN K18            [get_ports rxd[7]]
set_property PACKAGE_PIN N18            [get_ports rxd[4]]
set_property PACKAGE_PIN N19            [get_ports rxd[2]]
set_property PACKAGE_PIN P19            [get_ports rxd[0]]
set_property PACKAGE_PIN R19            [get_ports txclk]
set_property PACKAGE_PIN M18            [get_ports rxd[6]]
set_property PACKAGE_PIN M19            [get_ports rxd[3]]
set_property PACKAGE_PIN L17            [get_ports lms_rst_n]
set_property PACKAGE_PIN K17            [get_ports dcen]
set_property PACKAGE_PIN N17            [get_ports rxclk_f]
set_property PACKAGE_PIN P17            [get_ports txclk_ref]
set_property PACKAGE_PIN P18            [get_ports rxd[1]]
set_property PACKAGE_PIN R18            [get_ports rxiqsel]
set_property PACKAGE_PIN U19            [get_ports txd[1]]
set_property PACKAGE_PIN V19            [get_ports txd[3]]
set_property PACKAGE_PIN W18            [get_ports txd[6]]
set_property PACKAGE_PIN W19            [get_ports txd[5]]
set_property PACKAGE_PIN T17            [get_ports lms_sck]
set_property PACKAGE_PIN T18            [get_ports txiqsel]
set_property PACKAGE_PIN U17            [get_ports txd[2]]
set_property PACKAGE_PIN U18            [get_ports txd[0]]
set_property PACKAGE_PIN V16            [get_ports txd[8]]
set_property PACKAGE_PIN V17            [get_ports txd[4]]
set_property PACKAGE_PIN W16            [get_ports txd[10]]
set_property PACKAGE_PIN W17            [get_ports txd[7]]
set_property PACKAGE_PIN V15            [get_ports txd[9]]
set_property PACKAGE_PIN W15            [get_ports txd[11]]
set_property PACKAGE_PIN W13            [get_ports txsw]
set_property PACKAGE_PIN W14            [get_ports sda]
set_property PACKAGE_PIN U15            [get_ports lms_sdio]
set_property PACKAGE_PIN U16            [get_ports lms_sdo]
set_property PACKAGE_PIN V13            [get_ports txen]
set_property PACKAGE_PIN V14            [get_ports scl]
set_property PACKAGE_PIN U14            [get_ports lms_sen]


##########################################################
# BANK 34 (tied to VCCINT)
##########################################################

##########################################################
# BANK 35 (GPIO)
##########################################################
set_property IOSTANDARD LVCMOS18        [get_ports {gpio[*] gpled[*] gpledint}]

set_property PACKAGE_PIN G3             [get_ports gpledint]
set_property PACKAGE_PIN G2             [get_ports gpio[9]]
set_property PACKAGE_PIN H2             [get_ports gpio[8]]
set_property PACKAGE_PIN J2             [get_ports gpio[10]]
set_property PACKAGE_PIN H1             [get_ports gpio[7]]
set_property PACKAGE_PIN J1             [get_ports gpio[6]]
# set_property PACKAGE_PIN K2             [get_ports ??? ]
set_property PACKAGE_PIN L2             [get_ports gpio[3]]
set_property PACKAGE_PIN L1             [get_ports gpio[5]]
# set_property PACKAGE_PIN J3             [get_ports ??? ]
set_property PACKAGE_PIN K3             [get_ports gpio[11]]
set_property PACKAGE_PIN L3             [get_ports gpio[12]]
set_property PACKAGE_PIN M3             [get_ports gpio[13]]
set_property PACKAGE_PIN M2             [get_ports gpio[1]]
set_property PACKAGE_PIN M1             [get_ports gpio[4]]
set_property PACKAGE_PIN N2             [get_ports gpled[0]]
set_property PACKAGE_PIN N1             [get_ports gpio[2]]
set_property PACKAGE_PIN N3             [get_ports gpio[14]]
set_property PACKAGE_PIN P3             [get_ports gpio[0]]
set_property PACKAGE_PIN P1             [get_ports gpled[1]]

# fake port for TOP module
set_property PULLUP      true           [get_ports {fpga_ref}]
set_property IOSTANDARD LVCMOS18        [get_ports fpga_ref]
set_property PACKAGE_PIN K2             [get_ports fpga_ref]

# fake port for TOP module
set_property PULLUP      true           [get_ports {board_rev[1]}]
set_property IOSTANDARD LVCMOS18        [get_ports board_rev[1]]
set_property PACKAGE_PIN J3             [get_ports board_rev[1]]


# ENOSC on breakout board, we need external clock in rev2
#set_property PULLUP      true           [get_ports gpio[5]]

##########################################################
# BANK 16 (1.8V USB)
##########################################################
set_property IOSTANDARD LVCMOS18        [get_ports {usb_d[*] usb_clk usb_stp usb_dir usb_nxt}]

set_property PACKAGE_PIN A14            [get_ports usb_d[7]]
set_property PACKAGE_PIN A15            [get_ports usb_d[6]]
set_property PACKAGE_PIN C15            [get_ports usb_d[5]]
set_property PACKAGE_PIN B15            [get_ports usb_d[3]]
set_property PACKAGE_PIN A16            [get_ports usb_d[4]]
set_property PACKAGE_PIN A17            [get_ports usb_d[2]]
set_property PACKAGE_PIN C16            [get_ports usb_clk]
set_property PACKAGE_PIN B16            [get_ports usb_d[1]]
set_property PACKAGE_PIN C17            [get_ports usb_stp]
set_property PACKAGE_PIN B17            [get_ports usb_nxt]
set_property PACKAGE_PIN B18            [get_ports usb_dir]
set_property PACKAGE_PIN A18            [get_ports usb_d[0]]


#########################################################
set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets {txclk_ref_IBUF}]

set_property LOC MMCME2_ADV_X1Y1 [get_cells xilinx_pci_mmcm/mmcm_i]

set_false_path -to [get_pins {xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/S0}]
set_false_path -to [get_pins {xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/S1}]

create_clock -name cfg_mclk -period 12  [get_pins STARTUPE2_inst/CFGMCLK]

create_clock -name usb_clk   -period 16 [get_ports usb_clk]
create_clock -name lms_rxclk -period 5  [get_ports rxclk_f]
create_clock -name lms_txclk -period 5  [get_ports txclk_ref]

create_clock -name sysref_clk -period 10 [get_ports sys_clk_p]


#create_generated_clock -name clk_125mhz_x0y0 [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT0]
#create_generated_clock -name clk_250mhz_x0y0 [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT1]
#create_generated_clock -name clk_dsp         [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT6]
#
#create_generated_clock -name clk_125mhz_mux_x0y0 \
#                        -source [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/I0] \
#                        -divide_by 1 \
#                        [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/O]

#create_generated_clock -name clk_250mhz_mux_x0y0 \
#                        -source [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/I1] \
#                        -divide_by 1 -add -master_clock [get_clocks clk_250mhz_x0y0] \
#                        [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/O]


#create_generated_clock -name clk_250mhz_mux_x0y0 \
#                        -source [get_pins pclk_i1_bufgctrl.pclk_i1/I1] \
#                        -divide_by 1 -add -master_clock [get_clocks -of [get_pins pclk_i1_bufgctrl.pclk_i1/I1]] \
#                        [get_pins pclk_i1_bufgctrl.pclk_i1/O]
                        
#set_clock_groups -name pcieclkmux -physically_exclusive -group clk_125mhz_mux_x0y0 -group clk_250mhz_mux_x0y0

#create_generated_clock -name userclk1 [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT2]

#create_generated_clock -name clk_125mhz_x0y0 [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT0]
#create_generated_clock -name clk_250mhz_x0y0 [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT1]
# create_generated_clock -name clk_dsp         [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT6]

create_generated_clock -name clk_125mhz_x0y0_e \
                       -master_clock [get_clocks -of [get_pins xilinx_pci_mmcm/mmcm_i/CLKIN2]] \
                       [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT0]
create_generated_clock -name clk_125mhz_x0y0_u \
                       -master_clock [get_clocks usb_clk] \
                       [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT0]
set_clock_groups -name clk_125mhz_x0y0 -physically_exclusive -group clk_125mhz_x0y0_e -group clk_125mhz_x0y0_u
create_generated_clock -name clk_250mhz_x0y0_e \
                       -master_clock [get_clocks -of [get_pins xilinx_pci_mmcm/mmcm_i/CLKIN2]] \
                       [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT1]
create_generated_clock -name clk_250mhz_x0y0_u \
                       -master_clock [get_clocks usb_clk] \
                       [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT1]
set_clock_groups -name clk_250mhz_x0y0 -physically_exclusive -group clk_250mhz_x0y0_e -group clk_250mhz_x0y0_u



create_generated_clock -name clk_125mhz_mux_x0y0 \
                        -source [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/I0] \
                        -divide_by 1 \
                        [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/O]

create_generated_clock -name clk_250mhz_mux_x0y0_e \
                        -source [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/I1] \
                        -divide_by 1  -add -master_clock [get_clocks clk_250mhz_x0y0_e] \
                        [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/O]

create_generated_clock -name clk_250mhz_mux_x0y0_u \
                        -source [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/I1] \
                        -divide_by 1  -add -master_clock [get_clocks clk_250mhz_x0y0_u] \
                        [get_pins xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/O]

#create_generated_clock -name clk_250mhz_mux_x0y0 \
#                        -source [get_pins pclk_i1_bufgctrl.pclk_i1/I1] \
#                        -divide_by 1 -add -master_clock [get_clocks -of [get_pins pclk_i1_bufgctrl.pclk_i1/I1]] \
#                        [get_pins pclk_i1_bufgctrl.pclk_i1/O]
                        
#set_clock_groups -name pcieclkmux -physically_exclusive -group clk_125mhz_mux_x0y0 -group clk_250mhz_mux_x0y0_e -group clk_250mhz_mux_x0y0_u
set_clock_groups -name pcieclkmux0 -physically_exclusive -group  [get_clocks {clk_125mhz_mux_x0y0 clk_250mhz_mux_x0y0_u}]
set_clock_groups -name pcieclkmux1 -physically_exclusive -group  [get_clocks {clk_125mhz_mux_x0y0 clk_250mhz_mux_x0y0_e}] 
set_clock_groups -name pcieclkmux2 -physically_exclusive -group  [get_clocks {clk_250mhz_mux_x0y0_u clk_250mhz_mux_x0y0_e}] 

#create_generated_clock -name userclk1 [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT2]

create_generated_clock -name userclk1 \
                        -master_clock [get_clocks -of [get_pins xilinx_pci_mmcm/mmcm_i/CLKIN2]] \
                        [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT2]
                        
create_generated_clock -name userclk1_u \
                        -master_clock [get_clocks usb_clk] \
                        [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT2]                       
                                     
set_clock_groups -name userclkmux -physically_exclusive -group userclk1 -group userclk1_u
                    
create_generated_clock -name clk_dsp_e \
                       -master_clock [get_clocks -of [get_pins xilinx_pci_mmcm/mmcm_i/CLKIN2]] \
                       [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT6]
create_generated_clock -name clk_dsp_u \
                       -master_clock [get_clocks usb_clk] \
                       [get_pins xilinx_pci_mmcm/mmcm_i/CLKOUT6]
set_clock_groups -name clk_dsp -physically_exclusive -group clk_dsp_e -group clk_dsp_u

#########################################################


set_false_path -from [get_clocks cfg_mclk] -to [get_clocks clk_125mhz_x0y0_e]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks clk_125mhz_x0y0_u]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks clk_250mhz_x0y0_e]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks clk_250mhz_x0y0_u]


#########################################################

# RX/TX <-> DSP

set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks lms_rxclk]
set_false_path -from [get_clocks lms_rxclk] -to [get_clocks clk_dsp_e]

set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks lms_txclk]
set_false_path -from [get_clocks lms_txclk] -to [get_clocks clk_dsp_e]

set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks lms_rxclk]
set_false_path -from [get_clocks lms_rxclk] -to [get_clocks clk_dsp_u]

set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks lms_txclk]
set_false_path -from [get_clocks lms_txclk] -to [get_clocks clk_dsp_u]

# MCFG <-> USERCLK

set_false_path -from [get_clocks userclk1] -to [get_clocks cfg_mclk]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks userclk1]

set_false_path -from [get_clocks userclk1_u] -to [get_clocks cfg_mclk]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks userclk1_u]

# RX/TX <-> USERCLK

set_false_path -from [get_clocks userclk1] -to [get_clocks lms_rxclk]
set_false_path -from [get_clocks lms_rxclk] -to [get_clocks userclk1]

set_false_path -from [get_clocks userclk1] -to [get_clocks lms_txclk]
set_false_path -from [get_clocks lms_txclk] -to [get_clocks userclk1]

set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_rxclk]
set_false_path -from [get_clocks lms_rxclk] -to [get_clocks userclk1_u]

set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_txclk]
set_false_path -from [get_clocks lms_txclk] -to [get_clocks userclk1_u]


# DSP <-> USERCLK

set_false_path -from [get_clocks userclk1] -to [get_clocks clk_dsp_e]
set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks userclk1]

set_false_path -from [get_clocks userclk1_u] -to [get_clocks clk_dsp_e]
set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks userclk1_u]

set_false_path -from [get_clocks userclk1] -to [get_clocks clk_dsp_u]
set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks userclk1]

set_false_path -from [get_clocks userclk1_u] -to [get_clocks clk_dsp_u]
set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks userclk1_u]


# USB CLK
set_false_path -from [get_clocks userclk1] -to [get_clocks usb_clk]
set_false_path -from [get_clocks usb_clk] -to [get_clocks userclk1]

set_false_path -from [get_clocks userclk1_u] -to [get_clocks usb_clk]
set_false_path -from [get_clocks usb_clk] -to [get_clocks userclk1_u]

set_false_path -from [get_clocks usb_clk] -to [get_clocks cfg_mclk]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks usb_clk]

