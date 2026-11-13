#############################
# clocks for Xsdr famaliy

set_false_path -from [get_ports lml1_txnrx]

create_clock -name cfg_mclk  -period 12    [get_pins common/STARTUPE2_inst/CFGMCLK]

create_clock -name usb_clk   -period 16    [get_ports usb_clk]

create_clock -name lms_rxclk -period 2.667   [get_ports lml1_clk_m]
create_clock -name lms_txclk -period 2.667   [get_ports lml2_clk_m]

create_clock -name sysref_clk -period 10 [get_ports sys_clk_p]


set_property LOC MMCME2_ADV_X1Y1 [get_cells common/xilinx_pci_mmcm/mmcm_i]

set_false_path -to [get_pins {common/xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/S0}]
set_false_path -to [get_pins {common/xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/S1}]

#create_generated_clock -name clk_125mhz_x0y0 [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT0]
#create_generated_clock -name clk_250mhz_x0y0 [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT1]
#create_generated_clock -name clk_dsp         [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT6]

create_generated_clock -name clk_125mhz_x0y0_e \
                       -master_clock [get_clocks -of [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKIN2]] \
                       [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT0]
create_generated_clock -name clk_125mhz_x0y0_u \
                       -master_clock [get_clocks usb_clk] \
                       [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT0]
set_clock_groups -name clk_125mhz_x0y0 -physically_exclusive -group clk_125mhz_x0y0_e -group clk_125mhz_x0y0_u
create_generated_clock -name clk_250mhz_x0y0_e \
                       -master_clock [get_clocks -of [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKIN2]] \
                       [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT1]
create_generated_clock -name clk_250mhz_x0y0_u \
                       -master_clock [get_clocks usb_clk] \
                       [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT1]
set_clock_groups -name clk_250mhz_x0y0 -physically_exclusive -group clk_250mhz_x0y0_e -group clk_250mhz_x0y0_u



create_generated_clock -name clk_125mhz_mux_x0y0 \
                        -source [get_pins common/xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/I0] \
                        -divide_by 1 \
                        [get_pins common/xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/O]

create_generated_clock -name clk_250mhz_mux_x0y0_e \
                        -source [get_pins common/xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/I1] \
                        -divide_by 1  -add -master_clock [get_clocks clk_250mhz_x0y0_e] \
                        [get_pins common/xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/O]

create_generated_clock -name clk_250mhz_mux_x0y0_u \
                        -source [get_pins common/xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/I1] \
                        -divide_by 1  -add -master_clock [get_clocks clk_250mhz_x0y0_u] \
                        [get_pins common/xilinx_pci_mmcm/pclk_i1_bufgctrl.pclk_i1/O]

#create_generated_clock -name clk_250mhz_mux_x0y0 \
#                        -source [get_pins common/pclk_i1_bufgctrl.pclk_i1/I1] \
#                        -divide_by 1 -add -master_clock [get_clocks -of [get_pins common/pclk_i1_bufgctrl.pclk_i1/I1]] \
#                        [get_pins common/pclk_i1_bufgctrl.pclk_i1/O]

#set_clock_groups -name pcieclkmux -physically_exclusive -group clk_125mhz_mux_x0y0 -group clk_250mhz_mux_x0y0_e -group clk_250mhz_mux_x0y0_u
set_clock_groups -name pcieclkmux0 -physically_exclusive -group  [get_clocks {clk_125mhz_mux_x0y0 clk_250mhz_mux_x0y0_u}]
set_clock_groups -name pcieclkmux1 -physically_exclusive -group  [get_clocks {clk_125mhz_mux_x0y0 clk_250mhz_mux_x0y0_e}]
set_clock_groups -name pcieclkmux2 -physically_exclusive -group  [get_clocks {clk_250mhz_mux_x0y0_u clk_250mhz_mux_x0y0_e}]



create_generated_clock -name userclk1 \
                        -master_clock [get_clocks -of [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKIN2]] \
                        [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT2]

create_generated_clock -name userclk1_u \
                        -master_clock [get_clocks usb_clk] \
                        [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT2]
set_clock_groups -name userclkmux -physically_exclusive -group userclk1 -group userclk1_u


create_generated_clock -name clk_dsp_e \
                       -master_clock [get_clocks -of [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKIN2]] \
                       [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT6]
create_generated_clock -name clk_dsp_u \
                       -master_clock [get_clocks usb_clk] \
                       [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT6]
set_clock_groups -name clk_dsp -physically_exclusive -group clk_dsp_e -group clk_dsp_u


create_generated_clock -name clk_200mhz_e \
                       -master_clock [get_clocks -of [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKIN2]] \
                       [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT5]
create_generated_clock -name clk_200mhz_u \
                       -master_clock [get_clocks usb_clk] \
                       [get_pins common/xilinx_pci_mmcm/mmcm_i/CLKOUT5]
set_clock_groups -name clk_200mhz -physically_exclusive -group clk_200mhz_e -group clk_200mhz_u



#########################################################


set_false_path -from [get_clocks cfg_mclk] -to [get_clocks clk_125mhz_x0y0_e]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks clk_125mhz_x0y0_u]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks clk_250mhz_x0y0_e]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks clk_250mhz_x0y0_u]

#create_generated_clock -name lms_txclk_iqsel_io_a -master_clock [get_clocks lms_rxclk]  [get_pins {common/phy/mmcm_gen.mmcme2/CLKOUT0}]
#create_generated_clock -name lms_txclk_iqsel_io_b -master_clock [get_clocks lms_txclk]  [get_pins {common/phy/mmcm_gen.mmcme2/CLKOUT0}]
#set_clock_groups -name lms_txclk_iqsel_io -physically_exclusive -group lms_txclk_iqsel_io_a -group lms_txclk_iqsel_io_b

create_generated_clock -name lms_txclk_io_a -master_clock [get_clocks lms_rxclk]  [get_pins {common/phy/mmcm_gen.mmcme2/CLKOUT1}]
create_generated_clock -name lms_txclk_io_b -master_clock [get_clocks lms_txclk]  [get_pins {common/phy/mmcm_gen.mmcme2/CLKOUT1}]
set_clock_groups -name lms_txclk_io -physically_exclusive -group lms_txclk_io_a -group lms_txclk_io_b

create_generated_clock -name lms_rxclk_io_a -master_clock [get_clocks lms_rxclk]  [get_pins {common/phy/mmcm_gen.mmcme2/CLKOUT2}]
create_generated_clock -name lms_rxclk_io_b -master_clock [get_clocks lms_txclk]  [get_pins {common/phy/mmcm_gen.mmcme2/CLKOUT2}]
set_clock_groups -name lms_rxclk_io -physically_exclusive -group lms_rxclk_io_a -group lms_rxclk_io_b


# -divide_by 2 -add
#create_generated_clock -name lms_txclk_iqsel_a -source [get_pins {common/phy/mmcm_gen.mrcc_div2_iq/I}]  -master_clock [get_clocks lms_txclk_iqsel_io_a] [get_pins {common/phy/mmcm_gen.mrcc_div2_iq/O}]
#create_generated_clock -name lms_txclk_iqsel_b -source [get_pins {common/phy/mmcm_gen.mrcc_div2_iq/I}]  -master_clock [get_clocks lms_txclk_iqsel_io_b] [get_pins {common/phy/mmcm_gen.mrcc_div2_iq/O}]
#set_clock_groups -name lms_txclk_iqsel -physically_exclusive -group lms_txclk_iqsel_a -group lms_txclk_iqsel_b

create_generated_clock -name lms_txclk_bb_a -source [get_pins {common/phy/mmcm_gen.mrcc_div2/mrcc_div2/I}]  -master_clock [get_clocks lms_txclk_io_a] [get_pins {common/phy/mmcm_gen.mrcc_div2/mrcc_div2/O}]
create_generated_clock -name lms_txclk_bb_b -source [get_pins {common/phy/mmcm_gen.mrcc_div2/mrcc_div2/I}]  -master_clock [get_clocks lms_txclk_io_b] [get_pins {common/phy/mmcm_gen.mrcc_div2/mrcc_div2/O}]
set_clock_groups -name lms_txclk_bb -physically_exclusive -group lms_txclk_bb_a -group lms_txclk_bb_b

# TX
#create_generated_clock -name lms_txclk_bb_a -source [get_pins {common/phy/mmcm_gen.mrcc_div2/I}]  -master_clock [get_clocks lms_txclk_iqsel_io_a] [get_pins {common/phy/mmcm_gen.mrcc_div2/O}]
#create_generated_clock -name lms_txclk_bb_b -source [get_pins {common/phy/mmcm_gen.mrcc_div2/I}]  -master_clock [get_clocks lms_txclk_iqsel_io_b] [get_pins {common/phy/mmcm_gen.mrcc_div2/O}]
#set_clock_groups -name lms_txclk_bb -physically_exclusive -group lms_txclk_bb_a -group lms_txclk_bb_b

#common/phy/mmcm_gen.lmle_clk/in_bufio
#common/phy/mmcm_gen.lmle_clk/mrcc_div2
#create_generated_clock -name lms_rxclk_bb_a -source [get_pins {common/phy/mmcm_gen.lmle_mrcc_div2/I}]  -master_clock [get_clocks lms_rxclk_io_a] [get_pins {common/phy/mmcm_gen.lmle_mrcc_div2/O}]
#create_generated_clock -name lms_rxclk_bb_b -source [get_pins {common/phy/mmcm_gen.lmle_mrcc_div2/I}]  -master_clock [get_clocks lms_rxclk_io_b] [get_pins {common/phy/mmcm_gen.lmle_mrcc_div2/O}]

create_generated_clock -name lms_rxclk_bb_a -source [get_pins {common/phy/mmcm_gen.lmle_clk/mrcc_div2/I}]  -master_clock [get_clocks lms_rxclk_io_a] [get_pins {common/phy/mmcm_gen.lmle_clk/mrcc_div2/O}]
create_generated_clock -name lms_rxclk_bb_b -source [get_pins {common/phy/mmcm_gen.lmle_clk/mrcc_div2/I}]  -master_clock [get_clocks lms_rxclk_io_b] [get_pins {common/phy/mmcm_gen.lmle_clk/mrcc_div2/O}]
set_clock_groups -name lms_rxclk_bb -physically_exclusive -group lms_rxclk_bb_a -group lms_rxclk_bb_b


set_false_path -from [get_clocks lms_txclk_iqsel_a]   -to [get_clocks lms_txclk_bb_b]
set_false_path -from [get_clocks lms_txclk_bb_b]   -to [get_clocks lms_txclk_iqsel_a]
set_false_path -from [get_clocks lms_txclk_iqsel_b]   -to [get_clocks lms_txclk_bb_a]
set_false_path -from [get_clocks lms_txclk_bb_a]   -to [get_clocks lms_txclk_iqsel_b]



# global fanout clocks thtough BUFGCTL
#create_generated_clock -name lms_txclk_bb_ag -source [get_pins {common/phy/refbuf/I}]  -master_clock [get_clocks lms_txclk_bb_a] [get_pins {common/phy/refbuf/O}]
#create_generated_clock -name lms_txclk_bb_bg -source [get_pins {common/phy/refbuf/I}]  -master_clock [get_clocks lms_txclk_bb_b] [get_pins {common/phy/refbuf/O}]
#set_clock_groups -name lms_txclk_bb_g -physically_exclusive -group lms_txclk_bb_ag -group lms_txclk_bb_bg


set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks -of [get_pins common/ddr_r/clk/mrcc_div2/O]]
set_false_path -from [get_clocks -of [get_pins common/ddr_r/clk/mrcc_div2/O]] -to [get_clocks clk_dsp_e]
set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks -of [get_pins common/ddr_r/clk/mrcc_div2/O]]
set_false_path -from [get_clocks -of [get_pins common/ddr_r/clk/mrcc_div2/O]] -to [get_clocks clk_dsp_u]

set_false_path -from [get_clocks userclk1]   -to [get_clocks -of [get_pins common/ddr_r/clk/mrcc_div2/O]]
set_false_path -from [get_clocks userclk1_u] -to [get_clocks -of [get_pins common/ddr_r/clk/mrcc_div2/O]]
set_false_path -from [get_clocks -of [get_pins common/ddr_r/clk/mrcc_div2/O]] -to [get_clocks userclk1]
set_false_path -from [get_clocks -of [get_pins common/ddr_r/clk/mrcc_div2/O]] -to [get_clocks userclk1_u]



set_false_path -from [get_clocks userclk1]   -to [get_clocks lms_rxclk_bb_a]
set_false_path -from [get_clocks userclk1]   -to [get_clocks lms_rxclk_bb_b]
set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_rxclk_bb_a]
set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_rxclk_bb_b]
set_false_path -from [get_clocks lms_rxclk_bb_a] -to [get_clocks userclk1]
set_false_path -from [get_clocks lms_rxclk_bb_b] -to [get_clocks userclk1]
set_false_path -from [get_clocks lms_rxclk_bb_a] -to [get_clocks userclk1_u]
set_false_path -from [get_clocks lms_rxclk_bb_b] -to [get_clocks userclk1_u]

set_false_path -from [get_clocks userclk1]   -to [get_clocks lms_txclk_bb_a]
set_false_path -from [get_clocks userclk1]   -to [get_clocks lms_txclk_bb_b]
set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_txclk_bb_a]
set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_txclk_bb_b]
set_false_path -from [get_clocks lms_txclk_bb_a] -to [get_clocks userclk1]
set_false_path -from [get_clocks lms_txclk_bb_b] -to [get_clocks userclk1]
set_false_path -from [get_clocks lms_txclk_bb_a] -to [get_clocks userclk1_u]
set_false_path -from [get_clocks lms_txclk_bb_b] -to [get_clocks userclk1_u]

set_false_path -from [get_clocks userclk1]   -to [get_clocks lms_txclk_iqsel_a]
set_false_path -from [get_clocks userclk1]   -to [get_clocks lms_txclk_iqsel_b]
set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_txclk_iqsel_a]
set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_txclk_iqsel_b]
set_false_path -from [get_clocks lms_txclk_iqsel_a] -to [get_clocks userclk1]
set_false_path -from [get_clocks lms_txclk_iqsel_b] -to [get_clocks userclk1]
set_false_path -from [get_clocks lms_txclk_iqsel_a] -to [get_clocks userclk1_u]
set_false_path -from [get_clocks lms_txclk_iqsel_b] -to [get_clocks userclk1_u]


#set_false_path -from [get_clocks userclk1]   -to [get_clocks lms_txclk_bb_ag]
#set_false_path -from [get_clocks userclk1]   -to [get_clocks lms_txclk_bb_bg]
#set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_txclk_bb_ag]
#set_false_path -from [get_clocks userclk1_u] -to [get_clocks lms_txclk_bb_bg]
#set_false_path -from [get_clocks lms_txclk_bb_ag] -to [get_clocks userclk1]
#set_false_path -from [get_clocks lms_txclk_bb_bg] -to [get_clocks userclk1]
#set_false_path -from [get_clocks lms_txclk_bb_ag] -to [get_clocks userclk1_u]
#set_false_path -from [get_clocks lms_txclk_bb_bg] -to [get_clocks userclk1_u]



set_false_path -from [get_clocks userclk1] -to [get_clocks cfg_mclk]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks userclk1]

set_false_path -from [get_clocks userclk1] -to [get_clocks usb_clk]
set_false_path -from [get_clocks usb_clk] -to [get_clocks userclk1]

set_false_path -from [get_clocks usb_clk] -to [get_clocks cfg_mclk]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks usb_clk]

set_false_path -from [get_clocks userclk1] -to [get_clocks clk_dsp_e]
set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks userclk1]

set_false_path -from [get_clocks userclk1] -to [get_clocks clk_dsp_u]
set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks userclk1]

######################

set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks lms_rxclk_bb_a]
set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks lms_rxclk_bb_b]
set_false_path -from [get_clocks lms_rxclk_bb_a] -to [get_clocks clk_dsp_e]
set_false_path -from [get_clocks lms_rxclk_bb_b] -to [get_clocks clk_dsp_e]


set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks lms_txclk_bb_a]
set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks lms_txclk_bb_b]
set_false_path -from [get_clocks lms_txclk_bb_a] -to [get_clocks clk_dsp_e]
set_false_path -from [get_clocks lms_txclk_bb_b] -to [get_clocks clk_dsp_e]


set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks lms_rxclk_bb_a]
set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks lms_rxclk_bb_b]
set_false_path -from [get_clocks lms_rxclk_bb_a] -to [get_clocks clk_dsp_u]
set_false_path -from [get_clocks lms_rxclk_bb_b] -to [get_clocks clk_dsp_u]


set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks lms_txclk_bb_a]
set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks lms_txclk_bb_b]
set_false_path -from [get_clocks lms_txclk_bb_a] -to [get_clocks clk_dsp_u]
set_false_path -from [get_clocks lms_txclk_bb_b] -to [get_clocks clk_dsp_u]


#######################



set_false_path -from [get_clocks userclk1_u] -to [get_clocks cfg_mclk]
set_false_path -from [get_clocks cfg_mclk] -to [get_clocks userclk1_u]

set_false_path -from [get_clocks userclk1_u] -to [get_clocks clk_dsp_e]
set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks userclk1_u]

set_false_path -from [get_clocks userclk1_u] -to [get_clocks clk_dsp_u]
set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks userclk1_u]

set_false_path -from [get_clocks userclk1_u] -to [get_clocks usb_clk]
set_false_path -from [get_clocks usb_clk] -to [get_clocks userclk1_u]


######################
#set rx_clk_direct [get_clocks -of [get_pins common/ddr_r/clk/mrcc_div2/O]]
#create_generated_clock -name rx_clk_nommcm \
#                       -source [get_pins common/ddr_r/clk/mrcc_div2/I] \
#                       -divide_by 2 \
#                       [get_pins common/ddr_r/clk/mrcc_div2/O]
#create_generated_clock -name rx_clk_nommcm \
#                       -source [get_pins common/ddr_r/clk/mrcc_div2/I] \
#                       -divide_by 2 \
#                       -add \
#                       -master_clock lms_rxclk \
#                       [get_pins common/ddr_r/clk/mrcc_div2/O]

#set_false_path -from [get_clocks clk_dsp_e] -to [get_clocks rx_clk_nommcm]
#set_false_path -from [get_clocks rx_clk_nommcm] -to [get_clocks clk_dsp_e]
#set_false_path -from [get_clocks clk_dsp_u] -to [get_clocks rx_clk_nommcm]
#set_false_path -from [get_clocks rx_clk_nommcm] -to [get_clocks clk_dsp_u]

#set_false_path -from [get_clocks userclk1]   -to [get_clocks rx_clk_nommcm]
#set_false_path -from [get_clocks userclk1_u] -to [get_clocks rx_clk_nommcm]
#set_false_path -from [get_clocks rx_clk_nommcm] -to [get_clocks userclk1]
#set_false_path -from [get_clocks rx_clk_nommcm] -to [get_clocks userclk1_u]




# connect_debug_port dbg_hub/clk [get_nets common/user_clk_pcie]
connect_debug_port dbg_hub/clk [get_nets common/cfg_mclk]