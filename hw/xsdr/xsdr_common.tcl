set origin_dir [file dirname [info script]]
create_project $project_name $origin_dir/$project_name
set proj_dir [get_property directory [current_project]]
set obj [get_projects $project_name]


set root_dir [file normalize "$origin_dir/../.."]
set lib_dir [file normalize "$root_dir/lib"]
set libx_dir [file normalize "$root_dir/lib"]


set_property "part" "$fpga_part" $obj
set_property "simulator_language" "Mixed" $obj
set_property "source_mgmt_mode" "DisplayOnly" $obj
set_property "target_language" "Verilog" $obj

if {[string equal [get_filesets -quiet sources_1] ""]} {
  create_fileset -srcset sources_1
}

set obj [get_filesets sources_1]
add_files -norecurse -fileset $obj $lib_dir/bus
add_files -norecurse -fileset $obj $lib_dir/cc
add_files -norecurse -fileset $obj $lib_dir/dma
add_files -norecurse -fileset $obj $lib_dir/dsp
add_files -norecurse -fileset $obj $lib_dir/fe
add_files -norecurse -fileset $obj $lib_dir/gpio
add_files -norecurse -fileset $obj $lib_dir/i2c
add_files -norecurse -fileset $obj $lib_dir/mem
add_files -norecurse -fileset $obj $lib_dir/misc
add_files -norecurse -fileset $obj $lib_dir/pci
add_files -norecurse -fileset $obj $lib_dir/spi
add_files -norecurse -fileset $obj $lib_dir/uart
add_files -norecurse -fileset $obj $lib_dir/xilinx
add_files -norecurse -fileset $obj $lib_dir/usb
add_files -norecurse -fileset $obj $lib_dir/lfsr
add_files -norecurse -fileset $obj $lib_dir/dsp_fft
add_files -norecurse -fileset $obj [list \
    [file normalize "$origin_dir/xsdr_common_top.v"] \
    [file normalize "$origin_dir/app_xsdr.v"] \
    [file normalize "$origin_dir/data_snapshoter.v"] \
    [file normalize "$origin_dir/dc_estimator.v"] \
    [file normalize "$origin_dir/ddr_realign.v"] \
    [file normalize "$origin_dir/ddr_bitslip_fsm.v"] \
    [file normalize "$origin_dir/ddr_reciver.v"] \
    [file normalize "$origin_dir/xlnx_ddr_clock.v"] \
    [file normalize "$origin_dir/xlnx_serdes_clk.v"] \
    [file normalize "$origin_dir/xlnx_lml_phy.v"] \
    [file normalize "$origin_dir/ssdrpro_common_top.v"] \
    [file normalize "$origin_dir/$sdr_top_module.v"] \
    [file normalize "$lib_dir/../hw/usdr_app_generic_us.v"] \
]


set_property include_dirs [list "$root_dir/hw/" "$lib_dir/../hw/"] $obj
set_property "top" "$sdr_top_module" $obj

if {[string equal [get_filesets -quiet constrs_1] ""]} {
  create_fileset -constrset constrs_1
}

import_ip $origin_dir/ip/blk_mem_gen_nrx/blk_mem_gen_nrx.xci -quiet
import_ip $origin_dir/ip/blk_mem_gen_ntx/blk_mem_gen_ntx.xci -quiet
import_ip $origin_dir/ip/blk_mem_gen_usb/blk_mem_gen_usb.xci -quiet
import_ip $origin_dir/ip/blk_mem_gen_nrx_128_64/blk_mem_gen_nrx_128_64.xci -quiet
import_ip $origin_dir/ip/blk_mem_gen_ntx_128_64_2/blk_mem_gen_ntx_128_64_2.xci -quiet
import_ip $origin_dir/ip/xfft_0/xfft_0.xci -quiet
import_ip $origin_dir/ip/vio_0/vio_0.xci -quiet
import_ip $origin_dir/ip/vio_4/vio_4.xci -quiet
import_ip $origin_dir/ip/vio_startup/vio_startup.xci -quiet
import_ip $origin_dir/ip/$pcie_core/${pcie_core}.xci -quiet

upgrade_ip [get_ips blk_mem_gen_nrx]
upgrade_ip [get_ips blk_mem_gen_ntx]
upgrade_ip [get_ips blk_mem_gen_usb]
upgrade_ip [get_ips blk_mem_gen_nrx_128_64]
upgrade_ip [get_ips blk_mem_gen_ntx_128_64_2]
upgrade_ip [get_ips $pcie_core]
upgrade_ip [get_ips xfft_0]
upgrade_ip [get_ips vio_4]
upgrade_ip [get_ips vio_0]
upgrade_ip [get_ips vio_startup]
set obj [get_filesets constrs_1]

set xdc_files [list \
     [file normalize "$origin_dir/xsdr_impl.xdc"] \
     [file normalize "$origin_dir/$xdc_top_file"] \
]
add_files -norecurse -fileset $obj $xdc_files
set_property "file_type" "XDC" [get_files -of_objects [get_filesets constrs_1] $xdc_files]


set_property STEPS.WRITE_BITSTREAM.ARGS.BIN_FILE true [get_runs impl_1]


report_ip_status
