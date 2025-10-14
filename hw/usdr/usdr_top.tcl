set origin_dir [file dirname [info script]]
set root_dir [file normalize "$origin_dir/../.."]
set lib_dir [file normalize "$root_dir/lib"]

create_project usdr $origin_dir/usdr

set proj_dir [get_property directory [current_project]]
set obj [get_projects usdr]
set_property "part" "xc7a35tcpg236-2" $obj
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
add_files -norecurse -fileset $obj $lib_dir/dsp_fft
add_files -norecurse -fileset $obj $lib_dir/fe
add_files -norecurse -fileset $obj $lib_dir/gpio
add_files -norecurse -fileset $obj $lib_dir/i2c
add_files -norecurse -fileset $obj $lib_dir/mem
add_files -norecurse -fileset $obj $lib_dir/misc
add_files -norecurse -fileset $obj $lib_dir/pci
add_files -norecurse -fileset $obj $lib_dir/spi
add_files -norecurse -fileset $obj $lib_dir/uart
add_files -norecurse -fileset $obj $lib_dir/usb
add_files -norecurse -fileset $obj $lib_dir/xilinx
add_files -norecurse -fileset $obj [list \
    [file normalize "$origin_dir/usdr_top_all.v"] \
    [file normalize "$origin_dir/app_usdr_pcie.v"] \
    [file normalize "$root_dir/hw/usdr_app_generic_us.v"] \
]

set_property include_dirs $root_dir/hw/ $obj
set_property "top" "usdr_top_all" $obj


if {[string equal [get_filesets -quiet constrs_1] ""]} {
  create_fileset -constrset constrs_1
}

import_ip $origin_dir/ip/blk_mem_gen_nrx/blk_mem_gen_nrx.xci -quiet
import_ip $origin_dir/ip/blk_mem_gen_ntx/blk_mem_gen_ntx.xci -quiet
import_ip $origin_dir/ip/blk_mem_gen_usb/blk_mem_gen_usb.xci -quiet
import_ip $origin_dir/ip/pcie_7x_0/pcie_7x_0.xci -quiet
import_ip $origin_dir/ip/xfft_0/xfft_0.xci -quiet


upgrade_ip [get_ips blk_mem_gen_nrx]
upgrade_ip [get_ips pcie_7x_0]

set obj [get_filesets constrs_1]

set xdc_file [file normalize "$origin_dir/usdr_ucf_r3.xdc"]
add_files -norecurse -fileset $obj $xdc_file
set_property "file_type" "XDC" [get_files -of_objects [get_filesets constrs_1] [list "*$xdc_file"]]


report_ip_status
