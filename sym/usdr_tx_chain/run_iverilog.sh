#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
BUILD_DIR="$ROOT_DIR/sym/usdr_tx_chain/build/iverilog"

mkdir -p "$BUILD_DIR"

iverilog -g2012 -DSYM \
  -o "$BUILD_DIR/tb_usdr_tx_chain.out" \
  "$ROOT_DIR/hw/usdr/usdr_tx_chain.v" \
  "$ROOT_DIR/lib/bus/axis_opt_pipeline.v" \
  "$ROOT_DIR/lib/cc/axis_cc_fifo.v" \
  "$ROOT_DIR/lib/cc/cc_counter.v" \
  "$ROOT_DIR/lib/cc/synchronizer.v" \
  "$ROOT_DIR/lib/dsp/reconf_dsp_elem.v" \
  "$ROOT_DIR/lib/dsp/reconf_dsp_fir.v" \
  "$ROOT_DIR/lib/mem/axis_fifo_trd.v" \
  "$ROOT_DIR/lib/mem/srl_ra.v" \
  "$ROOT_DIR/lib/mem/ram_sxp.v" \
  "$ROOT_DIR/lib/xilinx/dsp48e1_pipeline.v" \
  "$ROOT_DIR/lib/xilinx/dsp48e2_pipeline.v" \
  "$ROOT_DIR/sym/lib/ram_sxp_sym.v" \
  "$ROOT_DIR/sym/xilinx/BUFG.v" \
  "$ROOT_DIR/sym/xilinx/BUFR.v" \
  "$ROOT_DIR/sym/xilinx/BUFIO.v" \
  "$ROOT_DIR/sym/xilinx/DSP48E1.v" \
  "$ROOT_DIR/sym/xilinx/DSP48E2.v" \
  "$ROOT_DIR/sym/xilinx/ODDR.v" \
  "$ROOT_DIR/sym/xilinx/OSERDESE2.v" \
  "$ROOT_DIR/sym/xilinx/SRL16E.v" \
  "$ROOT_DIR/sym/xilinx/SRLC32E.v" \
  "$ROOT_DIR/sym/usdr_tx_chain/usdr_tx_chain_tb_deps.v" \
  "$ROOT_DIR/sym/usdr_tx_chain/tb_usdr_tx_chain.v"

vvp "$BUILD_DIR/tb_usdr_tx_chain.out"
