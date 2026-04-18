#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
BUILD_DIR="$ROOT_DIR/sym/usdr_tx_chain/build/vivado"

: "${XVLOG:=xvlog}"
: "${XELAB:=xelab}"
: "${XSIM:=xsim}"

if [[ -n "${XILINX_VIVADO:-}" ]]; then
  VIVADO_ROOT="$XILINX_VIVADO"
else
  XVLOG_BIN=$(command -v "$XVLOG")
  VIVADO_ROOT=$(cd "$(dirname "$XVLOG_BIN")/.." && pwd)
fi
GLBL_V="$VIVADO_ROOT/data/verilog/src/glbl.v"

mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

"$XVLOG" \
  "$ROOT_DIR/hw/usdr/usdr_tx_chain.v" \
  "$ROOT_DIR/lib/bus/axis_opt_pipeline.v" \
  "$ROOT_DIR/lib/cc/axis_cc_fifo.v" \
  "$ROOT_DIR/lib/cc/cc_counter.v" \
  "$ROOT_DIR/lib/cc/synchronizer.v" \
  "$ROOT_DIR/lib/dsp/reconf_dsp_elem.v" \
  "$ROOT_DIR/lib/dsp/reconf_dsp_fir.v" \
  "$ROOT_DIR/lib/mem/srl_ra.v" \
  "$ROOT_DIR/lib/mem/ram_sxp.v" \
  "$ROOT_DIR/lib/xilinx/dsp48e1_pipeline.v" \
  "$ROOT_DIR/lib/xilinx/dsp48e2_pipeline.v" \
  "$ROOT_DIR/sym/usdr_tx_chain/usdr_tx_chain_tb_deps.v" \
  "$ROOT_DIR/sym/usdr_tx_chain/tb_usdr_tx_chain.v" \
  "$GLBL_V"

"$XELAB" -L unisims_ver -L secureip tb_usdr_tx_chain glbl -s tb_usdr_tx_chain

XSIM_LOG="$BUILD_DIR/xsim.stdout.log"
"$XSIM" tb_usdr_tx_chain -runall | tee "$XSIM_LOG"

if grep -q "ERROR:" "$XSIM_LOG"; then
  echo "Vivado xsim reported an error; see $XSIM_LOG" >&2
  exit 1
fi

if ! grep -q "tb_usdr_tx_chain PASS" "$XSIM_LOG"; then
  echo "Vivado xsim did not report tb_usdr_tx_chain PASS; see $XSIM_LOG" >&2
  exit 1
fi
