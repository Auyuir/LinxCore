#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYC_ROOT="/Users/zhoubot/pyCircuit"

source "${PYC_ROOT}/scripts/lib.sh"
pyc_find_pyc_compile

OUT_CPP="${ROOT_DIR}/generated/cpp/linxcore_ooo_pyc"
OUT_V="${ROOT_DIR}/generated/verilog/linxcore_ooo_pyc"
mkdir -p "${OUT_CPP}" "${OUT_V}"

TMP_PYC="$(mktemp -t linxcore_ooo_pyc.XXXXXX.pyc)"
trap 'rm -f "${TMP_PYC}"' EXIT

PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="${PYC_ROOT}/python:${ROOT_DIR}/pyc" \
python3 -m pycircuit.cli emit "${ROOT_DIR}/pyc/linxcore_ooo_pyc.py" -o "${TMP_PYC}"

TRY_V_OUTDIR="${LINXCORE_TRY_V_OUTDIR:-0}"
if [[ "${TRY_V_OUTDIR}" == "1" ]]; then
  if ! "${PYC_COMPILE}" "${TMP_PYC}" --emit=verilog --out-dir="${OUT_V}" >/dev/null 2>&1; then
    MONO_V="${OUT_V}/linxcore_ooo_pyc.v"
    "${PYC_COMPILE}" "${TMP_PYC}" --emit=verilog -o "${MONO_V}"
    python3 "${ROOT_DIR}/scripts/split_verilog_modules.py" \
      --src "${MONO_V}" \
      --out-dir "${OUT_V}" \
      --top "linxcore_ooo_pyc"
  fi
else
  MONO_V="${OUT_V}/linxcore_ooo_pyc.v"
  "${PYC_COMPILE}" "${TMP_PYC}" --emit=verilog -o "${MONO_V}"
  python3 "${ROOT_DIR}/scripts/split_verilog_modules.py" \
    --src "${MONO_V}" \
    --out-dir "${OUT_V}" \
    --top "linxcore_ooo_pyc"
fi

"${PYC_COMPILE}" "${TMP_PYC}" --emit=cpp --out-dir="${OUT_CPP}"
echo "${OUT_CPP}"
echo "${OUT_V}"
