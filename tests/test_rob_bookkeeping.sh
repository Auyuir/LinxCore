#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYC_ROOT="/Users/zhoubot/pyCircuit"
GEN_CPP_DIR="${ROOT_DIR}/generated/cpp/linxcore_ooo_pyc"
GEN_HDR="${GEN_CPP_DIR}/linxcore_ooo_pyc.hpp"
SRC="${ROOT_DIR}/tests/test_rob_bookkeeping.cpp"
EXE="${GEN_CPP_DIR}/test_rob_bookkeeping"
MEMH="${ROOT_DIR}/tests/benchmarks/build/dhrystone_compat.memh"

if [[ ! -f "${GEN_HDR}" ]]; then
  bash "${ROOT_DIR}/scripts/update_generated_linxcore_ooo.sh" >/dev/null
fi
if [[ ! -f "${MEMH}" ]]; then
  bash "${ROOT_DIR}/scripts/build_linxisa_benchmarks_memh_compat.sh" >/dev/null
fi

need_build=0
if [[ ! -x "${EXE}" ]]; then
  need_build=1
elif [[ "${SRC}" -nt "${EXE}" || "${GEN_HDR}" -nt "${EXE}" ]]; then
  need_build=1
fi

if [[ "${need_build}" -ne 0 ]]; then
  "${CXX:-clang++}" -std=c++17 -O2 -Wall -Wextra \
    -I "${PYC_ROOT}/include" \
    -I "${GEN_CPP_DIR}" \
    -o "${EXE}" \
    "${SRC}"
fi

PYC_BOOT_PC=0x10000 \
PYC_BOOT_SP=0x00000000000ff000 \
PYC_MAX_CYCLES=12000 \
  "${EXE}" "${MEMH}"

echo "rob bookkeeping test: ok"
