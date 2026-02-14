#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PYC_ROOT="/Users/zhoubot/pyCircuit"
GEN_CPP_DIR="${ROOT_DIR}/generated/cpp/linxcore_ooo_pyc"
HDR="${GEN_CPP_DIR}/linxcore_ooo_pyc.hpp"
TB_SRC="${ROOT_DIR}/tb/tb_linxcore_ooo_pyc.cpp"
TB_EXE="${GEN_CPP_DIR}/tb_linxcore_ooo_pyc_cpp"
TB_CXXFLAGS="${PYC_TB_CXXFLAGS:--O3 -DNDEBUG}"

need_regen=0
if [[ ! -f "${HDR}" ]]; then
  need_regen=1
elif find "${ROOT_DIR}/pyc/linxcore" "${ROOT_DIR}/pyc/linxcore_ooo_pyc.py" -name '*.py' -newer "${HDR}" | grep -q .; then
  need_regen=1
fi

if [[ "${need_regen}" -ne 0 ]]; then
  bash "${ROOT_DIR}/scripts/update_generated_linxcore_ooo.sh" >/dev/null
fi

need_build=0
if [[ ! -x "${TB_EXE}" ]]; then
  need_build=1
elif [[ "${TB_SRC}" -nt "${TB_EXE}" || "${HDR}" -nt "${TB_EXE}" ]]; then
  need_build=1
fi

if [[ "${need_build}" -ne 0 ]]; then
  tmp_exe="${TB_EXE}.tmp.$$"
  "${CXX:-clang++}" -std=c++17 ${TB_CXXFLAGS} \
    -I "${PYC_ROOT}/include" \
    -I "${GEN_CPP_DIR}" \
    -o "${tmp_exe}" \
    "${TB_SRC}"
  mv -f "${tmp_exe}" "${TB_EXE}"
fi

if [[ $# -gt 0 ]]; then
  "${TB_EXE}" "$@"
else
  "${TB_EXE}"
fi
