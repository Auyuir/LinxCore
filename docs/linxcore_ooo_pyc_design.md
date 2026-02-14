# LinxCore OOO PYC Design (Memory-First)

## Scope

This document describes the standalone LinxCore OoO refactor implemented in:

- `/Users/zhoubot/LinxCore/pyc/linxcore_ooo_pyc.py`
- `/Users/zhoubot/LinxCore/pyc/linxcore/*`

Primary goals:

- Split source by hardware component and pipeline stage.
- Keep the M1 retire-trace schema stable for QEMU lockstep tools.
- Improve memory-side throughput with I/D separation and store-retire decoupling.

## Module split

Top-level modules instantiated with real `m.instance` boundaries:

- `LinxCoreMem2R1W` (`/Users/zhoubot/LinxCore/pyc/linxcore/mem/mem2r1w.py`)
- `LinxCoreFrontend` (`/Users/zhoubot/LinxCore/pyc/linxcore/frontend/frontend.py`)
- `LinxCoreBackend` (`/Users/zhoubot/LinxCore/pyc/linxcore/backend/backend.py`)

Shared/common logic:

- `/Users/zhoubot/LinxCore/pyc/linxcore/common/{isa.py,decode_f4.py,exec_uop.py,types.py,params.py,util.py}`

Backend stage contracts are split into dedicated files:

- `/Users/zhoubot/LinxCore/pyc/linxcore/backend/{decode.py,rename.py,dispatch.py,issue.py,wakeup.py,lsu.py,rob.py,commit.py,code_template_unit.py}`

## Memory-first behavior

- I/D path separation via dual memories in `LinxCoreMem2R1W`:
  - `imem` read port for fetch.
  - `dmem` read/write path for LSU.
  - host writes mirrored to both memories.
- Store-retire decoupling:
  - retired stores enqueue into a committed store buffer.
  - independent drain path writes one store/cycle to `dmem`.
  - MMIO stores at commit still fire immediately:
    - `0x10000000` UART
    - `0x10000004` EXIT
- LSU ordering/forwarding:
  - stall load if an older unresolved store exists.
  - forward load data from older completed store or committed store buffer on address match.

## Retire trace contract

Per-commit-slot taps exported at top level (slot `0..3`):

- `commit_fireX`, `commit_pcX`, `commit_robX`, `commit_opX`
- `commit_lenX`, `commit_insn_rawX`
- `commit_wb_validX`, `commit_wb_rdX`, `commit_wb_dataX`
- `commit_mem_validX`, `commit_mem_is_storeX`, `commit_mem_addrX`, `commit_mem_wdataX`, `commit_mem_rdataX`, `commit_mem_sizeX`
- `commit_trap_validX`, `commit_trap_causeX`, `commit_next_pcX`

These are consumed by:

- `/Users/zhoubot/LinxCore/tb/tb_linxcore_ooo_pyc.cpp`
- `/Users/zhoubot/LinxCore/cosim/linxcore_lockstep_runner.cpp`

## Scripts

- Generate split artifacts:
  - `/Users/zhoubot/LinxCore/scripts/update_generated_linxcore_ooo.sh`
- Build/run C++ TB:
  - `/Users/zhoubot/LinxCore/scripts/run_linxcore_ooo_cpp.sh`
- Run lockstep co-sim:
  - `/Users/zhoubot/LinxCore/scripts/run_cosim_lockstep.sh`
- Benchmark harness:
  - `/Users/zhoubot/LinxCore/scripts/run_linxcore_benchmarks.sh`

## Validation gates

- `/Users/zhoubot/LinxCore/tests/test_runner_protocol.sh`
- `/Users/zhoubot/LinxCore/tests/test_trace_schema_and_mem.sh`
- `/Users/zhoubot/LinxCore/tests/test_cosim_smoke.sh`

## Notes

Co-sim smoke is validated with `trigger_pc == boot_pc` and bounded terminate windows.
CoreMark/Dhrystone completion is supported by script; cycle tuning remains iterative.
