# LinxCore OOO PYC Design (Memory-First)

## Scope

This document describes the standalone LinxCore/Janus stage-mapped OoO refactor implemented in:

- `/Users/zhoubot/LinxCore/pyc/linxcore_ooo_pyc.py`
- `/Users/zhoubot/LinxCore/pyc/linxcore/*`
- `/Users/zhoubot/LinxCore/pyc/linxcore/janus/*`

Primary goals:

- Split source by hardware component and pipeline stage.
- Keep the M1 retire-trace schema stable for QEMU lockstep tools.
- Improve memory-side throughput with I/D separation and store-retire decoupling.

## Module split

Top-level modules instantiated with real `m.instance` boundaries:

- `JanusTop` integration (`/Users/zhoubot/LinxCore/pyc/linxcore/janus/top.py`)
- `LinxCoreMem2R1W` (`/Users/zhoubot/LinxCore/pyc/linxcore/mem/mem2r1w.py`)
- `LinxCoreBackend` compatibility path (`/Users/zhoubot/LinxCore/pyc/linxcore/backend/backend.py`)

Shared/common logic:

- `/Users/zhoubot/LinxCore/pyc/linxcore/common/{isa.py,decode_f4.py,exec_uop.py,types.py,params.py,util.py}`

Janus BCC stage contracts are split into dedicated files:

- IFU: `/Users/zhoubot/LinxCore/pyc/linxcore/janus/bcc/ifu/{f0.py,f1.py,f2.py,f3.py,f4.py,icache.py,ctrl.py}`
- OOO: `/Users/zhoubot/LinxCore/pyc/linxcore/janus/bcc/ooo/{dec1.py,dec2.py,ren.py,s1.py,s2.py,rob.py,pc_buffer.py,flush_ctrl.py,renu.py}`
- IEX: `/Users/zhoubot/LinxCore/pyc/linxcore/janus/bcc/iex/{iex.py,iex_alu.py,iex_bru.py,iex_fsu.py,iex_agu.py,iex_std.py}`
- BCtrl: `/Users/zhoubot/LinxCore/pyc/linxcore/janus/bcc/bctrl/{bctrl.py,bisq.py,brenu.py,brob.py}`
- LSU: `/Users/zhoubot/LinxCore/pyc/linxcore/janus/bcc/lsu/{liq.py,lhq.py,stq.py,scb.py,mdb.py,l1d.py}`
- TMU/TMA/CUBE/TAU: `/Users/zhoubot/LinxCore/pyc/linxcore/janus/tmu/*`, `/Users/zhoubot/LinxCore/pyc/linxcore/janus/tma/tma.py`, `/Users/zhoubot/LinxCore/pyc/linxcore/janus/cube/cube.py`, `/Users/zhoubot/LinxCore/pyc/linxcore/janus/tau/tau.py`

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
  - `/Users/zhoubot/LinxCore/scripts/build_linxisa_benchmarks_memh_compat.sh` now defaults `LINX_INCLUDE_LIBM=0` to avoid unresolved soft-float runtime symbols on minimal Linx toolchains. Set `LINX_INCLUDE_LIBM=1` only when builtins are available.

## Validation gates

- `/Users/zhoubot/LinxCore/tests/test_runner_protocol.sh`
- `/Users/zhoubot/LinxCore/tests/test_trace_schema_and_mem.sh`
- `/Users/zhoubot/LinxCore/tests/test_cosim_smoke.sh`
- `/Users/zhoubot/LinxCore/tests/test_stage_connectivity.sh`

## Notes

Co-sim smoke is validated with `trigger_pc == boot_pc` and bounded terminate windows.
CoreMark/Dhrystone completion is supported by script; cycle tuning remains iterative.
