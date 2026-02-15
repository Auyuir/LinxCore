# Janus Stage Migration Map

This map records ownership migration from legacy LinxCore blocks into explicit Janus stage/component files.

## Frontend / IFU

- Legacy `pyc/linxcore/frontend/ifetch.py` -> `pyc/linxcore/janus/bcc/ifu/f0.py`, `pyc/linxcore/janus/bcc/ifu/f2.py`
- Legacy `pyc/linxcore/frontend/ibuffer.py` -> `pyc/linxcore/janus/bcc/ifu/f3.py`
- Legacy predictor/control split -> `pyc/linxcore/janus/bcc/ifu/f1.py`, `pyc/linxcore/janus/bcc/ifu/ctrl.py`
- IFU output align -> `pyc/linxcore/janus/bcc/ifu/f4.py`

## OOO

- Decode entry -> `pyc/linxcore/janus/bcc/ooo/dec1.py`
- UOP refine -> `pyc/linxcore/janus/bcc/ooo/dec2.py`
- Rename entry -> `pyc/linxcore/janus/bcc/ooo/ren.py`
- Scheduler stage1 -> `pyc/linxcore/janus/bcc/ooo/s1.py`
- Scheduler stage2 -> `pyc/linxcore/janus/bcc/ooo/s2.py`
- ROB lifecycle -> `pyc/linxcore/janus/bcc/ooo/rob.py`
- PC side buffer -> `pyc/linxcore/janus/bcc/ooo/pc_buffer.py`
- Flush authority -> `pyc/linxcore/janus/bcc/ooo/flush_ctrl.py`
- Commit-side rename update -> `pyc/linxcore/janus/bcc/ooo/renu.py`

## IEX

- IEX lane orchestrator -> `pyc/linxcore/janus/bcc/iex/iex.py`
- ALU lane -> `pyc/linxcore/janus/bcc/iex/iex_alu.py`
- BRU lane -> `pyc/linxcore/janus/bcc/iex/iex_bru.py`
- FSU lane -> `pyc/linxcore/janus/bcc/iex/iex_fsu.py`
- AGU lane -> `pyc/linxcore/janus/bcc/iex/iex_agu.py`
- STD lane -> `pyc/linxcore/janus/bcc/iex/iex_std.py`

## LSU

- LIQ -> `pyc/linxcore/janus/bcc/lsu/liq.py`
- LHQ -> `pyc/linxcore/janus/bcc/lsu/lhq.py`
- STQ -> `pyc/linxcore/janus/bcc/lsu/stq.py`
- SCB -> `pyc/linxcore/janus/bcc/lsu/scb.py`
- MDB -> `pyc/linxcore/janus/bcc/lsu/mdb.py`
- L1D interface -> `pyc/linxcore/janus/bcc/lsu/l1d.py`

## Block Control + PEs + TMU

- BISQ -> `pyc/linxcore/janus/bcc/bctrl/bisq.py`
- BRENU -> `pyc/linxcore/janus/bcc/bctrl/brenu.py`
- BROB -> `pyc/linxcore/janus/bcc/bctrl/brob.py`
- BCTRL -> `pyc/linxcore/janus/bcc/bctrl/bctrl.py`
- TMU NOC -> `pyc/linxcore/janus/tmu/noc/node.py`, `pyc/linxcore/janus/tmu/noc/pipe.py`
- TMU tile register -> `pyc/linxcore/janus/tmu/sram/tilereg.py`
- TMA -> `pyc/linxcore/janus/tma/tma.py`
- CUBE -> `pyc/linxcore/janus/cube/cube.py`
- TAU -> `pyc/linxcore/janus/tau/tau.py`

## Integration

- Canonical stage-linked integration: `pyc/linxcore/janus/top.py`
- Canonical wrapper entrypoint retained: `pyc/linxcore/top/top.py`
