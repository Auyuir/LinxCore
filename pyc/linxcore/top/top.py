from __future__ import annotations

from pycircuit import Circuit

from ..janus.top import build_janus_top


def build_top(m: Circuit, *, mem_bytes: int = (1 << 20)) -> None:
    build_janus_top(m, mem_bytes=mem_bytes)


build_top.__pycircuit_name__ = "linxcore_ooo_pyc"
