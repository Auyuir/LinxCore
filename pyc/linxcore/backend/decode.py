from __future__ import annotations

from dataclasses import dataclass

from pycircuit import Wire


@dataclass(frozen=True)
class DecodeSlotView:
    valid: Wire
    pc: Wire
    op: Wire
    insn_raw: Wire
    length: Wire
