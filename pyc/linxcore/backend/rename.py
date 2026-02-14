from __future__ import annotations

from dataclasses import dataclass

from pycircuit import Wire


@dataclass(frozen=True)
class RenameSlotView:
    valid: Wire
    rob: Wire
    pdst: Wire
