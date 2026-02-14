from __future__ import annotations

from dataclasses import dataclass

from pycircuit import Wire


@dataclass(frozen=True)
class LsuStoreView:
    valid: Wire
    addr: Wire
    data: Wire
    size: Wire
