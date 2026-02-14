from __future__ import annotations

from dataclasses import dataclass

from pycircuit import Wire


@dataclass(frozen=True)
class CommitCtrl:
    redirect_valid: Wire
    redirect_pc: Wire
