from __future__ import annotations

from dataclasses import dataclass

from pycircuit import Wire


@dataclass(frozen=True)
class RobCommitView:
    fire: Wire
    pc: Wire
    rob: Wire
    op: Wire
