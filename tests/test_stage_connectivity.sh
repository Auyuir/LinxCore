#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

python3 "${ROOT_DIR}/scripts/lint_stage_naming.py"

python3 - <<'PY'
import sys
from pathlib import Path

sys.path.insert(0, '/Users/zhoubot/LinxCore/pyc')
from linxcore.janus.common.interfaces import INTERFACE_SPEC

root = Path('/Users/zhoubot/LinxCore')
janus_text = []
for p in (root / 'pyc/linxcore/janus').rglob('*.py'):
    janus_text.append(p.read_text(encoding='utf-8'))
joined = '\n'.join(janus_text)

missing = []
for prefix in INTERFACE_SPEC:
    if prefix not in joined:
        missing.append(prefix)

if missing:
    raise SystemExit('missing interface prefix references in Janus source tree: ' + ', '.join(missing))

print('stage connectivity prefix check passed')
PY
