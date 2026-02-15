# Reference Submodules (Read-Only)

This repository keeps architecture/tooling references under `third_party/` as read-only git submodules.

## Policy

- Submodules are reference-only; LinxCore build scripts do not import source from them at runtime.
- Pins are updated manually with explicit review.
- Default operation is detached at pinned commit.

## Locations

- `third_party/xiangshan` -> `git@github.com:OpenXiangShan/XiangShan.git`
- `third_party/pycircuit_ref` -> `git@github.com:LinxISA/pyCircuit.git`
- `third_party/qemu_ref` -> `git@github.com:LinxISA/qemu.git`

## Update Flow

1. `git submodule update --init --recursive`
2. Enter target submodule directory.
3. Checkout desired commit.
4. Return to repository root and commit updated gitlink.

## Read-Only Rule

Do not commit local modifications inside submodule worktrees as part of LinxCore changes.
