# solidmind-engine-gazebo

Gazebo engine for SolidMind — package→SDF compilation, PX4 SITL, 5-DOF teleop.

Split out of [solidmind-cad](https://github.com/John-Cusack/solidmind-cad) with
its history intact. This repository is an **application**, not a library: it
speaks the [Engine Integration Contract](https://github.com/John-Cusack/solidmind-cad/blob/main/docs/engine-contract.md)
over a TCP socket and shares nothing with core but that contract.

## Running it

```bash
python3 -m gazebo_bridge.bridge_server --port 9879 --runtime stub
```

Core finds it through a descriptor — no registration, no core edit:

```toml
# ~/.solidmind/engines.d/gazebo.toml
name = "gazebo"
port = 9879
launch = ["python3", "-m", "gazebo_bridge.bridge_server", "--port", "${PORT}"]
cwd = "~/repos/solidmind-engine-gazebo"
when_to_use = "…what this engine is good at, in one line for the copilot…"
```

## Proving conformance

The TCK is vendored here so this repo's CI can run it without core:

```bash
python3 -m gazebo_bridge.bridge_server --port 9879 --runtime stub &
python3 -m tck --port 9879
```

Exit code 0 means conformant. Attach its output to any bug report — that is
the support boundary between this engine and core.

Re-vendor `tck/` and `schemas/` from core whenever the contract moves; the
handshake's `contract_versions_supported` is what tells you it has.

## Boundaries

* This repo must never import `server.*` — `tests/test_import_boundary.py`
  enforces it, because a single reverse import re-couples the two interpreters.
* Core emits only the canonical sim package, meshes and a courtesy URDF. Any
  vendor dialect this engine needs (SDF, MJCF, autopilot params) is compiled
  **here**, at load time.
* Nothing in this repo may assume another engine exists.

## Tests

```bash
python3 -m unittest
```

Tests that need the real engine installed skip themselves when it isn't.
