# Tests that still reach into core

These came across in the split but import `server.*`, so they cannot run in
this repository as-is. That import is exactly what the split removes: an
engine repo has its own interpreter and shares nothing with core but the
contract.

Porting each one means swapping the core dependency for a local equivalent:

| Core import | Use instead |
|---|---|
| `server.engine_client` | the vendored `tck.client.TckClient`, or this engine's own client |
| `server.motion_models` | plain mechanism dicts — the wire format is dicts anyway |
| `server.sim_export` / `server.sim_package_manifest` | a fixture package under `tests/fixtures/` |

They are excluded from discovery (this directory has no `__init__.py`), so CI
stays green while the work happens. Core keeps its copies until then.
