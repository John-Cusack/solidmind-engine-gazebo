"""This engine must never import core.

The whole point of the split is two repositories with two interpreters and one
contract between them.  A single ``import server.…`` re-couples the
environments and un-does it, so the check is static and runs in CI.
"""

from __future__ import annotations

import ast
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
_PACKAGES = ("gazebo_bridge",)
_FORBIDDEN = ("server",)


def _imported_roots(path: Path) -> set[tuple[str, int]]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: set[tuple[str, int]] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                found.add((alias.name.split(".")[0], node.lineno))
        elif isinstance(node, ast.ImportFrom) and node.module and not node.level:
            found.add((node.module.split(".")[0], node.lineno))
    return found


class TestNoCoreImports(unittest.TestCase):
    def test_no_server_imports(self) -> None:
        violations: list[str] = []
        for package in _PACKAGES:
            for path in sorted((_REPO_ROOT / package).rglob("*.py")):
                for root, lineno in _imported_roots(path):
                    if root in _FORBIDDEN:
                        violations.append(f"{path.relative_to(_REPO_ROOT)}:{lineno} imports {root!r}")
        self.assertEqual(
            violations,
            [],
            "This engine must not import core:\n  " + "\n  ".join(violations),
        )

    def test_the_scan_saw_something(self) -> None:
        """Guard the guard — but a C++ package legitimately has no Python."""
        scanned = [p for pkg in _PACKAGES for p in (_REPO_ROOT / pkg).rglob("*.py")]
        self.assertTrue(scanned, f"no python files scanned in {_PACKAGES}")


if __name__ == "__main__":
    unittest.main()
