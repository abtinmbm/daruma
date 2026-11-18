"""Shared helpers: build the firmware's host-testable C components into a shared library for ctypes."""

from __future__ import annotations

import ctypes
import os
import shutil
import subprocess
import sys
from collections.abc import Callable
from pathlib import Path

import pytest

COMPONENTS = Path(__file__).resolve().parents[2] / "firmware" / "components"

# The same flags the firmware relies on for bit-exactness: float32 C99, no FMA contraction.
CFLAGS = ["-std=c99", "-O2", "-ffp-contract=off", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC"]

BuildLib = Callable[[str, list[str]], ctypes.CDLL]


@pytest.fixture(scope="session")
def c_compiler() -> str:
    """Path to the host C compiler. Skips locally without one; fails in CI."""
    cc = shutil.which("gcc") or shutil.which("cc")
    if cc is None:
        if os.environ.get("CI"):
            pytest.fail("no C compiler in CI")
        pytest.skip("no C compiler on PATH")
    return cc


@pytest.fixture(scope="session")
def build_c_lib(tmp_path_factory: pytest.TempPathFactory, c_compiler: str) -> BuildLib:
    """Returns build(name, ["component/file.c", ...]) -> CDLL."""
    cc = c_compiler
    out_dir = tmp_path_factory.mktemp("cbuild")
    suffix = ".dll" if sys.platform == "win32" else ".so"

    def build(name: str, sources: list[str]) -> ctypes.CDLL:
        out = out_dir / f"{name}{suffix}"
        includes = sorted({f"-I{COMPONENTS / Path(s).parts[0] / 'include'}" for s in sources})
        cmd = [cc, *CFLAGS, *includes, *(str(COMPONENTS / s) for s in sources), "-o", str(out), "-lm"]
        subprocess.run(cmd, check=True)
        return ctypes.CDLL(str(out))

    return build
