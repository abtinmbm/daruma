"""Compiles firmware/main on the host against stand-in ESP-IDF headers (tests/esp_stubs).

The golden-vector tests only build the pure-math components, so a plain C error in main.c (such as a
duplicate variable) would otherwise only surface in the slower ESP-IDF build. This is a syntax and type
check with warnings as errors, not a substitute for `idf.py build`: the stubs declare only what main.c uses.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

from conftest import COMPONENTS

MAIN = COMPONENTS.parent / "main"
STUBS = Path(__file__).resolve().parent / "esp_stubs"


def required_components() -> list[str]:
    """Components listed after REQUIRES in firmware/main/CMakeLists.txt, so a header from an undeclared
    component fails here as it would in the ESP-IDF build."""
    m = re.search(r"REQUIRES([^)]*)\)", (MAIN / "CMakeLists.txt").read_text(encoding="utf-8"))
    assert m, "no REQUIRES in firmware/main/CMakeLists.txt"
    return m.group(1).split()


def test_firmware_main_compiles_on_the_host(c_compiler: str) -> None:
    deps = required_components()
    assert {"control", "fusion"} <= set(deps)
    includes = [f"-I{STUBS}", *(f"-I{COMPONENTS / d / 'include'}" for d in deps if (COMPONENTS / d / "include").is_dir())]
    sources = sorted(MAIN.glob("*.c"))
    assert sources, "no C files in firmware/main"
    cmd = [c_compiler, "-std=gnu17", "-Wall", "-Wextra", "-Werror", "-fsyntax-only", *includes, *map(str, sources)]
    done = subprocess.run(cmd, capture_output=True, text=True)
    assert done.returncode == 0, done.stderr
