#!/usr/bin/env python3
"""Read-only checks for the specific existing SE/AE /MD dependency build."""
from pathlib import Path
import os
import re
import subprocess
import sys

REVISION = "8ffc2b74af015f837de53cc02b0cb470a15678e4"


def check(root):
    root = Path(root).resolve()
    source = root / "linux/_deps/commonlibsse-src"
    library = root / "linux/_deps/commonlibsse-build/CommonLibSSE.lib"
    git_environment = dict(os.environ, GIT_OPTIONAL_LOCKS="0")
    revision = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"],
                                       text=True, env=git_environment).strip()
    if revision != REVISION:
        raise ValueError(f"CommonLib source revision differs: {revision}")
    dirty = subprocess.check_output(
        ["git", "-C", str(source), "status", "--porcelain", "--untracked-files=no"],
        text=True, env=git_environment)
    if dirty.strip():
        raise ValueError("CommonLib tracked source is modified")
    cache = (root / "linux/CMakeCache.txt").read_text()
    expected = {"ENABLE_SKYRIM_SE": "ON", "ENABLE_SKYRIM_AE": "ON", "ENABLE_SKYRIM_VR": "OFF",
                "SKSE_SUPPORT_PATCH_SAFETY": "OFF", "SKSE_SUPPORT_XBYAK": "OFF",
                "REX_OPTION_INI": "OFF", "REX_OPTION_JSON": "OFF", "REX_OPTION_TOML": "OFF",
                "CMAKE_BUILD_TYPE": "Release"}
    for key, value in expected.items():
        if not re.search(rf"^{key}:[^=]+={re.escape(value)}$", cache, re.M):
            raise ValueError(f"Cached build must have {key}={value}")
    ninja = (root / "linux/build.ninja").read_text()
    marker = "build _deps/commonlibsse-build/CMakeFiles/CommonLibSSE.dir/src/RE/B/BSFaceGenAnimationData.cpp.obj:"
    block = ninja.split(marker, 1)[1].split("\n\n", 1)[0]
    if " -MD " not in block or "-std:c++latest" not in block or "ENABLE_SKYRIM_VR" in block:
        raise ValueError("CommonLib compiler flags do not match SE/AE C++latest /MD")
    if not library.is_file() or library.stat().st_size < 1_000_000:
        raise ValueError(f"Missing compiled archive: {library}")
    for relative in ("prefix/lib/cmake/spdlog/spdlogConfig.cmake", "prefix/share/directxtk/directxtk-config.cmake",
                     "prefix/share/directxmath/directxmath-config.cmake"):
        if not (root / relative).is_file():
            raise ValueError(f"Missing dependency config: {relative}")
    imgui = root / "linux/_deps/imgui-src"
    if not re.search(r'^#define IMGUI_VERSION\s+"1\.92\.9"$', (imgui / "imgui.h").read_text(), re.M):
        raise ValueError("Cached ImGui source must be version 1.92.9")
    for name in ("imgui.cpp", "imgui_draw.cpp", "imgui_tables.cpp", "imgui_widgets.cpp", "backends/imgui_impl_dx11.cpp"):
        if not (imgui / name).is_file():
            raise ValueError(f"Missing ImGui source: {name}")
    print(f"Cached CommonLib validated: {revision}; SE/AE; Release /MD; {library.stat().st_size:,} bytes")
    return root


if __name__ == "__main__":
    try:
        if len(sys.argv) != 2:
            raise ValueError("Usage: check-cache.py CACHE_DIRECTORY")
        check(sys.argv[1])
    except (ValueError, OSError, subprocess.CalledProcessError, IndexError) as error:
        sys.exit(f"Dependency check failed: {error}")
