#!/usr/bin/env python3
"""Small independent DLL build; invoke in the user's normal host terminal."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from importlib.util import module_from_spec, spec_from_file_location

ROOT = Path(__file__).resolve().parent.parent
sys.dont_write_bytecode = True
spec = spec_from_file_location("pose_cache", ROOT / "tools/check-cache.py")
cache_module = module_from_spec(spec)
spec.loader.exec_module(cache_module)


def run(command, environment, log):
    print("Running:", subprocess.list2cmdline([str(part) for part in command]), flush=True)
    # One isolated process group makes cancellation scoped to this build.
    process = subprocess.Popen(command, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, start_new_session=os.name != "nt")
    try:
        for line in process.stdout:
            print(line, end="", flush=True)
            log.write(line)
            log.flush()
        code = process.wait()
        if code:
            raise subprocess.CalledProcessError(code, command)
    except BaseException:
        if os.name == "nt":
            process.kill()
            process.wait()
            raise
        for sig, seconds in ((signal.SIGTERM, 5), (signal.SIGKILL, 2)):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=seconds)
            except subprocess.TimeoutExpired:
                pass
        raise


def main():
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Validate dependencies only; do not run Wine or compile")
    args = parser.parse_args()
    cache_path = os.environ.get("GMODPOSE_CACHE_ROOT")
    cache = cache_module.check(cache_path) if cache_path else None
    if args.check:
        print("Validated the dependency cache." if cache else "The build will use configured packages and pinned sources.")
        return 0
    environment = dict(os.environ)
    toolchain = []
    if os.name != "nt":
        wrappers = os.environ.get("GMODPOSE_MSVC_BIN")
        if wrappers:
            environment["PATH"] = wrappers + os.pathsep + environment["PATH"]
        environment.setdefault("WINEDEBUG", "-all")
        toolchain = [f"-DCMAKE_TOOLCHAIN_FILE={ROOT}/cmake/msvc-wine.cmake"]
    build = ROOT / "build/standalone-build"
    build.mkdir(parents=True, exist_ok=True)
    result = build / "result.json"
    result.unlink(missing_ok=True)
    package_spec = spec_from_file_location("pose_package", ROOT / "tools/package.py")
    package_module = module_from_spec(package_spec)
    package_spec.loader.exec_module(package_module)
    sources = package_module.source_hashes()
    start = time.monotonic()
    success = False
    try:
        with (build / "host-build.log").open("w") as log:
            run(["cmake", "-S", str(ROOT), "-B", str(build), "-G", "Ninja",
                 "-DCMAKE_BUILD_TYPE=Release", f"-DGMODPOSE_CACHE_ROOT={cache or ''}", *toolchain], environment, log)
            run(["cmake", "--build", str(build), "--target", "GModPose", "--parallel", "2"], environment, log)
        success = (build / "out/GModPose.dll").is_file()
        if not success:
            raise ValueError("Build returned without producing GModPose.dll")
        if package_module.source_hashes() != sources:
            success = False
            raise ValueError("Source changed while building; rerun before packaging")
    finally:
        result.write_text(json.dumps({"success": success, "utc": datetime.now(timezone.utc).isoformat(),
                                      "elapsed_seconds": round(time.monotonic() - start, 2),
                                      "dll": str(build / "out/GModPose.dll"), "installed": False,
                                      "source_sha256": sources,
                                      "dll_sha256": hashlib.sha256((build / "out/GModPose.dll").read_bytes()).hexdigest()
                                          if success else None}, indent=2) + "\n")
    archives = package_module.package()
    print(f"Built: {build / 'out/GModPose.dll'}")
    for archive in archives:
        print(f"Packaged and verified: {archive}")
    print("Nothing installed.")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit("Build cancelled; its process group was terminated.")
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        sys.exit(f"Build failed: {error}")
