#!/usr/bin/env python3
"""Package a successful host build whose source identities match this workspace."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import struct
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parent.parent


def source_hashes():
    paths = [ROOT / "CMakeLists.txt", Path(__file__).resolve()]
    paths.extend(p for p in (ROOT / "cmake").rglob("*") if p.is_file())
    paths.extend(ROOT / "tools" / name for name in ("build-host.py", "check-cache.py"))
    paths.extend(p for p in (ROOT / "tools/pandora").glob("*") if p.suffix in {".cs", ".csproj", ".xml"})
    paths.extend(p for p in (ROOT / "src").rglob("*") if p.suffix in {".cpp", ".h"})
    paths.extend(p for p in (ROOT / "assets").rglob("*") if p.is_file())
    return {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(paths)}


def package():
    version = re.search(r"project\(GModPose VERSION ([\d.]+)", (ROOT / "CMakeLists.txt").read_text()).group(1)
    build = ROOT / "build/standalone-build"
    result = json.loads((build / "result.json").read_text())
    if not result.get("success") or result.get("source_sha256") != source_hashes():
        raise ValueError("Rebuild with build.sh before packaging: source does not match a successful recorded build")
    raw = (build / "out/GModPose.dll").read_bytes()
    dll_hash = hashlib.sha256(raw).hexdigest()
    if dll_hash != result.get("dll_sha256"):
        raise ValueError("DLL differs from successful host build")
    if raw[:2] != b"MZ":
        raise ValueError("DLL lacks a PE header")
    pe = struct.unpack_from("<I", raw, 0x3C)[0]
    if (raw[pe:pe+4] != b"PE\0\0" or struct.unpack_from("<H", raw, pe+4)[0] != 0x8664 or
        struct.unpack_from("<H", raw, pe+24)[0] != 0x20B or not struct.unpack_from("<H", raw, pe+22)[0] & 0x2000):
        raise ValueError("Expected a Windows x64 DLL")
    for export in (b"SKSEPlugin_Load", b"SKSEPlugin_Query", b"SKSEPlugin_Version"):
        if export + b"\0" not in raw:
            raise ValueError(f"Missing SKSE export name: {export!r}")
    if f"GModPose IK editor {version};".encode() not in raw:
        raise ValueError("DLL version differs from project version")
    files = {"SKSE/Plugins/GModPose.dll": raw}
    for name in ("LICENSE", "THIRD_PARTY_NOTICES.md"):
        files["licenses/GModPose/" + name] = (ROOT / name).read_bytes()
    for license_file in (ROOT / "licenses").glob("*.txt"):
        files["licenses/GModPose/licenses/" + license_file.name] = license_file.read_bytes()
    data = ROOT / "assets"
    font = "SKSE/Plugins/GModPose/fonts/Sovngarde Light.ttf"
    files[font] = (data / font).read_bytes()
    for name in ("ArmPrior.bin", "LegPrior.bin"):
        path = f"SKSE/Plugins/GModPose/{name}"
        files[path] = (data / path).read_bytes()
    for kind in ("Face", "Hand"):
        folder = data / "SKSE/Plugins/GModPose/Presets/Premade" / kind
        presets = sorted(folder.glob("*.json"))
        if not presets:
            raise ValueError(f"No bundled {kind.lower()} presets")
        for preset in presets:
            files[preset.relative_to(data).as_posix()] = preset.read_bytes()
    sharing_assets = (
        "meshes/actors/character/animations/GModPoseShare/ShareStill.hkx",
        "meshes/actors/character/animations/GModPoseShare/FNIS_GModPoseShare_List.txt",
        "meshes/actors/character/behaviors/FNIS_GModPoseShare_Behavior.hkx",
        "Pandora_Engine/mod/gmodposeshare/info.xml",
        "Pandora_Engine/mod/gmodposeshare/patches/0_master.xml",
        "Pandora_Engine/mod/gmodposeshare/native/GModPose.Pandora/GModPose.Pandora.dll",
    )
    regular = {**files, **{asset: (data / asset).read_bytes() for asset in sharing_assets}}
    standalone = files.copy()
    ini_path = "SKSE/Plugins/GModPose.ini"
    regular[ini_path] = (data / ini_path).read_bytes()
    standalone[ini_path] = regular[ini_path]
    for setting in (b"Enabled", b"DetachSeatedPose", b"PeriodSeconds"):
        standalone[ini_path], count = re.subn(
            rb"(?m)^" + setting + rb"=[0-9]+$", setting + b"=0", standalone[ini_path])
        if count != 1:
            raise ValueError(f"Expected exactly one {setting.decode()} in GModPose.ini")

    fomod = {f"Common/{name}": payload for name, payload in files.items()}
    for folder, profile in (("Standalone", standalone), ("PlayerHub", regular)):
        fomod.update({f"{folder}/{name}": payload for name, payload in profile.items() if name not in files})
    for name in ("ModuleConfig.xml", "playerhub.png", "standalone.png"):
        fomod[f"fomod/{name}"] = (data / "fomod" / name).read_bytes()
    fomod["fomod/info.xml"] = (
        '<?xml version="1.0" encoding="utf-8"?>\n'
        '<fomod><Name>GModPose</Name><Author>Macinbomzh</Author>'
        f'<Version>{version}</Version></fomod>\n'
    ).encode()

    dist = ROOT / "dist"
    dist.mkdir(exist_ok=True)
    dest = dist / "GModPose.zip"
    manifest_path = dest.with_suffix(".manifest.json")
    with tempfile.TemporaryDirectory(prefix=".posing-packages-", dir=dist) as temporary:
        stage = Path(temporary)
        output = stage / dest.name
        with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
            for name, payload in sorted(fomod.items()):
                archive.writestr(name, payload)
        with zipfile.ZipFile(output) as archive:
            names = archive.namelist()
            if archive.testzip() is not None or len(names) != len(fomod) or set(names) != set(fomod):
                raise ValueError(f"Package member/CRC verification failed for {dest.name}")
            for name, payload in fomod.items():
                if archive.read(name) != payload:
                    raise ValueError(f"Package content verification failed for {dest.name}, {name}")
        manifest = {
            "archive": str(dest), "variant": "fomod", "version": version,
            "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
            "archive_bytes": output.stat().st_size, "dll_sha256": dll_hash,
            "source_sha256": result["source_sha256"],
            "entries": {name: hashlib.sha256(payload).hexdigest() for name, payload in sorted(fomod.items())},
            "verification": "CRC, exact contents, host build source identity, DLL identity, Windows x64 header: PASS",
            "installed": False,
        }
        staged_manifest = stage / manifest_path.name
        staged_manifest.write_text(json.dumps(manifest, indent=2) + "\n")
        output.replace(dest)
        staged_manifest.replace(manifest_path)
    return (dest,)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    try:
        for archive in package():
            print(f"Packaged and verified: {archive}")
    except (OSError, ValueError, KeyError, struct.error, zipfile.BadZipFile) as error:
        raise SystemExit(f"Packaging failed: {error}")
