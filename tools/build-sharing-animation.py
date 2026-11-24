#!/usr/bin/env python3
"""Generate the stationary GModPose animation and its Pandora registration."""
import argparse
from pathlib import Path
import math
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as xml

ROOT = Path(__file__).resolve().parent.parent
NAMESPACE = "GModPoseShare"
EVENT = "GModPoseShareStart"
CLIP = "ShareStill.hkx"
REVISION = "GModPoseShareRevision"
DATA = ROOT / "assets"

sys.dont_write_bytecode = True


def parameter(parent, name, value):
    xml.SubElement(parent, "hkparam", name=name).text = str(value)


def write_xml(tree, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    xml.indent(tree)
    xml.ElementTree(tree).write(destination, encoding="utf-8", xml_declaration=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pynifly-hkx", type=Path, required=True, help="PyNifly HKX module directory")
    parser.add_argument("--skeleton", type=Path, required=True, help="Local Skyrim skeleton HKX")
    parser.add_argument("--behavior-compiler", type=Path, required=True, help="Path to BehaviorCompiler.dll")
    parser.add_argument("--dotnet", default="dotnet")
    args = parser.parse_args()
    sys.path.insert(0, str(args.pynifly_hkx))
    from anim_fo4 import AnimationData, TrackData
    from anim_skyrim import load_skyrim_animation, load_skyrim_skeleton, write_skyrim_animation

    animation = DATA / "meshes/actors/character/animations" / NAMESPACE
    behaviors = DATA / "meshes/actors/character/behaviors"
    pandora = DATA / "Pandora_Engine/mod/gmodposeshare"
    for folder in (animation, behaviors):
        folder.mkdir(parents=True, exist_ok=True)
    skeleton = load_skyrim_skeleton(args.skeleton)
    frames = 31
    still = AnimationData(duration=1, num_frames=frames, num_tracks=len(skeleton.bones), frame_duration=1 / 30,
        bone_names=list(skeleton.bones), original_skeleton_name=skeleton.name,
        track_to_bone_indices=list(range(len(skeleton.bones))))
    still.tracks = [TrackData(translations=[list(pose.translation)] * frames, rotations=[list(pose.rotation)] * frames,
        scales=[[1.0, 1.0, 1.0]] * frames) for pose in skeleton.reference_pose]
    write_skyrim_animation(animation / CLIP, still, ptr_size=8)
    reread = load_skyrim_animation(animation / CLIP)
    assert reread.num_frames == frames and reread.track_to_bone_indices == still.track_to_bone_indices
    assert math.isclose(reread.duration, still.duration, abs_tol=1e-5)
    for track in reread.tracks:
        for values in (track.translations, track.rotations, track.scales):
            assert all(value == values[0] for value in values), "Stationary clip contains motion"

    source = ROOT / "tools/pandora/ShareStillBehavior.xml"
    behavior_name = f"FNIS_{NAMESPACE}_Behavior.hkx"
    reference = behaviors / behavior_name
    with tempfile.TemporaryDirectory(prefix="gmodpose-share-") as directory:
        work = Path(directory)
        output = work / behavior_name
        subprocess.run([args.dotnet, args.behavior_compiler, "pack", source, output, reference], check=True)
        packed = xml.parse(output.with_name(output.name + ".xml")).getroot()
        clip = packed.find(".//hkobject[@class='hkbClipGenerator']")
        assert clip.find("hkparam[@name='animationName']").text == f"Animations\\{NAMESPACE}\\{CLIP}"
        assert clip.find("hkparam[@name='mode']").text == "MODE_LOOPING"
        assert EVENT in {event.text for event in packed.findall(".//hkparam[@name='eventNames']/hkcstring")}
        (behaviors / behavior_name).write_bytes(output.read_bytes())
    (animation / f"FNIS_{NAMESPACE}_List.txt").write_text(f"b -md {EVENT} {CLIP}\n", encoding="ascii")

    info = xml.Element("mod", code=pandora.name)
    xml.SubElement(info, "name").text = "GModPose Sharing"
    xml.SubElement(info, "author").text = "Macinbomzh"
    write_xml(info, pandora / "info.xml")
    patch = xml.Element("patch")
    names = xml.SubElement(patch, "Append", path="#0106/eventNames")
    xml.SubElement(names, "hkcstring").text = EVENT
    infos = xml.SubElement(patch, "Append", path="#0108/eventInfos")
    parameter(xml.SubElement(infos, "hkobject"), "flags", 0)
    names = xml.SubElement(patch, "Append", path="#0106/variableNames")
    xml.SubElement(names, "hkcstring").text = REVISION
    values = xml.SubElement(patch, "Append", path="#0107/wordVariableValues")
    parameter(xml.SubElement(values, "hkobject"), "value", 1)
    infos = xml.SubElement(patch, "Append", path="#0108/variableInfos")
    variable = xml.SubElement(infos, "hkobject")
    role = xml.SubElement(xml.SubElement(variable, "hkparam", name="role"), "hkobject")
    parameter(role, "role", "ROLE_DEFAULT")
    parameter(role, "flags", 0)
    parameter(variable, "type", "VARIABLE_TYPE_INT32")
    write_xml(patch, pandora / "patches/0_master.xml")
    print(f"Verified {frames} stationary frames, {reread.num_tracks} bound tracks, and one looping behavior state")


if __name__ == "__main__":
    main()
