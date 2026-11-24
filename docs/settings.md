# Settings and usage

GModPose is a standalone SKSE plugin for 64-bit Skyrim SE and AE. Use the
matching SKSE and Address Library for your runtime. XPMSSE is optional.
The Face Poser gaze sliders use [Mfg Fix NG](https://www.nexusmods.com/skyrimspecialedition/mods/133568).

## Install a profile

Open `GModPose.zip` in your mod manager and choose one FOMOD option.

| Profile | Behavior |
| --- | --- |
| Standalone | Local posing, including native seated editing. Use this if you don't use PlayerHub. |
| PlayerHub | Shares standing, seated, and weapon-drawn poses through PlayerHub. Requires Pandora 5. Use this if you use PlayerHub. |

For the PlayerHub profile, enable **GModPose Sharing** in Pandora 5 and
generate behaviors. The installer supplies the patch and animation that
Pandora uses to generate the game's behaviors.

## Default controls

| Input | Action |
| --- | --- |
| Apostrophe | Open or exit the pose editor |
| Mouse and WASD | Look and move the camera |
| Left click and drag | Move a handle |
| Right click and drag | Rotate a handle |
| Mouse wheel while holding a handle | Adjust depth or roll |
| Arrows and Space/Ctrl | Slide, raise, or lower the whole pose |
| Left Alt | Toggle Nudge mode |
| Backslash | Toggle Mirror mode |
| Middle click on the chest, hand, or face circle | Open Body, Finger, or Face Poser |
| Comma / Period | Undo / redo |
| Semicolon | Open or close settings |

In Nudge mode, click an aimed handle to latch its beam. The mouse moves
the camera. Arrows and Space/Ctrl edit the latched handle, and the opposite
mouse button switches between movement and rotation. Press Alt again to exit.

In Finger Poser, drag a triangle toward the palm to curl or sideways to
spread. Hover over a triangle and scroll to twist that joint.

## Preferences and presets

The editor reads and saves settings in `Data/SKSE/Plugins/GModPose.ini`.
The Settings panel covers key bindings, rotation sensitivity, yaw, pitch,
and roll inversion, breathing movement, and shortcut guide placement.

Moving a wrist or ankle preserves its bend relative to the forearm or calf.
Torso edits pin supported feet and carry raised feet with the body. Head
turns share some rotation with the neck.
An ankle diamond has a short bar underneath when the foot is supported.
During a torso drag, the bar shows which feet the grip keeps planted.

Khajiit, Argonians, and custom races with compatible tails get tail diamonds.
Drag the tip to move it through space. Drag a middle diamond to bend the tail
while keeping the tip in place. The tail stays attached to the pelvis. Hold
the rotate binding and scroll to twist a segment, or rotate the tip in place.

A visible drawn weapon has a rotation-only triangle. Right-drag or use the
amber Nudge beam to rotate the weapon independently of the hand. Sheathed
weapons and raised fists have no weapon triangle.

See the [pose format](pose.schema.json) and the [face pose format](face-pose.schema.json).
