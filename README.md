# GModPose

Pose your Skyrim player character GMOD style. Press `'` (Apostrophe) in game to open the position editor. Press `;` (Semicolon) while in the editor to open settings.

[Settings and usage](docs/settings.md) · [Build on Fedora 44](#build-on-fedora-44) · [Third-party notices](THIRD_PARTY_NOTICES.md)

## Build on Fedora 44

Run from this repository's directory. Install the x64 MSVC wrappers and Windows SDK using [msvc-wine](https://github.com/mstorsjo/msvc-wine). The paths below use `~/gmodpose-toolchain` as an example for the installed toolchain and its Wine prefix.

```bash
sudo dnf install -y cmake ninja-build git python3 wine binutils

export PATH="$HOME/gmodpose-toolchain/msvc/bin/x64:$PATH"
export WINEPREFIX="$HOME/gmodpose-toolchain/wine-prefix"
export WINEDEBUG=-all

cmake -S . -B build/dependencies -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="$PWD/cmake/msvc-wine.cmake" \
  -DCMAKE_BUILD_TYPE=Release -DGMODPOSE_DEPENDENCIES_ONLY=ON
cmake --build build/dependencies
cmake --install build/dependencies --prefix "$PWD/build/prefix"

CMAKE_PREFIX_PATH="$PWD/build/prefix" GMODPOSE_CACHE_ROOT= python3 tools/build-host.py
```

The output is `dist/GModPose.zip`. It contains the FOMOD with both install options, the Windows x64 SKSE plugin, and license notices.

The first build downloads pinned dependencies. `tools/build-host.py` builds the Windows DLL and creates and verifies the install ZIP. Its build log is at `build/standalone-build/host-build.log`.

The PlayerHub profile includes the prebuilt `GModPose.Pandora.dll`. Rebuild the C# project separately only when changing the Pandora patch source. After installing the PlayerHub profile, enable **GModPose Sharing** in Pandora 5 and generate behaviors.
