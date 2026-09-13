# Installation {#page_install}

Two halves install independently: the ROS 2 workspace on the PC, and the Unity toolchain that
produces the headset APK. Neither needs the other to build.

## ROS 2 side

### Workspace dependencies

`vr` depends on `mj_kdl_wrapper`, which needs the secorolab Orocos KDL fork built as its own
workspace package. This is not optional and not a preference: the ROS distro ships
`liborocos-kdl.so.1.5` and `python3-pykdl` with the **same SONAME and module name** as the fork,
and a single process can hold only one — the loader keeps the first and silently drops the
other's symbols. Building the fork as a workspace package is what makes every package consume one
shared KDL.

Neither dependency is committed here; both are their own git repositories:

```bash
cp -r ~/work/ms/src/mj_kdl_wrapper ~/work/ms/src/orocos_kinematics_dynamics src/
```

or clone them directly:

| Repository | Branch |
|---|---|
| `github.com/vamsikalagaturu/mj_kdl_wrapper` | `dev` |
| `github.com/secorolab/orocos_kinematics_dynamics` | `feature/achd_fixed_joint` |

`mj_kdl_wrapper` ships no `package.xml`. colcon still builds it: the `colcon-cmake` extension
discovers any directory with a `CMakeLists.txt` and names the package after its `project()` call,
and `<depend>mj_kdl_wrapper</depend>` in this package still orders the build correctly. It will
not appear in `ros2 pkg list`, which is expected.

### System packages

```bash
sudo apt install ros-jazzy-rosbridge-suite \
    cmake g++ libeigen3-dev libglfw3-dev libgl-dev libegl-dev ffmpeg
```

### Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Two workspace files matter here. `colcon.meta` turns off the wrapper's examples and tests, which
this workspace does not need. `unity/COLCON_IGNORE` stops colcon descending into the Unity
project — without it, colcon discovers a package inside Unity's Android build temp and aborts the
entire build.

MuJoCo (3.9.0) is taken from `~/.cache/mj_kdl_wrapper/mujoco-3.9.0`, the same copy the wrapper
fetches, so both link one MuJoCo. Override with `-DVR_MUJOCO_DIR=...`. **No system paths are
searched on purpose:** a stray `/opt/mujoco-3.8.0` would otherwise be found and silently mismatch
the wrapper's ABI.

## Unity side

### The VIVE plugin

The VIVE OpenXR plugin is a 361 MB tarball, deliberately not in git — it is a redistributable
release artefact, so it belongs in a setup step rather than in history. `manifest.json` references
it by relative path, so Unity cannot open the project until it is fetched:

```bash
./tools/fetch_vive_plugin.sh
```

Idempotent and checksum-pinned; re-running it on a good file does nothing. `VIVE_OPENXR_VERSION`
fetches a different release, after which `manifest.json` needs the new filename.

The `.unitypackage` on the releases page is **not** the plugin — it is a 2 KB bootstrap script
that downloads it, and is unnecessary here.

### Unity Hub and editor

```bash
sudo install -d /etc/apt/keyrings
curl -fsSL https://hub.unity3d.com/linux/keys/public \
  | sudo gpg --dearmor -o /etc/apt/keyrings/unityhub.gpg
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/unityhub.gpg] \
https://hub.unity3d.com/linux/repos/deb stable main" \
  | sudo tee /etc/apt/sources.list.d/unityhub.list
sudo apt update && sudo apt install unityhub

unityhub --headless install --version 6000.0.83f1 \
  --module android android-sdk-ndk-tools android-open-jdk --childModules

sudo apt install android-tools-adb     # if adb is not already present
```

### Licence

The Hub CLI cannot sign in, so this step is manual exactly once: open Unity Hub, sign in, take a
**Personal** licence. Until then the editor refuses to open a project. Modern Unity records it at
`~/.config/unity3d/Unity/licenses/UnityEntitlementLicense.xml`, not the old `Unity_lic.ulf` — a
missing `.ulf` is not a sign that anything is wrong.

### Versions

Pinned, and `manifest.json` agrees with `packages-lock.json`, so nothing drifts when the project
is opened.

| Component | Version |
|---|---|
| Unity Editor | 6000.0.83f1 (Unity 6 LTS) |
| VIVE OpenXR Plugin | 2.5.1 |
| Unity glTFast | 6.15.1 |
| OpenXR Plugin | 1.16.1 |
| XR Hands | 1.5.1 |
| XR Interaction Toolkit | 3.0.11 |
| Newtonsoft Json | 3.2.2 |

## Generating the documentation

The guides read as plain markdown, so this is only needed for the HTML site with the C++ API
reference.

**Doxygen 1.16 or newer is required.** Ubuntu 24.04 packages 1.9.8, which generates readable but
visibly broken pages: 1.9.x ships its own `div.fragment` rule, and that selector is more specific
than the theme's `.fragment`, so code blocks lose their padding and the navigation tree's icons
overlap their labels. Take the official binary instead:

```bash
curl -fsSL -o /tmp/doxygen.tar.gz \
  https://github.com/doxygen/doxygen/releases/download/Release_1_16_1/doxygen-1.16.1.linux.bin.tar.gz
mkdir -p ~/.local/opt && tar -xzf /tmp/doxygen.tar.gz -C ~/.local/opt
export DOXYGEN=~/.local/opt/doxygen-1.16.1/bin/doxygen
```

```bash
colcon build --packages-select vr --cmake-args -DBUILD_DOCS=ON "-DDOXYGEN_EXECUTABLE=$DOXYGEN"
cmake --build build/vr --target docs      # -> build/vr/docs/html/index.html
```

`BUILD_DOCS` stays in the CMake cache, so a later plain `colcon build` will fail the version
check unless the newer doxygen is still reachable. Turn it back off with `-DBUILD_DOCS=OFF`.

`BUILD_DOCS` is off by default. Doxygen takes `include/` for the API and these guides as pages;
the relative links between them resolve in both GitHub and the generated site.

Without configuring the ROS package at all — what CI uses, and faster locally:

```bash
./tools/build_docs.sh            # -> build/docs/docs/html/index.html
```

It substitutes the same `Doxyfile.in`, so the two paths cannot drift, and **fails on any doxygen
warning** — a broken reference or a page missing from `INPUT` is otherwise a silent hole in the
published site.

`.github/workflows/docs.yml` publishes to GitHub Pages on every push to `main` and `dev`, and
builds (without publishing) on pull requests. Both branches publish to the same site, so
whichever runs last wins.

Next: [Running](run.md), or [The Unity client](unity.md) to build the APK.
