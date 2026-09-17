# Installation {#page_install}

Two halves install independently: the ROS 2 workspace on the PC, and the Unity toolchain that
produces the headset APK. Neither needs the other to build.

## ROS 2 side

### Setup

One script, run from the workspace root after cloning:

```bash
mkdir -p ~/work/p/vrws && cd ~/work/p/vrws
git clone git@github.com:secorolab/vive-focus-vision-ros2.git src/vive-vr-ros2
vcs import src < src/vive-vr-ros2/dependencies.repos    # apt install python3-vcstool

python3 -m venv --system-site-packages venv && source venv/bin/activate
./src/vive-vr-ros2/scripts/setup.sh          # --no-scenes, --no-unity to skip parts
```

**`--system-site-packages`** is not optional: without it the venv hides `rclpy`, `launch` and
the rest of ROS 2's Python packages. The script installs the scene tooling into whatever
virtualenv is active, downloads the kitchen asset packs and fetches the VIVE plugin. Then:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_SCENES=ON
```

The rest of this section is what that script does, and why.

### Workspace dependencies

This repository is a single ROS 2 package, cloned into a workspace's `src/` rather than built in
place:

```bash
mkdir -p ~/work/p/vrws/src && cd ~/work/p/vrws
git clone git@github.com:secorolab/vive-focus-vision-ros2.git src/vive-vr-ros2
vcs import src < src/vive-vr-ros2/dependencies.repos    # apt install python3-vcstool
```

`vive_vr_ros2` depends on `mj_kdl_wrapper`, which needs the secorolab Orocos KDL fork built as
workspace package. This is not optional and not a preference: the ROS distro ships
`liborocos-kdl.so.1.5` and `python3-pykdl` with the **same SONAME and module name** as the fork,
and a single process can hold only one — the loader keeps the first and silently drops the
other's symbols. Building the fork as a workspace package is what makes every package consume one
shared KDL.

Neither dependency is committed here; both are their own git repositories, listed in
`dependencies.repos` so `vcs import` places them beside this one:

| Repository | Branch |
|---|---|
| `github.com/vamsikalagaturu/mj_kdl_wrapper` | `dev` |
| `github.com/secorolab/orocos_kinematics_dynamics` | `feature/achd_fixed_joint` |

The resulting workspace:

```
~/work/p/vrws/
├── colcon.meta                 # BUILD_EXAMPLES/BUILD_TESTS off for the wrapper
└── src/
    ├── vive-vr-ros2/           # this repository
    ├── mj_kdl_wrapper/
    └── orocos_kinematics_dynamics/
```

`mj_kdl_wrapper` ships no `package.xml`. colcon still builds it: the `colcon-cmake` extension
discovers any directory with a `CMakeLists.txt` and names the package after its `project()` call,
and `<depend>mj_kdl_wrapper</depend>` in this package still orders the build correctly. It will
not appear in `ros2 pkg list`, which is expected.

### System packages

```bash
sudo apt install ros-jazzy-rosbridge-suite \
    cmake g++ libeigen3-dev libglfw3-dev libgl-dev libegl-dev ffmpeg
```

### Python environment

Use a venv, and create it **with `--system-site-packages`**: without that flag the venv hides
`rclpy`, `launch` and the rest of the ROS 2 Python packages, and nothing in the workspace runs
inside it.

```bash
cd ~/work/p/vrws
python3 -m venv --system-site-packages venv
source venv/bin/activate
```

Only the scene tooling needs anything beyond ROS. RoboCasa builds the kitchen; robosuite is
reinstalled from master afterwards because RoboCasa pins an older one than its own code needs:

```bash
pip install "git+https://github.com/robocasa/robocasa.git"
pip install --force-reinstall --no-deps \
    "git+https://github.com/ARISE-Initiative/robosuite.git@master"
python3 -m robocasa.scripts.download_kitchen_assets --type tex fixtures_lw objs_lw
```

Those three asset packs are the kitchen's textures, its fixtures and the objects that are made
graspable. `--type all` is around 10 GB, most of it Objaverse and AI-generated sets that nothing
here loads.

### Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

With the environment above active, add `-DBUILD_SCENES=ON` and the build also exports the
scenes in `scenes/` into `~/.cache/vive_vr_ros2/scenes/`. It is off by default so that a code
build needs neither RoboCasa nor the assets.

`colcon.meta` belongs to the workspace, not to this package, and turns off the wrapper's examples
and tests. `unity/COLCON_IGNORE` is kept as a safety net for anyone who runs colcon from inside
the repository: colcon otherwise discovers a package inside Unity's Android build temp and aborts
the whole build.

MuJoCo is not searched for here at all. `mj_kdl_wrapper` fetches it, validates its
`mjVERSION_HEADER`, and exports it as `mujoco::mujoco`; this package links that target, so it
cannot end up on a different copy than the wrapper did — a stray `/opt/mujoco-3.8.0` would
otherwise be found and silently mismatch the wrapper's ABI. Point the wrapper elsewhere with
`-DMJ_KDL_MUJOCO_DIR=...` and this package follows.

`mujoco::mujoco` carries MuJoCo alone. Linking `mj_kdl_wrapper::mj_kdl_wrapper` would work too,
but it also brings KDL, glfw and OpenGL, and `vive_vr_core` stays free of those so an
application can embed `BodyPosePublisher` without them — see [Embedding](embedding.md).

## Unity side

### The VIVE plugin

The VIVE OpenXR plugin is a 361 MB tarball, deliberately not in git — it is a redistributable
release artefact, so it belongs in a setup step rather than in history. `manifest.json` references
it by relative path, so Unity cannot open the project until it is fetched:

```bash
./scripts/fetch_vive_plugin.py
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

`docs/` is a CMake project in its own right, needing nothing but Doxygen — no ament, no rclcpp,
no MuJoCo. That is what CI uses, and it is faster locally:

```bash
cmake -S docs -B build/docs "-DDOXYGEN_EXECUTABLE=$DOXYGEN"
cmake --build build/docs           # -> build/docs/docs/html/index.html
```

The ROS package pulls the same project in with `add_subdirectory` when asked, so there is one
implementation and the two paths cannot drift:

```bash
colcon build --packages-select vive_vr_ros2 \
    --cmake-args -DBUILD_DOCS=ON "-DDOXYGEN_EXECUTABLE=$DOXYGEN"
```

`BUILD_DOCS` is off by default and stays in the CMake cache, so a later plain `colcon build`
fails the version check unless the newer doxygen is still reachable. Turn it back off with
`-DBUILD_DOCS=OFF`.

Doxygen takes `include/` for the API and these guides as pages; the relative links between them
resolve in both GitHub and the generated site. `WARN_AS_ERROR` is on, because a broken reference
or a page missing from `INPUT` is otherwise a silent hole in the published site.

`.github/workflows/docs.yml` publishes to GitHub Pages on every push to `main` and `dev`, and
builds (without publishing) on pull requests. Both branches publish to the same site, so
whichever runs last wins.

Next: [Running](run.md), or [The Unity client](unity.md) to build the APK.
