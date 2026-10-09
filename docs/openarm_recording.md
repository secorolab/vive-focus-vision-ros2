# Record right-arm demonstrations

This recorder observes the existing real OpenArm controller. Normal recording never commands motors. The explicit hold-Y shortcut invokes
the existing guarded return-to-zero routine. SmolVLA weights are not needed for recording and no
data is uploaded. The first dataset uses the **right arm and gripper**, the main
D455f, and the **right-wrist D405**. Left-camera preview is optional and is not
included in this dataset.

## Start

Keep the usual real launch running on domain 84. Stop the previous camera-preview
script with Ctrl+C so only one process owns the cameras. In a new terminal:

```zsh
source /opt/ros/jazzy/setup.zsh
cd ~/Desktop/vive-focus-vision-ros2
/usr/bin/python3 scripts/openarm_cameras.py \
  --record-root ~/robot_ai/episodes/right_cube_v1 \
  --task "Put the cube in the basket"
```

The single connected D405 is now assigned to **right**, not left. For multiple
D405s use explicit `--left` and `--right` stable `/dev/v4l/by-id/...` paths.
The service captures at 30 FPS and records aligned samples at 15 FPS. Do not run
a second camera service. The current ROS installation is sufficient: no ROS
rebuild or IK dependency upgrade is required for recording.

The default LeRobot Python executable is
`~/robot_ai/lerobot_env/bin/python`; override with `--lerobot-python` if needed.
It runs separately from the system Python interpreter used for ROS and cameras.

Install the new APK, then reopen VR. Lock the world, center controls, and use the
usual enable service when ready to teleoperate. Park the unused left arm.

## Record one attempt

The VR status appears when `--record-root` is enabled:

1. Wait for **Ready**. Release A/B/X before issuing the first command.
2. Press **A** (right controller) to start. Perform the task with the existing
   rear-trigger arm toggle and joystick gripper controls.
3. Press **A** again to stop recording. This does **not** stop robot motion:
   disengage arm following using its usual rear-trigger toggle.
4. Press **B** to save a successful demonstration, or hold **X** for 1.2 seconds
   to discard the unsaved attempt. Discard is only available after stopping.
5. Wait for saving to finish before starting the next attempt. Reset the cube
   and arm between episodes, not while recording.

The controls require WORLD LOCKED. Unlocking, missing controller tracking, camera
loss, stale feedback, inactive controllers or a driver fault invalidates the
attempt. The status shows the reason. An invalid attempt can only be discarded;
it is never automatically added to the training dataset. Other robot safety
checks still belong to the existing controller.

Episodes stop for review after 120 seconds. At least 2 GB free disk is required
to begin; recording stops below 1 GB. There is no automatic start or Save.

## Stored data and timing

Each frame contains:

| LeRobot feature | Meaning |
| --- | --- |
| `observation.images.main` | Main RGB camera, 640×480 |
| `observation.images.right` | Right-wrist RGB camera, 640×480 |
| `observation.state` | Right J1–J7 in radians, followed by finger joint position in metres |
| `action` | The same ordered positions from the arm and gripper controllers' `reference.positions` |
| `task` | The instruction supplied at startup |

Actions are controller reference positions, including their interpolation, not
raw VR poses or inferred actions copied from measured feedback. Later policy
execution must use the same action meaning, joint ordering, units and 15 Hz rate.

The observer subscribes to `/joint_states`, the two right controller
`controller_state` topics, right driver health and VR input. It only calls the
read-only `list_controllers` service. It does not publish robot commands.

Sampling uses a 100 ms buffer for alignment (not added to robot control latency).
It selects samples within 60 ms of each recording instant and rejects cross-source
skew above 80 ms. These are **host timestamps**, not hardware-synchronized camera
exposures. ROS timestamps must be fresh. Source timestamps are retained in raw
records for checking timing before training. Missing samples are not silently
replaced with old images.

## Files, saving and recovery

The session folder contains:

- `session.json`: fixed task, feature meaning and sampling settings.
- `pending/`: unsaved or interrupted attempts, excluded from training.
- `episodes/`: accepted raw JPEGs and timestamped JSONL records.
- `dataset/`: symlink to the latest complete local LeRobot v3 dataset.
- `exports/`: generated video/Parquet/metadata snapshot.
- `export.log`: most recent conversion output.

For this initial 3–5-episode workflow, Save rebuilds the dataset from all accepted
raw episodes, then switches the dataset symlink atomically. This is deliberately
simple and recoverable but grows slower as the session grows. Do not train or
read the dataset concurrently while saving another version.

If export fails, the raw episode remains accepted, and the previous good dataset
is retained. **B retries export**. A recording process crash leaves its pending
attempt on disk; it is not automatically accepted on restart. Relaunch with the
same arguments to continue the session. Changed task/schema requires a new folder.
To regenerate the dataset manually:

```zsh
~/robot_ai/lerobot_env/bin/python \
  ~/Desktop/vive-focus-vision-ros2/scripts/openarm_export_lerobot.py \
  ~/robot_ai/episodes/right_cube_v1
```

Record 3–5 trials first, then inspect decoded videos, state/action values and the
retained timing records before collecting a larger dataset. Recording quality
and camera placement still need a real user demonstration check. Nothing here
trains or deploys a policy.


## Y: discard and return to zero

While WORLD LOCKED, hold **Y (left secondary button) for two seconds**. A countdown
warning appears. Release before two seconds to abort. After the hold, the unsaved
attempt is stopped and discarded, then **both arms** return to existing calibrated
zero at the existing bounded return speed. Grippers are unchanged. Saved episodes
are never discarded. This is not encoder recalibration.

Keep the direct return path clear. **A cancels an active return**. Missing VR input
or recorder status polling cancels the return; the existing driver's feedback and
fault checks also remain active. No new episode can start during the return.
After completion, VR remains disarmed: use the usual enable command again before
teleoperating. If return fails, inspect `return_to_zero.log`; hold X to acknowledge
or retry Y after fixing the cause. A failed return does not restore a discarded
attempt. Y's simulation-reset action is suppressed while the recorder UI is active.


### Lower-load camera configuration

After the USB investigation, keep cameras at the tested 15 FPS and record at
10 Hz. Use a separate session directory: sampling rate is fixed per dataset.

```zsh
source /opt/ros/jazzy/setup.zsh
cd ~/Desktop/vive-focus-vision-ros2
/usr/bin/python3 scripts/openarm_cameras.py --fps 15 --record-fps 10 \
  --record-root ~/robot_ai/episodes/right_cube_10hz \
  --task "Put the cube in the basket"
```

Stop the previous preview service before starting this. This configuration's
future policy playback rate must match 10 Hz. Do not mix 10 and 15 Hz episodes
within one session. The original 30-camera/15-recording option remains available.
