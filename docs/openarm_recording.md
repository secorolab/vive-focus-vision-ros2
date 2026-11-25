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

1. Wait for **Ready**. Release A/B/X/Y before issuing the first command.
2. Press **X** (left controller) to start. Perform the task with the existing
   arm-following and gripper controls.
3. Press **X** again to stop recording. This does **not** stop robot motion:
   disengage arm following using its usual toggle.
4. Press **Y** to save a successful demonstration, or hold **A** for 1.2 seconds
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
- `exports/`: separately named video/Parquet/metadata snapshots; older exports are retained.
- `export.log`: most recent conversion output.

**Y saves the raw episode locally and immediately returns to idle.** You can press
X to start the next episode, or hold B to return to zero. Saving does not run
video conversion and does not rebuild the training dataset.

After the recording session, use **Export saved episodes to LeRobot** in the
laptop panel. Export rebuilds all accepted episodes and atomically replaces the
`dataset` symlink to point to the newest complete version. Older export folders
are retained, and raw episodes are never replaced by export. Run it after recording to avoid CPU and disk contention.
Do not train from the dataset while replacing it. If conversion fails or is
interrupted, the raw saved episodes and previous complete dataset remain safe;
run Export again. Pending attempts are never included.

A recording process crash leaves its pending attempt on disk; it is not
automatically accepted on restart. Relaunch with the same arguments to continue.
Changed task/schema requires a new folder. To export manually:

```zsh
~/robot_ai/lerobot_env/bin/python \
  ~/Desktop/vive-focus-vision-ros2/scripts/openarm_export_lerobot.py \
  ~/robot_ai/episodes/right_cube_30hz
```

Record 3–5 trials first, then inspect decoded videos, state/action values and the
retained timing records before collecting a larger dataset. Recording quality
and camera placement still need a real user demonstration check. Nothing here
trains or deploys a policy.


## B: discard and return to zero

While WORLD LOCKED, hold **B (right secondary button) for two seconds**. A countdown
warning appears. Release before two seconds to abort. After the hold, the unsaved
attempt is stopped and discarded, then **both arms** return to existing calibrated
zero at the existing bounded return speed. Grippers are unchanged. Saved episodes
are never discarded. This is not encoder recalibration.

Keep the direct return path clear. **X cancels an active return**. Missing VR input
or recorder status polling cancels the return; the existing driver's feedback and
fault checks also remain active. No new episode can start during the return.
After successful completion, release all controller buttons: VR arms and grippers
are enabled automatically with fresh tracking and a locked world. Arm following
still requires a new side-grip press on green. If automatic enable is blocked,
the UI explains that manual Enable VR is needed. If return fails, inspect `return_to_zero.log`; hold A to acknowledge
or retry B after fixing the cause. A failed return does not restore a discarded
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

### Testing 30 Hz recording

For 30 Hz episodes, capture both cameras at 60 FPS so nearest-frame selection has
headroom. Stop the previous camera service first, then run:

```zsh
source /opt/ros/jazzy/setup.zsh
source ~/openarm_ws/install/setup.zsh
source ~/vive_vr_ws/install/setup.zsh
cd ~/Desktop/vive-focus-vision-ros2
/usr/bin/python3 scripts/openarm_cameras.py --fps 60 --record-fps 30 \
  --record-root ~/robot_ai/episodes/right_cube_30hz \
  --task "Put the cube in the basket"
```

This increases camera/recording load, not motor update rate or movement speed.
The headset preview still requests up to 15 FPS. Existing 10 Hz episodes stay in
their original dataset; do not relabel them as 30 Hz. `/status` reports requested
and measured camera FPS. Validate a short recorded episode before collecting a
larger dataset: stale camera frames, stale feedback and synchronization failures still
stop recording. A short benchmark does not establish stability during extended
teleoperation. Future policy target timing must match the dataset rate.

Video export automatically tests NVIDIA H.264 NVENC and uses it when available.
It falls back to CPU H.264 if the GPU encoder cannot initialize. To require the GPU,
pass `--encoder gpu` to `scripts/openarm_export_lerobot.py`; `--encoder cpu` forces
software encoding. Raw episode saving stays separate and does not encode video.

After saving with Y, hold right-controller B for two seconds to return both arms
to zero, then release B. X cancels return. Unsaved attempts are discarded by this
shortcut; saved episodes remain. During return, VR arm/gripper controls are locked
and grippers keep their opening. Return uses a smooth profile capped at 0.10 rad/s
and keeps holding at zero. After a successful B-button return, VR is enabled once
both controllers are freshly tracked, the world is locked, and buttons are released.
Press side grip on green to follow again; grippers require a fresh trigger press.
Failed/cancelled returns remain disarmed. If automatic enable is blocked for ten
seconds, use Enable VR manually after resolving the reported condition.
Standalone homing remains disarmed by default.

The laptop panel scrolls with the mouse wheel or its right-hand scrollbar.
Its export counters separate waiting episodes from exported episodes and total
saved recordings. Exporting retains the raw recordings. **Export only newest
saved episode** writes a separate `dataset-latest` link; the full `dataset` link
is unchanged. Successful exports record their source episode IDs for these counts.

Repeated camera frames are allowed when they remain within the camera freshness
and synchronization bounds. Each 30 Hz sample retains its own joint state/action
and original camera timestamp, even when its image repeats the previous sample.
A stopped/stale camera still blocks recording.
