# External controller interface

The app can load an optional trusted native controller through the
`external_control_library` launch argument. The default is empty, so ordinary
teleoperation, recording and homing do not depend on an external controller.

The interface is declared in `include/vive_vr_ros2/external_control.hpp`.
An extension exports `openarm_create_external_control_v1` and implements
`ExternalControl`. Build it against the same ROS distribution and interface header
as the host. A missing library or incompatible entry point prevents bridge startup.
The library remains loaded until process exit so its ROS callbacks remain valid.

The host supplies measured right-arm state and joint ranges, command publication,
health checks, hold and stop callbacks. An active extension excludes VR engagement
and homing. Hardware faults and operator disable stop the extension. The extension
owns its input validation, trajectory timing and command bounds; the existing
hardware-driver checks remain in the app. This is an in-process trusted extension,
not a security boundary or a substitute for physical collision protection.

Pass its absolute shared-library path when starting hardware:

```sh
ros2 launch vive_vr_ros2 openarm_real.launch.py \
  host_ip:=LAPTOP_WIFI_IP external_control_library:=/absolute/path/controller.so
```

Do not run a second robot launch alongside an existing launch. The app's UI,
camera server, recording and dataset export remain independent of the extension.
