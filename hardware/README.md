# Hardware configuration

`my_follower_joints.yaml` holds per-servo calibration overrides (`homing_offset`,
`range_min`/`range_max`, PID coefficients) passed to the Feetech driver via the
`joint_config_file` launch argument.

**These values are specific to one physical arm.** They come from LeRobot
calibration output, transferred **by joint name** (never by row position — a
positional transfer once gave one joint its neighbour's values, a 184° error).

Copy your own file here rather than reusing these — servo homing offsets do not
transfer between assemblies.

Two things worth knowing before you calibrate:

- **Sweep every joint through its full travel**, and home each one at its true
  physical neutral. A joint that isn't swept comes back with `range_min: 0,
  range_max: 4095` and an offset derived from wherever it happened to be sitting
  — which can put its "zero" at the mechanical stop.
- **Sync this file before relaunching ROS.** The driver writes these values into
  servo EEPROM on every startup, so a stale file burns stale values back in.

The `p_coefficient` values are not from calibration — they are gravity-sag
tuning. `elbow_flex` is raised to 48 and `wrist_flex` to 32 because those joints
carry the most load; see `docs/PROJECT_SUMMARY.md` for the gain sweep and the
error curve behind those numbers.

Usage:

```bash
ros2 launch so101_bringup follower_split.launch.py \
  hardware_type:=real use_rviz:=false \
  joint_config_file:=/path/to/my_follower_joints.yaml
```
