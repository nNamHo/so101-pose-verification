# Hardware configuration

`my_follower_joints.yaml` holds per-servo calibration overrides (`homing_offset`,
`range_min`/`range_max`, PID coefficients) passed to the Feetech driver via the
`joint_config_file` launch argument.

**These values are specific to one physical arm.** They come from LeRobot
calibration output, with `wrist_flex range_min` corrected by hand after an
encoder wrap during recording produced an invalid value of 58 (reference: 1039).

Copy your own file here rather than reusing these — servo homing offsets do not
transfer between assemblies.

Usage:

```bash
ros2 launch so101_bringup follower_split.launch.py \
  hardware_type:=real use_rviz:=false \
  joint_config_file:=/path/to/my_follower_joints.yaml
```
