# SO-101 Task-Space Pose Commander

Tell a robot arm where to put its gripper. Then have a **second, independent program check whether it actually got there.**

"The motion planner said it worked" and "the arm is where I asked" are two different claims. Most robotics code only checks the first. This one checks both.

Built on a real **SO-101** 5-DOF arm with Feetech servos, running ROS 2 Jazzy and MoveIt 2.

## How it works

```
targets.py          three goal poses, shared by both programs below
      │
      ├──►  pose_commander.py   MoveIt 2 plans a path and moves the arm
      │
      └──►  verify_pose.py      reads the arm's real joint angles, works out
                                where the gripper ended up, reports the error
```

The verifier shares no code with the planner. It reads the arm's geometry from the robot description and does its own forward-kinematics math, so it can't just repeat the planner's assumptions back. When the two agree, that's two separate implementations reaching the same answer.

## Results

On real hardware, target 0:

| | Position error | Orientation error |
|---|---|---|
| Measured | **20.0 mm** | **4.6°** |
| Tolerance | 25 mm | 5° |

Repeated readings agree exactly. In simulation, where there's no gravity or gear slop, the same code lands within **0.56 mm** — so the math is sound and the 20 mm is physical: mostly the elbow servo sagging under the weight of the forearm.

A third target sits 5 cm beyond the arm's maximum reach. It's supposed to fail, and *how* it fails is the interesting part — the solver keeps trying until it runs out of time, instead of rejecting it straight away.

## Running it

Needs ROS 2 Jazzy on Ubuntu 24.04, plus the [`legalaspro/so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai) stack.

```bash
cd ~/so101_ws && colcon build --symlink-install && source install/setup.bash
```

Move the arm (use `hardware_type:=real` for a real robot):

```bash
ros2 launch so101_pose_milestone pose_commander.launch.py hardware_type:=mock target_index:=0
```

Then check where it went, in a second terminal:

```bash
cd ~/so101_ws/src/so101_pose_milestone/so101_pose_milestone && python3 verify_pose.py --target 0
```

The verifier also runs on its own, with no robot and no ROS nodes:

```bash
python3 verify_pose.py --home                  # gripper pose at zero joint angles
python3 verify_pose.py --fk 0 0.3 -0.6 0.3 0   # pose for given joint angles
python3 verify_pose.py --maxreach              # find the edge of the workspace
```

> **Calibration is per-arm.** Servo offsets don't transfer between arms — make your own with `lerobot-calibrate` before running on hardware.

## More detail

| Document | Contents |
|---|---|
| [docs/PROJECT_SUMMARY.md](docs/PROJECT_SUMMARY.md) | Full results, the debugging record, and every bug found along the way |
| [hardware/README.md](hardware/README.md) | Per-arm calibration notes |

Built with ROS 2 Jazzy, MoveIt 2, OMPL, and `ros2_control`. Kinematics follow *Modern Robotics* (Lynch & Park).

Developed with AI assistance (Claude) for code scaffolding and debugging. Hardware integration, testing, and design decisions are my own.
