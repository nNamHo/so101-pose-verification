# SO-101 Task-Space Pose Commander

Tell a robot arm where to put its gripper, and it goes there — then a **second, independent program checks whether it actually did**.

The idea: "the motion planner said it worked" and "the arm is where I asked" are two different claims. Most code only checks the first. This project checks both.

```
pose_commander.py  →  MoveIt 2 plans and moves the arm
verify_pose.py     →  reads the arm's real joint angles and
                      independently works out where the gripper ended up
```

The verifier shares no code with the planner. It derives the arm's geometry from the robot description and does its own forward-kinematics math, so it can't just repeat the planner's assumptions back.

## Results

| Target | Result | Position error | Orientation error |
|---|---|---|---|
| Mid-workspace | PASS | 0.56 mm | 0.02° |
| Extended reach | PASS | 0.36 mm | 0.00° |
| Just past the arm's reach | Inverse kinematics failed after 0.257 s | — | — |

Tolerances were 10 mm and 5°. Runs used simulated hardware.

The third target is the interesting one. It sits 5 cm beyond the arm's maximum reach of 0.546 m, and the solver didn't reject it instantly — it kept trying until it ran out of time, while reachable targets solved in milliseconds. That's the arm's Jacobian losing rank at the edge of its workspace: at full stretch, no combination of joint motions can push the gripper any further out, so the math the solver relies on breaks down.

## Running it

Needs ROS 2 Jazzy on Ubuntu 24.04, plus the [`legalaspro/so101-ros-physical-ai`](https://github.com/legalaspro/so101-ros-physical-ai) stack for the robot description and MoveIt config.

```bash
cd ~/so101_ws/src
git clone <this-repo> so101_pose_milestone
cd ~/so101_ws && colcon build --symlink-install && source install/setup.bash

# move the arm (simulated hardware)
ros2 launch so101_pose_milestone pose_commander.launch.py hardware_type:=mock

# check where it went, in a second terminal
ros2 run so101_pose_milestone verify_pose
```

## Status

Everything works on simulated hardware. Real-hardware runs are blocked by a serial communication problem in the virtual machine — one batched read command times out while single reads succeed every time. Six other hardware issues were found and fixed along the way.

## More detail

| Document | Contents |
|---|---|
| [Project summary](docs/PROJECT_SUMMARY.md) | Full results, what the tests do and don't prove, next steps |


Built with ROS 2 Jazzy, MoveIt 2, OMPL, and `ros2_control`. Kinematics follow *Modern Robotics* (Lynch & Park).
