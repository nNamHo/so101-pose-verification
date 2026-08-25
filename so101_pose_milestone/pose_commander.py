#!/usr/bin/env python3
# pose_commander.py — sends ONE hardcoded SE(3) target to MoveIt2 via moveit_py
# and executes a joint-space trajectory on the SO-101.
#
# This script ONLY commands. It never claims success — that is verify_pose.py's job.
#
# Must be started through launch/pose_commander.launch.py so MoveItPy receives the
# full moveit_config parameter set (URDF, SRDF, kinematics, pipelines).
#
# Uses TWO-STAGE planning (IK first, then joint-space plan) so that an IK failure
# and a planning failure are reported SEPARATELY — required to characterise the
# near-singular target properly.

import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

# Dual import: works as an installed ROS 2 package and as a plain file.
try:
    from so101_pose_milestone.targets import TARGETS, target_quaternion
except ImportError:
    from targets import TARGETS, target_quaternion

# ---------------- config — confirmed against the repo's SRDF ----------------
PLANNING_GROUP = "manipulator"        # <group name="manipulator">
EE_LINK        = "gripper_frame_link" # tip_link of the manipulator chain
# Poses are expressed in the MODEL frame, which is "world" because the SRDF's
# virtual joint attaches base_link to world. The launch file publishes an
# IDENTITY world->base_link transform, so world and base_link are numerically
# coincident — verify_pose.py computing FK from base_link still agrees.
BASE_FRAME     = "world"
IK_TIMEOUT_S   = 1.0                  # IK solve budget. Reachable targets solve in
                                      # 0.05-0.18s; the margin absorbs VM jitter.
                                      # The near-singular target still fails — it
                                      # just iterates the full budget first, which
                                      # is the failure signature being documented.
RETURN_TO_REST_ON_SIGINT = True       # Ctrl+C -> try to park the arm safely
# ----------------------------------------------------------------------------


def target_index() -> int:
    # Which of TARGETS[] to command: "--target N", default 0.
    # A plain flag rather than a ROS parameter, so this matches verify_pose.py
    # exactly and works whether launched by ros2 launch or run by hand.
    # ros2 launch appends --ros-args, which this ignores.
    if "--target" in sys.argv:
        return int(sys.argv[sys.argv.index("--target") + 1])
    return 0


def build_pose_stamped(target: dict) -> PoseStamped:
    # Frame stated EXPLICITLY — an implicit frame is the classic silent bug.
    ps = PoseStamped()
    ps.header.frame_id = BASE_FRAME
    px, py, pz = target["position"]
    qx, qy, qz, qw = target_quaternion(target)  # RPY (deg) -> quaternion
    ps.pose.position.x = px
    ps.pose.position.y = py
    ps.pose.position.z = pz
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


def current_robot_state(moveit) -> RobotState:
    # Snapshot the live joint state, then release the scene lock immediately.
    state = RobotState(moveit.get_robot_model())
    with moveit.get_planning_scene_monitor().read_only() as scene:
        state.set_joint_group_positions(
            PLANNING_GROUP,
            scene.current_state.get_joint_group_positions(PLANNING_GROUP))
    state.update()
    return state


def _hard_exit(moveit):
    # MoveItCpp's C++ destructor can segfault during Python interpreter teardown
    # (process dies with exit code -11 AFTER the work completed successfully).
    # Bypassing normal cleanup avoids it. The reference implementation
    # (so101_moveit_test.py) does exactly this, for exactly this reason.
    try:
        moveit.shutdown()
    except Exception:
        pass
    try:
        rclpy.shutdown()
    except Exception:
        pass
    os._exit(0)


def main():
    rclpy.init()
    logger = get_logger("pose_commander")

    index = target_index()
    target = TARGETS[index]
    logger.info(f"Target [{index}] '{target['name']}' "
                f"pos={target['position']} rpy_deg={target['orientation_rpy_deg']} "
                f"frame='{BASE_FRAME}' ee_link='{EE_LINK}'")

    # MoveItCpp HARDCODES its joint-state topic to "joint_states", ignoring the
    # planning_scene_monitor_options value in YAML. The bringup is namespaced,
    # so the topic must be remapped here or the planning scene never sees state.
    moveit = MoveItPy(
        node_name="pose_commander",
        remappings={"joint_states": "/follower/joint_states"},
    )
    arm = moveit.get_planning_component(PLANNING_GROUP)

    # ---- Ctrl+C safety: try to park at the SRDF "rest" pose before dying ----
    shutdown_event = threading.Event()

    def _sigint(_sig, _frm):
        if shutdown_event.is_set():
            return
        shutdown_event.set()
        if not RETURN_TO_REST_ON_SIGINT:
            return
        logger.info("SIGINT — attempting to park at 'rest'...")
        try:
            arm.set_start_state_to_current_state()
            arm.set_goal_state(configuration_name="rest")
            res = arm.plan()
            if res:
                moveit.execute(res.trajectory, controllers=[])
                time.sleep(2.0)
        except Exception as e:
            logger.error(f"Park failed: {e}")

    signal.signal(signal.SIGINT, _sigint)

    # ---- STAGE 1: inverse kinematics, as its own reportable step ----
    # Seed IK from the all-zero configuration, NOT from wherever the arm happens
    # to rest. Newton-Raphson IK converges or fails depending on its seed: from
    # the folded rest pose (elbow fully flexed, joints against limits) it failed
    # 100% of the time, while the same target solved in 0.05s from a raised pose.
    # Zero is mid-workspace for this arm, and it is exactly the seed the mock
    # pipeline used (GenericSystem boots at zeros) — the proven-working case.
    # The PLAN below still starts from the arm's true current state.
    goal_state = current_robot_state(moveit)
    goal_state.set_joint_group_positions(
        PLANNING_GROUP,
        [0.0] * len(goal_state.get_joint_group_positions(PLANNING_GROUP)))
    goal_state.update()
    t0 = time.monotonic()
    ik_ok = goal_state.set_from_ik(
        PLANNING_GROUP, build_pose_stamped(target).pose, EE_LINK, IK_TIMEOUT_S)
    ik_time = time.monotonic() - t0

    if not ik_ok:
        logger.error(
            f"IK FAILED after {ik_time:.3f}s for '{target['name']}'. "
            "The pose has no joint solution — either it is off the 5-DOF "
            "reachable manifold, or (for the near-singular target) the Jacobian "
            "is too ill-conditioned to converge. Record: FAILURE MODE = IK.")
        _hard_exit(moveit)
    goal_state.update()
    logger.info(f"IK solved in {ik_time:.3f}s")

    # ---- STAGE 2: joint-space plan to the IK solution ----
    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=goal_state)

    t0 = time.monotonic()
    plan_result = arm.plan()
    plan_time = time.monotonic() - t0

    if not plan_result:
        logger.error(
            f"PLANNING FAILED after {plan_time:.2f}s (IK had succeeded). "
            "Record: FAILURE MODE = PLANNING, not IK — a joint solution exists "
            "but no valid path to it was found.")
        _hard_exit(moveit)
    logger.info(f"Plan found in {plan_time:.2f}s. Executing...")

    # ---- execute via the joint_trajectory_controller ----
    t0 = time.monotonic()
    moveit.execute(plan_result.trajectory, controllers=[])
    logger.info(f"Execution returned after {time.monotonic() - t0:.2f}s. "
                "NOT proof of convergence — the controller reports success on "
                "its own feedback. Now run: verify_pose.py --target "
                f"{index}")

    _hard_exit(moveit)


if __name__ == "__main__":
    main()