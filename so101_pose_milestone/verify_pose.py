#!/usr/bin/env python3
# verify_pose.py — INDEPENDENT check that the arm actually reached the target.
# No MoveIt code in this file: it reads the URDF for geometry and /joint_states
# for where the servos really are, computes forward kinematics itself, and
# reports the numeric error against the commanded pose.
#
# Modes:
#   verify_pose.py                     verify arm against TARGETS[TARGET_INDEX]
#   verify_pose.py --home              print EE pose at zero joint angles
#   verify_pose.py --fk t1 t2 t3 t4 t5 print EE pose for given joint angles
#                                      (use this to GENERATE reachable targets)
#   add --urdf <path> to any mode to read a URDF file instead of the live topic.

import sys
import math
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Dual import: works as an installed ROS 2 package and as a plain file.
try:
    from so101_pose_milestone.targets import TARGETS, target_quaternion
except ImportError:
    from targets import TARGETS, target_quaternion

# ---------------- config — confirmed against the repo's SRDF ----------------
BASE_LINK    = "base_link"          # <chain base_link="base_link" ...
EE_LINK      = "gripper_frame_link" # ... tip_link="gripper_frame_link"/>
URDF_FILE    = "/home/peter-ho/so101_ws/urdf/so101.urdf"                   # optional URDF path; "" = read live topic
# Topics are namespaced by the bringup (default namespace "follower").
ROBOT_DESCRIPTION_TOPIC = "/follower/robot_description"
JOINT_STATE_TOPIC       = "/follower/joint_states"

TARGET_INDEX = 0                    # which target to verify against
POS_TOL_M    = 0.010                # 1 cm
ORI_TOL_RAD  = math.radians(5)      # 5 deg
# ----------------------------------------------------------------------------


# ============================ math (Modern Robotics) ============================

def skew(w):
    # [w] — skew-symmetric matrix, so that [w] p = w x p   (MR Ch 3.2.1)
    return np.array([[0.0, -w[2],  w[1]],
                     [w[2],  0.0, -w[0]],
                     [-w[1], w[0],  0.0]])


def rot_exp(w, theta):
    # Rodrigues' formula: rotation by theta about unit axis w   (MR eq. 3.51)
    W = skew(w)
    return np.eye(3) + math.sin(theta) * W + (1.0 - math.cos(theta)) * (W @ W)


def screw_exp(S, theta):
    # Matrix exponential of a revolute screw S = (w, v), |w| = 1   (MR eq. 3.88)
    # G(theta) v is the translation induced by rotating about an axis that does
    # not pass through the origin.
    w, v = S[:3], S[3:]
    W = skew(w)
    G = np.eye(3) * theta + (1.0 - math.cos(theta)) * W \
        + (theta - math.sin(theta)) * (W @ W)
    T = np.eye(4)
    T[:3, :3] = rot_exp(w, theta)
    T[:3, 3] = G @ v
    return T


def rpy_to_R(r, p, y):
    # URDF fixed-axis convention: R = Rz(yaw) Ry(pitch) Rx(roll)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def R_to_rpy_deg(R):
    # Inverse of the above, in degrees (matches targets.py's input format).
    if abs(R[2, 0]) < 1.0 - 1e-9:
        pitch = math.asin(-R[2, 0])
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:  # pitch = +/-90: roll and yaw are not separable, fix yaw = 0
        pitch = math.copysign(math.pi / 2, -R[2, 0])
        roll = math.atan2(-R[1, 2], R[1, 1])
        yaw = 0.0
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def make_T(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def quat_xyzw_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def R_to_quat_xyzw(R):
    # Shepperd's method: divide by whichever component is largest (stable).
    tr = np.trace(R)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x, y, z = 0.25*s, (R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        w, x, y, z = (R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        w, x, y, z = (R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s
    else:
        s = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        w, x, y, z = (R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s
    return (x, y, z, w)


def rotation_angle_between(R_a, R_b):
    # Smallest rotation angle separating two orientations — the honest
    # single-number orientation error.
    c = (np.trace(R_a.T @ R_b) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, c)))


# ============================ URDF -> PoE quantities ============================

def get_urdf_xml(timeout_s=10.0, path_override=None):
    # A file if given (--urdf or URDF_FILE), else the latched
    # /robot_description topic. TRANSIENT_LOCAL so a late subscriber still
    # receives the message that was published once at startup.
    path = path_override or URDF_FILE
    if path:
        with open(path) as f:
            return f.read()

    node = Node("urdf_fetcher")
    got = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, ROBOT_DESCRIPTION_TOPIC,
                             lambda msg: got.setdefault("xml", msg.data), qos)
    deadline = time.monotonic() + timeout_s
    while "xml" not in got and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if "xml" not in got:
        raise RuntimeError(
            f"Could not fetch {ROBOT_DESCRIPTION_TOPIC} within timeout and no "
            "URDF file was given. Is your robot bringup running?")
    return got["xml"]


def parse_chain(urdf_xml, base_link, ee_link):
    # Walk backwards EE -> base (each link has exactly one parent joint, so the
    # backward walk is unambiguous), then reverse to get base -> EE order.
    root = ET.fromstring(urdf_xml)
    joints_by_child = {j.find("child").attrib["link"]: j
                       for j in root.findall("joint")}
    chain, link = [], ee_link
    while link != base_link:
        j = joints_by_child.get(link)
        if j is None:
            raise RuntimeError(
                f"No joint has child link '{link}' — check BASE_LINK/EE_LINK "
                f"against your URDF (links found: {sorted(joints_by_child)})")
        chain.append(j)
        link = j.find("parent").attrib["link"]
    chain.reverse()
    return chain


def build_poe(chain):
    # One pass at the ZERO configuration:
    #  - accumulate each joint's fixed <origin> into a running transform T,
    #  - at each revolute joint read its axis w and a point q on it, both in the
    #    BASE frame, giving the screw axis S = (w, -w x q),
    #  - after the last joint, T IS the home configuration M.
    T = np.eye(4)
    screws, names, limits = [], [], []
    for j in chain:
        origin = j.find("origin")
        xyz, rpy = np.zeros(3), np.zeros(3)
        if origin is not None:
            if "xyz" in origin.attrib:
                xyz = np.array([float(v) for v in origin.attrib["xyz"].split()])
            if "rpy" in origin.attrib:
                rpy = np.array([float(v) for v in origin.attrib["rpy"].split()])
        T = T @ make_T(rpy_to_R(*rpy), xyz)

        jtype = j.attrib["type"]
        if jtype in ("revolute", "continuous"):
            axis_el = j.find("axis")
            a_local = (np.array([float(v) for v in axis_el.attrib["xyz"].split()])
                       if axis_el is not None else np.array([1.0, 0.0, 0.0]))
            # The URDF <axis> is in the joint's own frame — rotate it into base.
            w = T[:3, :3] @ a_local
            w = w / np.linalg.norm(w)
            q = T[:3, 3]
            screws.append(np.concatenate([w, -np.cross(w, q)]))
            names.append(j.attrib["name"])
            lim = j.find("limit")
            limits.append((float(lim.attrib["lower"]), float(lim.attrib["upper"]))
                          if lim is not None else (-math.pi, math.pi))
        elif jtype != "fixed":
            raise RuntimeError(f"Unsupported joint type '{jtype}' "
                               f"(joint '{j.attrib['name']}')")
    return names, screws, T, limits


def fk_poe(screws, thetas, M):
    # Product of Exponentials, space form (MR eq. 4.13):
    #   T(theta) = e^{[S1]t1} e^{[S2]t2} ... e^{[Sn]tn} M
    T = np.eye(4)
    for S, th in zip(screws, thetas):
        T = T @ screw_exp(S, th)
    return T @ M


# ============================ joint state acquisition ===========================

def get_joint_angles(joint_names, timeout_s=10.0):
    # Read what the servos actually report, mapped BY NAME — JointState does
    # not guarantee ordering, so index-based reading is unsafe.
    node = Node("joint_state_reader")
    latest = {}

    def cb(msg):
        for n, p in zip(msg.name, msg.position):
            latest[n] = p

    node.create_subscription(JointState, JOINT_STATE_TOPIC, cb, 10)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(n in latest for n in joint_names):
            break
    node.destroy_node()

    missing = [n for n in joint_names if n not in latest]
    if missing:
        raise RuntimeError(f"Joints never appeared on {JOINT_STATE_TOPIC}: {missing}")
    return [latest[n] for n in joint_names]


# ==================================== report ====================================

def print_pose(tag, T):
    p = T[:3, 3]
    rpy = R_to_rpy_deg(T[:3, :3])
    q = R_to_quat_xyzw(T[:3, :3])
    print(f"{tag}")
    print(f"  position       (m)  :  x={p[0]:+.4f}  y={p[1]:+.4f}  z={p[2]:+.4f}")
    print(f"  orient rpy   (deg)  :  roll={rpy[0]:+.2f}  pitch={rpy[1]:+.2f}  yaw={rpy[2]:+.2f}")
    print(f"  orient quat (xyzw)  :  ({q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f})")


def main():
    rclpy.init()
    args = sys.argv[1:]

    # Optional --urdf <path>, so offline runs need no ROS bringup.
    urdf_path = None
    if "--urdf" in args:
        i = args.index("--urdf")
        urdf_path = args[i + 1]
        del args[i:i + 2]

    urdf = get_urdf_xml(path_override=urdf_path)
    joint_names, screws, M, limits = build_poe(parse_chain(urdf, BASE_LINK, EE_LINK))
    print(f"[verify_pose] chain {BASE_LINK} -> {EE_LINK}: "
          f"{len(joint_names)} revolute joints: {joint_names}")

    # ---- print the home pose (all joints at zero) ----
    if args and args[0] == "--home":
        print_pose("EE pose at zero configuration (M):", M)
        rclpy.shutdown()
        return

    # ---- find the workspace boundary (max radial reach) = the singularity ----
    if args and args[0] == "--maxreach":
        # Grid-search the joints that extend the arm radially. At the maximum,
        # no joint velocity can push the EE further out, so the Jacobian loses
        # rank -> this is the boundary singularity (MR Ch 5.3).
        n = len(joint_names)
        idx = list(range(1, min(4, n)))          # lift, elbow, wrist_flex
        grids = [np.linspace(limits[i][0], limits[i][1], 45) for i in idx]
        best = None
        for a in grids[0]:
            for b in grids[1]:
                for c in (grids[2] if len(grids) > 2 else [0.0]):
                    th = [0.0] * n
                    th[idx[0]], th[idx[1]] = float(a), float(b)
                    if len(idx) > 2:
                        th[idx[2]] = float(c)
                    p = fk_poe(screws, th, M)[:3, 3]
                    r = float(np.hypot(p[0], p[2]))
                    if best is None or r > best[0]:
                        best = (r, list(th))
        r, th = best
        print(f"\nMax radial reach = {r:.4f} m at theta = {[round(t, 4) for t in th]}")
        print_pose("Boundary (singular) pose:", fk_poe(screws, th, M))
        print("  ^ ON the boundary. For the required FAILURE case, push the")
        print("    target a few cm FURTHER out than this.")
        rclpy.shutdown()
        return

    # ---- print FK for given joint angles (target generation) ----
    if args and args[0] == "--fk":
        thetas = [float(a) for a in args[1:]]
        if len(thetas) != len(joint_names):
            raise SystemExit(f"--fk needs {len(joint_names)} angles "
                             f"(order: {joint_names})")
        print_pose(f"FK pose for theta={thetas}:", fk_poe(screws, thetas, M))
        print("  ^ paste 'position' and 'orient rpy (deg)' into targets.py — "
              "this pose is guaranteed reachable.")
        rclpy.shutdown()
        return

    # ---- default: verify the real arm against the commanded target ----
    target = TARGETS[TARGET_INDEX]
    thetas = get_joint_angles(joint_names)
    T_actual = fk_poe(screws, thetas, M)

    p_target = np.array(target["position"])
    R_target = quat_xyzw_to_R(target_quaternion(target))

    pos_err = float(np.linalg.norm(T_actual[:3, 3] - p_target))
    ori_err = rotation_angle_between(R_target, T_actual[:3, :3])

    print(f"\n=== verification vs target [{TARGET_INDEX}] '{target['name']}' ===")
    print(f"measured joint angles (rad): {[f'{t:+.4f}' for t in thetas]}")
    print_pose("FK of ACTUAL joint state:", T_actual)
    print_pose("COMMANDED target:", make_T(R_target, p_target))
    print(f"\nposition error    : {pos_err*1000:7.2f} mm   (tol {POS_TOL_M*1000:.0f} mm)")
    print(f"orientation error : {math.degrees(ori_err):7.2f} deg  (tol {math.degrees(ORI_TOL_RAD):.0f} deg)")

    pos_ok, ori_ok = pos_err <= POS_TOL_M, ori_err <= ORI_TOL_RAD
    print(f"\nRESULT: {'PASS' if (pos_ok and ori_ok) else 'FAIL'}  "
          f"(position {'OK' if pos_ok else 'OUT'}, "
          f"orientation {'OK' if ori_ok else 'OUT'})")

    rclpy.shutdown()


if __name__ == "__main__":
    main()