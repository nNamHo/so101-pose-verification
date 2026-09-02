#!/usr/bin/env python3
"""
gravity_torque.py — reproduces every quantitative claim in docs/TECHNICAL_NOTE.md
Sections 4 and 5, from the URDF and the recorded run logs.

Run:  python3 docs/analysis/gravity_torque.py

It computes, per measured pose:
  * forward kinematics of the commanded and measured joint vectors
  * per-joint tracking error (commanded vs encoder-reported, in the servo's own frame)
  * gravity torque at each joint from the URDF inertial data
  * the implied joint stiffness tau/dtheta

The gravity-torque model is the falsification test in Section 4: if the elbow
tracking error were a static compliance under gravity, two poses with equal
computed elbow torque must show equal error. They do not.

The FK implementation here is deliberately written from the URDF joint origins
directly, independent of verify_pose.py's Product-of-Exponentials code. The
self-check below asserts the two agree, so a bug in either is caught rather than
propagated into the note.
"""

import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

# The expanded URDF is generated, not committed (see .gitignore). Regenerate with:
#   ros2 run xacro xacro $(ros2 pkg prefix so101_description)/share/so101_description/\
#     urdf/so101_arm.urdf.xacro variant:=follower use_ros2_control:=false > so101.urdf
# Pass a path as argv[1], set SO101_URDF, or drop so101.urdf in one of the paths below.
URDF_CANDIDATES = [
    os.environ.get("SO101_URDF", ""),
    os.path.join(os.path.dirname(__file__), "so101.urdf"),
    os.path.expanduser("~/so101_ws/urdf/so101.urdf"),
]


def find_urdf():
    for path in ([sys.argv[1]] if len(sys.argv) > 1 else []) + URDF_CANDIDATES:
        if path and os.path.isfile(path):
            return path
    raise SystemExit(
        "Could not find the expanded URDF. Pass it as an argument, set SO101_URDF, "
        "or regenerate it with the xacro command in the header of this file.\n"
        "Looked in: " + ", ".join(p for p in URDF_CANDIDATES if p))


CHAIN = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
G = np.array([0.0, 0.0, -9.81])

# Links whose weight loads each joint (everything distal to it).
DISTAL = {
    "shoulder_lift": ["upper_arm_link", "lower_arm_link", "wrist_link",
                      "gripper_link", "moving_jaw_so101_v1_link"],
    "elbow_flex":    ["lower_arm_link", "wrist_link",
                      "gripper_link", "moving_jaw_so101_v1_link"],
    "wrist_flex":    ["wrist_link", "gripper_link", "moving_jaw_so101_v1_link"],
}

# ---------------------------------------------------------------- measured data
# Commanded values are the final `CMD CHANGE` line of each run log (the setpoint
# actually written to the servos). Measured values are verify_pose.py's mean over
# ~12 samples of the settled pose. Position/orientation errors are verify_pose's.
RUNS = [
    dict(label="T0 P=16", target=0, P=16, pos_mm=50.11, ori_deg=11.09,
         cmd=[-0.0000, -0.5490,  0.4471, -0.4485, -0.0000],
         mea=[-0.0199, -0.5384,  0.5860, -0.4050,  0.0169]),
    dict(label="T0 P=32", target=0, P=32, pos_mm=21.57, ori_deg=4.85,
         cmd=[ 0.0001, -0.5530,  0.4544, -0.4518,  0.0001],
         mea=[ 0.0061, -0.5476,  0.5123, -0.4326,  0.0169]),
    dict(label="T0 P=48", target=0, P=48, pos_mm=20.04, ori_deg=4.59,
         cmd=[-0.0001, -0.5525,  0.4538, -0.4515, -0.0000],
         mea=[ 0.0123, -0.5446,  0.5031, -0.4326,  0.0169]),
    # Target 1 was run twice. Both trials share the commanded vector: IK is seeded
    # from the zero configuration, so the solution for a given target is deterministic.
    dict(label="T1 a P=48", target=1, P=48, pos_mm=28.14, ori_deg=5.02,
         cmd=[-0.0000,  0.8710, -1.3454,  0.4740, -0.0000],
         mea=[-0.0092,  0.7639, -1.0370,  0.3590,  0.0138]),
    dict(label="T1 b P=48", target=1, P=48, pos_mm=21.26, ori_deg=3.78,
         cmd=[-0.0000,  0.8710, -1.3454,  0.4740, -0.0000],
         mea=[-0.0153,  0.7547, -1.0508,  0.3590,  0.0123]),
]

# ------------------------------------------------------------------- kinematics
def rpy_to_R(rpy):
    a, b, c = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    Ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    Rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def homog(R, p):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def axis_rotation(axis, theta):
    a = axis / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def load_urdf(path=None):
    path = path or find_urdf()
    root = ET.parse(path).getroot()
    joints, links = {}, {}
    for j in root.findall("joint"):
        o, ax = j.find("origin"), j.find("axis")
        joints[j.get("name")] = dict(
            parent=j.find("parent").get("link"), child=j.find("child").get("link"),
            type=j.get("type"),
            xyz=np.array([float(v) for v in o.get("xyz", "0 0 0").split()]) if o is not None else np.zeros(3),
            rpy=[float(v) for v in o.get("rpy", "0 0 0").split()] if o is not None else [0, 0, 0],
            axis=np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([0, 0, 1.0]))
    for l in root.findall("link"):
        i = l.find("inertial")
        if i is None or i.find("mass") is None:
            continue
        o = i.find("origin")
        links[l.get("name")] = dict(
            m=float(i.find("mass").get("value")),
            com=np.array([float(v) for v in o.get("xyz", "0 0 0").split()]) if o is not None else np.zeros(3))
    return joints, links


def frames(theta, joints):
    """World transform of every link, plus each chain joint's origin and axis."""
    T_cur, out, jinfo = np.eye(4), {"base_link": np.eye(4)}, {}
    for name, th in zip(CHAIN, theta):
        j = joints[name]
        T_j = T_cur @ homog(rpy_to_R(j["rpy"]), j["xyz"])
        jinfo[name] = dict(q=T_j[:3, 3].copy(), w=T_j[:3, :3] @ j["axis"])
        T_cur = T_j @ homog(axis_rotation(j["axis"], th), np.zeros(3))
        out[j["child"]] = T_cur.copy()
    for _ in range(3):  # trailing fixed/unactuated links (gripper frame, jaw)
        for j in joints.values():
            if j["child"] not in out and j["parent"] in out:
                out[j["child"]] = out[j["parent"]] @ homog(rpy_to_R(j["rpy"]), j["xyz"])
    return out, jinfo


def gravity_torque(theta, joint, joints, links):
    """Static gravity torque about `joint`'s axis, from the weight of distal links."""
    out, jinfo = frames(theta, joints)
    q, w = jinfo[joint]["q"], jinfo[joint]["w"] / np.linalg.norm(jinfo[joint]["w"])
    tau = 0.0
    for name in DISTAL[joint]:
        if name not in links or name not in out:
            continue
        p_com = (out[name] @ np.append(links[name]["com"], 1.0))[:3]
        tau += float(np.dot(w, np.cross(p_com - q, links[name]["m"] * G)))
    return tau


# ------------------------------------------------------------------- self-check
def self_check(joints):
    """Assert this FK matches verify_pose.py's independent PoE implementation."""
    cases = [([0, 0, 0, 0, 0], [0.3914, 0.0, 0.2265], "home / zero configuration"),
             ([0, -0.55, 0.45, -0.45, 0], [0.3078, 0.0, 0.3223], "target-0 generating pose")]
    for theta, expect, what in cases:
        got = frames(theta, joints)[0]["gripper_frame_link"][:3, 3]
        if not np.allclose(got, expect, atol=2e-4):
            raise SystemExit(f"FK SELF-CHECK FAILED ({what}): {np.round(got,4)} != {expect}")
        print(f"  ok  {what:28s} {np.round(got, 4)}")


def main():
    joints, links = load_urdf()
    print("FK self-check against verify_pose.py (independent PoE implementation):")
    self_check(joints)

    print("\n" + "=" * 78)
    print("PER-JOINT TRACKING ERROR  (commanded vs encoder-reported, servo's own frame)")
    print("=" * 78)
    print(f"{'run':9s}" + "".join(f"{n:>12s}" for n in CHAIN))
    for r in RUNS:
        errs = [math.degrees(m - c) for c, m in zip(r["cmd"], r["mea"])]
        print(f"{r['label']:9s}" + "".join(f"{e:+11.2f}d" for e in errs))

    print("\n" + "=" * 78)
    print("GRAVITY TORQUE AND IMPLIED STIFFNESS AT THE ELBOW")
    print("=" * 78)
    print(f"{'run':9s} {'tau_elbow':>10s} {'error':>8s} {'P':>4s} {'K=tau/dth':>11s} {'dth*P':>8s}")
    for r in RUNS:
        tau = abs(gravity_torque(r["mea"], "elbow_flex", joints, links))
        err = abs(math.degrees(r["mea"][2] - r["cmd"][2]))
        K = tau / math.radians(err)
        print(f"{r['label']:9s} {tau:9.4f}N {err:7.2f}d {r['P']:4d} {K:9.2f}Nm/r {err*r['P']:8.1f}")

    t0 = abs(gravity_torque(RUNS[2]["mea"], "elbow_flex", joints, links))
    e0 = abs(math.degrees(RUNS[2]["mea"][2] - RUNS[2]["cmd"][2]))
    print("\nFALSIFICATION TEST (all at P=48):")
    print(f"  T0        tau={t0:.4f} N.m   elbow error={e0:.2f} deg")
    for r in RUNS[3:]:
        t1 = abs(gravity_torque(r["mea"], "elbow_flex", joints, links))
        e1 = abs(math.degrees(r["mea"][2] - r["cmd"][2]))
        print(f"  {r['label']} tau={t1:.4f} N.m   elbow error={e1:.2f} deg"
              f"   -> torque x{t1/t0:.2f}, error x{e1/e0:.2f}")
    print("  -> equal load, ~6x the error in both trials:")
    print("     static gravity compliance does NOT explain the pose-to-pose variation.")

    print("\nREPEATABILITY (target 1, the only repeated condition):")
    pa, pb = RUNS[3]["pos_mm"], RUNS[4]["pos_mm"]
    ea = abs(math.degrees(RUNS[3]["mea"][2] - RUNS[3]["cmd"][2]))
    eb = abs(math.degrees(RUNS[4]["mea"][2] - RUNS[4]["cmd"][2]))
    print(f"  task-space position error: {pa:.2f} and {pb:.2f} mm  (range {abs(pa-pb):.2f} mm)")
    print(f"  elbow tracking error:      {ea:.2f} and {eb:.2f} deg (range {abs(ea-eb):.2f} deg)")
    print("  -> joint-level tracking repeats tightly; the task-space figure inherits")
    print("     several mm from sub-degree differences in the other joints at 90% reach.")

    print("\nSensitivity check — elbow torque is not degenerate (lift=0, sweeping elbow):")
    for e in [-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5]:
        print(f"  elbow={e:+.2f} rad   tau={gravity_torque([0,0,e,0,0],'elbow_flex',joints,links):+.4f} N.m")

    print("\nShoulder-lift torque (the surviving coupling candidate):")
    for r in RUNS:
        print(f"  {r['label']}: tau_lift={abs(gravity_torque(r['mea'],'shoulder_lift',joints,links)):.4f} N.m")


if __name__ == "__main__":
    main()
