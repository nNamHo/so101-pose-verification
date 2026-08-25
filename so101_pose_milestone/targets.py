# targets.py — the 3 hardcoded SE(3) targets. Single source of truth:
# BOTH pose_commander.py and verify_pose.py import from here, so the pose that
# gets commanded and the pose that gets verified can never silently diverge.
#
# You now write orientation as ROLL, PITCH, YAW in DEGREES — the human-readable
# form. The conversion to a quaternion (what ROS needs on the wire) happens once,
# here, in rpy_deg_to_quat_xyzw(), so both scripts share exactly one convention.
#
#   position          = (x, y, z) in metres, in BASE_FRAME  -> WHERE the tip goes
#   orientation_rpy_deg = (roll, pitch, yaw) in degrees      -> HOW it's aimed
#     roll  = twist about the reach axis
#     pitch = nose up(-)/down(+)     (pitch = 90 -> gripper points straight down)
#     yaw   = turn left/right about vertical
#
# IMPORTANT — these numbers are PLACEHOLDERS. The SO-101 is a 5-DOF arm, so an
# arbitrary 6-DOF pose is generically UNREACHABLE. Generate real targets by
# picking joint angles and running:
#     python3 verify_pose.py --fk <t1> <t2> <t3> <t4> <t5>
# then paste the printed position + RPY here. That guarantees every target lies
# on the arm's reachable 5-DOF manifold. (Explained in EXPLANATION.md.)

import math


def rpy_deg_to_quat_xyzw(roll_deg, pitch_deg, yaw_deg):
    # Fixed-axis ROS/URDF convention: R = Rz(yaw) Ry(pitch) Rx(roll).
    # Returns the quaternion (x, y, z, w). This is the ONLY place RPY becomes a
    # quaternion, so there is exactly one convention in the whole project.
    hr = math.radians(roll_deg)  * 0.5
    hp = math.radians(pitch_deg) * 0.5
    hy = math.radians(yaw_deg)   * 0.5
    cr, sr = math.cos(hr), math.sin(hr)
    cp, sp = math.cos(hp), math.sin(hp)
    cy, sy = math.cos(hy), math.sin(hy)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (qx, qy, qz, qw)


def target_quaternion(target):
    # Convenience: hand a target dict, get its quaternion (x, y, z, w).
    return rpy_deg_to_quat_xyzw(*target["orientation_rpy_deg"])


TARGETS = [
    {
        # Target 0 — EASY: mid-workspace, far from joint limits and singularities.
        # If this one fails, the pipeline is broken, not the geometry.
        #
        # From --fk 0 -0.55 0.45 -0.45 0. Chosen so it can actually play that role:
        #   - nearest joint limit is 1.20 rad away (no joint anywhere near a stop)
        #   - 0.446 m radial = 82% of the 0.546 m boundary (not scraping it)
        #   - pitch 58 deg, i.e. 32 deg clear of the +/-90 gimbal-lock point where
        #     roll and yaw stop being well-determined
        # The previous value (0.4409, 0, 0.2148) rpy (154.12, 83.63, 153.99) sat at
        # pitch 83.63 and 90% of max reach — too marginal to be a sanity check.

        "name": "easy_mid_workspace",
        "position": (0.3078, 0.0, 0.3223),        # metres, in BASE_FRAME
        "orientation_rpy_deg": (4.54, 58.38, 5.33), # roll, pitch, yaw (degrees)
    },
    {
        # Target 1 — EXTENDED REACH: near the workspace boundary but not on it.
        # Tests IK convergence with little joint-space slack.

        "name": "extended_reach",
        "position": (0.4499, 0.0000, 0.2260),
        "orientation_rpy_deg": (89.99, 87.21, 89.99),
    },
    {
        # Target 2 — NEAR-SINGULAR (required failure case): full-extension
        # boundary singularity — arm stretched straight out, Jacobian loses
        # rank radially. Expected to fail; document HOW it fails.
        # Best generated with:  verify_pose.py --fk 0 <lift> <0> <0> <0>
        # using angles that straighten shoulder-elbow-wrist into one line.

        "name": "near_singular_full_extension",
        "position": (0.3111, 0.0000, 0.5083),
        "orientation_rpy_deg": (1.55, 29.00, 3.19),
    },
]
