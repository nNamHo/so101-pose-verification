# SO-101 Pose Commander — Project Summary

Task-space pose commanding with **independent** forward-kinematics verification. One script commands MoveIt2; a second script computes FK from the robot's actual reported joint state using its own Product-of-Exponentials implementation and reports numeric error. "Verified" means the arm arrived — not that a library said so.

---

## Status at a glance

| Layer | State |
|---|---|
| Math kernel (PoE FK) | ✅ Validated — analytic tests + real SO-101 geometry |
| ROS 2 package | ✅ Builds, installs, launches |
| MoveIt2 integration | ✅ Plans and executes |
| Mock hardware pipeline | ✅ All 3 targets run end-to-end |
| Singularity failure case | ✅ Characterised and documented |
| Real hardware execution | ❌ Blocked — VM/USB transport limitation |

**Everything except the final hardware step is complete and validated.**

---

## What was built

**`targets.py`** — single source of truth for the 3 target poses, imported by both scripts so commanded and verified poses can never diverge. Input format is 6 human-readable numbers: position (m) + roll/pitch/yaw (degrees). `rpy_deg_to_quat_xyzw()` is the *only* place RPY becomes a quaternion, so both scripts share one convention.

**`pose_commander.py`** — sends one SE(3) target via `moveit_py`. Two-stage design: IK first (`set_from_ik`), then joint-space plan to that solution. This separation is what makes the failure analysis possible — an IK failure and a planning failure are reported distinctly rather than collapsing into one opaque result.

**`verify_pose.py`** — the independent verifier. Zero MoveIt code. Fetches the URDF, walks the chain base→EE, extracts screw axes Sᵢ = (ω, −ω×q) and home matrix M in a single zero-configuration pass, then evaluates FK with its own Rodrigues/screw-exponential math. Utility modes: `--home` (prints M), `--fk θ₁…θ₅` (generates reachable targets), `--maxreach` (grid-searches the workspace boundary = the singularity).

**Packaging** — `package.xml`, `setup.py`, `setup.cfg`, resource marker, launch file, config YAML. Plus `bootstrap_pkg.sh` and `verify_pkg.sh` helper scripts.

---

## Validation results

**Math kernel** — tested offline against an analytic 2R planar arm (5 angle pairs, exact match), a rotated-origin/offset-axis case (catches the classic PoE-from-URDF bug where the joint axis isn't rotated into the base frame), and quaternion round-trips. RPY→quaternion verified against the FK code's own rotation convention across 2000 random orientations: max difference 8×10⁻¹⁶.

**Against real geometry** — chain resolves to exactly the right 5 joints (`shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll`), matching the official SO-101 motor list minus the gripper. Home pose 0.452 m from base; plausible for this arm class.

**Mock hardware, all 3 targets:**

| # | Target | Result | Position err | Orientation err |
|---|---|---|---|---|
| 0 | easy_mid_workspace | PASS | 0.56 mm | 0.02° |
| 1 | extended_reach | PASS | 0.36 mm | 0.00° |
| 2 | near_singular (5 cm beyond boundary) | **IK FAILED @ 0.257 s** | — | — |

Target 0 and 1 passing proves MoveIt's kinematics and the independent PoE FK **agree** — two different implementations, sub-millimetre convergence.

**The singularity result.** `--maxreach` found the workspace boundary at **0.546 m** radial. Target 2 was placed 5.0 cm beyond it, scaled radially along the same direction with orientation held fixed. IK failed after **0.257 s** against a 0.2 s budget — meaning it *iterated until it ran out of time* rather than instantly rejecting an out-of-bounds request. That's Newton–Raphson failing to converge on an ill-conditioned Jacobian (MR Ch 5.3, 6.2), not a trivial bounds check. Contrast with Targets 0/1, which solved in milliseconds.

---

## Failure log — real hardware

Chronological, with what each step eliminated.

| # | Symptom | Root cause | Resolution |
|---|---|---|---|
| 1 | Arm lurches to impossible pose (z<0) on bringup | No calibration on this machine | Ran LeRobot calibration |
| 2 | Calibration ran but nothing changed | Driver **writes** config to EEPROM each init, overwriting it | Use `joint_config_file` |
| 3 | `wrist_flex range_min = 58` | Encoder wrap during calibration (ref: 1039) | Corrected manually |
| 4 | `ModelSeries::kSts` error, all 6 joints | **`feetech_ros2_driver/` was an empty git submodule** — a stale/wrong driver was loading | `git submodule update --init --recursive` |
| 5 | Driver still not built | `--packages-up-to so101_bringup` doesn't include it (no dependency) | `colcon build --packages-select feetech_ros2_driver` |
| 6 | `read_exact [Read timeout]` at init | **Hardcoded 5 ms serial timeout** vs VM scheduling jitter | Raised to 100 ms → init now succeeds |
| 7 | `sync_read [Read timeout]` at runtime | **Unresolved** | — |

**On #7 — what was eliminated:** timeout raised to 500 ms (a ~100× margin over the sub-millisecond expected round-trip — so the reply is *absent*, not late); update rate dropped 100→50→20→5 Hz; `return_delay_time` tried at both 0 and 20; USB cable swapped; baud confirmed at 1 Mbaud (servos answer there, silent at 115200); `bus_check.py` single reads work reliably every time.

**The distinguishing fact:** single-register reads always succeed; the batched `sync_read` never does. Sync-read broadcasts one request and expects six servos to reply in sequence on a half-duplex bus — tight framing that a USB-serial bridge's chunked buffering, plus VirtualBox passthrough, appears to break. `check_head` failing means the packet header isn't being found.

`sync_read` is called at exactly one site (`feetech_ros2_driver.cpp:262`) with no fallback path, no mode parameter, no config option.

---

## Also fixed along the way

- **MoveItCpp ignores `joint_state_topic` in YAML** — it hardcodes `"joint_states"`. The namespaced topic requires the `remappings=` kwarg on the `MoveItPy` constructor. Found in the reference implementation's own comment.
- **Missing `.trajectory_execution(...)`** in our launch — without it `execute()` has no controller to send trajectories to.
- **Segfault on exit** (`exit code -11`) — MoveItCpp's C++ destructor racing Python teardown. Fixed with `os._exit(0)`, same as the reference implementation.
- **Race condition** — commander started before controllers were up; added a 10 s `TimerAction`.
- **Name corrections** vs. initial placeholders: `PLANNING_GROUP` arm→**manipulator**, `EE_LINK` gripper_link→**gripper_frame_link**, `planner_id` RRTConnectkConfigDefault→**RRTConnect**. 3 of 5 guesses were wrong — which is exactly why the spec insisted on checking.

---

## What to do next

**1. Dual-boot Ubuntu (recommended).** Eliminates USB passthrough buffering entirely and removes the scheduling jitter behind the overrun warnings. ~30 minutes, and it's the only remaining move that addresses the actual root cause. Alternative: run bringup on a Raspberry Pi, keep the VM for development.

**2. Then, if `sync_read` still fails**, patch `feetech_ros2_driver.cpp:262` to loop individual reads instead of one batched read. Six round-trips per cycle instead of one — slower, but single reads are proven to work. Worth doing *after* dual-booting, so you can tell whether the patch was actually necessary.

**3. Then finish the hardware runs.** Expect millimetre-scale errors rather than mock's sub-millimetre — that gap *is* the result (backlash, servo resolution, gravity sag). Watch Target 1 especially: longest moment arm means largest gravity sag, so an orientation error that passed in mock may fail on hardware. That's a finding, not a bug.

**Optional, no hardware needed — the boundary sweep.** Set Target 2 to +1, +2, +3, +4 cm beyond 0.546 m and record IK solve time and success at each. You'd map the *transition* from convergence to failure, showing IK degrading progressively as conditioning worsens rather than switching off cleanly. ~15 minutes, only `targets.py` changes, and it's the strongest Jacobian result available to you.

---

## Local modifications to third-party code

Will be overwritten by `git submodule update` — record them:

- `feetech_driver/include/feetech_driver/serial_port.hpp:69` — timeout 5 ms → 500 ms
- `so101_bringup/config/ros2_control/follower_split_controllers.yaml` — `update_rate` 100 → 5
- `~/so101_ws/my_follower_joints.yaml` — local calibration overrides
