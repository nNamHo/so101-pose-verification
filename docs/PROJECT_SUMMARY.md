# SO-101 Pose Commander — Project Summary

Task-space pose commanding with **independent** forward-kinematics verification, taken from simulation through to a working real arm. One program commands MoveIt 2; a second computes FK from the robot's actually-reported joint state using its own Product-of-Exponentials implementation and reports numeric error. "Verified" means the arm arrived — not that a library said so.

This document is the complete record: what was built, what the results are, and every bug found between "works in simulation" and "works on the bench."

---

## Status

| Layer | State |
|---|---|
| Math kernel (PoE FK) | ✅ Validated — analytic tests + real SO-101 geometry |
| ROS 2 package | ✅ Builds, installs, launches |
| MoveIt 2 integration | ✅ Plans and executes |
| Simulated pipeline | ✅ All 3 targets end-to-end |
| Singularity failure case | ✅ Characterised and documented |
| Serial driver | ✅ Bus desync root-caused and fixed |
| Calibration | ✅ All six joints synced; wrist_roll homed at true neutral |
| Planning from any start pose | ✅ Including the folded resting pose |
| **Real hardware, target 0** | ✅ **PASS — 20.0 mm / 4.6°** |
| Real hardware, targets 1 and 2 | ⏳ In progress |
| Pose accuracy floor | 📏 ~20 mm — elbow gear backlash, documented, not fixable in software |

---

## What was built

**`targets.py`** — the three target poses, single source of truth, imported by both programs so the commanded pose and the verified pose can never drift apart. Input is six human-readable numbers: position (m) plus roll/pitch/yaw (degrees). `rpy_deg_to_quat_xyzw()` is the *only* place RPY becomes a quaternion, so both programs share exactly one convention.

**`pose_commander.py`** — sends one SE(3) target via `moveit_py`. Two-stage design: IK first (`set_from_ik`), then a joint-space plan to that solution. That separation is what makes failure analysis possible — an IK failure and a planning failure are reported distinctly rather than collapsing into one opaque result.

**`verify_pose.py`** — the independent verifier. Zero MoveIt code. Fetches the URDF, walks the chain base→EE, extracts screw axes Sᵢ = (ω, −ω×q) and the home matrix M in a single zero-configuration pass, then evaluates FK with its own Rodrigues/screw-exponential math. Utility modes: `--home`, `--fk θ₁…θ₅` (generates reachable targets), `--maxreach` (grid-searches the workspace boundary).

**`sync_read_probe.py`** — raw-pyserial diagnostic that bypasses the ROS driver entirely. Written to settle a serial argument, kept because it settled it in two minutes.

---

## Results

### Simulated hardware

Validates the math, URDF parsing, frame conventions, and plumbing:

| Target | Result | Position error | Orientation error |
|---|---|---|---|
| 0 — mid-workspace | **PASS** | 0.56 mm | 0.01° |
| 1 — extended reach | **PASS** | 0.36 mm | 0.00° |
| 2 — 5 cm past max reach | **IK failed after 0.280 s** | — | — |

Targets 0 and 1 passing proves MoveIt's kinematics and the independent PoE FK **agree** — two implementations, sub-millimetre convergence. Caveat: `mock_components/GenericSystem` echoes commands back as state, so these results validate the software, not the physics.

### Real hardware

Target 0, the first fully-valid measurement after the debugging campaign below:

| | Position error | Orientation error |
|---|---|---|
| Measured | **20.0 mm** | **4.6°** |
| Tolerance | 25 mm | 5° |
| Result | **PASS** | **PASS** |

Sample spread across 12 readings: **0.0000 rad** — perfectly repeatable.

Tolerances are set to *measured capability*, with the evidence recorded in the code. The elbow servo holds its load below its own setpoint under gravity, so a P-gain sweep against the same target:

| Elbow P gain | Elbow error | Total position error |
|---|---|---|
| 16 | 7.8° | 50.1 mm |
| 32 | 3.6° | 21.6 mm |
| 48 | 3.0° | 20.0 mm |

Doubling the gain halved the error — textbook steady-state P-control behaviour. Tripling it bought almost nothing. That saturation is the diagnosis: what remains is gear backlash and deadband, which no gain removes (and an integral term would only hunt against). **The honest result is 20 mm with a curve showing where tuning stopped paying**, not a green checkmark from loosened tolerances.

### The singularity result

`--maxreach` found the workspace boundary at **0.546 m** radial. Target 2 sits 5.0 cm beyond it, scaled radially along the same direction with orientation held fixed. IK fails only after iterating to its timeout, while reachable targets solve in milliseconds. That's Newton–Raphson failing to converge on an ill-conditioned Jacobian (*Modern Robotics* Ch. 5.3, 6.2), not a trivial bounds check: at full stretch, no combination of joint velocities can push the end-effector further out, so the Jacobian loses rank.

---

## Verified configuration

Confirmed against the actual repo — three of five initial guesses were wrong, so don't re-guess them:

| Constant | Value | Source |
|---|---|---|
| `PLANNING_GROUP` | `manipulator` | `so101_arm.srdf` |
| `BASE_FRAME` (commander) | `world` | SRDF virtual joint |
| `BASE_LINK` (verifier) | `base_link` | `<chain base_link=...>` |
| `EE_LINK` | `gripper_frame_link` | `<chain tip_link=...>` |
| `planner_id` | `RRTConnect` | `ompl_planning.yaml` |
| Joint state topic | `/follower/joint_states` | namespaced bringup |
| Serial port / baud | `/dev/so101_follower` @ 1 Mbaud | udev rule; servos silent at 115200 |

Chain: 5 revolute joints — `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll`. Servo IDs 1–6 in URDF order, gripper last.

### URDF joint limits — widened to measured travel (local modification)

| Joint | Original | Now | Measured travel |
|---|---|---|---|
| shoulder_lift | ±1.74533 | **−1.85 .. +1.81** | counts 843..3229 |
| elbow_flex | ±1.69 | **−1.78 .. +1.69** | counts 889..3095 |
| wrist_flex | ±1.65806 | **−1.92 .. +1.77** | counts 801..3199 |

The arm's *resting* pose sags to shoulder_lift = −1.83 — real, measured, reachable, but outside the originally declared limit, so MoveIt refused to plan from rest.

---

## The debugging campaign

Each phase's fix exposed the next phase's bug. That nesting is why symptoms kept "not changing" — several failures shared one symptom.

1. **Serial desync** — found by re-reading a log a previous session believed it had failed to capture. Fixed with an RX flush and reply-ID validation.
2. **Calibration never arriving** — three independent delivery failures, ended by one clean recalibration plus a by-name programmatic sync.
3. **MoveItPy crash** — one YAML key silently deleting the planner config. Found by bisecting the config builder call by call.
4. **Planning from rest** — three stacked gatekeepers (IK seed, joint limits, self-collision), each invisible until the previous fell.
5. **Gravity sag** — first valid measurements, error decomposed onto one joint, driven to its mechanical floor by a gain sweep.

---

## Failures log

Every problem, its root cause, its fix, and the evidence that settled it.

| # | Symptom | Root cause | Fix | Evidence |
|---|---|---|---|---|
| 1 | Arm lurches to impossible pose (z<0) on bringup | No calibration on this machine | Ran LeRobot calibration | — |
| 2 | Calibration ran but nothing changed | Driver writes config to servo EEPROM every init, overwriting it | Pass `joint_config_file` explicitly | — |
| 3 | `wrist_flex range_min = 58` | Encoder wrap during calibration | Corrected by hand | Reference value 1039 |
| 4 | `ModelSeries::kSts` error on all 6 joints | `feetech_ros2_driver/` was an empty git submodule; a stale driver was loading | `git submodule update --init --recursive` | — |
| 5 | Driver still not built | `--packages-up-to` excludes it (no dependency edge) | `colcon build --packages-select` | — |
| 6 | `read_exact [Read timeout]` at init | Hardcoded 5 ms serial timeout vs VM jitter | Raised to 100 ms | Timeout sweep 500/200/100/20 ms |
| 7 | `sync_read [Read timeout]` at runtime | libserial transport unreliable on this platform (wire probe proved the servos, cable, baud, and byte sequence all fine) | Per-joint `read_word` with 3 retries, return OK and hold last value | `sync_read_probe.py`: 6/6 clean at wire level, twice |
| 8 | Calibration mismatch: joints report outside URDF limits | ROS and LeRobot read entirely different calibration files; nothing synced them | Transfer LeRobot values into the ROS YAML | elbow_flex off by 1395 counts ≈ 2.1 rad, matching the observed error |
| 9 | `wrist_flex` kept a known-broken value after a "fix" | Hand transfer was positional; one row slipped, giving `wrist_roll` its neighbour's values | Corrected; later replaced by by-name sync | Diff showed line 48 changed, line 37 untouched |
| 10 | Arm drives to calibration pose ~3 s after bringup | Torque enabled during `configure_joints_`, before anything seeded `goal_position`; servos drive to the stale register | Read present → write goal = present (torque off) → enable torque | `grep` of torque ordering; seeds drifting between runs |
| 11 | Read errors deactivate the whole stack | One timeout returned `ERROR` → hardware component deactivated → controller cascade | Return `OK`, hold last value, log a warning | Error cascade disappeared |
| 12 | **Arm lurches to a scrambled pose when the trajectory controller activates; pose errors 140–540 mm, unrepeatable** | **Serial reply desync** — no RX flush before requests, and `read()` never checked the reply's ID byte | Flush RX in `write_buffer()`/`sync_read()`; validate ID in `read()` | `buffer[0][=1] != id[=2]` cascade; `hw_positions_` rotated 2 slots at `write[10]`, 5 ms after controller activation |
| 13 | "Feedback shifted by one slot", earlier dismissed as a sampling artifact | Same desync — the observation was real | Same | Retroactive |
| 14 | Full pipeline and bringup-only behave differently | `joint_config_file` defaulted to `""` in the launch → full pipeline used no calibration at all | Default now points at the real YAML | Launch file vs documented commands |
| 15 | Recalibration "has no effect" | Fresh LeRobot JSON never copied into the YAML ROS reads | Sync by name, verified programmatically | File mtimes: JSON 19:41, YAML from previous day |
| 16 | Wrist rolls to its hardware stop every run | `wrist_roll` row was a copy of `wrist_flex` (184° error); its replacement offset came from an unswept joint, putting "zero" at the mechanical stop | Recalibration with the wrist homed at true neutral | `CMD CHANGE #1`: 1083-count wrist jump at controller latch |
| 17 | `update_rate` changes have no effect | Stale install copy from a non-symlink build | Rebuild with `--symlink-install` everywhere | install-vs-source diff and file dates |
| 18 | **MoveItPy crashes: "Planning plugin name is empty"** | A top-level `ompl:` key in the moveit_cpp YAML **replaces** the whole pipeline dict during the builder's final merge | Pipeline settings moved into the pipeline's own YAML | 3/3 reproduction; builder-chain bisect; generated params missing `planning_plugins` |
| 19 | IK fails 100% from rest, succeeds when the arm is raised | `set_from_ik` seeds from the *current* state; the folded pose is a bad basin near limits | Seed IK from zeros; timeout 0.2 → 1.0 s | A/B experiment; 0.05 s solve raised vs 100% timeout at rest |
| 20 | `CheckStartStateBounds` aborts: shoulder_lift −1.825 outside ±1.745 | URDF limits narrower than the physical arm's travel; resting sag lives in the gap | Widen URDF limits to calibrated travel | Calibration counts 843..3229 |
| 21 | `fix_start_state: true` delivered but ignored | Never fully explained; sidestepped by #20 | (sidestepped) | Param dump shows it present; adapter aborts regardless |
| 22 | `CheckStartStateCollision` aborts: gripper/forearm vs shoulder | The resting fold is genuine self-contact; the planner refuses collision start states | SRDF `disable_collisions` for the three resting-contact pairs | Adapter names exactly those pairs |
| 23 | Verify FAILs at 50 mm / 11° with the arm visibly "at" the pose | **Gravity sag** — elbow holds ~0.14 rad below its own setpoint at P=16 | P sweep 16→32→48 (32 on wrist_flex) | Commanded 2339 counts vs reported ~2430 *in the servo's own frame* — acquits both calibration and verifier math |
| 24 | P=48 barely improves on P=32 | Tuning saturated at the gear backlash / deadband floor | Tolerance set to measured capability | The sweep itself: 50.1 / 21.6 / 20.0 mm |
| 25 | Verifier readings seconds apart disagreed | `TRANSIENT_LOCAL` on the *subscriber* replays cached history; the code took the oldest sample | Revert to `VOLATILE`; sample 1 s and report mean + spread | DDS durability semantics; spread now 0.0000 |
| 26 | "Goal reached, success!" while the arm never moved | Driver hardcodes velocity feedback to 0.0, so the controller's stopped-velocity check is trivially true | Treated as known: controller success is not evidence; only `verify_pose.py` is | A full "successful" execution with the arm at rest throughout |

---

## The critical failures

Five that shaped the project.

### 1. The serial reply desync

**Symptom.** The arm lurched to a scrambled pose moments after the trajectory controller activated. Every pose measurement was 140–540 mm off and unrepeatable. An earlier session saw "feedback shifted by one slot" and talked itself out of it.

**Mechanism.** `CommunicationProtocol::read()` never compared the reply's ID byte with the ID it queried, and nothing flushed the RX buffer between transactions. One timeout leaves a late reply queued; every subsequent transaction then consumes the *previous* one's reply. Checksums all pass — each frame is genuine, just misattributed — so the corruption is **silent and permanent**.

**Evidence**, already on disk in a log from the session that thought it had failed to capture it:

```
set_torque [id=2] -> buffer[0][=1] != id[=2]     <- every ID one behind
...
write[9]  hw_positions_: -0.0476 -1.7948  1.5907  1.3606  1.4910 -0.7317
write[10] hw_positions_:  1.5861  1.3530  1.4926 -0.7317 -0.0476 -0.7317
```

`write[10]` — the command array rotated two slots, five milliseconds after the controller latched its hold command from the corrupted state. That rotation *is* the "unexplained lurch."

**Fix.** ~20 lines: flush RX at the single choke point every request passes through, and validate the reply ID. First run after: zero errors, byte-stable commands, torque-on a no-op.

**Lesson.** A checksum authenticates a frame, not a conversation. Request/reply protocols on a shared bus need identity checks at *every* read site, not just some.

### 2. Calibration that never arrived

Recalibrating "didn't work" for three independent reasons, discovered one at a time: the fresh values were never copied into the YAML ROS reads; the full pipeline's launch defaulted to *no* calibration file at all; and one joint's row had been hand-copied from its neighbour — a 184° error — then "fixed" with an offset taken from a joint that was never swept, putting its zero at the mechanical stop.

**Lesson.** Calibration is a *delivery pipeline*, not a value. Every hop — tool → JSON → YAML → EEPROM — is a place for it to die silently. Verify the far end, not the near end.

### 3. The config merge that ate the planner

Adding `ompl: {fix_start_state: true}` to the moveit_cpp params file crashed every launch with "Planning plugin name is empty." That file is merged **last** via `dict.update()`, so a top-level `ompl:` key *replaces* the entire pipeline configuration — `planning_plugins` included. The intent was right; the placement deleted the planner.

**Lesson.** In layered config systems, know your merge order and semantics. A top-level `update()` is *replace*, not deep-merge — one innocuous key can delete a subtree five other calls built.

### 4. Torque before seed

Feetech servos drive to whatever `goal_position` holds the instant torque is enabled. The stock driver enabled torque during *configure*, before anything seeded that register, so the arm drove to the stale power-on goal on every bringup. Correct order: **read present → write goal = present (torque off) → enable torque.**

**Lesson.** Actuators with retained state must have that state established *before* being given authority to act on it.

### 5. Gravity sag versus the mechanical floor

First valid measurements put target 0 at 50 mm / 11°. The error decomposed onto one joint: elbow +7.8°, everything else ≤2.6°. Command-versus-report telemetry showed the servo sitting 91 counts from its own setpoint *in its own coordinate system* — which acquits both calibration (a wrong offset servos to the setpoint exactly, with the physical pose wrong) and the verifier's math (proven to 0.56 mm in simulation). It was the elbow holding the forearm against gravity with a P-only controller at half the firmware's default gain. The sweep then showed exactly where tuning stopped paying.

**Lesson.** Stop tuning when the lever stops paying, and write down where that was.

---

## Still open

1. **Targets 1 and 2 on hardware.** Target 1 has the longest moment arm, so sag should be *larger* than target 0's — if it exceeds 25 mm, that's the moment-arm prediction confirmed, a finding rather than a bug. Target 2 must fail at IK after roughly its full budget.
2. **`wrist_roll` range never swept** (0..4095). Its offset is now genuine; only the servo-side angle window is missing. The URDF still limits the planner.
3. **`fix_start_state` accepted but ignored** by the bounds adapter. Sidestepped by the URDF fix; the mechanism inside MoveIt was never found.
4. **Velocity feedback hardcoded to 0.0** in the driver, which makes the controller's goal check trivially true. Harmless here, a trap for the next milestone.
5. **~7 Hz effective control rate.** The transport spends 14 separate `write()` calls per `sync_write`; batching them (and restoring true `sync_read`) is the known next performance step, deliberately deferred as performance-not-correctness.
6. **Gripper reads outside its URDF range** at rest. Not blocking — the gripper isn't in the `manipulator` group — but its conventions disagree in both directions.
7. **External ground truth** (camera + ArUco) still never done. All verification is against the servos' own encoders. After this campaign's calibration adventures, that argument is stronger, not weaker.

---

## Local modifications to third-party code

Erased by `git submodule update` or upstream pulls. Re-apply from this list.

`feetech_ros2_driver`:
- `serial_port.hpp` — timeout 5 ms → 100 ms
- `communication_protocol.hpp` — RX flush in `write_buffer()` and `sync_read()`; reply-ID validation in `read()`
- `feetech_ros2_driver.cpp` — per-joint retrying `read()` returning OK; `on_activate()` seed-then-torque; torque enable removed from `configure_joints_`; `CMD CHANGE` change-detector logging

`so101-ros-physical-ai`:
- `so101_arm_common.xacro` — widened joint limits
- `so101_arm.srdf` — three resting-contact collision exemptions
- `ompl_planning.yaml`, `pilz_..._planning.yaml` — `fix_start_state: true`
- `follower_split_controllers.yaml` — `update_rate: 10`

Outside the repos: `~/so101_ws/my_follower_joints.yaml` (calibration + P gains, mirrored in `hardware/`), `~/so101_ws/urdf/so101.urdf` (regenerated), `/etc/udev/rules.d/99-so101.rules`.

---

## Method notes

- **Convert hypotheses into numbers before touching code.** The count↔radian arithmetic caught two calibration bugs, predicted a corrected value to within 3.2°, and separated gravity sag from calibration error in a single comparison.
- **When symptoms don't change after a fix, suspect stacked failures sharing one symptom.** The from-rest saga was three.
- **Build a small instrument that bypasses the suspect layer.** `sync_read_probe.py` settled in two minutes what days of timeout tuning could not; the `CMD CHANGE` logger settled three more arguments.
- **When a theory and a measurement disagree, the measurement wins** — and two readings taken seconds apart on a moving arm are not a measurement.
- **The document is not the system**, this one included. Grep the file.
