# Go2-W implementation-plan completion audit

Audit date: 2026-08-06 (Asia/Shanghai)  
Constraint: robot movement is prohibited

> 2026-08-13 addendum: later authorized work is documented in
> `reports/go2w_semantic_navigation_semantic_search_handoff_20260813.md`. In particular,
> current rotation observability is still BLOCKED: a deterministic 720-bearing
> audit finds unobservable swept-annulus space in every bearing (0.155 m on
> the lateral axes; worst sampled gap 0.290780 m). A bare
> `rotation_clearance_validation.valid=true` is now rejected unless the full
> physical evidence contract is present. This addendum supersedes any earlier
> implication that a no-return side sector alone validates rotation.

This audit maps the plan's final checklist to current evidence. `PASS` means
the stated scope is directly evidenced; `PARTIAL` does not satisfy a broader
requirement; `BLOCKED` identifies the missing physical or runtime evidence.

## Baseline and regression

| Requirement | Status | Evidence / missing proof |
|---|---|---|
| Correct main project located | PASS | `/home/brov/robot/robot_scene_demo`, expected origin recorded in deployment report |
| Pre-existing changes preserved | PASS | baseline patch and memory-file preservation recorded; unrelated dirty work remains untouched |
| Original smoke tests | PASS | six-test smoke suite in stage-1 evidence |
| Original mock flow | PASS | expected mock outputs recorded in stage-1 evidence |
| Original real image flow | BLOCKED | no external paid vision/API invocation was authorized during this safety run |
| Original video search | PARTIAL | focused video/search regressions pass; no new real-model/GPU run |
| ROS 2 workspace build | PASS | Humble packages build under system Python 3.10 |

## Sensors, time, and geometry

| Requirement | Status | Evidence / missing proof |
|---|---|---|
| Built-in camera bridge / Image / CompressedImage | PASS | explicit read-only RPC, 10 synchronized 1920x1080 triples |
| CameraInfo message contract | PASS | ten live synchronized 1920x1080 triples carry installed nonzero K/P |
| Physical camera calibration | PASS | measured 9x6/15 mm board, 105-view ROS optimization, installer validation and independent-frame reprojection sanity check |
| Camera TF | BLOCKED | official Go2-W URDF contains no camera link; no measured camera pose |
| LiDAR and LiDAR IMU input | PASS | live frequency, fields, finiteness, and stationary IMU evidence |
| Time diagnosis and bridge | PASS | 120 s fit; raw payload preservation and aligned-header checks |
| Self filtering, scan, clearance, stale timeout | PASS | 20-set live validation and 0.158 s stale transition |
| Manufacturer dimensions / LiDAR TF | PASS | pinned Unitree reference plus 0.569-degree live ground-plane check |
| Full required URDF/TF | BLOCKED | camera and `base_footprint` height remain unmeasured |

## RGB-LiDAR and localization

| Requirement | Status | Evidence / missing proof |
|---|---|---|
| RGB-LiDAR recording/extraction software | CORE_PASS | stationary recorder plus synchronized bag extractor and annotation templates; synthetic ROS bag round-trip passed |
| RGB-LiDAR PnP/overlay software | CORE_PASS | distortion-aware projection, PnP, overlay, metrics; 60-pair synthetic CLI round-trip passed |
| RGB-LiDAR physical extrinsics | BLOCKED | requires calibrated CameraInfo, real correspondences, five overlay scenes and moved-position recheck |
| 3D fusion core | PASS | timestamp, CameraInfo, mask-point, outlier and clustering gates have synthetic tests |
| Live 3D target fusion | SOFTWARE_GATE_PASS / PHYSICAL_BLOCKED | full ROS node exists; after CameraInfo passed, loopback runtime correctly moved the blocker to unconfirmed RGB-LiDAR extrinsics and fabricated no 3D pose |
| Stationary LIO | PASS | official Point-LIO five-minute stationary evidence |
| Motion-validated LIO | BLOCKED | straight/rectangle/rotation tests require robot movement |
| `odom -> base_link` single publisher | PARTIAL | stationary Point-LIO endpoint validated; motion/restart topology not validated |
| SLAM / `map -> odom` | BLOCKED | mapping requires movement |

## Live application

| Requirement | Status | Evidence / missing proof |
|---|---|---|
| Atomic Frame Bundle | PASS | READY-last atomic writer and reader validation |
| Bounded 1 Hz latest-frame spool | PASS | 45 s live evidence: 30 retained Bundles, 7.8 MiB, 1.015--1.045 s gaps |
| Ten-minute Level-A transport soak | PASS | corrected isolated-spool rerun passed 603.24 s, 489 Bundles/0.816 Hz, 0.354 s final age, bounded disk/RSS and complete cleanup |
| Conda / ROS Worker isolation | PASS | Conda 3.11 application and system 3.10 ROS workers remain separated |
| Real-time target search | PARTIAL | real Bundle entry and output contract work; calibration gate stops before a defensible live target result |
| Evidence gate / no LLM confirmation | PASS | focused tests and live blocked-session artifacts |
| Observation memory / scene graph | PASS | existing pipeline reused and focused tests pass |
| Short-step state machine | PARTIAL | software state gate passes; MOVE/STOP cycle prohibited |

## Control and navigation

| Requirement | Status | Evidence / missing proof |
|---|---|---|
| Control arbiter | PASS (core) | unsafe/nonzero input selects zero under blocked state |
| `/cmd_vel` leased bridge | BLOCKED for execution | core exists; runtime defaults disabled and creates no Action client |
| 300 ms watchdog | PASS (core) | configured and tested without robot execution |
| Remote takeover | BLOCKED for physical validation | fail-safe state modeled; no takeover motion trial in this session |
| Velocity Smoother / Collision Monitor | PARTIAL | conservative configs and launch wiring exist; live execution validation prohibited |
| Physical footprint | PARTIAL | manufacturer standing envelope used; posture/tire physical validation missing |
| Nav2 health and plan-only | BLOCKED | Level D, map, full TF, planner/runtime freshness remain closed |
| Nav2 execute | CORRECTLY NOT TESTED | explicit no-motion constraint and execution gates block it |
| Cancel, emergency stop, lease/sensor STOP | PARTIAL | existing leased-control project evidence exists; integrated autonomous chain cannot be exercised without movement authority |

## Safety and delivery

| Requirement | Status | Evidence / missing proof |
|---|---|---|
| Final robot stationary | PASS | only read-only sensor/RPC sessions were started |
| No abnormal owned processes | PASS after cleanup fix | orphaned read-only ROS nodes from the failed soak were identified and terminated; launcher now escalates owned groups after grace windows |
| No low-level control / firmware changes | PASS | no low command, joint, posture, firmware, or safety-disable operation used |
| README / deployment guide / final report | PASS | README, this audit, deployment guide and deployment report exist |
| ROS regression result | PASS | 10 packages build; 54 tests pass with zero errors/failures/skips |

## Current highest defensible level

The deployment remains **Level 0 plus validated stationary sensor/LIO software
subsystems**. Measured CameraInfo and the corrected ten-minute transport soak
now pass; Level A remains blocked by camera TF. Levels C--F are additionally blocked
by physical RGB-LiDAR extrinsics, motion-validated
LIO, a map, complete TF, and movement safety acceptance.

The shortest next physical step is to measure the stationary built-in camera
pose and collect real RGB-LiDAR structural correspondences. No movement mode
may be enabled from this audit.
