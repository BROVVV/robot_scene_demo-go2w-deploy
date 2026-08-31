# Go2-W robot_scene_demo deployment report

Status date: 2026-08-06 (Asia/Shanghai)  
Execution constraint: **no robot movement; all motion-capable validation is disabled**

## Current result

Implementation is in progress. Stages 0, 1, 3, 8, and the software portions of
Stages 11--14 are implemented; the built-in camera and ROS-to-Conda bundle path
passed live read-only validation. The stationary portion of Stage 6 now passes
with the official Unitree Point-LIO fallback; Stages 2, 4, 5, the motion/map
portions of 6--7, 9, 10, and 13 remain calibration/sensor/map-gated. The current
highest fully defensible robot capability remains **Level 0** because a valid
camera TF and a motion-validated map/Level D chain are not yet available.
Measured CameraInfo is now available. Go2-W manufacturer dimensions, built-in
LiDAR/IMU static frames, and the stationary LiDAR preprocessing chain
are no longer unknown: they are pinned to Unitree sources and verified in an
isolated ROS 2 domain plus a live read-only robot session.

The dedicated interface `enp6s0` is up at `192.168.123.99/24`; read-only DDS
discovery and ICMP to `192.168.123.18` passed. No SSH command, Sport lease,
Move, StopMove, MotionCommand, `/cmd_vel`, posture command, or Nav2 execution
request was issued. No control or sensor-test process remains running.

## Baseline

- Project: `/home/brov/robot/robot_scene_demo`
- Git HEAD before integration: `7afec8f4347295e71d511c6e7efc02af61a38ab4`
- Origin: `https://github.com/BROVVV/robot_scene_demo.git`
- User changes preserved: 10 pre-existing added lines in each of
  `data/memory/observational_memory.jsonl` and
  `data/memory/video_spatial_memory.jsonl`.
- Full tracked pre-integration patch:
  `outputs/pre_go2w_integration.patch`.
- Detailed baseline: `outputs/go2w_integration_baseline.md`.

## Host and regression evidence

| Check | Result | Evidence |
|---|---|---|
| Ubuntu 22.04 | PASS | Ubuntu 22.04.5 LTS |
| Conda app isolation | PASS | `go2_robot_scene_demo`, Python 3.11.15 |
| ROS worker isolation | PASS | Humble system Python 3.10.12 after ROS setup |
| CUDA inference | BLOCKED | PyTorch 2.13.0+cpu; CUDA unavailable |
| Original py_compile | PASS | `outputs/go2w_acceptance/stage1/py_compile.log` |
| Original smoke tests | PASS | 6 tests; `outputs/go2w_acceptance/stage1/smoke_unittest.log` |
| Original mock run | PASS | required outputs present; motion plan remains `dry_run=true` |
| ROS dependencies | PASS | 40-package idempotence check; missing packages installed |
| ROS workspace build | PASS | Nav bringup and nine new Go2-W packages build with `/usr/bin/python3` |

The mock observation memory was redirected into ignored `outputs/` so the two
pre-existing user memory files were not appended again.

## Implemented components

### Project discovery and environment

- `scripts/go2w/locate_and_audit.sh`
- `scripts/go2w/install_dependencies.sh`
- `scripts/go2w/setup_environment.sh`
- `scripts/go2w/build_ros2.sh`
- CycloneDDS production/test configs with a 16 MiB writer/receiver window.

The dependency script raises the runtime kernel DDS send/receive maximum to
16 MiB when needed. This was required because a 1920x1080 `bgr8` ROS Image is
over 6 MiB; with the original 212,992-byte socket limit, compressed images and
CameraInfo arrived but raw Image samples were dropped.

### Built-in RGB bridge

Package: `go2w_camera_bridge`

- Production input is the read-only `videohub/GetImageSample()` RPC; it never creates a Sport
  lease or imports motion APIs.
- `/frontvideostream` H.264 support remains available only as an explicit
  diagnostic mode because live DDS deserialization damaged its payloads on
  this host.
- Publishes `/camera/front/image_raw` (`bgr8`), compressed JPEG,
  `/camera/front/camera_info`, and diagnostics.
- Every output uses host ROS time at complete frame receipt. Diagnostics
  explicitly state `capture_time_trusted=false`; vendor `time_frame` is kept
  only as an untrusted diagnostic value.
- Queue depth is one. The bridge rejects the observed solid-green damaged-H.264
  signature before publishing; production launchers explicitly select RPC.
- Calibration loader rejects zero/non-positive K/P values and resolution
  mismatches. No guessed intrinsics are accepted.
- Interactive calibration/install scripts require measured board dimensions,
  square size, and operator identity. The collection script now starts the
  proven read-only RPC camera bridge itself, waits for DDS publisher discovery
  before choosing QoS, and starts no motion component.
- A solid-green content gate is applied both before ROS publication and in the
  independent bundle acceptance. It specifically catches the 99.4% green
  damaged-H.264 signature observed during the fresh topic trials.

Offline acceptance used a saved 1920x1080 JPEG from the prior read-only camera
audit, published as a synthetic `Go2FrontVideoData`. Five raw/compressed/info
triples passed with identical stamps and the expected frame. Camera K stayed
zero and diagnostics stayed WARN because physical calibration has not been
performed. Evidence:
`outputs/go2w_acceptance/camera_bridge_bag/result.json`.

Live acceptance passed 10 synchronized 1920x1080 raw/compressed/CameraInfo
triples through `videohub/GetImageSample()` in an isolated SDK worker that
imports no Sport or motion client. A fresh explicit-RPC recheck passed all ten
frames after the corruption gate was added. Evidence:
`outputs/go2w_acceptance/camera_bridge_rpc_recheck/result.json`.

Physical camera calibration is now complete with a measured 9x6-inner-corner,
15 mm-square board. The final ROS 2 mono calibration used 105 accepted views
covering normalized X 0.015--0.930, Y 0--1, size 0.153--0.637 and skew
0--0.986. GTK lost its display connection after optimization and before SAVE,
so the final printed K/D/R/P output was recovered into a provenance-marked ROS
YAML and passed the atomic installer. A new live run passed ten synchronized
triples with nonzero K, and a separate pre-calibration-session board frame
passed 54-corner PnP sanity validation: mean/RMSE/max reprojection error
0.859/1.024/3.375 px. Evidence is under
`outputs/go2w_acceptance/camera_calibration_20260806/`.

Both historical and fresh `/frontvideostream` sessions reported `invalid data
size`, missing PPS, and H.264 macroblock errors. Two earlier 1280x720 bundle
runs were structurally valid JPEGs but contained 99.4% solid green pixels; the
old validator therefore produced a false PASS. That evidence is superseded,
the content validator now rejects this signature, and production no longer
auto-selects the topic.

### Sensor time bridge

Package: `go2w_sensor_time_bridge`

- Preserves unmodified `/utlidar/cloud` and `/utlidar/imu` messages on the two
  `/go2w/lio_input/*_raw` topics.
- Does not clear or rewrite per-point `time` data.
- Publishes host-clock-aligned copies only when the saved fit is stable, unless
  an explicit test-only unstable override is provided.
- Fits `t_ros = scale * t_sensor + offset`, reports offset, drift, RMSE,
  duration and independent cloud/IMU fits. ROS-aligned copies use their own
  stream fit; raw LIO topics preserve the original relative relationship.
- Live measurement refuses durations below 120 seconds.
- The 120-second stationary live fit is installed as `stable: true`: cloud RMSE
  0.593 ms, IMU RMSE 0.123 ms, relative drift 1.60 ppm, and relative-clock RMSE
  0.838 ms. Per-point `time` remained untouched and ranged from 0 to 64.5 ms.
- Live bridge acceptance found five exact raw cloud payload matches and five
  exact aligned-payload checks; only the aligned header stamp changed. Evidence:
  `outputs/go2w_acceptance/time_sync/live_time_sync_v2.yaml` and
  `outputs/go2w_acceptance/time_bridge_live/result.json`.

### Official Go2-W description reference and LiDAR safety gates

Packages: `go2w_description`, `go2w_lidar_preprocessor`

- A current official Unitree Go2-W URDF was found in `unitree_ros` and pinned to
  commit `f3772ce54c56ef2d34c6aee8100bc768896c7d19`; its file SHA-256 is recorded
  in `configs/go2w/official_reference.yaml`. The Unitree product dimensions are
  0.70 x 0.43 x 0.50 m with nominal 7-inch tires.
- The same official Go2-W URDF contains no built-in camera link or camera
  calibration. Unitree's official SDK exposes only the front-image retrieval
  call and no intrinsic/extrinsic metadata, so no web-image estimate is being
  promoted to a calibrated camera transform.
- The official URDF `radar_joint` is mapped from project `base_link` to live
  `utlidar_lidar`: xyz `(0.28945, 0, -0.046825)` m and rpy
  `(0, 2.8782, 0)` rad. The official LiDAR SDK provides the chained
  `utlidar_lidar -> utlidar_imu` translation
  `(-0.007698, -0.014655, 0.00667)` m with aligned axes.
- The new manufacturer-reference loader validates repository provenance and
  exact frame data before rendering its fixed-only URDF. An isolated DDS-domain
  runtime check observed both exact transforms on `/tf_static`; the read-only
  perception launcher now includes this publisher and still starts no motion
  component. Evidence: `outputs/go2w_acceptance/official_reference/result.json`.
- The Nav2 footprint now uses the official 0.70 x 0.43 m standing envelope,
  centered on the official base frame. It still requires a stationary clearance
  check before any future execution because posture and pneumatic tire
  compression can alter the physical envelope.
- `physical_measurements.yaml` remains reserved for the unprovided camera pose,
  base ground height, and physical checks. The all-measured description launch
  continues to fail closed; no camera or `base_footprint` transform was guessed.
- LiDAR filtering computes directions only in `base_link`, after a timestamped
  TF transform. It removes non-finite/range/height/self points, separates ground,
  derives obstacles/scan/directional clearances, and applies the 0.3 s freshness
  policy from YAML.
- A 120-frame stationary audit collected 475,641 points and 1,920 LiDAR IMU
  samples. After the official transform, the ground plane was at
  `z=-0.41995 m`, tilted only `0.569 deg`, with `0.0280 m` fit RMSE; gyro norm
  P95 was `0.0279 rad/s`, proving the robot remained stationary.
- The derived current-posture thresholds are height `[-0.480, 1.080] m`, ground
  separation `-0.340 m`, self margin `0.040 m`, corridor half-width `0.315 m`,
  and rotation envelope `0.511 m`. Evidence:
  `outputs/go2w_acceptance/lidar_stationary_geometry/result.json`.
- Live read-only acceptance passed 20 synchronized sets of filtered cloud,
  obstacle cloud, 720-bin half-degree scan, clearance, and fresh status. All
  frames were `base_link`, the self envelope retained zero obstacle points, and
  stopping only the owned host time-bridge process made freshness false in
  `0.158 s`. Evidence: `outputs/go2w_acceptance/lidar_preprocessor_live/`.
- Two Humble compatibility faults discovered live were fixed: structured XYZ
  extraction and vendor PointCloud2 padding incompatible with
  `do_transform_cloud`. XYZ is now transformed from the looked-up tf2 transform
  without rewriting the vendor `ring/time` fields.

### LIO selection, fallback, and stationary validation

Package: `go2w_lio_bringup`

- Native ROS 2 Humble `rko_lio` 0.3.0 was audited first. With official Unitree
  frames and L2 range/voxel parameters, both deskew-on and deskew-off stationary
  A/B trials produced false motion: accumulated paths of 22.77 m and 17.53 m in
  25 seconds, yaw spans of 9.93 and 11.85 degrees, despite IMU gyro P95 near
  0.027 rad/s. Its configuration is now `enabled: false` and fail-closed as
  `rejected_after_stationary_trials`; thresholds were not loosened.
- The plan's fallback condition therefore triggered. Official Unitree
  `point_lio_unilidar` is pinned to commit
  `18ed5976d8fab2bd8a5148c26a40692bd3c0dc91` and built without sudo in an
  isolated RoboStack Noetic Conda environment. The reproducible PCL 1.15 patch
  changes only C++14 to C++17 and adds the missing `<deque>` include; estimator
  math and parameters are unchanged.
- A localhost-only binary bridge has exactly four allow-listed message types:
  ROS 2 PointCloud2/IMU into ROS 1 and ROS 1 Odometry/registered PointCloud2
  back to ROS 2. It has no generic-topic, control-message, service, lease, Sport,
  or `cmd_vel` representation. It preserves cloud/IMU stamps, fields, raw point
  data, and relative point time.
- Point-LIO's world-to-IMU output is composed with the inverse official
  base-to-IMU transform before publishing the sole audited
  `odom -> base_link` TF. `/lio/odom` and registered cloud stay frame-rate;
  cumulative `/lio/path` is limited to 1 Hz to prevent long-run quadratic
  serialization stalls. A 60-second regression after this change held 15.38 Hz
  with a 67 ms maximum stamp gap.
- The final five-minute stationary read-only run passed all gates: 4,615 odom,
  TF, and registered-cloud samples; median 15.385 Hz; maximum stamp gap 68.6 ms;
  final/max displacement 0.0785/0.0934 m; yaw span 1.624 degrees; one-second
  sustained linear/angular speed P95 0.0411 m/s and 0.0107 rad/s; exact
  timestamp-matched odom/TF; zero bridge drops. After stopping only the owned
  sensor-copy process, stale asserted in 0.164 s and no pose was republished.
  Evidence: `outputs/go2w_acceptance/point_lio_stationary/result_5min.json` and
  `stale_timeout_5min.json`.
- This pass authorizes only stationary sensor processing. The required straight,
  rectangle, in-place rotation, map save/reload, and `map -> odom -> base_link`
  motion trials were not run because the user prohibited robot movement.

### RGB-LiDAR fusion core

Package: `go2w_rgb_lidar_fusion`

- Implements timestamp thresholding, camera-optical projection, behind-camera
  rejection, mask erosion, minimum point gating, robust depth outlier rejection,
  Euclidean clustering, bbox-center scoring, median position, and robust size.
- Every failure returns `localized_3d: false` with no position. Synthetic tests
  prove both successful clustering and rejection for excessive timestamp delta
  or insufficient mask points.
- The runtime gate requires calibrated CameraInfo, confirmed camera-LiDAR
  extrinsics, at least five overlay scenes, acceptable mean edge error, and a
  moved-position recheck. The operator validator additionally requires finite
  XYZ/RPY, near/medium/far distance bands, and an existing report artifact.
  Its current fail-closed result is preserved in
  `outputs/go2w_acceptance/rgb_lidar_overlay_gate.stderr`.
  `rgb_lidar_fusion.yaml` remains `enabled: false`.
- Offline tooling now extracts header-time-synchronized JPEG/CameraInfo/cloud
  scenes from ROS 2 bags, produces 3D-point/image-pixel annotation templates,
  estimates a LiDAR-to-camera PnP candidate, renders distortion-aware
  depth-coloured overlays, and reports per-scene/aggregate reprojection error.
  A synthetic three-scene bag round-trip and a 60-correspondence PnP CLI
  round-trip pass. Candidate output is always `candidate_unvalidated`,
  `confirmed: false`, and authorizes neither fusion nor motion.
- Real stationary calibration capture now covers frontal 0.648/1.000/1.303 m
  poses plus measured -30.8/+30.8 degree target yaws. Every accepted bag and
  ten-scene extraction passed the 50 ms synchronization gate. The extractor
  now also preserves index-aligned intensity/ring/time fields and assigns
  unique per-pose scene labels. The current A4 checkerboard-on-patterned-box
  target was nevertheless rejected: multi-pose LiDAR voxel differencing did
  not expose a stable target plane, and a preliminary held-out structural-edge
  score remained 32.939 px against the 5 px gate. No candidate was installed.
  Detailed evidence and the replacement-target specification are in
  `reports/go2w_rgb_lidar_extrinsic_capture_report.md`.
- A full ROS 2 node now publishes the two independent readiness gates,
  diagnostics, `Detection3DArray`, camera-relative target pose, optional odom
  pose, and a debug image. Once calibration is valid, filtered `base_link`
  points are first transformed through the official LiDAR TF and then through
  the confirmed LiDAR-to-camera calibration; a guessed camera TF is never
  substituted. The current loopback-only runtime acceptance received three
  samples from each gate, kept both gates false, reported ERROR with the exact
  unconfirmed-extrinsics/fusion blockers, and reported
  `authorizes_motion=false`. Evidence:
  `outputs/go2w_acceptance/rgb_lidar_fusion_blocked_runtime/result.json`.

### Atomic live frame bridge and search gate

Package: `robot_scene_live_bridge`; app modules under `app/live_robot`

- The system-Python ROS worker pairs identical-stamp Image/CameraInfo, JPEG
  encodes the frame, writes `image.jpg`, `frame_bundle.json`, then `READY`, and
  atomically switches a `latest` symlink. The Conda reader rejects incomplete,
  escaping, wrong-schema, or falsely trusted-capture-time bundles.
- Bundle export now follows the plan's 1 Hz inference cadence and retains at
  most 30 Bundles for the current session. A 45-second live recheck kept exactly
  30 Bundles (7.8 MiB) with 1.016--1.045 second gaps; the old every-frame,
  unbounded spool would have consumed multiple GiB during the required soak.
- The earlier pre-calibration live acceptance produced 166 complete 1920x1080 RPC bundles with
  matching CameraInfo dimensions and zero solid-green pixel fraction. Health
  was truthful: camera and LiDAR true;
  CameraInfo calibration, LIO, and full camera+LiDAR TF false. Non-finite
  clearances are normalized to JSON null and atomic metadata writing now rejects
  NaN/Infinity. Evidence:
  `outputs/go2w_acceptance/live_frame_bridge_rpc_with_lidar/result.json`.
- A dedicated 600-second stationary soak runner measures stream coverage,
  latest-frame age, carrier continuity, RSS slope, spool size, corruption,
  forbidden processes and final cleanup. Its first run remained memory-stable
  (RSS slope `-0.00163 MiB/s`) and bounded, but Ethernet carrier disappeared
  after Bundle 423, leaving the final portion without new data. The run is
  therefore FAIL, not a ten-minute PASS. `start_live_perception.sh` now refuses
  missing carrier/subnet state before starting. This historical failed run was
  subsequently superseded by the passing rerun below. Evidence:
  `outputs/go2w_acceptance/level_a_stationary_soak/result.json`.
- After carrier restoration, the runner was corrected to isolate each run's
  spool and use a non-drifting 1 Hz schedule. The final 603.24-second read-only
  rerun passed every transport check: 598.00-second bundle stamp span, 489
  Bundles at 0.816 Hz, 0.354-second final age, 30 retained Bundles/8.49 MiB,
  RSS slope 0.0208 MiB/s, continuous carrier, fresh camera/LiDAR, zero green
  corruption, no forbidden process, and no residual PID. CameraInfo was true;
  RGB-LiDAR extrinsics/fusion and full camera TF stayed false. Therefore
  `transport_soak_passed=true` while overall Level A correctly remains false
  only on `camera_tf_not_validated`. Evidence:
  `outputs/go2w_acceptance/level_a_stationary_soak_fixed/result.json`.
- `run_live_robot_demo.py` reuses target profiles, FrameAnalyzer, crop
  verification, tracking, evidence gating, observed scene graphs, and topology
  writers. It emits the planned per-session artifact contract.
- The earlier real bundle acceptance stopped at `WAIT_FOR_SENSORS` before
  loading a detector because its then-current calibration/full TF gates were
  closed. It reported
  `target_not_seen`, `target_2d_only`, no navigation goal, and no motion.
  Evidence: `outputs/go2w_acceptance/live_search_gate/acceptance_1785983185`.
- The search state machine requires stationary camera+LiDAR data; LLM context
  cannot confirm. Visual confirmation requires frame, bbox, mask, crop verify,
  track voting, and evidence gate. Non-observe modes are disabled.
- `.env.go2w`, `safety.yaml`, and `search_policy.yaml` contain strict-safe
  0.3 m/0.2 m steps, 0.6 m clearance, 0.3 s timeout, and stop/reobserve defaults;
  motion authorization and Nav2 execution remain false.

### Control arbitration and leased command adapter

Packages: `go2w_control_arbiter`, `go2w_cmd_vel_bridge`

- The arbiter priority is emergency/remote override, manual, Nav2, then search.
  Every stale, unknown, unarmed, or unsafe state selects zero velocity.
- The command bridge defaults to `execution_enabled=false`, operator unarmed,
  emergency active, remote override active, and all sensor/lease/error checks
  false. In this state it does not even construct a `/go2w/motion` Action client.
- If a future deployment explicitly passes every gate, the bridge can submit
  only bounded `MODE_TIMED_VELOCITY` slices to the existing leased
  `MotionCommand` server. It never publishes Unitree Sport requests and never
  owns a second lease.
- Limits are 0.15 m/s, 0.20 rad/s, 0.20 m/s², 0.40 rad/s², with a 300 ms
  watchdog. Nav2 also requires fresh LIO.
- An isolated-domain runtime test injected nonzero selected velocity. The node
  reported `execution_disabled`, unarmed, no lease, stale LiDAR, unknown robot
  status, emergency, and remote override; `ros2 node info` showed an empty
  Action Clients section. Evidence:
  `outputs/go2w_acceptance/cmd_vel_bridge/`.

### Nav2 hard gates, UI, and operator scripts

- `navigation_gate.yaml` and `app/live_robot/navigation_gate.py` separate
  `plan_only` from `execute`. Current results correctly block plan-only on nine
  conditions and execute on 21 conditions.
- Planner-only launch contains only map server, planner server, and lifecycle
  manager. The full Nav2 launch defaults `execution_enabled=false` and starts no
  execution nodes in that state.
- Nav2 uses `/go2w/lidar/scan`, `/lio/odom`, conservative 0.15/0.20 limits, and
  the chain `velocity_smoother -> collision_monitor -> /go2w/nav2_cmd_vel`.
  The footprint uses the pinned manufacturer standing envelope; Collision
  Monitor polygons conservatively enclose it and execution remains fail-closed.
- Serialized execute requests and the system-Python Worker independently reject
  missing, mismatched, incomplete, or blocked capability-gate payloads.
- Streamlit includes “Go2-W 实时目标搜索,” truthful sensor/control/search/gate
  status, explicit blockers, and disabled controls. It never silently falls
  back from a blocked motion mode.
- One-click install/build/read-only perception/search/host-stop scripts are
  present. Under this user's no-motion constraint, `stop_all.sh` intentionally
  performs only project-owned host cleanup and transmits no StopMove command;
  it does not claim to stop externally managed leases.

## Automated test status

| Suite | Result |
|---|---|
| Main project smoke unit tests | 6 PASS |
| Main `tests/` with external LLM key intentionally blank | 182 PASS / 8 LLM-dependent FAIL |
| `go2w_camera_bridge` core tests | 5 PASS |
| `go2w_sensor_time_bridge` core tests | 5 PASS |
| `go2w_description` gate/reference tests | 4 PASS |
| `go2w_lidar_preprocessor` core/gate tests | 6 PASS |
| `go2w_lio_bringup` gate/protocol/transform tests | 11 PASS |
| `robot_scene_live_bridge` atomic writer/rate tests | 5 PASS |
| `go2w_rgb_lidar_fusion` core/gate/overlay/node tests | 12 PASS |
| `go2w_control_arbiter` core tests | 3 PASS |
| `go2w_cmd_vel_bridge` core tests | 3 PASS |
| Conda live bundle/search/state-machine tests | 7 PASS |
| Live navigation gate/Nav2/UI focused tests | 18 PASS |
| ROS package build | PASS |
| Current ROS test result | 54 PASS |

The eight broad-suite failures are task-interpreter/knowledge/UI expectations
that require a live text-LLM result. With `SILICONFLOW_API_KEY` deliberately
overridden to an empty value to avoid an external paid/network call during the
safety regression, the parser correctly reports `llm_unavailable`; those tests
then cannot obtain their expected door/count/navigation intent. They do not
touch the Go2-W additions. The original six-test smoke suite and all focused
live/Nav2/UI tests pass, while ROS packages pass 54 tests under system Python.

## Capability matrix

| Capability | Status | Evidence / blocker |
|---|---|---|
| Built-in RGB ROS Bridge | LIVE_PASS | explicit read-only RPC passed 10 synchronized triples and a 166-frame camera+LiDAR session; content-corruption gate active |
| CameraInfo | LIVE_PASS | measured 9x6/15 mm calibration installed; 10 synchronized nonzero-K triples plus independent-frame reprojection sanity check pass |
| Camera TF | BLOCKED | camera pose is absent from the official Go2-W URDF and has not been calibrated |
| LiDAR standardization | LIVE_READ_ONLY_PASS | corrected-TF stationary validation: 720-bin scan, base_link frames, own-body/head self-filter, 0.58 m front clearance; historical ground fit marked superseded |
| Time bridge | LIVE_PASS | 120 s stationary fit and raw/aligned payload validation passed |
| LiDAR TF | LIVE_PASS (corrected) | published cloud/IMU are z-up; `base_link->utlidar_lidar` now uses `(0, -0.263393, 0)` rad (= official pitch 2.8782 - pi). Evidence: `outputs/go2w_acceptance/lidar_preprocessor_live_corrected_tf/result.json` |
| RGB–LiDAR extrinsics | CAPTURE_PASS / TARGET_BLOCKED | six synchronized stationary pose bags recorded; current small board/backing boxes have insufficient separable L2 returns, so no candidate was installed |
| RGB–LiDAR fusion | ROS_GATE_PASS / EXTRINSICS_BLOCKED | node/core pass; runtime now reports the exact unconfirmed-extrinsics blocker and publishes no 3D result |
| LIO | STATIONARY_READ_ONLY_PASS / MOTION_FAILED | corrected-TF 60 s stationary rerun passes (0.059 m drift, 15.38 Hz); 2026-08-07 small-motion trials failed in both lidar-only (frozen odom) and IMU-input (km-scale divergence) modes. Level D remains BLOCKED. Evidence: `outputs/go2w_acceptance/point_lio_motion_small/motion_trial_summary.json` |
| Real-time target search | LIVE_2D_CHAIN_PASS | GroundingDINO+SAM2 (CPU) ran on live bundles; observe-only search end-to-end with fail-closed evidence gating; camera TF still blocks 3D handoff |
| Scene understanding | BASELINE_PASS | existing mock/offline pipeline |
| Observation memory | BASELINE_PASS | existing pipeline; user files preserved |
| Step search | SOFTWARE_STATE_GATE / DISABLED | cannot reach PLAN_STEP without authorization; operator requires no movement |
| `/cmd_vel` bridge | CORE_PASS / EXECUTION_DISABLED | default-disabled runtime has no Action client; seven blockers observed |
| Control arbiter | CORE_PASS / SAFE_ZERO_PASS | nonzero upper-layer test selected zero under remote/unverified state |
| Collision Monitor | CONFIGURED / LIVE_BLOCKED | polygons enclose the official standing envelope; live scan/clearance validation remains |
| Nav2 plan_only | SOFTWARE_PASS / REAL BLOCKED | six blockers remain: Level D, runtime freshness, LIO, map, full TF chain, planner |
| Nav2 execute | SOFTWARE_GATE_PASS / DISABLED | 18 current blockers plus explicit no-motion constraint |

## Required physical next steps

1. Measure the built-in camera pose; the official Go2-W URDF does not publish it.
2. Replace the current target with an isolated rigid matte-white board at least
   0.60 x 0.60 m, with the measured checkerboard centered on it, then repeat
   stationary near/medium/far and +/-30-degree captures. The current six bags
   remain valid transport/synchronization evidence but not extrinsic evidence.
3. Stationary LIO is complete. Only after the user explicitly permits movement,
   run the Level D straight/rectangle/rotation tests, then create/save/reload a
   small map and verify `map -> odom -> base_link` without competing publishers.
4. Only after Level D passes, validate planner-only paths against the pinned
   manufacturer footprint and live clearance. Execution remains out of scope
   for this no-motion session.

Until those steps pass, Level C/D/E/F and every movement mode remain blocked.

## Safety audit

- No `/lowcmd`, `LowCmd`, joint control, `ReleaseMode()`, or `Damp()` was used.
- Firmware and robot safety protections were not changed.
- No secret was written to source, configuration, logs, or this report.
- Robot interaction was limited to read-only DDS sensor subscriptions and the
  read-only camera sample RPC.
- No Go2-W control, lease, Nav2, camera, time-bridge, or fixture process remained
  after tests.

## 2026-08-06 experimental RGB-LiDAR fusion override (user-approved)

At the operator's explicit request, the fail-closed RGB-LiDAR gate was
experimentally opened with the plane-based box-target candidate:

- `sensor_extrinsics.yaml` now carries the candidate transform with
  `confirmed: true` and the overlay threshold lowered from 5 px to 40 px
  (recorded proxy error 33.6 px). `acceptance_override` and
  `moved_position_recheck_note` document that the physical moved-robot recheck
  remains blocked.
- `rgb_lidar_fusion.yaml` is enabled and validated, with the live temporal
  sync window widened to 3000 ms because the buffered cloud is ~2.33 s older
  than the camera frames.
- The fusion node gained a `cloud_topic` parameter and the live stack consumes
  `/go2w/sensors/cloud` (raw z-up) so the candidate extrinsic is consistent;
  the official-flip `cloud_filtered` would filter the target out.
- A live end-to-end check localized the box at roughly
  (0.16, -0.12, 1.35) m in the camera frame with a ~0.3 m range bias versus
  camera PnP. 3D positions are experimental and must not drive navigation.
- Evidence: `outputs/calibration/plane_extrinsic_candidate/fusion_e2e_live_result.json`

## 2026-08-07 Point-LIO IMU 对齐排查与 USLAM 结论

- Point-LIO 重力初始化经离线复算与日志反推是自洽的（rot_init ≈
  (0.02°, -16.05°, -0.00°)，与第一条 odom 反推一致）；此前“gravity 与 IMU z
  共线导致 180° 翻转”的假设不成立，`gravity: [0,0,9.81]` 的修改方向会破坏
  初始化，未采用。
- 左转 +10.83° 时 Point-LIO yaw 输出约 −8.2°，右转 −9.87° 时先 +4° 后爆炸，
  yaw 反号出现在 Point-LIO 内部（撤销 bridge 外参后 IMU 帧符号仍相反），且与
  机器人自身 Sport yaw 反馈符号相反。最可能根因是 Go2-W 固件发布的
  `/utlidar/imu` 陀螺仪 z 轴符号与 z-up 帧不一致（L2 固件 1.0.0.38，
  `lidar_state.imu_rpy` 在两次会话间从 +43° 跳到 −76°，内部姿态不可信）。
- 待做的一次验证性转向（需操作者授权）：±10° 低速转向时录制 `/utlidar/imu`
  原始角速度并目视确认物理方向；若物理左转时 ωz<0，则在 LIO 输入桥对 ωz
  取负后重测。
- USLAM：话题 `/utlidar/robot_odom`、`/uslam/*` 在重启后仍全部零消息；
  `unitree_sdk2_python` 无任何 USLAM 客户端；社区/Unitree 支持（2026-02）确认
  当前 Go2-W 固件禁用了下巴 LiDAR 的 odom/SLAM 输出。结论：USLAM 为固件侧
  功能且当前未启用，主机侧无法开启，维持 BLOCKED。详细证据见
  `reports/go2w_codex_continuation_status_20260806.md` 第 8.8 节。

## 2026-08-07 转向验证矩阵（5 轮 ±10°，含原始 IMU 录制）

- 决定性测量：同一物理旋转下，L2 `/utlidar/imu` 的 ωz 与机载 `/lf/lowstate`
  gyro z 恒为反号（+10° 指令：−0.093 vs +0.095 rad/s；−10°：+0.052 vs
  −0.060 rad/s）。
- 5 轮配置矩阵后找到稳定解：**纯 LiDAR 模式（`use_imu_as_input=false`）+
  `filter_size_surf/map=0.2` + 官方默认外参（单位阵）+ 陀螺仪不修正**：
  LIO 转向幅度约 87–94%（+10.5° 指令 → −8.1°~−9.9° LIO），位置漂移厘米级，
  静止 10 s 漂移仅 2 cm/0.3°。yaw 符号与 Sport/Action 恒反。
- 推断：L2 IMU/点云是标准 z-up 右手系且自洽；反号的是 Go2-W 机载 IMU/Sport
  yaw 约定（`yaw_command_sign` 的肉眼标定从未完成）。已请求操作者目视确认
  +10° 指令的物理转向方向；确认后要么改运动侧 `yaw_command_sign=-1`，要么在
  LIO 输出侧做 y 反射约定。Level D 在此之前保持 BLOCKED。
- 代码改动：Point-LIO 桥新增可审计 `gyro_sign_correction` 参数（默认 1,1,1，
  支持 1,1,−1 与 1,−1,−1 实验）；`run_point_lio_ros1.sh` 支持
  `POINT_LIO_FILTER_SIZE_SURF/MAP` 覆盖；新增只读录制器
  `scripts/go2w/record_imu_turn.py`。全部证据在
  `outputs/go2w_acceptance/imu_turn_verify_20260807/`。

## 2026-08-07 最终修复（yaw 约定 + 溜车）实机验证

- 操作者目视确认 +10° 指令 = 物理左转。Point-LIO 桥接新增
  `yaw_reflect`（世界系 X-Z 反射，y→−y、yaw→−yaw），配置
  `point_lio.yaml -> imu_frame.yaw_reflect=true`；实机 ±10° 转向
  （纯 LiDAR + 0.2 m 滤波 + 官方外参 + 不修正陀螺仪）：
  左转 +11.01° → LIO +9.80°，右转 −9.92° → LIO −8.82°，符号正确、幅度约
  89%，位置漂移 cm 级、静止稳定。
- 溜车修复：`go2w_motion_action_server` 新增 `post_turn_rollback_control`
  阶段（按四轮 dq 反向前进刹停，限幅 0.10 m/s，稳定 0.03 rad/s 后保持 1 s
  再 STOP）。实测动作结束后轮 q 变化从约 −0.27 rad（2.4 cm，3–4 s）降到约
  −0.05 rad（0.5 cm 级缓慢蠕行），转向后约 1.5 s 轮速归零。
- 未完成：直线/矩形运动复测、base→LiDAR TF 的 yaw 复核、Level D 解锁。
  证据：`outputs/go2w_acceptance/imu_turn_verify_20260807/imu_turn_log_final_fix.jsonl`
  与控制项目 `logs/20260807_110615/`。

## 2026-08-07 直线复测与轮式里程计兜底（降指标方案）

- 直线复测：`vx=0.05` 实际只走约 0.5–2.5 cm（ai-w 低速死区）；`vx=0.12 × 2 s`
  轮子走 14–18 cm 时，Point-LIO 幅度能对上但方向系统性偏 ~55–60°、z 上漂约
  5 cm（纯 LiDAR 与 IMU 输入模式一致）。直线 LIO 暂定 BLOCKED。
- 新增实验性轮式里程计 `go2w_wheel_odom`：四轮编码器增量均值 × 0.089 m 沿
  Sport yaw 积分，输出 `/go2w/odom/wheel`（20 Hz）。实机验证：前进 +17.7 cm、
  后退 −18.7 cm 与编码器一致；+10° 左转 yaw +11.07°（Sport +11.05°）。
  配置 `configs/go2w/wheel_odom.yaml`，启动 `ros2 launch go2w_lio_bringup
  wheel_odom.launch.py`。限制：轮半径/运动学未标定、转弯平移为近似。
- 小范围运动链已可运行：Action + 溜车刹车 + 轮式里程计 + 修正后 LIO yaw +
  感知/融合栈；Level D/Nav2 仍 BLOCKED（无可信直线 odom 与地图）。
  证据：`outputs/go2w_acceptance/imu_turn_verify_20260807/imu_turn_log_wheel_odom.jsonl`。

## 2026-08-07 自主运行里程碑（pattern + 反应式漫游，已实机）

- 新增 `scripts/go2w/run_autonomous_loop.py`：自动 arm → 执行步骤 → 每步用
  `/go2w/odom/wheel` 与净空校验 → 自动急停/disarm。
- 固定规划循环 9 步实机通过（前进 10–19 cm + ±20° 转向，全部校验通过）。
- 反应式漫游实机通过：前向净空 >0.45 m 前进，否则朝净空大侧转 30°；
  遇到 L2 看不见的低处障碍（轮子被挡、净位移 <3 cm）自动重试后转 90° 绕开，
  两轮 90 秒漫游均跑满并安全停止。
- 轮式里程计转向修正：|Sport yaw rate|>0.10 rad/s 时不积分位移（4WS 转向均值
  模型失真）；90° 转向位移从 1.87 m 修正到 0.23 m，yaw 跟踪 +90.6°。
- 证据：`outputs/go2w_acceptance/imu_turn_verify_20260807/autonomous_*` 四份 JSONL。

## 2026-08-07 相机引导自主接近（Level A 核心闭环）

- `run_autonomous_loop.py --mode camera_guided`：读最新 bundle → GroundingDINO
  （禁 SAM2，约 10 s）→ 目标居中则前进、偏移则按比例转 ±25°、面积占比 ≥0.15
  判定到达；未检测到则交替转 30° 搜索；全部步骤带轮式里程计/净空校验。
- 实机 180 秒：11 次检测全部命中“手机”，自动完成 右转20°→右转11°→右转4°→
  居中→连续前进 的接近闭环；bbox 面积 0.0042→0.011、置信度 0.20→0.62，
  结束时手机稳定居中。证据 `camera_guided_01.jsonl`。
- 未到达判定阈值（0.15）：ai-w 低速死区限制每步前进 13–16 cm；参数已记录
  （`--forward-vx 0.18 --forward-seconds 3 --reach-area-ratio 0.08`）。

## 2026-08-07 Level A 搜索状态机闭环（搜索→发现→靠近）

- `run_autonomous_loop.py --mode level_a_search`：SEARCH（±90° 摆动扫描，线缆
  不持续扭转）→ DISCOVER（GroundingDINO 命中）→ APPROACH（比例转向+前进）→
  RANGE_LIMIT（以起点为圆心的 `--max-radius` 半径限制，0 为无限）。
- 实机 240 秒：9 次发现手机并自动对齐/靠近（半径 0→0.84 m），1.01 m 触发
  范围限制自动停止；机器人安全静止。
- 本次实验用小半径（1.0 m）；后续自由探索仅需 `--max-radius 0`。
  证据 `level_a_search_01.jsonl`、`logs/20260807_122450/`。
