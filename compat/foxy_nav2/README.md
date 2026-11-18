# Isolated Foxy Nav2 compatibility overlay

Sources are pinned to official Nav2 1.1.20, commit
`a097086719c88f781aa59788eca29ac6ca5e56db`, matching the workstation packages.
`SOURCE_LOCK.json` records the original files. Headers retain their licenses;
the upstream license inventory is included in each package.

This directory is outside `ros2_ws/src` and has `COLCON_IGNORE`. Normal desktop
workspace builds do not discover or replace these packages. The dedicated
build scripts select package paths explicitly and require native
aarch64/Foxy. It installs into a separate runtime prefix.

Adapters cover the Foxy LifecycleNode constructor, required parameter types,
TF lookup helpers, finite Twist validation, and the older geometry include
path. OdomSmoother is copied from the same upstream commit so its subscription
uses the lifecycle node executor directly, avoiding Foxy's auxiliary NodeThread.
The TF listener uses a bounded executor wait so rapid startup/cancel cannot
leave teardown blocked in join. Foxy lifecycle management uses services; its manager
does not implement the newer lifecycle bond protocol. The production execution
gate remains closed while component and whole-chain safety tests are pending.

No controller, SDK lease, physical command bridge or production service is
started by building this overlay. Its initial runtime tests must use an
isolated ROS domain and synthetic inputs. A successful build is not an
execution-chain acceptance.

The parameter adapter preserves required types without inventing defaults.
The separately namespaced footprint subscriber is copied from the same pinned
upstream commit; its stamped transform uses Foxy `lookupTransform` because the
native Foxy `getCurrentPose` overload cannot accept the footprint timestamp.
The compatibility tests distinguish the historical footprint pose from the
latest robot pose and reject missing transforms. Native costmap geometry
helpers are reused; the system costmap package is not replaced.

The unused `CollisionMonitorState` test include is removed: this pinned node
neither publishes nor consumes that newer message. Required-parameter errors,
TF failure and sensor timeouts remain visible. In particular, upstream 1.1.20
can skip a stale observation source; proving the independent guard/bridge
stops the whole chain remains necessary before production execution.

Build and test entries:

```sh
bash scripts/go2w/build_nav2_safety_foxy.sh
bash scripts/go2w/test_nav2_safety_foxy.sh
```

The test entry explicitly binds CycloneDDS to loopback on domain 99. Neither
entry loads the robot profile, acquires a lease or starts a physical bridge.

RPP and Spin use additional independent build prefixes:

```sh
bash scripts/go2w/build_rpp_foxy.sh
bash scripts/go2w/test_rpp_foxy.sh
bash scripts/go2w/build_spin_foxy.sh
bash scripts/go2w/probe_nav2_safety_foxy.sh NEW_OUTPUT_DIRECTORY --with-controller
```

`go2w_regulated_pure_pursuit_controller` is a distinct plugin adapted to the
native Foxy Controller interface. It requires the configured native
SimpleGoalChecker and rejects tolerance changes while active. The native
checker snapshots its parameters at initialization. The tests include actual
computed commands after dynamic speed updates, not just parameter ACKs.

`go2w_nav2_compat_msgs/Spin` preserves the pinned server-side time allowance
without replacing the system nav2_msgs ABI. `go2w_nav2_spin_compat` adapts the
pinned Spin to Foxy's Recovery plugin interface. Its separately namespaced
collision checker preserves one costmap/footprint fetch per projected sweep
and uses the timestamp-correct footprint subscriber above. These plugins
are explicitly selected by the Jetson profile; the Foxy production execution
gate still remains closed pending hardware and downstream safety acceptance.

BT ComputePathToPose, FollowPath and Wait have a separate plugin library and
bounded IO executor. Each goal owns its callback state. Timeout and halt latch
cancellation until late acceptance; stale results cannot complete a new goal.
ACK state and deadline are sampled under the same lock. The existing 20ms ACK
bound is preserved. Native lifecycle tests cover delayed acceptance and old
results. The full synthetic probe also checks all eight child exit codes.

```sh
bash scripts/go2w/build_bt_foxy.sh
bash scripts/go2w/test_bt_foxy.sh
bash scripts/go2w/probe_navigation_chain_foxy.sh NEW_OUTPUT_DIRECTORY
```

The full probe requires nonzero motion before injecting its stop faults. It
uses domain 99, loopback only, synthetic pose/sensors and acceptance output;
there is no SDK or physical bridge. Its participant index bound is explicitly
120, matching the project DDS configuration. Graceful shutdown verifies the
launcher's sole child lifecycle manager, uses that launcher's ROS environment,
and checks every managed node reaches finalized before SIGINT. The same owned
shutdown helper is used by the paired production stop entry.

`SOURCE_MAP.json` maps pinned originals to adapted files. Regenerate the review
diff with `python3 compat/foxy_nav2/tools/generate_upstream_diff.py LOCAL_NAV2_GIT_REPO`;
this first verifies every original against `SOURCE_LOCK.json`. New build/test
files and the Foxy IO transport are authored adapters; the diff is for review,
not a substitute for native builds or whole-chain acceptance.

The candidate `go2w_nav2_controller_compat` restores the configured controller
exception tolerance, costmap readiness and path/goal handling. Its distinct executable/core namespace uses the
official Foxy 0.4.7 Controller and GoalChecker ABI and the installed native
plugins. `CONTROLLER_SOURCE_LOCK.json` pins that base and the official Humble
1.1.20 reference separately, including geometry utilities and four pinned Foxy
API context files. `CONTROLLER_UPSTREAM_PATCHES.diff` is regenerated
with `tools/generate_controller_upstream_diff.py LOCAL_NAV2_GIT_REPO` after
verifying both sets of original hashes. The small authored
`controller_patience.hpp` policy is called directly by the server.

The policy returns zero on all six axes during an allowed controller-plugin
PlannerException, retains the strict upstream `elapsed > tolerance` boundary,
resets time on a valid command or a distinct new action, and propagates unrelated
exceptions. Pose and progress failures stay outside that policy. Parameter
validation does not mutate cached state. As in the pinned Humble server, core
updates are rejected during an action; plugin-scoped speed updates remain
available. Committed core values are read before a new action. The native odometry subscriber is
reused with a locked snapshot getter to match its locked write path.
The configured default `publish_zero_velocity=true` is preserved. Its optional
false value applies only to normal success; cancellation, failures and shutdown
still zero as in the pinned server. Its bool type and action-time update gate
are covered by the parameter tests; native output behavior remains to be tested.

Remaining-distance feedback measures the path from its closest pose, with the
robot first transformed into the path frame. The target remains stamped in its
map frame and is transformed for each goal check at the current robot pose's
time; failed transforms cannot substitute a default pose. A stale costmap emits
zero and waits with cancellation/liveness checks between 10ms sleeps. Invalid
or canceled pending goals cannot continue computing the previous path. These
are source candidates, not measured physical stopping times. Lifecycle shutdown
deactivates the action, and destruction joins it before destroying its mutex,
plugins or odometry.

Only the Foxy launch branch selects this package. Desktop Humble still uses
`nav2_controller`. The new prefix is versioned per release; neither a prior
compatibility prefix nor the native system package is overwritten.

```sh
bash scripts/go2w/build_controller_foxy.sh
bash scripts/go2w/test_controller_foxy.sh
# Then run the full synthetic probe from this release, before any execution gate review.
bash scripts/go2w/probe_navigation_chain_foxy.sh NEW_OUTPUT_DIRECTORY
```

Native acceptance on 2026-11-09 used Python 3.8/Foxy on aarch64, separate
versioned prefixes, domain 99 and loopback-only DDS. The 05 candidate built and
passed parameter/navigation cases but failed process cleanup: the native Foxy
SimpleProgressChecker retains a strong node reference. Its source is now pinned
in CONTROLLER_SOURCE_LOCK.json. Cleanup joins the action first and releases both
checker instances; shutdown from active also deactivates and cleans the owned
resources before ROS exits. The action remains unavailable before activation.

The corrected 20261109_01 candidate passed the 30 native policy/semantic cases
and the complete synthetic navigation chain, including actual parameter RPCs,
computed speed changes, navigation, cancel, obstacle/localization stops and Spin.
All six managed nodes finalized and all eight child exit codes were zero. The
first 09 whole-chain attempt exposed a fixed 0.4s startup assumption in the test;
it now requires fresh nonzero commands from the current goal within a bounded
2s window before injecting faults. Stop deadlines and failure assertions remain.

Additional native action/lifecycle regression is reproducible with:

```sh
bash scripts/go2w/probe_controller_lifecycle_foxy.sh NEW_OUTPUT_DIRECTORY
```

It checks inactive-goal rejection, invalid preemption, progress failure, missing
path transforms, stale-costmap cancellation, repeated configure/activate cycles
and shutdown while a synthetic goal is executing, followed by an actual clean
process exit. Its private test configuration shortens progress time and enables
observation freshness; production settings are unchanged. It creates no SDK,
physical bridge or robot lease. Receipt-time camera data, continuous loaded
localization, downstream hardware/watchdog behavior and supervised movement are
separate acceptance requirements. A successful isolated test is not a physical
stopping-distance measurement or permission to open the production gate.

General Nav2 ABI equivalence is not claimed: native Foxy FollowPath lacks Humble's
per-goal goal-checker selector, and the project uses its configured singleton
checker. The project speed route uses the plugin's SetParameters API; the newer
generic SpeedLimit topic is not implemented here. These differences remain in
the compatibility audit. The production execution gate remains closed.
