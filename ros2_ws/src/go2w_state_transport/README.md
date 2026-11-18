# Read-only native state transport (review candidate)

This optional package replaces the Python LowState JSON capture and local
relay processes. It has no motion client, Action client, service or SDK
channel. The capture retains 16 motor q/dq, IMU and battery fields; the relay
retains the existing 20 ms timer and reliable depth-20 publisher. Atomic file
replacement and fsync are retained. A boot identifier is added; both wall and
monotonic clocks must be current. Legacy files without boot metadata require
both clocks to be current as well.

Build on the Jetson with the explicitly selected release profile:

```bash
bash scripts/go2w/build_state_transport_foxy.sh
```

Run the generated codec test executable under the runtime build directory,
and repeat an isolated ROS-domain interoperability test before use. The
build script alone does not run tests or start any robot process.

The existing Python scripts select native executables only when
`GO2W_NATIVE_LOW_STATE_CAPTURE` and `GO2W_NATIVE_LOW_STATE_RELAY` respectively
contain explicit executable paths. Missing selected binaries fail startup;
there is no silent fallback. Unset variables preserve the Python backend.
The native executables accept the same application CLI arguments, not extra
ROS CLI arguments. They use the ROS DDS environment; do not inject the
separate SDK DDS library into these processes.

`network_low_state_file` and `sdk_state_file` are existing motion-launch
arguments. The private tmpfs paths used during the stationary experiment
were supplied through those arguments, not made desktop defaults. Do not
reuse an old experiment directory or treat persisted state as a current pose.

Validation on 2026-11-09: eight native codec tests and isolated full-field
ROS serialization equivalence passed. The 15W stationary control experiment
still had LiDAR and localization freshness failures. Therefore neither native
selection nor tmpfs paths are enabled by the Jetson profile or the normal
motion launcher. This candidate does not grant motion authority or establish
performance acceptance. Preserve all existing arming and safety checks.
