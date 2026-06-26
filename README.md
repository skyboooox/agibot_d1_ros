# agibot_d1_ros

ROS 2 bridge for Agibot D1 Ultra/Edu (`zsl-1`) using the official Agibot D1
HighLevel SDK.

This is a community ROS 2 wrapper. It is not an official Agibot ROS SDK. The
official public D1 Ultra/Edu control API is the `zsl-1` HighLevel SDK exposed
by `agibot_d1_sdk`.

`unitree_ros2` is used only as a packaging reference: standalone ROS package,
setup scripts, state example, and high-level control example. D1 does not expose
Unitree-style native DDS ROS messages, so this package must not copy Unitree
topics, request formats, low-level examples, or CycloneDDS assumptions.

## Scope

- Subscribe to a guarded `geometry_msgs/msg/Twist` topic.
- Normalize bounded velocity commands to `HighLevel.move(vx, vy, yaw_rate)`.
- Run `standUp()` automatically before live motion, require SDK standing-state confirmation,
  then stream `move()` at 20 Hz.
- Publish read-only SDK state as JSON on `/agibot_d1/state`.
- Fail closed by default: dry-run mode is enabled unless explicitly disabled.
- Reject low-level or high-risk motion by omission: no `passive`, `lieDown`,
  `jump`, `frontJump`, `backflip`, `twoLegStand`, `attitudeControl`, or
  `sendMotorCmd` API is exposed.

## Supported Robot

- Agibot D1 Ultra/Edu, official SDK variant `zsl-1`.

This package does not target D1 Ultra-W (`zsl-1w`) or D1 Max (`zsm-1w`).

## Build

This repository tracks the official Agibot D1 SDK as a Git submodule under
`third_party/agibot_d1_sdk`.

Initialize submodules after cloning:

```bash
git submodule update --init --recursive
```

From a ROS 2 workspace:

```bash
colcon build --packages-select agibot_d1_ros
```

Optional environment setup, modeled after `unitree_ros2` but limited to D1 SDK
IP/port and ROS overlay sourcing:

```bash
source third_party/agibot_d1_ros/setup.sh
```

The setup scripts intentionally avoid `set -u` because ROS and colcon setup
files may read optional environment variables that are not defined in a fresh
shell.

To override the submodule SDK, pass:

```bash
colcon build --packages-select agibot_d1_ros \
  --cmake-args -DAGIBOT_D1_SDK_ROOT=/path/to/agibot_d1_sdk
```

The SDK root, from submodule or override, must contain:

- `include/zsl-1/highlevel.h`
- `lib/zsl-1/<x86_64|aarch64>/libmc_sdk_zsl_1_<arch>.so`

For policy-only CI without the SDK or ROS runtime, compile the self-test:

```bash
g++ -std=c++17 -I third_party/agibot_d1_ros/include \
  third_party/agibot_d1_ros/test/policy_self_test.cpp \
  -o /tmp/agibot_d1_policy_self_test
/tmp/agibot_d1_policy_self_test
```

`colcon test` intentionally runs package-owned tests only. Generic
`ament_lint_auto` is not enabled because it recursively scans the official SDK
submodule, which must remain vendor-owned and unmodified.

## Run

Dry-run mode logs accepted commands but does not call the SDK:

```bash
ros2 run agibot_d1_ros d1_highlevel_bridge_node \
  --ros-args -p dry_run:=true
```

The same bridge can be started through launch:

```bash
ros2 launch agibot_d1_ros d1_highlevel_bridge.launch.py dry_run:=true
```

Read-only state dry-run:

```bash
ros2 run agibot_d1_ros d1_state_node --ros-args -p dry_run:=true
```

Do not use `d1_state_node` live on a standing robot for routine status checks.
The official SDK treats an SDK connection as the active control source; if that
read-only process does not keep sending control frames, the robot-side watchdog
can switch the D1 into damping/lie-down. Use robot-owned ROS/eCAL state topics
for passive status. Live SDK state reads require both
`allow_watchdog_damping_risk:=true` and
`AGIBOT_D1_ALLOW_WATCHDOG_DAMPING_RISK=1`.

State launch:

```bash
ros2 launch agibot_d1_ros d1_state.launch.py dry_run:=true
```

Live mode requires both a ROS parameter and an environment gate:

```bash
export AGIBOT_D1_ALLOW_LIVE_SDK=1
ros2 run agibot_d1_ros d1_highlevel_bridge_node \
  --ros-args \
  -p dry_run:=false \
  -p allow_live_motion:=true \
  -p startup_standup:=true \
  -p local_ip:=192.168.168.100 \
  -p dog_ip:=192.168.168.168 \
  -p local_port:=43988
```

`startup_standup` defaults to `true`: live bridge startup automatically calls
`standUp()` when the SDK does not already report standing. If
`getCurrentCtrlmode()` already reports `ctrl_mode=1`, the bridge skips
`standUp()` to avoid repeating a stance transition. Only set
`startup_standup:=false` for expert debugging; if the robot is visually
standing but `getCurrentCtrlmode()` is stale or reports a non-standing mode,
also set `assume_standing_when_skip_standup:=true` after manual confirmation.

Bounded live smoke through ROS topic publishing:

```bash
ros2 run agibot_d1_ros d1_live_smoke \
  --live --allow-motion --command rotate --duration-s 2.0 --yaw-rate 0.10
```

`d1_live_smoke` starts the bridge, waits for a subscription on
`/agibot_d1/cmd_vel_safe`, publishes a bounded `Twist`, and stops the process
group on exit.
It uses automatic startup `standUp()` by default. Use
`--assume-standing` only when the operator has visually confirmed the D1 is
already standing and explicitly wants to skip `standUp()`.

`initRobot()` is called only by the live motion bridge after ROS publishers,
subscriptions, and SDK stream timers are created and the executor has started.
It is not used for routine status checks. After `initRobot()` succeeds, the
startup sequence either confirms the SDK already reports standing or calls
`standUp()` and waits for SDK `ctrl_mode=1` or `ctrl_mode=18` before any
`move()` command is allowed. The bridge does not call `move()` during the
standing transition because the official SDK documents `move()` as valid only
from the standing state.

Default command topic:

```text
/agibot_d1/cmd_vel_safe
```

Do not connect raw teleop, Nav2, or public `/cmd_vel` directly to this node.
Route commands through a safety gate first.

## Safety Defaults

- `vx` safe limit: `0.15 m/s`
- `vy` safe limit: `0.12 m/s`
- `yaw_rate` safe limit: `0.20 rad/s`
- Upstream command timeout before zero-speed fallback: `0.5 s`
- SDK command stream: `20 Hz` in live mode
- Stop hold: `0.5 s`
- Startup standing-state confirmation timeout: `8.0 s`
- Minimum battery for live motion: `20%`

The official D1 SDK FAQ states the robot may switch to damping/lie-down if SDK
communication is interrupted for about 3 seconds. This bridge therefore keeps a
20 Hz SDK stream in live mode and falls back to zero velocity before that SDK
timeout.

Inputs below the official non-zero minimum are normalized to zero. Inputs above
the configured safe limit are blocked and converted to a stop command.

`standUp()` returning success is not treated as proof that motion is safe. In
live mode, the bridge must also observe SDK `ctrl_mode=1` or `ctrl_mode=18`
before any `HighLevel.move()` command is sent. If the SDK never reports a
motion-ready mode, the node exits fail-closed instead of streaming movement.

## License

This package uses `GPL-3.0-or-later` in `package.xml`. The official Agibot D1
SDK is kept as a submodule and retains its upstream license.
