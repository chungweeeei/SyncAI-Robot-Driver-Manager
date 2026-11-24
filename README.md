# syncai_driver_manager (Rust)

The ROS 2 package `syncai_driver_manager`, written with [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)
(`rclrs`), plus a Docker environment for developing it on its own.

**This repo is itself a single colcon package** (`package.xml` / `Cargo.toml` live at the root).
Like SyncAI-Robot-Backend, it is pulled into SyncAI-Robot-Workspace with vcstool as
`src/syncai_driver_manager`; see [Using it in SyncAI-Robot-Workspace](#using-it-in-syncai-robot-workspace).

## Why Docker

ROS 2 has no official macOS support (binaries are only provided for Ubuntu / Windows), and building
ROS 2 + `rclrs` natively on macOS tends to get stuck on dependencies. So the whole environment is
packaged into a Linux container: **edit code on the host, build and run inside the container**.

What the container provides:

| Item | Notes |
| --- | --- |
| ROS 2 Humble | LTS release (supported until 2027-05), `ros-base`, on Ubuntu 22.04 |
| Rust 1.85 | The minimum version `rclrs` requires |
| `colcon-cargo` / `colcon-ros-cargo` | Teach `colcon` about `ament_cargo` packages |
| `rosidl_rust` | The generator that turns `.msg` files into Rust types |
| Non-root user `ros` | uid/gid 1000 with passwordless `sudo`, so mounted files never end up owned by root |
| `rclrs` (from source) | `rclrs` 0.7.0 on crates.io depends on `rosidl_runtime_rs` ^0.6, but the generator on the main branch emits 0.7 and mixing them does not compile, so `rclrs` is built from source too |
| Rebuilt message packages | The apt message packages have no Rust bindings, so `std_msgs`, `example_interfaces`, etc. are rebuilt from source |

All of the above lives in the underlay workspace `/opt/ros2_rust_underlay` inside the image and is
sourced automatically when you enter the container.

## Quick start (VS Code Dev Container)

The only development environment is the Dev Container (`.devcontainer/`); there is no Makefile or
docker-compose. Once the package is in SyncAI-Robot-Workspace it is built with the Workspace's own
image, so this setup is only for developing this repo on its own.

1. Install the VS Code [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension
2. Open this repo in VS Code and run **Dev Containers: Reopen in Container**
3. The first run builds the image (slow, since message packages are compiled from source); later
   runs open instantly. After the container is created, `postCreateCommand` automatically
   `vcs import`s the shared message package `syncai_common` and runs `colcon build` once
4. Run the node from a VS Code terminal (the namespace comes from `robot_id` in
   `~/robot_ws/config/system.ini`, falling back to `default_robot`):

```bash
ros2 launch syncai_driver_manager driver_manager.launch.py
```

Without VS Code, use the [devcontainer CLI](https://github.com/devcontainers/cli):
`devcontainer up --workspace-folder .`, then `docker exec -it syncai-ros2-rust bash -l`.

The G23's addresses in `params/models/g23.yaml` are the real robot's (receive on
`192.168.1.103:50010`, send to `192.168.1.120:50051`); the code defaults without any params
file are `0.0.0.0:50012` and `192.168.1.120:50051`. Without that NIC the bind fails and the node
exits. When not connected to the robot, use loopback:

```bash
ros2 run syncai_driver_manager driver_manager_node --ros-args -r __ns:=/default_robot \
    -p telemetry_recv_ip:=127.0.0.1 -p command_target_ip:=127.0.0.1
```

In another terminal, check it with the ROS 2 CLI:

```bash
ros2 topic list
ros2 service call /default_robot/set_motion_key syncai_common/srv/SetMotionKey "{key: '0'}"
ros2 topic echo /default_robot/battery_state
```

The node's interface (topics / services / parameters / UDP packet format) is identical to the C++
`syncai_driver_manager` in SyncAI-Robot-Workspace; see that README for details.

### Dev Container configuration

| File | Purpose |
| --- | --- |
| `Dockerfile` | ROS 2 + Rust + rclrs dependencies (the build context is `.devcontainer/`) |
| `entrypoint.sh` / `setup_env.sh` | Source in order: ROS 2 → underlay → workspace |
| `devcontainer.json` | Build args, volumes, host network, environment variables, VS Code extensions and settings |

Notes:

* The container name is fixed to `syncai-ros2-rust`, so `docker exec syncai-ros2-rust ...` works
  from the host.
* It uses the host network (UDP and DDS go over the physical NIC / lo). `ROS_DOMAIN_ID` defaults to
  2 and the RMW is cyclonedds; both are in `containerEnv` in `devcontainer.json`.
* `/workspace` is the named volume `syncai-ros2-rust_workspace`; the cargo cache and the Claude Code
  settings each have a volume too, so they survive container rebuilds. To start from scratch,
  `docker volume rm syncai-ros2-rust_workspace`.
* rust-analyzer only works after one `colcon build`: the message crates are wired in by
  `colcon-ros-cargo` at build time (`postCreateCommand` already does this).

### Common commands (inside the container)

```bash
cd /workspace && colcon build --symlink-install            # build
cd /workspace/src/syncai_driver_manager
cargo fmt                                                  # format (CI uses --check)
cargo clippy --target-dir /workspace/build/.clippy --all-targets
cargo test --target-dir /workspace/build/.clippy
```

## Project layout

```
.                           # = the ROS 2 package syncai_driver_manager (build_type: ament_cargo)
├── package.xml
├── Cargo.toml / Cargo.lock
├── rustfmt.toml / clippy.toml / .editorconfig  # formatting and lint settings
├── launch/driver_manager.launch.py   # robot_id (namespace) and robot_model (params file) from system.ini
├── params/driver_manager_params.yaml # what every model shares: the monitor's period and timeouts
├── params/models/g23.yaml  # G23: protocol, UDP addresses, Livox topics, velocity correction gains
├── params/models/m20.yaml  # Lynx M20: protocol, motion host, axis full scale, vendor lidar topics
├── src/
│   ├── main.rs
│   └── driver_manager_node/
│       ├── mod.rs          # wiring: parameters → sockets → pub/sub/service → telemetry thread
│       ├── parameters.rs   # robot_protocol, then that robot's set: G23 (UDP, gains) or M20; monitor
│       ├── protocol.rs     # pure packet <-> struct functions; cargo test needs no ROS
│       ├── session.rs      # UDP sockets
│       ├── command.rs      # send commands to the controller, safety lock
│       ├── telemetry.rs    # receive telemetry and publish it as ROS messages
│       ├── health.rs       # pure sensor bookkeeping: rates, statuses, the stdout table
│       ├── monitor.rs      # lidar / camera subscriptions + the status-table thread
│       ├── publishers.rs   # imu / motor_states (+ _low_rate) / battery_state / mode / safety_locked
│       ├── subscriber.rs   # cmd_vel -> AXES (G23) / the M20 link
│       ├── drive.rs        # which robot: G23 (CommandLink) or M20 (M20Link)
│       ├── m20_protocol.rs # pure Lynx M20 header + JSON functions; cargo test needs no ROS
│       ├── m20_link.rs     # the M20 socket: heartbeat, 20 Hz axis command, status log
│       ├── service.rs      # set_motion_key / set_policy_mode / set_speed_scale / set_safety_lock
│       └── tests/          # unit tests, one file per module (included with #[path])
│
│   # Only for developing this repo on its own; SyncAI-Robot-Workspace does not use these
├── interface.repos         # vcstool list: where the shared syncai_common messages come from
└── .devcontainer/          # Dockerfile + devcontainer.json
    └── livox_msgs/         # messages-only livox_ros_driver2 + import_livox_msgs.sh
```

The workspace inside the container looks like this, matching the paths in SyncAI-Robot-Workspace:

```
/workspace/                     # named volume syncai-ros2-rust_workspace
├── build/ install/ log/ .cargo/
└── src/
    ├── syncai_common/          # shared messages, vcs-imported by postCreateCommand
    ├── livox_ros_driver2/      # messages only, from .devcontainer/livox_msgs (postCreateCommand)
    └── syncai_driver_manager/  # bind mount: this repo
```

## Using it in SyncAI-Robot-Workspace

The Workspace pulls this repo into `src/syncai_driver_manager` with vcstool (replacing the C++
version; the two packages share a name and cannot coexist):

```yaml
repositories:
  src/syncai_driver_manager:
    type: git
    url: https://github.com/chungweeeei/SyncAI-Robot-Driver-Manager.git
    version: main
```

`syncai_common` comes from the Workspace's own `interface.repos` and `livox_ros_driver2` from its
`third-party.repos` (the real driver, built against Livox-SDK2); this repo's `interface.repos` and
`.devcontainer/` are not used there (colcon only looks at `package.xml`). The Workspace image needs
the Rust toolchain, `colcon-ros-cargo` and the rclrs dependencies.

## The syncai_driver_manager node

The boundary between ROS 2 and the low-level controller (gait controller): ASCII commands go out
over UDP and ASCII telemetry comes in over UDP. It is a Rust port of the C++ (rclcpp)
`syncai_driver_manager` in SyncAI-Robot-Workspace, and **its external interface is deliberately
identical**: the node name `driver_manager`, the executable `driver_manager_node`, parameter names,
topics, message types, QoS and service names are all the same, except for the `safety_locked` topic
(added) and the `set_safety_lock` service (replaces `reset_safety`); see the differences below.
Anything that calls `reset_safety` must switch to `set_safety_lock`. For behavioural details
such as the packet format, the motion key table and where the velocity correction comes from, the
C++ README is authoritative.

| Direction | Interface |
| --- | --- |
| Publishes | `imu` (`syncai_common/IMUState`, SensorData), `motor_states` (`syncai_common/MotorStates`, SensorData), `motor_states_low_rate` (the same messages thinned to 20 Hz, SensorData; what `syncai_robot_state` and the backend subscribe — `motor_states` stays at the telemetry rate, ~200 Hz), `battery_state` (`sensor_msgs/BatteryState`, reliable depth 10), `mode` (`std_msgs/Int32MultiArray`, reliable depth 10), `safety_locked` (`std_msgs/Bool`, reliable + transient local depth 1; published at startup and whenever the safety lock changes; **Rust-only addition**, not in the C++ version) |
| Subscribes | `cmd_vel` (`geometry_msgs/Twist`) → `AXES vx vy wz`; dropped (nothing sent) while the safety lock is engaged. For the sensor monitor only (**Rust-only addition**, counted, never used): `monitor_lidar_topic` (default `livox/lidar`, `livox_ros_driver2/CustomMsg` or `sensor_msgs/PointCloud2` by `monitor_lidar_type`), `monitor_imu_topic` (default `livox/imu`, `sensor_msgs/Imu`), `image_raw/compressed` (`sensor_msgs/CompressedImage`), all best-effort |
| Services | `set_motion_key`, `set_policy_mode`, `set_speed_scale`, `set_safety_lock` (`std_srvs/SetBool`: `true` engages the safety lock without sending any command, `false` releases it **and always sends `RESET_SAFETY`**, which clears the controller's own lock; **replaces the C++ version's `reset_safety`** (`std_srvs/Trigger`), so callers of `reset_safety` must switch) |
| Parameters | `robot_protocol` (`g23` / `m20`, read-only, declared first: only that robot's own parameters are declared). G23 only: `telemetry_recv_ip/port`, `command_target_ip/port` (read-only), `scale_fwd` / `scale_back` / `scale_left` / `scale_right` / `scale_turn_l` / `scale_turn_r` (>= 0, changeable at runtime with `ros2 param set` or `set_speed_scale`; never written back to the YAML). M20 only: `motion_host_ip/port`, `max_vx` / `max_vy` / `max_wz`, `cmd_timeout` (read-only; see "Driving a Lynx M20"). Both: `monitor_report_period` / `monitor_telemetry_timeout` / `monitor_lidar_timeout` / `monitor_imu_timeout` / `monitor_camera_timeout` / `monitor_command_idle` (seconds, read-only; see below), `monitor_lidar_topic` / `monitor_lidar_type` / `monitor_imu_topic` (read-only; see below) |

### Params files and robot models

The launch passes two files, the second winning key by key:

| File | Holds |
| --- | --- |
| `params/driver_manager_params.yaml` (`params_file:=`) | what every model shares: the sensor monitor's period and timeouts |
| `params/models/<robot_model>.yaml` | everything that is one robot's: `g23.yaml` the protocol, UDP endpoints, Livox topics and velocity scales; `m20.yaml` the protocol, motion host, axis full scale and the vendor's lidar topics |

`<robot_model>` is `[system] robot_model` of the system INI, read as `g23` (with a warning) when the
key is missing, as the Workspace's own launches read it. **A model with no file is refused**: the
launch raises instead of starting the node on the code defaults (telemetry `0.0.0.0:50012`, every
scale 1.0), which would bind, look healthy and never hear the controller. Adding a model means
adding its file here.

Run without the launch (`ros2 run`), only the code defaults apply, and they are not robot01's for
the G23 keys either; pass the model file with `--params-file` or the values with `-p`.

### Driving a Lynx M20 (`robot_protocol: m20`)

The same node drives DEEP Robotics' Lynx M20 when `robot_protocol` is `m20`. The launch sets it by
loading `params/models/m20.yaml` after the factory params when the system INI's
`[system] robot_model` is `m20`, so the session spec is the same for both robots. The ROS
interface does not change; what is behind it does:

| ROS call | Sent to the M20 (UDP `motion_host_ip:motion_host_port`, 16-byte header + JSON) |
| --- | --- |
| `set_motion_key "0"` (stand) | axis zero, usage mode Regular (`1101/5 Mode 0`), then motion state `2/22 MotionParam 1` |
| `set_motion_key "2"` (lie down) | axis zero, then `2/22 MotionParam 4` |
| `cmd_vel` | axis command `2/21`: X = `linear.x / max_vx`, Y = `linear.y / max_vy`, Yaw = `angular.z / max_wz`, each clamped to [-1, 1]; re-sent at 20 Hz while younger than `cmd_timeout`, then zero three times |
| every other motion key, `set_policy_mode`, `set_safety_lock` | nothing sent; `success: false` with "Not supported on the M20" |
| `set_speed_scale` | nothing changed; `success: false` |

A heartbeat (`100/100`) goes out every 500 ms from start-up, followed once by Regular mode: the
robot reports its status only to an address that keeps sending heartbeats, and honours axis
commands only in Regular mode. The node logs each change of the reported `MotionState` (e.g.
`[M20] MotionState 1 (stand)`) and counts the robot's datagrams in the monitor's telemetry row;
it publishes none of the G23 topics (`imu`, `motor_states`, `battery_state`, `mode`), and opens
neither G23 socket.

`max_*` are unmeasured: the defaults (2.0 m/s, 1.0 m/s, 2.0 rad/s) are chosen so a command
maps to a smaller fraction of full speed than intended, never a larger one. The protocol source is
the "Lynx M20 Series Software Development Manual (Beta)" V0.0.9-0; the ASDU's `Time` is sent in
UTC, not local time, for want of a time zone database in std.

Testing without the robot: `-p robot_protocol:=m20 -p motion_host_ip:=127.0.0.1` and a UDP listener
on 30000; **never point it at `10.21.31.103` with the robot in reach of people**.

### Sensor status on stdout

The node watches five things and prints them as one table every `monitor_report_period` seconds
(5 s by default), so attaching to the `driver_manager` pane answers "is everything alive?" at a
glance:

```
[Monitor] Sensor status (uptime 00:12:34)
  SENSOR     STATUS        RATE       AGE  DETAIL
  telemetry  OK         19.5 Hz    0.02 s  from 192.168.1.120:50051, parse warnings 0 (total 0)
  command    IDLE        0.0 Hz   12.30 s  sent 1234, failed 0, safety_lock off
  lidar      OK         10.0 Hz    0.05 s  points 19968
  imu        OK        200.0 Hz    0.00 s  received 145210
  camera     STALE       0.0 Hz    3.21 s  last frame 98 KB
```

| Row | Source | Statuses |
| --- | --- | --- |
| `telemetry` | datagrams on the telemetry socket | `NO DATA` (never), `OK`, `STALE` (none for `monitor_telemetry_timeout`) |
| `command` | sends on the command socket | `IDLE` (nothing sent for `monitor_command_idle`: normal while nobody drives), `OK`, `ERROR` (a send failed within the last 2 s, e.g. ICMP port unreachable). UDP has no ack, so the controller can never be shown as down from this side |
| `lidar` | `monitor_lidar_topic` (G23: `livox/lidar` CustomMsg; M20: `/LIDAR/POINTS_UDP` PointCloud2, absolute because the vendor publishes it without a namespace; `monitor_lidar_type` `livox` / `pointcloud2` picks the message type) | `NO DATA`, `OK`, `STALE` (`monitor_lidar_timeout`) |
| `imu` | `monitor_imu_topic` (`sensor_msgs/Imu`; G23: `livox/imu`, the MID360's; M20: the vendor's `/IMU`) | `NO DATA`, `OK`, `STALE` (`monitor_imu_timeout`). Its own row, not part of `lidar`, because the two stop independently (separate UDP ports on the MID360, separate sources on the M20) |
| `camera` | `image_raw/compressed` from `syncai_camera_feeder` | `NO DATA`, `OK`, `STALE` (`monitor_camera_timeout`) |

Rates are counts over the last 2 s. A change of status is logged the moment it happens (checked
every 0.5 s): `WARN` into `STALE` / `NO DATA` / `ERROR`, `INFO` on recovery; `OK` <-> `IDLE` on the
command row is not logged. The table replaces the old once-a-second `[Telemetry] Received N bytes`
line.

Watching the lidar and camera costs a deserialised copy of every message in this process (a ~20k
point CustomMsg at 10 Hz, a JPEG at 30 Hz): rclrs has no serialized subscription to avoid it.

### Threads

| Work | Runs on | C++ equivalent |
| --- | --- | --- |
| `cmd_vel` | its own rclrs Worker | `cmd_vel_cb_group_` |
| The four services | one shared Worker, run one at a time | `services_cb_group_` |
| Telemetry receive (G23) | its own `std::thread`, outside the executor | same |
| M20 send / receive (M20) | two `std::thread`s in `M20Link`: heartbeat + 20 Hz axis command, status | — (Rust-only) |
| Lidar / camera subscriptions | one rclrs Worker of their own | — (Rust-only) |
| Status table | its own `std::thread` (`monitor`) | — (Rust-only) |

Everything shared across workers / threads (velocity gains, the safety lock, the command socket) is
thread-safe: the gains are ROS parameters, the safety lock is an `AtomicBool`, and the socket is an
`Arc<UdpSocket>`.

### Differences from the C++ version

* **The params YAML key is `/**`, not `/**/driver_manager`.** rclrs only matches keys that are
  exactly `/**` or the node's full name (`/<robot_id>/driver_manager`) and **does not expand
  wildcards**; `/**/driver_manager` raises no error, it just silently falls back to the code
  defaults for everything (the velocity gains go back to 1.0).
* **The velocity gains are ROS parameters** and can be changed at runtime; the range is >= 0 and
  negative values are rejected (the C++ version accepts them).
* **`reset_safety` is replaced by `set_safety_lock` (`std_srvs/SetBool`).** It can also engage the
  lock from outside (`true`, no lie-down command sent), not only release it (`false`). While the
  lock is engaged, cmd_vel is dropped as well, not only motion keys other than ESTOP.
* **Releasing (`false`) always sends `RESET_SAFETY` to the controller**, whether or not this node's
  lock was engaged. The controller keeps a lock of its own, engaged by ESTOP (key 4), damping
  (key 3, `MODE R`), a lost-control judgement or a rejected StandUp output; while it holds, every
  cycle is forced to damping, and over ROS UDP only `RESET_SAFETY` releases it — without it the
  robot stayed in damping until the controller was restarted. After the reset the controller
  waits for a stand command (key 0); it does not stand up by itself. On an unlocked controller the
  command only zeroes the velocity it holds. This node does not yet read the controller's
  `SAFETY_LOCKED` telemetry, so `safety_locked` still shows this node's lock only.
* **Failed command sends are logged** (throttled to once a second); the C++ version discards the
  return value of `sendto()`.
* **There is no SIGINT handler.** rclrs does not handle signals, so Ctrl-C / `ros2 launch` shutdown
  terminates the process with the default action and `Drop` does not run (the OS closes the
  sockets). Also, a process backgrounded with `&` from a non-interactive shell script ignores
  SIGINT; stop it with SIGTERM in scripts.

### Testing

```bash
# Inside the container
cd /workspace/src/syncai_driver_manager && cargo test --target-dir /workspace/build/.clippy
```

The unit tests are in `src/driver_manager_node/tests/`, one file per module, each included at the
end of its module with `#[cfg(test)] #[path = "tests/<module>.rs"] mod tests;`, so they can still
test private functions. `protocol.rs`, `m20_protocol.rs` and `health.rs` are pure functions, so
their unit tests need neither a ROS environment nor the robot. For end-to-end tests use loopback
(`-p telemetry_recv_ip:=127.0.0.1 -p command_target_ip:=127.0.0.1` plus different ports) and a
separate `ROS_DOMAIN_ID`, and **never send commands to `192.168.1.120`**: that is the real robot's
controller.

## Code highlights

A few notable ways `rclrs` differs from `rclcpp` / `rclpy`:

* **The executor comes before the node.** First `Context::default_from_env()?.create_basic_executor()`,
  then create the node from the executor, and finally `executor.spin(...)`.
* **A Worker is rclrs's callback group.** `node.create_worker::<T>(initial)` creates a worker that
  owns some state, and the subscription / service / timer callbacks it creates receive `&mut T`.
  * Callbacks under the same worker **run one at a time** (like an rclcpp `MutuallyExclusive`
    group), so the payload does not need to be wrapped in `Arc<Mutex<..>>`.
  * Each worker has **its own wait-set thread** and its callbacks run directly on it, so different
    workers run **in parallel**, even with the single-threaded `BasicExecutor`.
  * `node.create_subscription(...)` / `node.create_service(...)` attached directly to the node all
    queue on the executor's single thread and block each other.
* **Message types come from `ros-env`.** `use ros_env::std_msgs::msg::String;`. Message packages
  are not listed under `[dependencies]` in `Cargo.toml`; they are declared in `package.xml` and
  wired in by `colcon-ros-cargo` at build time.

## The shared message package syncai_common

The `msg` / `srv` / `action` definitions are not in this repo but in
[SyncAI-Robot-Interface](https://github.com/chungweeeei/SyncAI-Robot-Interface)
(colcon package name `syncai_common`), shared by the whole syncai stack. As in
`SyncAI-Robot-Backend` and `SyncAI-Robot-Workspace`, it is pulled into the container workspace's
`src/` with [vcstool](https://github.com/dirk-thomas/vcstool) rather than a git submodule.

`postCreateCommand` imports it automatically when the Dev Container is created (skipped if it is
already there). To update it by hand, inside the container:

```bash
cd /workspace
vcs import < src/syncai_driver_manager/interface.repos           # first time, creates src/syncai_common/
vcs import --force < src/syncai_driver_manager/interface.repos   # after changing the version, or to discard local changes
```

Things to know:

* **`vcstool` is in the container; the host does not need it.** The image already has
  `python3-vcstool`.
* **It lives in the workspace volume, not in this repo.** It is another git repo's working tree:
  change messages in that checkout inside the container and commit them there. The next `--force`
  import overwrites anything uncommitted, and deleting the workspace volume deletes it too.
* **It is pinned to the `dev` branch**, consistent with the backend / workspace (`main` lags behind
  `dev`). To change versions, edit `version` in `interface.repos` and commit that one-line diff.

### Using these messages in the node

`syncai_common` is a standard `rosidl` package; during `colcon build` the underlay's
`rosidl_generator_rs` also generates Rust bindings for it (`msg` / `srv` / `action`).
Use it like `std_msgs`: **do not add it to `Cargo.toml`**; add it to the package's `package.xml`:

```xml
<depend>syncai_common</depend>
```

Then in code:

```rust
use ros_env::syncai_common::msg::RobotState;
```

This works through the `ros-env` build script: it scans every
`<prefix>/share/<package>/rust/Cargo.toml` on `AMENT_PREFIX_PATH` and `include!`s every crate marked
`[package.metadata.ros-env] include = true` into the `ros_env` crate; the crate generated for
`syncai_common` carries that marker. The `AMENT_PREFIX_PATH` colcon passes when building a package
only contains **the dependencies that package declared**, so if `package.xml` is missing the
`<depend>`, `ros_env::syncai_common` does not exist.

## The container user

The container runs as `ros` (uid/gid 1000) by default, not root, so even when the bind-mounted repo
is written from inside the container (e.g. by `cargo fmt`) the files do not become root-owned on the
host. `sudo` needs no password when you need to install packages.

The Rust toolchain is installed in `/home/ros/.cargo`, where the cargo registry / git cache volumes
are mounted too.

If your host user's uid is not 1000 (common on Linux; macOS is unaffected), change `USER_UID` /
`USER_GID` under `build.args` in `devcontainer.json` and **Rebuild Container**.

## Switching ROS 2 distro

The default is Humble. The distro name is passed into the Dockerfile from `build.args.ROS_DISTRO`
in `devcontainer.json` and used for apt package names, the `.repos` URL and the setup script;
**Rebuild Container** after changing it. Note that `FROM ros:humble-ros-base` in
`.devcontainer/Dockerfile` is currently hard-coded and has to be changed along with it.

Distros that `ros2_rust` currently ships a `.repos` file for: `humble`, `jazzy`, `kilted`, `lyrical`,
`rolling`.

> Humble runs on Ubuntu 22.04, whose pip does not know `--break-system-packages`; from Jazzy on
> (24.04) it is required instead. The pip step in the Dockerfile already branches on the distro.

## Known limitations

* All nodes currently run in the same container. If they are split into several containers later,
  they must share a Docker network and the same `ROS_DOMAIN_ID`.
* No GUI (RViz / rqt). Using one needs X11 forwarding set up separately (XQuartz on macOS).

## References

* [ros2_rust](https://github.com/ros2-rust/ros2_rust)
* [rclrs examples](https://github.com/ros2-rust/examples/tree/main/rclrs)
* [rclrs on docs.rs](https://docs.rs/rclrs)
