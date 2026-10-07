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

The defaults in `params/driver_manager_params.yaml` are the real robot's addresses (receive on
`192.168.1.103:50010`, send to `192.168.1.120:50051`). Without that NIC the bind fails and the node
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
├── launch/driver_manager.launch.py   # reads robot_id from system.ini as the namespace
├── params/driver_manager_params.yaml # UDP addresses and velocity correction gains
├── src/
│   ├── main.rs
│   └── driver_manager_node/
│       ├── mod.rs          # wiring: parameters → sockets → pub/sub/service → telemetry thread
│       ├── parameters.rs   # velocity gains (dynamically settable), UDP addresses (read-only)
│       ├── protocol.rs     # pure packet <-> struct functions; cargo test needs no ROS
│       ├── session.rs      # UDP sockets
│       ├── command.rs      # send commands to the controller, safety lock
│       ├── telemetry.rs    # receive telemetry and publish it as ROS messages
│       ├── publishers.rs   # imu / motor_states / battery_state / mode / safety_locked
│       ├── subscriber.rs   # cmd_vel -> AXES
│       └── service.rs      # set_motion_key / set_policy_mode / set_speed_scale / reset_safety
│
│   # Only for developing this repo on its own; SyncAI-Robot-Workspace does not use these
├── interface.repos         # vcstool list: where the shared syncai_common messages come from
└── .devcontainer/          # Dockerfile + devcontainer.json
```

The workspace inside the container looks like this, matching the paths in SyncAI-Robot-Workspace:

```
/workspace/                     # named volume syncai-ros2-rust_workspace
├── build/ install/ log/ .cargo/
└── src/
    ├── syncai_common/          # shared messages, vcs-imported by postCreateCommand
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

`syncai_common` comes from the Workspace's own `interface.repos`; this repo's `interface.repos` and
`.devcontainer/` are not used there (colcon only looks at `package.xml`). The Workspace image needs
the Rust toolchain, `colcon-ros-cargo` and the rclrs dependencies.

## The syncai_driver_manager node

The boundary between ROS 2 and the low-level controller (gait controller): ASCII commands go out
over UDP and ASCII telemetry comes in over UDP. It is a Rust port of the C++ (rclcpp)
`syncai_driver_manager` in SyncAI-Robot-Workspace, and **its external interface is deliberately
identical**: the node name `driver_manager`, the executable `driver_manager_node`, parameter names,
topics, message types, QoS and service names are all the same, so it is a drop-in replacement for
the C++ version and `syncai_robot_state` / `syncai_backend` need no changes. For behavioural details
such as the packet format, the motion key table and where the velocity correction comes from, the
C++ README is authoritative.

| Direction | Interface |
| --- | --- |
| Publishes | `imu` (`syncai_common/IMUState`, SensorData), `motor_states` (`syncai_common/MotorStates`, SensorData), `battery_state` (`sensor_msgs/BatteryState`, reliable depth 10), `mode` (`std_msgs/Int32MultiArray`, reliable depth 10), `safety_locked` (`std_msgs/Bool`, reliable + transient local depth 1; published at startup and whenever the safety lock changes; **Rust-only addition**, not in the C++ version) |
| Subscribes | `cmd_vel` (`geometry_msgs/Twist`) → `AXES vx vy wz` |
| Services | `set_motion_key`, `set_policy_mode`, `set_speed_scale`, `reset_safety` |
| Parameters | `telemetry_recv_ip/port`, `command_target_ip/port` (read-only), `scale_fwd` / `scale_back` / `scale_left` / `scale_right` / `scale_turn_l` / `scale_turn_r` (>= 0, changeable at runtime with `ros2 param set` or `set_speed_scale`; never written back to the YAML) |

### Threads

| Work | Runs on | C++ equivalent |
| --- | --- | --- |
| `cmd_vel` | its own rclrs Worker | `cmd_vel_cb_group_` |
| The four services | one shared Worker, run one at a time | `services_cb_group_` |
| Telemetry receive | its own `std::thread`, outside the executor | same |

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

`protocol.rs` is pure functions (packet <-> struct), so the unit tests need neither a ROS
environment nor the robot. For end-to-end tests use loopback
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
