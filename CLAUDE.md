# CLAUDE.md

用 Rust（`rclrs` 0.8）寫的 ROS 2 套件 `syncai_driver_manager`：ROS 2 與四足機器人下位機
（gait controller）之間的 UDP bridge。它是 `~/SyncAI-Robot-Workspace/src/syncai_driver_manager`
（C++ / rclcpp）的移植，**對外介面必須跟 C++ 版保持一致**（node 名稱、執行檔名、參數名、
topic、訊息型別、QoS、service 名稱）——`syncai_robot_state`、`syncai_backend` 都依賴它。
行為有疑問時以 C++ 版為準。

## 建置與測試

所有東西都在容器 `syncai-ros2-rust` 裡跑（`make up` 啟動；整個 repo 掛在 `/workspace`）。
host 上沒有 ROS / cargo。

```bash
docker exec syncai-ros2-rust bash -lc 'cd /workspace && colcon build --symlink-install --packages-select syncai_driver_manager'
docker exec syncai-ros2-rust bash -lc 'cd /workspace/src/syncai_driver_manager && cargo fmt --check'
docker exec syncai-ros2-rust bash -lc 'cd /workspace/src/syncai_driver_manager && cargo clippy --target-dir /workspace/build/.clippy --all-targets'
docker exec syncai-ros2-rust bash -lc 'cd /workspace/src/syncai_driver_manager && cargo test --target-dir /workspace/build/.clippy'
```

- `cargo` 指令要先 `colcon build` 過一次：`.cargo/config.toml` 是 colcon-ros-cargo 產生的。
- 輸出裡 `/opt/ros2_rust_underlay/...`（rclrs 本身）的 warning 不是我們的，忽略。
- 收尾前 build / clippy / fmt 都要零 warning，unit test 全過。

## 實機安全（重要）

- 這台機器接著實機：控制器在 `192.168.1.120:50051`，telemetry 送到 `192.168.1.103:50010`。
  旁邊還有 `robot01` 等容器在跑 ROS（domain 1）。
- 測試時**絕對不要對 `192.168.1.120` 送指令**（cmd_vel / set_motion_key 會讓機器人動）。用 loopback：
  `-p telemetry_recv_ip:=127.0.0.1 -p command_target_ip:=127.0.0.1`，換掉 port，
  並 `export ROS_DOMAIN_ID=77`（不要用 1 或 2）。
- 只「聽」50010 確認下位機有沒有在送 telemetry 是安全的；但同一時間只能有一個程式 bind 它。

## 架構（`src/syncai_driver_manager/src/driver_manager_node/`）

| 檔案 | 職責 |
| --- | --- |
| `mod.rs` | 組裝：參數 → 兩個 UDP socket → worker / service / telemetry thread |
| `protocol.rs` | 封包 <-> 資料結構的**純函式**，不碰 socket / Node / ROS 訊息型別，unit test 都在這 |
| `parameters.rs` | 速度增益（`MandatoryParameter`，>= 0）、UDP 位址（`ReadOnlyParameter`） |
| `session.rs` | UDP socket（telemetry `listen`、command `connect`） |
| `command.rs` | `CommandLink`（送 ASCII 指令）、`SafetyLock`（目前沒有觸發條件） |
| `telemetry.rs` | 收封包的 `std::thread`，解析後轉 ROS 訊息發佈 |
| `publishers.rs` / `subscriber.rs` / `service.rs` | ROS 介面 |

執行緒：`cmd_vel` 一個 rclrs Worker、四個 service 共用一個 Worker（等同 C++ 版的兩個
MutuallyExclusive callback group，兩個 worker 會並行），telemetry 自己一條 `std::thread`。
新的封包解析邏輯放 `protocol.rs` 並補 unit test。

## rclrs 的坑（都已實際踩過）

- **params YAML 的 key 不展開萬用字元**：只認 `/**` 或 node 完整名稱。`/**/driver_manager`
  會被默默忽略、全部變預設值。這個套件的 YAML 用 `/**:`。
- **Worker = callback group**：`node.create_*` 的 callback 全排在 executor 的單一 thread；
  `worker.create_*` 的 callback 在該 worker 自己的 thread。callback 形式是
  `FnMut(&mut Payload, Msg)`。
- **沒有 SIGINT handler**：Ctrl-C 直接結束程序，`Drop` 不會跑。
  在非互動 shell script 裡用 `&` 背景執行的程序會忽略 SIGINT，要停請送 SIGTERM。
- **`ros-env` 版本**：套件用 `ros-env` 0.2，rclrs 內部用 0.3，兩邊的訊息型別不相容。
  不要用 rclrs 回傳訊息型別的 helper（例如 `Time::to_ros_msg`），自己從 `Time::nsec` 組。
- `Message` trait 在 `rclrs::*` 底下是 `rosidl_runtime_rs::Message`。
- 訊息套件不寫進 `Cargo.toml`，寫在 `package.xml` 的 `<depend>`；漏了 `ros_env::<pkg>` 就不存在。
- `launch/`、`params/` 靠 `Cargo.toml` 的 `[package.metadata.ros] install_to_share` 裝到 share。

## 慣例

- 程式碼與設定檔（Rust、Cargo.toml、Makefile、compose、toml…）的**註解一律用英文**；log 訊息也是英文，並加 `[Module]` 前綴（例如 `[Telemetry]`、`[DriverManagerNode]`）。README / CLAUDE.md 等說明文件用繁體中文。
- 格式依 repo 根目錄的 `rustfmt.toml`（max_width 100）；Cargo.toml 開了 `unsafe_code = "forbid"`
  和 `clippy::all`。
- Commit 用 Conventional Commits（`.github/prompt/copilot-commit-message-instructions.md`），英文。
- `src/syncai_common/` 是 vcstool 拉下來的另一個 repo，不要在這裡 commit 它；
  要改訊息定義去 SyncAI-Robot-Interface 改。
- 已知待確認（留在原始碼 TODO，需要上實機）：`angular.z` 正負號、IMU_RPY 單位、關節順序、
  safety 觸發條件（電量 < 20%、JOINT_TEMP 過熱）。
