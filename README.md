# SyncAI-ROS2-Rust

用 [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)（`rclrs`）寫 ROS 2 節點的實驗專案。
內含 `syncai_driver_manager` 節點，以及一套 Docker 開發環境。

## 為什麼用 Docker

ROS 2 沒有正式支援 macOS（官方只提供 Ubuntu / Windows 的 binary），
在 macOS 原生編譯 ROS 2 + `rclrs` 很容易卡在相依套件上。
所以這裡把整套環境包進 Linux container：**程式碼在 host 上編輯，編譯與執行都在容器內**。

容器裡準備好的東西：

| 項目 | 說明 |
| --- | --- |
| ROS 2 Humble | LTS 版本（支援到 2027-05），`ros-base`，底層是 Ubuntu 22.04 |
| Rust 1.85 | `rclrs` 要求的最低版本 |
| `colcon-cargo` / `colcon-ros-cargo` | 讓 `colcon` 認得 `ament_cargo` 型別的套件 |
| `rosidl_rust` | 把 `.msg` 轉成 Rust 型別的產生器 |
| 非 root 使用者 `ros` | uid/gid 1000，可 `sudo` 免密碼；避免掛載進來的檔案被寫成 root 所有 |
| `rclrs`（原始碼） | crates.io 上的 `rclrs` 0.7.0 相依 `rosidl_runtime_rs` ^0.6，但 main 分支的產生器產出的是 0.7，混用會編不過，所以 `rclrs` 也從原始碼編 |
| 重建過的訊息套件 | apt 版訊息套件沒有 Rust 綁定，所以 `std_msgs`、`example_interfaces` 等從原始碼重編一次 |

以上都放在 image 內的 underlay workspace `/opt/ros2_rust_underlay`，進 container 時會自動 source。

## 快速開始

```bash
# 1. 建立 image（第一次會比較久，要從原始碼編訊息套件）
make image

# 2. 啟動容器
make up

# 3. 拉共用訊息套件 syncai_common 進 src/（見「共用訊息套件」一節）
make interface

# 4. 編譯 workspace
make build

# 5. 進容器執行節點（每秒發佈一則 std_msgs/String 到 /chatter）
make shell
ros2 run syncai_driver_manager syncai_driver_manager
```

另開一個終端機，用 ROS 2 的 CLI 工具驗證：

```bash
make topics                # 列出所有 topic
make echo                  # ros2 topic echo /chatter
```

`make echo` 應該會看到：

```
data: 'hi #1'
---
data: 'hi #2'
---
```

## 用 VS Code Dev Container 開發

除了 `make`，也可以直接讓 VS Code 進到容器裡開發，rust-analyzer、除錯器等都在容器內跑。

1. 安裝 VS Code 的 [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) 擴充套件
2. 用 VS Code 開啟這個 repo，執行 **Dev Containers: Reopen in Container**
3. 第一次會 build image（跟 `make image` 是同一個 image），之後就秒開
4. 在 VS Code 的終端機裡直接 `colcon build --symlink-install`、`ros2 run syncai_driver_manager syncai_driver_manager`

設定在 `.devcontainer/`：

| 檔案 | 作用 |
| --- | --- |
| `devcontainer.json` | 指定 compose service、VS Code 擴充套件與設定 |
| `docker-compose.devcontainer.yml` | 疊在根目錄的 `docker-compose.yml` 上，把**整個 repo** 掛到 `/workspace`，並固定以 `ros`（uid/gid 1000）執行 |

幾點注意：

* Dev Container 的容器名稱是 `syncai-ros2-rust-devcontainer`，跟 `make up` 的容器分開，
  但共用同一個 image、repo 目錄（編譯產物）與 cargo cache volume。
* rust-analyzer 要先 `colcon build` 過一次才會正常：訊息套件的 crate 是由 `colcon-ros-cargo`
  在 build 時接上的。
* 新增 Rust 套件後，要把它的 `Cargo.toml` 加進 `devcontainer.json` 的 `rust-analyzer.linkedProjects`。

## 其他指令

| 指令 | 作用 |
| --- | --- |
| `make shell` | 進到容器裡的 bash，環境都已 source 好 |
| `make interface` | `vcs import < interface.repos`，把 `syncai_common` 拉進 `src/` |
| `make interface-update` | 同上但加 `--force`，改了 `interface.repos` 的 pin 之後用 |
| `make fmt` | 用 rustfmt 格式化所有 Rust 套件 |
| `make fmt-check` | 只檢查格式、不改檔（CI 用） |
| `make lint` | 用 clippy 檢查所有 Rust 套件（要先 `make build` 過一次） |
| `make down` | 停止容器 |
| `make clean` | 清掉 `build/` `install/` `log/` |
| `make distclean` | 停止容器並刪掉 cargo cache 的 named volume |

## 專案結構

```
.
├── docker/
│   ├── Dockerfile          # ROS 2 + Rust + rclrs 相依環境
│   └── entrypoint.sh       # 依序 source：ROS 2 → underlay → workspace
├── docker-compose.yml      # 整個 repo 掛到 /workspace，cargo cache 用 named volume
├── interface.repos         # vcstool 清單：共用訊息套件 syncai_common 從哪裡拉
├── Makefile                # 常用指令包裝
├── rustfmt.toml / clippy.toml / .editorconfig  # 共用的格式與 lint 設定
├── .devcontainer/          # VS Code Dev Container 設定
├── docs/                   # 筆記
└── src/
    ├── syncai_common/          # vcs import 拉下來的共用訊息套件（gitignore，不在這個 repo 裡）
    └── syncai_driver_manager/  # ROS 2 套件（build_type: ament_cargo）
        ├── Cargo.toml
        ├── Cargo.lock
        ├── package.xml
        └── src/
            └── main.rs     # 每秒發佈一則 std_msgs/String 到 /chatter
```

## 程式碼重點

`rclrs` 的寫法跟 `rclcpp` / `rclpy` 有幾個明顯差異：

* **Executor 先於 Node。** 先 `Context::default_from_env()?.create_basic_executor()`，
  再用 executor 建 node，最後 `executor.spin(...)`。
* **Worker 取代自己管 `Arc<Mutex<..>>`。** `node.create_worker::<T>(初始值)` 產生一個持有狀態的 worker，
  它建立的 subscription / timer callback 會拿到 `&mut T`，由 rclrs 保證不會同時被兩個 callback 存取。
* **訊息型別來自 `ros-env`。** `use ros_env::std_msgs::msg::String;`
  ——訊息套件不寫在 `Cargo.toml` 的 `[dependencies]`，而是宣告在 `package.xml` 裡，
  由 `colcon-ros-cargo` 在編譯時接上。

## 共用訊息套件 syncai_common

`msg` / `srv` / `action` 不定義在這個 repo，而是放在
[SyncAI-Robot-Interface](https://github.com/chungweeeei/SyncAI-Robot-Interface)
（colcon 套件名 `syncai_common`），整個 syncai stack 共用一份。
這裡跟 `SyncAI-Robot-Backend`、`SyncAI-Robot-Workspace` 一樣用
[vcstool](https://github.com/dirk-thomas/vcstool) 把它拉進 `src/`，不用 git submodule：

```bash
make interface          # 第一次 clone 之後跑一次，會建立 src/syncai_common/
make interface-update   # 改了 interface.repos 的 version、或想丟掉本機改動時用（--force）
```

幾個要知道的點：

* **`vcstool` 在容器裡，host 不用裝。** image 已經有 `python3-vcstool`；
  `make interface` 把 `interface.repos` 從 stdin 餵給容器內的 `vcs import`，
  所以不依賴容器掛了哪些目錄。
  寫進 `/workspace/src` 等於寫進 host 的 `./src`。
* **`src/syncai_common/` 在 `.gitignore` 裡。** 它是另一個 git repo 的工作目錄：
  訊息要改就在那個 checkout 裡改、在那邊 commit，不要 commit 回這個 repo。
  下一次 `--force` import 會蓋掉沒 commit 的東西。
* **pin 在 `dev` 分支**，跟 backend / workspace 一致（`main` 會落後 `dev`）。
  要換版本就改 `interface.repos` 的 `version`，commit 那一行 diff。

### 在節點裡用這些訊息

`syncai_common` 是標準的 `rosidl` 套件，`make build` 時 underlay 的
`rosidl_generator_rs` 會順便產出 Rust 綁定（`msg` / `srv` / `action` 都有）。
用法跟 `std_msgs` 一樣——**不要寫進 `Cargo.toml`**，而是在套件的 `package.xml` 加：

```xml
<depend>syncai_common</depend>
```

然後在程式裡：

```rust
use ros_env::syncai_common::msg::RobotState;
```

機制是 `ros-env` 的 build script：它掃 `AMENT_PREFIX_PATH` 上每個
`<prefix>/share/<套件>/rust/Cargo.toml`，把有 `[package.metadata.ros-env] include = true`
的全部 `include!` 進 `ros_env` 這個 crate——`syncai_common` 產生出來的 crate 就有這個標記。
而 colcon 編某個套件時給的 `AMENT_PREFIX_PATH` 只包含**那個套件宣告過的相依**，
所以 `package.xml` 漏掉 `<depend>` 的話，`ros_env::syncai_common` 就不存在。

## 加新套件

```bash
mkdir -p src/<新套件>/src
```

`Cargo.toml` 的 package name 要和 `package.xml` 的 `<name>` 一致，
並在 `package.xml` 加上：

```xml
<export>
  <build_type>ament_cargo</build_type>
</export>
```

之後 `make build` 就會一起編。

## 容器內的使用者

容器預設以 `ros`（uid/gid 1000）身分執行，不是 root。這樣 bind mount 進來的 `src/`
即使在容器內被寫入，在 host 上也不會變成 root 所有。需要裝套件時 `sudo` 免密碼。

Rust toolchain 裝在 `/home/ros/.cargo`，cargo 的 registry / git cache 也掛在那裡。

如果你的 host 使用者 uid 不是 1000（Linux 上常見，macOS 不受影響），重建時覆蓋掉：

```bash
USER_UID=$(id -u) USER_GID=$(id -g) make image
```

## 換 ROS 2 版本

預設是 Humble。distro 名稱一路帶到 base image、apt 套件名、`.repos` 檔網址與 setup script，
所以改一個環境變數就能切換：

```bash
ROS_DISTRO=jazzy make image
ROS_DISTRO=jazzy make up
ROS_DISTRO=jazzy make build
```

要永久改掉的話，動 `docker/Dockerfile` 的 `ARG ROS_DISTRO=humble` 即可
（`docker-compose.yml` 會跟著這個環境變數，預設值也是 humble）。

目前 `ros2_rust` 有提供 `.repos` 檔的版本：`humble`、`jazzy`、`kilted`、`lyrical`、`rolling`。

> Humble 跑在 Ubuntu 22.04，那裡的 pip 不認得 `--break-system-packages`；
> Jazzy 之後（24.04）反而必須加。Dockerfile 裡的 pip 步驟已經依 distro 分流處理。

## 已知限制

* 目前所有節點都跑在同一個容器內。若之後要拆成多個容器，
  它們必須共用同一個 Docker network，且 `ROS_DOMAIN_ID` 要一致。
* 沒有 GUI（RViz / rqt）。要用的話得另外設定 X11 forwarding（macOS 需搭配 XQuartz）。

## 參考

* [ros2_rust](https://github.com/ros2-rust/ros2_rust)
* [rclrs 範例](https://github.com/ros2-rust/examples/tree/main/rclrs)
* [rclrs docs.rs](https://docs.rs/rclrs)
