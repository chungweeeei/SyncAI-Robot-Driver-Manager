# SyncAI-ROS2-Rust

用 [`ros2_rust`](https://github.com/ros2-rust/ros2_rust)（`rclrs`）寫 ROS 2 節點的實驗專案。
內含一組最小的 talker / listener 範例，以及一套 Docker 開發環境。

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

# 3. 編譯 workspace
make build

# 4. 開兩個終端機，一個跑 publisher、一個跑 subscriber
make talker
make listener
```

`make listener` 應該會看到：

```
[INFO] [syncai_listener]: [#1] I heard: 'Hello from rclrs! #1'
[INFO] [syncai_listener]: [#2] I heard: 'Hello from rclrs! #2'
```

也可以直接用 ROS 2 的 CLI 工具驗證，不必寫 subscriber：

```bash
make topics                # 列出所有 topic
make echo                  # ros2 topic echo /syncai/chatter
```

## 其他指令

| 指令 | 作用 |
| --- | --- |
| `make shell` | 進到容器裡的 bash，環境都已 source 好 |
| `make down` | 停止容器 |
| `make clean` | 清掉 `build/` `install/` `log/` |
| `make distclean` | 連 named volume（cargo cache、編譯產物）一起刪掉 |

## 專案結構

```
.
├── docker/
│   ├── Dockerfile          # ROS 2 + Rust + rclrs 相依環境
│   └── entrypoint.sh       # 依序 source：ROS 2 → underlay → workspace
├── docker-compose.yml      # 掛載 src/、cargo 與 colcon 的 cache volume
├── Makefile                # 常用指令包裝
└── src/
    └── syncai_rust_demo/   # ROS 2 套件（build_type: ament_cargo）
        ├── Cargo.toml
        ├── package.xml
        └── src/
            ├── talker.rs   # 每秒發佈一則 std_msgs/String
            └── listener.rs # 訂閱並印出
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

* 目前 talker / listener 都跑在同一個容器內。若之後要拆成多個容器，
  它們必須共用同一個 Docker network，且 `ROS_DOMAIN_ID` 要一致。
* 沒有 GUI（RViz / rqt）。要用的話得另外設定 X11 forwarding（macOS 需搭配 XQuartz）。

## 參考

* [ros2_rust](https://github.com/ros2-rust/ros2_rust)
* [rclrs 範例](https://github.com/ros2-rust/examples/tree/main/rclrs)
* [rclrs docs.rs](https://docs.rs/rclrs)
