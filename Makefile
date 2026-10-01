# SyncAI-ROS2-Rust — 常用指令
#
#   make image     建立 Docker image（第一次要跑，約 10-20 分鐘）
#   make up        啟動容器
#   make interface 拉共用訊息套件 syncai_common 進 src/（第一次 clone 之後要跑）
#   make build     在容器內用 colcon 編譯 workspace
#   make fmt       用 rustfmt 格式化所有 Rust 套件
#   make fmt-check 只檢查格式、不改檔（CI 用）
#   make lint      用 clippy 檢查所有 Rust 套件（要先 make build 過一次）
#   make shell     進入容器的互動式 shell
#   make down      停止並移除容器
#   make clean     清掉編譯產物

COMPOSE := docker compose
SERVICE := ros2-rust
EXEC    := $(COMPOSE) exec $(SERVICE)

.PHONY: image up down shell interface interface-update build fmt fmt-check lint echo topics clean distclean

image:
	$(COMPOSE) build

up:
	$(COMPOSE) up -d

down:
	$(COMPOSE) down

shell: up
	$(EXEC) bash

# 共用訊息套件 syncai_common（SyncAI-Robot-Interface），見 interface.repos。
# 用容器內的 vcstool，host 不用另外裝。interface.repos 用 stdin 餵進去，這樣不管容器掛了
# 哪些目錄都能用（exec -T 才不會配 TTY，管線才成立）。
# 寫進 /workspace/src 等於寫進 host 的 ./src，import 完直接 make build 就會一起編。
interface: up
	$(COMPOSE) exec -T $(SERVICE) bash -lc "cd /workspace && vcs import" < interface.repos

# 改了 interface.repos 的 version（或想丟掉本機改動）時用；--force 會重新 checkout
interface-update: up
	$(COMPOSE) exec -T $(SERVICE) bash -lc "cd /workspace && vcs import --force" < interface.repos

build: up
	$(EXEC) bash -lc "colcon build --symlink-install"

# src/ 底下每個 Cargo 套件（colcon 套件不在同一個 cargo workspace，要逐一跑）
MANIFESTS := find src -name Cargo.toml -not -path '*/target/*'

fmt: up
	$(EXEC) bash -lc "$(MANIFESTS) | xargs -r -n1 cargo fmt --manifest-path"

fmt-check: up
	$(EXEC) bash -lc "$(MANIFESTS) | xargs -r -n1 cargo fmt --check --manifest-path"

# 靠 colcon build 產生的 .cargo/config.toml 把 rclrs / 訊息 crate 指到 underlay
lint: up
	$(EXEC) bash -lc "$(MANIFESTS) | xargs -r -n1 cargo clippy --target-dir build/.clippy --all-targets --manifest-path"

echo: up
	$(EXEC) bash -lc "ros2 topic echo /chatter"

topics: up
	$(EXEC) bash -lc "ros2 topic list"

# build/install/log 在 repo 目錄裡（掛載進容器），用容器內的 ros 使用者清，避免權限問題
clean: up
	$(EXEC) bash -lc "find /workspace/build /workspace/install /workspace/log -mindepth 1 -delete"

distclean:
	$(COMPOSE) down -v
