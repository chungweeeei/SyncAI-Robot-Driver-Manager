# SyncAI-ROS2-Rust — 常用指令
#
#   make image     建立 Docker image（第一次要跑，約 10-20 分鐘）
#   make up        啟動容器
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

.PHONY: image up down shell build fmt fmt-check lint echo topics clean distclean

image:
	$(COMPOSE) build

up:
	$(COMPOSE) up -d

down:
	$(COMPOSE) down

shell: up
	$(EXEC) bash

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
	$(EXEC) bash -lc "ros2 topic echo /syncai/chatter"

topics: up
	$(EXEC) bash -lc "ros2 topic list"

# build/install/log 是掛載進來的 volume，目錄本身刪不掉，只能清內容
clean: up
	$(EXEC) bash -lc "find /workspace/build /workspace/install /workspace/log -mindepth 1 -delete"

distclean:
	$(COMPOSE) down -v
