# SyncAI-ROS2-Rust — 常用指令
#
#   make image     建立 Docker image（第一次要跑，約 10-20 分鐘）
#   make up        啟動容器
#   make build     在容器內用 colcon 編譯 workspace
#   make talker    執行 publisher
#   make listener  執行 subscriber
#   make shell     進入容器的互動式 shell
#   make down      停止並移除容器
#   make clean     清掉編譯產物

COMPOSE := docker compose
SERVICE := ros2-rust
EXEC    := $(COMPOSE) exec $(SERVICE)

.PHONY: image up down shell build talker listener echo topics clean distclean

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

talker: up
	$(EXEC) bash -lc "ros2 run syncai_rust_demo talker"

listener: up
	$(EXEC) bash -lc "ros2 run syncai_rust_demo listener"

echo: up
	$(EXEC) bash -lc "ros2 topic echo /syncai/chatter"

topics: up
	$(EXEC) bash -lc "ros2 topic list"

# build/install/log 是掛載進來的 volume，目錄本身刪不掉，只能清內容
clean: up
	$(EXEC) bash -lc "find /workspace/build /workspace/install /workspace/log -mindepth 1 -delete"

distclean:
	$(COMPOSE) down -v
