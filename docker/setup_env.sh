# 依序 source：ROS 2 本體 -> Rust underlay -> 使用者 workspace
# 這個檔案同時被 entrypoint 與互動式 shell 引用，所以不要用 `set -e`。

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash" --
fi

if [ -f "${ROS2_RUST_UNDERLAY}/install/setup.bash" ]; then
  source "${ROS2_RUST_UNDERLAY}/install/setup.bash" --
fi

# 使用者的 workspace 還沒 build 過時不存在，屬正常情況
if [ -f "/workspace/install/setup.bash" ]; then
  source "/workspace/install/setup.bash" --
fi
