# ROS 2 + ros2_rust (rclrs) 開發環境
#
# 這個 image 內含：
#   * ROS 2 (預設 Humble, LTS)
#   * Rust toolchain + colcon 的 Rust 外掛 (colcon-cargo / colcon-ros-cargo)
#   * 一個 underlay workspace，裡面放 rclrs、rosidl_rust (Rust 訊息產生器)
#     以及重新編譯過、帶有 Rust 支援的標準訊息套件
#   * 一個非 root 的使用者 (預設 uid/gid 1000)，避免掛載進來的檔案被寫成 root 所有
#
# 使用者自己的程式碼不放在 image 裡，而是以 volume 掛載到 /workspace。
ARG ROS_DISTRO=humble
FROM ros:humble-ros-base
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive

# --- 系統相依套件 -----------------------------------------------------------
# libclang-dev: rclrs 用 bindgen 綁定 rcl 的 C API 時需要
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        clang \
        cmake \
        curl \
        git \
        libclang-dev \
        python3-pip \
        python3-vcstool \
        sudo \
        tmux \
        vim \
        ros-${ROS_DISTRO}-example-interfaces \
        ros-${ROS_DISTRO}-test-msgs \
    && rm -rf /var/lib/apt/lists/*

# --- colcon 的 Rust 外掛（裝在系統層，所有使用者共用）----------------------
# Humble 跑在 Ubuntu 22.04，pip 較舊且不是 externally-managed，所以不能加
# --break-system-packages；Jazzy 之後（24.04）則必須加。
RUN if [ "${ROS_DISTRO}" = "humble" ]; then \
        pip install --no-cache-dir --upgrade pytest \
        && pip install --no-cache-dir \
            git+https://github.com/colcon/colcon-cargo.git \
            git+https://github.com/colcon/colcon-ros-cargo.git; \
    else \
        pip install --break-system-packages --no-cache-dir pytest \
            git+https://github.com/colcon/colcon-ros-cargo.git \
            git+https://github.com/colcon/colcon-cargo.git; \
    fi

# --- 非 root 使用者 ---------------------------------------------------------
# uid/gid 對齊 host 上一般 Linux 使用者的 1000，這樣 bind mount 進來的原始碼
# 讀寫權限才會正確。macOS 的 Docker Desktop 會自動轉換擁有者，不受影響。
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# 這些目錄之後會被 named volume 蓋掉。先在 image 裡建好並設定好擁有者，
# Docker 建立 volume 時就會沿用這個擁有者，不然 volume 會是 root 所有。
ENV ROS2_RUST_UNDERLAY=/opt/ros2_rust_underlay
RUN mkdir -p /workspace/src /workspace/build /workspace/install /workspace/log \
             /home/${USERNAME}/.cargo/registry /home/${USERNAME}/.cargo/git \
             ${ROS2_RUST_UNDERLAY} \
    && chown -R ${USER_UID}:${USER_GID} /workspace /home/${USERNAME} ${ROS2_RUST_UNDERLAY}

USER ${USERNAME}
ENV HOME=/home/${USERNAME}

# --- Rust（裝在使用者家目錄）-----------------------------------------------
ARG RUST_VERSION=1.85.0
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
      | sh -s -- --default-toolchain ${RUST_VERSION} -y --no-modify-path --component rustfmt --component clippy
ENV PATH=${HOME}/.cargo/bin:${PATH}

# --- underlay: rclrs + Rust 訊息產生器 + 重新編譯的訊息套件 -----------------
# apt 版的訊息套件沒有 Rust 綁定，所以要從原始碼重建一次。
# rclrs 本身也從原始碼編：crates.io 上的 rclrs 0.7.0 相依 rosidl_runtime_rs ^0.6，
# 但 main 分支的訊息產生器產出的是 0.7，混用會出現「兩個版本的 rosidl_runtime_rs」錯誤。
RUN mkdir -p ${ROS2_RUST_UNDERLAY}/src \
    && cd ${ROS2_RUST_UNDERLAY} \
    && curl -fsSL -o ros2_rust.repos \
        https://raw.githubusercontent.com/ros2-rust/ros2_rust/main/ros2_rust_${ROS_DISTRO}.repos \
    && vcs import src < ros2_rust.repos \
    && rm -rf src/ros2-rust/examples \
    && git clone --depth 1 https://github.com/ros2-rust/ros2_rust.git src/ros2-rust/ros2_rust

RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cd ${ROS2_RUST_UNDERLAY} \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf ${ROS2_RUST_UNDERLAY}/build ${ROS2_RUST_UNDERLAY}/log

# --- 環境設定與進入點 -------------------------------------------------------
# setup_env.sh 由三個地方引用，確保不論是 `docker run`、`docker compose exec`
# 還是互動式 shell，ROS 2 與 underlay 都已經 source 好。
USER root
COPY docker/setup_env.sh /opt/ros2_rust_setup.sh
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh \
    && printf '. /opt/ros2_rust_setup.sh\n' > /etc/profile.d/10-ros2-rust.sh \
    && printf '. /opt/ros2_rust_setup.sh\n' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}

# --- Claude Code（選用，Dev Container 會開啟）-------------------------------
# 放在最後面，開關它不會讓前面編 underlay 的 layer 失效。
# 設定與登入資訊放在 CLAUDE_CONFIG_DIR，Dev Container 會掛 named volume 保存；
# 先建好目錄並設定擁有者，volume 才不會是 root 所有。
ARG INSTALL_CLAUDE_CODE=false
ENV CLAUDE_CONFIG_DIR=${HOME}/.claude
ENV PATH=${HOME}/.local/bin:${PATH}
RUN mkdir -p ${CLAUDE_CONFIG_DIR} \
    && if [ "${INSTALL_CLAUDE_CODE}" = "true" ]; then \
        curl -fsSL https://claude.ai/install.sh | bash; \
    fi

WORKDIR /workspace
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]
