#!/usr/bin/env bash
# Put the messages-only livox_ros_driver2 (see package.xml here) into a colcon workspace:
# this directory's package.xml + CMakeLists.txt, plus the upstream .msg files of LIVOX_VERSION.
#
#   .devcontainer/livox_msgs/import_livox_msgs.sh /workspace
#
# Skipped when the workspace already has a livox_ros_driver2 (like `vcs import --skip-existing`).
# Keep LIVOX_VERSION equal to SyncAI-Robot-Workspace's third-party.repos pin, so the generated
# type matches the one the robot's driver publishes.
set -euo pipefail

LIVOX_VERSION=1.2.6
workspace=${1:-/workspace}
dest="${workspace}/src/livox_ros_driver2"
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [ -e "${dest}/package.xml" ]; then
    echo "livox_ros_driver2 already in ${dest}, skipping"
    exit 0
fi

mkdir -p "${dest}/msg"
cp "${here}/package.xml" "${here}/CMakeLists.txt" "${dest}/"
for msg in CustomMsg CustomPoint; do
    curl -fsSL -o "${dest}/msg/${msg}.msg" \
        "https://raw.githubusercontent.com/Livox-SDK/livox_ros_driver2/${LIVOX_VERSION}/msg/${msg}.msg"
done
echo "livox_ros_driver2 ${LIVOX_VERSION} messages imported into ${dest}"
