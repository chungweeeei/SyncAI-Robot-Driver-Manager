#!/usr/bin/env bash
# Put the messages-only drdds (see package.xml here) into a colcon workspace: this directory's
# package.xml + CMakeLists.txt, plus the upstream .msg files of SDK_DEPLOY_VERSION.
#
#   .devcontainer/drdds_msgs/import_drdds_msgs.sh /workspace
#
# Skipped when the workspace already has a drdds (like `vcs import --skip-existing`). Keep
# SDK_DEPLOY_VERSION equal to SyncAI-Robot-Workspace's third-party-sparse.repos pin, so the
# generated type matches the one the M20 publishes (Humble matches types by name only: a
# changed definition decodes as garbage, not as an error).
set -euo pipefail

SDK_DEPLOY_VERSION=20279073bc43522aa06b0abf611bb2daaa53d87d
workspace=${1:-/workspace}
dest="${workspace}/src/drdds"
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

if [ -e "${dest}/package.xml" ]; then
    echo "drdds already in ${dest}, skipping"
    exit 0
fi

mkdir -p "${dest}/msg"
cp "${here}/package.xml" "${here}/CMakeLists.txt" "${dest}/"
for msg in MetaType BatteryDataValue BatteryData; do
    curl -fsSL -o "${dest}/msg/${msg}.msg" \
        "https://raw.githubusercontent.com/DeepRoboticsLab/sdk_deploy/${SDK_DEPLOY_VERSION}/src/drdds/msg/${msg}.msg"
done
echo "drdds ${SDK_DEPLOY_VERSION} messages imported into ${dest}"
