#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Build phase: rbnx codegen + colcon-build the vendored roboarm_description.
#
# Why both:
#   * codegen produces atlas_pb2 / atlas_pb2_grpc Python stubs so
#     scripts/atlas_register_and_launch.py can RegisterPrimitive on
#     atlas (which is what unblocks rbnx boot's wait_for_registration
#     loop). Same as ranger_description_rbnx.
#   * colcon-build roboarm_description installs the URDF + mesh STLs
#     under share/roboarm_description/, which the URDF's mesh
#     `package://roboarm_description/meshes/...` references resolve
#     against. Strictly speaking robot_state_publisher only needs
#     the joint geometry to compute TFs (mesh missing == warnings,
#     not failure), but the build is fast (ament_cmake, install-only,
#     ~1s) and downstream consumers like RViz / MoveIt do need the
#     mesh resolution to render correctly.
#
# Output layout:
#   rbnx-build/codegen/                   atlas_pb2 stubs
#   rbnx-build/ws/install/roboarm_description/share/roboarm_description/
#                                         URDF + meshes (ament index'd)
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[roboarm_description/build] clean: removing rbnx-build/"
    rm -rf rbnx-build
fi
mkdir -p rbnx-build/ws/src rbnx-build/data

# Symlink the vendored roboarm_description into rbnx-build/ws/src/ so
# colcon picks it up. Symlink (not copy) keeps edits to src/ live.
ln -snf "$PKG/src/roboarm_description" "$PKG/rbnx-build/ws/src/roboarm_description"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

echo "[roboarm_description/build] colcon build (roboarm_description, install-only)"
cd "$PKG/rbnx-build/ws"
# install-only build — roboarm_description has no compiled targets.
colcon build --symlink-install \
    --packages-select roboarm_description \
    --event-handlers console_direct+ \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
cd "$PKG"

FLAGS=(--out-dir "$PKG/rbnx-build/codegen")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[roboarm_description/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

touch "$PKG/rbnx-build/.rbnx-built"
echo "[roboarm_description/build] done."
