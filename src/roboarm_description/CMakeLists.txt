cmake_minimum_required(VERSION 3.5)
project(roboarm_description)

find_package(ament_cmake REQUIRED)

# Install-only ament package: no compiled targets, just the URDF + meshes
# that `package://roboarm_description/...` in roboarm.urdf resolves against.
#
# Only the two directories the deploy needs are listed:
#   * urdf/    — roboarm.urdf (plus its generator at the rbnx package root)
#   * meshes/  — STL files, vendored from AgileX piper_description + the
#                LinkerHand O6 URDF package, split into piper/ and hand/
# Listing a directory that does not exist makes ament_cmake_symlink_install
# fail with "can't find DIRECTORY", so a partial checkout fails loudly here
# instead of silently shipping a URDF whose meshes 404 at RViz time.
install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
