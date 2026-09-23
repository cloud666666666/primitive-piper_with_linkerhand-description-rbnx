# SPDX-License-Identifier: MulanPSL-2.0
"""URDF + robot_state_publisher launch for the Piper + adapter + LinkerHand O6 body.

This is the only thing this primitive starts. It is deliberately bare:

    * loads <urdf_path> with `cat` — the URDF is pre-expanded by
      urdf/make_urdf.py, so there is no xacro pass.
    * remaps robot_state_publisher's `joint_states` subscription to
      /arm/joint_states_single (what piper_ctl publishes; the `/arm`
      namespace is baked into the arm driver's launch).
    * starts no joint_state_publisher / _gui — they publish a fake
      all-zero state that would fight the real driver's feedback.
    * starts no RViz — rbnx boot is headless.

Frame contract published here:

    base_link            ← arm mechanical root (not prefixed)
    link1 … link6        ← driven by /arm/joint_states_single (joint1..joint6)
    rh_adapter_joint     ← FIXED: the Ø39 mm / 17.5 mm coaxial adapter on the
                           link6 flange (latched on /tf_static)
    rh_hand_base_link    ← LinkerHand O6 RIGHT hand root, then 11 finger links

The 11 finger links are FIXED AT THEIR URDF ZERO POSE: the hand primitive
serves state over the gRPC `get_state` rpc, not as a ROS JointState, so RSP
never hears finger motion. Nothing in the grasp pipeline needs finger TF —
it uses link6 and the palm, both rigid relative to the flange.

Camera hand-eye is NOT published here: service-piper_with_linkerhand-grasp-pose-rbnx maps image
pixels to arm/base_link XY through a calibrated 2D homography.

Launch arguments:
    urdf_path           absolute path to roboarm.urdf. Defaults to env
                        ROBOARM_URDF_PATH if set (scripts/atlas_register_and_launch.py
                        exports it), else the vendored location.
    joint_states_topic  topic RSP subscribes to, default /arm/joint_states_single.
"""
from __future__ import annotations

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_urdf_path() -> str:
    """Resolve the vendored URDF path.

    Priority:
      1. ROBOARM_URDF_PATH (exported by scripts/atlas_register_and_launch.py,
         which is also where the file is existence-checked)
      2. <pkg_root>/src/roboarm_description/urdf/roboarm.urdf
    """
    env = os.environ.get("ROBOARM_URDF_PATH", "").strip()
    if env:
        return env
    pkg_root = Path(__file__).resolve().parent.parent
    return str(pkg_root / "src" / "roboarm_description" / "urdf" / "roboarm.urdf")


def generate_launch_description() -> LaunchDescription:
    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=_default_urdf_path(),
        description="Absolute path to roboarm.urdf",
    )
    js_arg = DeclareLaunchArgument(
        "joint_states_topic",
        default_value="/arm/joint_states_single",
        description="Topic robot_state_publisher subscribes to",
    )

    robot_description = ParameterValue(
        Command(["cat ", LaunchConfiguration("urdf_path")]),
        value_type=str,
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="roboarm_robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        # Without this remap RSP subscribes to the global `joint_states`,
        # which nobody publishes here — TF would stay frozen at zero.
        remappings=[
            ("joint_states", LaunchConfiguration("joint_states_topic")),
        ],
    )

    return LaunchDescription([urdf_arg, js_arg, rsp_node])
