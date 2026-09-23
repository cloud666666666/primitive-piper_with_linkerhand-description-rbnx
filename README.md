# primitive-piper_with_linkerhand-description-rbnx

Body description for the **roboarm** deploy: AgileX Piper (gripper removed) +
Ø39 mm / 17.5 mm coaxial adapter + LinkerHand O6 dexterous hand (right).

Catalog name: `robonix.primitive.piper_with_linkerhand.description`.

This package owns the static + joint-driven TF tree for the robot:

```
base_link → link1 … link6 → rh_adapter_joint → rh_hand_base_link → 11 finger links
```

and is the **only** place the 连接件 (adapter) exists in the system — the arm
driver and the hand driver never model it, they only publish joint feedback.

## Why this package exists

The catalog's `robonix.primitive.agilex.piper.description` publishes
`base_link → … → link6 (→ gripper links)` — a Piper with a gripper. This robot
ends in a dexterous hand bolted on through an adapter, so both the chain and
the mesh set differ. Same reason the IK service and the flat-grasp skill cannot
use the upstream arm model either.

## What it runs

`scripts/start.sh` → `scripts/atlas_register_and_launch.py` (a
`robonix_api.Primitive` under `robonix/primitive/robot_description`) which
spawns:

```
ros2 launch launch/roboarm_urdf.launch.py
```

That launch starts **only** `robot_state_publisher`, subscribing to
`/arm/joint_states_single` (what the arm primitive publishes). No RViz, no
joint_state_publisher (it would fight the real driver's feedback).

No capability is routed: TF in ROS 2 is a global side-channel, so atlas
registration + the shared lifecycle driver is the whole contract surface.

## The URDF is generated — do not edit it

```
src/roboarm_description/urdf/roboarm.urdf     ← generated, committed
urdf/make_urdf.py                             ← the generator
urdf/source/piper_no_gripper.urdf             ← vendored AgileX URDF (inputs)
urdf/source/linkerhand_o6_right.urdf          ← vendored hand URDF
```

```bash
python3 urdf/make_urdf.py            # regenerate
python3 urdf/make_urdf.py --check    # verify the committed file matches the generator
```

`--check` also self-checks the result: one root link, no link with two parents,
the expected link/joint set, and every `package://` mesh present on disk. Point
it at newer sources with `ROBOARM_PIPER_URDF_SRC` / `ROBOARM_HAND_URDF_SRC`.

The 17.5 mm mount offset comes from the MuJoCo scene
(`sim/piper_linker/scene.xml`, `<body name="rh_hand_base_link" pos="0 0
0.0175">`); keeping the URDF and the scene in agreement is what lets grasp
poses be validated in sim.

## Known limitation: finger frames do not move

The hand primitive serves state through the gRPC
`robonix/primitive/hand/get_state` rpc, **not** as a ROS `JointState`, so
`robot_state_publisher` never hears finger motion and the 11 hand links sit at
their URDF zero pose. Nothing in the grasp pipeline needs finger TF (it uses
`link6` and the palm, both rigid relative to the flange). Driving it would mean
republishing hand joint states onto the arm's `joint_states` stream.

## License

MulanPSL-2.0 (`LICENSE`). The meshes under `src/roboarm_description/meshes/`
are vendored from AgileX `piper_description` and the LinkerHand O6 URDF
package; the URDF sources are vendored under `urdf/source/`.
