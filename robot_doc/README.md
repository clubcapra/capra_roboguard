# Robot models

Robot geometry lives here, with each URDF beside its meshes and supporting data:

- `rove_caged_urdf/`: caged rover, used by the simulator and default IK engine.
- `rove_standard_urdf/`: arm-equipped rover, Robotiq gripper, and saved arm poses.
- `ik_legacy/robot.urdf`: the IK engine's original exported fallback model.
  Its meshes remain in `rove_ik_engine/data/meshes`; the URDF references them
  relative to its own location. The preferred arm export is still
  `rove_ik_engine/data/scene.forgebot`.

The simulator profiles and IK TOML configs resolve model paths relative to
their own files. Joint/link names and geometry are unchanged by relocation.
The duplicate caged URDF in the IK data directory has been consolidated here.
Simulation caches are regenerated locally and are not tracked.
