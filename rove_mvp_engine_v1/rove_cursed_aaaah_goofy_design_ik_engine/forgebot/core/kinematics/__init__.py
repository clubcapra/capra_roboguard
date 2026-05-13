"""ForgeBOT kinematics — public package boundary.

This module is the embed point for inverse kinematics on a robot. It is
deliberately self-contained: depends on numpy and `forgebot.core.model`
(Pydantic data classes), not on FastAPI, the editor frontend, or any
I/O code. Any consumer with a `Project` object (from a `.forgebot` file
or built directly in Python) can call into this module to run IK.

Headless usage:

    from forgebot.io.serializer.forgebot_file import load
    from forgebot.core.kinematics import solve_position_ik

    p = load("/path/to/robot.forgebot")
    r = solve_position_ik(
        p, base="ent_link_core", tip="ent_link_gripper",
        target_world=(0.5, 0.0, 0.3),
        target_rotation=(0.0, 0.0, 0.0, 1.0),
        initial_joint_values={},
        mode="pos_primary",
    )
    print(r.joint_values, r.pos_residual, r.rot_residual)

CLI smoke test (run IK once from the shell):

    forgebot ik solve PROJECT.forgebot --base BASE_ID --tip TIP_ID \\
            --target X Y Z

The editor (FastAPI + WS) wraps this module in
`forgebot.api.routes.kinematics`. A robot's onboard control software
should import from here directly and skip the API layer.
"""

from .chain import KinematicChain, extract_chain
from .forward import world_transforms
from .inverse import IKResult, solve_position_ik
from .transforms import (
    axis_angle_to_quat,
    from_position_quat,
    identity,
    joint_offset_transform,
)
from .tune import (
    TuneScore,
    TuneStatus,
    build_scenarios,
    candidate_grid,
    evaluate_profile,
    run_tune,
)

__all__ = [
    "IKResult",
    "KinematicChain",
    "TuneScore",
    "TuneStatus",
    "axis_angle_to_quat",
    "build_scenarios",
    "candidate_grid",
    "evaluate_profile",
    "extract_chain",
    "from_position_quat",
    "identity",
    "joint_offset_transform",
    "run_tune",
    "solve_position_ik",
    "world_transforms",
]
