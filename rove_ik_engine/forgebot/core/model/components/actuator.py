"""Actuator component: motor/cylinder/transmission attached to a joint."""

from __future__ import annotations

from typing import Literal

from .base import BaseComponent, register_component


ActuatorKind = Literal["motor", "cylinder", "servo", "stepper", "vacuum", "transmission"]


@register_component("actuator")
class ActuatorComponent(BaseComponent):
    kind: ActuatorKind = "motor"
    target_joint: str = ""  # entity id of the joint this drives
    max_force: float = 0.0    # N
    max_torque: float = 0.0   # N·m
    max_velocity: float = 0.0
    gear_ratio: float = 1.0
    driver: str = ""           # free-form: "EtherCAT", "GPIO", "CAN", etc.
