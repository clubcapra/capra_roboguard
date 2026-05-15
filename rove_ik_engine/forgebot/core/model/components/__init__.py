"""Component registry. Importing this module registers all built-ins."""

from __future__ import annotations

from .actuator import ActuatorComponent
from .base import (
    BaseComponent,
    all_component_keys,
    get_component_class,
    parse_component,
    register_component,
)
from .controller import ControllerComponent, ControllerGains
from .conveyor import ConveyorComponent
from .end_effector import EndEffectorComponent
from .joint import JointComponent, JointDynamics, JointLimits, JointType
from .link import Geometry, Inertial, InertiaTensor, LinkComponent
from .metadata import MetadataComponent
from .safety_zone import SafetyZoneComponent
from .sensor import (
    CameraComponent,
    ForceTorqueComponent,
    IMUComponent,
    LidarComponent,
    SensorComponent,
)
from .signal_port import SignalPortComponent
from .transform import TransformComponent

__all__ = [
    "ActuatorComponent",
    "BaseComponent",
    "CameraComponent",
    "ControllerComponent",
    "ControllerGains",
    "ConveyorComponent",
    "EndEffectorComponent",
    "ForceTorqueComponent",
    "Geometry",
    "IMUComponent",
    "Inertial",
    "InertiaTensor",
    "JointComponent",
    "JointDynamics",
    "JointLimits",
    "JointType",
    "LidarComponent",
    "LinkComponent",
    "MetadataComponent",
    "SafetyZoneComponent",
    "SensorComponent",
    "SignalPortComponent",
    "TransformComponent",
    "all_component_keys",
    "get_component_class",
    "parse_component",
    "register_component",
]
