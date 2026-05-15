"""Unit system for ForgeBOT projects.

All in-memory values are stored in SI units (meters, radians, kilograms).
Importers convert from foreign units on load; exporters convert on save.
The `Units` model only declares what units the *file* used — the runtime
model is always SI.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel

LengthUnit = Literal["meters", "millimeters", "centimeters", "inches", "feet"]
AngleUnit = Literal["radians", "degrees"]
MassUnit = Literal["kilograms", "grams", "pounds"]


_LENGTH_TO_M: dict[str, float] = {
    "meters": 1.0,
    "millimeters": 0.001,
    "centimeters": 0.01,
    "inches": 0.0254,
    "feet": 0.3048,
}

_MASS_TO_KG: dict[str, float] = {
    "kilograms": 1.0,
    "grams": 0.001,
    "pounds": 0.45359237,
}


class Units(BaseModel):
    """Declared units for a project file."""

    length: LengthUnit = "meters"
    angle: AngleUnit = "radians"
    mass: MassUnit = "kilograms"

    def length_to_meters(self, value: float) -> float:
        return value * _LENGTH_TO_M[self.length]

    def angle_to_radians(self, value: float) -> float:
        if self.angle == "degrees":
            from math import pi
            return value * pi / 180.0
        return value

    def mass_to_kilograms(self, value: float) -> float:
        return value * _MASS_TO_KG[self.mass]
