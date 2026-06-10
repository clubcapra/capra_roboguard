"""FrictionField: a raster of per-cell ground friction, painted like an image.

A grid over world XY holds a longitudinal friction coefficient (mu_long) per
cell. The brush-track actuator (actuation/tracks.py) looks up mu per ground
CONTACT by its world XY and scales its traction cap, so the robot slips on a
painted ice patch and grips on gravel -- emergent, per-contact, anisotropic.
Unpainted cells hold the field default (= the nominal floor friction), so an
empty field changes nothing.

Materials are (mu_long, mu_lat) presets; the grid stores mu_long (the actuator
scales lateral proportionally). Save/load is plain JSON per world.
"""
from __future__ import annotations

import json

import numpy as np

# name -> (mu_long, mu_lat). Nominal hardwood floor ~ (0.6, 0.15).
MATERIALS = {
    "nominal":  (0.60, 0.15),
    "hardwood": (0.60, 0.15),
    "gravel":   (0.75, 0.40),
    "mud":      (0.40, 0.30),
    "sand":     (0.45, 0.28),
    "ice":      (0.10, 0.05),
    "rock_wet": (0.20, 0.10),
}


class FrictionField:
    def __init__(self, origin=(-25.0, -25.0), extent=(50.0, 50.0),
                 cell=0.25, default=0.6):
        self.x0, self.y0 = float(origin[0]), float(origin[1])
        self.cell = float(cell)
        self.default = float(default)
        self.nx = max(1, int(round(extent[0] / self.cell)))
        self.ny = max(1, int(round(extent[1] / self.cell)))
        self.mu = np.full((self.ny, self.nx), self.default, np.float32)

    # -- indexing -----------------------------------------------------------
    def _ij(self, x: float, y: float):
        return int((x - self.x0) // self.cell), int((y - self.y0) // self.cell)

    def cell_center(self, i: int, j: int):
        return self.x0 + (i + 0.5) * self.cell, self.y0 + (j + 0.5) * self.cell

    def lookup(self, x: float, y: float) -> float:
        i, j = self._ij(x, y)
        if 0 <= i < self.nx and 0 <= j < self.ny:
            return float(self.mu[j, i])
        return self.default                  # outside the painted region = nominal

    # -- painting -----------------------------------------------------------
    def paint(self, x: float, y: float, radius: float, mu_long: float) -> None:
        r = int(radius / self.cell) + 1
        ci, cj = self._ij(x, y)
        r2 = radius * radius
        for j in range(max(0, cj - r), min(self.ny, cj + r + 1)):
            for i in range(max(0, ci - r), min(self.nx, ci + r + 1)):
                wx, wy = self.cell_center(i, j)
                if (wx - x) ** 2 + (wy - y) ** 2 <= r2:
                    self.mu[j, i] = mu_long

    def paint_material(self, x: float, y: float, radius: float, name: str) -> None:
        self.paint(x, y, radius, MATERIALS.get(name, (self.default, 0))[0])

    # -- persistence --------------------------------------------------------
    def to_dict(self) -> dict:
        return {"origin": [self.x0, self.y0], "cell": self.cell,
                "default": self.default, "nx": self.nx, "ny": self.ny,
                "mu": self.mu.tolist()}

    @classmethod
    def from_dict(cls, d: dict) -> "FrictionField":
        fld = cls(origin=d["origin"], cell=d["cell"], default=d["default"],
                  extent=(d["nx"] * d["cell"], d["ny"] * d["cell"]))
        fld.mu = np.array(d["mu"], np.float32)
        return fld

    def load_dict(self, d: dict) -> None:
        """Restore field contents IN PLACE (keeps actuator references valid)."""
        self.x0, self.y0 = float(d["origin"][0]), float(d["origin"][1])
        self.cell = float(d["cell"]); self.default = float(d["default"])
        self.nx, self.ny = int(d["nx"]), int(d["ny"])
        self.mu = np.array(d["mu"], np.float32)

    def save(self, path: str) -> None:
        with open(path, "w") as f:
            json.dump(self.to_dict(), f)

    @classmethod
    def load(cls, path: str) -> "FrictionField":
        with open(path) as f:
            return cls.from_dict(json.load(f))
