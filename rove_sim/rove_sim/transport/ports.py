"""Shared sim ⇄ HALo port map.

Reads the SAME `ports.toml` the Rust `HALo` reads (its
`config/ports.toml`), so the two processes can never drift into a port overlap.
In HALo-backend mode the sim publishes telemetry on the *backend*
ports (default 6000+); the Rust mock drivers subscribe there and re-serve on the
canonical 5000+ ports to autonomy.

The default location mirrors the repo layout: `HALo/config/ports.toml`
sits a couple levels up from `rove_sim/`. Override with an explicit path.
"""
from __future__ import annotations

import os
import tomllib
from typing import Dict, Optional

# Telemetry channels in a fixed order (index i -> backend_base + 2*i). This MUST
# match the Rust CHANNEL_ORDER so backend ports line up. 8 ODrives: drums 31-34
# (tracks) + flippers 41-44.
CHANNEL_ORDER = ["vectornav", "kinova", "robotiq",
                 "odrive_31", "odrive_32", "odrive_33", "odrive_34",
                 "odrive_41", "odrive_42", "odrive_43", "odrive_44", "pmic", "livox_40", "livox_41", "radiacode"]

# Override keys in ports.toml use the HALo *served* ids; map the sim
# channel name to that id where they differ.
_SERVED_ID = {"vectornav": "vectornav_sim", "kinova": "kinova_arm",
              "robotiq": "robotiq_gripper", "radiacode": "radiacode_arm"}

_DEFAULTS = {"served_base": 5000, "backend_base": 6000,
             "control_port": 5100, "livox_imu_port": 56401}


def _default_ports_path() -> str:
    here = os.path.dirname(os.path.abspath(__file__))            # .../rove_sim/rove_sim/transport
    root = os.path.dirname(os.path.dirname(os.path.dirname(here)))  # .../capra_roboguard
    return os.path.join(root, "HALo", "config", "ports.toml")


class PortMap:
    """Parsed ports.toml with the same fallback rules as the Rust SimConfig."""

    def __init__(self, data: Optional[dict] = None):
        d = data or {}
        self.served_base = int(d.get("served_base", _DEFAULTS["served_base"]))
        self.backend_base = int(d.get("backend_base", _DEFAULTS["backend_base"]))
        self.control_port = int(d.get("control_port", _DEFAULTS["control_port"]))
        self.livox_imu_port = int(d.get("livox_imu_port", _DEFAULTS["livox_imu_port"]))
        self.overrides = dict(d.get("overrides", {}) or {})

    @classmethod
    def load(cls, path: Optional[str] = None) -> "PortMap":
        path = path or _default_ports_path()
        try:
            with open(path, "rb") as f:
                return cls(tomllib.load(f))
        except (FileNotFoundError, tomllib.TOMLDecodeError):
            return cls(None)

    def _override(self, channel: str, key: str) -> Optional[int]:
        served_id = _SERVED_ID.get(channel, channel)
        o = self.overrides.get(served_id) or self.overrides.get(channel)
        if isinstance(o, dict) and key in o:
            return int(o[key])
        return None

    def backend_telemetry_ports(self) -> Dict[str, int]:
        """channel -> backend UDP port the sim publishes telemetry on."""
        out: Dict[str, int] = {}
        for i, ch in enumerate(CHANNEL_ORDER):
            out[ch] = self._override(ch, "backend") or (self.backend_base + 2 * i)
        # control stays on its own channel/port (autonomy -> sim).
        out["control"] = self.control_port
        return out
