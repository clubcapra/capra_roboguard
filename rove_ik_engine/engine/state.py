"""Shared engine state. Single-threaded asyncio writes; transports may
swap-in a new Ovis at any await point but never tear it.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from forgebot.core.model import Project

from .proto import Ovis


@dataclass
class EngineState:
    project: Project
    joint_values: dict[str, float] = field(default_factory=dict)
    joint_velocities: dict[str, float] = field(default_factory=dict)
    latest_ovis: Ovis | None = None
    start_time: float = field(default_factory=time.monotonic)
    last_tip: str = ""  # most recent Ovis.target, kept after Ovis goes None
    # Per-link TCP (centroid) offset in the link's local frame. Populated at
    # engine startup from mesh geometry; used as the rotation pivot / IK
    # position target when an Ovis arrives with no tcp_offset_local set.
    tcp_offsets: dict[str, np.ndarray] = field(default_factory=dict)
    # Latest kinova_arm state pushed by rove_sensor_api. Ordered by kinova
    # actuator index (1..N). None until the first frame arrives.
    latest_kinova_positions: list[float] | None = None
    latest_kinova_t: float = 0.0
    # Per-joint offset (radians) captured at sync time:
    #     offset[id] = kinova_q_at_sync - model_q_at_sync
    # After sync, mapping kinova readings into the model frame is:
    #     model_q = kinova_q - offset
    # At sync time itself this evaluates to the model's pre-sync value, so
    # the model doesn't visually jump when the user clicks Sync.
    kinova_offsets: dict[str, float] = field(default_factory=dict)
    # Ordered list of joint entity ids matching kinova actuator index 1..N.
    # Captured at sync time so the per-tick mirror loop doesn't have to
    # re-resolve the chain.
    kinova_chain_joint_ids: list[str] = field(default_factory=list)
    # Per-joint sign multiplier (+1 or -1) applied to kinova reads. Set from
    # HardwareConfig.inverted_joints at sync time. Used both when capturing
    # the offset and when mirroring, so the formulas stay consistent:
    #     offset[i] = sign[i] * kinova_q_at_sync - model_q_at_sync
    #     model_q   = sign[i] * kinova_q_now    - offset[i]
    kinova_signs: dict[str, float] = field(default_factory=dict)

    # ---- flippers (track sub-drives 41..44, each its own ODrive) ----------
    # Latest motor position (revolutions) per ODrive node id, pushed by
    # rove_sensor_api. One subscriber per node writes its own key, so a dead
    # node (43/44 are fried on the current robot) simply never appears.
    latest_flipper_positions: dict[int, float] = field(default_factory=dict)
    # Per-node last-frame monotonic timestamp, for staleness gating.
    latest_flipper_t: dict[int, float] = field(default_factory=dict)
    # Calibration captured at flipper Sync, keyed by JOINT entity id:
    #     offset[eid] = sign*scale*pos_at_sync - model_q_at_sync
    #     model_q     = sign*scale*pos_now     - offset[eid]
    # where scale = 2*pi / gear_ratio (motor rev -> joint radian). The model
    # is not overwritten at sync, so it doesn't visually jump.
    flipper_offsets: dict[str, float] = field(default_factory=dict)
    flipper_signs: dict[str, float] = field(default_factory=dict)
    flipper_scales: dict[str, float] = field(default_factory=dict)  # rad per motor rev
    # JOINT entity id -> ODrive node id, resolved once at startup so the
    # per-tick mirror knows which node feeds which joint.
    flipper_joint_to_node: dict[str, int] = field(default_factory=dict)
    # ---- flipper commanding (output) ----
    # Held normalised step per ODrive node ({-1,0,+1}); a non-zero step ramps
    # that flipper's target each tick. Set by the command input, held until changed.
    flipper_cmd_steps: dict[int, int] = field(default_factory=dict)
    # Commanded target angle (rad) per JOINT entity id. While present, that
    # flipper is operator-owned: the read-only mirror leaves it alone and (if
    # output is enabled) the command sender drives the real flipper to it.
    flipper_targets: dict[str, float] = field(default_factory=dict)
    # Command parameters, populated from config at startup (FlipperBank.resolve).
    flipper_step_rate_rad_s: float = 0.35     # how fast a held +-1 step ramps
    flipper_gear_ratio: float = 1.0           # motor revs per joint rev (for the persistence guard)
    # Per-joint sign tuned/persisted at runtime (overrides the config sign at
    # resolve). Lets the operator flip a mirrored flipper's direction live.
    flipper_signs_persisted: dict[str, float] = field(default_factory=dict)
    flipper_limits: dict[str, tuple[float, float]] = field(default_factory=dict)  # eid -> (min,max) rad
    # Last-known PHYSICAL flipper angle (rad), persisted across restarts. The
    # flipper ODrives lose their encoder zero on power-cycle (pos_estimate -> 0)
    # but the worm gear means the flipper can't physically move uncommanded — so
    # we restore the angle and re-derive the offset from the first frame.
    flipper_phys_persisted: dict[str, float] = field(default_factory=dict)
    # Flipper joints awaiting a first-frame re-anchor (set at boot from the
    # persisted physical angle; cleared once the offset is re-derived).
    flipper_reanchor: set[str] = field(default_factory=set)
    # Velocity command (normalised -1..1) per ODrive node for velocity-mode
    # drives (drums). Held until changed; scaled by the node's max_vel_rev_s.
    drive_vel_cmd: dict[int, float] = field(default_factory=dict)
    # JOINT entity ids of velocity-mode drives (the drums). These are continuous
    # wheels — never posed as joint-space positions — so pose-to-pose moves skip
    # them. Position-mode flippers are NOT here, so a saved pose DOES move them.
    drive_velocity_joints: set[str] = field(default_factory=set)
    # Monotonic time of the last drive frame (drums + flipper steps) received on
    # the drive UDP input. The drive watchdog stops the ODrives when this goes
    # stale — drive_vel_cmd / flipper_cmd_steps PERSIST (unlike consume-once
    # Ovis), so without it the drums hold their last velocity and the flippers
    # keep ramping the last step forever when packets stop.
    latest_drive_t: float = 0.0

    # Named pose library (name -> {joint eid -> angle rad}). "home" is just a
    # reserved name used by Reset-to-home / from-home Sync. Captured via "Set
    # home"/"Save pose" and persisted so they survive restart.
    poses: dict[str, dict[str, float]] = field(default_factory=dict)

    # Active joint-space trajectory (engine.motion.Motion) or None. While set,
    # the tick drives the planned joints toward the target and the mirrors /
    # IK leave those joints alone. Typed loosely to avoid an import cycle.
    active_motion: "object | None" = None

    # Live IK runtime flags. Initialised from engine.toml [ik] at startup
    # and mutable at runtime via the /api/v1/ik/collision HTTP endpoint
    # (so operators can disable collision-aware IK after the arm locks
    # itself into a self-collision without restarting the engine).
    collision_aware: bool = True

    # Rate-limiting state for per-tick warnings — written by the tick loop,
    # read by nothing else. Leading underscore = "internal state, don't
    # serialise" (no proto field corresponds).
    _last_bad_target_warn_t: float = 0.0

    def elapsed(self) -> float:
        return time.monotonic() - self.start_time

    def set_ovis(self, ovis: Ovis | None) -> None:
        self.latest_ovis = ovis
        if ovis is not None and ovis.target:
            self.last_tip = ovis.target

    def take_ovis(self) -> Ovis | None:
        """Read-and-clear. Each Ovis is consumed by one tick; subsequent ticks
        with no fresh input hold the robot still (qdot = 0)."""
        o = self.latest_ovis
        self.latest_ovis = None
        return o
