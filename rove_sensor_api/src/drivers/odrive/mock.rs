//! Sim-backed ODrive mock driver (one per axis: 4 drums + 4 flippers).
//!
//! Telemetry comes from the sim's `odrive_3x`/`4x` channel (the sim emits the real
//! `OdriveNodeState` field names) — EXCEPT the **drive-state machine**, which the
//! sim does not model. The sim is pure motor physics; it has no notion of arming,
//! idling, or faults, so it streams a constant `axis_state = 8`. The mock therefore
//! OWNS the axis state machine and overlays it onto the telemetry, faithful to
//! ODrive firmware 0.6.11:
//!
//!   * IDLE (1)             — axis idling; setpoints are NOT actuated.
//!   * CLOSED_LOOP_CONTROL (8) — axis armed; `input_vel` reaches the sim.
//!   * error/disarmed       — a latched fault (e.g. ESTOP_REQUESTED) in
//!     `active_errors`/`disarm_reason`; the axis sits in IDLE and REFUSES to arm
//!     until `clear_errors`. `procedure_result` reports SUCCESS/BUSY/DISARMED.
//!
//! Velocity is the one setpoint with a sim sink (`input_vel` rev/s → normalized
//! `tracks.{left,right}` for drums, deploy sign for flippers) and is applied ONLY
//! while armed, so "Idle" actually stops the wheels. Other fine setpoints (gains,
//! traj, control_mode, …) have no sim sink and are accepted-and-ignored.

use std::sync::atomic::{AtomicU32, AtomicU8, Ordering};

use serde_json::{json, Value};

use crate::core::driver::{CommandMode, FieldDescriptor, SensorDriver};
use crate::core::error::DriverError;
use crate::drivers::sim::control::{set_flipper, set_track, SharedControl};
use crate::drivers::sim::feed::SimFeed;

use super::node::{command_schema, data_schema};

// ── ODrive fw 0.6.11 constants (axis_state is u8 in OdriveNodeState) ──────────
const AXIS_IDLE: u8 = 1; // AxisState::Idle
const AXIS_CLOSED_LOOP: u8 = 8; // AxisState::ClosedLoopControl
/// ODriveError::ESTOP_REQUESTED bit (active_errors / disarm_reason bitmask).
const ERR_ESTOP_REQUESTED: u32 = 0x0100_0000;
// ProcedureResult (fw ≥ 0.6): reported in `procedure_result`.
const PROC_SUCCESS: u8 = 0;
const PROC_BUSY: u8 = 1;
const PROC_DISARMED: u8 = 3;

/// What an ODrive axis drives. The rover has 8: 4 drums (track sides) + 4
/// flippers, each on its own ODrive node.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    /// A main-track side ("left" | "right").
    Track(&'static str),
    /// A flipper ("fl" | "fr" | "rl" | "rr").
    Flipper(&'static str),
}

impl Axis {
    fn label(self) -> &'static str {
        match self {
            Axis::Track(s) => s,
            Axis::Flipper(s) => s,
        }
    }
}

pub struct OdriveMock {
    id_str: String,
    display_name: String,
    axis: Axis,
    feed: SimFeed,
    control: SharedControl,
    /// rev/s that maps to full-scale (|track| = 1.0) when normalizing input_vel.
    max_vel_rev_s: f64,
    // ── Mock-owned drive state (the sim models physics, not the ODrive state
    // machine). Atomics: SensorDriver methods take &self and are shared across
    // the request + telemetry threads.
    axis_state: AtomicU8,
    procedure_result: AtomicU8,
    active_errors: AtomicU32,
    disarm_reason: AtomicU32,
}

impl OdriveMock {
    pub fn new(
        node_id: u8,
        axis: Axis,
        host: &str,
        backend_port: u16,
        control: SharedControl,
        max_vel_rev_s: f64,
    ) -> Self {
        let kind = match axis {
            Axis::Track(_) => "drum",
            Axis::Flipper(_) => "flipper",
        };
        Self {
            id_str: format!("odrive_{node_id}"),
            display_name: format!("ODrive Node {node_id} (sim, {kind} {})", axis.label()),
            axis,
            feed: SimFeed::subscribe(host, backend_port),
            control,
            max_vel_rev_s: if max_vel_rev_s.abs() < 1e-6 { 20.0 } else { max_vel_rev_s },
            // Power-on default is IDLE with no errors, like a freshly-booted ODrive.
            axis_state: AtomicU8::new(AXIS_IDLE),
            procedure_result: AtomicU8::new(PROC_SUCCESS),
            active_errors: AtomicU32::new(0),
            disarm_reason: AtomicU32::new(0),
        }
    }

    fn is_armed(&self) -> bool {
        self.axis_state.load(Ordering::Relaxed) == AXIS_CLOSED_LOOP
    }

    /// Stop driving this axis in the sim (used whenever we leave closed-loop).
    fn zero_output(&self) {
        match self.axis {
            Axis::Track(side) => set_track(&self.control, side, 0.0),
            Axis::Flipper(key) => set_flipper(&self.control, key, 0),
        }
    }

    /// Disarm to IDLE, zeroing the drive. `proc` is the resulting ProcedureResult.
    fn disarm(&self, proc: u8) {
        self.axis_state.store(AXIS_IDLE, Ordering::Relaxed);
        self.procedure_result.store(proc, Ordering::Relaxed);
        self.zero_output();
    }

    /// Request a new axis_state (the CAN Set_Axis_State command surface).
    /// Arming into CLOSED_LOOP is refused while errors are active — exactly like
    /// real firmware, which immediately disarms back to IDLE with DISARMED.
    fn request_axis_state(&self, req: u8) {
        if req == AXIS_CLOSED_LOOP {
            if self.active_errors.load(Ordering::Relaxed) != 0 {
                self.disarm(PROC_DISARMED); // can't arm with a latched fault
            } else {
                self.axis_state.store(AXIS_CLOSED_LOOP, Ordering::Relaxed);
                self.procedure_result.store(PROC_BUSY, Ordering::Relaxed); // running
            }
        } else if req == AXIS_IDLE {
            self.disarm(PROC_SUCCESS); // clean stop
        } else {
            // calibration/other states: not actuated by the sim — treat as not
            // armed, but echo the requested state back so the UI reflects it.
            self.axis_state.store(req, Ordering::Relaxed);
            self.procedure_result.store(PROC_BUSY, Ordering::Relaxed);
            self.zero_output();
        }
    }
}

impl SensorDriver for OdriveMock {
    fn id(&self) -> &str {
        &self.id_str
    }

    fn display_name(&self) -> &str {
        &self.display_name
    }

    fn command_mode(&self) -> CommandMode {
        CommandMode::Stream { interval_ms: 100 }
    }

    fn data_schema(&self) -> Vec<FieldDescriptor> {
        data_schema()
    }

    fn command_schema(&self) -> Vec<FieldDescriptor> {
        command_schema()
    }

    fn read_data(&self) -> Result<Value, DriverError> {
        // Start from the sim's physics telemetry, then overlay the mock-owned
        // drive-state fields so arm/idle/error are authoritative here (the sim
        // hardcodes axis_state = 8 and never reports faults).
        let mut v = self.feed.latest_or_empty();
        if !v.is_object() {
            v = Value::Object(Default::default());
        }
        if let Some(obj) = v.as_object_mut() {
            let errs = self.active_errors.load(Ordering::Relaxed);
            obj.insert("axis_state".into(), json!(self.axis_state.load(Ordering::Relaxed)));
            obj.insert("procedure_result".into(), json!(self.procedure_result.load(Ordering::Relaxed)));
            obj.insert("active_errors".into(), json!(errs));
            obj.insert("axis_error".into(), json!(errs)); // keep the UI's error pill consistent
            obj.insert("disarm_reason".into(), json!(self.disarm_reason.load(Ordering::Relaxed)));
        }
        Ok(v)
    }

    fn execute_command(&self, payload: &Value) -> Result<Value, DriverError> {
        let mut sent: Vec<&str> = Vec::new();

        // clear_errors first: a real ODrive will not arm until faults are cleared.
        if payload.get("clear_errors").is_some() {
            self.active_errors.store(0, Ordering::Relaxed);
            self.disarm_reason.store(0, Ordering::Relaxed);
            self.procedure_result.store(PROC_SUCCESS, Ordering::Relaxed);
            sent.push("clear_errors");
        }

        // Set_Axis_State (arm = 8, idle = 1, …). The mock owns this state machine.
        if let Some(req) = payload.get("axis_state").and_then(Value::as_u64) {
            self.request_axis_state(req as u8);
            sent.push("axis_state");
        }

        // Velocity setpoint — the one command with a sim sink. Applied ONLY while
        // armed (closed-loop); otherwise acknowledged but not actuated, so an idle
        // or faulted axis never moves the rover.
        if let Some(vel) = payload.get("input_vel").and_then(Value::as_f64) {
            if self.is_armed() {
                match self.axis {
                    Axis::Track(side) => {
                        set_track(&self.control, side, (vel / self.max_vel_rev_s).clamp(-1.0, 1.0));
                    }
                    Axis::Flipper(key) => {
                        let cmd = if vel > 1e-3 { 1 } else if vel < -1e-3 { -1 } else { 0 };
                        set_flipper(&self.control, key, cmd);
                    }
                }
            }
            sent.push("input_vel");
        }

        // Remaining fine setpoints: accepted, no sim sink (no physical effect).
        for k in [
            "input_pos", "input_vel_ff", "input_torque", "input_torque_ff",
            "control_mode", "input_mode", "velocity_limit", "current_limit", "traj_vel_limit",
            "traj_accel_limit", "traj_decel_limit", "traj_inertia", "pos_gain", "vel_gain",
            "vel_integrator_gain", "reboot",
        ] {
            if payload.get(k).is_some() {
                sent.push(k);
            }
        }

        if sent.is_empty() {
            return Err(DriverError::CommandFailed(
                "no recognised command fields in payload".into(),
            ));
        }
        Ok(json!({ "sent": sent }))
    }

    fn has_estop(&self) -> bool {
        true
    }

    fn estop(&self) -> Result<Value, DriverError> {
        // Faithful to hardware: e-stop LATCHES a fault and disarms. The axis drops
        // to IDLE with ESTOP_REQUESTED set; re-arming requires clear_errors first.
        self.active_errors.store(ERR_ESTOP_REQUESTED, Ordering::Relaxed);
        self.disarm_reason.store(ERR_ESTOP_REQUESTED, Ordering::Relaxed);
        self.disarm(PROC_DISARMED);
        Ok(json!({ "estop": "sim", "axis": self.axis.label(), "active_errors": ERR_ESTOP_REQUESTED }))
    }
}
