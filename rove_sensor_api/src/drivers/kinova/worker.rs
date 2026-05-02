//! Single-threaded SDK owner.
//!
//! The Kinova SDK is not thread-safe; one OS thread owns the `KinovaSdk`
//! and serialises every call. Telemetry polling and command execution are
//! interleaved on the same thread:
//!
//! - Commands arrive on a `mpsc` channel from `KinovaArm::execute_command`.
//! - When the channel is idle (no command for `TELEMETRY_INTERVAL`), the
//!   worker polls one round of telemetry and updates the shared state.
//! - When the channel is idle for `WATCHDOG_TIMEOUT`, the worker fires
//!   `EraseAllTrajectories` once to halt any in-flight motion. Fire-once
//!   semantics — re-arms when the next command arrives.

use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use super::ffi::{
    angular_position_point, angular_velocity_point, robotiq_activate_frame, robotiq_probe_frame,
    AngularPosition, QuickStatus, SensorsInfo,
};
use super::sdk::KinovaSdk;
use super::state::KinovaState;

/// Spacing between telemetry ticks. Each tick performs only **one** group
/// of SDK calls (round-robin across pos/vel, force/current, sensors+status)
/// so a single tick costs ~5–10 ms instead of the ~30 ms a full 6-call burst
/// would. With 100 ms spacing × 3 groups, every value refreshes every ~300
/// ms while still leaving most of the worker's time available to forward
/// velocity setpoints.
pub const TELEMETRY_INTERVAL: Duration = Duration::from_millis(100);

/// While a non-zero velocity setpoint is held, the worker re-sends it to the
/// SDK at this cadence — matching the arm's internal 100 Hz / 10 ms DSP loop.
/// Per Kinova docs the publish rate **must** be 100 Hz; below that the
/// robot can't track the commanded velocity reliably. The user's stream
/// rate doesn't matter because each user packet just refreshes
/// `held_velocity` and its `last_resend` timestamp — if it lands ≤10 ms
/// before the next tick, that tick is suppressed (no double-send).
pub const VELOCITY_RESEND_INTERVAL: Duration = Duration::from_millis(10);

/// If no *new* velocity command arrives within this window, the held velocity
/// is cleared and a zero-velocity setpoint is sent once. Acts as the safety
/// net for an abandoned client — without it, a single velocity packet would
/// drive the arm indefinitely.
///
/// Tuned for streaming clients: any reasonable client (UDP at ≥10 Hz) will
/// keep the hold refreshed comfortably. On client disconnect / button-up,
/// the arm halts within this window — feels snappy at the operator end.
pub const VELOCITY_HOLD_TIMEOUT: Duration = Duration::from_millis(300);

/// Externally-visible "expected stream interval" reported via `command_mode`.
/// Clients streaming faster than this are inside the safety window.
pub const STREAM_HINT_INTERVAL: Duration = Duration::from_millis(100);

/// Commands posted by `KinovaArm::execute_command` and consumed by the worker.
#[derive(Debug, Clone)]
pub enum Cmd {
    /// 6-DOF angular position setpoint (degrees, joint 1..6).
    SetAngularPosition([f32; 6]),
    /// 6-DOF angular velocity setpoint (deg/s, joint 1..6).
    SetAngularVelocity([f32; 6]),
    /// Switch the arm to angular control mode.
    SetAngularControl,
    /// `Ethernet_StartControlAPI` — required after `Estop` to re-arm.
    StartControl,
    /// Move the arm to its built-in HOME pose.
    MoveHome,
    /// Clear the SDK error log.
    ClearErrors,
    /// Cancel any queued trajectories without disabling control.
    EraseTrajectories,
    /// Emergency stop: erase trajectories + StopControlAPI. Caller must send
    /// `StartControl` to recover.
    Estop,
    /// Persist the current position of one actuator as its new zero. Argument
    /// is the actuator bus address (use `joint_to_actuator_address`).
    SetJointZero(i32),
    /// Diagnostic only — switch the arm into RS-485 passthrough mode, send
    /// one Robotiq Modbus probe frame to slave 9, read back any response
    /// bytes, and log everything. **Hijacks the bus**: after this command
    /// runs, the normal `Ethernet_*` API stops responding and the process
    /// must be restarted. Use exclusively for diagnosing whether the JACO2
    /// tool-port RS485 reaches the gripper at a usable baud / framing.
    GripperProbe,
    /// Non-destructive diagnostic — calls `Ethernet_GetGripperStatus` and
    /// `Ethernet_InitFingers` (no RS485 hijack). Tells us whether the
    /// arm firmware exposes a Robotiq attached at the joint-7 slot through
    /// the standard SDK gripper API.
    GripperDiagnostic,
    /// **Hijacks the bus** like `GripperProbe`, then sends a single Robotiq
    /// 2F-140 Modbus RTU activation frame (FC06, slave 9, reg 0x03E8 = 1)
    /// over `OpenRS485_Write`. Open-loop: no read-back. Watch the gripper
    /// LED — solid red → blinking red/blue means the write reached the
    /// gripper at the right baud / framing.
    GripperActivate,
}

/// Run the worker loop on the current thread until the command channel is
/// closed. Spawn this via `std::thread::spawn`.
///
/// **Velocity hold:** the SDK expires velocity setpoints internally after
/// ~250 ms, so a one-shot velocity command from the user would only twitch
/// the arm. The worker latches the most recent non-zero velocity and
/// re-sends it to the SDK at `VELOCITY_RESEND_INTERVAL` until either:
///   - a new velocity command replaces it,
///   - any non-velocity command (position, estop, home, erase) clears it,
///   - or `VELOCITY_HOLD_TIMEOUT` elapses with no fresh velocity command,
///     at which point the worker sends a zero-velocity setpoint once and
///     drops the hold (safety net for an abandoned client).
///
/// **Position setpoints** are not held — the arm FIFO-executes them on its
/// own and re-sending would re-trigger motion endlessly.
pub fn run(sdk: KinovaSdk, rx: Receiver<Cmd>, state: Arc<RwLock<KinovaState>>) {
    let mut last_telemetry = Instant::now() - TELEMETRY_INTERVAL;
    let mut telem_idx: u8 = 0; // round-robin selector for the next telemetry group
    // Held velocity setpoint. `Some` while we're actively re-sending; `None`
    // when the arm should be idle (or in position mode, completing a FIFO
    // trajectory the SDK is driving on its own).
    let mut held_velocity: Option<[f32; 6]> = None;
    let mut velocity_set_at = Instant::now();
    let mut last_resend = Instant::now();
    // Local mirror of the post-E-stop "API control disabled" state. While
    // true, we drop incoming setpoint commands silently — the SDK would
    // reject them with code 1022 anyway, and an active streaming client
    // (e.g. the test UI) would otherwise spam the log dozens of times per
    // second. Cleared by `Cmd::StartControl`.
    let mut estopped = false;

    loop {
        let now = Instant::now();
        let next_telemetry = last_telemetry + TELEMETRY_INTERVAL;
        // Compute the next velocity-related deadline only if we're actually holding one.
        let next_resend = held_velocity.map(|_| last_resend + VELOCITY_RESEND_INTERVAL);
        let next_expire = held_velocity.map(|_| velocity_set_at + VELOCITY_HOLD_TIMEOUT);

        let mut next_event = next_telemetry;
        if let Some(t) = next_resend {
            next_event = next_event.min(t);
        }
        if let Some(t) = next_expire {
            next_event = next_event.min(t);
        }
        let timeout = next_event.saturating_duration_since(now);

        match rx.recv_timeout(timeout) {
            Ok(first) => {
                // Coalesce: drain everything that's already queued. Stale
                // velocity setpoints are dropped (latest-wins); one-shots
                // (estop, home, etc.) are kept in arrival order. Without
                // this, a single ~100 ms SDK stall lets a 75 Hz stream
                // accumulate ~7 packets which then drip out one per tick,
                // showing up as multi-second lag at the operator end.
                let mut latest_velocity: Option<[f32; 6]> = None;
                let mut one_shots: Vec<Cmd> = Vec::new();

                let mut classify = |c: Cmd| match c {
                    Cmd::SetAngularVelocity(v) => latest_velocity = Some(v),
                    other => one_shots.push(other),
                };
                classify(first);
                loop {
                    match rx.try_recv() {
                        Ok(c) => classify(c),
                        Err(TryRecvError::Empty) => break,
                        Err(TryRecvError::Disconnected) => {
                            tracing::info!("Kinova worker: channel disconnected mid-drain");
                            // Fall through; the outer Disconnected branch
                            // will catch this on the next iteration.
                            break;
                        }
                    }
                }

                let now = Instant::now();

                // Process one-shots in arrival order (estop / start / etc.).
                for cmd in &one_shots {
                    let suppressed = estopped
                        && matches!(cmd, Cmd::SetAngularPosition(_));
                    if !suppressed {
                        handle_cmd(&sdk, cmd, &state);
                    }
                    match cmd {
                        Cmd::SetAngularPosition(_)
                        | Cmd::EraseTrajectories
                        | Cmd::MoveHome => {
                            held_velocity = None;
                        }
                        Cmd::Estop => {
                            held_velocity = None;
                            latest_velocity = None; // anything queued before is moot
                            estopped = true;
                        }
                        Cmd::StartControl => {
                            estopped = false;
                        }
                        _ => {}
                    }
                }

                // Velocity setpoint: cache the latest value, then send to the
                // SDK only if at least `VELOCITY_RESEND_INTERVAL` has elapsed
                // since the last send. This rate-limits regardless of who
                // triggers the send (user packet vs timer), guaranteeing
                // exactly ~100 Hz to the SDK and avoiding the double-send
                // race that would otherwise let the arm's 2000-entry FIFO
                // accumulate stale entries (multi-second perceived lag).
                //
                // Exception: an all-zero velocity is "stop now" intent —
                // send it immediately, regardless of the rate gate.
                if let Some(v) = latest_velocity {
                    if !estopped {
                        if v.iter().all(|&x| x == 0.0) {
                            handle_cmd(&sdk, &Cmd::SetAngularVelocity(v), &state);
                            held_velocity = None;
                            last_resend = now;
                        } else {
                            held_velocity = Some(v);
                            velocity_set_at = now;
                            if now >= last_resend + VELOCITY_RESEND_INTERVAL {
                                handle_cmd(&sdk, &Cmd::SetAngularVelocity(v), &state);
                                last_resend = now;
                            }
                            // Otherwise the timer branch picks it up at the
                            // next deadline using the just-cached `held_velocity`.
                        }
                    }
                }
            }
            Err(RecvTimeoutError::Disconnected) => {
                tracing::info!("Kinova worker: command channel closed, shutting down");
                // Best-effort: send a zero velocity so the arm halts even if
                // a velocity hold was in flight when the channel closed.
                if held_velocity.is_some() {
                    let _ = sdk.send_advance_trajectory(angular_velocity_point([0.0; 6]));
                }
                break;
            }
            Err(RecvTimeoutError::Timeout) => {
                let now = Instant::now();

                if now >= next_telemetry {
                    poll_telemetry_group(&sdk, &state, telem_idx);
                    telem_idx = (telem_idx + 1) % 3;
                    last_telemetry = now;
                }

                if let Some(v) = held_velocity {
                    if now >= velocity_set_at + VELOCITY_HOLD_TIMEOUT {
                        // Hold expired — send a single zero and drop the hold.
                        if let Err(e) =
                            sdk.send_advance_trajectory(angular_velocity_point([0.0; 6]))
                        {
                            tracing::warn!(error = %e, "Kinova hold-expire zero-vel send failed");
                        }
                        tracing::info!(
                            "Kinova velocity hold expired after {:?} of no new commands — arm halted",
                            VELOCITY_HOLD_TIMEOUT
                        );
                        held_velocity = None;
                    } else if now >= last_resend + VELOCITY_RESEND_INTERVAL {
                        if let Err(e) = sdk.send_advance_trajectory(angular_velocity_point(v)) {
                            tracing::debug!(error = %e, "Kinova velocity resend failed (transient)");
                        }
                        last_resend = now;
                    }
                }
            }
        }
    }
}

fn handle_cmd(sdk: &KinovaSdk, cmd: &Cmd, state: &Arc<RwLock<KinovaState>>) {
    let result = match cmd {
        Cmd::SetAngularPosition(joints) => {
            sdk.send_basic_trajectory(angular_position_point(*joints))
        }
        Cmd::SetAngularVelocity(joints) => {
            sdk.send_advance_trajectory(angular_velocity_point(*joints))
        }
        Cmd::SetAngularControl => sdk.set_angular_control(),
        Cmd::StartControl => {
            let r = sdk.start_control();
            if r.is_ok() {
                state.write().unwrap().estopped = false;
            }
            r
        }
        Cmd::MoveHome => sdk.move_home(),
        Cmd::ClearErrors => sdk.clear_error_log(),
        Cmd::EraseTrajectories => sdk.erase_all_trajectories(),
        Cmd::Estop => {
            // Best-effort: try erase first, then stop control. Always set the
            // state flag even if SDK calls fail — the operator's intent is the
            // source of truth.
            let _ = sdk.erase_all_trajectories();
            let r = sdk.stop_control();
            state.write().unwrap().estopped = true;
            r
        }
        Cmd::SetJointZero(addr) => sdk.set_joint_zero(*addr),
        Cmd::GripperProbe => run_gripper_probe(sdk),
        Cmd::GripperDiagnostic => run_gripper_diagnostic(sdk),
        Cmd::GripperActivate => run_gripper_activate(sdk),
    };
    if let Err(e) = result {
        tracing::warn!(?cmd, error = %e, "Kinova command failed");
    }
}

/// Update one slice of the telemetry cache. Splitting the work across three
/// groups (run on consecutive ticks) caps each tick at ~10 ms instead of the
/// ~30 ms a full snapshot takes — that's the difference between the worker
/// missing one DSP cycle and missing three when telemetry runs.
fn poll_telemetry_group(sdk: &KinovaSdk, state: &Arc<RwLock<KinovaState>>, group: u8) {
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0);

    match group {
        // Group 0 — joint kinematics. Most useful for control feedback so it
        // gets a tick to itself.
        0 => {
            let pos = read_or_default(sdk.get_angular_position(), "GetAngularPosition");
            let vel = read_or_default(sdk.get_angular_velocity(), "GetAngularVelocity");
            let mut s = state.write().unwrap();
            s.joint_pos = actuators_to_array(&pos);
            s.joint_vel = actuators_to_array(&vel);
            s.timestamp_ns = now_ns;
        }
        // Group 1 — torque + motor current.
        1 => {
            let force = read_or_default(sdk.get_angular_force(), "GetAngularForce");
            let current = read_or_default(sdk.get_angular_current(), "GetAngularCurrent");
            let mut s = state.write().unwrap();
            s.joint_torque = actuators_to_array(&force);
            s.joint_current = actuators_to_array(&current);
        }
        // Group 2 — sensors block + quick status.
        _ => {
            let sensors: SensorsInfo =
                read_or_default(sdk.get_sensors_info(), "GetSensorsInfo");
            let qs: QuickStatus = read_or_default(sdk.get_quick_status(), "GetQuickStatus");
            let mut s = state.write().unwrap();
            s.joint_temp = [
                sensors.ActuatorTemp1,
                sensors.ActuatorTemp2,
                sensors.ActuatorTemp3,
                sensors.ActuatorTemp4,
                sensors.ActuatorTemp5,
                sensors.ActuatorTemp6,
            ];
            s.bus_voltage = sensors.Voltage;
            s.bus_current = sensors.Current;
            s.accel_x = sensors.AccelerationX;
            s.accel_y = sensors.AccelerationY;
            s.accel_z = sensors.AccelerationZ;
            // SDK semantics: 0 = ON, non-zero = OFF.
            s.control_enabled = qs.ControlEnableStatus == 0;
            s.retract_state = qs.RetractType;
            s.robot_type = qs.RobotType;
            s.torque_sensors_available = qs.TorqueSensorsStatus != 0;
        }
    }
}

/// Diagnostic that probes the Robotiq 2F-140 over the arm's RS-485 bus.
///
/// Sequence:
/// 1. `OpenRS485_Activate` — switch the arm/comm-layer into raw RS-485 mode.
///    *This permanently disables the normal `Ethernet_*` API for this
///    process — caller must restart the API afterwards.*
/// 2. Build one Modbus RTU "Read Status" frame addressed to Robotiq slave
///    9 (`09 03 07 D0 00 03 [crc]`), shoehorn its 8 bytes into the first
///    8 bytes of an `RS485Message` (16-byte data union → bytes 8..19 stay
///    zero), and `OpenRS485_Write` it.
/// 3. `OpenRS485_Read` up to 8 messages back-to-back, log every byte.
///
/// Three possible outcomes — the operator interprets:
/// - **Modbus reply present** (slave 9 echo + 9 bytes of data) → bus runs
///   at usable baud, framing survives. Build a real driver.
/// - **Read returns 0 messages or `RS485_TIMEOUT (1020)`** → bus reaches
///   gripper but baud / framing unusable. MCU bridge required.
/// - **Activate / Write returns nonzero** → arm rejected mode switch.
fn run_gripper_probe(sdk: &super::sdk::KinovaSdk) -> Result<(), super::sdk::SdkError> {
    use super::ffi::RS485Message;
    tracing::warn!(
        "Kinova RS485 gripper probe starting — arm API will be unresponsive after this. \
         Restart the rove_sensor_api process to recover normal control."
    );

    // Some Kinova bus-mode entries fail silently when StartControlAPI is
    // active. Stop control first so the comm layer can take over the link.
    if let Err(e) = sdk.stop_control() {
        tracing::warn!(error = %e, "StopControlAPI before RS485 — non-fatal");
    } else {
        tracing::info!("StopControlAPI ok (preparing for RS485)");
    }
    std::thread::sleep(std::time::Duration::from_millis(100));

    sdk.rs485_activate()?;
    tracing::info!("RS485 mode activated");

    let hex = |b: &[u8]| {
        b.iter()
            .map(|x| format!("{:02x}", x))
            .collect::<Vec<_>>()
            .join(" ")
    };

    fn run_one_probe(
        sdk: &super::sdk::KinovaSdk,
        label: &str,
        msg: RS485Message,
    ) -> Result<(), super::sdk::SdkError> {
        let bytes = msg.as_bytes();
        let hex_out: String = bytes
            .iter()
            .map(|x| format!("{:02x}", x))
            .collect::<Vec<_>>()
            .join(" ");
        tracing::info!(label, probe_hex = %hex_out, "writing");
        let sent = sdk.rs485_write(std::slice::from_ref(&msg))?;
        tracing::info!(label, sent, "Write returned ok");
        std::thread::sleep(std::time::Duration::from_millis(50));
        match sdk.rs485_read(8) {
            Ok(replies) => {
                if replies.is_empty() {
                    tracing::warn!(label, "Read: 0 messages back");
                } else {
                    for (i, m) in replies.iter().enumerate() {
                        let b = m.as_bytes();
                        let h: String = b
                            .iter()
                            .map(|x| format!("{:02x}", x))
                            .collect::<Vec<_>>()
                            .join(" ");
                        tracing::info!(label, idx = i, raw = %h, "received");
                    }
                }
            }
            Err(e) => tracing::warn!(label, error = %e, "Read failed"),
        }
        Ok(())
    }
    let _ = hex; // shadowed by the closure inside run_one_probe; kept to silence lint warmup
    let _ = std::mem::size_of::<RS485Message>(); // unused-import suppression

    // ── Probe 1: Robotiq Modbus, slave 9 (default), Read Status -----------
    run_one_probe(sdk, "modbus_slave_9_FC03_status", robotiq_probe_frame())?;

    // ── Probe 2: Kinova-protocol GET_DEVICE_INFO (cmd 0x29) to actuator 16
    //    (joint 1). If the bus is electrically alive at the Kinova baud /
    //    framing, this should elicit a Kinova-shaped reply (cmd 0x2A
    //    SEND_DEVICE_INFO). 0x10 = decimal 16, the J1 actuator address.
    let mut kinova_j1 = RS485Message::zeroed();
    kinova_j1.Command = 0x0029;
    kinova_j1.SourceAddress = 0x00; // host / null
    kinova_j1.DestinationAddress = 0x10;
    run_one_probe(sdk, "kinova_get_device_info_actuator_16_J1", kinova_j1)?;

    // ── Probe 3: Same query, addressed to actuator 25 (joint 7 / gripper
    //    slot per the kinova-ros address table). If the Capra firmware /
    //    arm physical bus exposes the gripper slot at all, this is where
    //    it lives.
    let mut kinova_j7 = RS485Message::zeroed();
    kinova_j7.Command = 0x0029;
    kinova_j7.SourceAddress = 0x00;
    kinova_j7.DestinationAddress = 0x19; // 25
    run_one_probe(sdk, "kinova_get_device_info_actuator_25_J7", kinova_j7)?;

    // ── Probe 4: Sweep — Kinova GET_DEVICE_INFO to every actuator address
    //    1..32. If anything answers, we'll see it. Cheap because each is a
    //    single 50 ms read.
    for addr in 1u8..=32u8 {
        let mut m = RS485Message::zeroed();
        m.Command = 0x0029;
        m.SourceAddress = 0x00;
        m.DestinationAddress = addr;
        let _ = sdk.rs485_write(std::slice::from_ref(&m));
        std::thread::sleep(std::time::Duration::from_millis(20));
        if let Ok(replies) = sdk.rs485_read(8) {
            for (i, r) in replies.iter().enumerate() {
                let b = r.as_bytes();
                let h: String = b
                    .iter()
                    .map(|x| format!("{:02x}", x))
                    .collect::<Vec<_>>()
                    .join(" ");
                tracing::info!(sweep_addr = addr, idx = i, raw = %h, "sweep reply");
            }
        }
    }

    tracing::warn!(
        "Probe complete. Arm control is now hijacked into RS485 mode — \
         restart the rove_sensor_api process to resume joint control."
    );
    Ok(())
}

/// Non-destructive gripper diagnostic. Calls `GetGripperStatus`,
/// `InitFingers`, and `GetGripperStatus` again. Logs the model string and
/// per-finger { connected, init, address, position, current, communication
/// errors } before & after init. Does *not* enter RS-485 mode.
fn run_gripper_diagnostic(sdk: &super::sdk::KinovaSdk) -> Result<(), super::sdk::SdkError> {
    fn cstr_to_string(buf: &[u8]) -> String {
        let len = buf.iter().position(|&b| b == 0).unwrap_or(buf.len());
        String::from_utf8_lossy(&buf[..len]).into_owned()
    }
    fn log_status(sdk: &super::sdk::KinovaSdk, label: &str) {
        match sdk.get_gripper_status() {
            Err(e) => tracing::warn!(label, error = %e, "GetGripperStatus failed"),
            Ok(g) => {
                let model = cstr_to_string(&g.Model);
                tracing::info!(
                    label,
                    model = %model,
                    "Gripper struct received (model field shown)"
                );
                for (i, f) in g.Fingers.iter().enumerate() {
                    let id = cstr_to_string(&f.ID);
                    tracing::info!(
                        label,
                        finger = i + 1,
                        id = %id,
                        is_connected = f.IsFingerConnected,
                        is_init = f.IsFingerInit,
                        finger_address = f.FingerAddress,
                        actual_position = f.ActualPosition,
                        actual_current = f.ActualCurrent,
                        comm_errors = f.CommunicationErrors,
                        device_id = f.DeviceID,
                        code_version = f.CodeVersion,
                        "finger state"
                    );
                }
            }
        }
    }

    tracing::info!("Kinova gripper diagnostic — non-destructive");
    log_status(sdk, "before_init");

    match sdk.init_fingers() {
        Ok(()) => tracing::info!("InitFingers returned ok"),
        Err(e) => tracing::warn!(error = %e, "InitFingers returned error"),
    }

    // Give the arm a moment to drive the fingers through their range.
    std::thread::sleep(std::time::Duration::from_secs(2));

    log_status(sdk, "after_init");
    tracing::info!("Kinova gripper diagnostic complete");
    Ok(())
}

/// Send a single Robotiq 2F-140 activation frame over the arm's RS-485
/// passthrough. Open-loop: no read attempt afterwards. The signal we're
/// looking for is the gripper's LED transitioning from solid red to
/// blinking red/blue, which proves writes are reaching it at 115200 8N1
/// Modbus framing. Hijacks the bus exactly like `run_gripper_probe`.
fn run_gripper_activate(sdk: &super::sdk::KinovaSdk) -> Result<(), super::sdk::SdkError> {
    tracing::warn!(
        "Kinova RS485 gripper activate starting — arm API will be unresponsive after this. \
         Restart the rove_sensor_api process to recover normal control."
    );

    if let Err(e) = sdk.stop_control() {
        tracing::warn!(error = %e, "StopControlAPI before RS485 — non-fatal");
    } else {
        tracing::info!("StopControlAPI ok (preparing for RS485)");
    }
    std::thread::sleep(std::time::Duration::from_millis(100));

    sdk.rs485_activate()?;
    tracing::info!("RS485 mode activated");

    let frame = robotiq_activate_frame();
    let bytes = frame.as_bytes();
    let hex_out: String = bytes
        .iter()
        .map(|x| format!("{:02x}", x))
        .collect::<Vec<_>>()
        .join(" ");
    tracing::info!(frame_hex = %hex_out, "writing Robotiq activation frame (FC06 slave 9 reg 0x03E8 = 1)");

    let sent = sdk.rs485_write(std::slice::from_ref(&frame))?;
    tracing::info!(sent, "Write returned ok — watch the gripper LED for ~2 seconds");

    // No read. The whole point of this command is to see whether the LED
    // changes. Caller restarts the process afterwards either way.
    std::thread::sleep(std::time::Duration::from_secs(2));
    tracing::warn!(
        "Activate complete. Arm control is now hijacked into RS485 mode — \
         restart the rove_sensor_api process to resume joint control."
    );
    Ok(())
}

fn read_or_default<T: Default>(r: Result<T, super::sdk::SdkError>, call: &'static str) -> T {
    match r {
        Ok(v) => v,
        Err(e) => {
            // Transient UDP timeouts (typically code 1015) are common at this
            // poll rate and recover on the next tick — log at debug level so
            // they don't drown the operator. Steady-state failures will show
            // up as stale `timestamp_ns` in `read_data`.
            tracing::debug!(error = %e, "Kinova {call} failed (transient)");
            T::default()
        }
    }
}

fn actuators_to_array(p: &AngularPosition) -> [f32; 6] {
    [
        p.Actuators.Actuator1,
        p.Actuators.Actuator2,
        p.Actuators.Actuator3,
        p.Actuators.Actuator4,
        p.Actuators.Actuator5,
        p.Actuators.Actuator6,
    ]
}
