use std::sync::mpsc::{Receiver, RecvTimeoutError, TryRecvError};
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use super::ffi::{angular_position_point, angular_velocity_point};
use super::sdk::KinovaSdk;
use super::state::KinovaState;

// Telemetry cadence. Idle (no velocity hold) polls aggressively so the UI feels
// responsive. Streaming polls slower so each GetGeneralInformations call
// (~rx_timeout_ms) doesn't starve the velocity resend loop — operators still
// need live joint pos / current while teleoping.
//
// Both calls share the SDK's single UDP socket; serializing them on this one
// worker thread is what keeps responses from aliasing. Don't introduce a
// second SDK-calling thread.
pub const TELEMETRY_INTERVAL_IDLE: Duration = Duration::from_millis(100); // 10 Hz idle
pub const TELEMETRY_INTERVAL_STREAMING: Duration = Duration::from_millis(200); // 5 Hz while streaming

pub const DEFAULT_COMMAND_RATE_HZ: u32 = 100;

pub const STREAM_HINT_INTERVAL: Duration = Duration::from_millis(100);

pub const VELOCITY_HOLD_TIMEOUT: Duration = Duration::from_millis(300);

// When idle, periodically call SetAngularControl to prevent the ARM from
// exiting API control mode after its ~30 s inactivity timeout.
const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(10);

#[derive(Debug, Clone)]
pub enum Cmd {
    SetAngularPosition([f32; 6]),
    SetAngularVelocity([f32; 6]),
    MoveHome,
    EraseTrajectories,
    SetJointZero(i32),
}

pub fn run(
    sdk: KinovaSdk,
    rx: Receiver<Cmd>,
    state: Arc<RwLock<KinovaState>>,
    offsets: [f32; 6],
    command_rate_hz: u32,
) {
    let velocity_resend_interval =
        Duration::from_millis((1000u64 / command_rate_hz.max(1) as u64).max(1));
    if command_rate_hz != DEFAULT_COMMAND_RATE_HZ {
        tracing::info!(
            command_rate_hz,
            resend_interval_ms = velocity_resend_interval.as_millis() as u64,
            "Kinova velocity resend cadence overridden from default 100 Hz"
        );
    }

    let mut last_telemetry = Instant::now() - TELEMETRY_INTERVAL_IDLE;
    let mut held_velocity: Option<[f32; 6]> = None;
    let mut velocity_set_at = Instant::now();
    let mut last_resend = Instant::now() - velocity_resend_interval;
    let mut last_keepalive = Instant::now();
    let mut consecutive_vel_failures: u32 = 0;
    // Numerical-differentiation state for joint_vel. The Kinova firmware caches
    // sub-threshold velocity readings (so GetAngularVelocity returns the last
    // non-zero value indefinitely after motion stops). Diffing successive
    // joint_pos samples gives a true zero when the arm is stationary.
    let mut prev_pos_sample: Option<([f32; 6], Instant)> = None;

    loop {
        let now = Instant::now();

        let mut wake_at;
        if held_velocity.is_some() {
            // While streaming: wake for velocity resend, hold-timeout, *or*
            // the slower telemetry deadline so operators see live data.
            wake_at = (last_resend + velocity_resend_interval)
                .min(velocity_set_at + VELOCITY_HOLD_TIMEOUT)
                .min(last_telemetry + TELEMETRY_INTERVAL_STREAMING);
        } else {
            // Idle: wake for telemetry and keepalive.
            wake_at = (last_telemetry + TELEMETRY_INTERVAL_IDLE)
                .min(last_keepalive + KEEPALIVE_INTERVAL);
        }
        let timeout = wake_at.saturating_duration_since(now);

        match rx.recv_timeout(timeout) {
            Ok(first) => {
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
                        Err(TryRecvError::Empty) | Err(TryRecvError::Disconnected) => break,
                    }
                }

                for cmd in &one_shots {
                    handle_one_shot(&sdk, cmd, &offsets);
                    match cmd {
                        Cmd::SetAngularPosition(_)
                        | Cmd::EraseTrajectories
                        | Cmd::MoveHome => {
                            held_velocity = None;
                        }
                        _ => {}
                    }
                }

                if let Some(v) = latest_velocity {
                    let now = Instant::now();
                    send_velocity(&sdk, v, &mut consecutive_vel_failures);
                    last_resend = now;
                    last_keepalive = now;
                    if v.iter().all(|&x| x == 0.0) {
                        held_velocity = None;
                    } else {
                        held_velocity = Some(v);
                        velocity_set_at = now;
                    }
                }
            }
            Err(RecvTimeoutError::Disconnected) => {
                tracing::info!("Kinova worker: command channel closed, shutting down");
                if held_velocity.is_some() {
                    let _ = sdk.send_basic_trajectory(angular_velocity_point([0.0; 6]));
                }
                break;
            }
            Err(RecvTimeoutError::Timeout) => {}
        }

        let now = Instant::now();

        if let Some(v) = held_velocity {
            if now >= velocity_set_at + VELOCITY_HOLD_TIMEOUT {
                send_velocity(&sdk, [0.0; 6], &mut consecutive_vel_failures);
                tracing::info!(
                    timeout_ms = VELOCITY_HOLD_TIMEOUT.as_millis(),
                    "Kinova velocity hold expired — arm halted"
                );
                held_velocity = None;
                last_resend = now;
                last_keepalive = now;
            } else if now >= last_resend + velocity_resend_interval {
                send_velocity(&sdk, v, &mut consecutive_vel_failures);
                last_resend = now;
                last_keepalive = now;
            }
        } else if now >= last_keepalive + KEEPALIVE_INTERVAL {
            tracing::debug!("Kinova: idle keepalive (SetAngularControl)");
            if let Err(e) = sdk.set_angular_control() {
                tracing::debug!(error = %e, "Kinova: keepalive SetAngularControl failed");
            }
            last_keepalive = now;
        }

        // Telemetry. Idle: 10 Hz. Streaming: 5 Hz, ordered after the velocity
        // resend in this iteration so the next resend deadline is fresh.
        // Only poll if we've already serviced any pending velocity send for
        // this iteration (the resend block above ran or wasn't due) — that
        // keeps SDK calls strictly serialized on this thread without ever
        // missing a resend deadline.
        let telem_interval = if held_velocity.is_some() {
            TELEMETRY_INTERVAL_STREAMING
        } else {
            TELEMETRY_INTERVAL_IDLE
        };
        if now >= last_telemetry + telem_interval {
            poll_telemetry(&sdk, &state, &offsets, &mut prev_pos_sample);
            last_telemetry = Instant::now();
        }
    }
}

/// Send a velocity command.  Returns true if the send failed.
fn send_velocity(sdk: &KinovaSdk, joints: [f32; 6], consecutive_failures: &mut u32) -> bool {
    match sdk.send_basic_trajectory(angular_velocity_point(joints)) {
        Ok(()) => {
            if *consecutive_failures >= 3 {
                // Log recovery from a sustained failure run.
                tracing::info!(after = *consecutive_failures, "Kinova velocity: recovered");
            }
            *consecutive_failures = 0;
            false
        }
        Err(e) => {
            *consecutive_failures += 1;
            // Sporadic single failures are normal on the Ethernet/UDP path —
            // the ARM occasionally takes longer than rx_timeout_ms to ACK a
            // velocity command.  Only warn once a run becomes sustained (≥ 3)
            // to avoid drowning the log at ~75 Hz.
            if *consecutive_failures == 1 {
                tracing::debug!(error = %e, "Kinova velocity send failed (transient)");
            } else if *consecutive_failures == 3 {
                tracing::warn!(consecutive = *consecutive_failures, error = %e,
                    "Kinova velocity send failing — arm may stutter");
            } else if *consecutive_failures > 3 {
                tracing::debug!(consecutive = *consecutive_failures, error = %e,
                    "Kinova velocity still failing");
            }
            true
        }
    }
}

fn handle_one_shot(sdk: &KinovaSdk, cmd: &Cmd, offsets: &[f32; 6]) {
    let result = match cmd {
        Cmd::SetAngularPosition(joints) => {
            let mut adjusted = *joints;
            for i in 0..6 {
                adjusted[i] += offsets[i];
            }
            sdk.send_basic_trajectory(angular_position_point(adjusted))
        }
        Cmd::MoveHome => sdk.move_home(),
        Cmd::EraseTrajectories => sdk.erase_all_trajectories(),
        Cmd::SetJointZero(addr) => sdk.set_joint_zero(*addr),
        Cmd::SetAngularVelocity(_) => unreachable!(),
    };
    if let Err(e) = result {
        tracing::warn!(?cmd, error = %e, "Kinova command failed");
    }
}

fn poll_telemetry(
    sdk: &KinovaSdk,
    state: &Arc<RwLock<KinovaState>>,
    offsets: &[f32; 6],
    prev_pos_sample: &mut Option<([f32; 6], Instant)>,
) {
    // Read pattern follows the working ROS2 wrapper at
    // clubcapra/KinovaArmController. Three SDK calls in sequence, no delays.
    // (Wrapper also reads GetAngularVelocity, but its firmware caches the
    // last non-zero value below the encoder threshold — joints stay frozen
    // on the last commanded velocity after motion stops. We derive vel from
    // successive position samples instead, which gives a true zero at rest.)
    let pos_res = sdk.get_angular_position();
    let cur_res = sdk.get_angular_current();
    let gen_res = sdk.get_general_informations();

    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0);

    let mut s = state.write().unwrap();

    if let Ok(pos) = pos_res {
        let a = &pos.Actuators;
        let raw = [a.Actuator1, a.Actuator2, a.Actuator3, a.Actuator4, a.Actuator5, a.Actuator6];
        if raw.iter().all(|&v| v.is_finite()) && !raw.iter().all(|&v| v == 0.0) {
            let mut joint_pos = raw;
            for i in 0..6 { joint_pos[i] -= offsets[i]; }
            let now_inst = Instant::now();

            // Differentiate against the previous fresh sample. dt is the
            // wall-clock elapsed since the last update — typically the
            // telemetry interval (100 ms idle / 200 ms streaming) but can
            // be longer if a previous SDK call failed and we skipped a tick.
            if let Some((prev_pos, prev_t)) = *prev_pos_sample {
                let dt = now_inst.duration_since(prev_t).as_secs_f32();
                if dt > 0.0 {
                    for i in 0..6 {
                        s.joint_vel[i] = (joint_pos[i] - prev_pos[i]) / dt;
                    }
                }
            }
            *prev_pos_sample = Some((joint_pos, now_inst));

            s.joint_pos = joint_pos;
            s.timestamp_ns = now_ns;
        }
    }

    if let Ok(cur) = cur_res {
        let a = &cur.Actuators;
        let raw = [a.Actuator1, a.Actuator2, a.Actuator3, a.Actuator4, a.Actuator5, a.Actuator6];
        if raw.iter().all(|&v| v.is_finite()) {
            s.joint_current = raw;
        }
    }

    if let Ok(gen) = gen_res {
        let temps = [
            gen.ActuatorsTemperatures[0], gen.ActuatorsTemperatures[1],
            gen.ActuatorsTemperatures[2], gen.ActuatorsTemperatures[3],
            gen.ActuatorsTemperatures[4], gen.ActuatorsTemperatures[5],
        ];
        if temps.iter().all(|&v| v.is_finite()) {
            s.joint_temp = temps;
        }
        if gen.AccelerationX.is_finite() && gen.AccelerationY.is_finite() && gen.AccelerationZ.is_finite() {
            s.accel_x = gen.AccelerationX;
            s.accel_y = gen.AccelerationY;
            s.accel_z = gen.AccelerationZ;
        }
        if gen.SupplyVoltage.is_finite() && gen.SupplyVoltage > 10.0 && gen.SupplyVoltage < 40.0 {
            s.bus_voltage = gen.SupplyVoltage;
        }
        if gen.TotalCurrent.is_finite() && gen.TotalCurrent >= 0.0 && gen.TotalCurrent < 30.0 {
            s.bus_current = gen.TotalCurrent;
        }
    }
}
