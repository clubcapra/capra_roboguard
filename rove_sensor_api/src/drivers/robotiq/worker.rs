//! Background tokio task that owns the Modbus context.
//!
//! Single owner because `tokio_modbus::client::Context` is `!Sync` — every
//! request goes through this task via the `Cmd` mpsc channel. Between
//! commands the task polls status registers at the configured interval and
//! updates the shared `RobotiqState`.

use std::sync::{Arc, RwLock};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use tokio::sync::mpsc;
use tokio::time::{Instant, interval_at};
use tokio_modbus::prelude::*;

use super::state::RobotiqState;

/// Modbus register ranges (Robotiq 2F-140 manual, "Robot Output" / "Robot Input").
pub const OUTPUT_BASE: u16 = 0x03E8; // commands: write 3 holding regs here
pub const INPUT_BASE: u16 = 0x07D0; // status: read 3 holding regs here

/// Clamp + rebuild the 6-byte command frame and pack it into 3 Modbus regs.
///
/// Layout (Robotiq manual):
///   reg0 hi: ACTION_REQUEST = rACT | rGTO<<3 | rATR<<4 | rARD<<5
///   reg0 lo: reserved
///   reg1 hi: reserved
///   reg1 lo: rPR (position request)
///   reg2 hi: rSP (speed)
///   reg2 lo: rFR (force)
fn pack_command(act: bool, gto: bool, atr: bool, ard: bool, pr: u8, sp: u8, fr: u8) -> [u16; 3] {
    let action: u8 =
        (act as u8) | ((gto as u8) << 3) | ((atr as u8) << 4) | ((ard as u8) << 5);
    [
        u16::from_be_bytes([action, 0]),
        u16::from_be_bytes([0, pr]),
        u16::from_be_bytes([sp, fr]),
    ]
}

/// Decode the 3 status registers into `RobotiqState` fields. See manual,
/// "Robot Input Registers".
fn decode_status(regs: &[u16]) -> Option<DecodedStatus> {
    if regs.len() < 3 {
        return None;
    }
    let gripper_status = (regs[0] >> 8) as u8;
    let fault_byte = (regs[1] >> 8) as u8;
    Some(DecodedStatus {
        activated: gripper_status & 0x01 != 0,
        going_to_position: (gripper_status >> 3) & 0x01 != 0,
        status: (gripper_status >> 4) & 0x03,
        object_status: (gripper_status >> 6) & 0x03,
        fault: fault_byte & 0x0F,
        position_request_echo: (regs[1] & 0xFF) as u8,
        position: (regs[2] >> 8) as u8,
        current_raw: (regs[2] & 0xFF) as u8,
    })
}

struct DecodedStatus {
    activated: bool,
    going_to_position: bool,
    status: u8,
    object_status: u8,
    fault: u8,
    position_request_echo: u8,
    position: u8,
    current_raw: u8,
}

/// Commands posted by `RobotiqGripper::execute_command` and consumed by the
/// worker. `position`, `speed`, and `force` are merged into the worker's
/// held setpoints, so a packet only needs to send what changed.
#[derive(Debug, Clone, Default)]
pub struct GripperCommand {
    pub position: Option<u8>,
    pub speed: Option<u8>,
    pub force: Option<u8>,
    /// `Some(true)` = rGTO=1 (go), `Some(false)` = rGTO=0 (stop), `None` =
    /// keep current.
    pub goto: Option<bool>,
    /// `Some(true)` = rACT=1 (activate / re-activate), `Some(false)` =
    /// rACT=0 (deactivate / reset). Activating from a fault state requires
    /// a 0→1 edge, which the caller can issue via two packets.
    pub activate: Option<bool>,
    /// `Some(0)` = closing emergency release, `Some(1)` = opening, `None` =
    /// disarm. Setting this asserts rATR.
    pub auto_release: Option<u8>,
}

#[derive(Debug)]
pub enum Cmd {
    Apply(GripperCommand),
    /// Emergency stop: clears rGTO so the gripper holds at current position.
    /// Does *not* assert rATR (use `auto_release` for that).
    Stop,
}

/// Held setpoints — the worker keeps the most recent values so partial
/// updates don't reset position/speed/force to zero.
#[derive(Debug, Clone, Copy)]
struct Setpoints {
    activate: bool,
    goto: bool,
    auto_release: bool,
    auto_release_dir: bool,
    position: u8,
    speed: u8,
    force: u8,
}

impl Default for Setpoints {
    fn default() -> Self {
        // Mid speed, mid force, fully open on startup. Conservative defaults
        // — a missing field in the first command packet won't surprise the
        // operator with max-force squeezing.
        Self {
            activate: false,
            goto: false,
            auto_release: false,
            auto_release_dir: false,
            position: 0,
            speed: 128,
            force: 64,
        }
    }
}

impl Setpoints {
    fn merge(&mut self, c: &GripperCommand) {
        if let Some(p) = c.position {
            self.position = p;
        }
        if let Some(s) = c.speed {
            self.speed = s;
        }
        if let Some(f) = c.force {
            self.force = f;
        }
        if let Some(g) = c.goto {
            self.goto = g;
        }
        if let Some(a) = c.activate {
            self.activate = a;
        }
        if let Some(d) = c.auto_release {
            self.auto_release = true;
            self.auto_release_dir = d != 0;
        }
    }

    fn packed(&self) -> [u16; 3] {
        pack_command(
            self.activate,
            self.goto,
            self.auto_release,
            self.auto_release_dir,
            self.position,
            self.speed,
            self.force,
        )
    }
}

pub struct WorkerHandle {
    pub cmd_tx: mpsc::UnboundedSender<Cmd>,
}

/// Spawn the worker task. Takes ownership of the Modbus context.
pub fn spawn(
    mut ctx: client::Context,
    state: Arc<RwLock<RobotiqState>>,
    poll_interval: Duration,
    auto_activate: bool,
    activation_timeout: Duration,
) -> WorkerHandle {
    let (cmd_tx, mut cmd_rx) = mpsc::unbounded_channel::<Cmd>();
    let mut setpoints = Setpoints::default();

    tokio::spawn(async move {
        if auto_activate {
            // Robotiq requires a 0→1 edge on rACT to activate. If the gripper
            // was already activated (or stuck mid-activate) when we attached,
            // sending rACT=1 alone is a no-op and gSTA never moves to 3.
            // Reset first, wait for gSTA to drop to 0, then assert rACT=1.
            setpoints.activate = false;
            setpoints.goto = false;
            if let Err(e) = write_command(&mut ctx, setpoints).await {
                tracing::warn!(error = %e, "Robotiq: deactivate write failed");
            }
            if let Err(e) = wait_for_status(&mut ctx, &state, 0, Duration::from_millis(500)).await
            {
                tracing::debug!(error = %e, "Robotiq: deactivate wait timed out (continuing)");
            }

            setpoints.activate = true;
            if let Err(e) = write_command(&mut ctx, setpoints).await {
                tracing::warn!(error = %e, "Robotiq: activate write failed");
            } else if let Err(e) =
                wait_for_status(&mut ctx, &state, 3, activation_timeout).await
            {
                let s = state.read().unwrap().clone();
                tracing::warn!(
                    error = %e,
                    gSTA = s.status,
                    gACT = s.activated,
                    gFLT = s.fault,
                    gPO = s.position,
                    "Robotiq: activation did not complete in time",
                );
            }
        }

        let start = Instant::now() + poll_interval;
        let mut tick = interval_at(start, poll_interval);

        loop {
            tokio::select! {
                _ = tick.tick() => {
                    if let Err(e) = poll_status(&mut ctx, &state).await {
                        tracing::warn!(error = %e, "Robotiq: status read failed");
                    }
                }
                maybe_cmd = cmd_rx.recv() => {
                    let Some(cmd) = maybe_cmd else { break; };
                    match cmd {
                        Cmd::Apply(c) => setpoints.merge(&c),
                        Cmd::Stop => setpoints.goto = false,
                    }
                    if let Err(e) = write_command(&mut ctx, setpoints).await {
                        tracing::warn!(error = %e, "Robotiq: command write failed");
                    }
                }
            }
        }

        tracing::info!("Robotiq worker shutting down");
        let _ = ctx.disconnect().await;
    });

    WorkerHandle { cmd_tx }
}

async fn write_command(
    ctx: &mut client::Context,
    s: Setpoints,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let regs = s.packed();
    ctx.write_multiple_registers(OUTPUT_BASE, &regs).await??;
    Ok(())
}

async fn poll_status(
    ctx: &mut client::Context,
    state: &Arc<RwLock<RobotiqState>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let regs = ctx.read_holding_registers(INPUT_BASE, 3).await??;
    let Some(d) = decode_status(&regs) else {
        return Ok(());
    };
    let now_ns = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0);
    let mut s = state.write().unwrap();
    s.activated = d.activated;
    s.going_to_position = d.going_to_position;
    s.status = d.status;
    s.object_status = d.object_status;
    s.fault = d.fault;
    s.position_request_echo = d.position_request_echo;
    s.position = d.position;
    s.current_raw = d.current_raw;
    s.timestamp_ns = now_ns;
    s.link_up = true;
    Ok(())
}

/// Spin on status reads until `gSTA == target` or the timeout elapses.
/// Polls at 50 ms regardless of the configured cadence so startup feels
/// snappy.
async fn wait_for_status(
    ctx: &mut client::Context,
    state: &Arc<RwLock<RobotiqState>>,
    target: u8,
    timeout: Duration,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        poll_status(ctx, state).await?;
        if state.read().unwrap().status == target {
            tracing::info!(gSTA = target, "Robotiq: reached target status");
            return Ok(());
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    Err(format!("timed out waiting for gSTA={target}").into())
}
