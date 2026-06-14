# rove_control_bridge

Operator-to-hardware translator. Takes teleop intent over UDP, converts it to
per-actuator commands, and ships those to **rove_sensor_api** (the core robot
API that owns the hardware) and **rove_ik_engine** (the simulation /
inverse-kinematics layer that drives the arm).

```
┌──────────────────┐        ┌──────────────────────┐
│ capra_steamdeck  │ UDP    │ rove_control_bridge  │
│  teleop_interface│ ─────► │  (this package)      │
│  RoveControl.proto        │                      │
└──────────────────┘        └──────────┬───────────┘
                                       │
                ┌──────────────────────┼──────────────────────────┐
                │                      │                          │
                ▼                      ▼                          ▼
     ┌───────────────────┐  ┌───────────────────┐  ┌────────────────────┐
     │ rove_sensor_api   │  │ rove_sensor_api   │  │ rove_ik_engine     │
     │  ODrive cmd ports │  │  Robotiq gripper  │  │  IK + collision /  │
     │  (tracks/flippers)│  │  cmd port         │  │  world model       │
     └───────────────────┘  └───────────────────┘  └─────────┬──────────┘
                                                             │
                                                             │ (future)
                                                             ▼
                                                  ODrive cmd ports
                                                  for arm joints
```

`rove_sensor_api` exposes per-sensor command UDP ports and advertises them via
HTTP `/discover`; the bridge resolves the ports at startup and dispatches
fire-and-forget command packets per tick.

`rove_ik_engine` will, over time, take ownership of tracks + flippers
conversion too (collision-aware control against the world model). At that
point this bridge's job shrinks to receive → parse → forward.

---

## Quick start

```sh
# Recommended on a Jetson — bootstraps venv, compiles protos, runs the bridge:
./scripts/run.sh

# Install as a boot-time systemd service:
sudo ./scripts/install.sh

# Manual / dev: build protos once, then run the module by hand.
python scripts/build_protos.py
python -m rove_control_bridge --config rove_control_bridge/config/default.yaml
```

`python -m rove_control_bridge --help` lists CLI overrides that take precedence
over anything in the YAML file.

## Module layout

```
rove_control_bridge/
├── __main__.py          CLI entry: parse args, load config, call bridge.start()
├── requirements.txt     Python deps (used by scripts/run.sh's venv bootstrap)
├── bridge/              Runtime
│   ├── core.py          RoveControlBridge: receive loop, IDLE/ACTIVE state, dispatch
│   └── factory.py       build_strategy(cfg) + start(cfg) orchestration
├── config/              Configuration
│   ├── schema.py        Typed dataclasses (one per YAML section)
│   ├── loader.py        load(path) → BridgeConfig
│   └── default.yaml     Shipped operator defaults
├── strategies/          RoveControl → ODrive command translators
│   ├── base.py          ConversionStrategy ABC + NodeCommand dataclass
│   ├── tracks_velocity  All drums in velocity mode
│   ├── tracks_torque    All drums in torque mode
│   └── tracks_mixed     Per side: velocity governor + one torque drum
├── transport/           Outbound I/O
│   ├── sensor_api_client.py  JSON-over-UDP client for rove_sensor_api
│   ├── discovery.py     Resolve sensor command ports via /discover
│   └── ovis_forwarder.py     Re-wrap + UDP arm twist to rove_ik_engine
├── scripts/             Ops glue (not imported by the package)
│   ├── run.sh           Jetson launcher: venv + protos + python -m
│   ├── install.sh       Register rove-control-bridge.service in systemd
│   ├── uninstall.sh     Remove the systemd service
│   └── build_protos.py  Regenerate proto/ from .proto sources
└── proto/               Generated protobuf (do not edit by hand)
```

## What happens on startup

1. `__main__` loads YAML config, applies CLI overrides, calls `bridge.start(cfg)`.
2. `start()` hits `rove_sensor_api`'s `/discover` HTTP endpoint to learn the
   command port for every ODrive node and (optionally) the gripper.
3. `build_strategy(cfg)` picks a `ConversionStrategy` based on
   `tracks.strategy` (`velocity`, `torque`, or `mixed`).
4. If `ovis.enabled`, an `OvisForwarder` is opened to `rove_ik_engine`.
5. `RoveControlBridge.run()` enters the receive loop.

## What happens per packet

1. Bind UDP socket on `listen.host:listen.port`, parse the `RoveControl` proto.
2. On the first packet after silence, transition `IDLE → ACTIVE`.
3. `strategy.convert(msg)` returns a `list[NodeCommand]` — one entry per
   ODrive node that needs a setpoint.
4. Each `NodeCommand` is sent to its discovered command port via
   `SensorApiUdpClient`.
5. `msg.gripper.position` is forwarded to the Robotiq command port (only when
   the value changes, to avoid spamming).
6. `msg.ovis` is forwarded to `rove_ik_engine` if the forwarder is open.

## IDLE / ACTIVE state machine

- **IDLE**: ODrives at `axis_state=1`. No setpoints flowing.
- **ACTIVE**: every `convert()` payload carries `axis_state=8`
  (`ClosedLoopControl`), so the very first packet after silence arms the
  drives and starts moving.
- Watchdog thread checks each tick; after `idle_timeout_s` of silence
  (default 0.5 s) the bridge sends `estop()` commands and drops back to IDLE.

## Adding a new strategy

1. Drop a file in `strategies/` defining a subclass of `ConversionStrategy`
   with `name`, `initialize()`, `convert()`, `zero_commands()`, `estop()`.
2. Export it from `strategies/__init__.py`.
3. Add a branch in `bridge/factory.py::build_strategy()`.
4. Document it in `config/default.yaml`'s `tracks.strategy` comment.

## Common pitfalls

- **"No ODrive sensors found in /discover"** — `rove_sensor_api` isn't
  running or no ODrives are on the CAN bus.
- **"Protobuf generated files not found"** — run `scripts/build_protos.py` first (or use `scripts/run.sh`, which does this automatically).
- **Robot doesn't move at full stick** — see the ODrive `current_soft_max`
  and `enable_torque_mode_vel_limit` settings; firmware-side clamps will
  silently attenuate setpoints below what the bridge sends.
