# Adding hardware to HALo

Add a driver, an instance TOML configuration and one registration block in the
driver catalogue. Shared core, HTTP, UDP and console code should not change.
The Rove simulator is a robot adapter, not the template for physical hardware.

## 1. Driver and configuration

Create `src/drivers/<family>/` with `config.rs`, the `SensorDriver` implementation
and its transport worker. Export it from `src/drivers/mod.rs`.

Configuration follows the same contract used by all physical drivers:

```rust
#[derive(serde::Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub port: String,
}

impl halo::core::config::DriverConfig for Config {
    fn validate(&self) -> Result<(), String> {
        if self.port.is_empty() { Err("port is required".into()) } else { Ok(()) }
    }
    // Optional: resolve_paths(base) for instance-relative files.
    // Optional: reserved_udp_ports() for local ports used by the hardware SDK.
}
```

Inside HALo, import the trait through `crate::core::config`. Use
`load_driver_config::<Config>("family.toml")` on every connection attempt;
`parse_driver_config` applies the same validation when an operator edits TOML.
Missing configuration disables the configured family; malformed configuration
must report an error. Ship `config/family.toml.example`, then provision
`family.toml` for each actual installation. Hardware addresses belong there.
Translate vendor formats privately inside the driver (as Livox does for SDK JSON).

Every driver supplies a stable `id`, display name, data and command schemas,
command mode, cached `read_data` and validated `execute_command` implementation.
Identity must describe the same hardware after reconnecting; avoid discovery-order
indexes or mutable device paths. The catalogue can supply a configured identity
when registering the driver.

Expose real hardware readiness: opening a serial port or SDK controller is not
proof that actuators or sensors are operational. Override `readiness` for device
initialization, `telemetry_status` for sample freshness/link state/vendor faults
and `diagnostics` for worker errors. Convert vendor errors into readable messages
and retain numeric codes or unknown bits for diagnosis. Use source sample times,
not the time a cached value was read by HTTP.

Command descriptors declare types, units, `requiring_ready`,
`requiring_ready_for`, `standalone` and optional `confirm_for` consequences.
The generic console generates fields from these declarations. The driver still
validates ranges, coupled parameters and device behavior. Every moving driver
implements `has_estop` and `estop`; optional `estop_confirmation` describes the
physical consequence. Stop commands must remain usable before full readiness.

Transport workers belong to the driver. Implement `shutdown` to cancel and join
them and close native callbacks before releasing SDK/library memory. The
supervisor owns retries and prevents overlapping initializations after timeouts.
Keep `read_data` nonblocking; serialize non-reentrant SDK calls in the worker.

## 2. Catalogue registration

Register the connection factory in `src/drivers/loader.rs`:

```rust,ignore
manager.add("family", std::sync::Arc::new(|| {
    Box::pin(async {
        let cfg = crate::core::config::load_driver_config::<Config>("family.toml")
            .map_err(|error| error.to_string())?
            .ok_or("disabled: no family.toml")?;
        let driver = connect(cfg).await.map_err(|error| error.to_string())?;
        Ok(vec![(driver.id().to_owned(), std::sync::Arc::new(driver)
            as std::sync::Arc<dyn crate::core::driver::SensorDriver>)])
    })
}))?;
configure::<Config>(manager, "family", "family.toml")?;
```

A factory may return several devices sharing one CAN bus or SDK session. These
devices form one retry group; independent physical connections use separate
groups. Reserve configured identities before asynchronous probing when possible.
For an SDK binding fixed local UDP ports, include the configuration's
`reserved_udp_ports` in `DriverCatalog::reserved_udp_ports` before allocation.
All of these changes stay in driver/catalogue code.

HALo then supplies health, bounded recovery, logs, persistent served port pairs,
discovery, OpenAPI, dynamic HTTP routes, UDP subscriptions and generated console
tabs. Do not add device-name branches to those layers. Saved ports survive
hardware removal and additions; changing a factory or base never renumbers them.

## 3. Source updates and optional capabilities

Most drivers use cached snapshots. High-rate sources declare
`DataDelivery::Updates` and return a broadcast receiver from `subscribe_updates`.
Publish `DataUpdate::Data` with bounded JSON payloads. HALo handles multiple
subscribers, acknowledgements, sequence numbers, lag reporting and reconnects.
Do not put consumer destinations in the core or console. Existing proprietary
compatibility mirrors, if required, remain optional and private to their driver.

Use the standard configuration, calibration, endpoint and save capabilities
where appropriate. A vendor compatibility route that cannot use those contracts
can be returned as a `DriverHttpExtension` from the catalogue; the extension owns
its router and OpenAPI documentation. Generic HTTP code just merges them.

## Verify the driver

Test controller-connected/hardware-absent behavior, stale telemetry, readable
known/unknown faults, estop and explicit recovery, worker shutdown and reconnect.
Check discovery schemas and port continuity through the standard interface.
`tests/halo_lifecycle.rs` registers an unbranded driver/configuration fixture
without extending the shared layer; `tests/test_sim_flow.py` validates the running
service and offline console against real simulator telemetry.
