# TODO

Items to implement. Remove each item after completion.

---

## Ergot PRs (upstream contributions)

### Generic COBS-over-AsyncRead/AsyncWrite helper

The same COBS RX/TX worker pattern is duplicated in oxifoc-host-lib, oxifoc-virtual, and ergot's own TCP/serial toolkits (~80 lines each). A generic helper would eliminate all of them.

Proposed API in `ergot::toolkits::tokio_stream` (new module):

```rust
pub async fn register_controller_stream<N: NetStackHandle>(
    stack: &N,
    reader: impl AsyncRead + Unpin + Send + 'static,
    writer: impl AsyncWrite + Unpin + Send + 'static,
    queue: &StdQueue,
) -> Result<(), SocketAlreadyActive>
```

- Takes any `AsyncRead/AsyncWrite` pair (TCP socket halves, serial port, etc.)
- Spawns COBS accumulator RX worker + stream consumer TX worker internally
- Sets interface to `Active` immediately (controller mode — host initiates)
- A `register_target_stream` variant sets `Inactive` and transitions on first incoming packet

This replaces the manual `CobsAccumulator` + `feed_raw` + `process_frame` loop and the `stream_consumer().wait_read()` + `write_all` loop that every transport currently implements by hand.

After this lands, oxifoc-host-lib's `backend_main` drops from ~120 lines to ~20 lines, and oxifoc-virtual's tcp_server.rs similarly simplifies.

### Clone for Endpoints

`Endpoints<NS>` wraps an `NS: NetStackHandle` which requires `Clone`, but `Endpoints` itself doesn't implement `Clone`. This prevents writing generic client functions that need to pass endpoints to multiple spawned tasks.

Fix: add `Clone` impl (or derive) on `Endpoints<NS>`. One-line change:

```rust
impl<NS: NetStackHandle> Clone for Endpoints<NS> {
    fn clone(&self) -> Self {
        Self { inner: self.inner.clone() }
    }
}
```

Same for `Topics<NS>` and `Services<NS>` if they have the same issue.

### Controller-side TCP toolkit

Currently `ergot::toolkits::tokio_tcp` only has `new_target_stack` + `register_target_interface` (edge/target, node 2, starts Inactive). Host tools that initiate communication need a controller variant (node 1, starts Active).

Add:
- `new_controller_stack(queue, mtu) -> EdgeStack` — creates node 1 controller with `InterfaceState::Active`
- `register_controller_interface(stack, socket, queue)` — sets up RX/TX workers without requiring an incoming packet for activation

This would be unnecessary if the generic stream helper (above) lands, since that works for any transport.

---

## Host tools

### UDP and USB transports

Add UDP and USB transport backends to `oxifoc-host-lib/src/transport/`. Follow the same `AsyncRead/AsyncWrite` pattern as TCP/serial.

**UDP**: Ergot has `tokio_udp` toolkit. UDP is framed (not COBS-streamed), so it needs a different code path. The `register_edge_interface` for UDP uses `framed_consumer` instead of `stream_consumer`. May need ergot's toolkit directly rather than the generic AsyncRead approach.

**USB**: Ergot has `nusb_v0_1` toolkit but only as Router. For host connecting to a USB device, need either:
- `nusb` bulk endpoint → wrap as `AsyncRead/AsyncWrite` → use COBS stream (same as serial)
- Or a new ergot toolkit for USB edge/controller

USB is lower priority since serial-over-USB (ttyACM) already works for all current devices.

### Host-lib refactor after ergot PRs

After the generic stream helper and Endpoints Clone land in ergot:

1. Replace manual COBS RX/TX workers in `backend_main` with `register_controller_stream()`
2. Extract client logic into `async fn run_client(endpoints: Endpoints<NS>, ...)` — generic over stack type
3. TCP, serial, RTT all share the same client code path
4. Remove ~80 lines of COBS boilerplate

### GUI: TCP connect option

Replace the removed "Simulate" checkbox in oxifoc-host-slint with a TCP connection option. The UI should have a text field for host:port (default "127.0.0.1:2025") alongside the existing serial/probe selection. When TCP is selected, `start_host` is called with `TransportType::Tcp` config.

This lets the GUI connect to `oxifoc-virtual` for development without hardware.

---

## Virtual device

### Motor control E2E test

Test motor start/stop through the full protocol path:

```
oxifoc-host-cli --transport tcp start --duty 50
# verify: virtual motor spins (omega_e > 0 in telemetry)
oxifoc-host-cli --transport tcp stop
# verify: motor stops (omega_e ≈ 0)
```

Can be a shell script or a Rust integration test. For CI, spawn `oxifoc-virtual` as a background process, run CLI commands, verify output.

### Config round-trip E2E test

Test config write/read through the protocol:

```
oxifoc-host-cli --transport tcp config write pi-gains --kp 0.5 --ki 50.0
oxifoc-host-cli --transport tcp config read pi-gains
# verify: kp=0.5, ki=50.0
oxifoc-host-cli --transport tcp config reset-all
oxifoc-host-cli --transport tcp config read pi-gains
# verify: NotFound (defaults)
```

Requires adding `config` subcommand to CLI (see below).

### ADC current values in telemetry

Currently the virtual device sends `ia=0, ib=0, ic=0` in ADC snapshots (raw ADC counts are zero because there's no real ADC). Should synthesize ADC-like values from the virtual motor's phase currents so telemetry is more realistic. Convert `out.ia/ib/ic` (Amps) to synthetic ADC counts using the board's shunt/gain parameters.

---

## CLI

### Config subcommand

Add `config` subcommand to `oxifoc-host-cli` for reading/writing persistent configuration:

```
oxifoc-host-cli config read <group>
oxifoc-host-cli config write <group> [fields...]
oxifoc-host-cli config reset-all
```

Groups: motor-params, hall-calibration, dc-offsets, current-limits, voltage-limits, pwm-config, pi-gains, hall-tuning.

Uses the `ConfigEndpoint` (cmd/config) added in Phase 4. Requires `HostCommand` variants for config operations and corresponding handling in the host-lib backend.

---

## CI

### Virtual device integration tests

Add a CI job that:
1. Builds `oxifoc-virtual` and `oxifoc-host-cli`
2. Starts `oxifoc-virtual` in background
3. Runs CLI commands: `list`, `monitor --seconds 1`, `start --duty 10`, `stop`
4. Verifies exit codes and output patterns
5. Stops virtual device

This tests the full protocol stack in CI without hardware. Can be a `just test-integration` recipe and a GitHub Actions job.

---

## Config storage

### Apply stored current/voltage limits at runtime

Phase 3 applies stored PI gains and motor params at boot, but `current_limits` and `voltage_limits` from `RuntimeConfig` aren't used yet. The fault detection functions (`check_current_faults`, `check_voltage_faults`) read limits from `BoardConfig` directly. Need to:

1. Make fault check functions accept limit values as parameters (instead of reading from BoardConfig)
2. Or store effective limits in a mutable global that the ISR reads
3. Apply stored limits from RuntimeConfig at boot
4. Update limits when config is written via protocol

### Apply stored Hall calibration at boot

If `RuntimeConfig.hall_calibration` is `Some`, apply the calibration to `HallSensor` during FOC init. Currently the stored value is loaded but not applied — need to call `hall_sensor.apply_calibration()` or equivalent with the stored sector angles.

### Apply stored Hall tuning at boot

If `RuntimeConfig.hall_tuning` is `Some`, apply interpolation parameters to `HallSensor` during init: `set_interp_min_erpm`, `set_drift_correction_gain`, `set_rate_limit_factor`, `set_timeout_us`.
