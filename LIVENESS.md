# Ergot Liveness Tracking & State Notifications

This document describes the new liveness tracking and state notification system added to the local ergot dependency. It's intended to guide implementation of host-side periodic pings and device-side liveness detection in oxifoc.

## What Changed in Ergot

The local ergot at `../ergot/crates/ergot` (branch `liveness-tracking-and-state-notifications`) adds:

### `LivenessConfig`

```rust
use ergot::interface_manager::LivenessConfig;

let liveness = Some(LivenessConfig { timeout_ms: 3000 });
```

Opt-in per-interface. When enabled, the RxWorker tracks incoming frames. If no frame arrives within `timeout_ms`, the interface transitions to `InterfaceState::Down`. Recovery is automatic — when frames resume, `process_frame` transitions back to `Active`.

The timer only starts after the **first frame is received**, so initial connection handshake is not affected.

### `state_notify: Option<Arc<WaitQueue>>`

```rust
use ergot::toolkits::tokio_stream::WaitQueue;
use std::sync::Arc;

let state_notify = Arc::new(WaitQueue::new());
```

Fires (via `wake_all()`) on every interface state transition:
- `Down` → `Inactive` (stream registered)
- `Inactive` → `Active` (first frame sets net_id)
- `Active` → `Down` (liveness timeout or transport error)
- `Down` at worker exit

To wait for state changes:
```rust
loop {
    let _ = state_notify.wait().await;
    let state = stack.manage_profile(|im| im.interface_state(()));
    match state {
        Some(InterfaceState::Active { .. }) => { /* connected */ }
        Some(InterfaceState::Down) => { /* disconnected */ }
        _ => {}
    }
}
```

### Updated `register_*_stream` Signatures

Both `register_target_stream` and `register_controller_stream` now take two additional parameters:

```rust
stream_kit::register_controller_stream(
    stack.clone(),
    reader,
    writer,
    queue,
    Some(LivenessConfig { timeout_ms: 3000 }),  // NEW - pass None to disable
    Some(state_notify.clone()),                  // NEW - pass None to skip notifications
)
```

Pass `None, None` for the old behavior.

## Current State of oxifoc-host-lib

### What's Already Done

1. **Liveness enabled on host (controller) side** with 3s timeout in `run_cobs_stream_with_reconnect`
2. **State monitor** watches `state_notify` and updates `connected_flag: Arc<AtomicBool>`
3. **Reconnection loop** for COBS transports (TCP, Serial, RTT):
   - Interface goes `Down` → `connected_flag = false` → UI shows "Connecting..."
   - Waits up to 10s for recovery (frames resume on existing transport)
   - If recovered → back to "Connected"
   - If 10s expires → tears down transport, reopens, re-registers stream
   - Protocol tasks (telemetry subscribers, command handler) survive reconnections
4. **Serial transport** clears buffers and sends COBS sync byte `[0]` on connect
5. **RTT transport** sends COBS sync byte `[0]` via RTT down channel on attach

### What's NOT Done — Needs Implementation

**The device side cannot detect host disconnection** because the host doesn't send periodic traffic. The host sends a single `DeviceInfo` request at startup, then only listens for telemetry. Without incoming frames, the device's liveness timer would fire immediately.

**Solution needed:** The host should send periodic traffic to the device so the device can track liveness. Two approaches:

#### Approach A: Add Periodic Pings from Host (simple)

Add a ping task to `spawn_protocol_tasks` in `oxifoc-host-lib/src/lib.rs`:

```rust
// Periodic ping to keep device liveness alive
tokio::spawn({
    let stack = stack.clone();
    let cancel_token = cancel_token.clone();
    async move {
        let device_addr = ergot::Address { network_id: 1, node_id: 2, port_id: 0 };
        let mut interval = tokio::time::interval(Duration::from_secs(1));
        loop {
            tokio::select! {
                _ = interval.tick() => {
                    let _ = tokio::time::timeout(
                        Duration::from_millis(500),
                        stack.stack().endpoints().request::<ErgotPingEndpoint>(device_addr, &0u32, None),
                    ).await;
                }
                _ = cancel_token.cancelled() => break,
            }
        }
    }
});
```

This keeps the device's liveness timer alive. If the host dies, pings stop, device detects it.

#### Approach B: Change Slow Telemetry to Polling (better architecture)

Currently both fast and slow telemetry are push-based (device broadcasts via Topics). A better design:

- **Fast telemetry** stays push-based (Topic broadcast) — high frequency, latency-sensitive
- **Slow telemetry** becomes pull-based (Endpoint request/response) — host polls every 100ms

This gives the device regular incoming frames (the poll requests) which naturally keep liveness alive. No separate ping task needed.

**Files to change for Approach B:**
- `oxifoc-core/src/icd.rs` — change `SlowTelemetryTopic` to `SlowTelemetryEndpoint` (request: `()`, response: `SlowTelemetry`)
- `oxifoc-core/src/runtime/streaming.rs` — replace `slow_telemetry_stream` broadcast loop with an endpoint server
- `oxifoc-core/src/runtime/servers.rs` — add `SlowTelemetryEndpoint` server alongside existing servers
- `oxifoc-host-lib/src/lib.rs` — replace slow telemetry subscriber with a polling task that calls `request::<SlowTelemetryEndpoint>` periodically
- All firmware targets that use `slow_telemetry_stream` need updating

### What's Done in oxifoc-virtual

1. **State notifications enabled** (no liveness) — detects host disconnect via TCP EOF
2. **State monitor cancels all tasks** (servers + streaming) when interface goes `Down`
3. **Outer loop** accepts new connections after disconnect — ready for next host

## Key Design Points

- Liveness is **opt-in** — pass `None` to disable
- Timer starts after **first frame received** — doesn't race with initial handshake
- `state_notify` fires **outside the lock** — safe to call `manage_profile` in the handler
- The `connected_flag: Arc<AtomicBool>` in `HostRuntime` is updated by the state monitor — UI polls it via `app.set_is_connected()`
- Protocol tasks (endpoint servers, topic subscribers) survive transport reconnections — they operate on the stack, not the transport
- `WaitQueue` is from `maitake_sync` — works in both `no_std` and `std`
