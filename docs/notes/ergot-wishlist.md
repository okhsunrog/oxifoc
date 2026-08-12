# ergot wishlist — from the oxifoc integration

Status: running list, updated as oxifoc work exposes friction. Cross-repo
companion to [protocol-versioning.md](protocol-versioning.md) (whose §3-§5
items — L1 wire version, L2 introspection, `#[schema(evolve)]` — remain the
versioning-side entries and are not repeated here). Collected 2026-08-12
during the architecture review + hardening session.

Each entry: the pain as observed in oxifoc, then the upstream shape.

## High value

1. **Message-sized grants in `cobs_stream::Sink`.** `send_ty` does
   `grant_exact(max_encoding_length(MTU)+1)` regardless of the actual
   message size: a 40-byte command needs ~2057 contiguous free bytes in the
   bridge's 4096-byte UART queue, so the queue effectively holds 1–2 frames
   and one large in-flight frame starves every subsequent send into
   `InterfaceFull` (head-of-line blocking a size-aware grant would avoid).
   Upstream shape: serialize to measure (or grant with `grant_max_remaining`
   and shrink on commit), reserving only what the frame needs.

2. **Built-in request timeout.** `Endpoints::request()` awaits forever;
   every consumer wraps it by hand (bridge ping 750 ms, host handshake
   800 ms, core delivery ladder races a Timer, and the 2026-08-12 motor-ack
   fix in oxifoc-core does the same select dance). Upstream shape: a
   `request_with_deadline` taking an implementation of the same Timer-style
   trait core uses, or a first-class `deadline` param on the client handle.

3. **Edge net-id discovery without an app-layer keep-alive.** The
   `EdgeFrameProcessor` can only learn its net_id from an inbound frame
   addressed to it, so oxifoc-bridge runs `upstream_link_task` — a 2 s ping
   loop whose only real job is to provoke that frame — and the f405 serves
   `ping_handler` mostly to answer it. Upstream shape: a built-in periodic
   hello/solicitation on edge interfaces (tiny frame, off by default, one
   config knob), which would also give liveness a TX-side signal.

4. **Split "link dead" from "not yet discovered".**
   `revert_to_link_local_on_timeout()` folds both into
   `Active { net_id: 0 }`, so the bridge keeps its own
   `upstream_discovered()` predicate. Upstream shape: either a distinct
   `AwaitingDiscovery` interface state or an explicit `discovered: bool` in
   the Active payload.

5. **Multi-hop address resolution for clients.** The host hardcodes
   `{net 0, node 1, port 0}` — correct point-to-point, wrong through the
   bridge (over BLE that address IS the bridge, not the motor controller;
   nothing in host-lib can currently reach the STM32 behind it).
   `discover_sockets` exists; what's missing is the convenience:
   "resolve the full Address of the (unique) socket serving key K anywhere
   on the net, with a deadline". With that, host-lib's `DEVICE_ADDR` becomes
   a resolved value instead of a constant, and the BLE topology starts
   working end-to-end. Refusal on ambiguity (two controllers serving the
   same key) should be the default, mirroring oxifoc's USB identity pinning.

## Medium value

6. **Seed-lease scheduling.** ergot provides `bridge_seed_assign` /
   `bridge_seed_refresh` / `release_seed_lease` but no scheduler; the bridge
   hand-rolls the refresh deadline math (including the subtle
   "recompute-per-iteration starves the refresh under traffic" fix), the
   re-assign fallback and the bounded acquire loop. Upstream shape: a
   `run_seed_lease(stack, upstream_ident, ident) -> impl Future` worker that
   owns the whole lifecycle and exposes state via a watch.

7. **QoS: two traffic classes per interface.** One bounded queue per
   interface means a saturated telemetry stream delays command responses
   behind it (the book documents this honestly). Full priority queues are
   likely overkill; two classes (small/control vs bulk) with the TX worker
   draining control first would remove the worst failure mode. oxifoc
   mitigates today by keeping telemetry opt-in and batches small.

8. **Local broadcast NoSpace observability.** A broadcast into a
   full subscriber queue counts as delivered (`inner.rs` treats
   `SocketSendError::NoSpace` as success for broadcasts), so a saturated
   consumer is invisible to the producer. Upstream shape: per-socket drop
   counters readable via the stack (or a `BroadcastOutcome { delivered,
   dropped }` return) — oxifoc counts drops app-side at every stage, but the
   stack layer is a blind spot in the middle.

9. **Uniform MTU error for locally-originated sends.** Forwarded frames get
   the diagnostic `PacketTooBig { mtu }`; locally-originated typed sends
   that exceed the sink MTU surface as a generic `InterfaceFull` from the
   serializer. Same condition, two errors — the local path should also say
   "too big, mtu is N".

## Small

10. **Port exhaustion should not panic.** `alloc_port` panics when all 254
    ports are taken; a stack under socket churn should get `Err` and let the
    caller shed load.

11. **`NetStackHandle` for `&&NetStack` ergonomics.** Service free functions
    called with `&'static Stack` end up passing `&&NetStack`
    (`bridge_seed_assign(&stack, ...)` all over the bridge). Blanket impl or
    by-value handles would tidy every call site.

## Landed (kept for the record)

- **Split endpoint request API** (`send_request` + `recv`, `attach_boxed`)
  — ergot branch `split-request-api`, rev `e93b03b`, born from the
  2026-07-06 deadman hunt: awaiting a request inline in the affirm loop
  delayed the next affirm past the device's 150 ms deadman. oxifoc rides it
  since `2f4365b`.
- **`edge_link_local()` + revert-to-link-local liveness policy** (#221,
  rev `5a5fab8`) — adopted in oxifoc 2026-08-06; its "dead vs undiscovered"
  ambiguity is entry 4 above.
- **`wait_for_value` lost-wakeup guidance** in the reliability book
  (documented in `e93b03b` after the g474 state_monitor edge bug).
