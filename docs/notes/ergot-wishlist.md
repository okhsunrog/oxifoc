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

3. **Edge net-id discovery via solicit, not keep-alive.** (Reframed
   2026-08-13 — the earlier "hello service" phrasing conflated two needs.)
   The `EdgeFrameProcessor` can only learn its net_id from an inbound
   frame addressed to it and ergot generates no traffic itself, so
   discovery parasitizes on application traffic: oxifoc-bridge runs
   `upstream_link_task` — a 2 s ping loop, forever, even once discovered —
   and every central must remember to serve `ping_handler` (the g474
   didn't; a bridge behind it could never discover its upstream).
   Upstream shape: an *event-driven* solicit — after registration or a
   liveness revert the edge sends a link-local "assign me a net_id"
   request (well-known endpoint next to the seed-router family, or via
   the `direct_edge.rs` TODO "accept any packet if we don't have a
   net_id yet"), retries until first answer, then goes quiet. Pairs with
   the `AwaitingDiscovery` state (entry 4): a quiet discovered link stays
   silent and stable; "undiscovered" is visible in state, not logs.
   Deliberately NOT a periodic keepalive: liveness stays a passive RX
   watchdog, which is fine — links with natural traffic feed it already,
   and fast dead-link detection on genuinely quiet links is an
   application concern (oxifoc doesn't need it: safety rides the ISR
   deadman, and the host's slow-poll feeds liveness on its own).

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

12. **[discussion only] Reserved "tree root" destination.** The strict-tree
    invariant guarantees exactly one root; a reserved sentinel dst (net
    65535 is already reserved) that routers forward strictly upstream and
    the root delivers locally would make "address the arbiter" generic and
    cheap (TTL already bounds it). Deliberately NOT part of oxifoc's plan:
    our remote targets a configured *drive-master role* via named
    SocketQuery (see notes/remote-design.md §10), because role ≠ topology
    on the bench. Filed here because the mechanism is elegant and other
    ergot users may want it.

## Design fit vs ergot's stated goals (assessed 2026-08-12)

Checked every entry against ergot's own written intent (book chapters
`_01.._04`, `notes/`, conformance spec, code TODOs). Summary:

- **Item 1 (message-sized grants) — aligned, precedent in-repo.**
  `send_raw` is already size-aware; `socket/borrow.rs:257` does a measured
  grant; `borrow.rs:194` carries the exact TODO ("we could probably use a
  smaller grant here than the MTU"); ergot #211 fixed the same defect in
  the defmt sink with exact-size grants. postcard's `ser_flavors::Size`
  is available ungated for a measuring pass. Also fixes the
  `grant_exact` early-wrap failure mode (MTU-sized request fails with
  plenty of aggregate free space). Straight PR, no design discussion.
- **Item 2 (request timeout) — explicitly invited.** Book `_02:40`:
  socket kinds with timeouts/retries "in the future… please start a
  discussion". The runtime-agnostic shape already exists in-repo: the
  sleeper-closure pattern (`futures_io.rs:170`) and
  `delegated_seed_rpc_timeout` (caller supplies the timeout future). So:
  `request_with_deadline(dst, req, name, timeout_future)` — no Timer
  trait, no runtime coupling.
- **Item 3 (edge hello/keepalive) — on the roadmap already** ("Keepalives,
  notice when device drops" under Sessionful Sockets, `_03:281`). Services
  are user-spawned plain futures (no_std-clean, no executor assumption) —
  a `Services::edge_hello(interval_sleeper)` fits the existing model
  exactly; use the sleeper closure to avoid embassy-time version coupling.
- **Item 4 (dead vs undiscovered) — fixing a self-documented deficiency.**
  The trade-off comment ("state alone no longer distinguishes…
  diagnostics move to logs or counters") is duplicated verbatim in
  futures_io.rs:105 and eio.rs:150. Cost measured: 117 references,
  26 match sites, only 3 exhaustive matches (all in edge_port.rs — incl.
  the transmit gate, which a new still-transmitting variant must handle);
  the review surface is the `matches!(.., Active{..})` fallthroughs.
- **Item 5 (resolve-by-key) — MOSTLY ALREADY WORKS.** Port-255 broadcasts
  flood across bridges (router re-floods incl. upstream, split-horizon by
  source ident, TTL 16); responses unicast back; ergot's own
  `e2e_bridge_discovery` test proves host→bridge→root discovery end to
  end. `discover_sockets` returns full Addresses. Remaining upstream
  gaps: (a) tokio-std only — fine for the oxifoc host, blocks the no_std
  remote; (b) one socket per device per query (query_searcher returns
  first match); (c) always burns the full timeout (no early-exit for
  resolve-one); (d) the typed-helper TODO (`discovery.rs:70`:
  `discover_endpoint_socket`). **oxifoc can adopt this host-side TODAY
  with zero ergot changes** — see the plan below.
- **Item 6 (seed-lease worker) — aligned** with the services model
  (user-spawned `-> !` futures); pure upstreaming of the bridge's
  hand-rolled lifecycle.
- **Item 7 (QoS classes) — AGAINST the written stance, reframed.**
  `_04:136`: "the answer … is not a QoS policy — shed the high-volume,
  low-value traffic"; the QoS header field was considered in
  notes/interfaces.md and not adopted; "higher QoS" is parked under
  future Sessionful Sockets. But the book also rules MTU/fragmentation/
  integrity "the concern of a given interface" — priority can live
  there too. Reframed: if oxifoc ever needs it, implement a two-queue
  sink in oxifoc's own interface impl; do not propose core changes.
- **Item 8 (broadcast NoSpace observability) — semantics are normative,
  counters are sanctioned.** Full-subscriber-counts-as-delivered is
  codified in the conformance spec (do not touch); the transport docs
  themselves say "diagnostics move to logs or counters" while zero
  counters exist. Shape: feature-gated per-stack drop/delivery counters.
- **Item 9 (uniform MTU error) — solves together with item 1**: measure
  first → size > mtu ⇒ `PacketTooBig { mtu }` (matches the recent typed
  ProtocolError/MTU-discovery direction, #200); else grant(size) failure
  ⇒ genuine `InterfaceFull`.
- **Item 10 (port exhaustion) — trivial**: non-panicking
  `try_attach_socket` already exists pub(crate); expose a fallible
  attach. The allocator itself already returns Option.
- **Item 11 (&&NetStack) — cosmetic, neutral.**

Non-goals to respect in any proposal: no retransmission/ack/priority in
the netstack core, no background tasks inside the stack (passive,
mutex-serialized, everything immediate), drop-don't-block, strict tree /
one arbiter per segment, no fragmentation or integrity in core (interface
concern), no auth.

### oxifoc adoption plan for item 5 (no ergot changes needed)

Host-lib connect flow: after the interface is Active, run
`discover_sockets(SocketQuery { key: MotorEndpoint::REQ_KEY, frame_kind:
ENDPOINT_REQ, broadcast: false, .. })` (and the device-info
interrogation for UUID pinning), refuse ambiguity, and replace the
`DEVICE_ADDR = {0,1,0}` constant with the resolved Address per
connection generation. Caveat: responders behind a bridge are reachable
only once their segment has a routable net_id (seed lease) — gate the
resolution on that, with the existing RECOVERY_TIMEOUT. The no_std
remote needs the same resolve later → that is the real driver for a
no_std `discover_endpoint_socket` upstream (item 5a).

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
