# Protocol versioning & compatibility — design notes

Status: **Phases 0–3 IMPLEMENTED** (8ee6f84/3ef160a/cde3a80/4f00909, 2026-06;
header was stale until 2026-07-06). Remaining work is the ergot-side L1 items
below. Cross-repo (some of this lands in
[ergot](https://github.com/okhsunrog/ergot), some in oxifoc). Captured so it is
not forgotten; supersedes the old one-line "`protocol_version` in HardwareInfo"
TODO (whose premise was wrong — see §1).

## 1. The pivotal mechanic: ergot addresses ARE schema-versioned

`endpoint!`/`topic!` compute the wire key as
`Key::for_path::<Payload>(PATH)` (ergot `traits.rs`) — an 8-byte hash of the
path **plus the recursive postcard `Schema` of the payload type**. Therefore:

- Change a field in a request/response/topic type → its `Schema` changes → its
  `Key` changes → it is a **different address on the wire**.
- A host targeting the old key against a device serving the new key gets a
  **routing failure (`NoRoute`), not silent garbage.** Type identity is baked
  into the address — ergot is **fail-closed**, not unversioned.
- Granularity is **per-(endpoint, type) automatically.** Changing `HardwareInfo`
  breaks only that endpoint; `config`/`detect` keep working on their own keys.
- 8-byte hash → collision ≈ 2⁻⁶⁴, treat as exact.
- `FastTelemetryBatch<N>`: the heapless `Vec<_, N>` capacity is **not** in the
  schema, so device `Batch<64>` ↔ host `Batch<256>` stay compatible.

**Correction to the old note:** "postcard has no self-description → mismatch =
silent garbage" is false at the routing layer. The risk is *not* corruption; it
is **a clean failure the host cannot attribute** ("is this a version mismatch or
a transient/absent endpoint?") — and for **topics**, the failure is **silent
absence** (the host's subscription on the old key just never fires; no error).
That silent-topic case is the real reason to gate at connect.

## 2. Three version layers (do not conflate)

| Layer | What | Authority? | Owner |
|---|---|---|---|
| **L1 — ergot wire/proto version** | Header/FrameKind/addressing format | must match first | **ergot** |
| **L2 — schema-key set** | which typed endpoints/topics exist & their shape | mechanical, per-endpoint | ergot (introspection) + the keys themselves |
| **L3 — app semver** | human string for logs/UI | **not** a compat gate | oxifoc |

The keys (L2) are the *real* application-protocol version; L1 is the deeper
gate; L3 is display only.

## 3. L1 — ergot wire version + standard handshake (ergot-side)

Today neither `Header` nor well-known `DeviceInfo` carries a version. If ergot's
own frame format ever changes, peers break with no diagnostic. Proposal:

- Add `ergot_proto_version: u16` to well-known `DeviceInfo`.
- Add a **handshake endpoint** `ErgotDeviceInfoEndpoint: () -> DeviceInfo`
  (point-to-point), alongside the existing broadcast `DeviceInfo` topic (bus
  discovery). Host calls it on connect → identity + `ergot_proto_version`
  (+ optional digest, §4) in one round-trip; confirms alive + compatible.

## 4. L2 — socket-table introspection (ergot-side)

ergot already stores per-socket `(key, nash, frame_kind)` in the router table,
so key-level introspection is "serialize what already exists" — **no new
state.** Cost only appears at the *full-schema* level (a schema pointer per
socket). Forms, cheapest → heaviest:

| Form | Client gets | MCU cost |
|---|---|---|
| **A. `served_digest` (8 B in DeviceInfo)** | commutative fold of all served keys | ~free (incremental on register); fast "identical?" check |
| **B. `SocketQuery` by key** (exists) | present/port for one key | ~free/query; N round-trips for N endpoints |
| **C. enumerate-all** | list of `(key:8, nash:4, kind:1)` ≈13 B/socket | cheap CPU, no new state; ~200 B response → needs pagination/stream for small MTU |
| **D. full DataModel report** (postcard-rpc style) | full recursive schema of every type | heavy: schema pointer per socket (+RAM) + KB on wire → feature-gated, big targets only |

- Global digest equality = "strict identical ICD," **not** "compatible" (host &
  device legitimately serve different key sets — client vs server sockets,
  optional/feature-gated endpoints). So **digest = fast-path only**; on mismatch,
  fall back to per-key probing (B) or enumeration (C) to localize.
- Fold the digest over **app sockets only** (exclude the `ergot/.well-known/*`
  namespace) so it reflects the application surface, not ergot internals.

**Recommended MVP:** A (digest) + B (targeted probe). C is the nice one-shot with
precise "endpoint X changed" diagnostics. D is opt-in.

### 4.1 Why the cost cliff (confirmed runtime fact)

`SocketHeader` (ergot `socket/mod.rs`) holds only `key: Key` (8 B) +
`nash: Option<NameHash>` (4 B) + `frame_kind` — **the schema is NOT retained at
runtime** (it is compile-time, already folded into the Key). So A/B/C
"serialize what's already there" (no new state); **only D** needs new per-socket
storage (a `&'static Schema` pointer) + the schema-descriptor statics in flash
(recursive postcard-schema trees, ~KB for all types). That is the whole reason
D is a different cost class, not a bigger version of C.

### 4.2 The three depths (this is what really separates them)

A Key is a **one-way hash** of `(path ⊕ schema)` — you cannot recover the name
or the shape from it. So introspection comes in three fundamentally different
depths of knowledge:

- **Recognition** — "does this match something I already computed?" (digest, and
  membership of *my own* keys). Cannot interpret anything unknown.
- **Inventory** — "which keys / names exist?" (enumeration): set membership and
  diff at key granularity. An *unrecognized* key is still opaque.
- **Understanding** — "what is the shape, and which field changed?" (DataModel):
  the only depth that makes an *unknown* endpoint intelligible, and the only one
  a generic tool (no ICD source) can consume — like gRPC reflection / OpenAPI.

digest+enumeration live in recognition/inventory (work off keys); DataModel is
the only "understanding" tier — which is exactly why it costs the schema storage.

### 4.3 Enumeration's `(key, nash)` diagnostics — its real value

`nash` carries the (hashed) **name**, `key` carries the (hashed) **shape**. The
pair gives name-level diagnostics without shipping any schema:

| host vs device | verdict |
|---|---|
| same nash, same key | endpoint identical |
| same nash, **different key** | endpoint `X` exists but its **type changed** |
| nash on host, absent on device | endpoint `X` **missing** on device |
| nash on device, absent on host | device serves an endpoint the host doesn't know |

So enumeration localizes to the *named endpoint* and detects shape mismatch —
without full schemas. What it still can't do: explain an endpoint whose **name**
the host doesn't recognize (a truly novel one) — that needs D.

### 4.4 Worked example — device changes `SlowTelemetry.vbus_mv: u32 → u16`

| mechanism | what the host learns |
|---|---|
| `SocketQuery(old key)` | `NoRoute` on the old key — "gone" (not what replaced it) |
| digest | "something differs" (doesn't even know it's SlowTelemetry) |
| enumeration | "`slow_telemetry` exists but its **shape changed** (key ≠ mine)" |
| DataModel | "`SlowTelemetry.vbus_mv`: was `u32`, now `u16`" |

### 4.5 They compose, not compete

```
digest (8 B)   → fast gate: equal ⇒ compatible, stop here
   ↓ differs
enumeration    → localize: which named endpoints diverged in shape
   ↓ need "how" / tooling / codegen
DataModel      → field-level diff, understand the unknown (opt-in)
```
`SocketQuery` is orthogonal: address resolution for a specific key (already used
by `net_stack/discovery.rs::discover_sockets`).

## 5. Schema evolve — opt-in append-tolerant keys (ergot-side, ambitious)

Strict schema-keying gives fail-closed but **no backward-evolvability** (adding a
field = new address). Idea: a per-type opt-in

```rust
#[derive(Schema)] #[schema(evolve)]
struct SlowTelemetry { /* stable prefix */, /* append-only optional tail */ }
```

A type marked `evolve` hashes only its **stable prefix** into the Key; appending
trailing optional fields does **not** change the Key → an old client still
routes and decodes the prefix, new fields read as default.

- Gives true append-compatibility for chosen slow-moving descriptors
  (DeviceInfo, SlowTelemetry, AppInfo) while keeping strict fail-closed
  everywhere else.
- **Trade-off:** for `evolve` types you lose automatic fail-closed on tail-shape
  drift (caught only by append-only discipline). So **opt-in, never default**;
  bad for Pod frames, good for descriptors. Separate large RFC against ergot /
  postcard-schema. **Don't forget — architecturally the nicest of the lot.**

## 6. oxifoc-side: split the custom device-info

The current `HardwareInfo` conflates two concerns; split them:

- **Identity** (name / uuid / mcu / description) → **ergot well-known
  `DeviceInfo`** (it duplicates ergot's job). Drop from oxifoc.
- **Motor descriptor** (foc_freq_hz, max_current_a, **`BoardCalib`** — see
  [telemetry-enrichment.md](telemetry-enrichment.md), app semver L3) → keep a
  lean oxifoc `AppInfoEndpoint`. Candidate for `#[schema(evolve)]` once §5 exists
  so the descriptor can grow without a hard break.
- `set_link_active` (today fired by the HardwareInfo request) → move to ergot
  interface-Active (`state_monitor` already watches `STATE_NOTIFY`).

Host at-connect order: L1 check (ergot_proto_version) → L2 (needed keys present
via digest/probe) → read AppInfo → build `EnrichCtx`. L1/L2 mismatch → friendly
error **before** relying on silent topic data.

## 7. Bidirectional compatibility — stance

Asymmetric. oxifoc is a monorepo (device+host built together) so same-release
skew is rare; full N×M compat is overkill. What's worth it, in order:

1. **Clean mismatch detection** (friendly "firmware too old/new") instead of
   silent failure — cheap, high value (L1 + digest).
2. **Graceful degradation** (bind matching endpoints, disable the rest) — medium
   value, moderate cost (L2 enumeration).
3. **Full multi-version coexistence** (serve V1+V2 in parallel, host negotiates)
   — only per-endpoint, temporarily, when a fleet can't update atomically.

Backward compat (new host ↔ old device) matters more than forward (the host app
updates more often than field firmware).

## 8. Work split

**ergot (upstream):** `ergot_proto_version` in `DeviceInfo`; `DeviceInfo`
handshake endpoint; `served_digest` + `SocketQuery` enumerate-all; later
`#[schema(evolve)]` (own RFC). *(Maintained by us — feasible to land.)*

**oxifoc:** migrate identity → ergot `DeviceInfo`; lean `AppInfoEndpoint`
(foc/current/**BoardCalib**/semver); host connect-time L1/L2 checks; drop the
HardwareInfo handshake role.

## 9. Roadmap / order (agreed)

Build enrichment first (the real goal, pure-core, CI-testable, no hardware);
defer the heavy versioning/ergot work until "host updates separately from
firmware" actually bites — but slot the tiny L1 in before any release.

- **Phase 0** — ✅ DONE (`5d9bb17`) — design notes committed.
- **Phase 1** — ✅ DONE (`8ee6f84`) — core `Scale` codec + `enrich`/`RichSample`/
  `EnrichCtx` + `BoardCalib` (bridge, not sub-struct — see 1c) +
  `ShuntCurrentSense::from_calib` + 6 tests. See
  [telemetry-enrichment.md](telemetry-enrichment.md).
- **Phase 2** — ✅ DONE (`3ef160a`) — `BoardCalib` field on `HardwareInfo` (3
  boards); host builds `EnrichCtx` at connect (calib + DcOffsets + MotorParams,
  graceful fallback); `enrich` in `record` (parquet amps/id/iq/… columns) +
  `watch`. GUI enrichment — ✅ DONE (`8436a01`).
- **Phase 3** — ✅ DONE (`cde3a80`, oxifoc side) — `sw` via
  `env!("CARGO_PKG_VERSION")`; `ICD_PROTO_VERSION` + `HardwareInfo.proto_version`
  + host connect-time mismatch warning. **1c** (BoardCalib as a genuine
  sub-struct) — ✅ DONE (`4f00909`). **Remaining:** the ergot-wire
  `ergot_proto_version` in ergot's `DeviceInfo` (ergot repo, L1 §3). Known
  limitation (review 2026-07-06): the mismatch warning only fires for
  semantics-only changes with identical shapes — a shape change to
  `HardwareInfo` itself changes the response schema key, ergot refuses
  delivery, and the user sees a handshake timeout with no version
  diagnostics instead of the friendly warning.
- **Phase 4** — heavy pass, only under real coexistence pressure: ergot
  `served_digest` + `SocketQuery` enumerate-all (L2); oxifoc device-info split
  (§6: identity → ergot `DeviceInfo`, motor descriptor → lean `AppInfoEndpoint`,
  drop the HardwareInfo handshake role); `#[schema(evolve)]` RFC (§5).

**Near-term fork — how `BoardCalib` reaches the host (Phase 2 vs Phase 4):**
§6 describes the clean end-state (`BoardCalib` in a lean `AppInfoEndpoint`, after
the device-info split). But the *agreed near-term* is to add `BoardCalib` as a
**field on the existing `HardwareInfo`** — a breaking wire change, which is free
on this solo experimental branch (everything rebuilds together). "Breaking the
wire twice" (now to add the field, later to restructure into `AppInfo`) costs
nothing here, and it avoids blocking the enrichment feature on the full
device-info refactor. Phase 4 absorbs the field into the clean `AppInfo`.
