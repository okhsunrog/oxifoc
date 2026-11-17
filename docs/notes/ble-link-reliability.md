# BLE link reliability: remote ↔ bridge

Bench measurements from 2026-11-17 and a prioritized plan for a stable link,
fast reconnects and fast connect at power-on. Setup: `oxifoc-remote` on a
nice!nano v2 clone (nRF52840, nrf-sdc + trouble, central) and
`oxifoc-bridge` on ESP32-C5 (esp-radio 1.0.0-beta.1 + trouble, peripheral,
one connection). Both log defmt with uptime timestamps.

## Measurements

| Scenario | Default params (80 ms interval, 8 s supervision) | 15–20 ms interval, 1 s supervision |
|---|---|---|
| Remote power-on, bridge advertising | 12/12 first attempt; connect 0.11 s, session active 1.5 s | connect 0.03 s, session active 0.41 s |
| Bridge reset, remote running | loss detected after 8 s; first attempt failed with 0x3E in 2 of 3 resets (4 in a row once) | 4/4 first attempt; loss detected ≈1 s after reset |
| Remote reset, bridge connected | bridge holds its only slot for 8 s, remote waits | slot freed after ≈1 s (not re-measured) |

Session active = service discovery, MTU exchange and notification subscribe
done. At 80 ms each ATT round trip costs about two connection events, so
discovery took ≈1.3 s; at 15–20 ms it takes ≈0.35 s. After a loss the remote
still sleeps 2 s before reconnecting, which now dominates the reconnect time.

**Connection establishment failures (0x3E).** The remote's controller reports
the connection when it sends CONNECT_IND, then "Connection Failed to be
Established" about 0.4 s later. The bridge never reports these connections,
so the C5 did not sync on the first connection events. The next attempt,
about 0.1 s later, succeeded. Failures appeared only after a link loss and
only with the 80 ms interval; none were seen in 4 resets at 15–20 ms. The
sample is small. Unconfirmed cause: esp-radio's C5 `ble_rtc_clk_init` is an
empty stub while the controller is told it has a 32768 Hz, 60 ppm sleep clock,
so timing over a long first interval may be off. nrf-sdc's
`NordicCompatModeWindowOffsetSet` (avoids WinOffset = 0) asserted inside the
SoftDevice Controller (`57:2031`) when sent, so it was not tested.

## Plan

### P0 — cheap, large effect

1. **Connection parameters** — done in `remote.rs`: 15–20 ms interval,
   latency 0, 1 s supervision timeout for every image. The 150 ms ISR
   deadman handles freshness separately. Tune the timeout under interference:
   shorter frees the slot and starts reconnecting sooner, longer rides out
   short fades without a reconnect.
2. **Reconnect without the 2 s sleep.** Retry immediately after a loss and
   back off only after repeated failures. `central.connect` has no timeout
   of its own; it waits until the bridge advertises.
3. **Two connections on the bridge.** With one slot a stale link, the laptop
   or the phone blocks the remote. esp-radio's C5 config defaults to
   `max_connections = 2`; trouble needs `CONNECTIONS_MAX = 2` and the bridge
   must keep advertising while one link is up. This is also required for
   remote + phone (remote-design §7).
4. **Fast advertising on the bridge.** The default interval is 160 ms. The
   bridge is board-powered, so use 20–30 ms whenever a remote slot is free.
5. **Let only the central request 2M PHY,** and read connection events on the
   remote (`PhyUpdated`, `ConnectionParamsUpdated`, failures). The remote
   never polls `conn.next()`, so these events are dropped today. Consider
   staying on 1M for the remote link: 1M has better receiver sensitivity,
   and throttle traffic does not need 2M throughput.
6. **TX power.** Remote: `nrf_sdc::Builder::default_tx_power` (+8 dBm max on
   nRF52840). Bridge: esp-radio `Config::with_default_tx_power` (default
   +9 dBm). Legacy `advertise` ignores trouble's `tx_power`.
7. **LF clock on the remote.** It runs on the RC oscillator declared as
   250 ppm. If the clone has the nice!nano 32.768 kHz crystal, use XTAL with
   its real accuracy: tighter timing and lower current.

### P1 — reconnect in tens of milliseconds

8. **Cache GATT handles.** The bridge's attribute table is fixed per firmware
   build. After the first discovery, use the cached handles
   (`write_handle_without_response`, CCCD via `write_handle`,
   `listen_all`) and fall back to discovery on error. trouble does not let
   callers construct a `Characteristic`, so this needs the handle-based API.
   The MTU exchange stays, since it is per connection.
9. **Bonding and privacy** (trouble `security` feature: LE Secure
   Connections, `BondInformation`, IRK). Needed anyway: the bridge accepts
   any central today. Once bonded, the bridge can restrict connections to the
   remote with its filter accept list and, after a loss, send high-duty
   directed advertising to it (`ConnectableNonscannableDirectedHighDuty`,
   up to 1.28 s). Bonds must be persisted by the application on both sides.
10. **Pairing instead of the hard-coded bridge address** in `remote.rs`.

### P2 — monitoring and environment

11. **Link quality.** Track RSSI (`LeReadRssi`, Nordic average RSSI) and
    affirm round-trip time; drive the remote-design §10 haptic warning.
12. **Wi-Fi coexistence** once the C5 runs Wi-Fi: esp-radio coex callbacks are
    mostly stubs; measure throttle timing under Wi-Fi load.
13. **Coded PHY** as a range fallback is not available as built: nrf-sdc
    gates extended scanning and coded PHY on both its `central` and
    `peripheral` features, and C5 support is unverified.

## Test procedure

Run each change against the same scenarios, with both RTT logs attached:

- remote resets while the bridge runs (power-on connect);
- bridge resets while the remote runs (loss detection and reconnect);
- remote resets while connected (stale-link slot);
- distance, hand and body blocking, and 2.4 GHz traffic.

Record per attempt: time to detect loss, connect result and disconnect reason,
time to an active session, and the affirm gap distribution against the 150 ms
deadman.
