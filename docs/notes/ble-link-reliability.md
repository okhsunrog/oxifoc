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

### After the P0 changes

Remote on the 32.768 kHz crystal, 1M PHY only, +8 dBm; bridge advertising
every 20–30 ms at +20 dBm; immediate reconnect after a lost session.

| Scenario | Result |
|---|---|
| Remote power-on (8 runs) | session active 0.74–1.03 s after boot, median 0.77 s |
| Bridge reset (6 runs) | loss detected ≈1 s after reset; loss → session active 287–673 ms, median ≈320 ms |

Remaining budget after a loss: 1 s supervision timeout, ≈0.7 s C5 boot when
the bridge itself restarted, ≈0.28 s GATT discovery + MTU exchange, and
≈0.1 s per failed establishment.

### 0x3E investigation — cause found

Failed establishment ran at roughly 30–50 % per attempt. The bridge logged
nothing for failed attempts, even with `trouble_host=debug`, and kept
advertising, so the C5 never accepted the CONNECT_IND.

**Cause: a display next to the bridge's antenna.** After moving it, the same
firmware gave 0 failures in 17 attempts (10 remote power-ons, 7 bridge
resets), and loss → session active settled at 287–312 ms. Keep displays and
their flex cables away from both antennas on the custom boards.

Checked on the way, not the cause:

- **C5 BLE low-power timer clock.** On the running bridge
  `MODEM_LPCON.LP_TIMER_CONF = 0` and `CLK_CONF.clk_lp_timer_en = 0`:
  esp-radio 1.0.0-beta.1 selects no source and leaves the clock off, while
  telling the controller `rtc_freq = 32768`. ESP-IDF uses the 48 MHz XTAL
  divided by 480 (100 kHz) with `rtc_freq = 100000`. A local esp-radio patch
  doing the same (`LP_TIMER_CONF = 0x1df4`) did not change the failure rate.
  Still worth reporting upstream.
- **Advertising channel.** Single-channel runs gave no consistent pattern.
- **`ble_npl_callout_is_active`.** The "missing real implementation" debug
  line is stale; the function is implemented.
- **nrf-sdc `NordicCompatModeWindowOffsetSet`** asserts in the SoftDevice
  Controller (`57:2031`), sent before or after host init. Untested.

### ESP32 comparison

`oxifoc-remote-stick` (M5StickC Plus, ESP32 v1.1, single core) runs a port of
the same link code against the same bridge, after the display was moved:

| | nRF52840 remote | ESP32 stick |
|---|---|---|
| Bridge reset: loss → session active | 287–312 ms, 0 failures in 7 | 280–419 ms, median ≈319 ms |
| Power-on: connect start → session active | ≈490 ms, 0 failures in 10 | 300–380 ms, 0 failures in 8 |
| Failed establishments, all runs | 0 of 17 | 2 of 17 |

Reconnect behaviour is equivalent. The nRF's extra ≈200 ms at power-on sits
between `connect` and the connection; the 32.768 kHz crystal also adds
≈250 ms before the stack starts. Range, hand blocking and long runs were not
compared.

## Plan

### P0 — cheap, large effect

1. **[done] Connection parameters** in `remote.rs`: 15–20 ms interval,
   latency 0, 1 s supervision timeout for every image. The 150 ms ISR
   deadman handles freshness separately. Tune the timeout under interference:
   shorter frees the slot and starts reconnecting sooner, longer rides out
   short fades without a reconnect.
2. **[done] Reconnect without the 2 s sleep.** A lost session and the first
   three failed attempts retry immediately; longer runs of failures back off
   50 ms → 1 s. `central.connect` has no timeout
   of its own; it waits until the bridge advertises.
3. **[done] Two connections on the bridge.** Two link slots share the
   trouble `Peripheral` through a mutex: a free slot advertises, a connected
   slot releases the mutex, so the bridge advertises while any slot is free.
   Each slot is its own ergot router interface with its own outbound queue,
   flushed on disconnect. With a laptop holding a `monitor --fast-hz 0`
   session, 6 remote power-ons connected without failures in 479–491 ms
   (connect start → session active), and the laptop stream (270 samples in
   135 s) was not interrupted.
4. **[done] Fast advertising on the bridge.** The default interval is 160 ms. The
   bridge is board-powered, so use 20–30 ms whenever a remote slot is free.
5. **[done] 1M PHY for the remote.** The remote's controller is built without
   2M support, and the bridge no longer requests a PHY (host tools may still
   ask for 2M). The remote logs connection events and rejects parameter
   requests from the peer.
6. **[done] TX power.** Remote: `nrf_sdc::Builder::default_tx_power` (+8 dBm max on
   nRF52840). Bridge: esp-radio `Config::with_default_tx_power` (default
   +9 dBm). Legacy `advertise` ignores trouble's `tx_power`.
7. **[done] LF clock on the remote:** the 32.768 kHz crystal, declared as
   50 ppm.

### P1 — reconnect in tens of milliseconds

8. **[done] Cache GATT handles, persisted in flash.** After a discovery the
   remote stores `{service range, RX, TX, TX CCCD}` in a `sequential-storage`
   map in the two pages below the bootloader (`0xF2000..0xF4000`, memory.x
   ends the firmware there; `build.rs` checks the overlap). Writes go through
   the MPSL flash driver (one timeslot session), so the radio keeps its
   timing. A cached connect re-checks the service range and enables
   notifications with an acknowledged CCCD write; any mismatch drops the cache
   and rediscovers. The fallback path is not yet exercised on hardware.
   Measured: connection → session active 282 ms with discovery, 92–122 ms
   cached; link loss → session active ≈140 ms; power-on → session active
   ≈0.54 s.

   **Fast remote restarts.** If the remote restarts within the bridge's 1 s
   supervision timeout, the bridge still holds the dead link to the same
   address, and the C5 ignores CONNECT_IND from that address: every attempt
   fails with 0x3E until the old link times out (seen as two failed attempts,
   ≈0.2 s). A random connection interval per attempt did not help; a fresh
   static random address per boot did (5 of 5 quick restarts without
   failures, power-on → session active 0.39–0.42 s), so the remote now picks
   one at boot. Bonding (item 9) replaces it with a resolvable private
   address; verify then that the C5 does not resolve it to the old identity
   and reject it the same way.
9. **Bonding, privacy and enrollment** — designed in
   [ble-pairing.md](ble-pairing.md): Ergot carries the keys, only bonded and
   encrypted BLE links reach Ergot, pairing windows opened by a trusted
   channel, MITM-protected methods only, at most 8 bonds. Replaces the
   hard-coded bridge address in `remote.rs` with bond-based discovery.

### P2 — monitoring and environment

- **USB flashing and logging for the remote,** as in the Voyager firmware. The
  final remote exposes only USB, not SWD.
- **Fast telemetry over BLE.** `monitor --fast-hz 100` breaks the BLE session
  ("Telemetry acknowledgement does not match the stream state"); `--fast-hz 0`
  works. Native-rate data stays on USB. Check whether 10 Hz works and whether a
  frame exceeds the 505-byte BLE frame limit.
- **Wi-Fi for closed-board telemetry** (future). Needs commands to switch Wi-Fi
  on and off and an RF coexistence test against the throttle link. With two
  motors there are two C5s: one can serve BLE and the other Wi-Fi, which
  avoids coexistence on a single radio.

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
