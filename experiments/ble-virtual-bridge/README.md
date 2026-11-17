# BLE / virtual-controller bridge

Linux test fixture. Advertises **OxiFOC Virtual**, serves the OxiFOC GATT service
(NUS-shaped, UUIDs from `oxifoc_core::ble`),
and translates each BLE packet to/from one COBS TCP frame. The target is
restricted to loopback; the default port is 25267. Start `oxifoc-virtual` on
that port first. Bluetooth must already be powered on.

```sh
# From the repository root, in separate terminals:
nix develop -c cargo run -p oxifoc-virtual -- --port 25267
nix develop -c cargo run --manifest-path experiments/ble-virtual-bridge/Cargo.toml
```

Connect from the Android BLE Lab. Only one central is supported. Ctrl+C removes
the GATT application and advertisement. Stop the virtual controller too.
The TCP socket is recreated on each notification subscription, allowing real
BLE reconnect tests against the normal virtual controller.
