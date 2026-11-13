# OxiFOC

Rust firmware for STM32 field-oriented motor control, with a desktop app,
command-line tools, and a virtual motor for testing. The firmware uses
Embassy for hardware and async tasks, and ergot for commands and telemetry.

## Get started

Enter the development environment on NixOS with `nix develop` or
`direnv allow`. The repository pins the Rust toolchains for both host and
firmware builds.

```sh
just cli --help
just gui
just test-core
```

| Target | Build | Connection |
|---|---|---|
| [Flipsky VESC 6 MK5](docs/hw/vesc6-mk5.md) | `just build-mk5-spi` | STM32 SPI → ESP32-C5 bridge → USB Serial/JTAG |
| [Cheap FOCer 2](docs/hw/cheap-focer2-notes.md) | `just build-f405-cf2` | STM32 USB |
| [NUCLEO-G474RE + IHM08M1](docs/hw/nucleo-g474re-ihm08m1.md) | `just build-g474` | USB and UART; see the board notes for motor-stage status |
| Virtual motor | `cargo run -p oxifoc-virtual -- --help` | TCP or UDP |

On a wired MK5/C5 bench, build the host CLI once with
`cargo build --release --locked -p oxifoc-host-cli`. Use
`just cli-bridge status` to check the controller. Board wiring, flashing
and recovery are in the [MK5 guide](docs/hw/vesc6-mk5.md).
The host and motor firmware must agree on the application protocol version.

## Commission and test a motor

Run the GUI's **Detect** wizard or `just cli-bridge detect all`. Choose the
motor class, pole pairs and feedback type, and confirm both DC source limits.
For scripted runs use `--input-current-a` and `--regen-current-a` with the
values for the connected source. Use zero regeneration for a bench supply
that cannot sink current. Review the result before saving. The MK5 CLI
capture recipe `just detect-bridge CAPTURE_DIR MOTOR_CLASS` records each
stage and prints the result in JSON. It writes the setup only with
`--apply`. See the [commissioning guide](docs/commissioning.md).

The CLI exposes current, speed and throttle control. The GUI exposes
current control. The firmware also implements multi-turn position control
through the shared motor API. The [architecture guide](docs/architecture.md)
explains feedback, control and config behavior.

Use [maneuvers](maneuvers/README.md) for repeatable captures. On the MK5 the
SPI image streams Packed28 via ergot and the C5 bridge. Request the native
rate for full-rate data, or a lower rate when the analysis allows it. Check
the capture's loss counters and command acknowledgements before using it
as evidence.

## Develop

```sh
just check                  # Host and device checks
cargo test --workspace      # Host tests
just test-core              # Core tests with virtual motor and HFI enabled
```

Start with the [documentation map](docs/README.md) for architecture, safety,
hardware notes, and bench evidence. Open work is tracked in the
[backlog](docs/TODO.md).

## License

Licensed under either [Apache-2.0](LICENSE-APACHE) or [MIT](LICENSE-MIT),
at your option.
