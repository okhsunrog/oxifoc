# Motor commissioning

Use **Detect** in the GUI or `detect all` in the CLI. Start with the
controller stopped, a secured motor that can rotate, and a configured
board/current-sense path. Select the closest motor class, confirm pole
pairs, and choose Hall or sensorless operation.

## Set the power-source limits

Enter **Input current limit** and **Regenerative current limit** in amps.
Both describe the connected DC source. Set regeneration to zero if the
supply cannot sink current. Confirm these limits again when changing from a
bench supply to a battery. The motor current rating is estimated separately
from the class's allowed winding loss and the measured resistance.

The interactive CLI asks for any missing source values. In scripts,
pass both `--input-current-a` and `--regen-current-a` explicitly. Use
`just cli-bridge detect --help` for the available class and diagnostic
options.

## Measure and review

The full sequence measures resistance, inductance and flux linkage, then
calibrates Hall if selected. The resistance step uses a low-current probe
and then a differential measurement at higher current. Inductance uses
separate alignment and AC excitation. The configured source limits,
inverter protection and the temporary test policy apply throughout. Advanced
options can lower the test-current ceiling for diagnostics.

On the MK5, this recipe records the full sequence into a new output directory.

```sh
just detect-bridge CAPTURE_DIR medium-outrunner \
    --input-current-a INPUT_AMPS --regen-current-a REGEN_AMPS --pole-pairs POLE_PAIRS
```

Replace the uppercase arguments with the bench values. The JSON capture
recipe reports the measured values and proposed limits without saving.
The GUI and interactive CLI offer a review before saving. A scripted caller
can explicitly save using `--apply`.

Saving installs one drive record containing the motor model, feedback,
calibration and limits. For Hall motors, the standard feedback uses Hall at
low speed and the back-EMF observer at higher speed. Sensorless selection
uses the observer. An explicit drive current override is available at
review and remains subject to the inverter limits.

## Adjust settings after commissioning

| Group | Behavior |
|---|---|
| `drive` | `invert_direction` defines forward in rider commands. |
| `derating` | Independent forward/reverse speed ceilings and taper start. A zero ceiling disables that direction's speed limit. The E-Skate profile sets duty taper start to 85%. |
| `voltage-feedforward` | Select `Disabled`, `CrossCoupling`, `BackEmf` or `Both`. |
| `motor-feedback` | Set the commissioned source and crossover band in electrical rad/s while stopped. The temporary `source` command is useful for bench A/B. |
| `pwm-config` | Choose the supported PWM frequency. The active rate changes after reboot and is reported alongside the staged value. |

The CLI's `--volatile` applies an edit in RAM. Omitting it requests
persistence, which requires a stopped controller. For example, while stopped:

```sh
just cli-bridge config get motor-feedback
just cli-bridge config set motor-feedback mode=hall blend_low_erad_s=150 blend_high_erad_s=300 --volatile
just cli-bridge config get pwm-config
```

See the [firmware session design](notes/motor-setup-wizard.md) for the
operation contract and the [observer tuning note](notes/hall-observer-tuning.md)
for the MK5 crossover baseline.
