# X-NUCLEO-IHM08M1 — official documentation

Low-voltage 3-phase BLDC/PMSM driver expansion board (ST morpho), used here on
top of the NUCLEO-G474RE. Pin mapping and jumper config for our board live one
level up in [`../nucleo-g474re-ihm08m1.md`](../nucleo-g474re-ihm08m1.md); this
folder holds the primary-source PDFs from ST.

## Board at a glance

| Item | Value |
|---|---|
| Nominal supply | 8–48 V DC (data brief states 10–48 V) |
| Output current | 15 A RMS, 30 A peak |
| Gate driver | **L6398** — half-bridge high/low-side driver (×3) |
| Power stage | **STL220N6F7** — 60 V, ~1.2 mΩ, STripFET F7 N-ch MOSFET (×6) |
| Current sense op-amp | **TSV994IPT** — quad rail-to-rail, 20 MHz (gain ≥ 4) |
| Current sensing | 3-shunt or 1-shunt (jumper-selectable); amplified on-board |
| Sensor interface | Hall / encoder connector (J3) with pull-ups (JP3) |
| Overcurrent | comparator-based OCP + protection (30 A peak) |

Note: the shield was designed for NUCLEO-F302R8 / F401RE; ST's silkscreen/pin
labels are for those MCUs. See the parent doc for the re-derived G474RE mapping.

## Documents in this folder

| File | Doc ID / Rev | What it covers |
|---|---|---|
| `UM1996_IHM08M1_getting_started.pdf` | UM1996 | User manual: HW description, connector/jumper tables, morpho pinout |
| `X-NUCLEO-IHM08M1_data_brief.pdf` | DB2778, Rev 3 | 2-page feature/spec overview + schematic thumbnails |
| `X-NUCLEO-IHM08M1_schematic.pdf` | schematic pack | Full schematic (power, sensing, analog conditioning, MCU pinout) |
| `X-NUCLEO-IHM08M1_quick_start_guide.pdf` | QSG v1.2 | Setup/demo walkthrough, jumper settings, key on-board parts |
| `L6398_gate_driver_datasheet.pdf` | DS18199, Rev 4 | Gate driver: ratings, timing, bootstrap, deadtime (320 ns) |
| `STL220N6F7_mosfet_datasheet.pdf` | DS10089, Rev 6 | Power MOSFET: SOA, Rds(on), gate charge, thermal |
| `TSV994_opamp_datasheet.pdf` | DS4975, Rev 16 | Op-amp: offset, GBW, stability (min gain 4 / -3) |

## Fetching the PDFs

All seven PDFs are already present in this folder.

`fetch-docs.sh` holds the canonical `st.com/resource/en/...` URLs for
re-fetching, but note: **st.com sits behind Akamai bot protection that
fingerprints TLS (JA3), so plain `curl`/`wget` stall at 0 bytes** regardless of
headers. These files were pulled through a real browser (Chrome), which passes.
To refresh a file, open its URL from the script in a browser and save it here,
or use a TLS-impersonating client (e.g. `curl-impersonate`).

## Related (not shield-specific)

- `../nucleo-g474re/` — the Nucleo carrier board (UM2505, data brief, MB1367 schematic)
- `../RM0440.pdf` — STM32G4 reference manual
