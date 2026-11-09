"""Export the first bench-hall edges over SWD after the bridge has stopped.

Uses the exact firmware ELF and USB status; never halts, resets or writes RAM.
The published prefix is immutable even if residual coast appends more edges.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import struct
import subprocess

CAPACITY = 128
SIZES = {"BENCH_HALL_EDGES": CAPACITY * 16,
         "BENCH_HALL_EDGE_COUNT": 4, "BENCH_HALL_EDGE_DROPPED": 4}


def decode(data, count):
    """Decode only records published before the read began.

    >>> edge = decode(struct.pack('<4I', 5, 2, 0xffffffff, 0x306), 1)[0]
    >>> (edge['ticks'], edge['adc_sequence'], edge['raw_state'])
    (8589934597, 4294967295, 6)
    >>> (edge['overcapture'], edge['boot_seed'])
    (True, True)
    >>> decode(b'', 0)
    []
    >>> decode(b'', 1)
    Traceback (most recent call last):
        ...
    ValueError: invalid Hall buffer prefix
    """
    if not 0 <= count <= CAPACITY or len(data) < count * 16:
        raise ValueError("invalid Hall buffer prefix")
    return [dict(ticks=lo | (hi << 32), adc_sequence=seq, raw_state=flags & 7,
                 overcapture=bool(flags & 0x100), boot_seed=bool(flags & 0x200),
                 flags_raw=flags)
            for lo, hi, seq, flags in struct.iter_unpack('<4I', data[:count * 16])]


def symbols_from(text):
    symbols = {}
    for line in text.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[3] in SIZES and parts[2] in 'BbDd':
            address, size = int(parts[0], 16), int(parts[1], 16)
            if size != SIZES[parts[3]] or address % 4:
                raise ValueError(f"incompatible symbol layout: {line}")
            symbols[parts[3]] = (address, size)
    if symbols.keys() != SIZES.keys():
        raise ValueError("ELF is missing the bench-hall edge buffer symbols")
    return symbols


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='JSON output; binary sidecars use this prefix')
    parser.add_argument('--elf', type=Path, required=True, help='Exact running firmware ELF')
    parser.add_argument('--probe', default='0483:3754:002300203234511537333934')
    args = parser.parse_args()
    out, elf = args.output.resolve(), args.elf.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[1]
    elf_sha256 = hashlib.sha256(elf.read_bytes()).hexdigest()
    listing = subprocess.check_output(['rust-nm', '-n', '-S', '-C', str(elf)], text=True)
    symbols = symbols_from(listing)
    Path(str(out) + '.symbols').write_text(listing)
    status = json.loads(subprocess.check_output(
        ['just', 'cli-usb', '--elf', str(elf), 'status'], cwd=repo, text=True))
    Path(str(out) + '.status.json').write_text(json.dumps(status, indent=2) + '\n')
    if status['control_mode'] != 'Stopped' or status['motor_state'] not in ('Stopped', 'Error'):
        raise RuntimeError('Hall export requires Stopped control mode and Stopped/Error state')

    # Prevent inherited probe presets or reset/power-cycle environment flags
    # from changing these read-only commands into an attach under reset.
    env = {key: value for key, value in os.environ.items() if key not in (
        'PROBE_RS_CONFIG_PRESET', 'PROBE_RS_CONNECT_UNDER_RESET', 'PROBE_RS_CYCLE_POWER')}

    def read(name, suffix):
        address, size = symbols[name]
        path = Path(str(out) + suffix)
        subprocess.check_output([
            'probe-rs', 'read', '--chip', 'STM32F405RGTx', '--probe', args.probe,
            '--non-interactive', 'b32', hex(address), str(size // 4),
            '--output', str(path), '--format', 'binary'], env=env)
        data = path.read_bytes()
        if len(data) != size:
            raise RuntimeError(f'incomplete memory read: {path}')
        return data

    def scalar(name, suffix):
        return struct.unpack('<I', read(name, suffix))[0]

    before = scalar('BENCH_HALL_EDGE_COUNT', '.count-before.bin')
    dropped_before = scalar('BENCH_HALL_EDGE_DROPPED', '.dropped-before.bin')
    data = read('BENCH_HALL_EDGES', '.edges.bin')
    after = scalar('BENCH_HALL_EDGE_COUNT', '.count-after.bin')
    dropped_after = scalar('BENCH_HALL_EDGE_DROPPED', '.dropped-after.bin')
    if not 0 <= before <= after <= CAPACITY or dropped_after < dropped_before:
        raise RuntimeError('Hall counters reset or wrapped during export')
    if hashlib.sha256(elf.read_bytes()).hexdigest() != elf_sha256:
        raise RuntimeError('ELF changed during export')
    edges = decode(data, before)
    result = dict(
        schema='bench-hall-edges', version=1, elf=str(elf),
        elf_sha256=elf_sha256,
        binary_sha256=hashlib.sha256(data).hexdigest(), bridge_stopped=True,
        status=status, tick_hz=1_000_000, tick_width_bits=64,
        time_domain='logical, delay-corrected Hall capture ticks',
        adc_sequence_semantics='latest observed ADC sequence, raw u32; not exact edge cycle',
        capacity=CAPACITY, count_before=before, count_after=after,
        dropped_before=dropped_before, dropped_after=dropped_after,
        overcapture_records=sum(edge['overcapture'] for edge in edges),
        limits='First published prefix only. Overcapture means unknown hardware edge loss; '
               'dropped counts buffer overflow. Residual coast can append during read.',
        edges=edges)
    out.write_text(json.dumps(result, indent=2) + '\n')
    print(f'{out}: {len(edges)} edges, {dropped_after} buffer drops, '
          f'{result["overcapture_records"]} overcapture flags')


if __name__ == '__main__':
    main()
