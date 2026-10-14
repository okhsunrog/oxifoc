"""Reproduce UTF-16 host timings and Cortex-M4 code sizes with uv run.

No board is flashed. Host nanoseconds are not Cortex-M4 cycle counts.
Example: uv run scripts/benchmark-utf16.py --out /tmp/utf16-results --cpu 0
"""
import argparse
import json
from pathlib import Path
import platform
import re
import statistics
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--cpu', type=int, default=0)
parser.add_argument('--sizes-only', action='store_true')
args = parser.parse_args()
args.out.mkdir(parents=True, exist_ok=True)
repo = Path(__file__).resolve().parents[1]
probes = repo / 'docs/experiments/core-float-math'
profiles = [(tc, opt) for tc in ['1.97', 'nightly-2026-10-14'] for opt in ['3', 's', 'z']]
result = {'metadata': {'machine': platform.machine(), 'cpu_affinity': args.cpu,
                      'host_target_cpu': 'default x86-64, no target-cpu=native',
                      'arm_target': 'thumbv7em-none-eabihf', 'host_samples': 15,
                      'sample_target_ms': 20, 'governor': None,
                      'timing_unit': 'host ns per call, includes call and black_box'},
          'arm_sizes': [], 'arm_isolated': [], 'host_timings': []}
info = Path('/proc/cpuinfo').read_text()
result['metadata']['cpu_model'] = re.search(r'model name\s*:\s*(.+)', info)[1]
gov = Path(f'/sys/devices/system/cpu/cpu{args.cpu}/cpufreq/scaling_governor')
if gov.exists():
    result['metadata']['governor'] = gov.read_text().strip()
result['metadata']['compilers'] = {
    tc: subprocess.check_output(['rustc', f'+{tc}', '-vV'], text=True)
    for tc in ['1.97', 'nightly-2026-10-14']}
for tc, opt in profiles:
    obj = args.out / f'{tc}-{opt}.o'
    subprocess.run(['rustc', f'+{tc}', '--edition', '2024', '--crate-type', 'lib',
                    '--target', 'thumbv7em-none-eabihf', '-C', f'opt-level={opt}',
                    '--emit=obj', str(probes / 'utf16-probe.rs'), '-o', str(obj)], check=True)
    nm = subprocess.check_output(['arm-none-eabi-nm', '-S', str(obj)], text=True)
    sizes = {m[2]: int(m[1], 16) for m in re.finditer(
        r'^[0-9a-f]+\s+([0-9a-f]+)\s+T\s+(\w+)', nm, re.M)}
    result['arm_sizes'].append({'toolchain': tc, 'opt_level': opt, 'bytes': sizes})
    # One implementation per object: include out-of-line next()/outlining
    # helpers, which would be missed by measuring the exported symbol alone.
    for mode, expression in [('count', 's.encode_utf16().count()'),
                             ('fold', 's.encode_utf16().fold(0usize, |n, _| n + 1)')]:
        source = args.out / f'isolated-{tc}-{opt}-{mode}.rs'
        source.write_text('#![no_std]\n#[unsafe(no_mangle)]\n'
                          f'pub fn utf16_len(s: &str) -> usize {{ {expression} }}\n')
        isolated_obj = source.with_suffix('.o')
        subprocess.run(['rustc', f'+{tc}', '--edition', '2024', '--crate-type', 'lib',
                        '--crate-name', 'utf16_isolated',
                        '--target', 'thumbv7em-none-eabihf', '-C', f'opt-level={opt}',
                        '--emit=obj', str(source), '-o', str(isolated_obj)], check=True)
        section_output = subprocess.check_output(
            ['arm-none-eabi-size', '-A', str(isolated_obj)], text=True)
        sections = {m[1]: int(m[2]) for m in re.finditer(
            r'^(\.\S+)\s+(\d+)\s+\d+', section_output, re.M)}
        result['arm_isolated'].append({'toolchain': tc, 'opt_level': opt, 'mode': mode,
            'text_bytes': sum(n for name, n in sections.items() if name.startswith('.text')),
            'sections': sections})
    if not args.sizes_only:
        binary = args.out / f'host-{tc}-{opt}'
        subprocess.run(['rustc', f'+{tc}', '--edition', '2024', '-C', f'opt-level={opt}',
                        '-C', 'codegen-units=1', '-C', 'lto=fat',
                        str(probes / 'utf16-bench.rs'), '-o', str(binary)], check=True)
print('Compilation completed', flush=True)
if not args.sizes_only:
    for tc, opt in profiles:
        binary = args.out / f'host-{tc}-{opt}'
        raw = subprocess.check_output(['taskset', '-c', str(args.cpu), str(binary)], text=True)
        (args.out / f'host-{tc}-{opt}.jsonl').write_text(raw)
        for line in raw.splitlines():
            row = json.loads(line)
            row.update({'toolchain': tc, 'opt_level': opt,
                        'count_median_ns': statistics.median(row['count_ns']),
                        'fold_median_ns': statistics.median(row['fold_ns']),
                        'paired_ratio_median': statistics.median(
                            f / c for c, f in zip(row['count_ns'], row['fold_ns']))})
            for mode in ['count', 'fold']:
                q = statistics.quantiles(row[f'{mode}_ns'], n=4)
                row[f'{mode}_q1_ns'], row[f'{mode}_q3_ns'] = q[0], q[2]
            result['host_timings'].append(row)
        print(f'Timing completed: {tc} opt-level={opt}', flush=True)
(args.out / 'results.json').write_text(json.dumps(result, indent=2) + '\n')
