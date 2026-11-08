"""Arm or export the finite trace in a bench-executor-trace firmware.

Use an exact `rust-nm -n -S -C ELF` symbol listing. Arm during the workload,
then export after the trace has frozen. SWD attach/read can perturb the device;
the RAM trace is finite, not a continuous production recording.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('action', choices=['arm','read'])
p.add_argument('--symbols', required=True)
p.add_argument('--out', help='Output binary path for read')
p.add_argument('--on-active', action='store_true', help='Arm for the next active FOC transition')
p.add_argument('--probe', default='0483:3754:002300203234511537333934')
a = p.parse_args()
symbols = {}
for line in Path(a.symbols).read_text().splitlines():
    match = re.match(r'([0-9a-f]+) ([0-9a-f]+) [Bb] (BENCH_TRACE_\w+)$',line)
    if match:
        addr, size, name = match.groups()
        symbols[name] = (int(addr,16),int(size,16))
control = symbols['BENCH_TRACE_CONTROL'][0]
base = ['probe-rs','--chip','STM32F405RGTx','--probe',a.probe]
def invoke(command, *args):
    return subprocess.check_output([base[0],command,*base[1:],*map(str,args)],text=True)
if a.action == 'arm':
    invoke('write','b32',hex(control),5 if a.on_active else 1)
    if not a.on_active:
        time.sleep(.3)  # 100 ms quiet delay + a finite ADC-driven trace window
else:
    if not a.out:
        p.error('--out is required for read')
    # At least one ADC period has passed after the final reservation. Reading
    # while writers are active would not be a coherent snapshot.
    state = invoke('read','b32',hex(control),1).split()[-1]
    if int(state,16) != 0:
        raise RuntimeError('trace is still recording; wait before exporting')
    count = int(invoke('read','b32',hex(symbols['BENCH_TRACE_COUNT'][0]),1).split()[-1],16)
    if count < symbols['BENCH_TRACE_EVENTS'][1]//12:
        raise RuntimeError('trace has not filled; arm it before exporting')
    time.sleep(.01)
    addr, size = symbols['BENCH_TRACE_EVENTS']
    invoke('read','b32',hex(addr),size//4,'--output',a.out,'--format','binary')
    data=Path(a.out).read_bytes()
    Path(a.out+'.json').write_text(json.dumps({
        'symbols':str(Path(a.symbols).resolve()),'bytes':len(data),
        'sha256':hashlib.sha256(data).hexdigest(),'nominal_timer_hz':84_000_000,
        'events':len(data)//12,'time_source':'TIM5, PSC=0, APB1 timer clock',
        'reservations':count,
        'state':0,'scope':'finite instrumented window; poll time includes IRQ preemption',
    },indent=2)+'\n')
