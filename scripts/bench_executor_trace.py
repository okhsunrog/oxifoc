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
p.add_argument('--task-progress', action='store_true', help='Count task progress from active FOC until Stop; export to OUT.tasks.bin')
p.add_argument('--active-delay-ms', type=float, default=0, help='Delay from active FOC to trace window')
p.add_argument('--timer-hz', type=int, required=True, help='Actual declared TIM5 input clock, PSC=0')
p.add_argument('--elf', type=Path, help='Exact firmware ELF, hashed into the read sidecar')
p.add_argument('--capture', help='Matching telemetry recording artifact')
p.add_argument('--schema', choices=['finite-trace-v1', 'netstack-v1'], default='finite-trace-v1')
p.add_argument('--probe', default='0483:3754:002300203234511537333934')
a = p.parse_args()
if a.timer_hz <= 0 or a.active_delay_ms < 0:
    p.error('timer clock must be positive and delay nonnegative')
if a.action == 'arm' and a.task_progress and not a.on_active:
    p.error('--task-progress requires --on-active; arm while Stopped')
symbols = {}
for line in Path(a.symbols).read_text().splitlines():
    match = re.match(r'([0-9a-f]+) ([0-9a-f]+) [Bb] (BENCH_\w+)$',line)
    if match:
        addr, size, name = match.groups()
        symbols[name] = (int(addr,16),int(size,16))
control = symbols['BENCH_TRACE_CONTROL'][0]
base = ['probe-rs','--chip','STM32F405RGTx','--probe',a.probe]
def invoke(command, *args):
    return subprocess.check_output([base[0],command,*base[1:],*map(str,args)],text=True)
if a.action == 'arm':
    if a.task_progress:
        invoke('write','b32',hex(symbols['BENCH_TASK_STATS_CONTROL'][0]),5)
    invoke('write','b32',hex(symbols['BENCH_TRACE_ACTIVE_DELAY'][0]),round(a.active_delay_ms*a.timer_hz/1000))
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
    metadata = {
        'symbols':str(Path(a.symbols).resolve()),'bytes':len(data),
        'sha256':hashlib.sha256(data).hexdigest(),'timer_hz':a.timer_hz,
        'events':len(data)//12,'time_source':'TIM5, PSC=0, APB1 timer clock',
        'reservations':count,
        'state':0,'scope':'finite instrumented window; poll time includes IRQ preemption',
    }
    metadata['schema'] = a.schema
    if a.elf:
        metadata['elf'] = str(a.elf.resolve())
        metadata['elf_sha256'] = hashlib.sha256(a.elf.read_bytes()).hexdigest()
        metadata['source_commit'] = subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()
        metadata['source_dirty'] = subprocess.run(['git','diff','--quiet']).returncode != 0
    if a.capture:
        metadata['capture'] = str(Path(a.capture).resolve())
    if a.schema == 'netstack-v1':
        metadata['events_schema'] = {
            '8/9': 'ADC span, append-time TIM5 timestamps',
            '15/16': 'successful NetStack mutex body begin/end, id=mutex address',
            '17': 'ADC entry snapshot: CNT[15:0], DIR[16], DIR transition[17]; append-time TIM5 stamp',
        }
        raw = invoke('read','b32','0x40010000',17)
        words = [int(word,16) for line in raw.splitlines() if ':' in line
                 for word in re.findall(r'\b[0-9a-fA-F]{8}\b',line.split(':',1)[1])]
        if len(words) != 17:
            raise RuntimeError('incomplete TIM1 configuration readback')
        metadata['tim1'] = dict(zip(['cr1','psc','arr','rcr','ccr4'],[words[i] for i in [0,10,11,12,16]]))
        metadata['tim1_hz'] = 2*a.timer_hz
    Path(a.out+'.json').write_text(json.dumps(metadata,indent=2)+'\n')
    if a.task_progress:
        def scalar(name):
            return int(invoke('read','b32',hex(symbols[name][0]),1).split()[-1],16)
        if scalar('BENCH_TASK_STATS_CONTROL') != 3:
            raise RuntimeError('task progress is not frozen; finish the active profile with Stop')
        if scalar('BENCH_TASK_STATS_OVERFLOW'):
            raise RuntimeError('task progress registry overflowed; results incomplete')
        progress_out=a.out+'.tasks.bin'
        addr,size=symbols['BENCH_TASK_STATS']
        invoke('read','b32',hex(addr),size//4,'--output',progress_out,'--format','binary')
        progress_data=Path(progress_out).read_bytes()
        Path(progress_out+'.json').write_text(json.dumps({
            'symbols':str(Path(a.symbols).resolve()),'timer_hz':a.timer_hz,
            'start_tick':scalar('BENCH_TASK_STATS_START'),'end_tick':scalar('BENCH_TASK_STATS_END'),
            'fields':['id','polls','readies','first_poll_tick','last_poll_tick','last_ready_tick','max_poll_gap'],
            'sha256':hashlib.sha256(progress_data).hexdigest(),
            'scope':'Active FOC to first Stop; boundary-spanning hooks may be excluded. Poll gaps include sleep and are not ready-to-poll latency.',
        },indent=2)+'\n')
