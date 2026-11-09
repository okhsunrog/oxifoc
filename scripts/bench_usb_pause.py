"""Pause the idle USB recorder to measure finite buffering headroom.

Run through `just test-usb-pause`; uses the existing CLI and capture recipes.
The recorder is always resumed, including when the experiment is interrupted.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('out')
p.add_argument('--rate', default='20000')
p.add_argument('--pauses-ms', default='5,15,30')
a = p.parse_args()
pauses = [float(value) for value in a.pauses_ms.split(',')]
if not pauses or any(not 0 < value <= 1000 for value in pauses):
    p.error('pause durations must be in (0, 1000] ms')
status = json.loads(subprocess.check_output(['just', 'cli-usb', 'status'], text=True))
if status['control_mode'] != 'Stopped' or status['motor_state'] != 'Stopped':
    raise RuntimeError('host-pause qualification requires a stopped controller')
seconds = str(6 + 3 * len(pauses))
proc = subprocess.Popen(['just', 'record-usb-trace', a.out, seconds, a.rate])
events = []
pid = None
try:
    time.sleep(4)  # CLI is already built; leave handshake and warmup outside pauses.
    matches = []
    for path in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = path.read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if args and args[0].endswith(b'/oxifoc-host-cli') and a.out.encode() in args:
            matches.append(int(path.parent.name))
    if len(matches) != 1:
        raise RuntimeError(f'expected one recorder, found {matches}')
    pid = matches[0]
    for milliseconds in pauses:
        start = time.monotonic()
        os.kill(pid, signal.SIGSTOP)
        try:
            time.sleep(milliseconds / 1000)
        finally:
            os.kill(pid, signal.SIGCONT)
        events.append(dict(host_utc=time.time(), requested_ms=milliseconds,
                           actual_ms=(time.monotonic() - start) * 1000))
        time.sleep(3)
finally:
    if pid is not None:
        try:
            os.kill(pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
    Path(a.out + '.pauses.json').write_text(json.dumps(events, indent=2) + '\n')
    code = proc.wait()
print(json.dumps(dict(recorder_exit=code, pauses=events)))
raise SystemExit(code)
