"""Measure the entire application process tree, including WebKit children."""
import argparse, json, os, statistics, subprocess, time
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('label');p.add_argument('pid',type=int);p.add_argument('log',type=Path);p.add_argument('--seconds',type=float,default=12);p.add_argument('--warmup',type=float,default=6);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
ticks=os.sysconf('SC_CLK_TCK')
def usage():
    records={}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            f=path.read_text().rsplit(') ',1)[1].split();records[int(path.parent.name)]=(int(f[1]),(int(f[11])+int(f[12]))/ticks)
        except (FileNotFoundError,ProcessLookupError):pass
    ids={a.pid}
    while True:
        more={pid for pid,(ppid,_) in records.items() if ppid in ids}
        if more<=ids:break
        ids|=more
    cpu={pid:records[pid][1] for pid in ids if pid in records};mem={'rss_mib':0.,'pss_mib':0.}
    for pid in ids:
        try:
            for line in Path(f'/proc/{pid}/smaps_rollup').read_text().splitlines():
                if line.startswith(('Rss:','Pss:')):
                    k,n,_=line.split();mem[k[:-1].lower()+'_mib']+=int(n)/1024
        except (FileNotFoundError,ProcessLookupError,PermissionError):pass
    return cpu,mem

time.sleep(a.warmup);offset=a.log.stat().st_size;before,_=usage();start=time.monotonic();memory=[]
while time.monotonic()-start<a.seconds:
    _,mem=usage();memory.append(mem);time.sleep(.5)
after,_=usage();elapsed=time.monotonic()-start
with a.log.open() as f:f.seek(offset);lines=f.readlines()
frames=[]
for line in lines:
    for marker in ['[desktop-probe] ','[plot-stats] ']:
        if marker in line:
            try:
                v=json.loads(line.split(marker,1)[1]);
                if 'fresh_hz' in v:frames.append(v)
            except json.JSONDecodeError:pass
result={'label':a.label,'seconds':elapsed,'pids':list(after),'cpu_percent_one_core':sum(after[k]-before.get(k,0) for k in after)/elapsed*100,**{k:statistics.mean(m[k] for m in memory) for k in memory[0]},'windows':frames}
a.output.parent.mkdir(parents=True,exist_ok=True)
with a.output.open('a') as f:f.write(json.dumps(result)+'\n')
print(json.dumps({k:round(v,2) if isinstance(v,float) else v for k,v in result.items() if k!='windows'}),flush=True)
if frames: print(json.dumps({'fresh_hz':statistics.mean(f['fresh_hz'] for f in frames),'p95_range':[min(f['interval_p95_ms'] for f in frames),max(f['interval_p95_ms'] for f in frames)]}),flush=True)
