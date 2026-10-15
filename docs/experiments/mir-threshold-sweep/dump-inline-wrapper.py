#!/usr/bin/env -S uv run --script
import os,sys
from pathlib import Path
args=sys.argv[2:]
try:
 crate=args[args.index('--crate-name')+1]
except ValueError:
 crate=''
selected={'ergot','postcard','bbqueue','sequential_storage','oxifoc_core','oxifoc_f405','maitake_sync','embassy_sync'}
if crate in selected and '--target' in args:
 dump=Path(os.environ['MIR_ANALYSIS_DUMP'])/crate
 dump.mkdir(parents=True,exist_ok=True)
 args += ['-Zdump-mir=Inline','-Zdump-mir-exclude-pass-number=yes','-Zdump-mir-dir='+str(dump)]
os.execv(sys.argv[1],[sys.argv[1],*args])
