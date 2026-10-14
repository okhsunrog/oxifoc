"""Compare two ARM firmware ELFs: uv run scripts/compare-firmware-codegen.py OLD NEW.

Uses GNU Arm binutils. Symbol deltas reflect inlining and outlining too;
instruction counts are static counts, not executed instructions or timings.
"""

import collections
import json
import re
import subprocess
import sys


def inspect(path):
    def run(tool, *args):
        return subprocess.check_output([f"arm-none-eabi-{tool}", *args, path], text=True)

    sections = {}
    for line in run("size", "-A").splitlines():
        match = re.match(r"(\.\S+)\s+(\d+)\s+\d+", line)
        if match:
            sections[match[1]] = int(match[2])
    symbols = collections.defaultdict(int)
    for line in run("nm", "-C", "-S").splitlines():
        match = re.match(r"[0-9a-f]+\s+([0-9a-f]+)\s+[tT]\s+(.+)", line)
        if match:
            symbols[match[2]] += int(match[1], 16)
    instructions = collections.Counter()
    for line in run("objdump", "-d", "--no-show-raw-insn").splitlines():
        match = re.match(r"\s*[0-9a-f]+:\s+(\S+)", line)
        if match and not match[1].startswith("."):
            instructions[match[1]] += 1
    return sections, symbols, instructions


old_sections, old_symbols, old_instructions = inspect(sys.argv[1])
new_sections, new_symbols, new_instructions = inspect(sys.argv[2])
changes = [
    {"symbol": name, "old": old_symbols.get(name, 0), "new": new_symbols.get(name, 0),
     "delta": new_symbols.get(name, 0) - old_symbols.get(name, 0)}
    for name in old_symbols.keys() | new_symbols.keys()
    if old_symbols.get(name, 0) != new_symbols.get(name, 0)
]
print(json.dumps({
    "old": sys.argv[1], "new": sys.argv[2],
    "sections": {name: {"old": old_sections.get(name, 0), "new": new_sections.get(name, 0),
                         "delta": new_sections.get(name, 0) - old_sections.get(name, 0)}
                 for name in old_sections.keys() | new_sections.keys()
                 if name in (".vector_table", ".text", ".rodata", ".data", ".bss")},
    "symbols": sorted(changes, key=lambda item: abs(item["delta"]), reverse=True),
    "instructions": {name: {"old": old_instructions.get(name, 0), "new": new_instructions.get(name, 0)}
                     for name in sorted(old_instructions.keys() | new_instructions.keys())},
}, indent=2))
