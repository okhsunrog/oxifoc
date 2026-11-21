"""F405 ADC ISR placement tools: link capture, controlled relinks, static reports.

Run from the repository root inside `nix develop`; rust-lld comes from the
oxifoc-f405 toolchain, rust-nm/rust-objdump/rust-objcopy from cargo-binutils.
See docs/performance.md ("Placement decides tens of cycles") for the method.

  build DIR [--features F] [--defmt-log L]
      Build the MK5 image and capture its final link: DIR/build.elf,
      DIR/build.txt (provenance), DIR/link/ (extracted `--reproduce` tar).
  relink DIR NAME [--res RE@R ...] [--resfrom DIR2] [--allres R]
                  [--shuffle SEED] [--pin LABEL=R ...] [--add-pin LABEL:GLOB@R ...]
                  [--foc-hot FILE]
      Re-run rust-lld on the captured objects with controlled placement and
      write DIR/NAME.elf. With no options the result must be byte-identical
      to DIR/build.elf (the control relink).
  residues ELF...
      Entry residue (mod 16) of every foc-hot.x section, and where the RAM
      statics the ISR references live relative to SRAM2.
  callgraph ELF... [--all]
      Functions reachable from ADC: instructions, entry residue, call sites.
  diff A.elf B.elf [--function NAME] [--hot]
      Per-function disassembly comparison with addresses normalised away.

Residues are entry addresses mod 16 (one Flash line); R is a hex digit.
"""
import argparse
import bisect
import difflib
import fnmatch
import hashlib
import os
import random
import re
import shutil
import subprocess
import sys
from collections import Counter, namedtuple
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
F405 = ROOT / "oxifoc-f405"
MK5_FEATURES = "transport-spi,board-vesc6-mk5"
DEFMT_LOG = "warn,oxifoc_f405::control::foc=info"
# One target dir for every capture: paths of build outputs can reach the
# image, so A/B builds must share it (and the checkout directory).
TARGET_DIR = "target/isr-capture"
SRAM2 = 0x2001C000
LINE = 16

# .text input sections that stay in place when shuffling or forcing residues.
HEAD = (".Reset", ".text.__pre_init")
HASH = re.compile(r"17h[0-9a-f]{16}E|::h[0-9a-f]{16}")


# --- map files and residue arithmetic (pure) ---------------------------------

InSec = namedtuple("InSec", "vma size align file name symbol")


def parse_map(text):
    """lld -Map text -> {output section: [InSec]} in link order.

    `symbol` is the first non-mapping symbol defined in the input section."""
    out, cur = {}, None
    num = re.compile(r"^\s*([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+(\d+) ")
    for line in text.splitlines():
        m = num.match(line)
        if not m:
            continue
        rest = line[m.end():]
        body = rest.lstrip(" ")
        indent = len(rest) - len(body)
        vma, size, align = int(m.group(1), 16), int(m.group(3), 16), int(m.group(4))
        if indent == 0:
            cur = out.setdefault(body, [])
        elif indent == 8 and cur is not None:
            s = re.match(r"^(.*):\((.*)\)$", body)
            if s:
                cur.append(InSec(vma, size, align, s.group(1), s.group(2), None))
        elif indent == 16 and cur and cur[-1].symbol is None and not body.startswith("$"):
            if vma & ~1 == cur[-1].vma:  # Thumb symbols carry bit 0
                cur[-1] = cur[-1]._replace(symbol=body)
    return out


def entry(cur, residue, align):
    """Lowest address >= cur at `residue` mod 16 that also honours `align`."""
    if residue % min(align, LINE):
        raise ValueError(f"residue {residue:x} conflicts with alignment {align}")
    a = (cur & ~(LINE - 1)) + residue
    if a < cur:
        a += LINE
    while a % align:
        a += LINE
    return a


def align_up(cur, align):
    return (cur + align - 1) // align * align


def key(sec):
    """Identity of an input section across builds: its first symbol
    without hash, else the section name."""
    return HASH.sub("", sec.symbol or sec.name)


def residues_from(ref_secs):
    """Reference link -> {key: [residues in link order]}."""
    ref = {}
    for s in ref_secs:
        ref.setdefault(key(s), []).append(s.vma % LINE)
    return ref


def plan_text(secs, order, wanted):
    """Assign addresses to .text input sections.

    `order` lists indices into `secs`; `wanted[i]` is a residue or None
    (natural packing). Returns ([(index, address)], conflicts, end)."""
    cur = secs[0].vma
    placed, conflicts = [], 0
    for i in order:
        s, want = secs[i], wanted.get(i)
        if want is not None and want % min(s.align, LINE):
            conflicts += 1
            want = None
        a = align_up(cur, s.align) if want is None else entry(cur, want, s.align)
        placed.append((i, a))
        cur = a + s.size
    return placed, conflicts, cur


def set_pin_residue(script, label, residue):
    """Change the entry residue of the foc-hot.x stanza `__foc_LABEL_start`."""
    rx = re.compile(r"\. = ALIGN\(16\)(?: \+ \d+)?;(\s*__foc_%s_start = \.;)" % re.escape(label))
    if len(rx.findall(script)) != 1:
        raise ValueError(f"no single __foc_{label}_start stanza in foc-hot.x")
    tail = f" + {residue}" if residue else ""
    return rx.sub(lambda m: f". = ALIGN(16){tail};{m.group(1)}", script)


def add_pin(script, label, glob, residue):
    """Append a pinned stanza (as foc-hot.x would declare it) to .foc_hot."""
    k = script.index("  } > FLASH")
    stanza = (f"    . = ALIGN(16) + {residue};\n    __foc_{label}_start = .;\n"
              f"    KEEP(*({glob}));\n    __foc_{label}_end = .;\n")
    check = f'ASSERT(__foc_{label}_end > __foc_{label}_start, "Missing FOC {label} section");\n'
    return script[:k] + stanza + script[k:] + check


def replace_text_body(link_x, lines):
    m = re.search(r"(\.text _stext :\s*\{)(.*?)(\} > FLASH)", link_x, re.S)
    if not m:
        raise ValueError("no .text output section in link.x")
    body = "\n".join(["    __stext = .;", *lines, "    . = ALIGN(4);", "    __etext = .;"])
    return link_x[:m.start(2)] + "\n" + body + "\n  " + link_x[m.start(3):]


# --- toolchain -----------------------------------------------------------------

def run(args, **kw):
    return subprocess.run(args, check=True, capture_output=True, text=True, **kw).stdout


def rust_lld():
    sysroot = run(["rustc", "--print", "sysroot"], cwd=F405).strip()
    host = re.search(r"^host: (\S+)", run(["rustc", "-vV"], cwd=F405), re.M).group(1)
    lld = Path(sysroot) / "lib" / "rustlib" / host / "bin" / "rust-lld"
    if not lld.exists():
        sys.exit(f"rust-lld not found at {lld}; install the llvm-tools component")
    return lld


def objcopy_bin(elf):
    tmp = Path(str(elf) + ".bin")
    run(["rust-objcopy", "-O", "binary", str(elf), str(tmp)])
    data = tmp.read_bytes()
    tmp.unlink()
    return data


# --- build ---------------------------------------------------------------------

def cmd_build(a):
    out = Path(a.dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    tar, link = out / "link.tar", out / "link"
    tar.unlink(missing_ok=True)
    shutil.rmtree(link, ignore_errors=True)
    env = dict(os.environ, DEFMT_LOG=a.defmt_log)
    subprocess.run(
        ["cargo", "rustc", "--release", "--locked", "--target-dir", TARGET_DIR,
         "--no-default-features", "--features", a.features, "--bin", "oxifoc-f405", "--",
         "-C", f"link-arg=--reproduce={tar}"],
        cwd=F405, env=env, check=True)
    elf = F405 / TARGET_DIR / "thumbv7em-none-eabihf/release/oxifoc-f405"
    shutil.copy(elf, out / "build.elf")
    # flip-link links twice; the second (final) link overwrites the tar.
    subprocess.run(["tar", "xf", str(tar), "-C", str(out)], check=True)
    if not (link / "response.txt").exists():
        sys.exit(f"{tar} did not unpack to {link}")
    rev = run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT).strip()
    dirty = len(run(["git", "status", "--porcelain"], cwd=ROOT).splitlines())
    size = run(["rust-size", str(out / "build.elf")]).splitlines()[-1].split()
    info = (f"rev={rev} dirty_files={dirty}\nfeatures={a.features}\nDEFMT_LOG={a.defmt_log}\n"
            f"rustc={run(['rustc', '-V'], cwd=F405).strip()}\n"
            f"text={size[0]} data={size[1]} bss={size[2]}\n"
            f"elf_sha256={hashlib.sha256((out / 'build.elf').read_bytes()).hexdigest()}\n"
            f"bin_sha256={hashlib.sha256(objcopy_bin(out / 'build.elf')).hexdigest()}\n")
    (out / "build.txt").write_text(info)
    print(info, end="")


# --- relink --------------------------------------------------------------------

def unique(root, name, prefer=None):
    hits = sorted(p for p in root.rglob(name) if p.is_file())
    if prefer:
        hits = [p for p in hits if prefer in str(p)] or hits
    if not hits:
        sys.exit(f"{name} not found under {root}")
    return hits[0]


class Link:
    """A captured link: DIR/link/ from `build`."""

    def __init__(self, d):
        self.dir = Path(d).resolve()
        self.root = self.dir / "link"
        if not (self.root / "response.txt").exists():
            sys.exit(f"{self.root}/response.txt missing; run `build {d}` first")
        # flip-link writes its memory.x to a temp dir that is not on the
        # captured -L path; the tar holds the file, so add its directory.
        self.memdir = unique(self.root / "home", "memory.x", "flip-link").parent
        self.link_x = unique(self.root / "home", "link.x")
        self.foc_hot = unique(self.root / "home", "foc-hot.x")

    def link(self, lld, name, scripts=None):
        """Link as NAME with replaced scripts {captured name: new text}."""
        rsp = (self.root / "response.txt").read_text()
        rsp = re.sub(r"(?m)^--Map \S+$", "", rsp)
        rsp = re.sub(r"(?m)^-o \S+$", f"-o {name}.elf", rsp) + f"\n-Map {name}.map\n"
        for orig, text in (scripts or {}).items():
            new = f"{name}.{orig}"
            (self.root / new).write_text(text)
            if f"--script {orig}" not in rsp:
                sys.exit(f"--script {orig} not in response.txt")
            rsp = rsp.replace(f"--script {orig}", f"--script {new}")
        (self.root / f"{name}.rsp").write_text(rsp)
        r = subprocess.run([str(lld), "-flavor", "gnu", f"@{name}.rsp", "-L",
                            str(self.memdir.relative_to(self.root))],
                           cwd=self.root, capture_output=True, text=True)
        if r.returncode:
            sys.exit(r.stderr[:3000])
        if r.stderr.strip():
            print(r.stderr[:1500], file=sys.stderr)
        return self.root / f"{name}.elf", self.root / f"{name}.map"

    def control_text(self, lld):
        """.text input sections of the unmodified link (lld demangles map symbols)."""
        m = self.root / "ctl.map"
        if not m.exists():
            self.link(lld, "ctl")
        return parse_map(m.read_text())[".text"]


def parse_res(text):
    rx, _, r = text.rpartition("@")
    return rx, int(r, 16)


def cmd_relink(a):
    if a.name in ("build", "link", "ctl"):
        sys.exit(f"{a.name} is reserved")
    lld = rust_lld()
    L = Link(a.dir)
    scripts = {}
    hot = Path(a.foc_hot).read_text() if a.foc_hot else L.foc_hot.read_text()
    for p in a.pin:
        label, r = p.split("=")
        hot = set_pin_residue(hot, label, int(r, 16))
    moved = []
    for p in a.add_pin:
        label, rest = p.split(":", 1)
        glob, r = parse_res(rest)
        hot = add_pin(hot, label, glob, r)
        moved.append(re.compile(fnmatch.translate(glob)))
    if a.foc_hot or a.pin or a.add_pin:
        scripts["foc-hot.x"] = hot

    explicit = a.res or a.resfrom or a.allres is not None or a.shuffle is not None
    planned = []
    if explicit:
        secs = [s for s in L.control_text(lld) if not any(rx.match(s.name) for rx in moved)]
        names = Counter(s.name for s in secs)
        dup = [n for n, c in names.items() if c > 1]
        if dup:
            sys.exit(f"input section names are not unique, cannot place them: {dup[:5]}")
        fixed = [i for i, s in enumerate(secs) if s.name in HEAD or s.name.startswith(".HardFault")]
        order = list(range(len(secs)))
        if a.shuffle is not None:
            mid = [i for i in order if i not in fixed]
            random.Random(a.shuffle).shuffle(mid)
            order = [i for i in order if i in fixed and secs[i].name in HEAD] + mid + \
                    [i for i in order if i in fixed and secs[i].name not in HEAD]
        wanted = {}
        if a.resfrom:
            ref = residues_from(Link(a.resfrom).control_text(lld))
            hit = 0
            for i, s in enumerate(secs):
                lst = ref.get(key(s))
                hit += bool(lst)
                wanted[i] = lst.pop(0) if lst else s.vma % LINE
            print(f"resfrom: {hit} of {len(secs)} sections matched by name")
        elif a.shuffle is None:
            wanted = {i: s.vma % LINE for i, s in enumerate(secs)}
        if a.allres is not None:
            wanted.update({i: int(a.allres, 16) for i in range(len(secs)) if i not in fixed})
        for spec in a.res:
            rx, r = parse_res(spec)
            rx = re.compile(rx)
            hits = [i for i, s in enumerate(secs) if rx.search(s.name) or rx.search(s.symbol or "")]
            if not hits:
                sys.exit(f"--res {spec}: no .text input section matches")
            wanted.update({i: r for i in hits})
        placed, conflicts, end = plan_text(secs, order, wanted)
        natural = secs[-1].vma + secs[-1].size
        print(f"text layout: {len(placed)} sections, {conflicts} residue/alignment conflicts kept natural, "
              f".text end {end - natural:+d} bytes")
        lines = [f'    . = 0x{addr:x}; *("{secs[i].name}");' for i, addr in placed]
        scripts["link.x"] = replace_text_body(L.link_x.read_text(), lines)
        planned = [(secs[i].name, addr) for i, addr in placed]

    elf, mp = L.link(lld, a.name, scripts)
    if planned:
        got = {s.name: s.vma for s in parse_map(mp.read_text())[".text"]}
        bad = [(n, x) for n, x in planned if got.get(n) != x]
        if bad:
            sys.exit(f"{len(bad)} sections off their planned address, first {bad[0][0]}")
    shutil.copy(elf, L.dir / f"{a.name}.elf")
    same = objcopy_bin(L.dir / f"{a.name}.elf") == objcopy_bin(L.dir / "build.elf")
    print(f"{L.dir / a.name}.elf: {'identical to' if same else 'differs from'} build.elf (objcopy -O binary)")


# --- static reports --------------------------------------------------------------

def symbols(elf):
    """rust-nm -> [(addr, size, type, mangled, demangled)] in symbol table order."""
    args = ["rust-nm", "-S", "-p", "--defined-only", str(elf)]
    raw = run(args).splitlines()
    dem = run(args[:1] + ["--demangle"] + args[1:]).splitlines()
    out = []
    for r, d in zip(raw, dem):
        p, q = r.split(None, 3), d.split(None, 3)
        if len(p) == 4:
            out.append((int(p[0], 16), int(p[1], 16), p[2], p[3], HASH.sub("", q[3])))
        elif len(p) == 3:
            out.append((int(p[0], 16), 0, p[1], p[2], HASH.sub("", q[2])))
    return out


def strip_generics(name):
    """Drop a trailing balanced `::<...>` (turbofish) from a demangled path."""
    if not name.endswith(">"):
        return name
    depth = 0
    for i in range(len(name) - 1, -1, -1):
        depth += {">": 1, "<": -1}.get(name[i], 0)
        if depth == 0:
            return name[:i - 2] if name[i - 2:i] == "::" else name
    return name


def short(name):
    """Readable form of a demangled Rust path: `FocDriver::step`,
    `PhaseManager as PhaseProvider::injection`, `state::run_foc_cycle`."""
    name = strip_generics(name)

    def base(path):
        m = re.match(r"[&*\[( ]*(?:mut |const |dyn )?(?:\w+::)*(\w+)", path)
        return m.group(1) if m else path
    if name.startswith("<"):
        depth = 0
        for i, c in enumerate(name):
            depth += {"<": 1, ">": -1}.get(c, 0)
            if depth == 0:
                break
        head, method = name[1:i], name[i + 3:]
        depth, split = 0, None
        for j, c in enumerate(head):
            depth += {"<": 1, ">": -1}.get(c, 0)
            if depth == 0 and head.startswith(" as ", j):
                split = j
        typ = base(head if split is None else head[:split])
        if split is not None:
            typ += " as " + base(head[split + 4:])
        return f"{typ}::{short(method)}" if method else typ
    return re.sub(r"\b(oxifoc_core|oxifoc_f405)::", "", name)


Func = namedtuple("Func", "addr size mangled name insns")
BRANCH = re.compile(r"^(bl|b|b\.w|b[a-z]{2}|b[a-z]{2}\.w|cbn?z)\t(?:\w+, )?0x([0-9a-f]+)\b")


def parse_objdump(text):
    """`rust-objdump -d --no-show-raw-insn` -> [(address, instruction)].

    Data in code (`.word` etc.) keeps its raw bytes in the output; they are
    dropped here, as are comments."""
    out = []
    for line in text.splitlines():
        m = re.match(r"^\s*([0-9a-f]+):\s(.*)$", line)
        if not m:
            continue
        fields = [f.strip() for f in m.group(2).split("\t")]
        fields = [f for f in fields if f and not re.fullmatch(r"(?:[0-9a-f]{2} ?)+", f)]
        if fields:
            out.append((int(m.group(1), 16), "\t".join(fields).split(" @ ")[0].rstrip()))
    return out


def functions(syms, insns):
    """Text symbols with a size, each with its instructions -> {addr: Func}."""
    funcs = {}
    for addr, size, typ, mangled, name in syms:
        addr &= ~1
        if typ in "tT" and size and addr not in funcs:
            funcs[addr] = Func(addr, size, mangled, name, [])
    starts = sorted(funcs)
    for addr, ins in insns:
        k = bisect.bisect_right(starts, addr) - 1
        if k >= 0 and addr < starts[k] + funcs[starts[k]].size:
            funcs[starts[k]].insns.append(ins)
    return funcs


def callees(func, funcs):
    """Direct calls and tail branches to other functions -> {addr: sites}."""
    out = Counter()
    for ins in func.insns:
        m = BRANCH.match(ins)
        if m:
            t = int(m.group(2), 16)
            if t in funcs and t != func.addr:
                out[t] += 1
    return out


def reachable(funcs, root="ADC"):
    """Functions reachable from `root` by direct calls -> {addr: depth}.

    Indirect calls (`blx rN`, function pointers) are not followed."""
    start = [a for a, f in funcs.items() if f.mangled == root]
    seen, stack = {}, [(a, 0) for a in start]
    while stack:
        a, depth = stack.pop()
        if a in seen:
            continue
        seen[a] = depth
        stack.extend((c, depth + 1) for c in callees(funcs[a], funcs))
    return seen


def load(elf):
    syms = symbols(elf)
    insns = parse_objdump(run(["rust-objdump", "-d", "--no-show-raw-insn", str(elf)]))
    return syms, functions(syms, insns)


def is_insn(ins):
    return not ins.startswith(".")


def label(elf):
    p = Path(elf)
    return f"{p.parent.name}/{p.stem}"[-14:]


def cmd_callgraph(a):
    tabs = []
    for elf in a.elf:
        _, funcs = load(elf)
        seen = reachable(funcs)
        sites = Counter()
        for f in seen:
            sites.update(callees(funcs[f], funcs))
        tabs.append({funcs[f].name: (f, sum(map(is_insn, funcs[f].insns)), sites[f]) for f in seen})
    names = sorted(set().union(*tabs), key=lambda n: -max(t.get(n, (0, 0))[1] for t in tabs))
    print("insns@residue (call sites on the ADC path); '-' = not reachable (inlined or absent)")
    print(f"{'function':64}", *[label(e).rjust(14) for e in a.elf])
    for n in names:
        cells = [f"{t[n][1]}@{t[n][0] % LINE:x} ({t[n][2]})" if n in t else "-" for t in tabs]
        if a.all or len(tabs) == 1 or len(set(cells)) > 1:
            print(f"{short(n)[:64]:64}", *[c.rjust(14) for c in cells])


def region(addr):
    if 0x10000000 <= addr < 0x10010000:
        return "CCM"
    if 0x20000000 <= addr < SRAM2:
        return "SRAM1"
    if SRAM2 <= addr < 0x20020000:
        return "SRAM2"
    return "?"


def ram_refs(insns):
    """RAM addresses formed by movw/movt pairs or literal words."""
    lo, out = {}, []
    for ins in insns:
        m = re.match(r"movw\s+(\w+), #(0x[0-9a-f]+|\d+)", ins)
        if m:
            lo[m.group(1)] = int(m.group(2), 0)
            continue
        m = re.match(r"movt\s+(\w+), #(0x[0-9a-f]+|\d+)", ins)
        if m and m.group(1) in lo:
            out.append((int(m.group(2), 0) << 16) | lo.pop(m.group(1)))
            continue
        m = re.match(r"\.word\s+(0x[0-9a-f]+)", ins)
        if m:
            out.append(int(m.group(1), 16))
    return [x for x in out if region(x) != "?"]


def owner(syms, addr):
    best = None
    for s in syms:
        if s[2] in "bBdD" and s[0] <= addr < s[0] + max(s[1], 1):
            if best is None or s[0] > best[0]:
                best = s
    return best


def cmd_residues(a):
    bad = 0
    for elf in a.elf:
        syms, funcs = load(elf)
        marks = {s[3]: s[0] for s in syms if s[3].startswith("__foc_")}
        print(f"== {elf}")
        print(f"pinned (foc-hot.x)   {'start':8} res  bytes  function")
        labels = sorted({m[6:-6] for m in marks if m.endswith("_start")}, key=lambda l: marks[f"__foc_{l}_start"])
        for label in labels:
            s, e = marks[f"__foc_{label}_start"], marks.get(f"__foc_{label}_end", 0)
            f = funcs.get(s)
            print(f"  {label:18} {s:08x}  {s % LINE:2x}  {e - s:5}  {short(f.name) if f else '?'}")
        refs = Counter()
        for f in reachable(funcs):
            for x in ram_refs(funcs[f].insns):
                o = owner(syms, x)
                refs[(o[0], o[1], o[4]) if o else (x, 0, f"?{x:#x}")] += 1
        print(f"RAM statics referenced on the ADC path (SRAM2 starts at {SRAM2:#x})")
        for (addr, size, name), n in sorted(refs.items()):
            end = addr + max(size, 1) - 1
            where = region(addr) if region(addr) == region(end) else f"{region(addr)}/{region(end)}"
            flag = "  <-- SRAM2" if "SRAM2" in where else ""
            bad += bool(flag)
            print(f"  {addr:08x} {size:6} {where:7} {n:3}x {name[:100]}{flag}")
        cold = {s[3]: s[0] for s in syms if s[3] in ("__bss_cold_start", "__bss_cold_end")}
        if len(cold) == 2:
            print(f"  .bss_cold filler {cold['__bss_cold_start']:08x}..{cold['__bss_cold_end']:08x}")
    if bad:
        sys.exit(f"{bad} ISR statics reach into SRAM2")


def normalise(func, funcs):
    """Instructions of `func` with addresses, literal values and branch
    offsets replaced, so the same code at another address compares equal.
    Call targets keep their (demangled) name."""
    out = []
    for ins in func.insns:
        ins = re.sub(r"^\.(word|short|byte|long)\s.*", r".\1 X", ins)

        def target(m):
            t = int(m.group(1), 16)
            if func.addr <= t < func.addr + func.size:
                return "<.>"
            return f"<{funcs[t].name}>" if t in funcs else "<?>"
        ins = re.sub(r"0x([0-9a-f]+) <[^>]*>", target, ins)
        ins = re.sub(r"0x[0-9a-f]+", "X", ins)
        if re.match(r"(movw|movt|b[a-z.]*|cbn?z)$", ins.split("\t")[0]):
            ins = re.sub(r"#-?(0x[0-9a-f]+|\d+)", "#N", ins)
        out.append(ins)
    return out


def norm_funcs(elf):
    """ELF -> ({unique name: normalised instructions}, names on the ADC path)."""
    _, funcs = load(elf)
    out, keys = {}, {}
    for addr in sorted(funcs):
        f = funcs[addr]
        k, i = f.name, 1
        while k in out:
            i += 1
            k = f"{f.name}#{i}"
        out[k] = normalise(f, funcs)
        keys[addr] = k
    return out, {keys[a] for a in reachable(funcs)}


def cmd_diff(a):
    A, hot_a = norm_funcs(a.a)
    B, hot_b = norm_funcs(a.b)
    if a.function:
        hits = [k for k in A if k == a.function or short(k) == a.function] or \
               [k for k in A if a.function in k]
        if not hits:
            sys.exit(f"no function matching {a.function!r} in {a.a}")
        for k in hits[:5]:
            if k not in B:
                print(f"== {k}: only in A")
                continue
            sm = difflib.SequenceMatcher(None, A[k], B[k], autojunk=False)
            hunks = [op for op in sm.get_opcodes() if op[0] != "equal"]
            print(f"== {short(k)}: A={len(A[k])} B={len(B[k])} instructions, {len(hunks)} hunks")
            for tag, i1, i2, j1, j2 in hunks:
                print(f"@@ A[{i1}:{i2}] B[{j1}:{j2}] {tag}")
                for x in A[k][i1:i2][:a.context]:
                    print("  -", x)
                for x in B[k][j1:j2][:a.context]:
                    print("  +", x)
        return
    keep = set(A) | set(B)
    if a.hot:
        keep &= hot_a | hot_b
    only_a = sorted(k for k in keep if k not in B)
    only_b = sorted(k for k in keep if k not in A)
    changed = sorted(k for k in keep if k in A and k in B and A[k] != B[k])
    na = sum(len(A[k]) for k in keep if k in A)
    nb = sum(len(B[k]) for k in keep if k in B)
    print(f"functions A={sum(k in A for k in keep)} B={sum(k in B for k in keep)} "
          f"instructions A={na} B={nb} only_A={len(only_a)} only_B={len(only_b)} changed={len(changed)}")
    for k in only_a:
        print(f"  - {short(k)} {len(A[k])}")
    for k in only_b:
        print(f"  + {short(k)} {len(B[k])}")
    for k in changed:
        print(f"  ~ {short(k)} {len(A[k])} -> {len(B[k])}")
    if not (only_a or only_b or changed):
        print("same code: any timing difference is placement")


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="build the MK5 image and capture its link")
    b.add_argument("dir")
    b.add_argument("--features", default=MK5_FEATURES)
    b.add_argument("--defmt-log", default=DEFMT_LOG)
    r = sub.add_parser("relink", help="relink a captured link with controlled placement")
    r.add_argument("dir")
    r.add_argument("name")
    r.add_argument("--res", action="append", default=[], metavar="RE@R",
                   help="entry residue R of .text input sections whose name or first symbol matches RE")
    r.add_argument("--resfrom", metavar="DIR", help="give every .text section the residue it has in DIR's link")
    r.add_argument("--allres", metavar="R", help="every .text section at residue R (except reset/HardFault)")
    r.add_argument("--shuffle", type=int, metavar="SEED", help="seeded random .text order, natural packing")
    r.add_argument("--pin", action="append", default=[], metavar="LABEL=R",
                   help="move foc-hot.x stanza __foc_LABEL_start to residue R")
    r.add_argument("--add-pin", action="append", default=[], metavar="LABEL:GLOB@R",
                   help="pin input sections matching GLOB at residue R in .foc_hot")
    r.add_argument("--foc-hot", metavar="FILE", help="use FILE instead of the captured foc-hot.x")
    for name, fn, h in (("residues", cmd_residues, "pinned residues and ISR statics vs SRAM2"),
                        ("callgraph", cmd_callgraph, "functions reachable from ADC")):
        s = sub.add_parser(name, help=h)
        s.add_argument("elf", nargs="+")
        if name == "callgraph":
            s.add_argument("--all", action="store_true", help="with several ELFs, also print equal rows")
        s.set_defaults(fn=fn)
    d = sub.add_parser("diff", help="per-function normalised disassembly comparison")
    d.add_argument("a")
    d.add_argument("b")
    d.add_argument("--function", help="show instruction hunks of this function (exact or substring)")
    d.add_argument("--context", type=int, default=12, help="instructions shown per hunk side")
    d.add_argument("--hot", action="store_true", help="only functions reachable from ADC")
    b.set_defaults(fn=cmd_build)
    r.set_defaults(fn=cmd_relink)
    d.set_defaults(fn=cmd_diff)
    a = p.parse_args(argv)
    try:
        a.fn(a)
    except (ValueError, subprocess.CalledProcessError) as e:
        sys.exit(f"error: {e}")


if __name__ == "__main__":
    main()
