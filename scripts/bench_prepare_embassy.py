"""Prepare a private, pinned HAL override for F405 clock experiments.

The upstream unchecked-overclocking feature misses the f247 PLL VCO assert.
Change that assert to rcc_assert and clamp experimental F4 flash latency to
WS7 in a disposable copy, never in Cargo's cache. Above 240 MHz this is an
explicit flash overclock, not a vendor-supported timing configuration.
Keep all sibling dependencies on the original git revision and use a
separate lockfile; production dependency resolution remains untouched.
"""
import json
from pathlib import Path
import re
import shutil
import subprocess
import tomllib

repo = Path(__file__).resolve().parent.parent
dep = tomllib.loads((repo / "oxifoc-f405/Cargo.toml").read_text())["dependencies"]["embassy-stm32"]
rev, git = dep["rev"], dep["git"]
candidates = list((Path.home() / ".cargo/git/checkouts").glob(f"embassy-*/{rev[:7]}/embassy-stm32"))
assert len(candidates) == 1, "fetch the pinned Embassy dependency before preparing the bench"
source = candidates[0]
actual = subprocess.check_output(["git", "-C", str(source), "rev-parse", "HEAD"], text=True).strip()
assert actual == rev, "unexpected cached Embassy revision"
root = Path("/home/okhsunrog/tmp_zfs/codex_tmp/oxifoc-overclock-deps") / rev / "vco-flash-ws7"
target = root / "embassy-stm32"
if not target.exists():
    root.mkdir(parents=True, exist_ok=True)
    shutil.copytree(source, target)
    manifest = target / "Cargo.toml"
    text = manifest.read_text()
    text, count = re.subn(r'path = "\.\./embassy-[^"]+"', f'git = "{git}", rev = "{rev}"', text)
    assert count >= 10, "upstream manifest changed; review the bench override"
    manifest.write_text(text)
    module = target / "src/rcc/f247.rs"
    text = module.read_text()
    before = "    assert!(max::PLL_VCO.contains(&vco_freq));"
    assert text.count(before) == 1, "upstream VCO guard changed; review the patch"
    text = text.replace(before, "    rcc_assert!(max::PLL_VCO.contains(&vco_freq));")
    before = "        let latency = (hclk.0 - 1) / FLASH_LATENCY_STEP;"
    assert text.count(before) == 1
    text = text.replace(before, before + '\n        #[cfg(all(stm32f4, feature = "unchecked-overclocking"))]\n        let latency = latency.min(7);')
    module.write_text(text)
lock = root / "Cargo.lock"
if not lock.exists():
    shutil.copyfile(repo / "oxifoc-f405/Cargo.lock", lock)
config = root / "bench.toml"
config.write_text(f'[patch.{json.dumps(git)}]\nembassy-stm32 = {{ path = {json.dumps(str(target))} }}\n'
                  f'[resolver]\nlockfile-path = {json.dumps(str(lock))}\n')
print(config)
