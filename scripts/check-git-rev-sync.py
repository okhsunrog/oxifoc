#!/usr/bin/env python3
"""Assert that every git-pinned dependency resolves to the SAME source
(URL + rev) across all Cargo.lock files in the repo.

The workspace excludes the firmware crates (different toolchain), so the
repo carries ~10 independent lock files. Bumping a git rev (ergot, slint,
blec, …) means touching all of them — and a stale rev in one lock file
has already slipped through once, surviving unnoticed until an on-target
test build happened to fail. This guard catches the desync directly and
deterministically instead of as a compilation side effect.
"""

import pathlib
import re
import sys

SKIP_PARTS = {"target", "build", ".git"}

# Trouble pins its optional crypto dependency to its own Embassy snapshot.
# The BLE builds leave Trouble's security feature disabled. Keep checking
# the active Embassy pins used by the motor controllers and host workspace.
TROUBLE_LOCKS = {"oxifoc-bridge/Cargo.lock", "oxifoc-remote/Cargo.lock"}
TROUBLE_CRYPTO_REV = "bd35bd7a7d19147c8bcae9c2eda99b25e597294c"


def main() -> int:
    root = pathlib.Path(__file__).resolve().parent.parent
    locks = sorted(
        p
        for p in root.rglob("Cargo.lock")
        if not SKIP_PARTS.intersection(p.relative_to(root).parts)
    )
    if not locks:
        print("check-git-rev-sync: no Cargo.lock files found", file=sys.stderr)
        return 1

    # package name -> source string -> set of lock files
    sources: dict[str, dict[str, set[str]]] = {}
    for lock in locks:
        for block in lock.read_text().split("[[package]]"):
            name_m = re.search(r'^name = "([^"]+)"', block, re.M)
            src_m = re.search(r'^source = "(git\+[^"]+)"', block, re.M)
            if name_m and src_m:
                rel = str(lock.relative_to(root))
                name, source = name_m.group(1), src_m.group(1)
                if (
                    name == "embassy-crypto"
                    and rel in TROUBLE_LOCKS
                    and source
                    == f"git+https://github.com/embassy-rs/embassy.git?rev={TROUBLE_CRYPTO_REV}#{TROUBLE_CRYPTO_REV}"
                ):
                    continue
                sources.setdefault(name, {}).setdefault(source, set()).add(rel)

    bad = {name: srcs for name, srcs in sources.items() if len(srcs) > 1}
    if bad:
        print("git-pinned dependency revs DIVERGE across lock files:\n")
        for name, srcs in sorted(bad.items()):
            print(f"  {name}:")
            for src, files in sorted(srcs.items()):
                print(f"    {src}")
                for f in sorted(files):
                    print(f"      - {f}")
        print(
            "\nbump the rev in every Cargo.toml that pins it, then refresh "
            "ALL lock files (cargo update -p <pkg> in each crate dir)."
        )
        return 1

    n_pinned = len(sources)
    print(
        f"git-rev sync OK: {n_pinned} git-pinned package(s) consistent "
        f"across {len(locks)} lock files"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
