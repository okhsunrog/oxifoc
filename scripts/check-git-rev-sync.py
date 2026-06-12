#!/usr/bin/env python3
"""Assert that every git-pinned dependency resolves to the SAME source
(URL + rev) across all Cargo.lock files in the repo.

The workspace excludes the firmware crates (different toolchain), so the
repo carries ~10 independent lock files. Bumping a git rev (ergot, slint,
bluest, …) means touching all of them — and a stale rev in one lock file
has already slipped through once, surviving unnoticed until an on-target
test build happened to fail. This guard catches the desync directly and
deterministically instead of as a compilation side effect.
"""

import pathlib
import re
import sys

SKIP_PARTS = {"target", "build", ".git"}


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
                sources.setdefault(name_m.group(1), {}).setdefault(
                    src_m.group(1), set()
                ).add(rel)

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
