"""Format staged Rust files with their crate's toolchain and edition."""

import os
from pathlib import Path
import subprocess
import tomllib


def git(*args, **kwargs):
    return subprocess.check_output(["git", *args], **kwargs)


def edition_for(path):
    for directory in path.parents:
        manifest = directory / "Cargo.toml"
        if not manifest.exists():
            continue
        data = tomllib.loads(manifest.read_text())
        if "package" not in data:
            continue
        edition = data["package"].get("edition", "2015")
        if isinstance(edition, str):
            return edition
        for workspace in [directory, *directory.parents]:
            manifest = workspace / "Cargo.toml"
            if manifest.exists():
                data = tomllib.loads(manifest.read_text())
                package = data.get("workspace", {}).get("package", {})
                if "edition" in package:
                    return package["edition"]
        raise SystemExit(f"Cannot resolve workspace edition for {path}")
    return "2024"


def main():
    root = Path(os.fsdecode(git("rev-parse", "--show-toplevel").rstrip(b"\n")))
    os.chdir(root)
    paths = git("diff", "--cached", "--name-only", "--diff-filter=ACMR", "-z").split(b"\0")
    staged_paths = {os.fsdecode(path) for path in paths if path.endswith(b".rs")}
    if not staged_paths:
        return
    updates = []
    for entry in git("ls-files", "--stage", "-z").split(b"\0"):
        if not entry:
            continue
        metadata, raw_path = entry.split(b"\t", 1)
        name = os.fsdecode(raw_path)
        mode, blob, stage = metadata.split()
        if name not in staged_paths or mode not in (b"100644", b"100755") or stage != b"0":
            continue
        path = root / name
        original = git("cat-file", "blob", blob.decode())
        result = subprocess.run(
            ["rustfmt", "--edition", edition_for(path), "--config", "skip_children=true", "--emit", "stdout"],
            cwd=path.parent, input=original, stdout=subprocess.PIPE, check=True,
        )
        formatted = result.stdout
        if formatted == original:
            continue
        if path.is_symlink() or not path.exists() or path.read_bytes() != original:
            raise SystemExit(
                f"{name} needs formatting and has unstaged changes. "
                "Format it, stage the desired changes, then retry the commit."
            )
        updates.append((name, mode.decode(), original, formatted))

    # Check every input before writing any formatted files.
    for name, mode, original, formatted in updates:
        path = root / name
        if path.is_symlink() or path.read_bytes() != original or git("show", f":{name}") != original:
            raise SystemExit(f"{name} changed during formatting. Retry the commit.")
    for name, mode, original, formatted in updates:
        (root / name).write_bytes(formatted)
        blob = git("hash-object", "-w", "--stdin", input=formatted).decode().strip()
        git("update-index", "--cacheinfo", mode, blob, name)
        print(f"Formatted {name}")


if __name__ == "__main__":
    main()
