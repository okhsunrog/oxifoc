"""Integration tests for the staged Rust formatting hook."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


SOURCE = Path(__file__).resolve().parents[1]


class FormatStagedTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(dir=os.environ.get("TMPDIR"))
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.git("init", "--quiet")
        self.git("config", "user.name", "Test User")
        self.git("config", "user.email", "test@example.invalid")
        self.git("config", "commit.gpgsign", "false")
        self.git("config", "core.hooksPath", ".githooks")
        for name in [".githooks/pre-commit", "scripts/format_staged.py"]:
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(SOURCE / name, path)
        self.write("Cargo.toml", '[package]\nname = "fixture"\nversion = "0.1.0"\nedition = "2024"\n')
        self.write("rust-toolchain.toml", '[toolchain]\nchannel = "stable"\n')

    def git(self, *args):
        return subprocess.check_output(["git", *args], cwd=self.root)

    def write(self, name, text):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)

    def hook(self):
        return subprocess.run([str(self.root / ".githooks/pre-commit")], cwd=self.root,
                              capture_output=True, text=True)

    def test_commit_formats_and_stages_a_file_with_spaces(self):
        name = "src/file with spaces.rs"
        self.write(name, "fn main(){let x=1;}\n")
        self.git("add", name)
        self.git("commit", "--quiet", "-m", "Fixture")
        formatted = "fn main() {\n    let x = 1;\n}\n"
        self.assertEqual((self.root / name).read_text(), formatted)
        self.assertEqual(self.git("show", f"HEAD:{name}").decode(), formatted)
        self.assertEqual(self.git("diff", "--", name), b"")

    def test_partial_file_failure_preserves_every_file_and_index(self):
        staged = "fn main(){let x=1;}\n"
        self.write("src/aaa.rs", staged)
        self.write("src/main.rs", staged)
        self.git("add", "src")
        working = staged + "// unstaged\n"
        self.write("src/main.rs", working)
        result = self.hook()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unstaged changes", result.stderr)
        self.assertEqual((self.root / "src/main.rs").read_text(), working)
        self.assertEqual((self.root / "src/aaa.rs").read_text(), staged)
        self.assertEqual(self.git("show", ":src/aaa.rs").decode(), staged)
        self.assertEqual(self.git("show", ":src/main.rs").decode(), staged)

    def test_formatted_partial_file_keeps_unstaged_hunks(self):
        staged = "fn main() {}\n"
        self.write("src/main.rs", staged)
        self.git("add", "src/main.rs")
        working = staged + "// unstaged\n"
        self.write("src/main.rs", working)
        self.assertEqual(self.hook().returncode, 0)
        self.assertEqual((self.root / "src/main.rs").read_text(), working)
        self.assertEqual(self.git("show", ":src/main.rs").decode(), staged)

    def test_excluded_crate_uses_its_edition_and_toolchain(self):
        self.write("device/Cargo.toml", '[package]\nname = "device"\nversion = "0.1.0"\nedition = "2021"\n')
        self.write("device/rust-toolchain.toml", (SOURCE / "oxifoc-f405/rust-toolchain.toml").read_text())
        self.write("device/src/lib.rs", "pub fn gen( ){ }\n")
        self.git("add", "device/src/lib.rs")
        result = self.hook()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git("show", ":device/src/lib.rs"), b"pub fn gen() {}\n")

    def test_workspace_edition_is_inherited(self):
        self.write("Cargo.toml", '[workspace]\nmembers = ["member"]\n[workspace.package]\nedition = "2021"\n')
        self.write("member/Cargo.toml", '[package]\nname = "member"\nversion = "0.1.0"\nedition.workspace = true\n')
        self.write("member/src/lib.rs", "pub fn gen( ){ }\n")
        self.git("add", "member/src/lib.rs")
        result = self.hook()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git("show", ":member/src/lib.rs"), b"pub fn gen() {}\n")

    def test_standalone_source_uses_edition_2024(self):
        (self.root / "Cargo.toml").unlink()
        self.write("standalone.rs", "async fn run( ){ }\n")
        self.git("add", "standalone.rs")
        result = self.hook()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.git("show", ":standalone.rs"), b"async fn run() {}\n")


if __name__ == "__main__":
    unittest.main()
