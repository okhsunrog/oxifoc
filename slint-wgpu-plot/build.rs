fn main() {
    // Mark UI file as an input — consumers import it via with_library_paths.
    println!("cargo::rerun-if-changed=ui/plot.slint");
}
