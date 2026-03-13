use std::collections::HashMap;
use std::path::PathBuf;

fn main() {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let plot_ui = manifest.parent().unwrap().join("slint-wgpu-plot").join("ui");

    let mut library_paths = HashMap::new();
    library_paths.insert("slint-wgpu-plot".to_string(), plot_ui);

    slint_build::compile_with_config(
        "ui/app.slint",
        slint_build::CompilerConfiguration::new().with_library_paths(library_paths),
    )
    .unwrap();
}
