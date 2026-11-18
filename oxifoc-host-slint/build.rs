fn main() {
    let libraries = std::collections::HashMap::from([(
        "lucide".to_string(),
        std::path::PathBuf::from(lucide_slint::lib()),
    )]);
    slint_kit_build::compile_with_config(
        "ui/app.slint",
        slint_kit_build::config_with_libraries(libraries).with_edition((1, 19)),
    )
    .expect("OxiFOC Slint Kit UI");
}
