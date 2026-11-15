fn main() {
    oxifoc_host_lib::init_tracing();
    // Isolated KWin can leave GTK's DPI at its sentinel value.
    use gtk::prelude::*;
    gtk::init().expect("GTK initialization");
    if let Some(settings) = gtk::Settings::default() {
        if settings.gtk_xft_dpi() <= 0 {
            settings.set_gtk_xft_dpi(96 * 1024);
        }
    }
    if let Some(screen) = gtk::gdk::Screen::default() {
        if screen.resolution() <= 0. {
            screen.set_resolution(96.);
        }
    }
    let compact = std::env::args().any(|s| s == "--compact");
    dioxus::LaunchBuilder::desktop()
        .with_cfg(
            dioxus::desktop::Config::new().with_menu(None).with_window(
                dioxus::desktop::WindowBuilder::new()
                    .with_title("OxiFOC · Drive studio")
                    .with_inner_size(dioxus::desktop::LogicalSize::new(
                        if compact { 420. } else { 1280. },
                        if compact { 900. } else { 920. },
                    )),
            ),
        )
        .launch(oxifoc_host_dioxus_ui::HostApp);
}
