fn main() {
    oxifoc_host_lib::init_tracing();
    // Isolated KWin can leave GTK's DPI at its sentinel value.
    use gtk::prelude::*;
    gtk::glib::set_prgname(Some("com.oxifoc.studio.webview"));
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
    let window = dioxus::desktop::WindowBuilder::new();
    let mut config = dioxus::desktop::Config::new();
    #[cfg(target_os = "linux")]
    {
        use dioxus::desktop::tao::platform::unix::EventLoopBuilderExtUnix;
        let mut builder = dioxus::desktop::tao::event_loop::EventLoopBuilder::with_user_event();
        builder.with_app_id("com.oxifoc.studio.webview");
        config = config.with_event_loop(builder.build());
    }
    let icon = dioxus::desktop::tao::window::Icon::from_rgba(
        include_bytes!("../../../../oxifoc-ui/assets/oxifoc-icon-64.rgba").to_vec(),
        64,
        64,
    )
    .expect("embedded icon dimensions");
    dioxus::LaunchBuilder::desktop()
        .with_cfg(
            config
                .with_menu(None)
                .with_window(window.with_title("OxiFOC · Drive studio").with_inner_size(
                    dioxus::desktop::LogicalSize::new(
                        if compact { 420. } else { 1280. },
                        if compact { 900. } else { 920. },
                    ),
                ))
                .with_icon(icon)
                .with_on_window(|window, _| {
                    #[cfg(target_os = "linux")]
                    {
                        use dioxus::desktop::tao::platform::unix::WindowExtUnix;
                        window
                            .gtk_window()
                            .set_icon_name(Some("com.oxifoc.studio.webview"));
                    }
                }),
        )
        .launch(oxifoc_host_dioxus_ui::HostApp);
}
