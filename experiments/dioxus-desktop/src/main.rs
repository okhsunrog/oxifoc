mod source;
use dioxus::prelude::*;
fn main() {
    // Headless Wayland sessions may leave GTK's DPI at the -1 sentinel.
    // WebKitGTK 2.54.2 exposes a negative devicePixelRatio in that case.
    use gtk::prelude::*;
    gtk::init().expect("GTK initialization");
    if let Some(settings) = gtk::Settings::default() {
        if settings.gtk_xft_dpi() <= 0 {
            settings.set_gtk_xft_dpi(96 * 1024);
        }
    }

    if let Some(screen) = gtk::gdk::Screen::default() {
        eprintln!("[desktop-dpi] resolution={}", screen.resolution());
        if screen.resolution() <= 0. {
            screen.set_resolution(96.);
        }
    }
    let bench = std::env::var_os("PLOT_BENCH").is_some();
    dioxus::LaunchBuilder::desktop()
        .with_cfg(
            dioxus::desktop::Config::new().with_menu(None).with_window(
                dioxus::desktop::WindowBuilder::new()
                    .with_title("OxiFOC • Desktop WebGL")
                    .with_inner_size(dioxus::desktop::LogicalSize::new(
                        if bench { 802. } else { 870. },
                        if bench { 400. } else { 650. },
                    )),
            ),
        )
        .launch(App);
}
#[component]
fn App() -> Element {
    rsx! {
        style { {include_str!("style.css")} }
        main { onmounted: move |_| { spawn(async move {
            let bench = std::env::var_os("PLOT_BENCH").is_some();
            let mut source=source::Source::new(bench);
            let script=format!("try {{ {}\n{} }} catch(e) {{ dioxus.send({{error:String(e),stack:e.stack}}); }}",include_str!("plot.js"),include_str!("webgl.js"));
            let mut eval = document::eval(&script);
            let _=eval.send(serde_json::json!({"bench":bench,"visible":std::env::var("PLOT_VISIBLE").ok().and_then(|s|s.parse::<u32>().ok()).unwrap_or(1200),"paused":std::env::var_os("PLOT_PAUSED").is_some()}));
            while let Ok(value) = eval.recv::<serde_json::Value>().await {
                if value["request"]==true { let _=eval.send(source.next()); }
                else if let Some(command)=value["command"].as_str(){source.command(command);}
                else {println!("[desktop-probe] {value}");}
            }
        }); },
        header { div { class: "brand", "O / OxiFOC" } span { "DESKTOP LAB · WEBGL 2" } }
        h1 { "Current scope" }
        p { id: "telemetry", "Starting host runtime…" }
        div { id: "controls" }
        canvas { id: "plot", width: "802", height: "400" }
        footer { span { "● Ia　" } span { "● Ib　" } span { "● Ic" } span { id: "metrics" } }
        details { summary { "GPU capabilities" } pre { id: "status", "Checking…" } }
        }
    }
}
