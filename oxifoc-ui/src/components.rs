use dioxus::prelude::*;

// Adapted from rust-ui/ui button.rs. Source revision and MIT license:
// ../UPSTREAM.md and ../licenses/RUST-UI-MIT.
#[derive(Clone, Copy, Default, PartialEq)]
pub enum ButtonVariant {
    #[default]
    Primary,
    Secondary,
    Ghost,
    Danger,
}
impl ButtonVariant {
    fn class(self) -> &'static str {
        match self {
            Self::Primary => "button-primary",
            Self::Secondary => "button-secondary",
            Self::Ghost => "button-ghost",
            Self::Danger => "button-danger",
        }
    }
}

#[component]
pub fn Button(
    #[props(default)] variant: ButtonVariant,
    #[props(default)] disabled: bool,
    #[props(default)] busy: bool,
    #[props(default)] class: String,
    #[props(default)] onclick: EventHandler<()>,
    children: Element,
) -> Element {
    rsx! { button {
        r#type: "button", class: "ox-button {variant.class()} {class}",
        disabled: disabled || busy, "aria-busy": busy,
        onclick: move |_| { if !disabled && !busy { onclick.call(()); } },
        onkeydown: move |event| { if keyboard_activate(&event) && !disabled && !busy { onclick.call(()); } },
        if busy { span { class: "spinner", "aria-hidden": "true" } }
        {children}
    } }
}

/// Blitz currently omits the default keyboard activation of HTML buttons.
/// Prevent the browser default too, so WebView dispatches the action once.
pub fn keyboard_activate(event: &KeyboardEvent) -> bool {
    if event.key() == Key::Named(NamedKey::Enter) || event.key() == Key::Character(" ".into()) {
        event.prevent_default();
        event.stop_propagation();
        !event.is_auto_repeating()
    } else {
        false
    }
}

// Semantic separator adapted from Dioxus Components. See UPSTREAM.md.
#[component]
pub fn Separator(#[props(default)] decorative: bool) -> Element {
    rsx! { div { class: "separator", role: if decorative { "none" } else { "separator" },
    aria_orientation: if !decorative { "horizontal" }, "data-orientation": "horizontal" } }
}

#[derive(Clone, Copy, Default, PartialEq)]
pub enum Tone {
    #[default]
    Neutral,
    Success,
    Warning,
    Danger,
}
impl Tone {
    pub fn class(self) -> &'static str {
        match self {
            Self::Neutral => "neutral",
            Self::Success => "success",
            Self::Warning => "warning",
            Self::Danger => "danger",
        }
    }
}
#[component]
pub fn Status(#[props(default)] tone: Tone, children: Element) -> Element {
    rsx! { span { class: "status {tone.class()}", span { class: "status-dot", "aria-hidden": "true" } {children} } }
}

#[component]
pub fn Field(
    id: String,
    label: String,
    value: String,
    #[props(default)] hint: String,
    #[props(default)] error: String,
    #[props(default)] placeholder: String,
    #[props(default)] numeric: bool,
    #[props(default)] disabled: bool,
    oninput: EventHandler<String>,
) -> Element {
    let description = format!("{id}-description");
    rsx! { div { class: "field",
        label { r#for: "{id}", "{label}" }
        input { id, r#type: "text", inputmode: if numeric { "numeric" } else { "text" },
            class: if error.is_empty() { "ox-input" } else { "ox-input invalid" },
            value, placeholder, disabled, spellcheck: "false", autocomplete: "off",
            "aria-invalid": !error.is_empty(), "aria-describedby": "{description}",
            oninput: move |e| oninput.call(e.value()),
        }
        if !error.is_empty() { p { id: "{description}", class: "field-error", "{error}" } }
        else if !hint.is_empty() { p { id: "{description}", class: "hint", "{hint}" } }
    } }
}

#[derive(Clone, Copy, PartialEq)]
pub enum IconKind {
    Bluetooth,
    Network,
    Usb,
    Arrow,
    Check,
    Close,
    Sun,
    Moon,
    Settings,
}
#[component]
pub fn Icon(kind: IconKind) -> Element {
    let path = match kind {
        IconKind::Bluetooth => "M7 7l10 10-5 4V3l5 4L7 17",
        IconKind::Network => {
            "M4 9a12 12 0 0 1 16 0M7 12a8 8 0 0 1 10 0M10 15a3 3 0 0 1 4 0M12 19h.01"
        }
        IconKind::Usb => "M12 21V3m-3 3 3-3 3 3M12 16l-6-4V9m6 4 6-4V6M5 7h2v2H5zM17 4h2v2h-2z",
        IconKind::Arrow => "M5 12h14m-5-5 5 5-5 5",
        IconKind::Check => "M5 12l4 4L19 6",
        IconKind::Close => "M6 6l12 12M6 18 18 6",
        IconKind::Sun => {
            "M12 3v2m0 14v2M3 12h2m14 0h2M5 5l2 2m10 10 2 2M5 19l2-2M17 7l2-2M16 12a4 4 0 1 1-8 0 4 4 0 0 1 8 0"
        }
        IconKind::Moon => "M20 15A9 9 0 0 1 9 4a8 8 0 1 0 11 11",
        IconKind::Settings => "M4 6h16M4 12h16M4 18h16M8 3v6m8 0v6M10 15v6",
    };
    // Literal SVG stroke keeps icons usable when Native cannot style SVG paths.
    rsx! { svg { class: "icon", width: "22", height: "22", view_box: "0 0 24 24", fill: "none",
    "aria-hidden": "true", path { d: path, stroke: "#8299b5", stroke_width: "1.7", stroke_linecap: "round", stroke_linejoin: "round" } } }
}

#[component]
pub fn ThemeButton(mut light: Signal<bool>) -> Element {
    rsx! { button { class: "theme-button", r#type: "button",
        "aria-label": if light() { "Use dark theme" } else { "Use light theme" },
        onclick: move |_| { let next = !light(); light.set(next); },
        onkeydown: move |e| { if keyboard_activate(&e) { let next = !light(); light.set(next); } },
        Icon { kind: if light() { IconKind::Moon } else { IconKind::Sun } }
    } }
}

#[component]
pub fn Brand() -> Element {
    rsx! { div { class: "brand", div { class: "brand-symbol", "O" } span { "OxiFOC" } } }
}

/// Inline disclosure has identical keyboard semantics in both renderers.
#[component]
pub fn Disclosure(title: String, children: Element) -> Element {
    let mut open = use_signal(|| false);
    rsx! { div { class: "disclosure",
        button { r#type: "button", class: "disclosure-toggle", "aria-expanded": open(),
            onclick: move |_| { let next = !open(); open.set(next); },
            onkeydown: move |e| { if keyboard_activate(&e) { let next = !open(); open.set(next); } },
            "{title}" span { if open() { "−" } else { "+" } }
        }
        if open() { div { class: "disclosure-body", {children} } }
    } }
}
