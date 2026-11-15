//! Shared visual components for OxiFOC WebView and Native hosts.
pub mod catalogue;
pub mod components;
pub mod connection;
pub use catalogue::Catalogue;
pub use components::*;
pub use connection::*;

pub const CSS: &str = include_str!("../assets/oxifoc.css");

/// Dark-theme plot series, matching --series-a through --series-e in the CSS.
pub const PLOT_COLORS: [[f32; 4]; 5] = [
    rgba(0x6f, 0xf5, 0xe6),
    rgba(0x7d, 0xc3, 0xff),
    rgba(0xff, 0xd4, 0x9a),
    rgba(0xc2, 0xa5, 0xef),
    rgba(0xf1, 0x9b, 0xc0),
];

const fn rgba(r: u8, g: u8, b: u8) -> [f32; 4] {
    [r as f32 / 255., g as f32 / 255., b as f32 / 255., 1.]
}

pub mod instruments;
pub use instruments::*;
