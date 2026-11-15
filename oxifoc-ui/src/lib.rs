//! Shared visual components for OxiFOC WebView and Native hosts.
pub mod catalogue;
pub mod components;
pub mod connection;
pub use catalogue::Catalogue;
pub use components::*;
pub use connection::*;

pub const CSS: &str = include_str!("../assets/oxifoc.css");
