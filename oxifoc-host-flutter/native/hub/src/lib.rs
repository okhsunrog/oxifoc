//! This `hub` crate is the entry point of the Rust logic for oxifoc-host-flutter.

mod actors;
mod signals;

use actors::create_actors;
use rinf::{dart_shutdown, write_interface};
use tokio::spawn;

write_interface!();

#[tokio::main(flavor = "multi_thread")]
async fn main() {
    // Initialize tracing for debug output
    oxifoc_host_lib::init_tracing();

    // Spawn the host actor
    spawn(create_actors());

    // Keep the main function running until Dart shutdown
    dart_shutdown().await;
}
