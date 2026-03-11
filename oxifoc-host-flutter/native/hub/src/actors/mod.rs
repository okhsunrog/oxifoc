//! Actor module for oxifoc-host-flutter

mod host;

use host::HostActor;
use messages::prelude::Context;
use tokio::spawn;

/// Creates and spawns the actors in the async system.
pub async fn create_actors() {
    let host_context = Context::new();
    let host_addr = host_context.address();

    let host_actor = HostActor::new(host_addr);
    spawn(host_context.run(host_actor));
}
