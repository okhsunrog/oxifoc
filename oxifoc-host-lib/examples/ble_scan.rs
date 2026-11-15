//! Read-only discovery; initializing it must not require a GUI runtime.
fn main() -> anyhow::Result<()> {
    oxifoc_host_lib::init_tracing();
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()?;
    let devices = runtime.block_on(oxifoc_host_lib::try_scan_ble_devices(
        std::time::Duration::from_secs(4),
    ))?;
    println!("NUS devices: {}", devices.len());
    for device in devices {
        println!("{device}");
    }
    Ok(())
}
