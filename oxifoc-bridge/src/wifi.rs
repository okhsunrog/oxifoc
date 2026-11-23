use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_net::{Runner, Stack, StackResources};
use embassy_time::Timer;
use esp_hal::{peripherals::WIFI, rng::Rng};
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, ControllerConfig, Interface, WifiController, WifiError,
    sta::StationConfig,
};
use static_cell::StaticCell;

pub fn start(spawner: Spawner, wifi: WIFI<'static>) -> Option<Stack<'static>> {
    match configure(wifi) {
        Ok(Some(controller)) => {
            static RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
            let rng = Rng::new();
            let seed = (u64::from(rng.random()) << 32) | u64::from(rng.random());
            let (stack, runner) = embassy_net::new(
                Interface::station(),
                embassy_net::Config::dhcpv4(Default::default()),
                RESOURCES.init(StackResources::new()),
                seed,
            );
            let task = run(controller, runner).expect("Wi-Fi task is spawned once");
            task.metadata().set_priority(1);
            spawner.spawn(task);
            Some(stack)
        }
        Ok(None) => None,
        Err(err) => {
            log::warn!("Wi-Fi setup failed: {err:?}");
            None
        }
    }
}

fn configure(wifi: WIFI<'static>) -> Result<Option<WifiController<'static>>, WifiError> {
    let (Some(ssid), Some(password)) = (
        option_env!("OXIFOC_PROBE_WIFI_SSID").filter(|s| !s.is_empty()),
        option_env!("OXIFOC_PROBE_WIFI_PASSWORD").filter(|s| !s.is_empty()),
    ) else {
        return Ok(None);
    };

    // With a 12 KiB region the radio drivers left about 15 KiB of the heap
    // free. A 20 kHz stream takes 13 to 16 KiB for frames in flight and 19 to
    // 22 KiB with a BLE central connected: the heap ran out, and esp-radio
    // panics when the buffer for a BLE packet cannot be allocated. This
    // region leaves 10 KiB free at that load. It is static, so the 64 KiB
    // region stays off the main stack's RAM; the USB queue gave up the
    // difference.
    esp_alloc::heap_allocator!(size: 28 * 1024);
    let station = StationConfig::default()
        .with_ssid(ssid.try_into()?)
        .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
            password.try_into()?,
        ));
    // The default Wi-Fi buffers exhaust the shared radio heap before BLE
    // can start. Keep frame queues small enough for both radios.
    let config = ControllerConfig::default()
        .with_static_rx_buf_num(4)
        .with_dynamic_rx_buf_num(8)
        .with_dynamic_tx_buf_num(10)
        .with_tx_queue_size(10)
        .with_ampdu_rx_enable(false)
        .with_ampdu_tx_enable(true)
        .with_initial_config(Config::Station(station));
    let controller = WifiController::new(wifi, config)?;
    Ok(Some(controller))
}

#[embassy_executor::task]
async fn run(mut controller: WifiController<'static>, mut runner: Runner<'static, Interface>) {
    join(runner.run(), async {
        loop {
            match controller.connect_async().await {
                Ok(_) => {
                    log::info!("Wi-Fi associated");
                    let _ = controller.wait_for_disconnect_async().await;
                }
                Err(err) => log::warn!("Wi-Fi association failed: {err:?}"),
            }
            Timer::after_secs(3).await;
        }
    })
    .await;
}
