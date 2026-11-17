//! ChaCha12 CSPRNG registered as the `embassy-crypto` RNG driver for BLE
//! pairing keys, seeded from the hardware RNG after the radio is on.

use core::cell::RefCell;

use embassy_sync::blocking_mutex::{Mutex, raw::CriticalSectionRawMutex};
use rand_chacha::ChaCha12Rng;
use rand_chacha::rand_core::{RngCore as _, SeedableRng as _};

static RNG: Mutex<CriticalSectionRawMutex, RefCell<Option<ChaCha12Rng>>> =
    Mutex::new(RefCell::new(None));

/// Seed from the hardware RNG. Call after the radio is enabled (it supplies the
/// entropy) and before the BLE stack can pair.
pub fn seed() {
    let mut seed = [0u8; 32];
    esp_hal::rng::Rng::new().read(&mut seed);
    RNG.lock(|rng| rng.replace(Some(ChaCha12Rng::from_seed(seed))));
}

struct Driver;

impl embassy_crypto::driver::Rng for Driver {
    fn fill_bytes(buf: &mut [u8]) {
        RNG.lock(|rng| {
            rng.borrow_mut()
                .as_mut()
                .expect("CSPRNG used before it was seeded")
                .fill_bytes(buf);
        });
    }
}

embassy_crypto::rng_impl!(Driver);
