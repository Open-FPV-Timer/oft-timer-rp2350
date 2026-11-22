//! Saved settings: the power-on config and the network settings, as JSON in two flash
//! sectors after the firmware slots (`oft_fwimage::settings` has the record format).
//!
//! Read once at start-up, straight from memory-mapped flash. Written only on request (a
//! save from the page or a client); a write stops everything for the tens of milliseconds
//! flash takes to erase a sector, so samples are lost while it runs.

use crate::fw::Fw;
use crate::state::{Saved, Timer};
use crate::web::SavedJson;
use oft_fwimage::settings::{choose, decode, COPIES};

/// Room for the JSON: a config with eight seats, each with its own threshold, plus the
/// network settings, is about 1.1 KiB.
const JSON_MAX: usize = 1536;

/// What flash holds, or the defaults if it holds nothing usable.
pub fn load() -> Saved {
    let seqs: [Option<u32>; COPIES] = core::array::from_fn(|c| decode(crate::fw::settings_sector(c)).map(|r| r.0));
    let Some(newest) = choose(seqs).0 else { return Saved::default() };
    let Some((_, payload)) = decode(crate::fw::settings_sector(newest)) else { return Saved::default() };
    match serde_json_core::from_slice::<SavedJson>(payload) {
        Ok((json, _)) => json.into_saved(),
        Err(_) => Saved::default(),
    }
}

/// Write `saved` to flash and, once it is safely there, record it as the saved state.
pub async fn save(timer: &Timer, fw: &Fw, saved: Saved) -> Result<(), &'static str> {
    let mut buf = [0u8; JSON_MAX];
    let n = serde_json_core::to_slice(&SavedJson::from(&saved), &mut buf).map_err(|_| "settings too large to save")?;
    fw.store.lock().await.write_settings(&buf[..n])?;
    timer.with(|i| {
        i.saved = saved;
        i.config_rev = i.config_rev.wrapping_add(1);
    });
    Ok(())
}
