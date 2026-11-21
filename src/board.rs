//! Pin map for the Waveshare RP2350-POE-ETH (B1 Timer) and the XIAO ESP32-C5 Node.
//!
//! W6300 pins come from Waveshare's demo (`lib/Config/DEV_Config.h`). The Node, SYNC and
//! EN/BOOT pins are a saved setting (`Pins`): the numbers in `node` are its defaults, our
//! choice of free header GPIOs. Everything hardware-specific is named here and nowhere else.

#![allow(dead_code)] // the pin constants document the wiring; embassy names pins by type (PIN_18)

/// W6300 Ethernet controller on SPI0, single-SPI mode.
pub mod eth {
    pub const SCK: u8 = 18;
    pub const MOSI: u8 = 19;
    pub const MISO: u8 = 16;
    pub const CS: u8 = 17;
    pub const RST: u8 = 20;
    pub const INT: u8 = 21;
    /// Waveshare's demo clocks it at 70 MHz; start lower until the wiring is proven.
    pub const SPI_HZ: u32 = 30_000_000;
}

/// The C5 Node on UART1: the default pins.
pub mod node {
    /// RP2350 TX -> XIAO D7 (ESP32-C5 GPIO12, UART0 RX).
    pub const TX: u8 = 4;
    /// RP2350 RX <- XIAO D6 (ESP32-C5 GPIO11, UART0 TX).
    pub const RX: u8 = 5;
    pub const BAUD: u32 = 921_600;
    /// SYNC pulse output, wired to a C5 input the Node firmware latches.
    pub const SYNC: u8 = 6;
    /// C5 EN (reset), active low.
    pub const EN: u8 = 7;
    /// C5 BOOT strap (GPIO28 on the C5 per Espressif; confirm on the XIAO), low = download mode.
    pub const BOOT: u8 = 8;
}

/// Which header GPIOs carry the Node's five signals. Read from the saved settings at
/// start-up, so a board wired differently needs a setting, not its own firmware.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Pins {
    /// UART1 TX, to the Node's RX.
    pub tx: u8,
    /// UART1 RX, from the Node's TX.
    pub rx: u8,
    pub sync: u8,
    pub en: u8,
    pub boot: u8,
}

impl Default for Pins {
    fn default() -> Self {
        Self { tx: node::TX, rx: node::RX, sync: node::SYNC, en: node::EN, boot: node::BOOT }
    }
}

/// Header GPIOs UART1 can transmit on (the RP2350's function table; `main.rs` spells out
/// the same list as pin types).
pub const TX_PINS: [u8; 6] = [4, 6, 8, 10, 22, 26];
/// Header GPIOs UART1 can receive on. GP9 could too, but it is the recovery jumper.
pub const RX_PINS: [u8; 4] = [5, 7, 11, 27];
/// Header GPIOs free to be a plain output (SYNC, EN, BOOT): every header GPIO but GP9.
pub const OUT_PINS: [u8; 19] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 22, 26, 27, 28];

impl Pins {
    /// Why these pins cannot be used, if they cannot.
    pub fn check(&self) -> Result<(), &'static str> {
        if !TX_PINS.contains(&self.tx) {
            return Err("tx must be one of GP4, GP6, GP8, GP10, GP22, GP26 (the pins UART1 can transmit on)");
        }
        if !RX_PINS.contains(&self.rx) {
            return Err("rx must be one of GP5, GP7, GP11, GP27 (the pins UART1 can receive on)");
        }
        let all = [self.tx, self.rx, self.sync, self.en, self.boot];
        if !all[2..].iter().all(|p| OUT_PINS.contains(p)) {
            return Err("sync, en and boot must be header GPIOs: 0-8, 10-15, 22, 26-28");
        }
        if (1..all.len()).any(|k| all[..k].contains(&all[k])) {
            return Err("each signal needs its own pin");
        }
        Ok(())
    }
}

/// Locally administered MAC: 02:4F:46:54 ("OFT") + two bytes from the chip id.
pub const MAC_PREFIX: [u8; 4] = [0x02, 0x4F, 0x46, 0x54];

/// TCP port the OFT Protocol is served on.
pub const OFT_PORT: u16 = 7270;
