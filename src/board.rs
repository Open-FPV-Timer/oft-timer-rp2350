//! Pin map for the Waveshare RP2350-POE-ETH (B1 Timer) and the XIAO ESP32-C5 Node.
//!
//! W6300 pins come from Waveshare's demo (`lib/Config/DEV_Config.h`). The Node, SYNC and
//! EN/BOOT pins are our choice of free header GPIOs; check them against the silkscreen.
//! Everything hardware-specific is named here and nowhere else.

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

/// The C5 Node on UART1.
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

/// Locally administered MAC: 02:4F:46:54 ("OFT") + two bytes from the chip id.
pub const MAC_PREFIX: [u8; 4] = [0x02, 0x4F, 0x46, 0x54];

/// TCP port the OFT Protocol is served on.
pub const OFT_PORT: u16 = 7270;
