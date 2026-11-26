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

/// Nodes one Timer can drive, each on its own serial link.
pub const MAX_NODES: usize = 4;

/// The first Node's default pins, and what every Node shares.
pub mod node {
    /// RP2350 TX -> XIAO D7 (ESP32-C5 GPIO12, UART0 RX).
    pub const TX: u8 = 4;
    /// RP2350 RX <- XIAO D6 (ESP32-C5 GPIO11, UART0 TX).
    pub const RX: u8 = 5;
    pub const BAUD: u32 = 921_600;
    /// SYNC pulse output, one wire to every Node's SYNC input.
    pub const SYNC: u8 = 6;
    /// C5 EN (reset), active low. One wire to every Node.
    pub const EN: u8 = 7;
    /// C5 BOOT strap, low = download mode. One wire to every Node.
    pub const BOOT: u8 = 8;
}

/// One Node's serial link.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NodePins {
    /// To the Node's RX.
    pub tx: u8,
    /// From the Node's TX.
    pub rx: u8,
}

/// Which header GPIOs carry the Node wiring. Read from the saved settings at start-up, so a
/// board wired differently needs a setting, not its own firmware.
///
/// How many Nodes the Timer drives is how many links are listed. Each Node has its own
/// link; SYNC, EN and BOOT are each one wire shared by every Node.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Pins {
    /// Node 1 first; filled from the front.
    pub nodes: [Option<NodePins>; MAX_NODES],
    pub sync: u8,
    pub en: u8,
    pub boot: u8,
}

impl Default for Pins {
    fn default() -> Self {
        Self { nodes: [Some(NodePins { tx: node::TX, rx: node::RX }), None, None, None], sync: node::SYNC, en: node::EN, boot: node::BOOT }
    }
}

/// How each Node's link is driven: Node 1 on UART1, Node 2 on UART0, Nodes 3 and 4 on PIO
/// state machines (a UART in software, on any pin).
pub const HW_UARTS: usize = 2;
/// Header GPIOs each hardware UART can transmit on (the RP2350's function table; `main.rs`
/// spells out the same lists as pin types).
const UART_TX_PINS: [&[u8]; HW_UARTS] = [&[4, 6, 8, 10, 22, 26], &[0, 2, 12, 14, 28]];
/// Header GPIOs each hardware UART can receive on. UART1 could use GP9 too, but it is the
/// recovery jumper.
const UART_RX_PINS: [&[u8]; HW_UARTS] = [&[5, 7, 11, 27], &[1, 3, 13, 15]];
/// Header GPIOs free to be a plain output (SYNC, EN, BOOT) or a PIO link pin: every header
/// GPIO but GP9.
pub const OUT_PINS: [u8; 19] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 22, 26, 27, 28];

/// The pins Node `slot`'s link can transmit on.
pub fn tx_pins(slot: usize) -> &'static [u8] {
    UART_TX_PINS.get(slot).copied().unwrap_or(&OUT_PINS)
}

/// The pins Node `slot`'s link can receive on.
pub fn rx_pins(slot: usize) -> &'static [u8] {
    UART_RX_PINS.get(slot).copied().unwrap_or(&OUT_PINS)
}

impl Pins {
    /// How many Nodes are wired.
    pub fn node_count(&self) -> usize {
        self.nodes.iter().take_while(|n| n.is_some()).count()
    }

    /// Why these pins cannot be used, if they cannot.
    pub fn check(&self) -> Result<(), &'static str> {
        let n = self.node_count();
        if n == 0 {
            return Err("at least one Node link is needed");
        }
        if self.nodes[n..].iter().any(|p| p.is_some()) {
            return Err("Node links must be listed without gaps");
        }
        let mut all: heapless::Vec<u8, { 2 * MAX_NODES + 3 }> = heapless::Vec::new();
        for (slot, p) in self.nodes.iter().flatten().enumerate() {
            if !tx_pins(slot).contains(&p.tx) {
                return Err(match slot {
                    0 => "Node 1 tx must be one of GP4, GP6, GP8, GP10, GP22, GP26 (the pins UART1 can transmit on)",
                    1 => "Node 2 tx must be one of GP0, GP2, GP12, GP14, GP28 (the pins UART0 can transmit on)",
                    _ => "tx must be a header GPIO: 0-8, 10-15, 22, 26-28",
                });
            }
            if !rx_pins(slot).contains(&p.rx) {
                return Err(match slot {
                    0 => "Node 1 rx must be one of GP5, GP7, GP11, GP27 (the pins UART1 can receive on)",
                    1 => "Node 2 rx must be one of GP1, GP3, GP13, GP15 (the pins UART0 can receive on)",
                    _ => "rx must be a header GPIO: 0-8, 10-15, 22, 26-28",
                });
            }
            let _ = all.push(p.tx);
            let _ = all.push(p.rx);
        }
        for p in [self.sync, self.en, self.boot] {
            if !OUT_PINS.contains(&p) {
                return Err("sync, en and boot must be header GPIOs: 0-8, 10-15, 22, 26-28");
            }
            let _ = all.push(p);
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
