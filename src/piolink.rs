//! Node links 3 and 4: UARTs made from PIO state machines, since the RP2350 has only two
//! hardware UARTs. **Not yet run on hardware.**
//!
//! Sending uses embassy-rp's PIO UART as it is. Receiving does not: its reader takes one
//! byte per wake-up from an 8-byte FIFO, and at 921600 baud that FIFO fills in under 90 µs,
//! far sooner than a busy executor comes back. Here a DMA channel empties the FIFO into a
//! ring in memory without any code running, forever, and the link task reads the ring every
//! couple of milliseconds.
//!
//! PIO0 state machines: Node 3 sends on 0 and receives on 1; Node 4 sends on 2 and receives
//! on 3. DMA channels 2 and 3 serve the two receivers (0 and 1 belong to the Ethernet SPI).

use crate::board::{NodePins, HW_UARTS};
use crate::node::{LinkRx, LinkTx};
use embassy_rp::pac;
use embassy_rp::peripherals::{DMA_CH2, DMA_CH3, PIO0};
use embassy_rp::pio::Pio;
use embassy_rp::pio_programs::uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram};
use embassy_rp::Peri;
use embassy_time::Timer as Delay;

/// Ring size. At 921600 baud the wire can carry 1 KiB in 11 ms; a Node sends a few KiB/s.
const RING: usize = 1024;
/// log2(RING), for the DMA channel's address wrap.
const RING_BITS: u8 = 10;

/// The DMA channel wraps its write address on a `RING`-byte boundary, so the buffer must
/// start on one.
#[repr(C, align(1024))]
pub struct RingBuf(core::cell::UnsafeCell<[u8; RING]>);

// Written only by the DMA channel, read only by the one `RingRx` that owns it.
unsafe impl Sync for RingBuf {}

static RINGS: [RingBuf; 2] = [const { RingBuf(core::cell::UnsafeCell::new([0; RING])) }; 2];

/// Bytes a PIO UART receiver has put in its ring.
pub struct RingRx {
    /// DMA channel number.
    ch: usize,
    base: *const u8,
    /// Next byte to hand out.
    next: usize,
}

// The pointer is to a static.
unsafe impl Send for RingRx {}

impl RingRx {
    /// Start DMA channel `ch` copying every byte PIO0 state machine `sm` receives into `ring`.
    fn start(ch: usize, sm: usize, ring: &'static RingBuf) -> Self {
        let base = ring.0.get() as *const u8;
        let d = pac::DMA.ch(ch);
        // The received byte is the low byte of the FIFO word (the program shifts it down).
        d.read_addr().write_value(pac::PIO0.rxf(sm).as_ptr() as u32);
        d.write_addr().write_value(base as u32);
        d.trans_count().write(|w| {
            w.set_mode(pac::dma::vals::TransCountMode::ENDLESS);
            w.set_count(1);
        });
        d.ctrl_trig().write(|w| {
            // Paced by this state machine's receive FIFO.
            w.set_treq_sel(pac::dma::vals::TreqSel::from(4 + sm as u8));
            w.set_data_size(pac::dma::vals::DataSize::SIZE_BYTE);
            w.set_incr_read(false);
            w.set_incr_write(true);
            w.set_ring_sel(true);
            w.set_ring_size(RING_BITS);
            // Chaining to itself means no chaining.
            w.set_chain_to(ch as u8);
            w.set_en(true);
        });
        Self { ch, base, next: 0 }
    }

    /// Wait until there is something in the ring and copy it out.
    pub async fn read(&mut self, out: &mut [u8]) -> usize {
        loop {
            let head = (pac::DMA.ch(self.ch).write_addr().read() as usize).wrapping_sub(self.base as usize) & (RING - 1);
            if head != self.next {
                let mut n = 0;
                while self.next != head && n < out.len() {
                    // Safety: inside the static ring; the DMA channel has moved past this byte.
                    out[n] = unsafe { core::ptr::read_volatile(self.base.add(self.next)) };
                    self.next = (self.next + 1) & (RING - 1);
                    n += 1;
                }
                return n;
            }
            Delay::after_millis(2).await;
        }
    }
}

/// A PIO pin by GPIO number, for the header GPIOs `Pins::check` lets through (each used
/// once, by nothing else in this firmware).
macro_rules! with_pin {
    ($n:expr, |$pin:ident| $body:expr) => {{
        use embassy_rp::peripherals::*;
        // Safety: as for `main::output`.
        unsafe {
            match $n {
                0 => { let $pin = PIN_0::steal(); $body }
                1 => { let $pin = PIN_1::steal(); $body }
                2 => { let $pin = PIN_2::steal(); $body }
                3 => { let $pin = PIN_3::steal(); $body }
                4 => { let $pin = PIN_4::steal(); $body }
                5 => { let $pin = PIN_5::steal(); $body }
                6 => { let $pin = PIN_6::steal(); $body }
                7 => { let $pin = PIN_7::steal(); $body }
                8 => { let $pin = PIN_8::steal(); $body }
                10 => { let $pin = PIN_10::steal(); $body }
                11 => { let $pin = PIN_11::steal(); $body }
                12 => { let $pin = PIN_12::steal(); $body }
                13 => { let $pin = PIN_13::steal(); $body }
                14 => { let $pin = PIN_14::steal(); $body }
                15 => { let $pin = PIN_15::steal(); $body }
                22 => { let $pin = PIN_22::steal(); $body }
                26 => { let $pin = PIN_26::steal(); $body }
                27 => { let $pin = PIN_27::steal(); $body }
                _ => { let $pin = PIN_28::steal(); $body }
            }
        }
    }};
}

/// Bring up the PIO links for Nodes 3 and 4 (`pins[0]` and `pins[1]`), those that are wired.
pub fn start(pio: Peri<'static, PIO0>, _dma: (Peri<'static, DMA_CH2>, Peri<'static, DMA_CH3>), pins: [Option<NodePins>; 2], baud: u32) -> [Option<(LinkTx, LinkRx)>; 2] {
    if pins[0].is_none() && pins[1].is_none() {
        return [None, None];
    }
    let Pio { mut common, sm0, sm1, sm2, sm3, .. } = Pio::new(pio, crate::Irqs);
    let tx_prg = PioUartTxProgram::new(&mut common);
    let rx_prg = PioUartRxProgram::new(&mut common);
    let a = pins[0].map(|p| {
        let tx = with_pin!(p.tx, |pin| PioUartTx::new(baud, &mut common, sm0, pin, &tx_prg));
        // Set up and left running; the DMA channel does the reading.
        core::mem::forget(with_pin!(p.rx, |pin| PioUartRx::new(baud, &mut common, sm1, pin, &rx_prg)));
        (LinkTx::Pio0(tx), LinkRx::Ring(RingRx::start(2, 1, &RINGS[0])))
    });
    let b = pins[1].map(|p| {
        let tx = with_pin!(p.tx, |pin| PioUartTx::new(baud, &mut common, sm2, pin, &tx_prg));
        core::mem::forget(with_pin!(p.rx, |pin| PioUartRx::new(baud, &mut common, sm3, pin, &rx_prg)));
        (LinkTx::Pio2(tx), LinkRx::Ring(RingRx::start(3, 3, &RINGS[1])))
    });
    // The PIO block stays configured for good.
    core::mem::forget((common, tx_prg, rx_prg));
    [a, b]
}

/// Change the baud rate of Node `n`'s PIO link (`n` is 2 or 3): both its state machines run
/// eight clocks to the bit.
pub fn set_baud(n: usize, baud: u32) {
    let first = (n - HW_UARTS) * 2;
    // The divider is 16.8 fixed point in the top 24 bits of the register.
    let div = (embassy_rp::clocks::clk_sys_freq() as u64 * 256 / (8 * baud as u64)) as u32;
    for sm in [first, first + 1] {
        pac::PIO0.sm(sm).clkdiv().write(|w| w.0 = div << 8);
    }
}
