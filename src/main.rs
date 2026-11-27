//! OFT Timer firmware for the Waveshare RP2350-POE-ETH (build B1).
//!
//! Tasks: W6300 Ethernet runner, network stack, rx/tx per Node link, SYNC pulses, a pool of
//! HTTP/WebSocket servers, and a status tick. All timing logic lives in `oft-core`.

// The router's type nests once per route.
#![recursion_limit = "256"]
#![no_std]
#![no_main]

mod board;
mod fw;
mod mdns;
mod netdev;
mod node;
mod piolink;
mod settings;
mod state;
#[cfg(feature = "usb-console")]
mod usb;
mod web;

use embassy_executor::Spawner;
use embassy_net::{Stack, StackResources};
use embassy_net_wiznet::chip::W6300;
use embassy_net_wiznet::{Device, Runner, State as WiznetState};
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{AnyPin, Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, PIO0, SPI0, UART0, UART1};
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig, TxPin};
use node::{LinkRx, LinkTx};
use embassy_rp::Peri;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
mod crash;
use state::Timer;
use static_cell::StaticCell;

bind_interrupts!(pub(crate) struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;
    UART1_IRQ => BufferedInterruptHandler<UART1>;
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    ADC_IRQ_FIFO => embassy_rp::adc::InterruptHandler;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>, embassy_rp::dma::InterruptHandler<DMA_CH1>;
});

/// picotool / boot ROM metadata.
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: embassy_rp::block::ImageDef = embassy_rp::block::ImageDef::secure_exe();

#[link_section = ".bi_entries"]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"oft-timer-rp2350"),
    embassy_rp::binary_info::rp_program_description!(c"Open FPV Timer: OFT Protocol over Ethernet, up to four C5 Nodes"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

type SpiDev = ExclusiveDevice<Spi<'static, SPI0, Async>, Output<'static>, Delay>;

#[embassy_executor::task]
async fn ethernet_task(runner: Runner<'static, W6300, SpiDev, Input<'static>, Output<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, netdev::LinkGated<Device<'static>>>) -> ! {
    runner.run().await
}

/// A pseudo-random seed from the ring oscillator; good enough for TCP sequence numbers.
fn seed() -> u64 {
    let mut rng = embassy_rp::clocks::RoscRng;
    rng.next_u64()
}

/// A plain output on header GPIO `n`.
///
/// The pin is taken by number, not from `Peripherals`: `Pins::check` only lets through
/// header GPIOs that nothing else in this firmware touches, each used once.
fn output(n: u8, level: Level) -> Output<'static> {
    Output::new(unsafe { AnyPin::steal(n) }, level)
}

/// Node 1's link: UART1 on the saved pins. embassy names UART pins by type, so every
/// allowed pin is spelled out here: the same lists as in `board::tx_pins` and `rx_pins`.
fn node1_uart(uart: Peri<'static, UART1>, pins: board::NodePins, tx_buf: &'static mut [u8], rx_buf: &'static mut [u8], cfg: UartConfig) -> BufferedUart {
    use embassy_rp::peripherals::{PIN_10, PIN_11, PIN_22, PIN_26, PIN_27, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8};
    fn on<T: TxPin<UART1>>(uart: Peri<'static, UART1>, tx: Peri<'static, T>, rx: u8, tx_buf: &'static mut [u8], rx_buf: &'static mut [u8], cfg: UartConfig) -> BufferedUart {
        // Safety: as for `output`.
        unsafe {
            match rx {
                7 => BufferedUart::new(uart, tx, PIN_7::steal(), Irqs, tx_buf, rx_buf, cfg),
                11 => BufferedUart::new(uart, tx, PIN_11::steal(), Irqs, tx_buf, rx_buf, cfg),
                27 => BufferedUart::new(uart, tx, PIN_27::steal(), Irqs, tx_buf, rx_buf, cfg),
                _ => BufferedUart::new(uart, tx, PIN_5::steal(), Irqs, tx_buf, rx_buf, cfg),
            }
        }
    }
    unsafe {
        match pins.tx {
            6 => on(uart, PIN_6::steal(), pins.rx, tx_buf, rx_buf, cfg),
            8 => on(uart, PIN_8::steal(), pins.rx, tx_buf, rx_buf, cfg),
            10 => on(uart, PIN_10::steal(), pins.rx, tx_buf, rx_buf, cfg),
            22 => on(uart, PIN_22::steal(), pins.rx, tx_buf, rx_buf, cfg),
            26 => on(uart, PIN_26::steal(), pins.rx, tx_buf, rx_buf, cfg),
            _ => on(uart, PIN_4::steal(), pins.rx, tx_buf, rx_buf, cfg),
        }
    }
}

/// Node 2's link: UART0 on the saved pins, as `node1_uart`.
fn node2_uart(uart: Peri<'static, UART0>, pins: board::NodePins, tx_buf: &'static mut [u8], rx_buf: &'static mut [u8], cfg: UartConfig) -> BufferedUart {
    use embassy_rp::peripherals::{PIN_0, PIN_1, PIN_12, PIN_13, PIN_14, PIN_15, PIN_2, PIN_28, PIN_3};
    fn on<T: TxPin<UART0>>(uart: Peri<'static, UART0>, tx: Peri<'static, T>, rx: u8, tx_buf: &'static mut [u8], rx_buf: &'static mut [u8], cfg: UartConfig) -> BufferedUart {
        // Safety: as for `output`.
        unsafe {
            match rx {
                3 => BufferedUart::new(uart, tx, PIN_3::steal(), Irqs, tx_buf, rx_buf, cfg),
                13 => BufferedUart::new(uart, tx, PIN_13::steal(), Irqs, tx_buf, rx_buf, cfg),
                15 => BufferedUart::new(uart, tx, PIN_15::steal(), Irqs, tx_buf, rx_buf, cfg),
                _ => BufferedUart::new(uart, tx, PIN_1::steal(), Irqs, tx_buf, rx_buf, cfg),
            }
        }
    }
    unsafe {
        match pins.tx {
            2 => on(uart, PIN_2::steal(), pins.rx, tx_buf, rx_buf, cfg),
            12 => on(uart, PIN_12::steal(), pins.rx, tx_buf, rx_buf, cfg),
            14 => on(uart, PIN_14::steal(), pins.rx, tx_buf, rx_buf, cfg),
            28 => on(uart, PIN_28::steal(), pins.rx, tx_buf, rx_buf, cfg),
            _ => on(uart, PIN_0::steal(), pins.rx, tx_buf, rx_buf, cfg),
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // Which firmware slot this is, and whether the boot ROM started it on trial after an update.
    let fw = fw::Fw::init(p.FLASH);

    // Identity: last two MAC bytes and the boot id from the chip's unique id.
    let uid = embassy_rp::otp::get_chipid().unwrap_or(0x1234_5678_9ABC_DEF0);
    let mac = [board::MAC_PREFIX[0], board::MAC_PREFIX[1], board::MAC_PREFIX[2], board::MAC_PREFIX[3], (uid >> 8) as u8, uid as u8];
    static TIMER: StaticCell<Timer> = StaticCell::new();
    // Saved settings. A jumper from GP9 (header pin 12) to the GND next to it (pin 13) at
    // power-up ignores a saved static address and uses DHCP: the way back from a wrong one.
    let saved = settings::load();
    let recovery_pin = Input::new(p.PIN_9, Pull::Up);
    embassy_time::Timer::after_millis(2).await;
    let jumper = recovery_pin.is_low();
    // A crash ended the last run: this one starts on safe pins too (`crash.rs`).
    let crashed = crash::take();
    let net_recovery = jumper && saved.net != state::NetConfig::Dhcp;
    let net_boot = if net_recovery { state::NetConfig::Dhcp } else { saved.net };
    drop(recovery_pin);
    let timer: &'static Timer = TIMER.init(Timer::new(seed() as u32, mac, saved, net_boot, net_recovery));

    // --- Nodes: the shared EN/BOOT lines and one serial link each, on the saved pins
    // (checked when they were saved and again when read back; `settings::load` falls back
    // to the defaults).
    // The same jumper ignores the saved Node pins for this start and uses the defaults (one
    // Node): the way back from pins the board cannot start with. The saved pins stay
    // saved, and the page shows that a restart would bring them back.
    let pins = if jumper || crashed.is_some() { board::Pins::default() } else { saved.pins };
    timer.with(|i| {
        i.pins_boot = pins;
        i.crash = crashed;
    });
    // EN and BOOT are optional. Where they are wired, the Nodes are reset here so they and
    // the Timer start together; without them a Node that is already running carries on.
    let mut wires = match (pins.en, pins.boot) {
        (Some(en), Some(boot)) => Some(node::Wires { en: output(en, Level::High), boot: output(boot, Level::High) }),
        _ => None,
    };
    if let Some(w) = &mut wires {
        node::reset_nodes(w, false).await;
    }
    static UART_TX_BUF: [StaticCell<[u8; 1024]>; board::HW_UARTS] = [const { StaticCell::new() }; board::HW_UARTS];
    static UART_RX_BUF: [StaticCell<[u8; 2048]>; board::HW_UARTS] = [const { StaticCell::new() }; board::HW_UARTS];
    let mut ucfg = UartConfig::default();
    ucfg.baudrate = board::node::BAUD;
    let mut links: [Option<(LinkTx, LinkRx)>; board::MAX_NODES] = [const { None }; board::MAX_NODES];
    if let Some(np) = pins.nodes[0] {
        let (tx, rx) = node1_uart(p.UART1, np, UART_TX_BUF[0].init([0; 1024]), UART_RX_BUF[0].init([0; 2048]), ucfg).split();
        links[0] = Some((LinkTx::Uart(tx), LinkRx::Uart(rx)));
    }
    if let Some(np) = pins.nodes[1] {
        let (tx, rx) = node2_uart(p.UART0, np, UART_TX_BUF[1].init([0; 1024]), UART_RX_BUF[1].init([0; 2048]), ucfg).split();
        links[1] = Some((LinkTx::Uart(tx), LinkRx::Uart(rx)));
    }
    let [l2, l3] = piolink::start(p.PIO0, (p.DMA_CH2, p.DMA_CH3), [pins.nodes[2], pins.nodes[3]], board::node::BAUD);
    links[2] = l2;
    links[3] = l3;
    // A link with no Node on it, or a Node without power, leaves its receive line floating,
    // and noise from the wires beside it then reads as data (CRC errors on a dead link, seen
    // 2026-11-27). Hold each receive line at its idle level.
    for np in pins.nodes.iter().flatten() {
        embassy_rp::pac::PADS_BANK0.gpio(np.rx as usize).modify(|w| {
            w.set_pue(true);
            w.set_pde(false);
        });
    }
    for (n, link) in links.into_iter().enumerate() {
        if let Some((tx, rx)) = link {
            spawner.spawn(node::node_rx_task(timer, n, rx).unwrap());
            spawner.spawn(node::node_tx_task(timer, n, tx).unwrap());
        }
    }
    spawner.spawn(node::sync_task(timer, output(pins.sync, Level::Low)).unwrap());
    spawner.spawn(node::node_ctl_task(timer, wires).unwrap());

    // --- Ethernet: W6300 on SPI0.
    let mut scfg = SpiConfig::default();
    scfg.frequency = board::eth::SPI_HZ;
    let spi = Spi::new(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_16, p.DMA_CH0, p.DMA_CH1, Irqs, scfg);
    let cs = Output::new(p.PIN_17, Level::High);
    let spi_dev = ExclusiveDevice::new(spi, cs, Delay).unwrap();
    let int = Input::new(p.PIN_21, Pull::Up);
    let rst = Output::new(p.PIN_20, Level::High);
    static WIZ_STATE: StaticCell<WiznetState<8, 8>> = StaticCell::new();
    let (device, eth_runner) = embassy_net_wiznet::new(mac, WIZ_STATE.init(WiznetState::new()), spi_dev, int, rst).await.unwrap();
    spawner.spawn(ethernet_task(eth_runner).unwrap());

    static RESOURCES: StaticCell<StackResources<{ web::WEB_TASK_POOL_SIZE + 3 }>> = StaticCell::new();
    let net_config = match net_boot {
        state::NetConfig::Dhcp => embassy_net::Config::dhcpv4(Default::default()),
        state::NetConfig::Static { address, prefix, gateway, dns } => {
            let mut dns_servers = heapless::Vec::new();
            if let Some(d) = dns {
                let _ = dns_servers.push(embassy_net::Ipv4Address::from(d));
            }
            embassy_net::Config::ipv4_static(embassy_net::StaticConfigV4 {
                address: embassy_net::Ipv4Cidr::new(embassy_net::Ipv4Address::from(address), prefix),
                gateway: gateway.map(embassy_net::Ipv4Address::from),
                dns_servers,
            })
        }
    };
    let (stack, net_runner) = embassy_net::new(netdev::LinkGated(device), net_config, RESOURCES.init(StackResources::new()), seed());
    spawner.spawn(net_task(net_runner).unwrap());

    // --- OFT Protocol.
    spawner.spawn(web::status_tick_task(timer).unwrap());
    spawner.spawn(web::net_watch_task(timer, stack).unwrap());
    spawner.spawn(mdns::mdns_task(timer, stack).unwrap());
    let adc = embassy_rp::adc::Adc::new(p.ADC, Irqs, Default::default());
    spawner.spawn(web::temp_task(timer, adc, embassy_rp::adc::Channel::new_temp_sensor(p.ADC_TEMP_SENSOR)).unwrap());
    for id in 0..web::WEB_TASK_POOL_SIZE {
        spawner.spawn(web::web_task(id, stack, timer, fw).unwrap());
    }
    spawner.spawn(fw::fw_task(fw, stack).unwrap());
    #[cfg(feature = "usb-console")]
    usb::start(spawner, p.USB, timer, fw);

    // Keep `main` alive doing nothing; the DMA channel handles are owned by the SPI.
    let _ = stack;
    loop {
        embassy_time::Timer::after_secs(60).await;
        crash::healthy();
    }
}

#[allow(dead_code)]
fn _assert_types(_: &Stack<'static>, _: &DMA_CH0, _: &DMA_CH1) {}
