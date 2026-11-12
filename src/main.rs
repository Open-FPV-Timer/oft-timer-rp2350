//! OFT Timer firmware for the Waveshare RP2350-POE-ETH (build B1).
//!
//! Tasks: W6300 Ethernet runner, network stack, Node UART rx/tx, SYNC pulses, a pool of
//! HTTP/WebSocket servers, and a status tick. All timing logic lives in `oft-core`.

#![no_std]
#![no_main]

mod board;
mod fw;
mod node;
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
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH1, SPI0, UART1};
use embassy_rp::spi::{Async, Config as SpiConfig, Spi};
use embassy_rp::uart::{BufferedInterruptHandler, BufferedUart, Config as UartConfig};
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use panic_halt as _;
use state::Timer;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    UART1_IRQ => BufferedInterruptHandler<UART1>;
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
    embassy_rp::binary_info::rp_program_description!(c"Open FPV Timer: OFT Protocol over Ethernet, one C5 Node on UART1"),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

type SpiDev = ExclusiveDevice<Spi<'static, SPI0, Async>, Output<'static>, Delay>;

#[embassy_executor::task]
async fn ethernet_task(runner: Runner<'static, W6300, SpiDev, Input<'static>, Output<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, Device<'static>>) -> ! {
    runner.run().await
}

/// A pseudo-random seed from the ring oscillator; good enough for TCP sequence numbers.
fn seed() -> u64 {
    let mut rng = embassy_rp::clocks::RoscRng;
    rng.next_u64()
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
    let net_recovery = recovery_pin.is_low() && saved.net != state::NetConfig::Dhcp;
    let net_boot = if net_recovery { state::NetConfig::Dhcp } else { saved.net };
    drop(recovery_pin);
    let timer: &'static Timer = TIMER.init(Timer::new(seed() as u32, mac, saved, net_boot, net_recovery));

    // --- Node: EN/BOOT lines and the UART.
    let mut en = Output::new(p.PIN_7, Level::High);
    let mut boot = Output::new(p.PIN_8, Level::High);
    node::reset_node(&mut en, &mut boot, false).await;
    static UART_TX_BUF: StaticCell<[u8; 1024]> = StaticCell::new();
    static UART_RX_BUF: StaticCell<[u8; 2048]> = StaticCell::new();
    let mut ucfg = UartConfig::default();
    ucfg.baudrate = board::node::BAUD;
    let uart = BufferedUart::new(p.UART1, p.PIN_4, p.PIN_5, Irqs, UART_TX_BUF.init([0; 1024]), UART_RX_BUF.init([0; 2048]), ucfg);
    let (uart_tx, uart_rx) = uart.split();
    spawner.spawn(node::node_rx_task(timer, uart_rx).unwrap());
    spawner.spawn(node::node_tx_task(timer, uart_tx).unwrap());
    spawner.spawn(node::sync_task(timer, Output::new(p.PIN_6, Level::Low)).unwrap());
    spawner.spawn(node::node_ctl_task(timer, en, boot).unwrap());

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

    static RESOURCES: StaticCell<StackResources<{ web::WEB_TASK_POOL_SIZE + 2 }>> = StaticCell::new();
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
    let (stack, net_runner) = embassy_net::new(device, net_config, RESOURCES.init(StackResources::new()), seed());
    spawner.spawn(net_task(net_runner).unwrap());

    // --- OFT Protocol.
    spawner.spawn(web::status_tick_task(timer).unwrap());
    spawner.spawn(web::net_watch_task(timer, stack).unwrap());
    let adc = embassy_rp::adc::Adc::new(p.ADC, Irqs, Default::default());
    spawner.spawn(web::temp_task(timer, adc, embassy_rp::adc::Channel::new_temp_sensor(p.ADC_TEMP_SENSOR)).unwrap());
    for id in 0..web::WEB_TASK_POOL_SIZE {
        spawner.spawn(web::web_task(id, stack, timer, fw).unwrap());
    }
    spawner.spawn(fw::fw_task(fw, stack).unwrap());
    #[cfg(feature = "usb-console")]
    usb::start(spawner, p.USB, timer, fw);

    // Keep `main` alive doing nothing; the DMA channel handles are owned by the SPI.
    let _ = (p.DMA_CH2, stack);
    loop {
        embassy_time::Timer::after_secs(60).await;
    }
}

#[allow(dead_code)]
fn _assert_types(_: &Stack<'static>, _: &DMA_CH0, _: &DMA_CH1) {}
