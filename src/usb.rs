//! Optional serial console on the USB-C port (cargo feature `usb-console`).
//!
//! The Timer shows up as a serial port (USB CDC-ACM; Windows, macOS and Linux have the
//! driver built in). It is for the moments the network cannot help: finding the address,
//! getting back from a wrong fixed address, a status line, restart, USB update mode.
//! The OFT Protocol is not served here.
//!
//! Line-based: type a command, press Enter. `help` lists them.

use crate::fw::Fw;
use crate::state::{NetConfig, Saved, Timer};
use crate::web::{ip_text, parse_net};
use core::fmt::Write as _;
use embassy_executor::Spawner;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{bind_interrupts, Peri};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use heapless::String;
use static_cell::StaticCell;

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

type Drv = Driver<'static, USB>;
type Port = CdcAcmClass<'static, Drv>;

/// pid.codes test ID: fine on a bench, not for boards that are sold. Get a real one first.
const VID_PID: (u16, u16) = (0x1209, 0x0001);
const PACKET: usize = 64;

const HELP: &str = "\
Commands:\r\n\
  status       what the Timer is doing\r\n\
  net          address now and the saved network settings\r\n\
  net dhcp     save: get the address from DHCP\r\n\
  net static <address>/<prefix> [gateway]\r\n\
               save a fixed address, e.g.\r\n\
               net static 192.168.1.50/24 192.168.1.1\r\n\
  restart      restart the Timer (network changes apply then)\r\n\
  bootsel      restart as the RP2350 USB drive, to install firmware\r\n\
  help\r\n";

pub fn start(spawner: Spawner, usb: Peri<'static, USB>, timer: &'static Timer, fw: &'static Fw) {
    static SERIAL: StaticCell<String<8>> = StaticCell::new();
    let serial = SERIAL.init(String::new());
    let _ = write!(serial, "{:02x}{:02x}", timer.mac[4], timer.mac[5]);
    let mut config = Config::new(VID_PID.0, VID_PID.1);
    config.manufacturer = Some("Open FPV Timer");
    config.product = Some("OFT Timer console");
    config.serial_number = Some(serial.as_str());
    config.max_power = 100;

    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 64]> = StaticCell::new();
    static CONTROL: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();
    let mut builder = Builder::new(Driver::new(usb, UsbIrqs), config, CONFIG_DESC.init([0; 256]), BOS_DESC.init([0; 64]), &mut [], CONTROL.init([0; 64]));
    let port = CdcAcmClass::new(&mut builder, STATE.init(State::new()), PACKET as u16);
    spawner.spawn(device_task(builder.build()).unwrap());
    spawner.spawn(console_task(port, timer, fw).unwrap());
}

#[embassy_executor::task]
async fn device_task(mut dev: UsbDevice<'static, Drv>) -> ! {
    dev.run().await
}

#[embassy_executor::task]
async fn console_task(mut port: Port, timer: &'static Timer, fw: &'static Fw) -> ! {
    loop {
        port.wait_connection().await;
        // Ends when the cable is pulled or the host resets the port; then wait again.
        let _ = session(&mut port, timer, fw).await;
    }
}

/// Send text in packets shorter than the endpoint size, so no transfer ends on a full
/// packet (which would need a zero-length one after it).
async fn say(port: &mut Port, text: &str) -> Result<(), EndpointError> {
    for chunk in text.as_bytes().chunks(PACKET - 1) {
        port.write_packet(chunk).await?;
    }
    Ok(())
}

async fn session(port: &mut Port, timer: &'static Timer, fw: &'static Fw) -> Result<(), EndpointError> {
    let mut line: String<96> = String::new();
    let mut greeted = false;
    let mut buf = [0u8; PACKET];
    loop {
        let n = port.read_packet(&mut buf).await?;
        if !greeted {
            // Greet on the first key press: writing before a terminal has opened the port
            // would block on some hosts.
            greeted = true;
            let mut s: String<128> = String::new();
            let _ = write!(s, "\r\nOFT Timer oft-{:02x}{:02x} console. Type help and press Enter.\r\n", timer.mac[4], timer.mac[5]);
            say(port, &s).await?;
        }
        for &b in &buf[..n] {
            match b {
                b'\r' | b'\n' => {
                    say(port, "\r\n").await?;
                    if !line.is_empty() {
                        run(port, timer, fw, line.as_str()).await?;
                        line.clear();
                    }
                    say(port, "> ").await?;
                }
                // backspace / delete
                8 | 127 => {
                    if line.pop().is_some() {
                        say(port, "\x08 \x08").await?;
                    }
                }
                32..=126 => {
                    if line.push(b as char).is_ok() {
                        port.write_packet(&[b]).await?;
                    }
                }
                _ => {}
            }
        }
    }
}

fn net_text(n: NetConfig, out: &mut String<160>) {
    match n {
        NetConfig::Dhcp => {
            let _ = out.push_str("DHCP");
        }
        NetConfig::Static { address, prefix, gateway, .. } => {
            let _ = write!(out, "fixed {}/{}", ip_text(address), prefix);
            if let Some(g) = gateway {
                let _ = write!(out, " gateway {}", ip_text(g));
            }
        }
    }
}

async fn show_net(port: &mut Port, timer: &'static Timer) -> Result<(), EndpointError> {
    let (now, boot, recovery, saved) = timer.with(|i| (i.net_now, i.net_boot, i.net_recovery, i.saved.net));
    let mut s: String<160> = String::new();
    match now {
        Some(n) => {
            let _ = write!(s, "Address now: {}/{}", ip_text(n.address), n.prefix);
            if let Some(g) = n.gateway {
                let _ = write!(s, " gateway {}", ip_text(g));
            }
            let _ = write!(s, "  ->  http://{}:{}/\r\n", ip_text(n.address), crate::board::OFT_PORT);
        }
        None => {
            let _ = s.push_str("Address now: none yet (no link, or DHCP has not answered)\r\n");
        }
    }
    say(port, &s).await?;
    s.clear();
    let _ = s.push_str("This start:  ");
    net_text(boot, &mut s);
    let _ = s.push_str(if recovery { " (recovery jumper fitted)\r\n" } else { "\r\n" });
    say(port, &s).await?;
    s.clear();
    let _ = s.push_str("Saved:       ");
    net_text(saved, &mut s);
    let _ = s.push_str(if saved != boot || recovery { "  (applies at the next restart)\r\n" } else { "\r\n" });
    say(port, &s).await
}

async fn save_net(port: &mut Port, timer: &'static Timer, fw: &'static Fw, net: NetConfig) -> Result<(), EndpointError> {
    let saved = timer.with(|i| Saved { net, ..i.saved });
    match crate::settings::save(timer, fw, saved).await {
        Ok(()) => {
            say(port, "Saved. Type restart to use it.\r\n").await?;
            show_net(port, timer).await
        }
        Err(e) => {
            say(port, "Could not save: ").await?;
            say(port, e).await?;
            say(port, "\r\n").await
        }
    }
}

async fn run(port: &mut Port, timer: &'static Timer, fw: &'static Fw, line: &str) -> Result<(), EndpointError> {
    let mut words = line.split_ascii_whitespace();
    match (words.next(), words.next(), words.next(), words.next()) {
        (Some("help" | "?"), ..) => say(port, HELP).await,
        (Some("status"), ..) => {
            let mut s: String<160> = String::new();
            let _ = write!(s, "Timer oft-{:02x}{:02x}, firmware {}", timer.mac[4], timer.mac[5], env!("CARGO_PKG_VERSION"));
            if let Ok(slot) = fw.slot {
                let _ = write!(s, ", slot {}", slot.name());
            }
            if let Some(b) = fw.build {
                let _ = write!(s, ", build {}", b);
            }
            let _ = write!(s, ", up {} s\r\n", crate::state::now_us() / 1_000_000);
            say(port, &s).await?;
            s.clear();
            timer.with(|i| {
                let clock = i.host.clock();
                let _ = write!(s, "Node {}, clock {}", if i.node_connected { "connected" } else { "NOT connected" }, if clock.is_synced() { "synced" } else { "not synced" });
                let _ = write!(s, ", lost samples {}, receive errors {}", i.host.dropped_samples, i.dropped_frames);
                if let Some(t) = i.timer_temp_dc {
                    let _ = write!(s, ", Timer {} C", t / 10);
                }
                if let Some(t) = i.node_temp_dc {
                    let _ = write!(s, ", Node {} C", t / 10);
                }
                let _ = s.push_str("\r\n");
            });
            say(port, &s).await?;
            show_net(port, timer).await
        }
        (Some("net"), None, ..) => show_net(port, timer).await,
        (Some("net"), Some("dhcp"), None, _) => save_net(port, timer, fw, NetConfig::Dhcp).await,
        (Some("net"), Some("static"), Some(addr), gw) => {
            let parsed = addr.split_once('/').and_then(|(a, p)| p.parse::<u8>().ok().map(|p| (a, p)));
            match parsed {
                None => say(port, "Give the address with its prefix, like 192.168.1.50/24 (24 = mask 255.255.255.0).\r\n").await,
                Some((a, p)) => match parse_net(a, p, gw) {
                    Ok(net) => save_net(port, timer, fw, net).await,
                    Err(e) => {
                        say(port, e).await?;
                        say(port, "\r\n").await
                    }
                },
            }
        }
        (Some("restart"), ..) => {
            if fw.on_trial() {
                say(port, "This firmware is still on trial after an update; wait for it to confirm.\r\n").await
            } else {
                fw.restart_soon();
                say(port, "Restarting.\r\n").await
            }
        }
        (Some("bootsel"), ..) => {
            say(port, "Restarting as the RP2350 drive.\r\n").await?;
            crate::web::BOOTSEL_REQUESTED.store(true, core::sync::atomic::Ordering::Relaxed);
            Ok(())
        }
        _ => say(port, "Not a command. Type help.\r\n").await,
    }
}
