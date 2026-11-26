//! mDNS / DNS-SD: the Timer announces itself as `<id>._oft._tcp.local.` (spec §6.1) and
//! answers for `<id>.local`, so clients find it without being told an address.
//! `oft_fwimage::mdns` builds and reads the messages; this is the socket around it.
//!
//! Same instance name and TXT keys as `oftd`: `id`, `fw`, `proto`, `seats`, `nodes`, `level`.

use crate::state::Timer;
use core::fmt::Write as _;
use embassy_futures::select::{select, Either};
use embassy_net::udp::{PacketMetadata, UdpSocket};
use embassy_net::{IpAddress, IpEndpoint, Ipv4Address, Stack};
use embassy_time::{Duration, Ticker};
use heapless::String;
use oft_fwimage::mdns::{announce, respond, Service, GROUP, PORT};
use static_cell::StaticCell;

const SERVICE: &str = "_oft._tcp";

/// What the TXT record is built from. A change is announced.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Facts {
    addr: [u8; 4],
    seats: u8,
    nodes: u8,
}

fn facts(timer: &Timer, stack: Stack<'static>) -> Option<Facts> {
    let addr = stack.config_v4()?.address.address().octets();
    let (seats, nodes) = timer.with(|i| (i.config.enabled_seats().count() as u8, (0..i.node_count()).filter(|&n| i.hosts.hello(n).is_some()).count() as u8));
    Some(Facts { addr, seats, nodes })
}

#[embassy_executor::task]
pub async fn mdns_task(timer: &'static Timer, stack: Stack<'static>) -> ! {
    static RX_META: StaticCell<[PacketMetadata; 4]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; 2]> = StaticCell::new();
    static RX_BUF: StaticCell<[u8; 1536]> = StaticCell::new();
    static TX_BUF: StaticCell<[u8; 768]> = StaticCell::new();
    static MSG: StaticCell<[u8; 1472]> = StaticCell::new();
    static OUT: StaticCell<[u8; 512]> = StaticCell::new();
    let mut socket = UdpSocket::new(stack, RX_META.init([PacketMetadata::EMPTY; 4]), RX_BUF.init([0; 1536]), TX_META.init([PacketMetadata::EMPTY; 2]), TX_BUF.init([0; 768]));
    let (msg, out) = (MSG.init([0; 1472]), OUT.init([0; 512]));
    // mDNS packets carry the largest hop limit, so a receiver can tell they are local.
    socket.set_hop_limit(Some(255));
    let _ = socket.bind(PORT);
    let group = IpEndpoint::new(IpAddress::Ipv4(Ipv4Address::from(GROUP)), PORT);

    let mut id: String<8> = String::new();
    let _ = write!(id, "oft-{:02x}{:02x}", timer.mac[4], timer.mac[5]);

    let mut tick = Ticker::every(Duration::from_secs(1));
    let mut known: Option<Facts> = None;
    // Announcements still to send for the current facts (two, a second apart).
    let mut to_announce = 0u8;
    let mut joined = false;
    loop {
        let event = select(socket.recv_from(msg), tick.next()).await;
        let now = facts(timer, stack);
        let Some(f) = now else {
            // No address: nothing to say, and the group membership went with the link.
            known = None;
            joined = false;
            continue;
        };
        if !joined {
            joined = stack.join_multicast_group(Ipv4Address::from(GROUP)).is_ok();
        }
        let (mut seats, mut nodes): (String<3>, String<3>) = (String::new(), String::new());
        let _ = write!(seats, "{}", f.seats);
        let _ = write!(nodes, "{}", f.nodes);
        let txt = [("id", id.as_str()), ("fw", env!("CARGO_PKG_VERSION")), ("proto", "1"), ("seats", seats.as_str()), ("nodes", nodes.as_str()), ("level", "L2")];
        let service = Service { service: SERVICE, instance: id.as_str(), host: id.as_str(), port: crate::board::OFT_PORT, addr: f.addr, txt: &txt };
        match event {
            Either::First(Ok((n, from))) => {
                if let Some(reply) = respond(&msg[..n], from.endpoint.port, &service, out) {
                    let to = if reply.unicast { from.endpoint } else { group };
                    let _ = socket.send_to(&out[..reply.len], to).await;
                }
            }
            Either::First(Err(_)) => {}
            Either::Second(()) => {
                if known != Some(f) {
                    known = Some(f);
                    to_announce = 2;
                }
                if to_announce > 0 {
                    to_announce -= 1;
                    let n = announce(&service, out);
                    if n > 0 {
                        let _ = socket.send_to(&out[..n], group).await;
                    }
                }
            }
        }
    }
}
