//! The Timer's shared state: the Nodes' `Hosts`, the config, crossing replay, a reduced
//! sample trace per seat, and the pub/sub channel the WebSocket clients drink from.
//!
//! Guarded by a blocking critical-section mutex: every access is short and never awaits.

use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;
use heapless::Deque;
use crate::board::MAX_NODES;
use oft_core::{Crossing, DetectConfig, Hosts, NodeMask, Threshold};

pub const MAX_SEATS: usize = 8;
/// Crossings kept for `/crossings?since=` and WebSocket replay.
pub const CROSSING_REPLAY: usize = 64;
/// Reduced trace: one sample per `TRACE_DT_US` per seat, `TRACE_LEN` deep (30 s at 100 Hz).
pub const TRACE_DT_US: u64 = 10_000;
pub const TRACE_LEN: usize = 3000;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SeatConfig {
    pub seat: u8,
    pub freq_mhz: u16,
    pub enabled: bool,
    /// This seat's own threshold; `None` = the shared one.
    pub threshold: Option<ThresholdConfig>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ThresholdConfig {
    Auto { snr_enter: i16, hysteresis: i16 },
    Manual { enter: i16, exit: i16 },
}

impl From<ThresholdConfig> for Threshold {
    fn from(t: ThresholdConfig) -> Self {
        match t {
            ThresholdConfig::Auto { snr_enter, hysteresis } => Threshold::Auto { snr_enter, hysteresis },
            ThresholdConfig::Manual { enter, exit } => Threshold::Manual { enter, exit },
        }
    }
}

/// How the Timer gets its address. Read once at start-up.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum NetConfig {
    #[default]
    Dhcp,
    Static { address: [u8; 4], prefix: u8, gateway: Option<[u8; 4]>, dns: Option<[u8; 4]> },
}

/// What was read from flash at start-up, and what a save writes back.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Saved {
    /// The power-on config; `None` = the built-in defaults.
    pub config: Option<Config>,
    pub net: NetConfig,
    pub pins: crate::board::Pins,
}

/// The address in use right now.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct NetNow {
    pub address: [u8; 4],
    pub prefix: u8,
    pub gateway: Option<[u8; 4]>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    pub seats: [Option<SeatConfig>; MAX_SEATS],
    pub threshold: ThresholdConfig,
    pub min_lap_ms: u32,
    pub exit_hold_ms: u32,
    /// Pass window, ms. 0 = off.
    pub pass_window_ms: u32,
}

impl Default for Config {
    fn default() -> Self {
        let mut seats = [None; MAX_SEATS];
        seats[0] = Some(SeatConfig { seat: 0, freq_mhz: 5800, enabled: true, threshold: None });
        Self { seats, threshold: ThresholdConfig::Auto { snr_enter: 150, hysteresis: 50 }, min_lap_ms: 2000, exit_hold_ms: 100, pass_window_ms: 0 }
    }
}

impl Config {
    pub fn detect(&self) -> DetectConfig {
        DetectConfig {
            threshold: self.threshold.into(),
            min_lap_us: self.min_lap_ms as u64 * 1000,
            exit_hold_us: self.exit_hold_ms as u64 * 1000,
            pass_window_us: self.pass_window_ms as u64 * 1000,
            ..DetectConfig::default()
        }
    }

    /// Give the hosts the shared detection settings and each seat's own threshold.
    pub fn apply_detection(&self, hosts: &mut Hosts<MAX_NODES>) {
        hosts.set_detect_config(self.detect());
        for (seat, s) in self.seats.iter().enumerate() {
            hosts.set_seat_threshold(seat as u8, s.and_then(|s| s.threshold).map(Into::into));
        }
    }

    pub fn enabled_seats(&self) -> impl Iterator<Item = &SeatConfig> {
        self.seats.iter().flatten().filter(|s| s.enabled)
    }
}

/// A reduced-rate trace for one seat.
pub struct Trace {
    pub samples: Deque<i16, TRACE_LEN>,
    /// Timer time (slot start) of the newest stored sample.
    pub t_last: u64,
    pub last_rssi: i16,
    pub rate_hz: u32,
    /// Accumulator for decimation: sum and count of full-rate samples in the current bin.
    acc_sum: i32,
    acc_n: u16,
    /// Slot number the accumulator is collecting.
    acc_t: u64,
}

impl Trace {
    pub const fn new() -> Self {
        Self { samples: Deque::new(), t_last: 0, last_rssi: 0, rate_hz: 0, acc_sum: 0, acc_n: 0, acc_t: 0 }
    }

    /// Feed one full-rate sample at Timer time `t`.
    ///
    /// The trace is one value per `TRACE_DT_US` slot of Timer time (slot `k` covers
    /// `[k, k+1) * TRACE_DT_US`), so a stored sample's index gives its time exactly, whatever
    /// the Node's sample rate. Slots with no samples (lost batches) repeat the last value.
    pub fn push(&mut self, t: u64, p: i16) {
        self.last_rssi = p;
        let slot = t / TRACE_DT_US;
        if self.acc_n > 0 && slot > self.acc_t {
            let avg = (self.acc_sum / self.acc_n as i32) as i16;
            // The finished slot, then any empty ones before this sample's slot.
            let n = (slot - self.acc_t).min(TRACE_LEN as u64);
            for _ in 0..n {
                if self.samples.is_full() {
                    self.samples.pop_front();
                }
                let _ = self.samples.push_back(avg);
            }
            self.t_last = (slot - 1) * TRACE_DT_US;
            self.acc_sum = 0;
            self.acc_n = 0;
        }
        if self.acc_n == 0 {
            self.acc_t = slot;
        }
        self.acc_sum += p as i32;
        self.acc_n += 1;
    }

    /// Timer time of the oldest stored sample.
    pub fn t_first(&self) -> u64 {
        self.t_last.saturating_sub((self.samples.len().saturating_sub(1) as u64) * TRACE_DT_US)
    }
}

/// Progress and result of the Node wiring test (`node::node_ctl_task`). EN and BOOT are
/// one wire each to every Node, so the test runs on all of them and reports each.
#[derive(Clone, Copy, Default)]
pub struct WireTest {
    pub running: bool,
    pub done: bool,
    pub step: &'static str,
    pub nodes: [NodeWireTest; MAX_NODES],
}

#[derive(Clone, Copy, Default)]
pub struct NodeWireTest {
    /// EN: the Node restarted when the Timer pulsed its reset line.
    pub en_ok: Option<bool>,
    pub en_ms: u32,
    /// BOOT: held low through a reset, the Node's ROM bootloader answered on the serial link.
    pub boot_ok: Option<bool>,
    /// The Node came back to normal operation afterwards.
    pub recovered: Option<bool>,
    pub recover_ms: u32,
}

/// Progress and result of writing firmware to the Nodes (`node::node_ctl_task`).
#[derive(Clone, Copy, Default)]
pub struct NodeFlash {
    pub running: bool,
    pub done: bool,
    pub ok: bool,
    pub step: &'static str,
    pub error: &'static str,
    /// Where in the Node's flash, and how much.
    pub offset: u32,
    pub total: u32,
    /// Bytes written to the Node being written now.
    pub written: u32,
    pub baud: u32,
    /// The Node being written now (or the one that failed), from 0.
    pub node: u8,
    /// Nodes this run writes, and how many are finished.
    pub nodes: u8,
    pub nodes_done: u8,
    /// Every Node written came back on the Node link afterwards.
    pub returned: Option<bool>,
    pub seconds: u32,
}

/// What the Timer knows about one Node link, beside what `Hosts` keeps.
#[derive(Clone, Copy, Default)]
pub struct NodeState {
    /// Heard from in the last few seconds.
    pub connected: bool,
    pub last_rx_us: Option<u64>,
    /// 0.1 °C.
    pub temp_dc: Option<i16>,
    /// The Node's last `Status` message.
    pub status: Option<oft_nodelink::msg::Status>,
    /// Node-link frames that failed their checksum or framing.
    pub dropped_frames: u32,
    /// Errors the UART reported on receive (overrun, framing, noise, break).
    pub uart_errors: u32,
    /// The Node's plan changed (or it restarted); its link task sends it and clears this.
    pub plan_dirty: bool,
}

pub struct Inner {
    /// Clocks, detectors and the seat schedule for every Node.
    pub hosts: Hosts<MAX_NODES>,
    pub nodes: [NodeState; MAX_NODES],
    pub config: Config,
    pub crossings: Deque<(u64, Crossing), CROSSING_REPLAY>,
    pub next_seq: u64,
    pub traces: [Option<Trace>; MAX_SEATS],
    /// The RP2350's own die temperature, 0.1 °C.
    pub timer_temp_dc: Option<i16>,
    pub wire_test: WireTest,
    pub node_flash: NodeFlash,
    /// What flash holds (kept in step by the save handlers).
    pub saved: Saved,
    /// The network settings this boot started with.
    pub net_boot: NetConfig,
    /// The recovery jumper was fitted at start-up, so a saved static address was ignored.
    pub net_recovery: bool,
    pub net_now: Option<NetNow>,
    /// The Node pins this boot started with.
    pub pins_boot: crate::board::Pins,
    /// Who may write (spec §6.6).
    pub control: oft_core::control::Control<MAX_CLAIMS>,
    /// `config_rev` in `/status`: moves whenever the config or what is saved changes.
    pub config_rev: u32,
}

impl Inner {
    /// Nodes wired to this Timer (the links in the pins this boot started with).
    pub fn node_count(&self) -> usize {
        self.pins_boot.node_count()
    }

    /// The Node each seat is on.
    pub fn seat_nodes(&self) -> [Option<u8>; MAX_SEATS] {
        core::array::from_fn(|s| self.hosts.seat_node(s as u8))
    }

    /// Change what the seat schedule depends on (`f` returns the Nodes whose plan changed),
    /// then have those plans sent and start afresh the traces of seats that moved.
    pub fn reschedule(&mut self, f: impl FnOnce(&mut Hosts<MAX_NODES>) -> NodeMask) {
        let before = self.seat_nodes();
        let changed = f(&mut self.hosts);
        self.rescheduled(before, changed);
    }

    /// The schedule may have changed since `before` was read: have the plans of `changed`
    /// sent, and start afresh the traces of seats that moved.
    pub fn rescheduled(&mut self, before: [Option<u8>; MAX_SEATS], changed: NodeMask) {
        self.plans_changed(changed);
        for seat in 0..MAX_SEATS {
            if self.hosts.seat_node(seat as u8) != before[seat] {
                self.traces[seat] = None;
            }
        }
    }

    /// Have these Nodes' plans sent.
    pub fn plans_changed(&mut self, nodes: NodeMask) {
        for (k, n) in self.nodes.iter_mut().enumerate() {
            if nodes & (1 << k) != 0 {
                n.plan_dirty = true;
            }
        }
    }

    /// Node `n` was heard from at `t`. A Node back from a silence joins the schedule again.
    pub fn node_heard(&mut self, n: usize, t: u64) {
        self.nodes[n].last_rx_us = Some(t);
        if !self.nodes[n].connected {
            self.nodes[n].connected = true;
            // Back after a restart, a flash or a cable: it may have no plan.
            self.nodes[n].plan_dirty = true;
            self.reschedule(|h| h.set_up(n, true));
        }
    }

    /// Node `n` has gone quiet: the others take its seats.
    pub fn node_lost(&mut self, n: usize) {
        if self.nodes[n].connected {
            self.nodes[n].connected = false;
            self.reschedule(|h| h.set_up(n, false));
        }
    }
}

/// Standing control claims: one per stream, and `events` takes four subscribers.
pub const MAX_CLAIMS: usize = 4;

pub struct Timer {
    /// Shared by the executor's tasks only, never by interrupt handlers, so the lock must not
    /// mask interrupts: some holders work for the better part of a millisecond (the clock
    /// fit, JSON building), and with interrupts off that long the UART's 32-byte receive
    /// FIFO overflows at 921600 baud and Node frames are lost.
    pub inner: Mutex<ThreadModeRawMutex, RefCell<Inner>>,
    pub events: PubSubChannel<CriticalSectionRawMutex, Event, 32, 4, 2>,
    pub boot_id: u32,
    pub mac: [u8; 6],
}

/// What WebSocket clients receive. Small and `Copy`: the channel clones it per subscriber.
#[derive(Clone, Copy, Debug)]
pub enum Event {
    Crossing { seq: u64, c: Crossing },
    /// A full-rate batch from the Node, already on the Timer clock.
    Samples { seat: u8, t0: u64, dt_us: u16, n: u8, p: [i16; 32], nf: i16 },
    Status,
    /// A claim or the holder of control changed.
    Control,
}

impl Timer {
    pub fn new(boot_id: u32, mac: [u8; 6], saved: Saved, net_boot: NetConfig, net_recovery: bool) -> Self {
        let config = saved.config.unwrap_or_default();
        let mut hosts = Hosts::new(config.detect());
        config.apply_detection(&mut hosts);
        // No Node is up yet: seats are handed out as the Nodes report in.
        hosts.set_seats(config.enabled_seats().map(|s| (s.seat, s.freq_mhz)));
        Self {
            inner: Mutex::new(RefCell::new(Inner {
                hosts,
                nodes: [NodeState::default(); MAX_NODES],
                config,
                crossings: Deque::new(),
                next_seq: 1,
                traces: [const { None }; MAX_SEATS],
                timer_temp_dc: None,
                wire_test: WireTest::default(),
                node_flash: NodeFlash::default(),
                saved,
                net_boot,
                net_recovery,
                net_now: None,
                pins_boot: saved.pins,
                control: oft_core::control::Control::new(boot_id),
                config_rev: 0,
            })),
            events: PubSubChannel::new(),
            boot_id,
            mac,
        }
    }

    /// Run `f` with the state locked. Keep it short; never await inside.
    pub fn with<R>(&self, f: impl FnOnce(&mut Inner) -> R) -> R {
        self.inner.lock(|c| f(&mut c.borrow_mut()))
    }

    pub fn set_config(&self, config: Config) {
        self.with(|i| {
            config.apply_detection(&mut i.hosts);
            // Only a change of seats or frequencies re-plans a Node, and only the Nodes it
            // touches. Re-planning restarts a Node's sample stream (a gap of about 60 ms),
            // and a seat that moves or retunes starts with a new detector, so a change of
            // levels alone must not do it.
            let old = i.config;
            i.reschedule(|h| h.set_seats(config.enabled_seats().map(|s| (s.seat, s.freq_mhz))));
            for seat in 0..MAX_SEATS {
                let freq = |c: &Config| c.seats[seat].filter(|s| s.enabled).map(|s| s.freq_mhz);
                if freq(&old) != freq(&config) {
                    i.traces[seat] = None;
                }
            }
            i.config = config;
            i.config_rev = i.config_rev.wrapping_add(1);
        });
    }
}

/// Timer clock: µs since boot.
pub fn now_us() -> u64 {
    embassy_time::Instant::now().as_micros()
}
