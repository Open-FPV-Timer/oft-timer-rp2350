//! The Timer's shared state: one Node's `NodeHost`, the config, crossing replay, a reduced
//! sample trace per seat, and the pub/sub channel the WebSocket clients drink from.
//!
//! Guarded by a blocking critical-section mutex: every access is short and never awaits.

use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::pubsub::PubSubChannel;
use heapless::Deque;
use oft_core::{Crossing, DetectConfig, NodeHost, Threshold};
use oft_nodelink::msg::{Hello, Plan, GAIN_AUTO};

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

    pub fn plan(&self) -> Plan {
        let mut p = Plan::with(0, 0, GAIN_AUTO);
        // One Node: every enabled seat goes to it (a Plan holds as many as there are seats).
        for s in self.seats.iter().flatten().filter(|s| s.enabled) {
            let _ = p.push(s.seat, s.freq_mhz);
        }
        p
    }

    /// Give the host the shared detection settings and each seat's own threshold. Call
    /// before `set_plan`, which builds the detectors from them.
    pub fn apply_detection(&self, host: &mut NodeHost) {
        host.set_detect_config(self.detect());
        for (seat, s) in self.seats.iter().enumerate() {
            host.set_seat_threshold(seat as u8, s.and_then(|s| s.threshold).map(Into::into));
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

/// Progress and result of the Node wiring test (`node::node_ctl_task`).
#[derive(Clone, Copy, Default)]
pub struct WireTest {
    pub running: bool,
    pub done: bool,
    pub step: &'static str,
    /// EN: the Node restarted when the Timer pulsed its reset line.
    pub en_ok: Option<bool>,
    pub en_ms: u32,
    /// BOOT: held low through a reset, the Node's ROM bootloader answered on the serial link.
    pub boot_ok: Option<bool>,
    /// The Node came back to normal operation afterwards.
    pub recovered: Option<bool>,
    pub recover_ms: u32,
}

/// Progress and result of writing firmware to the Node (`node::node_ctl_task`).
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
    pub written: u32,
    pub baud: u32,
    /// The Node came back on the Node link afterwards.
    pub returned: Option<bool>,
    pub seconds: u32,
}

pub struct Inner {
    pub host: NodeHost,
    pub config: Config,
    pub crossings: Deque<(u64, Crossing), CROSSING_REPLAY>,
    pub next_seq: u64,
    pub traces: [Option<Trace>; MAX_SEATS],
    pub node_connected: bool,
    pub node_temp_dc: Option<i16>,
    /// The RP2350's own die temperature, 0.1 °C.
    pub timer_temp_dc: Option<i16>,
    pub last_node_rx_us: Option<u64>,
    /// Node-link frames that failed their checksum or framing.
    pub dropped_frames: u32,
    /// Errors the UART reported on receive (overrun, framing, noise, break).
    pub uart_errors: u32,
    /// The Node's last `Status` message.
    pub node_status: Option<oft_nodelink::msg::Status>,
    pub wire_test: WireTest,
    pub node_flash: NodeFlash,
    /// Set by `set_config`; the node task sends the new plan and clears it.
    pub plan_dirty: bool,
    /// What flash holds (kept in step by the save handlers).
    pub saved: Saved,
    /// The network settings this boot started with.
    pub net_boot: NetConfig,
    /// The recovery jumper was fitted at start-up, so a saved static address was ignored.
    pub net_recovery: bool,
    pub net_now: Option<NetNow>,
    /// The Node pins this boot started with.
    pub pins_boot: crate::board::Pins,
}

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
}

impl Timer {
    pub fn new(boot_id: u32, mac: [u8; 6], saved: Saved, net_boot: NetConfig, net_recovery: bool) -> Self {
        let config = saved.config.unwrap_or_default();
        let mut host = NodeHost::new(config.detect());
        config.apply_detection(&mut host);
        host.set_plan(config.plan());
        Self {
            inner: Mutex::new(RefCell::new(Inner {
                host,
                config,
                crossings: Deque::new(),
                next_seq: 1,
                traces: [const { None }; MAX_SEATS],
                node_connected: false,
                node_temp_dc: None,
                timer_temp_dc: None,
                last_node_rx_us: None,
                dropped_frames: 0,
                uart_errors: 0,
                node_status: None,
                wire_test: WireTest { running: false, done: false, step: "", en_ok: None, en_ms: 0, boot_ok: None, recovered: None, recover_ms: 0 },
                node_flash: NodeFlash { running: false, done: false, ok: false, step: "", error: "", offset: 0, total: 0, written: 0, baud: 0, returned: None, seconds: 0 },
                plan_dirty: true,
                saved,
                net_boot,
                net_recovery,
                net_now: None,
                pins_boot: saved.pins,
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
            config.apply_detection(&mut i.host);
            // Only a change of seats or frequencies re-plans the Node. Re-planning restarts
            // its sample stream (a gap of about 60 ms) and resets the detectors, so a change
            // of levels alone must not do it.
            let plan = config.plan();
            if i.host.plan() != Some(&plan) {
                i.host.set_plan(plan);
                for t in i.traces.iter_mut() {
                    *t = None;
                }
                i.plan_dirty = true;
            }
            i.config = config;
        });
    }

    pub fn hello(&self) -> Option<Hello> {
        self.with(|i| i.host.hello().copied())
    }
}

/// Timer clock: µs since boot.
pub fn now_us() -> u64 {
    embassy_time::Instant::now().as_micros()
}
