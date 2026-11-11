# oft-timer-rp2350 — OFT Timer firmware (build B1)

Firmware for the **Waveshare RP2350-POE-ETH** (RP2350A, W6300 Ethernet, PoE module) driving one
**XIAO ESP32-C5** Node over UART. It serves the OFT Protocol (spec §6) on port 7270 and runs the
shared `oft-core` detection and clock mapping. Rust, Embassy, stable toolchain.

Status (2026-11-08): **compiles for the target; not yet run on hardware.** The boards are on
order. Everything above the chip drivers is the same code tested in the `oft` workspace.

## Build and flash

```sh
rustup target add thumbv8m.main-none-eabihf
cargo build --release
rustup component add llvm-tools
python3 tools/uf2.py target/thumbv8m.main-none-eabihf/release/oft-timer-rp2350 oft-timer.uf2
```

Hold **BOOT** on the board, plug in USB-C, release; copy `oft-timer.uf2` onto the `RP2350`
drive that appears. The board reboots into the firmware. No debug probe needed.

Then plug Ethernet (PoE) in. The Timer takes a DHCP address and serves
`http://<ip>:7270/oft/v1/info`. mDNS is not implemented yet: find the address on the switch
or router, or use `oftd`-style discovery later.

To update a board that already runs this firmware without pressing BOOT, leave its USB-C
plugged into a computer and send `POST /oft/v1/fw/bootsel`: it restarts as the `RP2350` drive.
There is no update over the network yet.

## Wiring

| Signal | RP2350 | XIAO ESP32-C5 |
|---|---|---|
| 5 V | 5V out | 5V |
| GND | GND | GND |
| UART TX → Node RX | GP4 | D7 (GPIO12) |
| UART RX ← Node TX | GP5 | D6 (GPIO11) |
| SYNC | GP6 → | free GPIO (input) |
| Node EN (reset) | GP7 → | EN / RST |
| Node BOOT | GP8 → | BOOT strap |

W6300: SPI0 SCK GP18, MOSI GP19, MISO GP16, CS GP17, RST GP20, INT GP21 (Waveshare's
`DEV_Config.h`). Board extras not used yet: WS2812 on GP25, USB detect GP24, PoE detect
GP23, VSYS ADC GP29. All pins are in `src/board.rs`.

## What it does

- `node.rs`: decodes Node-link frames (COBS + CRC-16) into `oft_core::NodeHost`, sends `Plan`
  and `Ping`, pulses SYNC at 20 Hz, resets the C5 at boot via EN/BOOT.
- `state.rs`: shared state behind a critical-section mutex; 64-crossing replay buffer; a 100 Hz,
  30 s trace per seat for `/samples`; a pub/sub channel feeding WebSocket clients.
- `web.rs`: picoserve HTTP + WebSocket: `/oft/v1/{info,config,status,time,crossings,samples,
  stream}`. Conformance level **L2** plus reduced-rate history (the hello advertises what it keeps).

## Not yet

- Flashing the C5 from the Timer (the EN/BOOT lines are wired for it; the ESP ROM loader
  protocol is not written).
- mDNS `_oft._tcp`, persistent config, OTA of the RP2350 itself, status LED.
- First-boot checks: SPI clock (starts at 30 MHz; Waveshare runs 70), UART pins vs the
  header silkscreen, whether the C5's BOOT strap is GPIO28 on the XIAO.

GPL-3.0-or-later.
