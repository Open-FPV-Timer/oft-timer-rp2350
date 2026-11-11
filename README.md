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

The tool writes two files and prints what is in them:

- `oft-timer.uf2`: **install over USB.** Hold **BOOT** on the board, plug in USB-C, release;
  copy the file onto the `RP2350` drive that appears. The board reboots into the firmware. No
  debug probe needed. This one file sets up the whole flash layout (below), so it is also the
  way back from any state: it works on a blank board, on a board with the older single-image
  layout, and on a board already using this layout.
- `oft-timer.bin`: **update over the network** (next section).

`python3 tools/uf2.py --inspect <file>` checks either file and prints its contents.

Then plug Ethernet (PoE) in. The Timer takes a DHCP address and serves
`http://<ip>:7270/oft/v1/info`. mDNS is not implemented yet: find the address on the switch
or router, or use `oftd`-style discovery later.

To reach the USB bootloader on a board that already runs this firmware without pressing
BOOT, leave its USB-C plugged into a computer and send `POST /oft/v1/fw/bootsel`: it restarts
as the `RP2350` drive.

## Update over the network

Needs the layout that `oft-timer.uf2` installs, so the first install of this scheme is over
USB. After that, open `http://<ip>:7270/`, and under **Firmware** choose `oft-timer.bin` and
press **Update over the network**. Or:

```sh
curl -H 'Content-Type: application/octet-stream' --data-binary @oft-timer.bin http://<ip>:7270/oft/v1/fw/timer
curl http://<ip>:7270/oft/v1/fw/timer     # repeat until "state":"confirmed" in the new slot
```

Timing stops for the whole update (well under a minute): flash cannot be read while it is
being written, so the Timer works with interrupts off for about 50 ms per 4 KiB and drops
Node data, and then it restarts.

How it works. The RP2350 boot ROM does the switching; there is no bootloader of ours.

```text
0x000000  4 K   partition table
0x001000  4 K   erased (the boot ROM's second partition-table slot)
0x002000  2 M   slot A (partition 0)
0x202000  2 M   slot B (partition 1, the "B" partner of A)
0x402000  ...   unused
```

The image is linked at `0x10000000` and runs from either slot: the boot ROM maps the slot it
starts to that address. Every image ends in an 88-byte block holding a build number and a
SHA-256 of the image (the format of `picotool seal --hash`), which the boot ROM checks before
it will start the image.

1. The upload is written to the slot that is **not** running. Nothing is erased until its
   first 4 KiB shows it is Timer firmware for this board with a consistent length. Its first
   sector is written last, after the whole image has been read back from flash and its
   SHA-256 checked, so a partial upload is never something the boot ROM can start.
2. The image is stored marked "try before you buy". The Timer asks the boot ROM to start it
   once (a "flash update" restart).
3. The new firmware confirms itself when Ethernet has an address **and** an HTTP request to
   `/oft/v1/info` or `/oft/v1/fw/timer` has been answered (the page polls; with curl, do it
   yourself). Until then a watchdog is running: without confirmation the board restarts,
   16.7 s after the firmware stops working, or 30 to 47 s after boot if it works but nobody
   reaches it. On a normal start the boot ROM ignores unconfirmed images, so the old firmware
   runs again.

| What goes wrong | What happens |
|---|---|
| Wrong file (another board, another chip, a `.uf2`, a truncated or padded `.bin`) | Refused with a reason. Nothing is erased. |
| Upload corrupted or cut off, or power lost during it | The spare slot is left without a first sector. The running firmware is untouched and starts as usual. |
| Power lost after the upload, before or during the restart | A normal start: the unconfirmed image is ignored, the old firmware runs. Upload again. |
| New firmware crashes or hangs | Watchdog restart within 16.7 s, old firmware runs. |
| New firmware runs but Ethernet never comes up, or nobody asks it anything | Watchdog restart within 47 s of boot, old firmware runs. |
| Power lost while the new firmware confirms itself (about 0.1 s) | The boot ROM rewrites one sector of the new image. If that is cut short the image fails its hash and the old firmware runs. |
| Both slots unusable (should not happen) or the partition table damaged | The board waits as the `RP2350` USB drive: copy `oft-timer.uf2`. |

After a confirmed update the old firmware usually stays in the other slot (the boot ROM picks
the higher build number, and falls back to the other slot if the preferred one ever fails its
hash). After a confirmed *downgrade* the boot ROM erases the newer image's first sector so the
older one keeps booting. `GET /oft/v1/fw/timer` shows the running slot and build, the trial
state, what the other slot holds, and what the boot ROM reported about this boot.

`python3 tools/uf2.py <elf> oft-timer-flat.uf2 --flat` writes the previous layout (one image
at the start of flash, no network update), should you want to go back to it. It also accepts
the raw `.bin` of an earlier build in place of the ELF, which turns a firmware you know works
into a UF2 that installs over any layout.

Not yet confirmed on hardware (2026-11-11): the images and the partition table are checked by
two independent implementations here (`tools/uf2.py` and `fwimage`), written from the boot
ROM and picotool sources, but not by the boot ROM itself. If you have `picotool`,
`picotool info -a oft-timer.bin` should report the version and a verified hash.

The decisions are in `fwimage/` (image checks, write order, running-slot check), with host
tests that include cutting power after every flash operation against a model of the boot
ROM's choice:

```sh
cd fwimage && cargo test --target x86_64-unknown-linux-gnu
OFT_FW_IMAGE=../oft-timer.bin cargo test --target x86_64-unknown-linux-gnu   # also checks a real image
```

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
- `fw.rs`: firmware update over Ethernet on the boot ROM's A/B partitions: `POST /oft/v1/fw/timer`,
  the trial after an update, the `fw.update` capability (`self`).

## Not yet

- Flashing the C5 from the Timer (the EN/BOOT lines are wired for it; the ESP ROM loader
  protocol is not written).
- mDNS `_oft._tcp`, persistent config, status LED.
- First-boot checks: SPI clock (starts at 30 MHz; Waveshare runs 70), UART pins vs the
  header silkscreen, whether the C5's BOOT strap is GPIO28 on the XIAO.

GPL-3.0-or-later.

## Acknowledgements

OFT is inspired by [FPVGate](https://github.com/LouisHitchcock/FPVGateC5MK), which showed
that an ESP32-C5 can time laps, and builds on [widscan58](https://github.com/CapnBry/widscan58),
esp-sdr, C5VRX and RotorHazard. The full list is in the
[oft README](https://github.com/Open-FPV-Timer/oft#acknowledgements).
