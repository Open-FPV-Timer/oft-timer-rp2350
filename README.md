# oft-timer-rp2350 — OFT Timer firmware (build B1)

Firmware for the **Waveshare RP2350-POE-ETH** (RP2350A, W6300 Ethernet, PoE module) driving one
to four **XIAO ESP32-C5** Nodes, each on its own serial link. It serves the OFT Protocol (spec §6) on port 7270 and runs the
shared `oft-core` detection and clock mapping. Rust, Embassy, stable toolchain.

Status (2026-11-27): **running on hardware with four Nodes** on a breadboard, two on the
hardware UARTs and two on PIO links: 1 to 8 seats, Nodes powered off and on while running,
Node firmware written through the Timer with no EN or BOOT wires, the Timer updated over
the network many times. Everything above the chip drivers is the same code tested in the
`oft` workspace. Not run: the EN and BOOT wire fallback since it was reworked, and a real
race.

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
0x402000  8 K   saved settings, two copies (not a partition; a USB install leaves them alone)
0x404000  ...   unused
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
| SYNC | GP6 → | D8 (GPIO8) |
| Node EN (reset), optional | GP7 → | EN pad (back of the XIAO) |
| Node BOOT, optional | GP8 → | BOOT pad (back of the XIAO) |

These are the default pins, for one Node. The wiring is a saved setting: the **Advanced**
page (or `PUT /oft/v1/pins`) moves it, applied at the next restart, so a board wired
differently runs the same firmware. SYNC, EN and BOOT take any header GPIO but GP9 (the
recovery jumper). Wrong pins cannot lock you out: the page stays reachable and a Node
stays silent until they match the wiring.

**EN and BOOT are optional and off by default** (since 2026-11-27). The Timer restarts a
Node (`Reset`) and sends it to its ROM bootloader (`Dfu`) over the Node link, so five wires
are a whole Node: 5 V, GND, two data wires and SYNC, all on the XIAO's header pins. A build
that has EN and BOOT wired sets them on the Advanced page (GP7 and GP8 in the table); the
Timer then still tries the link first and uses the wires when a Node does not answer. A
board that had them wired before this change needs that setting once, since the default is
now "not wired". Without the wires, a Node whose firmware no longer answers is recovered by
hand: hold its BOOT button, tap RESET, release BOOT, then upload from the Timer's page
again; or flash it over its own USB port.

### More than one Node

Set **Nodes** on the Advanced page to how many are wired (up to four) and give each its
data pins. Every Node has its own TX/RX pair. SYNC is one wire joined to every Node. EN
and BOOT, if fitted, are also one wire each joined to every Node, so using them restarts
them all.

| Node | Link | Timer TX (to the Node's D7) | Timer RX (from the Node's D6) |
|---|---|---|---|
| 1 | UART1 | GP4, GP6, GP8, GP10, GP22 or GP26 | GP5, GP7, GP11 or GP27 |
| 2 | UART0 | GP0, GP2, GP12, GP14 or GP28 | GP1, GP3, GP13 or GP15 |
| 3, 4 | PIO (a UART in software) | any free header GPIO | any free header GPIO |

- **Seats are shared out automatically** (spec §4.1, `oft-core`'s `sched`). With as many
  Nodes as seats, each Node watches one seat and never hops. With more seats than Nodes,
  neighbouring channels go to the same Node. `/status` says which Node has each seat.
- **A Node that goes quiet for 3 s loses its seats** to the others, and takes seats back
  when it returns. Seats that do not move are not disturbed.
- **Node firmware** goes to every wired Node, one after the other, from one upload
  (`?node=0` writes only the first, and so on). Over the link each Node restarts on its
  own and the others keep timing; with EN and BOOT they all restart together.
- **The Node restart test** reports each Node separately.
- **Run on hardware 2026-11-27** with four Nodes: all of the above. Pulling a Node's power
  moved its seats to the others, and it took seats back when it returned, with all eight
  seats covered throughout, down to two Nodes. The PIO links (`src/piolink.rs`: state
  machines emptied by DMA) lose nothing while the Timer writes its own flash, which the
  hardware UARTs do.
- **A Node gets a plan only after it has said Hello since it last started.** A Node repeats
  its Hello until a plan arrives, and the first is often lost in its start-up text. A Node
  that is talking but has not said Hello within 4 s (it kept running across a Timer
  restart) is restarted over its link, so it says it.

W6300: SPI0 SCK GP18, MOSI GP19, MISO GP16, CS GP17, RST GP20, INT GP21 (Waveshare's
`DEV_Config.h`). Board extras not used yet: WS2812 on GP25, USB detect GP24, PoE detect
GP23, VSYS ADC GP29. All pins, and which ones the Node may use, are in `src/board.rs`.

## Fixed address and the way back

Set a fixed address under **Network** on the Timer's page; it applies at the next restart.
If the address turns out wrong and the Timer cannot be reached: fit a jumper between header
pins 12 (GP9) and 13 (GND) and power it up. With the jumper fitted it ignores the saved
address and uses DHCP for that start; save working settings, then remove the jumper.

The same jumper also ignores the saved **Node pins** for that start and uses the defaults
(one Node on GP4/GP5), for pins the board cannot start with. Tried 2026-11-28: one Node up
on the default pins, the four-Node pins still saved.

## After a crash

A panic no longer halts the Timer. It records where and why in RAM that a restart keeps,
restarts, and the next start reports it as `crash` in `/status` (`{"count", "text",
"safe_pins"}`). If the panic came in the first minute of a run, that next start also runs on
the default Node pins, whatever is saved (`safe_pins`), so a Timer that fails at start-up
still reaches the network; restarting again (the Restart button, an update) goes back to the
saved pins. A panic after a minute of running restarts on the saved pins, so a race carries
on. `src/crash.rs`. This is what found the fault that stopped the first start with more
than one Node, and two faults in itself.

A **hang**, as opposed to a panic, is still not caught: see "Not yet".

## USB serial console (optional)

With the `usb-console` feature (on by default) the USB-C port is a serial port while the
Timer runs: open it in PuTTY or any terminal (speed does not matter), press Enter, type
`help`. Commands: `status`, `net`, `net dhcp`, `net static <address>/<prefix> [gateway]`,
`restart`, `bootsel`. It is a second way back from a wrong fixed address. The OFT Protocol
is not served there.

`cargo build --release --no-default-features` builds firmware with no USB stack at all.
`/info` lists `usb.console` in `capabilities` when the build has it. The USB IDs are the
pid.codes test pair (1209:0001): get real ones before boards go to other people.

## What it does

- `node.rs`: per Node, decodes Node-link frames (COBS + CRC-16) into `oft_core::Hosts`, sends
  its `Plan` and `Ping`; pulses SYNC at 20 Hz; restarts the C5s and reaches their
  bootloaders over the link (`Reset`, `Dfu`), or with EN/BOOT where those are wired.
  `piolink.rs` is the serial link for Nodes 3 and 4.
- `state.rs`: shared state behind a critical-section mutex; 64-crossing replay buffer; a 100 Hz,
  30 s trace per seat for `/samples`; a pub/sub channel feeding WebSocket clients.
- `web.rs`: picoserve HTTP + WebSocket: `/oft/v1/{info,config,status,time,crossings,samples,
  stream}`. Conformance level **L2** plus reduced-rate history (the hello advertises what it keeps).
- `settings.rs`: saved settings in two flash sectors after the firmware slots (kept across
  updates; a save always goes to the older copy, so a power cut keeps the previous one).
  `POST`/`DELETE /oft/v1/config/save` for the power-on config, `GET`/`PUT /oft/v1/network`
  for DHCP or a fixed address, `POST /oft/v1/restart`. A save stops timing for about 50 ms.
- Node firmware through the Timer: `POST /oft/v1/fw/node?offset=65536` with the Node's
  `firmware.bin` (or the bootloader at 0x2000, the partition table at 0x8000). The Timer
  stores and checks the file, restarts the C5 into its ROM bootloader (`Dfu` over the link;
  EN and BOOT as the fallback), writes it over the Node link's UART, compares an MD5 and
  starts the Node again (over the link: the bootloader is told to clear the C5's
  force-download request and arm the chip's watchdog). Run on hardware 2026-11-27 with no
  EN or BOOT wires: two writes of the 701 KiB firmware, 114 s at 115200 baud and 52 s at
  921600, both checked and both Nodes back on SYNC. Since then, on four Nodes and both
  kinds of link: 77 s at 115200 and 17 to 23 s at 921600 with no retries (first contact is
  always at 115200; the faster rate is set by command, which makes it exact). **A Node in
  its bootloader that hears the Timer's ordinary 921600-baud traffic before it is called
  settles on the wrong rate** and answers nothing at 115200; it looks dead but is not. The
  Timer now calls such a Node at the link's rate (it has to hear a whole SYNC there before
  it takes a command), tells it to move to 115200, and carries on; that rescued Nodes that
  had sat silent for minutes.
  **By hand**, for a Node whose firmware does not answer (tried 2026-11-28): hold its BOOT
  button, tap RESET, release BOOT; upload from the Timer's page; then tap RESET once more.
  The last tap is needed because the chip keeps the BOOT strap it read at the hardware
  reset, and the Timer's restart of it is not one that reads the strap again. **If a write fails, the Node is left in its
  bootloader** (not restarted into partly written firmware), and the next upload finds it
  there and carries on; that retry is not yet run on hardware. Power lost in that state
  still needs a hand: BOOT and RESET on the Node, then upload again, or its USB port.
  The lasting fix is two firmware slots on the Node (planned) (`fwimage/src/esprom.rs` has the
  protocol, host-tested). About two minutes for the firmware at 115200 baud. The C5's
  bootloader loses an answer every few seconds for a reason not yet found; the Timer starts
  a new write session at that point each time. Progress and a diagnosis at `GET`.
- `mdns.rs`: announces `<id>._oft._tcp.local.` with the same TXT keys as `oftd` (`id`, `fw`,
  `proto`, `seats`, `nodes`, `level`) and answers for `<id>.local`, so
  `http://oft-a867.local:7270/` works where the client resolves `.local` names. The
  messages are built in `fwimage/src/mdns.rs` (host-tested). IPv4 only, no name-conflict
  probing (the name comes from the chip id).
- `netdev.rs`: nothing is queued for sending while the Ethernet link is down. Without this
  the W6300's send buffer fills while unplugged and the cable is never noticed again.
- `fw.rs`: firmware update over Ethernet on the boot ROM's A/B partitions: `POST /oft/v1/fw/timer`,
  the trial after an update, the `fw.update` capability (`self`).

## Not yet

- **A watchdog in normal running.** A hang (the tasks stop, no panic) still leaves the
  Timer off the network, Ethernet lights on, until its RESET button is pressed. Seen
  2026-11-21 (during a firmware upload) and once on 2026-11-27 (about 90 s after a failed
  Node write, cause not found). The watchdog is written and switched off
  (`crash::WATCHDOG_ENABLED`). What the attempt found, 2026-11-28:
  - `embassy_rp::watchdog::Watchdog::start` (0.10) picks what the watchdog resets with the
    RP2040's list, which on the RP2350 leaves the processors out. `crash::watchdog_start`
    uses the pico-sdk's value for the RP2350 instead.
  - The boot ROM switches the watchdog off when it confirms an update (`explicit_buy`), so
    a freshly updated Timer has none; `crash::feed` switches it back on.
  - With the watchdog on, two firmware uploads out of two stopped the Timer part way, and a
    deliberate hang was not followed by a restart. Not understood. It needs a debug probe:
    each blind try costs a walk to the RESET button.
- **Two firmware slots on the Node**, so that a failed or interrupted Node write leaves the
  old firmware running. Today there is one slot: a write that fails leaves the Node waiting
  in its bootloader for another attempt, but power lost in that state needs a hand on the
  Node (BOOT and RESET, then upload again, or its USB port). The Node should describe its
  own flash layout to the Timer, so the Timer does not have to know it per chip.
- The web server refused one connection during a burst of requests (2026-11-27) and
  carried on; not looked into.
- Status LED.
- Renaming the Timer. Today `/info` says "OFT Timer" for every Timer, and the id
  (`oft-a867`) comes from the chip. A name set from the page and the USB console, saved
  with the other settings, should show in `/info`, in the page title and in the mDNS
  announcement (as the instance name, and as `<name>.local` where it makes a valid host
  name), so race software and people can tell Timers apart. The id stays fixed: clients
  key on it.
- First-boot checks: SPI clock (starts at 30 MHz; Waveshare runs 70), UART pins vs the
  header silkscreen, whether the C5's BOOT strap is GPIO28 on the XIAO.

GPL-3.0-or-later.

## Acknowledgements

OFT is inspired by [FPVGate](https://github.com/LouisHitchcock/FPVGateC5MK), which showed
that an ESP32-C5 can time laps, and builds on [widscan58](https://github.com/CapnBry/widscan58),
esp-sdr, C5VRX and RotorHazard. The full list is in the
[oft README](https://github.com/Open-FPV-Timer/oft#acknowledgements).
