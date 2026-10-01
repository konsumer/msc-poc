# msc-poc — USB mass storage proof-of-concept

[![build](https://github.com/konsumer/msc-poc/actions/workflows/build.yml/badge.svg)](https://github.com/konsumer/msc-poc/actions/workflows/build.yml)

Minimal ESP32-S2/S3 firmware that does exactly one thing: mount the SD card and
present it to a USB host as a removable SCSI disk.

It exists to separate "the device can act as a USB drive" from everything else
in [Launcher](https://github.com/bmorcelli/Launcher). The USB descriptor and MSC
callbacks are copied verbatim from Launcher's `src/massStorage.cpp`, including:

- **PR #297** — `bDeviceClass`/`bDeviceSubClass`/`bDeviceProtocol` are `0x00`
  (class described at interface level). This is what makes macOS enumerate the
  device as storage.
- **PR #424** — READ10/WRITE10 handle non-zero byte offsets and partial sectors
  instead of rejecting them. macOS issues those during mount and probe.

Nothing else from Launcher is present: no display init, no menus, no keyboard,
no WiFi, no OTA, no WebUI. If this firmware mounts on a host and Launcher does
not, the bug is in Launcher's surrounding code or timing — not in the MSC path.
If this firmware also fails, the MSC path itself (or the hardware/board wiring)
is the answer.

Deliberate differences from Launcher:

| | Launcher | msc-poc |
|---|---|---|
| iSerialNumber string | `TAB5-SD` | `MSC-POC` |
| iManufacturer / iProduct / VID / PID | `M5Stack` / `Launcher SD` / `0x303A` / `0x1001` | identical |
| bDeviceClass / config / endpoints | | identical |
| log file | `/Launcher_error.log` | `/msc-poc.log` |

## Status

- Builds clean for all three environments (PlatformIO 55.03.39).
- The transfer maths is covered by `./test/run.sh` (host, ASan/UBSan).
- **Not verified on hardware yet** — no device was available while writing this.
  The USB descriptor, PHY bring-up and per-board SD wiring only run on a real
  board. First run on a Cardputer ADV or T-Deck is the real test; if it does not
  enumerate at all, the log will say so (see below).

## Build and flash

Requires [PlatformIO](https://platformio.org/) (`pip install platformio` or
`brew install platformio`).

```bash
pio run -e cardputer-adv              # build
pio run -e cardputer-adv -t upload    # flash (add -p /dev/ttyACM0 or --upload-port COMx if needed)
pio device monitor                    # optional, only valid before USB mode starts
```

Environments:

| env | device |
|---|---|
| `cardputer` | M5Stack Cardputer |
| `cardputer-adv` | M5Stack Cardputer ADV |
| `t-deck` | LilyGo T-Deck |
| `t-deck-plus` | LilyGo T-Deck Plus |
| `t-deck-pro` | LilyGo T-Deck Pro (e-paper; SD shares the SPI bus with the panel) |

SD pin assignments are taken from Launcher's board configs for each device.

Prebuilt factory images for every environment are attached to each
[release](https://github.com/konsumer/msc-poc/releases) by CI. A factory image
contains bootloader + partitions + app and is flashed at offset `0x0`:

```bash
esptool.py --chip esp32s3 write_flash 0x0 msc-poc-t-deck-plus.bin
```

or with PlatformIO:

```bash
pio run -e t-deck-plus -t upload
```

## Usage

1. Insert a FAT-formatted SD card.
2. Flash, then unplug USB.
3. On devices with a power switch (T-Deck family), **turn power off** before
   plugging USB in — the switch tells the ESP32 to behave as a USB device.
   Cardputer has no switch; just plug in.
4. Plug USB in. The device should appear as a removable drive named after the
   card's volume label.
5. To leave USB mode: eject the volume from the host. The device reboots and
   the HW CDC serial port comes back for reflashing.

The firmware has no display output — it is meant to be diagnosed from the host
side and from the log file it writes.

## The log

`/msc-poc.log` in the SD card root, appended on every run: build/board/SD
geometry header, then periodic blocks with SCSI counters, an opcode histogram
and lifecycle/error events.

Each run starts with a header that identifies the image and the card it saw:

```
==== T-Deck Plus USB MSC PoC v0.1.0 ====
mcu ESP32-S3, sd cs=39 sck=40 miso=38 mosi=41
sd: 62333952 sectors x 512 bytes = 30431 MB
```

The version is the release tag for CI builds (the workflow passes
`POC_VERSION=<tag>`), or `dev` for local builds.

Two rules keep the log from perturbing the thing being tested:

1. **Nothing is written from inside a transfer callback.** Callbacks only bump
   counters and append to a RAM buffer; flushes happen from the same task that
   services TinyUSB, so log writes can never interleave with raw sector
   reads/writes of the exported volume.
2. **Nothing is written while the host has the volume mounted.** The host caches
   FAT and directory sectors, and writing the file behind its back could corrupt
   or confuse the filesystem under test. The log is flushed when the host is not
   using the card — which is the normal state in every failure case here — and
   forced out when the host sends an eject.

Consequence: if the device mounts successfully and you never eject it, that
session is not written. Eject (or power-cycle and re-run) before pulling the
card to get the full log.

`READ10`/`WRITE10` are aggregated rather than logged per call — a host FAT scan
can issue thousands. Everything else is counted per opcode.

### Reading it

```
---- flush 12345 (idle) ----
counters: read=812 (415744 B, lba 0..2047) write=0 (0 B, lba 0..0) errors=0
scsi 0x00: 2      # TEST_UNIT_READY
scsi 0x03: 1      # REQUEST_SENSE
scsi 0x12: 1      # INQUIRY
scsi 0x1A: 1      # MODE_SENSE_6
scsi 0x1E: 1      # PREVENT_ALLOW_MEDIUM_REMOVAL
scsi 0x25: 1      # READ_CAPACITY(10)
scsi 0x35: 3      # SYNCHRONIZE_CACHE(10)
[    4210] host mounted (configured)
[    4212] INQUIRY
```

| symptom in the log | meaning |
|---|---|
| file exists, has header, no flush blocks, no `host mounted` | host never configured the device: USB-level problem (cable, power switch, port, or host USB stack) |
| `host mounted` but zero `scsi` lines | host configured the device but never talked SCSI to it: descriptor/interface problem |
| `scsi_cb: unsupported opcode 0xNN` | host requested a command the firmware refuses — this is the shape of the pre-#424 bug |
| `readRAW failed` / `writeRAW failed` / `bad sector size` | SD layer is failing mid-transfer, not a USB problem |
| `read=`/`write=` counters climb and no errors | bulk transfers work; if the host still will not mount, the problem is in the filesystem/host mount path, not here |
| no file at all | SD never mounted (firmware halts on serial with `SD mount failed`), or the device never started USB |

An empty `scsi` histogram with a `host mounted` line is the single most useful
signal available: it proves the USB device enumerated and proves the host never
issued a single storage command.

## Tests

`src/msc_transfer.h` holds the sector/offset maths of the read/write callbacks
and has no Arduino or ESP-IDF dependency, so it is tested on the host:

```bash
./test/run.sh
```

This compiles `test/host_test.cpp` with ASan/UBSan and checks the transfer
arithmetic against an independent byte-offset model of a fake card: aligned and
partial sectors, transfers that cross sector boundaries, read-modify-write byte
preservation, buffer-canary checks, device failure propagation and geometry
rejection. Emulating the pre-#424 behaviour (rejecting non-aligned transfers)
makes it fail, which is exactly the regression it exists to catch.

Nothing else in this firmware can be tested without hardware: the USB
descriptor, PHY bring-up and SD wiring are only exercised on a device.

## Reporting a failure

Collect from the host (macOS):

```bash
system_profiler SPUSBDataType | grep -B2 -A20 -i "303a\|MSC-POC"
log show --last 10m --predicate 'eventMessage CONTAINS "USBMSC"' --style compact
sudo log show --last 10m --predicate 'subsystem == "com.apple.iokit.IOUSBMassStorageDriver"' --style compact
sudo log show --last 10m --predicate 'process == "diskarbitrationd"' --style compact
diskutil list
```

and attach `/msc-poc.log` from the SD card.
