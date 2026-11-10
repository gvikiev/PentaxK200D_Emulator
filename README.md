# PentaxK200D_Emulator

An ESP32-S3 firmware that emulates a Pentax K200D DSLR's USB personality:
it presents itself as a USB Mass Storage device and responds to the
camera's vendor-specific SCSI protocol (opcode `0xF0`), the same way a real
K200D does when its `[USB Connection]` menu is set to `[PC]`. Built to test
the [PentaxHub](https://github.com) firmware (the ESP32-S3 USB-host project
this is a companion to) without needing the real camera attached.

## What it emulates

- USB Mass Storage / Bulk-Only Transport enumeration (VID `0x0A17`, a
  Pentax-assigned ID; PID `0x0093`).
- The full vendor-SCSI command set the real firmware actually uses:
  connect-mode handshake, the 8-byte quick status and the 408-byte "get
  full status" buffer, `[18 xx]` parameter writes (shutter/aperture/ISO/EV
  comp/exposure mode/AF mode&area&point/white balance/metering/flash/JPEG
  quality&resolution/file format/RAW format/custom image/drive mode),
  shutter release and half-press AF, green button, dust removal, tether
  connect, bulb start/stop, and the RAM-buffer download path (buffer
  select/status/delete, segment info/next, download prepare + streamed
  result bytes) across all 10 RAM buffer slots.
- Parameter writes are validated against the same bounds the real
  PentaxHub firmware itself enforces, and reflected back into the 408-byte
  status buffer at the documented offsets, so a set-then-read-back round
  trip actually proves something.

See `main/pentax_cam_model.h`/`.c` for the exact command table and status
buffer layout (cross-referenced against the PentaxHub project's own
`main/pentax_status.h`/`.c` and `main/pentax_service.c`, which are the
authoritative source for every value there).

## What it deliberately doesn't do

- **One virtual photo.** Every RAM buffer slot serves the same tiny
  (2x2) but structurally valid JPEG. Good enough to exercise the
  segment/download/READ_RESULT protocol end-to-end; not a source of
  realistic photo content.
- **SD card access is a stub.** The real firmware also probes the
  camera's SD card over standard SCSI READ(10) -- itself flagged as
  unverified on real hardware in the PentaxHub project's own architecture
  doc (hypothesis H-1). `tud_msc_read10_cb`/`capacity_cb` here answer with
  a small (32 KiB), read-only, zero-filled fake block device so that path
  gets a well-formed response instead of stalling the whole BOT channel --
  it is not a real filesystem image.
- Fields the architecture doc itself flags as unconfirmed on real hardware
  (exposure_mode's encoding, af_area_select's encoding, the green button's
  exact post-reset exposure_mode value, etc.) are emulated with a
  plausible stand-in, not a verified-correct one.

## Why `tud_msc_scsi_cb` needed a workaround

This project implements the TinyUSB MSC class callbacks itself
(`tud_msc_scsi_cb` and friends, in `main/usb_device.c`) instead of using
`esp_tinyusb`'s higher-level storage-backed MSC driver
(`tinyusb_msc_install_driver()`/`tinyusb_msc_new_storage_*()`), because
that higher-level driver owns those exact callback names itself (around
real SPI-flash/SD/blockdev storage) with no extension point for a custom
vendor SCSI command. `CONFIG_TINYUSB_MSC_ENABLED` stays **off** in
`sdkconfig.defaults` so `esp_tinyusb`'s own implementation (`tinyusb_msc.c`)
never gets compiled in and collides with ours -- but that Kconfig option is
also what `esp_tinyusb`'s `tusb_config.h` uses to decide whether TinyUSB's
own core MSC class driver gets built at all. `main/tusb_config.h` works
around that: it `#include_next`s `esp_tinyusb`'s real config and overrides
just the one macro that matters (`CFG_TUD_MSC`), wired onto the tinyusb
library's own include path ahead of `esp_tinyusb`'s copy via
`main/CMakeLists.txt`. See the comments in both files for the full story --
**don't "simplify" this by turning `CONFIG_TINYUSB_MSC_ENABLED` on**; that
reintroduces the exact symbol collision this works around, and the
project won't link.

## Building

Targets ESP-IDF v5.5 on ESP32-S3, same as the PentaxHub project (see its
own `.devcontainer`, or build locally if you have the toolchain):

```bash
idf.py set-target esp32s3
idf.py -p PORT flash monitor
```

## Testing

`host_tests/` unit-tests the protocol state machine (`pentax_cam_model.c`)
directly -- it has zero TinyUSB/ESP-IDF dependency by design, so these run
with any C99 compiler, no hardware or even a build of the firmware itself
required:

```bash
sh host_tests/run.sh
```

This covers the full command table above, including fault injection
(argument-length mismatches, out-of-range parameter values, selecting an
empty or out-of-range RAM buffer slot, a download prepared against a bad
address) and a chunked-download test that mirrors how the real firmware's
`DL_CHUNK`-sized reads actually loop.

What these tests can't cover: whether a *real* USB host (the PentaxHub
firmware, or a PC) actually enumerates and talks to this device correctly
over real wire -- that still needs the device flashed and plugged in.

## Code layout

| File | Purpose |
|---|---|
| `main/pentax_cam_model.h`/`.c` | The protocol/state machine itself: pure C, no hardware dependency, host-testable. |
| `main/usb_device.c` | TinyUSB glue: descriptors, the mandatory MSC callbacks, the fake read-only block device, and the `tud_msc_scsi_cb` framing that hands opcode `0xF0` traffic to `pentax_cam_model`. |
| `main/tusb_config.h` | The `CFG_TUD_MSC` override described above. |
| `host_tests/` | Host-side unit tests for `pentax_cam_model.c`. |
