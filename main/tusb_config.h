/*
 * Overrides just two macros from esp_tinyusb's own tusb_config.h (chained
 * to below via #include_next -- see main/CMakeLists.txt for how this file
 * gets priority on the tinyusb library's include path).
 *
 * esp_tinyusb derives CFG_TUD_MSC straight from CONFIG_TINYUSB_MSC_ENABLED,
 * but that Kconfig option also decides whether esp_tinyusb's own
 * storage-backed tud_msc_*_cb implementations (tinyusb_msc.c) get compiled
 * -- and this project defines those callbacks itself (in usb_device.c) to
 * serve the Pentax vendor-SCSI protocol, which would collide with
 * tinyusb_msc.c's versions. So CONFIG_TINYUSB_MSC_ENABLED stays off, and
 * CFG_TUD_MSC is forced on here instead, giving TinyUSB's own MSC class
 * driver (src/class/msc/msc_device.c) without pulling in tinyusb_msc.c.
 */
#pragma once

#include_next "tusb_config.h"

#undef CFG_TUD_MSC
#define CFG_TUD_MSC 1

/* Must be >= the largest single vendor-SCSI data-stage transfer this
 * project ever serves in one tud_msc_scsi_cb() call. The real PentaxHub
 * firmware's READ_RESULT download chunks are up to 8192 bytes (DL_CHUNK in
 * its main/pentax_service.c); TinyUSB caps a non-READ10/WRITE10 command's
 * single data-stage transfer at CFG_TUD_MSC_EP_BUFSIZE (see msc_device.c),
 * so anything smaller would silently truncate a real download chunk. */
#define CFG_TUD_MSC_EP_BUFSIZE 8192
