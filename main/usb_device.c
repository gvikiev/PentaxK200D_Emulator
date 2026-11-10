#include "usb_device.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

#include "pentax_cam_model.h"

static const char *TAG = "USB_DEVICE";

/* Monotonic sequence for vendor SCSI requests (debug log correlation only). */
static uint32_t s_scsi_sequence = 0;

static uint32_t next_scsi_sequence(void)
{
    return ++s_scsi_sequence;
}

static uint64_t log_timestamp_ms(void)
{
    return (uint64_t)esp_timer_get_time() / 1000ULL;
}

#define PENTAX_SCSI_OPCODE        0xF0u
#define PENTAX_T_SEND_COMMAND     0x24u
#define PENTAX_T_GET_STATUS       0x26u
#define PENTAX_T_SEND_ARGS        0x4Fu
#define PENTAX_T_READ_RESULT      0x49u

/* The single camera-model instance this device emulates. */
static pentax_cam_model_t s_cam;

/* =========================================================
 * Virtual JPEG served for every RAM buffer slot.
 *
 * This is a deliberately tiny (but structurally valid) 2x2 JPEG: enough to
 * exercise the download/segment/READ_RESULT protocol end-to-end without
 * needing a real K200D-sized photo. A real multi-MB file could be supplied
 * through another storage source later without changing the transport
 * code in pentax_cam_model.c or below.
 * ========================================================= */
static const uint8_t s_test_jpeg[] = {
    0xFF, 0xD8,

    /* APP0 / JFIF */
    0xFF, 0xE0,
    0x00, 0x10,
    0x4A, 0x46, 0x49, 0x46, 0x00,
    0x01, 0x01,
    0x00,
    0x00, 0x01,
    0x00, 0x01,
    0x00, 0x00,

    /* DQT */
    0xFF, 0xDB,
    0x00, 0x43,
    0x00,

    0x08, 0x06, 0x06, 0x07, 0x06, 0x05, 0x08, 0x07,
    0x07, 0x07, 0x09, 0x0B, 0x0C, 0x0A, 0x0C, 0x0C,
    0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
    0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
    0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
    0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
    0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,
    0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C, 0x0C,

    /* SOF0 */
    0xFF, 0xC0,
    0x00, 0x11,
    0x08,
    0x00, 0x02, /* height = 2 */
    0x00, 0x02, /* width = 2 */
    0x03,
    0x01, 0x11, 0x00,
    0x02, 0x11, 0x01,
    0x03, 0x11, 0x01,

    /* DHT */
    0xFF, 0xC4,
    0x00, 0x1F,
    0x00,
    0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,

    /* DHT AC */
    0xFF, 0xC4,
    0x00, 0xB5,
    0x10,
    0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04,
    0x03, 0x05, 0x05, 0x04, 0x04, 0x00, 0x00,
    0x01, 0x7D,
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
    0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xA1, 0x08,
    0x23, 0x42, 0xB1, 0xC1, 0x15, 0x52, 0xD1, 0xF0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0A, 0x16,
    0x17, 0x18, 0x19, 0x1A, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2A, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3A, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6A, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
    0x7A, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8A, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
    0x99, 0x9A, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7,
    0xA8, 0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,
    0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4, 0xC5,
    0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2, 0xD3, 0xD4,
    0xD5, 0xD6, 0xD7, 0xD8, 0xD9, 0xDA, 0xE1, 0xE2,
    0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA,
    0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF7, 0xF8,
    0xF9, 0xFA,

    /* SOS */
    0xFF, 0xDA,
    0x00, 0x0C,
    0x03,
    0x01, 0x00,
    0x02, 0x11,
    0x03, 0x11,
    0x00,
    0x3F,
    0x00,

    /* compressed image data */
    0xF8,
    0xA2, 0x8A, 0x28, 0xA0,

    /* EOI */
    0xFF, 0xD9
};

/* =========================================================
 * Minimal fake block storage backing READ(10)/WRITE(10)/CAPACITY.
 *
 * The real PentaxHub firmware also probes the camera's SD card over
 * standard SCSI READ(10) (main/pentax_sd.c there) -- itself flagged as
 * unverified on real hardware in its own architecture doc (hypothesis
 * H-1). This exists so that path gets a well-formed (if content-free,
 * read-only) response instead of stalling the whole BOT channel; it is
 * NOT a real filesystem image.
 * ========================================================= */
#define FAKE_BLOCK_SIZE  512u
#define FAKE_BLOCK_COUNT 64u /* 32 KiB */

/* =========================================================
 * USB descriptors
 * ========================================================= */

static const tusb_desc_device_t pentax_device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = 0x0A17,
    .idProduct          = 0x0093,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

static const char *pentax_string_descriptors[] = {
    (const char[]){0x09, 0x04},
    "PENTAX Corporation",
    "PENTAX K200D Emulator",
    "K200D-ESP32S3-0001",
    "Pentax MSC Interface"
};

#define PENTAX_CONFIG_TOTAL_LEN \
    (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

static const uint8_t pentax_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, PENTAX_CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_MSC_DESCRIPTOR(0, 4, 0x01, 0x81, 64)
};

/* =========================================================
 * Helpers
 * ========================================================= */

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

/* =========================================================
 * Protocol logging helpers
 * ========================================================= */

static const char *pentax_transfer_name(uint8_t type)
{
    switch (type) {
    case PENTAX_T_SEND_COMMAND: return "SEND_COMMAND";
    case PENTAX_T_GET_STATUS:   return "GET_STATUS";
    case PENTAX_T_SEND_ARGS:    return "SEND_ARGS";
    case PENTAX_T_READ_RESULT:  return "READ_RESULT";
    default:                    return "UNKNOWN";
    }
}

static const char *pentax_command_name(uint8_t group, uint8_t command)
{
    switch (group) {
    case 0x00:
        switch (command) {
        case 0x00: return "CONNECT_MODE";
        case 0x01: return "GET_STATUS";
        case 0x08: return "GET_FULL_STATUS";
        case 0x09: return "DSP_WU_REQ";
        default:   return "UNKNOWN_SYS_COMMAND";
        }
    case 0x02:
        switch (command) {
        case 0x00: return "BUFFER_STATUS";
        case 0x01: return "BUFFER_SELECT";
        case 0x03: return "BUFFER_DELETE";
        default:   return "UNKNOWN_BUFFER_COMMAND";
        }
    case 0x04:
        switch (command) {
        case 0x00: return "SEGMENT_INFO";
        case 0x01: return "SEGMENT_NEXT";
        default:   return "UNKNOWN_SEGMENT_COMMAND";
        }
    case 0x06:
        switch (command) {
        case 0x00: return "DOWNLOAD_PREPARE";
        default:   return "UNKNOWN_DOWNLOAD_COMMAND";
        }
    case 0x10:
        switch (command) {
        case 0x05: return "SHUTTER";
        case 0x07: return "GREEN_BUTTON";
        case 0x0A: return "TETHER_CONNECT";
        case 0x0D: return "BULB";
        case 0x11: return "DUST_REMOVAL";
        default:   return "UNKNOWN_ACTION_COMMAND";
        }
    case 0x18:
        return "PARAM_WRITE";
    default:
        return "UNKNOWN_GROUP";
    }
}

static void log_data_preview(const char *label, const uint8_t *data, uint32_t len)
{
    uint32_t preview_len = (len > 16u) ? 16u : len;
    ESP_LOGI(TAG, "%s: %lu bytes (preview %lu bytes)",
             label, (unsigned long)len, (unsigned long)preview_len);
    if (preview_len > 0u && data != NULL) {
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, data, preview_len, ESP_LOG_INFO);
    }
}

static void log_camera_state(const char *stage, uint32_t seq)
{
    ESP_LOGI(TAG,
             "[%lu] STATE %s: connected=%u mask=0x%08lX selected=%d "
             "segment=%lu download_prepared=%u download_addr=0x%08lX "
             "download_remaining=%lu args_len=%lu result_len=%lu status=0x%02X",
             (unsigned long)seq, stage,
             (unsigned)s_cam.connected,
             (unsigned long)s_cam.buffer_mask,
             (int)s_cam.selected_slot,
             (unsigned long)s_cam.segment_index,
             (unsigned)s_cam.download_prepared,
             (unsigned long)s_cam.download_addr,
             (unsigned long)s_cam.download_length,
             (unsigned long)s_cam.args_len,
             (unsigned long)s_cam.result_len,
             s_cam.command_status);
}

/* =========================================================
 * Pentax vendor SCSI callback (tud_msc_scsi_cb)
 *
 * Framing only -- all actual protocol/state logic lives in
 * pentax_cam_model.c, which has no TinyUSB/ESP-IDF dependency and is
 * exercised directly by host_tests/.
 * ========================================================= */

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16], void *buffer, uint16_t bufsize)
{
    uint32_t seq = next_scsi_sequence();
    uint64_t timestamp = log_timestamp_ms();

    if (scsi_cmd == NULL) {
        ESP_LOGE(TAG, "[%lu] SCSI callback received NULL CDB", (unsigned long)seq);
        return -1;
    }

    if (scsi_cmd[0] != PENTAX_SCSI_OPCODE) {
        ESP_LOGW(TAG, "[%lu][%llu ms] Unexpected opcode in Pentax callback: 0x%02X",
                 (unsigned long)seq, (unsigned long long)timestamp, scsi_cmd[0]);
        tud_msc_set_sense(lun, 0x05, 0x20, 0x00);
        return -1;
    }

    uint8_t type = scsi_cmd[1];

    if (type == PENTAX_T_GET_STATUS) {
        ESP_LOGD(TAG, "[%lu][%llu ms] SCSI RX: lun=%u CDB=%02X %02X %02X %02X %02X %02X %02X %02X bufsize=%u",
                 (unsigned long)seq, (unsigned long long)timestamp, lun,
                 scsi_cmd[0], scsi_cmd[1], scsi_cmd[2], scsi_cmd[3],
                 scsi_cmd[4], scsi_cmd[5], scsi_cmd[6], scsi_cmd[7], bufsize);
    } else {
        ESP_LOGI(TAG, "[%lu][%llu ms] SCSI RX: lun=%u opcode=0x%02X type=0x%02X (%s) "
                 "CDB=%02X %02X %02X %02X %02X %02X %02X %02X bufsize=%u buffer=%p",
                 (unsigned long)seq, (unsigned long long)timestamp, lun,
                 scsi_cmd[0], type, pentax_transfer_name(type),
                 scsi_cmd[0], scsi_cmd[1], scsi_cmd[2], scsi_cmd[3],
                 scsi_cmd[4], scsi_cmd[5], scsi_cmd[6], scsi_cmd[7], bufsize, buffer);
        log_camera_state("BEFORE SCSI", seq);
    }

    /* ----------------------------- SEND ARGUMENTS ----------------------------- */
    if (type == PENTAX_T_SEND_ARGS) {
        uint16_t offset = read_le16(&scsi_cmd[2]);
        uint16_t requested_len = read_le16(&scsi_cmd[4]);

        if (offset != 0 || requested_len != bufsize ||
            requested_len > PENTAX_CAM_MAX_ARGS || (requested_len > 0 && buffer == NULL)) {
            ESP_LOGE(TAG, "[%lu] Invalid argument transfer: offset=%u cdb_len=%u actual_len=%u buffer=%p max=%u",
                     (unsigned long)seq, offset, requested_len, bufsize, buffer, PENTAX_CAM_MAX_ARGS);
            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);
            return -1;
        }

        if (!pentax_cam_model_send_args(&s_cam, (const uint8_t *)buffer, requested_len)) {
            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);
            return -1;
        }

        ESP_LOGI(TAG, "[%lu] ARGUMENTS RX: offset=%u length=%u", (unsigned long)seq, offset, requested_len);
        log_data_preview("Arguments RX", s_cam.args, s_cam.args_len);
        log_camera_state("AFTER ARGUMENTS", seq);
        return 0;
    }

    /* ----------------------------- SEND COMMAND ----------------------------- */
    if (type == PENTAX_T_SEND_COMMAND) {
        if (bufsize != 0) {
            ESP_LOGE(TAG, "[%lu] SEND_COMMAND has unexpected data length=%u", (unsigned long)seq, bufsize);
            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);
            return -1;
        }

        uint8_t group = scsi_cmd[2];
        uint8_t command = scsi_cmd[3];
        uint16_t expected_args_len = read_le16(&scsi_cmd[4]);

        ESP_LOGI(TAG, "[%lu] COMMAND EXEC: %s group=0x%02X command=0x%02X expected_args=%u actual_args=%lu",
                 (unsigned long)seq, pentax_command_name(group, command), group, command,
                 expected_args_len, (unsigned long)s_cam.args_len);

        pentax_cam_model_exec(&s_cam, group, command, expected_args_len);

        ESP_LOGI(TAG, "[%lu][%llu ms] COMMAND RESP: %s status=0x%02X result_len=%lu",
                 (unsigned long)seq, (unsigned long long)log_timestamp_ms(),
                 pentax_command_name(group, command), s_cam.command_status,
                 (unsigned long)s_cam.result_len);
        log_camera_state("AFTER COMMAND", seq);

        if (!s_cam.download_prepared && s_cam.result_len <= sizeof(s_cam.result)) {
            log_data_preview("Command result", s_cam.result, s_cam.result_len);
        }
        return 0;
    }

    /* ----------------------------- GET STATUS ----------------------------- */
    if (type == PENTAX_T_GET_STATUS) {
        if (bufsize < 8 || buffer == NULL) {
            ESP_LOGE(TAG, "[%lu] GET_STATUS invalid buffer: bufsize=%u buffer=%p",
                     (unsigned long)seq, bufsize, buffer);
            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);
            return -1;
        }
        pentax_cam_model_get_status(&s_cam, (uint8_t *)buffer);
        ESP_LOGD(TAG, "[%lu] STATUS TX: result_len=%lu status=0x%02X",
                 (unsigned long)seq, (unsigned long)s_cam.result_len, s_cam.command_status);
        ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, 8, ESP_LOG_DEBUG);
        return 8;
    }

    /* ----------------------------- READ RESULT ----------------------------- */
    if (type == PENTAX_T_READ_RESULT) {
        uint16_t requested_len = read_le16(&scsi_cmd[4]);

        ESP_LOGI(TAG, "[%lu] READ_RESULT RX: requested=%u bufsize=%u prepared=%u addr=0x%08lX remaining=%lu",
                 (unsigned long)seq, requested_len, bufsize,
                 (unsigned)s_cam.download_prepared,
                 (unsigned long)s_cam.download_addr,
                 (unsigned long)s_cam.download_length);

        if (requested_len > bufsize || (requested_len > 0 && buffer == NULL)) {
            ESP_LOGE(TAG, "[%lu] Invalid READ_RESULT transfer: requested=%u bufsize=%u buffer=%p",
                     (unsigned long)seq, requested_len, bufsize, buffer);
            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);
            return -1;
        }

        uint16_t n = pentax_cam_model_read_result(&s_cam, requested_len, (uint8_t *)buffer);

        ESP_LOGI(TAG, "[%lu] READ_RESULT TX: %u bytes", (unsigned long)seq, n);
        log_data_preview("Result TX", (const uint8_t *)buffer, n);
        log_camera_state("AFTER READ_RESULT", seq);
        return n;
    }

    ESP_LOGW(TAG, "[%lu] Unknown Pentax transfer type: 0x%02X", (unsigned long)seq, type);
    tud_msc_set_sense(lun, 0x05, 0x20, 0x00);
    return -1;
}

/* =========================================================
 * Remaining mandatory MSC callbacks.
 *
 * tud_msc_scsi_cb above handles opcode 0xF0 only; everything standard
 * (INQUIRY, TEST UNIT READY, READ CAPACITY, READ/WRITE(10)) is handled by
 * TinyUSB's own MSC class core, which calls these.
 * ========================================================= */

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], uint8_t product_id[16], uint8_t product_rev[4])
{
    (void)lun;
    memcpy(vendor_id, "PENTAX  ", 8);
    memcpy(product_id, "K200D Emulator  ", 16);
    memcpy(product_rev, "1.00", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    return true; /* "media" (the fake block store below) is always present */
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
    (void)lun;
    *block_count = FAKE_BLOCK_COUNT;
    *block_size = FAKE_BLOCK_SIZE;
}

bool tud_msc_is_writable_cb(uint8_t lun)
{
    (void)lun;
    return false; /* the fake block store is read-only */
}

int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    (void)lun;
    if (lba >= FAKE_BLOCK_COUNT) {
        return -1;
    }
    /* Zero-filled: this stub exists so a READ(10) probe (e.g. the real
     * firmware's SD-card browsing, itself unverified on real hardware --
     * see docs/pentax_k200d_architecture_gemeni.md hypothesis H-1) gets a
     * well-formed response instead of stalling the BOT channel. It is not a
     * real filesystem image. */
    memset(buffer, 0, bufsize);
    (void)offset;
    return (int32_t)bufsize;
}

int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    (void)lun; (void)lba; (void)offset; (void)buffer;
    /* Never reached in practice (tud_msc_is_writable_cb() returns false),
     * but TinyUSB requires this symbol to exist regardless. */
    return (int32_t)bufsize;
}

/* =========================================================
 * USB Device Event Handler
 * ========================================================= */

static void usb_device_event_handler(tinyusb_event_t *event, void *arg)
{
    (void)arg;

    switch (event->id) {
    case TINYUSB_EVENT_ATTACHED:
        ESP_LOGI(TAG, "[%llu ms] USB HOST ATTACHED", (unsigned long long)log_timestamp_ms());
        log_camera_state("USB ATTACHED", 0);
        break;

    case TINYUSB_EVENT_DETACHED:
        ESP_LOGW(TAG, "[%llu ms] USB HOST DETACHED", (unsigned long long)log_timestamp_ms());
        log_camera_state("BEFORE DETACH RESET", 0);
        /* Keep captured photos; only connection-related state resets. */
        s_cam.connected = false;
        s_cam.selected_slot = -1;
        s_cam.segment_index = 0;
        s_cam.download_prepared = false;
        s_cam.download_addr = 0;
        s_cam.download_length = 0;
        log_camera_state("AFTER DETACH RESET", 0);
        break;

    default:
        ESP_LOGI(TAG, "USB event: %d", event->id);
        break;
    }
}

/* =========================================================
 * USB Device Initialization
 * ========================================================= */

esp_err_t usb_device_init(void)
{
    ESP_LOGI(TAG, "[%llu ms] Initializing Pentax USB Device...", (unsigned long long)log_timestamp_ms());

    pentax_cam_model_init(&s_cam, s_test_jpeg, sizeof(s_test_jpeg));

    tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(usb_device_event_handler);
    tusb_cfg.descriptor.device = &pentax_device_descriptor;
    tusb_cfg.descriptor.string = pentax_string_descriptors;
    tusb_cfg.descriptor.string_count = sizeof(pentax_string_descriptors) / sizeof(pentax_string_descriptors[0]);
    tusb_cfg.descriptor.full_speed_config = pentax_configuration_descriptor;

    esp_err_t err = tinyusb_driver_install(&tusb_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB initialization failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Pentax USB Device initialized");
    ESP_LOGI(TAG, "VID: 0x%04X, PID: 0x%04X", pentax_device_descriptor.idVendor, pentax_device_descriptor.idProduct);
    ESP_LOGI(TAG, "USB Interface: MSC / SCSI / BOT");
    ESP_LOGI(TAG, "Virtual file: addr=0x%08lX size=%lu bytes",
             (unsigned long)s_cam.file_addr_base, (unsigned long)s_cam.file_size);

    return ESP_OK;
}
