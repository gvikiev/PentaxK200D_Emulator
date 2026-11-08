#include "usb_device.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tusb.h"

static const char *TAG = "USB_DEVICE";

/* =========================================================
 * Pentax vendor-specific SCSI commands
 * ========================================================= */

#define PENTAX_SCSI_OPCODE        0xF0

#define PENTAX_T_SEND_COMMAND     0x24
#define PENTAX_T_GET_STATUS       0x26
#define PENTAX_T_SEND_ARGS        0x4F
#define PENTAX_T_READ_RESULT      0x49


/* =========================================================
 * Pentax command groups / commands
 * ========================================================= */

#define PENTAX_G_SYS              0x00
#define PENTAX_C_CONNECT_MODE     0x00
#define PENTAX_C_GET_STATUS       0x01

#define PENTAX_G_BUFFER           0x02
#define PENTAX_C_BUFFER_STATUS    0x00
#define PENTAX_C_BUFFER_SELECT    0x01
#define PENTAX_C_BUFFER_DELETE    0x03

#define PENTAX_G_SEGMENT          0x04
#define PENTAX_C_SEGMENT_INFO     0x00
#define PENTAX_C_SEGMENT_NEXT     0x01

#define PENTAX_G_ACTION           0x10
#define PENTAX_C_SHUTTER          0x05

#define PENTAX_G_DOWNLOAD         0x06
#define PENTAX_C_DOWNLOAD_PREPARE 0x00


/* =========================================================
 * Pentax status values
 * ========================================================= */

#define PENTAX_ST_READY           0x01
#define PENTAX_ST_OK              0x01
#define PENTAX_ST_NOT_AVAILABLE   0x81
#define PENTAX_ST_ILLEGAL_PARAM   0x83


/* =========================================================
 * Segment types
 * ========================================================= */

#define PENTAX_SEG_LAST            2
#define PENTAX_SEG_DATA            3
#define PENTAX_SEG_FILE_LEN        4


/* =========================================================
 * Limits
 * ========================================================= */

#define PENTAX_MAX_ARGS            32

/*
 * Result buffer is only for command responses.
 *
 * It is NOT the photo storage.
 *
 * The actual file is represented separately by
 * s_file_data / s_file_size.
 */
#define PENTAX_MAX_RESULT          512


/* =========================================================
 * Virtual file address space
 * ========================================================= */

#define PENTAX_JPEG_ADDR            0x10000000u


/* =========================================================
 * Emulator state
 * ========================================================= */

static uint8_t  s_args[PENTAX_MAX_ARGS];
static uint32_t s_args_len;

static uint8_t  s_result[PENTAX_MAX_RESULT];
static uint32_t s_result_len;

static uint32_t s_buffer_mask = 0;

static uint8_t  s_command_status = PENTAX_ST_OK;

static bool s_connected = false;
static bool s_buffer_selected = false;
static bool s_download_prepared = false;


/*
 * Segment state.
 *
 * Segment 0:
 *     DATA
 *
 * Segment 1:
 *     LAST
 */
static uint32_t s_segment_index = 0;


/*
 * Current download operation.
 */
static uint32_t s_download_addr = 0;
static uint32_t s_download_length = 0;


/* =========================================================
 * Virtual JPEG
 *
 * Small valid JPEG used while testing the protocol.
 *
 * Later this will be replaced by an actual K200D-sized
 * JPEG / PEF / DNG source.
 * ========================================================= */

/*
 * 2x2 JPEG.
 *
 * This is deliberately kept small because the current
 * TinyUSB MSC buffer is 512 bytes.
 *
 * IMPORTANT:
 *
 * The transport architecture below does NOT depend on
 * this file being small.
 *
 * A real multi-MB file can later be supplied through
 * another storage source.
 */
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

    /* height = 2 */
    0x00, 0x02,

    /* width = 2 */
    0x00, 0x02,

    /* components = 3 */
    0x03,

    /* Y */
    0x01, 0x11, 0x00,

    /* Cb */
    0x02, 0x11, 0x01,

    /* Cr */
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


/*
 * Current virtual file.
 *
 * The important point is that this is now abstracted from
 * the Pentax command handling.
 */
static const uint8_t *s_file_data = s_test_jpeg;
static uint32_t s_file_size = sizeof(s_test_jpeg);


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


/* =========================================================
 * USB strings
 * ========================================================= */

static const char *pentax_string_descriptors[] = {
    (const char[]){0x09, 0x04},
    "PENTAX Corporation",
    "PENTAX K200D Emulator",
    "K200D-ESP32S3-0001",
    "Pentax MSC Interface"
};


/* =========================================================
 * USB configuration descriptor
 * ========================================================= */

#define PENTAX_CONFIG_TOTAL_LEN \
    (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

static const uint8_t pentax_configuration_descriptor[] = {

    TUD_CONFIG_DESCRIPTOR(
        1,
        1,
        0,
        PENTAX_CONFIG_TOTAL_LEN,
        0x00,
        100
    ),

    TUD_MSC_DESCRIPTOR(
        0,
        4,
        0x01,
        0x81,
        64
    )
};


/* =========================================================
 * Helpers
 * ========================================================= */

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] |
           ((uint16_t)data[1] << 8);
}


static uint32_t read_be32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           (uint32_t)data[3];
}


static void write_be32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value >> 24);
    data[1] = (uint8_t)(value >> 16);
    data[2] = (uint8_t)(value >> 8);
    data[3] = (uint8_t)value;
}


static void clear_result(void)
{
    memset(s_result, 0, sizeof(s_result));
    s_result_len = 0;
}


static void set_command_status(uint8_t status)
{
    s_command_status = status;
}


/*
 * Reset state related to current download.
 */
static void reset_download_state(void)
{
    s_segment_index = 0;

    s_download_addr = 0;
    s_download_length = 0;

    s_download_prepared = false;
}


/*
 * Reset virtual camera state.
 */
static void reset_camera_state(void)
{
    s_buffer_mask = 0;

    s_connected = false;
    s_buffer_selected = false;

    reset_download_state();

    clear_result();
    set_command_status(PENTAX_ST_OK);
}


/* =========================================================
 * Command: SYSTEM
 * ========================================================= */

static void process_system_command(uint8_t command)
{
    if (command == PENTAX_C_CONNECT_MODE) {

        ESP_LOGI(TAG, "Connect mode");

        if (s_args_len != 4) {
            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }

        uint32_t mode = read_be32(s_args);

        ESP_LOGI(TAG,
                 "Connect mode value=%lu",
                 (unsigned long)mode);

        if (mode != 0 && mode != 1) {
            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }

        s_connected = (mode != 0);

        ESP_LOGI(TAG,
                 "Camera connected=%s",
                 s_connected ? "true" : "false");

        return;
    }


    if (command == PENTAX_C_GET_STATUS) {

        ESP_LOGI(TAG, "Get camera status");

        /*
         * Return a realistic fixed-size status block.
         *
         * Current Host does not parse this block, but returning
         * 28 bytes is closer to the real protocol than zero.
         */
        memset(s_result, 0, sizeof(s_result));

        /*
         * Example status fields.
         *
         * We deliberately keep them conservative because the
         * current Host only checks command success.
         */
        write_be32(&s_result[0], 0x00000001);
        write_be32(&s_result[4], s_buffer_mask);

        s_result_len = 28;

        return;
    }


    set_command_status(PENTAX_ST_NOT_AVAILABLE);
}


/* =========================================================
 * Command: BUFFER
 * ========================================================= */

static void process_buffer_command(uint8_t command)
{
    /*
     * BUFFER STATUS
     *
     * 02/00
     */
    if (command == PENTAX_C_BUFFER_STATUS) {

        ESP_LOGI(TAG,
                 "Buffer status: mask=0x%08lX",
                 (unsigned long)s_buffer_mask);

        write_be32(s_result, s_buffer_mask);

        s_result_len = 4;

        return;
    }


    /*
     * BUFFER SELECT
     *
     * Host sends:
     *
     *     [0] = buffer number
     *     [1] = buffer type
     *     [2] = buffer resolution
     *
     * Each is BE32.
     *
     * Total = 12 bytes.
     */
    if (command == PENTAX_C_BUFFER_SELECT) {

        if (s_args_len != 12) {
            ESP_LOGE(TAG,
                     "Buffer select: invalid args length=%lu",
                     (unsigned long)s_args_len);

            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }

        uint32_t buffer = read_be32(&s_args[0]);
        uint32_t type   = read_be32(&s_args[4]);
        uint32_t res    = read_be32(&s_args[8]);

        if (buffer != 0) {
             ESP_LOGE(TAG,
             "Buffer select: invalid buffer index=%lu",
             (unsigned long)buffer);

            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }

        ESP_LOGI(TAG,
                 "Buffer select: buffer=%lu type=%lu res=%lu",
                 (unsigned long)buffer,
                 (unsigned long)type,
                 (unsigned long)res);

        if ((s_buffer_mask & 0x00000001u) == 0) {

            ESP_LOGW(TAG, "No image available");

            set_command_status(PENTAX_ST_NOT_AVAILABLE);
            return;
        }

        s_buffer_selected = true;

        reset_download_state();

        ESP_LOGI(TAG, "Buffer selected");

        return;
    }


    /*
     * BUFFER DELETE
     *
     * 02/03
     *
     * Host sends BE32 buffer number.
     */
    if (command == PENTAX_C_BUFFER_DELETE) {

        if (s_args_len != 4) {

            ESP_LOGE(TAG,
                     "Buffer delete: invalid args length=%lu",
                     (unsigned long)s_args_len);

            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }

        uint32_t buffer = read_be32(s_args);

        ESP_LOGI(TAG,
                 "Buffer delete: buffer=%lu",
                 (unsigned long)buffer);

        /*
         * We have one virtual image in buffer 0.
         */
        if (buffer != 0) {
            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }

        s_buffer_mask &= ~0x00000001u;

        s_buffer_selected = false;

        reset_download_state();

        ESP_LOGI(TAG,
                 "Virtual image deleted. mask=0x%08lX",
                 (unsigned long)s_buffer_mask);

        return;
    }


    set_command_status(PENTAX_ST_NOT_AVAILABLE);
}


/* =========================================================
 * Command: SEGMENT
 * ========================================================= */

static void process_segment_command(uint8_t command)
{
    /*
     * SEGMENT INFO
     *
     * 04/00
     *
     * Result:
     *
     *   +0  reserved
     *   +4  segment type
     *   +8  virtual address
     *   +12 segment length
     *
     * Host currently parses:
     *
     *   type   = result[4]
     *   addr   = result[8]
     *   length = result[12]
     */
    if (command == PENTAX_C_SEGMENT_INFO) {

        if (!s_buffer_selected ||
            (s_buffer_mask & 0x00000001u) == 0) {

            ESP_LOGW(TAG,
                     "Segment info requested without selected buffer");

            set_command_status(PENTAX_ST_NOT_AVAILABLE);
            return;
        }


        memset(s_result, 0, sizeof(s_result));


        if (s_segment_index == 0) {

            /*
             * First segment = image data.
             */
            write_be32(&s_result[4], PENTAX_SEG_DATA);

            write_be32(&s_result[8], PENTAX_JPEG_ADDR);

            write_be32(&s_result[12], s_file_size);

            s_result_len = 16;

            ESP_LOGI(TAG,
                     "Segment INFO: DATA addr=0x%08lX len=%lu",
                     (unsigned long)PENTAX_JPEG_ADDR,
                     (unsigned long)s_file_size);

            return;
        }


        /*
         * After DATA comes LAST.
         */
        if (s_segment_index == 1) {

            write_be32(&s_result[4], PENTAX_SEG_LAST);

            write_be32(&s_result[8], 0);
            write_be32(&s_result[12], 0);

            s_result_len = 16;

            ESP_LOGI(TAG, "Segment INFO: LAST");

            return;
        }


        set_command_status(PENTAX_ST_NOT_AVAILABLE);

        return;
    }


    /*
     * SEGMENT NEXT
     *
     * 04/01
     */
    if (command == PENTAX_C_SEGMENT_NEXT) {

        if (s_args_len != 4) {

            ESP_LOGE(TAG,
                     "Segment next: invalid args length=%lu",
                     (unsigned long)s_args_len);

            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }

        uint32_t value = read_be32(s_args);

        ESP_LOGI(TAG,
                 "Segment next: value=%lu",
                 (unsigned long)value);

        if (!s_buffer_selected) {
            set_command_status(PENTAX_ST_NOT_AVAILABLE);
            return;
        }

        if (s_segment_index < 1) {
            s_segment_index++;
        }

        return;
    }


    set_command_status(PENTAX_ST_NOT_AVAILABLE);
}


/* =========================================================
 * Command: DOWNLOAD
 * ========================================================= */

static void process_download_command(uint8_t command)
{
    /*
     * DOWNLOAD PREPARE
     *
     * 06/00
     *
     * Arguments:
     *
     *   BE32 address
     *   BE32 length
     */
    if (command == PENTAX_C_DOWNLOAD_PREPARE) {

        if (s_args_len != 8) {

            ESP_LOGE(TAG,
                     "Download prepare: invalid args length=%lu",
                     (unsigned long)s_args_len);

            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }


        if (!s_buffer_selected ||
            (s_buffer_mask & 0x00000001u) == 0) {

            ESP_LOGW(TAG,
                     "Download prepare without selected image");

            set_command_status(PENTAX_ST_NOT_AVAILABLE);
            return;
        }


        uint32_t addr = read_be32(&s_args[0]);
        uint32_t len  = read_be32(&s_args[4]);


        ESP_LOGI(TAG,
                 "Download prepare: addr=0x%08lX len=%lu",
                 (unsigned long)addr,
                 (unsigned long)len);


        /*
         * Address must point inside our virtual file.
         */
        if (addr < PENTAX_JPEG_ADDR) {

            ESP_LOGE(TAG,
                     "Download address below virtual file");

            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }


        uint32_t offset = addr - PENTAX_JPEG_ADDR;


        if (offset > s_file_size) {

            ESP_LOGE(TAG,
                     "Download offset outside file: %lu",
                     (unsigned long)offset);

            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }


        /*
         * Clamp requested length to remaining file size.
         */
        uint32_t remaining = s_file_size - offset;

        if (len > remaining) {
            len = remaining;
        }


        s_download_addr = addr;
        s_download_length = len;
        s_download_prepared = true;


        /*
         * The actual bytes are supplied by READ_RESULT (F0 49).
         *
         * Do NOT put the whole file into s_result.
         */
        s_result_len = len;


        ESP_LOGI(TAG,
                 "Download prepared: offset=%lu length=%lu",
                 (unsigned long)offset,
                 (unsigned long)len);

        return;
    }


    set_command_status(PENTAX_ST_NOT_AVAILABLE);
}


/* =========================================================
 * Command: ACTION
 * ========================================================= */

static void process_action_command(uint8_t command)
{
    /*
     * SHUTTER
     *
     * 10/05
     */
    if (command == PENTAX_C_SHUTTER) {

        if (s_args_len != 4) {

            ESP_LOGE(TAG,
                     "Shutter: invalid args length=%lu",
                     (unsigned long)s_args_len);

            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }


        uint32_t shutter_value = read_be32(s_args);

        ESP_LOGI(TAG,
                 "Shutter command: value=%lu",
                 (unsigned long)shutter_value);


        /*
         * Host uses:
         *
         *     full_press = 2
         */
        if (shutter_value != 2) {

            set_command_status(PENTAX_ST_ILLEGAL_PARAM);
            return;
        }


        /*
         * Simulate new photograph.
         */
        s_buffer_mask |= 0x00000001u;

        s_buffer_selected = false;

        reset_download_state();


        ESP_LOGI(TAG,
                 "Simulated photo captured. mask=0x%08lX",
                 (unsigned long)s_buffer_mask);

        return;
    }


    set_command_status(PENTAX_ST_NOT_AVAILABLE);
}


/* =========================================================
 * Pentax command dispatcher
 * ========================================================= */

static void process_pentax_command(uint8_t group, uint8_t command)
{
    clear_result();
    set_command_status(PENTAX_ST_OK);


    ESP_LOGI(TAG,
             "Pentax command: group=0x%02X command=0x%02X args=%lu",
             group,
             command,
             (unsigned long)s_args_len);


    switch (group) {

    case PENTAX_G_SYS:
        process_system_command(command);
        break;


    case PENTAX_G_BUFFER:
        process_buffer_command(command);
        break;


    case PENTAX_G_SEGMENT:
        process_segment_command(command);
        break;


    case PENTAX_G_ACTION:
        process_action_command(command);
        break;


    case PENTAX_G_DOWNLOAD:
        process_download_command(command);
        break;


    default:
        set_command_status(PENTAX_ST_NOT_AVAILABLE);
        break;
    }


    /*
     * Arguments belong only to this command.
     */
    memset(s_args, 0, sizeof(s_args));
    s_args_len = 0;
}


/* =========================================================
 * Pentax SCSI callback
 *
 * Called from the library's tud_msc_scsi_cb()
 * only for vendor opcode 0xF0.
 * ========================================================= */

int32_t pentax_msc_scsi_cb(uint8_t lun,
                           uint8_t const scsi_cmd[16],
                           void *buffer,
                           uint16_t bufsize)
{
    if (scsi_cmd[0] != PENTAX_SCSI_OPCODE) {

        ESP_LOGW(TAG,
                 "Unsupported SCSI opcode: 0x%02X",
                 scsi_cmd[0]);

        tud_msc_set_sense(lun, 0x05, 0x20, 0x00);

        return -1;
    }


    uint8_t type = scsi_cmd[1];


    /* =====================================================
     * SEND ARGUMENTS
     *
     * F0 4F
     *
     * Host -> Device
     * ===================================================== */

    if (type == PENTAX_T_SEND_ARGS) {

        uint16_t requested_len = read_le16(&scsi_cmd[4]);
        uint16_t offset        = read_le16(&scsi_cmd[2]);


        if (offset != 0 ||
            requested_len != bufsize ||
            requested_len > PENTAX_MAX_ARGS ||
            (requested_len > 0 && buffer == NULL)) {

            ESP_LOGE(TAG,
                     "Invalid argument transfer: "
                     "offset=%u len=%u received=%u",
                     offset,
                     requested_len,
                     bufsize);

            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

            return -1;
        }


        if (requested_len > 0) {
            memcpy(s_args, buffer, requested_len);
        }


        s_args_len = requested_len;


        ESP_LOGI(TAG,
                 "Received Pentax arguments: %u bytes",
                 requested_len);


        return 0;
    }


    /* =====================================================
     * SEND COMMAND
     *
     * F0 24
     *
     * No data transfer.
     * ===================================================== */

    if (type == PENTAX_T_SEND_COMMAND) {

        if (bufsize != 0) {

            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

            return -1;
        }


        uint8_t group = scsi_cmd[2];
        uint8_t command = scsi_cmd[3];

        uint16_t expected_args_len =
            read_le16(&scsi_cmd[4]);


        if (expected_args_len != s_args_len) {

            ESP_LOGE(TAG,
                     "Argument length mismatch: "
                     "expected=%u actual=%lu",
                     expected_args_len,
                     (unsigned long)s_args_len);


            set_command_status(PENTAX_ST_ILLEGAL_PARAM);

            clear_result();


            memset(s_args, 0, sizeof(s_args));
            s_args_len = 0;


            return 0;
        }


        process_pentax_command(group, command);

        return 0;
    }


    /* =====================================================
     * GET STATUS
     *
     * F0 26
     *
     * Device -> Host
     *
     * [0..3] result length LE32
     * [4..5] reserved
     * [6]    ready
     * [7]    status
     * ===================================================== */

    if (type == PENTAX_T_GET_STATUS) {

        if (bufsize < 8 || buffer == NULL) {

            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

            return -1;
        }


        uint8_t *out = (uint8_t *)buffer;


        out[0] = (uint8_t)s_result_len;
        out[1] = (uint8_t)(s_result_len >> 8);
        out[2] = (uint8_t)(s_result_len >> 16);
        out[3] = (uint8_t)(s_result_len >> 24);


        out[4] = 0;
        out[5] = 0;

        out[6] = PENTAX_ST_READY;
        out[7] = s_command_status;


        return 8;
    }


    /* =====================================================
     * READ RESULT
     *
     * F0 49
     *
     * Device -> Host
     *
     * For ordinary commands:
     *     return s_result
     *
     * For DOWNLOAD:
     *     return a slice of the virtual file.
     * ===================================================== */

    if (type == PENTAX_T_READ_RESULT) {

        uint16_t requested_len = read_le16(&scsi_cmd[4]);


        /*
         * -------------------------------------------------
         * Ordinary command result
         * -------------------------------------------------
         */
        if (!s_download_prepared) {

            if (requested_len > s_result_len ||
                requested_len > bufsize ||
                (requested_len > 0 && buffer == NULL)) {

                ESP_LOGE(TAG,
                         "Invalid result read: "
                         "requested=%u available=%lu buffer=%u",
                         requested_len,
                         (unsigned long)s_result_len,
                         bufsize);

                tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

                return -1;
            }


            if (requested_len > 0) {
                memcpy(buffer, s_result, requested_len);
            }


            ESP_LOGI(TAG,
                     "Result sent to Host: %u bytes",
                     requested_len);


            return requested_len;
        }


        /*
         * -------------------------------------------------
         * File download result
         * -------------------------------------------------
         */

        if (buffer == NULL && requested_len > 0) {

            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

            return -1;
        }


        if (requested_len > bufsize) {

            ESP_LOGE(TAG,
                     "Download result too large for USB buffer: "
                     "requested=%u bufsize=%u",
                     requested_len,
                     bufsize);

            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

            return -1;
        }


        if (!s_buffer_selected ||
            (s_buffer_mask & 0x00000001u) == 0) {

            ESP_LOGE(TAG,
                     "Download requested without valid buffer");

            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

            return -1;
        }


        /*
         * Address comes from the current prepared operation.
         */
        if (s_download_addr < PENTAX_JPEG_ADDR) {

            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

            return -1;
        }


        uint32_t offset =
            s_download_addr - PENTAX_JPEG_ADDR;


        if (offset > s_file_size) {

            ESP_LOGE(TAG,
                     "Download offset outside file: %lu",
                     (unsigned long)offset);

            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

            return -1;
        }


        uint32_t available =
            s_file_size - offset;


        /*
         * The requested read must not exceed the prepared
         * operation or the actual file.
         */
        if (requested_len > available ||
            requested_len > s_download_length) {

            ESP_LOGE(TAG,
                     "Invalid download read: "
                     "requested=%u available=%lu prepared=%lu",
                     requested_len,
                     (unsigned long)available,
                     (unsigned long)s_download_length);

            tud_msc_set_sense(lun, 0x05, 0x24, 0x00);

            return -1;
        }


        if (requested_len > 0) {

            memcpy(buffer,
                   s_file_data + offset,
                   requested_len);
        }


        ESP_LOGI(TAG,
                 "File data sent: "
                 "addr=0x%08lX offset=%lu len=%u",
                 (unsigned long)s_download_addr,
                 (unsigned long)offset,
                 requested_len);


        /*
         * Advance virtual file address.
         *
         * This means consecutive DOWNLOAD PREPARE operations
         * can read the file sequentially.
         */
        s_download_addr += requested_len;


        if (s_download_length >= requested_len) {
            s_download_length -= requested_len;
        } else {
            s_download_length = 0;
        }


        if (s_download_length == 0) {
            s_download_prepared = false;
        }


        return requested_len;
    }


    /* =====================================================
     * Unknown Pentax transfer type
     * ===================================================== */

    ESP_LOGW(TAG,
             "Unknown Pentax command type: 0x%02X",
             type);


    tud_msc_set_sense(lun, 0x05, 0x20, 0x00);

    return -1;
}


/* =========================================================
 * USB Device Event Handler
 * ========================================================= */

static void usb_device_event_handler(tinyusb_event_t *event,
                                     void *arg)
{
    (void)arg;


    switch (event->id) {

    case TINYUSB_EVENT_ATTACHED:

        ESP_LOGI(TAG,
                 "USB Host connected");

        break;


    case TINYUSB_EVENT_DETACHED:

        ESP_LOGW(TAG,
                 "USB Host disconnected");

        /*
         * USB session is gone.
         *
         * Keep the captured image itself, just reset
         * connection-related state.
         */
        s_connected = false;
        s_buffer_selected = false;

        reset_download_state();

        break;


    default:

        ESP_LOGI(TAG,
                 "USB event: %d",
                 event->id);

        break;
    }
}


/* =========================================================
 * USB Device Initialization
 * ========================================================= */

esp_err_t usb_device_init(void)
{
    ESP_LOGI(TAG,
             "Initializing Pentax USB Device...");


    reset_camera_state();


    tinyusb_config_t tusb_cfg =
        TINYUSB_DEFAULT_CONFIG(usb_device_event_handler);


    tusb_cfg.descriptor.device =
        &pentax_device_descriptor;


    tusb_cfg.descriptor.string =
        pentax_string_descriptors;


    tusb_cfg.descriptor.string_count =
        sizeof(pentax_string_descriptors) /
        sizeof(pentax_string_descriptors[0]);


    tusb_cfg.descriptor.full_speed_config =
        pentax_configuration_descriptor;


    esp_err_t err =
        tinyusb_driver_install(&tusb_cfg);


    if (err != ESP_OK) {

        ESP_LOGE(TAG,
                 "TinyUSB initialization failed: %s",
                 esp_err_to_name(err));

        return err;
    }


    ESP_LOGI(TAG,
             "Pentax USB Device initialized");


    ESP_LOGI(TAG,
             "VID: 0x%04X, PID: 0x%04X",
             pentax_device_descriptor.idVendor,
             pentax_device_descriptor.idProduct);


    ESP_LOGI(TAG,
             "USB Interface: MSC / SCSI / BOT");


    ESP_LOGI(TAG,
             "Virtual file size: %lu bytes",
             (unsigned long)s_file_size);


    return ESP_OK;
}