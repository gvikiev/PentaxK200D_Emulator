/*
 * Pentax K200D vendor-SCSI protocol emulation: pure state machine, no
 * TinyUSB/ESP-IDF dependency at all (just stdint/stddef/stdbool/string), so
 * it can be exercised directly from host-side tests (see host_tests/) the
 * same way it's driven for real from usb_device.c's tud_msc_scsi_cb().
 *
 * Field layout and command table cross-checked against the real PentaxHub
 * firmware this emulator talks to (the sibling PentaxK200D project's
 * main/pentax_status.h/.c and main/pentax_service.c -- those are the
 * authoritative source for every offset and group/command pair here).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PENTAX_CAM_RAM_SLOTS   10u
#define PENTAX_CAM_STATUS_LEN  408u
#define PENTAX_CAM_MAX_ARGS    32u
#define PENTAX_CAM_MAX_RESULT  512u

/* Must comfortably fit the template JPEG passed to pentax_cam_model_init()
 * plus the small COM marker segment capture_new_photo() inserts into every
 * rendered copy (see pentax_cam_model.c) -- a few hundred bytes of
 * overhead at most. */
#define PENTAX_CAM_MAX_PHOTO_SIZE 2048u

/* Pentax status byte values (S1), echoed via the 8-byte GET_STATUS reply
 * and used to pick the READY bit too. */
#define PENTAX_CAM_ST_READY         0x01u
#define PENTAX_CAM_ST_OK            0x01u
#define PENTAX_CAM_ST_NOT_AVAILABLE 0x81u
#define PENTAX_CAM_ST_REJECTED      0x82u
#define PENTAX_CAM_ST_ILLEGAL_PARAM 0x83u

typedef struct {
    bool connected;

    uint32_t buffer_mask;        /* bit i set => RAM slot i holds a photo */
    int32_t  selected_slot;      /* -1 = none selected */
    uint32_t selected_buf_type;
    uint32_t segment_index;      /* 0 = DATA, 1 = LAST */

    bool     download_prepared;
    uint32_t download_addr;
    uint32_t download_length;
    int32_t  download_slot;      /* which slot_files[] the in-progress download reads from */

    bool     bulb_active;
    uint32_t next_capture_seq;   /* monotonic; embedded in each capture's COM marker */

    /* Exposure / image-quality / AF state -- mirrors the real k200d_status_t
     * layout field-for-field so build_full_status() can place each one at
     * its documented [00 08] offset. */
    uint32_t shutter_nom, shutter_denom;
    uint32_t aperture_nom, aperture_denom;
    int32_t  ev_comp_nom, ev_comp_denom;
    uint32_t iso, auto_iso_min, auto_iso_max;
    uint32_t image_format, jpeg_res_idx, jpeg_quality_hw, raw_format;
    uint32_t custom_image_tone;
    int32_t  saturation, sharpness, contrast;
    uint32_t exposure_mode;
    uint32_t af_mode, af_area_select, af_point_mask;
    uint32_t drive_mode;
    bool     shake_reduction;
    int32_t  jpeg_hue;
    uint32_t meter_value_nom, meter_value_denom;
    uint32_t lens_max_aperture_nom, lens_max_aperture_denom;
    uint32_t lens_min_aperture_nom, lens_min_aperture_denom;
    uint32_t focused_af_point;
    uint32_t focal_length_nom, focal_length_denom;
    int32_t  focus_distance;

    /* Outcome of the last SEND_COMMAND, served by GET_STATUS/READ_RESULT. */
    uint8_t  command_status;
    uint8_t  result[PENTAX_CAM_MAX_RESULT];
    uint32_t result_len;

    uint8_t  args[PENTAX_CAM_MAX_ARGS];
    uint32_t args_len;

    /* The template JPEG every capture is rendered from (see
     * capture_new_photo() in pentax_cam_model.c): each RAM slot gets its
     * own copy with a small COM marker segment identifying the slot and
     * capture sequence number, so different slots -- and successive
     * captures into the same slot -- actually differ byte-for-byte instead
     * of every photo being identical. */
    const uint8_t *template_data;
    uint32_t template_size;
    uint32_t file_addr_base;

    uint8_t  slot_files[PENTAX_CAM_RAM_SLOTS][PENTAX_CAM_MAX_PHOTO_SIZE];
    uint32_t slot_file_size[PENTAX_CAM_RAM_SLOTS];
} pentax_cam_model_t;

/* `template_data`/`template_size` must stay valid for the model's whole
 * lifetime (only the pointer is kept, not a copy). */
void pentax_cam_model_init(pentax_cam_model_t *m, const uint8_t *template_data, uint32_t template_size);

/* SEND_ARGS (F0 4F). Returns false if `len` exceeds PENTAX_CAM_MAX_ARGS --
 * that's a transport-level problem (oversized transfer), not something a
 * Pentax status code expresses, so the caller should fail the SCSI command
 * itself rather than call pentax_cam_model_exec(). */
bool pentax_cam_model_send_args(pentax_cam_model_t *m, const uint8_t *data, uint16_t len);

/* SEND_COMMAND (F0 24). `expected_args_len` is the byte length the CDB
 * itself declares; a mismatch against what SEND_ARGS actually delivered is
 * rejected with PENTAX_CAM_ST_ILLEGAL_PARAM, same as a real body would.
 * Always clears the argument buffer afterward. */
void pentax_cam_model_exec(pentax_cam_model_t *m, uint8_t group, uint8_t command, uint16_t expected_args_len);

/* GET_STATUS (F0 26): fills the 8-byte reply. */
void pentax_cam_model_get_status(const pentax_cam_model_t *m, uint8_t out[8]);

/* READ_RESULT (F0 49): copies up to `want` bytes of whatever's pending (an
 * ordinary command result, or file bytes mid-download) into `out`, advancing
 * any in-progress download. Returns the number of bytes actually copied. */
uint16_t pentax_cam_model_read_result(pentax_cam_model_t *m, uint16_t want, uint8_t *out);

#ifdef __cplusplus
}
#endif
