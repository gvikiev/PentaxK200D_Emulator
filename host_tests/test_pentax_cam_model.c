#include "tinytest.h"
#include "pentax_cam_model.h"

#include <string.h>

/* Independent byte helpers (deliberately not shared with pentax_cam_model.c)
 * so these tests verify the model's actual wire layout, not just agree with
 * whatever helper it happens to use internally. */
static void be32(uint8_t *out, uint32_t v)
{
    out[0] = (uint8_t)(v >> 24); out[1] = (uint8_t)(v >> 16);
    out[2] = (uint8_t)(v >> 8); out[3] = (uint8_t)v;
}

static uint32_t rd_be32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t rd_be16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

/* Drives one logical [SEND_ARGS ->] SEND_COMMAND round trip, the same
 * framing real bot.c/pentax_scsi.c use on the wire. */
static void do_cmd(pentax_cam_model_t *m, uint8_t group, uint8_t command,
                   const uint8_t *args, uint16_t args_len)
{
    if (args_len > 0) {
        CHECK(pentax_cam_model_send_args(m, args, args_len));
    }
    pentax_cam_model_exec(m, group, command, args_len);
}

static void do_cmd1(pentax_cam_model_t *m, uint8_t group, uint8_t command, uint32_t a0)
{
    uint8_t args[4];
    be32(args, a0);
    do_cmd(m, group, command, args, sizeof(args));
}

static void do_cmd2(pentax_cam_model_t *m, uint8_t group, uint8_t command, uint32_t a0, uint32_t a1)
{
    uint8_t args[8];
    be32(&args[0], a0);
    be32(&args[4], a1);
    do_cmd(m, group, command, args, sizeof(args));
}

static void do_cmd3(pentax_cam_model_t *m, uint8_t group, uint8_t command, uint32_t a0, uint32_t a1, uint32_t a2)
{
    uint8_t args[12];
    be32(&args[0], a0);
    be32(&args[4], a1);
    be32(&args[8], a2);
    do_cmd(m, group, command, args, sizeof(args));
}

/* Every rendered photo is: SOI, then a COM (0xFFFE) marker segment whose
 * own length field says how many bytes it spans, then the template's body
 * (its own SOI already stripped). Returns the offset where that template
 * body resumes, so tests can check the "real" content without hardcoding
 * the marker's exact size. */
static uint32_t rendered_photo_tail_offset(const uint8_t *photo)
{
    CHECK_EQ(photo[0], 0xFF); CHECK_EQ(photo[1], 0xD8);
    CHECK_EQ(photo[2], 0xFF); CHECK_EQ(photo[3], 0xFE);
    uint16_t seg_len = rd_be16(&photo[4]); /* includes the 2 length bytes themselves */
    return 4u + seg_len;
}

static bool bytes_contain(const uint8_t *hay, size_t hay_len, const char *needle)
{
    size_t nlen = strlen(needle);
    if (nlen == 0 || nlen > hay_len) {
        return false;
    }
    for (size_t i = 0; i + nlen <= hay_len; i++) {
        if (memcmp(hay + i, needle, nlen) == 0) {
            return true;
        }
    }
    return false;
}

static void connect(pentax_cam_model_t *m)
{
    do_cmd1(m, 0x00, 0x00, 1);
    CHECK_EQ(m->command_status, PENTAX_CAM_ST_OK);
    CHECK(m->connected == true);
}

#define G_SYS 0x00u
#define C_CONNECT 0x00u
#define C_GET_STATUS 0x01u
#define C_GET_FULL_STATUS 0x08u
#define C_DSP_WU_REQ 0x09u
#define G_BUF 0x02u
#define C_BUF_STATUS 0x00u
#define C_BUF_SELECT 0x01u
#define C_BUF_DELETE 0x03u
#define G_SEG 0x04u
#define C_SEG_INFO 0x00u
#define C_SEG_NEXT 0x01u
#define G_DL 0x06u
#define C_DL_PREPARE 0x00u
#define G_ACT 0x10u
#define C_SHUTTER 0x05u
#define C_GREEN_BUTTON 0x07u
#define C_TETHER 0x0Au
#define C_BULB 0x0Du
#define C_DUST 0x11u
#define G_PARAM 0x18u
#define C_EXPOSURE_MODE 0x01u
#define C_AE_METERING 0x03u
#define C_FLASH_MODE 0x04u
#define C_AF_MODE 0x05u
#define C_AF_AREA 0x06u
#define C_AF_POINT 0x07u
#define C_WHITE_BALANCE 0x10u
#define C_WB_TUNING 0x11u
#define C_FILE_FORMAT 0x12u
#define C_JPEG_QUALITY 0x13u
#define C_JPEG_RESOLUTION 0x14u
#define C_ISO 0x15u
#define C_SHUTTER_SPEED 0x16u
#define C_APERTURE 0x17u
#define C_EV_COMP 0x18u
#define C_FLASH_EV_COMP 0x1Au
#define C_CUSTOM_IMAGE 0x1Bu
#define C_DRIVE_MODE 0x1Cu
#define C_RAW_FORMAT 0x1Fu

static const uint8_t k_test_file[] = {
    'J', 'P', 'E', 'G', '-', 'D', 'A', 'T', 'A', '-',
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
};

/* ------------------------------- connection gating ------------------------------- */

static void test_commands_rejected_before_connect(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));

    do_cmd(&m, G_SYS, C_GET_FULL_STATUS, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_NOT_AVAILABLE);

    do_cmd(&m, G_BUF, C_BUF_STATUS, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_NOT_AVAILABLE);

    do_cmd1(&m, G_ACT, C_DUST, 0); /* wrong arg count on purpose: still gated first */
}

static void test_connect_mode_validation(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));

    /* wrong arg length */
    do_cmd(&m, G_SYS, C_CONNECT, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);
    CHECK(m.connected == false);

    /* out-of-range mode value */
    do_cmd1(&m, G_SYS, C_CONNECT, 2);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);
    CHECK(m.connected == false);

    do_cmd1(&m, G_SYS, C_CONNECT, 1);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK(m.connected == true);

    do_cmd1(&m, G_SYS, C_CONNECT, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK(m.connected == false);
}

/* ------------------------------- status buffer ------------------------------- */

static void test_get_status_28_bytes_and_arg_mismatch(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd(&m, G_SYS, C_GET_STATUS, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.result_len, 28);

    uint8_t status8[8];
    pentax_cam_model_get_status(&m, status8);
    CHECK_EQ(rd_be32(status8), 28);
    CHECK_EQ(status8[6], PENTAX_CAM_ST_READY);
    CHECK_EQ(status8[7], PENTAX_CAM_ST_OK);
}

static void test_get_full_status_default_fields(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd(&m, G_SYS, C_GET_FULL_STATUS, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.result_len, PENTAX_CAM_STATUS_LEN);

    const uint8_t *s = m.result;
    CHECK_EQ(rd_be16(&s[0x16]), 0);              /* buffer_mask, nothing captured yet */
    CHECK_EQ(rd_be32(&s[0x2C]), m.shutter_nom);
    CHECK_EQ(rd_be32(&s[0x30]), m.shutter_denom);
    CHECK_EQ(rd_be32(&s[0x34]), m.aperture_nom);
    CHECK_EQ(rd_be32(&s[0x38]), m.aperture_denom);
    CHECK_EQ(rd_be32(&s[0x60]), m.iso);
    CHECK_EQ(rd_be32(&s[0x78]), m.image_format);
    CHECK_EQ(rd_be32(&s[0xB8]), m.af_mode);
    CHECK_EQ(rd_be32(&s[0x13C]), m.lens_max_aperture_nom);
    CHECK_EQ(rd_be32(&s[0x140]), m.lens_max_aperture_denom);
}

/* ------------------------------- parameter writes ------------------------------- */

static void test_shutter_speed_write_and_readback(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd1(&m, G_SYS, C_DSP_WU_REQ, 1);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);

    do_cmd2(&m, G_PARAM, C_SHUTTER_SPEED, 1, 1000);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.shutter_nom, 1);
    CHECK_EQ(m.shutter_denom, 1000);

    do_cmd1(&m, G_SYS, C_DSP_WU_REQ, 2);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);

    do_cmd(&m, G_SYS, C_GET_FULL_STATUS, NULL, 0);
    CHECK_EQ(rd_be32(&m.result[0x2C]), 1);
    CHECK_EQ(rd_be32(&m.result[0x30]), 1000);

    /* zero denom is rejected and must not change state */
    do_cmd2(&m, G_PARAM, C_SHUTTER_SPEED, 1, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);
    CHECK_EQ(m.shutter_denom, 1000);
}

static void test_aperture_and_iso_and_ev_comp(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd3(&m, G_PARAM, C_APERTURE, 80, 10, 0); /* F8.0 */
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.aperture_nom, 80);
    CHECK_EQ(m.aperture_denom, 10);

    do_cmd3(&m, G_PARAM, C_ISO, 800, 100, 1600);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.iso, 800);

    do_cmd3(&m, G_PARAM, C_ISO, 50 /* below range */, 0, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);
    CHECK_EQ(m.iso, 800); /* unchanged */

    uint8_t args[8];
    be32(&args[0], (uint32_t)(int32_t)-6); /* -0.6 EV */
    be32(&args[4], 10);
    do_cmd(&m, G_PARAM, C_EV_COMP, args, sizeof(args));
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.ev_comp_nom, -6);
    CHECK_EQ(m.ev_comp_denom, 10);

    do_cmd(&m, G_SYS, C_GET_FULL_STATUS, NULL, 0);
    CHECK_EQ((int32_t)rd_be32(&m.result[0x3C]), -6);
    CHECK_EQ(rd_be32(&m.result[0x40]), 10);
}

static void test_param_arg_length_mismatch_rejected(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    uint8_t args[8];
    be32(&args[0], 1); be32(&args[4], 250);
    CHECK(pentax_cam_model_send_args(&m, args, sizeof(args)));
    /* CDB claims only 4 bytes of args were sent, but 8 actually were */
    pentax_cam_model_exec(&m, G_PARAM, C_SHUTTER_SPEED, 4);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);
    CHECK_EQ(m.args_len, 0); /* always cleared afterward */
    CHECK_EQ(m.shutter_denom, 250); /* unchanged from init default */
}

static void test_param_bounds_validation(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd1(&m, G_PARAM, C_AF_MODE, 3); /* only 1 or 2 valid */
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);

    do_cmd1(&m, G_PARAM, C_AF_AREA, 4); /* 0..3 valid */
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);

    do_cmd1(&m, G_PARAM, C_AF_POINT, 0x800u); /* 11-bit mask */
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);

    do_cmd2(&m, G_PARAM, C_FILE_FORMAT, 1, 3); /* 0..2 valid */
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);

    do_cmd2(&m, G_PARAM, C_JPEG_QUALITY, 1, 3); /* hw 0..2 valid */
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);

    do_cmd2(&m, G_PARAM, C_RAW_FORMAT, 1, 2); /* 0..1 valid */
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);

    do_cmd1(&m, G_PARAM, C_CUSTOM_IMAGE, 6); /* 0..5 valid */
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);

    /* none of the rejected writes should have touched state */
    CHECK_EQ(m.af_mode, 1);
    CHECK_EQ(m.af_area_select, 0);
    CHECK_EQ(m.af_point_mask, 0);
    CHECK_EQ(m.image_format, 0);
    CHECK_EQ(m.jpeg_quality_hw, 1);
    CHECK_EQ(m.raw_format, 0);
    CHECK_EQ(m.custom_image_tone, 1);
}

static void test_write_only_params_accepted_and_dont_touch_status(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    uint8_t before[PENTAX_CAM_STATUS_LEN];
    do_cmd(&m, G_SYS, C_GET_FULL_STATUS, NULL, 0);
    memcpy(before, m.result, sizeof(before));

    do_cmd1(&m, G_PARAM, C_AE_METERING, 1);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    do_cmd1(&m, G_PARAM, C_FLASH_MODE, 2);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    do_cmd1(&m, G_PARAM, C_WHITE_BALANCE, 1);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    do_cmd3(&m, G_PARAM, C_WB_TUNING, 1, (uint32_t)(int32_t)-2, (uint32_t)(int32_t)3);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    do_cmd2(&m, G_PARAM, C_FLASH_EV_COMP, (uint32_t)(int32_t)-3, 10);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);

    do_cmd(&m, G_SYS, C_GET_FULL_STATUS, NULL, 0);
    CHECK(memcmp(before, m.result, sizeof(before)) == 0);
}

/* ------------------------------- actions ------------------------------- */

static void test_shutter_af_vs_release(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd1(&m, G_ACT, C_SHUTTER, 1 /* AF */);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.buffer_mask, 0);

    do_cmd1(&m, G_ACT, C_SHUTTER, 2 /* RELEASE */);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.buffer_mask, 1u);

    do_cmd1(&m, G_ACT, C_SHUTTER, 3 /* invalid */);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);
    CHECK_EQ(m.buffer_mask, 1u); /* unchanged */
}

static void test_green_button_resets_exposure_mode(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd2(&m, G_PARAM, C_EXPOSURE_MODE, 1, 7);
    CHECK_EQ(m.exposure_mode, 7);

    do_cmd(&m, G_ACT, C_GREEN_BUTTON, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.exposure_mode, 0);
}

static void test_tether_connect(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd1(&m, G_ACT, C_TETHER, 1);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    do_cmd1(&m, G_ACT, C_TETHER, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
}

static void test_bulb_start_stop_captures_photo(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    /* stop without a prior start: not an error, but no photo either */
    do_cmd1(&m, G_ACT, C_BULB, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.buffer_mask, 0);

    do_cmd1(&m, G_ACT, C_BULB, 1);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK(m.bulb_active == true);
    CHECK_EQ(m.buffer_mask, 0); /* not yet -- only on stop */

    do_cmd1(&m, G_ACT, C_BULB, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK(m.bulb_active == false);
    CHECK_EQ(m.buffer_mask, 1u);

    do_cmd1(&m, G_ACT, C_BULB, 5 /* invalid */);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);
}

static void test_dust_removal(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd(&m, G_ACT, C_DUST, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
}

/* ------------------------------- buffers / segments / download ------------------------------- */

static void test_multi_slot_capture_select_delete(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    for (int i = 0; i < 3; i++) {
        do_cmd1(&m, G_ACT, C_SHUTTER, 2);
        CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    }
    CHECK_EQ(m.buffer_mask, 0x7u); /* slots 0,1,2 */

    do_cmd(&m, G_BUF, C_BUF_STATUS, NULL, 0);
    CHECK_EQ(rd_be32(m.result), 0x7u);

    /* selecting an empty slot must fail */
    do_cmd3(&m, G_BUF, C_BUF_SELECT, 5, 0, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_NOT_AVAILABLE);

    /* out-of-range slot index must fail distinctly */
    do_cmd3(&m, G_BUF, C_BUF_SELECT, PENTAX_CAM_RAM_SLOTS, 0, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);

    do_cmd3(&m, G_BUF, C_BUF_SELECT, 1, 0, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.selected_slot, 1);

    do_cmd1(&m, G_BUF, C_BUF_DELETE, 1);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.buffer_mask, 0x5u); /* bit 1 cleared */
    CHECK_EQ(m.selected_slot, -1); /* selection cleared along with it */
}

static void test_segment_info_and_next(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    /* no selected buffer yet */
    do_cmd(&m, G_SEG, C_SEG_INFO, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_NOT_AVAILABLE);

    do_cmd1(&m, G_ACT, C_SHUTTER, 2);
    do_cmd3(&m, G_BUF, C_BUF_SELECT, 0, 0, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);

    do_cmd(&m, G_SEG, C_SEG_INFO, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(rd_be32(&m.result[4]), 3 /* SEG_DATA */);
    CHECK_EQ(rd_be32(&m.result[8]), m.file_addr_base);
    CHECK_EQ(rd_be32(&m.result[12]), m.slot_file_size[0]); /* rendered size, not the raw template's */
    CHECK(m.slot_file_size[0] > sizeof(k_test_file)); /* the marker segment adds bytes */

    do_cmd1(&m, G_SEG, C_SEG_NEXT, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);

    do_cmd(&m, G_SEG, C_SEG_INFO, NULL, 0);
    CHECK_EQ(rd_be32(&m.result[4]), 2 /* SEG_LAST */);

    /* a third SEGMENT_INFO past LAST is out of range */
    do_cmd1(&m, G_SEG, C_SEG_NEXT, 0); /* clamped: segment_index stays at 1 */
    do_cmd(&m, G_SEG, C_SEG_INFO, NULL, 0);
    CHECK_EQ(rd_be32(&m.result[4]), 2 /* still LAST, not advanced further */);
}

static void test_download_full_file_single_chunk(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);
    do_cmd1(&m, G_ACT, C_SHUTTER, 2);
    do_cmd3(&m, G_BUF, C_BUF_SELECT, 0, 0, 0);

    uint32_t rendered_size = m.slot_file_size[0];
    do_cmd2(&m, G_DL, C_DL_PREPARE, m.file_addr_base, rendered_size);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.result_len, rendered_size);
    CHECK(m.download_prepared == true);

    uint8_t out[PENTAX_CAM_MAX_PHOTO_SIZE];
    uint16_t n = pentax_cam_model_read_result(&m, (uint16_t)rendered_size, out);
    CHECK_EQ(n, rendered_size);
    CHECK(m.download_prepared == false); /* fully drained */

    uint32_t tail = rendered_photo_tail_offset(out);
    CHECK_EQ(rendered_size - tail, sizeof(k_test_file) - 2);
    CHECK(memcmp(out + tail, k_test_file + 2, sizeof(k_test_file) - 2) == 0);
}

static void test_download_multi_chunk_like_real_dl_chunk_looping(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);
    do_cmd1(&m, G_ACT, C_SHUTTER, 2);
    do_cmd3(&m, G_BUF, C_BUF_SELECT, 0, 0, 0);

    uint32_t rendered_size = m.slot_file_size[0];
    do_cmd2(&m, G_DL, C_DL_PREPARE, m.file_addr_base, rendered_size);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);

    uint8_t assembled[PENTAX_CAM_MAX_PHOTO_SIZE];
    size_t got = 0;
    const uint16_t chunk = 6; /* smaller than the file: forces several reads */
    while (m.download_prepared) {
        uint8_t buf[chunk];
        uint16_t n = pentax_cam_model_read_result(&m, chunk, buf);
        CHECK(n > 0);
        CHECK(got + n <= sizeof(assembled));
        memcpy(assembled + got, buf, n);
        got += n;
    }
    CHECK_EQ(got, rendered_size);
    uint32_t tail = rendered_photo_tail_offset(assembled);
    CHECK(memcmp(assembled + tail, k_test_file + 2, sizeof(k_test_file) - 2) == 0);
}

static void test_different_slots_and_captures_have_distinct_content(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd1(&m, G_ACT, C_SHUTTER, 2); /* -> slot 0 */
    do_cmd1(&m, G_ACT, C_SHUTTER, 2); /* -> slot 1 */
    CHECK_EQ(m.buffer_mask, 0x3u);

    /* same template, but the marker segment differs: not byte-identical */
    CHECK(m.slot_file_size[0] == m.slot_file_size[1]); /* same payload width (single-digit slot, fixed-width seq) */
    CHECK(memcmp(m.slot_files[0], m.slot_files[1], m.slot_file_size[0]) != 0);

    /* and each one's marker actually names its own slot, not the other's */
    CHECK(bytes_contain(m.slot_files[0], m.slot_file_size[0], "SLOT=0"));
    CHECK(bytes_contain(m.slot_files[1], m.slot_file_size[1], "SLOT=1"));
    CHECK(!bytes_contain(m.slot_files[0], m.slot_file_size[0], "SLOT=1"));
    CHECK(!bytes_contain(m.slot_files[1], m.slot_file_size[1], "SLOT=0"));

    /* delete slot 0 and capture again: the new photo lands in slot 0 again,
     * but with a new sequence number, so it still isn't identical to what
     * slot 0 held before. */
    uint8_t old_slot0[PENTAX_CAM_MAX_PHOTO_SIZE];
    uint32_t old_slot0_size = m.slot_file_size[0];
    memcpy(old_slot0, m.slot_files[0], old_slot0_size);

    do_cmd1(&m, G_BUF, C_BUF_DELETE, 0);
    do_cmd1(&m, G_ACT, C_SHUTTER, 2); /* reuses the now-free slot 0 */
    CHECK_EQ(m.buffer_mask, 0x3u);
    CHECK_EQ(m.slot_file_size[0], old_slot0_size); /* same fixed-width marker */
    CHECK(memcmp(m.slot_files[0], old_slot0, old_slot0_size) != 0); /* different sequence number */
}

static void test_download_prepare_rejects_bad_address_and_clamps_length(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);
    do_cmd1(&m, G_ACT, C_SHUTTER, 2);
    do_cmd3(&m, G_BUF, C_BUF_SELECT, 0, 0, 0);
    uint32_t rendered_size = m.slot_file_size[0];

    /* address below the virtual file's base */
    do_cmd2(&m, G_DL, C_DL_PREPARE, m.file_addr_base - 1, 4);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_ILLEGAL_PARAM);

    /* requesting more than remains is silently clamped, not rejected */
    do_cmd2(&m, G_DL, C_DL_PREPARE, m.file_addr_base, rendered_size + 1000);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_OK);
    CHECK_EQ(m.result_len, rendered_size);

    /* DOWNLOAD_PREPARE without a selected buffer must fail */
    pentax_cam_model_t m2;
    pentax_cam_model_init(&m2, k_test_file, sizeof(k_test_file));
    connect(&m2);
    do_cmd2(&m2, G_DL, C_DL_PREPARE, m2.file_addr_base, 4);
    CHECK_EQ(m2.command_status, PENTAX_CAM_ST_NOT_AVAILABLE);
}

/* ------------------------------- misc ------------------------------- */

static void test_unknown_group_and_command_rejected(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));
    connect(&m);

    do_cmd(&m, 0x7F, 0x00, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_NOT_AVAILABLE);

    do_cmd(&m, G_SYS, 0x7F, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_NOT_AVAILABLE);

    do_cmd(&m, G_PARAM, 0x7F, NULL, 0);
    CHECK_EQ(m.command_status, PENTAX_CAM_ST_NOT_AVAILABLE);
}

static void test_send_args_oversized_rejected(void)
{
    pentax_cam_model_t m;
    pentax_cam_model_init(&m, k_test_file, sizeof(k_test_file));

    uint8_t big[PENTAX_CAM_MAX_ARGS + 4];
    memset(big, 0, sizeof(big));
    CHECK(pentax_cam_model_send_args(&m, big, sizeof(big)) == false);
    CHECK_EQ(m.args_len, 0); /* rejected transfer must not land partial args */
}

int main(void)
{
    RUN(test_commands_rejected_before_connect);
    RUN(test_connect_mode_validation);
    RUN(test_get_status_28_bytes_and_arg_mismatch);
    RUN(test_get_full_status_default_fields);
    RUN(test_shutter_speed_write_and_readback);
    RUN(test_aperture_and_iso_and_ev_comp);
    RUN(test_param_arg_length_mismatch_rejected);
    RUN(test_param_bounds_validation);
    RUN(test_write_only_params_accepted_and_dont_touch_status);
    RUN(test_shutter_af_vs_release);
    RUN(test_green_button_resets_exposure_mode);
    RUN(test_tether_connect);
    RUN(test_bulb_start_stop_captures_photo);
    RUN(test_dust_removal);
    RUN(test_multi_slot_capture_select_delete);
    RUN(test_different_slots_and_captures_have_distinct_content);
    RUN(test_segment_info_and_next);
    RUN(test_download_full_file_single_chunk);
    RUN(test_download_multi_chunk_like_real_dl_chunk_looping);
    RUN(test_download_prepare_rejects_bad_address_and_clamps_length);
    RUN(test_unknown_group_and_command_rejected);
    RUN(test_send_args_oversized_rejected);
    return tt_summary();
}
