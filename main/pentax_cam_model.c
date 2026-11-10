#include "pentax_cam_model.h"

#include <string.h>

/* =========================================================
 * Pentax command groups / commands (see the sibling PentaxK200D
 * project's main/pentax_service.c, the authoritative source for
 * every value here).
 * ========================================================= */

#define G_SYS 0x00u
#define C_SET_CONNECT_MODE 0x00u
#define C_GET_STATUS       0x01u
#define C_GET_FULL_STATUS  0x08u
#define C_DSP_WU_REQ       0x09u

#define G_BUF 0x02u
#define C_BUF_STATUS       0x00u
#define C_BUF_SELECT       0x01u
#define C_BUF_DELETE       0x03u

#define G_SEG 0x04u
#define C_SEG_INFO         0x00u
#define C_SEG_NEXT         0x01u

#define G_DL  0x06u
#define C_DL_PREPARE       0x00u

#define G_ACT 0x10u
#define C_SHUTTER          0x05u
#define C_GREEN_BUTTON     0x07u
#define C_TETHER_CONNECT   0x0Au
#define C_BULB             0x0Du
#define C_DUST_REMOVAL     0x11u

#define SHUTTER_ACTION_AF      1u
#define SHUTTER_ACTION_RELEASE 2u

#define G_PARAM 0x18u
#define C_EXPOSURE_MODE    0x01u
#define C_AE_METERING      0x03u
#define C_FLASH_MODE       0x04u
#define C_AF_MODE          0x05u
#define C_AF_AREA          0x06u
#define C_AF_POINT         0x07u
#define C_WHITE_BALANCE    0x10u
#define C_WB_TUNING        0x11u
#define C_FILE_FORMAT      0x12u
#define C_JPEG_QUALITY     0x13u
#define C_JPEG_RESOLUTION  0x14u
#define C_ISO              0x15u
#define C_SHUTTER_SPEED    0x16u
#define C_APERTURE         0x17u
#define C_EV_COMP          0x18u
#define C_FLASH_EV_COMP    0x1Au
#define C_CUSTOM_IMAGE     0x1Bu
#define C_DRIVE_MODE       0x1Cu
#define C_RAW_FORMAT       0x1Fu

#define SEG_DATA 3u
#define SEG_LAST 2u

/* =========================================================
 * Byte helpers (big-endian, matching the K200D's pre-K-3 wire format)
 * ========================================================= */

static uint32_t rd32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static void wr16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

/* =========================================================
 * Init / small helpers
 * ========================================================= */

void pentax_cam_model_init(pentax_cam_model_t *m, const uint8_t *file_data, uint32_t file_size)
{
    memset(m, 0, sizeof(*m));
    m->selected_slot = -1;
    m->command_status = PENTAX_CAM_ST_OK;

    /* Plausible defaults, loosely an 18-55/F3.5-5.6 kit lens on a K200D. */
    m->shutter_nom = 1; m->shutter_denom = 250;
    m->aperture_nom = 56; m->aperture_denom = 10;    /* F5.6 */
    m->ev_comp_nom = 0; m->ev_comp_denom = 1;
    m->iso = 400; m->auto_iso_min = 100; m->auto_iso_max = 800;
    m->image_format = 0;      /* JPEG */
    m->jpeg_res_idx = 0;      /* 10M */
    m->jpeg_quality_hw = 1;   /* stars = 3 - 1 = 2 */
    m->raw_format = 0;        /* PEF */
    m->custom_image_tone = 1; /* Natural */
    m->saturation = 0; m->sharpness = 0; m->contrast = 0;
    m->exposure_mode = 0;
    m->af_mode = 1;           /* AF.S */
    m->af_area_select = 0;    /* Auto-5 */
    m->af_point_mask = 0;
    m->drive_mode = 0;        /* Single */
    m->shake_reduction = true;
    m->jpeg_hue = 0;
    m->meter_value_nom = 90; m->meter_value_denom = 10; /* EV9.0, arbitrary */
    m->lens_max_aperture_nom = 35; m->lens_max_aperture_denom = 10;  /* F3.5 */
    m->lens_min_aperture_nom = 220; m->lens_min_aperture_denom = 10; /* F22 */
    m->focused_af_point = 0;
    m->focal_length_nom = 35; m->focal_length_denom = 1; /* 35mm, mid-zoom */
    m->focus_distance = 2000;

    m->file_data = file_data;
    m->file_size = file_size;
    m->file_addr_base = 0x10000000u;
}

static void clear_result(pentax_cam_model_t *m)
{
    m->result_len = 0;
}

static void reset_download_state(pentax_cam_model_t *m)
{
    m->segment_index = 0;
    m->download_addr = 0;
    m->download_length = 0;
    m->download_prepared = false;
}

bool pentax_cam_model_send_args(pentax_cam_model_t *m, const uint8_t *data, uint16_t len)
{
    if (len > PENTAX_CAM_MAX_ARGS) {
        return false;
    }
    if (len > 0 && data != NULL) {
        memcpy(m->args, data, len);
    }
    m->args_len = len;
    return true;
}

/* =========================================================
 * Command: SYSTEM
 * ========================================================= */

static void build_full_status(const pentax_cam_model_t *m, uint8_t out[PENTAX_CAM_STATUS_LEN])
{
    memset(out, 0, PENTAX_CAM_STATUS_LEN);

    wr16(&out[0x16], (uint16_t)m->buffer_mask);

    wr32(&out[0x2C], m->shutter_nom);
    wr32(&out[0x30], m->shutter_denom);
    wr32(&out[0x34], m->aperture_nom);
    wr32(&out[0x38], m->aperture_denom);
    wr32(&out[0x3C], (uint32_t)m->ev_comp_nom);
    wr32(&out[0x40], (uint32_t)m->ev_comp_denom);

    wr32(&out[0x60], m->iso);
    wr32(&out[0x64], m->auto_iso_min);
    wr32(&out[0x68], m->auto_iso_max);

    wr32(&out[0x78], m->image_format);
    wr32(&out[0x7C], m->jpeg_res_idx);
    wr32(&out[0x80], m->jpeg_quality_hw);
    wr32(&out[0x84], m->raw_format);
    wr32(&out[0x88], m->custom_image_tone);
    wr32(&out[0x8C], (uint32_t)(m->saturation + 4));
    wr32(&out[0x90], (uint32_t)(m->sharpness + 4));
    wr32(&out[0x94], (uint32_t)(m->contrast + 4));

    wr32(&out[0xAC], m->exposure_mode);
    wr32(&out[0xB8], m->af_mode);
    wr32(&out[0xBC], m->af_area_select);
    wr32(&out[0xC0], m->af_point_mask);
    wr32(&out[0xCC], m->drive_mode);
    wr32(&out[0xDA], m->shake_reduction ? 1u : 0u);

    wr32(&out[0xF4], (uint32_t)(m->jpeg_hue + 4));

    wr32(&out[0x104], m->meter_value_nom);
    wr32(&out[0x108], m->meter_value_denom);

    wr32(&out[0x13C], m->lens_max_aperture_nom);
    wr32(&out[0x140], m->lens_max_aperture_denom);
    wr32(&out[0x144], m->lens_min_aperture_nom);
    wr32(&out[0x148], m->lens_min_aperture_denom);

    wr32(&out[0x150], m->focused_af_point);

    wr32(&out[0x17C], m->focal_length_nom);
    wr32(&out[0x180], m->focal_length_denom);
    wr32(&out[0x184], (uint32_t)m->focus_distance);
}

static void process_system_command(pentax_cam_model_t *m, uint8_t command)
{
    if (command == C_SET_CONNECT_MODE) {
        if (m->args_len != 4) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        uint32_t mode = rd32(m->args);
        if (mode != 0 && mode != 1) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        m->connected = (mode != 0);
        return;
    }

    if (!m->connected) {
        m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
        return;
    }

    if (command == C_GET_STATUS) {
        /* The real host (ensure_session()) only checks this call's own
         * success/failure, never its content -- a fixed-size, mostly-zero
         * reply is enough. */
        memset(m->result, 0, 28);
        wr32(&m->result[0], 1);
        wr32(&m->result[4], m->buffer_mask);
        m->result_len = 28;
        return;
    }

    if (command == C_GET_FULL_STATUS) {
        build_full_status(m, m->result);
        m->result_len = PENTAX_CAM_STATUS_LEN;
        return;
    }

    if (command == C_DSP_WU_REQ) {
        /* Brackets every [18 xx] parameter write; nothing to validate. */
        return;
    }

    m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
}

/* =========================================================
 * Command: BUFFER
 * ========================================================= */

static void process_buffer_command(pentax_cam_model_t *m, uint8_t command)
{
    if (!m->connected) {
        m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
        return;
    }

    if (command == C_BUF_STATUS) {
        wr32(m->result, m->buffer_mask);
        m->result_len = 4;
        return;
    }

    if (command == C_BUF_SELECT) {
        if (m->args_len != 12) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        uint32_t slot = rd32(&m->args[0]);
        uint32_t type = rd32(&m->args[4]);

        if (slot >= PENTAX_CAM_RAM_SLOTS) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        if ((m->buffer_mask & (1u << slot)) == 0) {
            m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
            return;
        }

        m->selected_slot = (int32_t)slot;
        m->selected_buf_type = type;
        reset_download_state(m);
        return;
    }

    if (command == C_BUF_DELETE) {
        if (m->args_len != 4) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        uint32_t slot = rd32(m->args);
        if (slot >= PENTAX_CAM_RAM_SLOTS) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        m->buffer_mask &= ~(1u << slot);
        if (m->selected_slot == (int32_t)slot) {
            m->selected_slot = -1;
            reset_download_state(m);
        }
        return;
    }

    m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
}

/* =========================================================
 * Command: SEGMENT
 * ========================================================= */

static void process_segment_command(pentax_cam_model_t *m, uint8_t command)
{
    if (!m->connected) {
        m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
        return;
    }

    if (command == C_SEG_INFO) {
        if (m->selected_slot < 0) {
            m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
            return;
        }
        memset(m->result, 0, 16);
        if (m->segment_index == 0) {
            wr32(&m->result[4], SEG_DATA);
            wr32(&m->result[8], m->file_addr_base);
            wr32(&m->result[12], m->file_size);
            m->result_len = 16;
            return;
        }
        if (m->segment_index == 1) {
            wr32(&m->result[4], SEG_LAST);
            m->result_len = 16;
            return;
        }
        m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
        return;
    }

    if (command == C_SEG_NEXT) {
        if (m->args_len != 4) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        if (m->selected_slot < 0) {
            m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
            return;
        }
        if (m->segment_index < 1) {
            m->segment_index++;
        }
        return;
    }

    m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
}

/* =========================================================
 * Command: DOWNLOAD
 * ========================================================= */

static void process_download_command(pentax_cam_model_t *m, uint8_t command)
{
    if (!m->connected) {
        m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
        return;
    }

    if (command == C_DL_PREPARE) {
        if (m->args_len != 8) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        if (m->selected_slot < 0) {
            m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
            return;
        }
        uint32_t addr = rd32(&m->args[0]);
        uint32_t len = rd32(&m->args[4]);

        if (addr < m->file_addr_base) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        uint32_t offset = addr - m->file_addr_base;
        if (offset > m->file_size) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        uint32_t remaining = m->file_size - offset;
        if (len > remaining) {
            len = remaining;
        }

        m->download_addr = addr;
        m->download_length = len;
        m->download_prepared = true;
        m->result_len = len; /* bytes themselves come via READ_RESULT */
        return;
    }

    m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
}

/* =========================================================
 * Command: ACTION
 * ========================================================= */

static void capture_new_photo(pentax_cam_model_t *m)
{
    for (uint32_t i = 0; i < PENTAX_CAM_RAM_SLOTS; i++) {
        if ((m->buffer_mask & (1u << i)) == 0) {
            m->buffer_mask |= (1u << i);
            break;
        }
    }
    /* A fresh capture invalidates whatever was selected before. */
    m->selected_slot = -1;
    reset_download_state(m);
}

static void process_action_command(pentax_cam_model_t *m, uint8_t command)
{
    if (!m->connected) {
        m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
        return;
    }

    if (command == C_SHUTTER) {
        if (m->args_len != 4) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        uint32_t action = rd32(m->args);
        if (action == SHUTTER_ACTION_AF) {
            /* Half-press: nothing observable changes besides "AF succeeded". */
            return;
        }
        if (action == SHUTTER_ACTION_RELEASE) {
            capture_new_photo(m);
            return;
        }
        m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
        return;
    }

    if (command == C_GREEN_BUTTON) {
        /* Resets exposure to program mode (encoding unconfirmed upstream;
         * 0 is used here as the "reset" stand-in, matching the real
         * firmware's own comment about this command's effect). */
        m->exposure_mode = 0;
        return;
    }

    if (command == C_TETHER_CONNECT) {
        if (m->args_len != 4) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        /* Nothing else observable: just acknowledge. */
        return;
    }

    if (command == C_BULB) {
        if (m->args_len != 4) {
            m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
            return;
        }
        uint32_t start = rd32(m->args);
        if (start == 1) {
            m->bulb_active = true;
            return;
        }
        if (start == 0) {
            if (m->bulb_active) {
                m->bulb_active = false;
                capture_new_photo(m);
            }
            return;
        }
        m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
        return;
    }

    if (command == C_DUST_REMOVAL) {
        return;
    }

    m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
}

/* =========================================================
 * Command: PARAMETER WRITES
 * ========================================================= */

/* Every [18 xx] write arrives already bracketed by DSPTaskWUReq on the real
 * wire (see process_system_command's C_DSP_WU_REQ) -- that framing doesn't
 * change anything here, the model just needs args_len/args to match. */
static void process_param_command(pentax_cam_model_t *m, uint8_t command)
{
    if (!m->connected) {
        m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
        return;
    }

    switch (command) {
    case C_EXPOSURE_MODE:
        if (m->args_len != 8) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        m->exposure_mode = rd32(&m->args[4]);
        return;

    case C_AE_METERING:
        if (m->args_len != 4) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        if (rd32(m->args) > 2) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        return; /* write-only: no [00 08] field to update */

    case C_FLASH_MODE:
        if (m->args_len != 4) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        return; /* write-only */

    case C_AF_MODE:
        if (m->args_len != 4) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t mode = rd32(m->args);
            if (mode != 1 && mode != 2) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->af_mode = mode;
        }
        return;

    case C_AF_AREA:
        if (m->args_len != 4) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t sel = rd32(m->args);
            if (sel > 3) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->af_area_select = sel;
        }
        return;

    case C_AF_POINT:
        if (m->args_len != 4) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t mask = rd32(m->args);
            if (mask > 0x7FFu) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->af_point_mask = mask;
        }
        return;

    case C_WHITE_BALANCE:
        if (m->args_len != 4) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        return; /* write-only */

    case C_WB_TUNING:
        if (m->args_len != 12) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        return; /* write-only */

    case C_FILE_FORMAT:
        if (m->args_len != 8) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t format = rd32(&m->args[4]);
            if (format > 2) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->image_format = format;
        }
        return;

    case C_JPEG_QUALITY:
        if (m->args_len != 8) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t hw = rd32(&m->args[4]);
            if (hw > 2) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->jpeg_quality_hw = hw;
        }
        return;

    case C_JPEG_RESOLUTION:
        if (m->args_len != 8) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t res = rd32(&m->args[4]);
            if (res > 2) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->jpeg_res_idx = res;
        }
        return;

    case C_ISO:
        if (m->args_len != 12) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t iso = rd32(&m->args[0]);
            uint32_t amin = rd32(&m->args[4]);
            uint32_t amax = rd32(&m->args[8]);
            if (iso < 100 || iso > 1600) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->iso = iso;
            m->auto_iso_min = amin;
            m->auto_iso_max = amax;
        }
        return;

    case C_SHUTTER_SPEED:
        if (m->args_len != 8) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t nom = rd32(&m->args[0]);
            uint32_t denom = rd32(&m->args[4]);
            if (nom == 0 || denom == 0) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->shutter_nom = nom;
            m->shutter_denom = denom;
        }
        return;

    case C_APERTURE:
        if (m->args_len != 12) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t nom = rd32(&m->args[0]);
            uint32_t denom = rd32(&m->args[4]);
            if (denom == 0) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->aperture_nom = nom;
            m->aperture_denom = denom;
        }
        return;

    case C_EV_COMP:
        if (m->args_len != 8) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            int32_t denom = (int32_t)rd32(&m->args[4]);
            if (denom == 0) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->ev_comp_nom = (int32_t)rd32(&m->args[0]);
            m->ev_comp_denom = denom;
        }
        return;

    case C_FLASH_EV_COMP:
        if (m->args_len != 8) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        if (rd32(&m->args[4]) == 0) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        return; /* write-only */

    case C_CUSTOM_IMAGE:
        if (m->args_len != 4) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t tone = rd32(m->args);
            if (tone > 5) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->custom_image_tone = tone;
        }
        return;

    case C_DRIVE_MODE:
        if (m->args_len != 4) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        m->drive_mode = rd32(m->args);
        return;

    case C_RAW_FORMAT:
        if (m->args_len != 8) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
        {
            uint32_t format = rd32(&m->args[4]);
            if (format > 1) { m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM; return; }
            m->raw_format = format;
        }
        return;

    default:
        m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE;
        return;
    }
}

/* =========================================================
 * Dispatcher
 * ========================================================= */

void pentax_cam_model_exec(pentax_cam_model_t *m, uint8_t group, uint8_t command, uint16_t expected_args_len)
{
    clear_result(m);
    m->command_status = PENTAX_CAM_ST_OK;

    if ((uint32_t)expected_args_len != m->args_len) {
        m->command_status = PENTAX_CAM_ST_ILLEGAL_PARAM;
        m->args_len = 0;
        return;
    }

    switch (group) {
    case G_SYS:   process_system_command(m, command); break;
    case G_BUF:   process_buffer_command(m, command); break;
    case G_SEG:   process_segment_command(m, command); break;
    case G_DL:    process_download_command(m, command); break;
    case G_ACT:   process_action_command(m, command); break;
    case G_PARAM: process_param_command(m, command); break;
    default:      m->command_status = PENTAX_CAM_ST_NOT_AVAILABLE; break;
    }

    m->args_len = 0;
}

void pentax_cam_model_get_status(const pentax_cam_model_t *m, uint8_t out[8])
{
    wr32(&out[0], m->result_len);
    out[4] = 0;
    out[5] = 0;
    out[6] = PENTAX_CAM_ST_READY;
    out[7] = m->command_status;
}

uint16_t pentax_cam_model_read_result(pentax_cam_model_t *m, uint16_t want, uint8_t *out)
{
    if (!m->download_prepared) {
        uint16_t n = (uint16_t)(want < m->result_len ? want : m->result_len);
        if (n > 0) {
            memcpy(out, m->result, n);
        }
        return n;
    }

    uint32_t offset = m->download_addr - m->file_addr_base;
    uint32_t available = (offset <= m->file_size) ? (m->file_size - offset) : 0;
    uint32_t n = want;
    if (n > available) {
        n = available;
    }
    if (n > m->download_length) {
        n = m->download_length;
    }
    if (n > 0 && m->file_data != NULL) {
        memcpy(out, m->file_data + offset, n);
    }

    m->download_addr += n;
    m->download_length -= n;
    if (m->download_length == 0) {
        m->download_prepared = false;
    }
    return (uint16_t)n;
}
