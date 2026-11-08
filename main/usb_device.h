#pragma once

#include "esp_err.h"

/**
 * @brief Initialize USB Device stack.
 *
 * Starts TinyUSB in USB Device mode.
 *
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t usb_device_init(void);