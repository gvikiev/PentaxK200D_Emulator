#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "usb_device.h"

static const char *TAG = "PENTAX_EMULATOR";

void app_main(void)
{
    ESP_LOGI(TAG, "Pentax K200D Emulator starting...");

    ESP_ERROR_CHECK(usb_device_init());

    ESP_LOGI(TAG, "USB Device is ready");

    while (1) {
        ESP_LOGI(TAG, "Camera emulator is running");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
