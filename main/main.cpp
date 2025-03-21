#include <stdio.h>
#include "ip5209_driver.h"
#include "freertos/FreeRTOS.h"    // 新增头文件
#include "freertos/task.h" 
#include <esp_log.h>

static const char* TAG = "MAIN";
extern "C" void app_main() {
    // 初始化I2C总线
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
        },
    };
    IP5209Driver ip5209(i2c_bus_config);
        if (ip5209.initialize() == ESP_OK) {
            while (true) {
                ESP_LOGE(TAG, "Driver initialized successfully.\n");
                float level = ip5209.getBatteryLevel();
                ESP_LOGE(TAG, "Current battery level: %.2f", level); 
                
                ip5209.getChargingStatus();
                
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize driver.\n");
        }
}