#include <stdio.h>
#include "ip5209_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h" 
#include <esp_log.h>
#include "config.h"

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
            ESP_LOGE(TAG, "Driver initialized successfully.\n");
            while (true) {
                float current = ip5209.getBatteryCurrent();
                ESP_LOGE(TAG, "Current battery Current: %.2f mA", current); 

                float level = ip5209.getBatteryLevel();
                ESP_LOGE(TAG, "Current battery level: %.2f", level); 
                
                uint8_t charge_status = ip5209.getChargingStatus();
                switch (charge_status) {
                    case 0x00:
                        ESP_LOGI("CHARGE", "Idle");
                        break;
                    case 0x01:
                        ESP_LOGI("CHARGE", "Trickle Charging");
                        break;
                    case 0x02:
                        ESP_LOGI("CHARGE", "Constant Current Charging");
                        break;
                    case 0x03:
                        ESP_LOGI("CHARGE", "Constant Voltage Charging");
                        break;
                    case 0x04:
                        ESP_LOGI("CHARGE", "Constant Voltage Stop Detection");
                        break;
                    case 0x05:
                        ESP_LOGI("CHARGE", "Charge Full");
                        break;
                    case 0x06:
                        ESP_LOGI("CHARGE", "Charge Timeout");
                        break;
                    default:
                        ESP_LOGI("CHARGE", "Unknown status");
                }
                
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }
        } else {
            ESP_LOGE(TAG, "Failed to initialize driver.\n");
        }
}