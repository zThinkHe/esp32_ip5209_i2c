#ifndef IP5209_DRIVER_H
#define IP5209_DRIVER_H

#include <stdint.h>
#include <esp_err.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

//I2C Address
#define IP5209_I2C_ADDR  0x75

// I2C 相关
#define I2C_IP5209_IRQ_IO  GPIO_NUM_7
#define I2C_MASTER_FREQ_HZ  400000
#define I2C_MASTER_TIMEOUT_MS  1000

// 充电电流设置宏定义
#define IP5209_CHARGE_CURRENT_500MA  6  // 500mA
#define IP5209_CHARGE_CURRENT_1A     10 // 1A
#define IP5209_CHARGE_CURRENT_1_2A   12 // 1.2A
#define IP5209_CHARGE_CURRENT_1_5A   18 // 1.5A
#define IP5209_CHARGE_CURRENT_2A     23 // 2A

// 充电状态
#define IP5209_CHARGE_STATUS_IDLE           0x0  // "Idle"
#define IP5209_CHARGE_STATUS_TRICKLE        0x01 // "Trickle Charging"
#define IP5209_CHARGE_STATUS_CCURRENT       0x02 // "Constant Current Charging"
#define IP5209_CHARGE_STATUS_CVOLTAGE       0x03 // "Constant Voltage Charging"
#define IP5209_CHARGE_STATUS_CVSTOP         0x04 // "Constant Voltage Stop Detection"
#define IP5209_CHARGE_STATUS_FULL           0x05 // "Charge Full"
#define IP5209_CHARGE_STATUS_TIMEOUT        0x06 // "Charge Timeout"
#define IP5209_CHARGE_STATUS_UNKNOWN        0x07 // "Unknown"

// 寄存器定义
#define IP5209_REG_SYS_CTL0        0x01
#define IP5209_REG_SYS_CTL1        0x02
#define IP5209_REG_SYS_CTL2        0x0C
#define IP5209_REG_SYS_CTL3        0x03
#define IP5209_REG_SYS_CTL4        0x04
#define IP5209_REG_SYS_CTL5        0x07
#define IP5209_REG_CHARGER_CTL1    0x22
#define IP5209_REG_CHARGER_CTL2    0x24
#define IP5209_REG_CHG_DIG_CTL4    0x25
#define IP5209_REG_BATVADC_DAT0    0xA2
#define IP5209_REG_BATVADC_DAT1    0xA3
#define IP5209_REG_BATIADC_DAT0    0xA4
#define IP5209_REG_BATIADC_DAT1    0xA5
#define IP5209_REG_BATOCV_DAT0     0xA8
#define IP5209_REG_BATOCV_DAT1     0xA9
#define IP5209_REG_READ0           0x71

class IP5209Driver {
    public:
        IP5209Driver(i2c_master_bus_config_t i2c_bus_config);
        ~IP5209Driver();
    
        esp_err_t writeRegister(uint8_t reg_addr, uint8_t data);
        esp_err_t readRegister(uint8_t reg_addr, uint8_t* data, size_t data_len);
    
        esp_err_t initialize();
    
        esp_err_t setChargeCurrent(uint8_t currentSetting);
        esp_err_t setChargeVoltage(uint8_t voltageSetting);
    
        float getBatteryVoltage();
        float getBatteryCurrent();
        float getBatteryOcVoltage();
        float getBatteryLevel();
    
        esp_err_t setFlashlight(bool enable);
        esp_err_t setBoost(bool enable);
        esp_err_t setCharger(bool enable);
    
        esp_err_t disableLowLoadAutoPowerOff();
        esp_err_t enableLowLoadAutoPowerOff();

        esp_err_t disable_ntc();
        esp_err_t enable_ntc();

        uint8_t getChargingStatus();
    
    private:
        uint8_t slave_addr;
        i2c_master_bus_handle_t  bus_handle;
        i2c_master_dev_handle_t  dev_handle;
    
        bool isI2CModeActive() {
            return gpio_get_level(I2C_IP5209_IRQ_IO) == 1;
        }
    };

#endif // IP5209_DRIVER_H