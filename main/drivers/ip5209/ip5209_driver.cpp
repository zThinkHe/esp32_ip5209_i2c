#include "ip5209_driver.h"
#include <esp_log.h>

static const char* TAG = "IP5209_DRIVER";

// I2C写寄存器
static esp_err_t ip5209_i2c_write_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data) {
    uint8_t write_buf[2] = {reg_addr, data};
    esp_err_t ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ip5209_i2c_write_reg failed: %d (reg_addr: 0x%02x, data: 0x%02x)", ret, reg_addr, data);
    }
    return ret;
}

// I2C读寄存器
static esp_err_t ip5209_i2c_read_reg(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t* data, size_t data_len) {
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, data_len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ip5209_i2c_read_reg failed: %d (reg_addr: 0x%02x)", ret, reg_addr);
    } else {
        ESP_LOGD(TAG, "Read %zu bytes from register 0x%02x", data_len, reg_addr);
        for (size_t i = 0; i < data_len; i++) {
            ESP_LOGD(TAG, "data[%zu] = 0x%02x", i, data[i]);
        }
    }
    return ret;
}

// 构造函数
IP5209Driver::IP5209Driver(i2c_master_bus_config_t i2c_bus_config) : bus_handle(nullptr), dev_handle(nullptr) {
  
    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C master bus creation failed: 0x%x", ret);
        return;
    }

    // 添加I2C设备
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IP5209_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: 0x%x", ret);
        i2c_del_master_bus(bus_handle);
        bus_handle = nullptr;
        return;
    }

    // 初始化GPIO
    gpio_reset_pin(I2C_IP5209_IRQ_IO);
    gpio_set_direction(I2C_IP5209_IRQ_IO, GPIO_MODE_INPUT);
}

// 析构函数
IP5209Driver::~IP5209Driver() {
    if (dev_handle) {
        i2c_master_bus_rm_device(dev_handle);
        dev_handle = nullptr;
    }
    if (bus_handle) {
        i2c_del_master_bus(bus_handle);
        bus_handle = nullptr;
    }
}

// 写寄存器
esp_err_t IP5209Driver::writeRegister(uint8_t reg_addr, uint8_t data) {
    if (!isI2CModeActive()) {
        ESP_LOGE(TAG, "I2C mode is not active (L3 is not high)");
        return ESP_FAIL;
    }
    return ip5209_i2c_write_reg(dev_handle, reg_addr, data);
}

// 读寄存器
esp_err_t IP5209Driver::readRegister(uint8_t reg_addr, uint8_t* data, size_t data_len) {
    if (!isI2CModeActive()) {
        ESP_LOGE(TAG, "I2C mode is not active (L3 is not high)");
        return ESP_FAIL;
    }
    return ip5209_i2c_read_reg(dev_handle, reg_addr, data, data_len);
}

// 初始化
esp_err_t IP5209Driver::initialize() {
    disableLowLoadAutoPowerOff();
    disable_ntc();
    // enableLowLoadAutoPowerOff();
    setFlashlight(false);
    setChargeCurrent(IP5209_CHARGE_CURRENT_2A);
    return ESP_OK;
}

// 设置充电电流
esp_err_t IP5209Driver::setChargeCurrent(uint8_t currentSetting) {
    uint8_t data;
    esp_err_t ret = readRegister(IP5209_REG_CHG_DIG_CTL4, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    data = (data & 0xE0) | (currentSetting & 0x1F);
    return writeRegister(IP5209_REG_CHG_DIG_CTL4, data);
}

// 设置充电电压
esp_err_t IP5209Driver::setChargeVoltage(uint8_t voltageSetting) {
    uint8_t data;
    esp_err_t ret = readRegister(IP5209_REG_CHARGER_CTL2, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    data = (data & 0xF0) | (voltageSetting & 0x0F);
    return writeRegister(IP5209_REG_CHARGER_CTL2, data);
}

// 读取电池电压
float IP5209Driver::getBatteryVoltage() {
    uint8_t dataLow, dataHigh;
    esp_err_t ret = readRegister(IP5209_REG_BATVADC_DAT0, &dataLow, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to read low register");
        return -1.0f;
    }
    ret = readRegister(IP5209_REG_BATVADC_DAT1, &dataHigh, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to read high register");
        return -1.0f;
    }

    // 判断是否为补码
    if ((dataHigh & 0x20) == 0x20) { // 补码情况
        int16_t a = ~dataLow;
        int16_t b = (~(dataHigh & 0x1F)) & 0x1F;
        int16_t c = (b << 8) | (a + 1);
        float voltage = 2600.0f - (c * 0.26855f); // 单位：mV
        return voltage / 1000.0f; // 转换为V
    } else { // 原码情况
        uint16_t batOcVadc = (static_cast<uint16_t>(dataHigh) << 8) | dataLow;
        float voltage = 2600.0f + (batOcVadc * 0.26855f); // 单位：mV
        return voltage / 1000.0f; // 转换为V
    }
}

// 读取电池电流
float IP5209Driver::getBatteryCurrent() {
    uint8_t dataLow, dataHigh;
    esp_err_t ret = readRegister(IP5209_REG_BATIADC_DAT0, &dataLow, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to read low register");
        return -1.0f;
    }
    ret = readRegister(IP5209_REG_BATIADC_DAT1, &dataHigh, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to read high register");
        return -1.0f;
    }

    // 判断正负值
    if ((dataHigh & 0x20) == 0x20) { // 负值
        int16_t a = ~dataLow;
        int16_t b = (~(dataHigh & 0x1F)) & 0x1F;
        int16_t batIadc = (b << 8) | (a + 1);
        return -batIadc * 0.745985f; // 单位：mA
    } else { // 正值
        int16_t batIadc = (static_cast<int16_t>(dataHigh) << 8) | dataLow;
        return batIadc * 0.745985f; // 单位：mA
    }
}

// 读取电池开路电压
// BATOCV=BATVADC+BATIADC*预设电池内阻（BATIADC 有方向）
float IP5209Driver::getBatteryOcVoltage() {
    uint8_t dataLow, dataHigh;
    esp_err_t ret = readRegister(IP5209_REG_BATOCV_DAT0, &dataLow, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to read low register");
        return -1.0f;
    }
    ret = readRegister(IP5209_REG_BATOCV_DAT1, &dataHigh, 1);
    if (ret != ESP_OK) {
        ESP_LOGE("I2C", "Failed to read high register");
        return -1.0f;
    }

    // 判断是否为补码
    if ((dataHigh & 0x20) == 0x20) { // 补码情况
        int16_t a = ~dataLow;
        int16_t b = (~(dataHigh & 0x1F)) & 0x1F;
        int16_t c = (b << 8) | (a + 1);
        float voltage = 2600.0f - (c * 0.26855f); // 单位：mV
        return voltage / 1000.0f; // 转换为V
    } else { // 原码情况
        uint16_t batOcVadc = (static_cast<uint16_t>(dataHigh) << 8) | dataLow;
        float voltage = 2600.0f + (batOcVadc * 0.26855f); // 单位：mV
        return voltage / 1000.0f; // 转换为V
    }
}

// 读取当前电量
float IP5209Driver::getBatteryLevel() {
    float voltage = getBatteryOcVoltage();
    ESP_LOGE(TAG, "Current battery OC voltage: %.2f V", voltage); 

    if (voltage < 0.0f) {
        return -1.0f;
    }

    if (voltage >= 4.2f) {
        return 100.0f;
    } else if (voltage >= 3.0f) {
        return (voltage - 3.0f) / (4.2f - 3.0f) * 100.0f;
    } else {
        return 0.0f;
    }
}

// 设置手电筒功能
esp_err_t IP5209Driver::setFlashlight(bool enable) {
    uint8_t data;
    esp_err_t ret = readRegister(IP5209_REG_SYS_CTL0, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    if (enable) {
        data |= (1 << 3);
    } else {
        data &= ~(1 << 3);
    }
    return writeRegister(IP5209_REG_SYS_CTL0, data);
}

// 设置升压功能
esp_err_t IP5209Driver::setBoost(bool enable) {
    uint8_t data;
    esp_err_t ret = readRegister(IP5209_REG_SYS_CTL0, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    if (enable) {
        data |= (1 << 2);
    } else {
        data &= ~(1 << 2);
    }
    return writeRegister(IP5209_REG_SYS_CTL0, data);
}

// 设置充电功能
esp_err_t IP5209Driver::setCharger(bool enable) {
    uint8_t data;
    esp_err_t ret = readRegister(IP5209_REG_SYS_CTL0, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    if (enable) {
        data |= (1 << 1);
    } else {
        data &= ~(1 << 1);
    }
    return writeRegister(IP5209_REG_SYS_CTL0, data);
}

// 关闭低负载自动关机功能
esp_err_t IP5209Driver::disableLowLoadAutoPowerOff() {
    uint8_t data;
    esp_err_t ret = readRegister(IP5209_REG_SYS_CTL1, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    data &= ~(1 << 1);
    return writeRegister(IP5209_REG_SYS_CTL1, data);
}

// 打开低负载自动关机功能
esp_err_t IP5209Driver::enableLowLoadAutoPowerOff() {
    uint8_t data;
    esp_err_t ret = readRegister(IP5209_REG_SYS_CTL1, &data, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    data |= (1 << 1);
    return writeRegister(IP5209_REG_SYS_CTL1, data);
}

// 获取充电状态
uint8_t IP5209Driver::getChargingStatus() {
    uint8_t data;
    uint8_t charge_status = 0x0F;
    esp_err_t ret = readRegister(IP5209_REG_READ0, &data, 1);
    if (ret == ESP_OK) {
        // 解析充电状态
        charge_status = (data >> 5) & 0x07;
    } else {
        ESP_LOGE("I2C", "Failed to read register");
    }
    return charge_status;
}

// 关闭NTC功能的函数
esp_err_t IP5209Driver::disable_ntc() {
    uint8_t data;
    // 读取SYS_CTL5寄存器的当前值
    esp_err_t ret = readRegister(IP5209_REG_SYS_CTL5, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SYS_CTL5 register");
        return ret;
    }
    // 设置Bit6为1，关闭NTC功能
    data |= (1 << 6);
    // 写回修改后的值到SYS_CTL5寄存器
    ret = writeRegister(IP5209_REG_SYS_CTL5, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write SYS_CTL5 register");
        return ret;
    }
    ESP_LOGI(TAG, "NTC function has been disabled");
    return ESP_OK;
}

// 打开NTC功能的函数
esp_err_t IP5209Driver::enable_ntc() {
    uint8_t data;
    // 读取SYS_CTL5寄存器的当前值
    esp_err_t ret = readRegister(IP5209_REG_SYS_CTL5, &data, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SYS_CTL5 register");
        return ret;
    }
    // 设置Bit6为0，打开NTC功能
    data |= ~(1 << 6);
    // 写回修改后的值到SYS_CTL5寄存器
    ret = writeRegister(IP5209_REG_SYS_CTL5, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write SYS_CTL5 register");
        return ret;
    }
    ESP_LOGI(TAG, "NTC function has been enabled");
    return ESP_OK;
}