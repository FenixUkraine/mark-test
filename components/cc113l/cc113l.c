#include "cc113l.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CC113L";

/**
 * @brief CC113L handle structure
 */
struct cc113l_handle_s {
    spi_device_handle_t spi_device;
    gpio_num_t cs_pin;
    gpio_num_t miso_pin;
    bool initialized;
};

/**
 * @brief Wait for chip ready (MISO goes low)
 */
static bool cc113l_wait_for_chip_ready(cc113l_handle_t handle, uint32_t timeout_ms) {
    if (!handle) return false;
    
    uint32_t timeout_ticks = timeout_ms / portTICK_PERIOD_MS;
    uint32_t start_time = xTaskGetTickCount();
    
    while ((xTaskGetTickCount() - start_time) < timeout_ticks) {
        if (gpio_get_level(handle->miso_pin) == 0) {
            return true;
        }
        vTaskDelay(1);
    }
    return false;
}

/**
 * @brief Perform SPI transaction with CS control
 */
static esp_err_t cc113l_spi_transaction(cc113l_handle_t handle, 
                                       const uint8_t *tx_data, uint8_t *rx_data, 
                                       size_t length) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = length * 8;
    t.tx_buffer = tx_data;
    t.rx_buffer = rx_data;
    
    gpio_set_level(handle->cs_pin, 0);
    
    if (!cc113l_wait_for_chip_ready(handle, 100)) {
        gpio_set_level(handle->cs_pin, 1);
        ESP_LOGE(TAG, "Chip not ready for transaction");
        return ESP_FAIL;
    }
    
    esp_err_t ret = spi_device_polling_transmit(handle->spi_device, &t);
    gpio_set_level(handle->cs_pin, 1);
    
    return ret;
}

/**
 * @brief Convert frequency in MHz to register value
 */
static uint32_t cc113l_frequency_to_register(float freq_mhz) {
    uint64_t freq_reg = ((uint64_t)(freq_mhz * 1000000.0) << 16) / CC113L_XOSC_FREQ;
    return (uint32_t)freq_reg;
}

esp_err_t cc113l_init(const cc113l_config_t *config, cc113l_handle_t *handle) {
    if (!config || !handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Allocate handle
    cc113l_handle_t cc113l = calloc(1, sizeof(struct cc113l_handle_s));
    if (!cc113l) {
        return ESP_ERR_NO_MEM;
    }
    
    // Store configuration
    cc113l->cs_pin = config->cs_pin;
    cc113l->miso_pin = config->miso_pin;
    
    // Configure SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = config->miso_pin,
        .mosi_io_num = config->mosi_pin,
        .sclk_io_num = config->sck_pin,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 128
    };
    
    esp_err_t ret = spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        free(cc113l);
        return ret;
    }
    
    // Configure SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->spi_clock_hz,
        .mode = 0,
        .spics_io_num = -1,  // We'll control CS manually
        .queue_size = 7,
    };
    
    ret = spi_bus_add_device(config->spi_host, &devcfg, &cc113l->spi_device);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(config->spi_host);
        free(cc113l);
        return ret;
    }
    
    // Configure CS pin
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << config->cs_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&cs_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS pin: %s", esp_err_to_name(ret));
        spi_bus_remove_device(cc113l->spi_device);
        spi_bus_free(config->spi_host);
        free(cc113l);
        return ret;
    }
    
    gpio_set_level(cc113l->cs_pin, 1);
    
    cc113l->initialized = true;
    *handle = cc113l;
    
    ESP_LOGI(TAG, "CC113L initialized successfully");
    return ESP_OK;
}

esp_err_t cc113l_deinit(cc113l_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (handle->initialized) {
        spi_bus_remove_device(handle->spi_device);
        handle->initialized = false;
    }
    
    free(handle);
    ESP_LOGI(TAG, "CC113L deinitialized");
    return ESP_OK;
}

esp_err_t cc113l_reset(cc113l_handle_t handle) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = cc113l_write_strobe(handle, CC113L_SRES);
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    if (!cc113l_wait_for_chip_ready(handle, 1000)) {
        ESP_LOGE(TAG, "Chip not ready after reset");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "CC113L reset completed");
    return ESP_OK;
}

esp_err_t cc113l_write_reg(cc113l_handle_t handle, uint8_t reg, uint8_t value) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t tx_data[2] = {reg, value};
    return cc113l_spi_transaction(handle, tx_data, NULL, 2);
}

esp_err_t cc113l_read_reg(cc113l_handle_t handle, uint8_t reg, uint8_t *value) {
    if (!handle || !handle->initialized || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t tx_data[2] = {reg | 0x80, 0x00};
    uint8_t rx_data[2];
    
    esp_err_t ret = cc113l_spi_transaction(handle, tx_data, rx_data, 2);
    if (ret == ESP_OK) {
        *value = rx_data[1];
    }
    
    return ret;
}

esp_err_t cc113l_write_strobe(cc113l_handle_t handle, uint8_t strobe) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    return cc113l_spi_transaction(handle, &strobe, NULL, 1);
}

esp_err_t cc113l_read_status(cc113l_handle_t handle, uint8_t reg, uint8_t *value) {
    if (!handle || !handle->initialized || !value) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t tx_data[2] = {reg | 0xC0, 0x00};
    uint8_t rx_data[2];
    
    esp_err_t ret = cc113l_spi_transaction(handle, tx_data, rx_data, 2);
    if (ret == ESP_OK) {
        *value = rx_data[1];
    }
    
    return ret;
}


esp_err_t cc113l_configure_ook_receiving(cc113l_handle_t handle) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    ret = cc113l_reset(handle);
    if (ret != ESP_OK) return ret;
    
    // GDO pins
    ret = cc113l_write_reg(handle, CC113L_IOCFG0, 0x2F);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_IOCFG1, 0x2E);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_IOCFG2, 0x2F);
    if (ret != ESP_OK) return ret;
    
    // Низкий порог FIFO для быстрого срабатывания
    ret = cc113l_write_reg(handle, CC113L_FIFOTHR, 0x07);  // 4 байта
    if (ret != ESP_OK) return ret;
    
    // Переменная длина пакета, БЕЗ CRC
    ret = cc113l_write_reg(handle, CC113L_PKTLEN, 255);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_PKTCTRL1, 0x00); // Без address check
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_PKTCTRL0, 0x40); // Переменная длина, БЕЗ CRC
    if (ret != ESP_OK) return ret;
    
    // Частота 418MHz
    ret = cc113l_write_reg(handle, CC113L_FREQ2, 0x0F);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_FREQ1, 0xEE);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_FREQ0, 0xF5);
    if (ret != ESP_OK) return ret;
    
    // Умеренная полоса канала для хорошей чувствительности
    ret = cc113l_write_reg(handle, CC113L_MDMCFG4, 0x5C);  // BW ~58kHz
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_MDMCFG3, 0x47);  // 8.1 kBaud
    if (ret != ESP_OK) return ret;
    
    // OOK БЕЗ синхронизации
    ret = cc113l_write_reg(handle, CC113L_MDMCFG2, 0x30);  // OOK, NO sync
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MDMCFG1, 0x02);  // Минимальная преамбула
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_MDMCFG0, 0xF8);
    if (ret != ESP_OK) return ret;
    
    // БЕЗ sync word
    ret = cc113l_write_reg(handle, CC113L_SYNC1, 0x00);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_SYNC0, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_DEVIATN, 0x47);
    if (ret != ESP_OK) return ret;
    
    // State machine - остаемся в RX
    ret = cc113l_write_reg(handle, CC113L_MCSM2, 0x07);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_MCSM1, 0x30);    // Stay in RX, no CCA
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_MCSM0, 0x14);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FOCCFG, 0x00);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_BSCFG, 0x00);
    if (ret != ESP_OK) return ret;
    
    // Настройки AGC для хорошей чувствительности но не максимальной
    ret = cc113l_write_reg(handle, CC113L_AGCCTRL2, 0x07); // Хорошее усиление
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_AGCCTRL1, 0x40);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_AGCCTRL0, 0x91);
    if (ret != ESP_OK) return ret;
    
    // Calibration
    ret = cc113l_write_reg(handle, CC113L_RESERVED_0x20, 0xFB);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_FREND1, 0x56);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_FSCAL3, 0xE9);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_FSCAL2, 0x2A);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_FSCAL1, 0x00);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_FSCAL0, 0x1F);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_TEST2, 0x81);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_TEST1, 0x35);
    if (ret != ESP_OK) return ret;
    ret = cc113l_write_reg(handle, CC113L_TEST0, 0x09);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "CC113L configured for SIGNAL-ONLY mode (no CRC, no sync)");
    return ESP_OK;
}


esp_err_t cc113l_configure_ook_scanning(cc113l_handle_t handle) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    // Reset chip first
    ret = cc113l_reset(handle);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Basic configuration for OOK scanning
    ret = cc113l_write_reg(handle, CC113L_IOCFG0, 0x06);   // GDO0 config
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_IOCFG1, 0x2E);   // GDO1 3-state
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_IOCFG2, 0x29);   // GDO2 CHIP_RDYn
    if (ret != ESP_OK) return ret;
    
    // FIFO threshold - low for sensitivity
    ret = cc113l_write_reg(handle, CC113L_FIFOTHR, 0x40);  // RX FIFO threshold = 4 bytes
    if (ret != ESP_OK) return ret;
    
    // Packet configuration - асинхронный режим для максимальной чувствительности
    ret = cc113l_write_reg(handle, CC113L_PKTLEN, 255);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_PKTCTRL1, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_PKTCTRL0, 0x00); // Fixed length, no CRC
    if (ret != ESP_OK) return ret;
    
    // Channel and frequency control
    ret = cc113l_write_reg(handle, CC113L_CHANNR, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FSCTRL1, 0x06);  // IF frequency
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FSCTRL0, 0x00);  // Frequency offset
    if (ret != ESP_OK) return ret;
    
    // Modem configuration для OOK на ~8kBaud
    ret = cc113l_write_reg(handle, CC113L_MDMCFG4, 0xCA);  // Channel BW ~100kHz, DRATE_E
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MDMCFG3, 0x83);  // DRATE_M for ~8kBaud
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MDMCFG2, 0x30);  // OOK, no sync word detection
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MDMCFG1, 0x22);  // FEC off, preamble
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MDMCFG0, 0xF8);  // Channel spacing
    if (ret != ESP_OK) return ret;
    
    // Deviation (не влияет на OOK)
    ret = cc113l_write_reg(handle, CC113L_DEVIATN, 0x00);
    if (ret != ESP_OK) return ret;
    
    // State machine
    ret = cc113l_write_reg(handle, CC113L_MCSM2, 0x07);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MCSM1, 0x30);    // Stay in RX after packet
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MCSM0, 0x18);
    if (ret != ESP_OK) return ret;
    
    // Frequency offset compensation (отключено для OOK)
    ret = cc113l_write_reg(handle, CC113L_FOCCFG, 0x16);
    if (ret != ESP_OK) return ret;
    
    // Bit synchronization
    ret = cc113l_write_reg(handle, CC113L_BSCFG, 0x6C);
    if (ret != ESP_OK) return ret;
    
    // AGC control - максимальная чувствительность для сканирования
    ret = cc113l_write_reg(handle, CC113L_AGCCTRL2, 0x03); // Max gain, MAGN_TARGET=33dB
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_AGCCTRL1, 0x40); // AGC settings
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_AGCCTRL0, 0x91); // OOK settings, 8dB decision boundary
    if (ret != ESP_OK) return ret;
    
    // Calibration registers
    ret = cc113l_write_reg(handle, CC113L_RESERVED_0x20, 0xFB);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FREND1, 0x56);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FSCAL3, 0xE9);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FSCAL2, 0x2A);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FSCAL1, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FSCAL0, 0x1F);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_TEST2, 0x81);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_TEST1, 0x35);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_TEST0, 0x09);
    if (ret != ESP_OK) return ret;
    
    // Verify chip identification
    uint8_t partnum, version;
    ret = cc113l_read_status(handle, CC113L_PARTNUM, &partnum);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_read_status(handle, CC113L_VERSION, &version);
    if (ret != ESP_OK) return ret;
    
    ESP_LOGI(TAG, "CC113L OOK configuration complete - PARTNUM: 0x%02X, VERSION: 0x%02X", 
             partnum, version);
    
    return ESP_OK;
}

esp_err_t cc113l_set_frequency(cc113l_handle_t handle, float freq_mhz) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Validate frequency range (general CC113L range)
    if (freq_mhz < 300.0 || freq_mhz > 928.0) {
        ESP_LOGE(TAG, "Frequency %.1f MHz out of range", freq_mhz);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t freq_reg = cc113l_frequency_to_register(freq_mhz);
    
    // Write frequency registers
    esp_err_t ret = cc113l_write_reg(handle, CC113L_FREQ2, (uint8_t)((freq_reg >> 16) & 0xFF));
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FREQ1, (uint8_t)((freq_reg >> 8) & 0xFF));
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FREQ0, (uint8_t)(freq_reg & 0xFF));
    if (ret != ESP_OK) return ret;
    
    // Recalibrate frequency synthesizer
    ret = cc113l_write_strobe(handle, CC113L_SCAL);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(5)); // Wait for calibration
    
    return ESP_OK;
}

esp_err_t cc113l_set_rx_mode(cc113l_handle_t handle) {
    if (!handle || !handle->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret = cc113l_write_strobe(handle, CC113L_SFRX);  // Flush RX FIFO
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_strobe(handle, CC113L_SRX);   // Enter RX mode
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(5));      // Wait for mode change
    
    return ESP_OK;
}

esp_err_t cc113l_get_rssi(cc113l_handle_t handle, int16_t *rssi_dbm) {
    if (!handle || !handle->initialized || !rssi_dbm) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t rssi_raw;
    esp_err_t ret = cc113l_read_status(handle, CC113L_RSSI, &rssi_raw);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Convert raw RSSI to dBm
    int16_t rssi_val = rssi_raw;
    if (rssi_val >= 128) {
        rssi_val = rssi_val - 256;
    }
    *rssi_dbm = rssi_val / 2 - 74;
    
    return ESP_OK;
}

esp_err_t cc113l_get_carrier_sense(cc113l_handle_t handle, bool *carrier_sense) {
    if (!handle || !handle->initialized || !carrier_sense) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t pktstatus;
    esp_err_t ret = cc113l_read_status(handle, CC113L_PKTSTATUS, &pktstatus);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *carrier_sense = (pktstatus & 0x40) != 0;
    
    return ESP_OK;
}

esp_err_t cc113l_get_rx_fifo_bytes(cc113l_handle_t handle, uint8_t *fifo_bytes) {
    if (!handle || !handle->initialized || !fifo_bytes) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t rxbytes;
    esp_err_t ret = cc113l_read_status(handle, CC113L_RXBYTES, &rxbytes);
    if (ret != ESP_OK) {
        return ret;
    }
    
    *fifo_bytes = rxbytes & 0x7F;
    
    return ESP_OK;
}

esp_err_t cc113l_scan_frequency(cc113l_handle_t handle, float freq_mhz, 
                               uint8_t samples, uint32_t dwell_time_ms, 
                               cc113l_scan_result_t *result) {
    if (!handle || !handle->initialized || !result || samples == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Set frequency and enter RX mode
    esp_err_t ret = cc113l_set_frequency(handle, freq_mhz);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_set_rx_mode(handle);
    if (ret != ESP_OK) return ret;
    
    // Initialize scan result
    result->frequency = freq_mhz;
    result->max_rssi = -127;
    result->min_rssi = 0;
    result->avg_rssi = 0;
    result->carrier_sense_count = 0;
    result->fifo_activity = 0;
    result->strong_signal_count = 0;
    
    int32_t rssi_sum = 0;
    
    // Sample RSSI multiple times on this frequency
    for (int sample = 0; sample < samples; sample++) {
        vTaskDelay(pdMS_TO_TICKS(dwell_time_ms));
        
        // Read RSSI
        int16_t rssi_dbm;
        ret = cc113l_get_rssi(handle, &rssi_dbm);
        if (ret != ESP_OK) return ret;
        
        // Read carrier sense and FIFO status
        bool carrier_sense;
        ret = cc113l_get_carrier_sense(handle, &carrier_sense);
        if (ret != ESP_OK) return ret;
        
        uint8_t fifo_bytes;
        ret = cc113l_get_rx_fifo_bytes(handle, &fifo_bytes);
        if (ret != ESP_OK) return ret;
        
        // Update statistics
        if (rssi_dbm > result->max_rssi) {
            result->max_rssi = rssi_dbm;
        }
        if (sample == 0 || rssi_dbm < result->min_rssi) {
            result->min_rssi = rssi_dbm;
        }
        rssi_sum += rssi_dbm;
        
        if (carrier_sense) {
            result->carrier_sense_count++;
        }
        
        if (fifo_bytes > 0) {
            result->fifo_activity++;
            // Flush FIFO to avoid overflow
            cc113l_write_strobe(handle, CC113L_SFRX);
            cc113l_write_strobe(handle, CC113L_SRX);
        }
        
        if (rssi_dbm > -90) {
            result->strong_signal_count++;
        }
    }
    
    result->avg_rssi = rssi_sum / samples;
    
    return ESP_OK;
}

esp_err_t cc113l_scan_range(cc113l_handle_t handle, 
                           float start_freq_mhz, float end_freq_mhz, float step_freq_mhz,
                           uint8_t samples_per_freq, uint32_t dwell_time_ms,
                           cc113l_scan_result_t *results, uint16_t max_results, 
                           uint16_t *actual_results) {
    if (!handle || !handle->initialized || !results || !actual_results) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (start_freq_mhz >= end_freq_mhz || step_freq_mhz <= 0 || samples_per_freq == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    float freq = start_freq_mhz;
    uint16_t result_count = 0;
    
    ESP_LOGI(TAG, "Starting frequency scan from %.1f to %.1f MHz (step %.1f MHz)", 
             start_freq_mhz, end_freq_mhz, step_freq_mhz);
    
    while (freq <= end_freq_mhz && result_count < max_results) {
        esp_err_t ret = cc113l_scan_frequency(handle, freq, samples_per_freq, 
                                             dwell_time_ms, &results[result_count]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to scan frequency %.1f MHz", freq);
            return ret;
        }
        
        cc113l_scan_result_t *result = &results[result_count];
        
        // Log interesting frequencies
        if (result->max_rssi > -95 || result->carrier_sense_count > 0 || result->fifo_activity > 0) {
            int16_t rssi_range = result->max_rssi - result->min_rssi;
            bool is_periodic = (result->carrier_sense_count > 0 && 
                               result->carrier_sense_count < samples_per_freq * 0.8);
            bool is_continuous = (result->carrier_sense_count > samples_per_freq * 0.8);
            
            const char *signal_type = "";
            if (is_continuous) signal_type = " [CONTINUOUS]";
            else if (is_periodic) signal_type = " [PERIODIC]";
            
            ESP_LOGI(TAG, "%.1f MHz: RSSI max=%d avg=%d range=%d CS=%d%% FIFO=%d%s <<<< ACTIVITY!", 
                     freq, result->max_rssi, result->avg_rssi, rssi_range,
                     (result->carrier_sense_count * 100) / samples_per_freq,
                     result->fifo_activity, signal_type);
        } else if ((int)(freq * 10) % 20 == 0) { // Print every 2 MHz
            ESP_LOGI(TAG, "%.1f MHz: RSSI max=%d avg=%d", 
                     freq, result->max_rssi, result->avg_rssi);
        }
        
        result_count++;
        freq += step_freq_mhz;
    }
    
    *actual_results = result_count;
    
    if (result_count >= max_results) {
        ESP_LOGW(TAG, "Scan reached maximum results limit (%d), truncating scan", max_results);
    }

    ESP_LOGI(TAG, "Frequency scan complete - scanned %d frequencies", result_count);
    return ESP_OK;
}