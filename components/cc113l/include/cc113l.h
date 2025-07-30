#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CC113L component configuration structure
 */
typedef struct {
    gpio_num_t cs_pin;      ///< Chip Select pin
    gpio_num_t mosi_pin;    ///< MOSI pin
    gpio_num_t miso_pin;    ///< MISO pin  
    gpio_num_t sck_pin;     ///< SCK pin
    spi_host_device_t spi_host; ///< SPI host (SPI2_HOST or SPI3_HOST)
    uint32_t spi_clock_hz;  ///< SPI clock frequency (max 10MHz for CC113L)
} cc113l_config_t;

/**
 * @brief CC113L handle type
 */
typedef struct cc113l_handle_s* cc113l_handle_t;

/**
 * @brief Scan result structure
 */
typedef struct {
    float frequency;                ///< Frequency in MHz
    int16_t max_rssi;              ///< Maximum RSSI during scan
    int16_t avg_rssi;              ///< Average RSSI during scan
    int16_t min_rssi;              ///< Minimum RSSI during scan
    uint8_t carrier_sense_count;    ///< Number of carrier sense detections
    uint8_t fifo_activity;         ///< FIFO activity count
    uint8_t strong_signal_count;   ///< Count of strong signals (>-90dBm)
} cc113l_scan_result_t;

// CC113L Command strobes
#define CC113L_SRES     0x30    ///< Reset chip
#define CC113L_SXOFF    0x32    ///< Turn off crystal oscillator
#define CC113L_SCAL     0x33    ///< Calibrate frequency synthesizer
#define CC113L_SRX      0x34    ///< Enable RX
#define CC113L_SIDLE    0x36    ///< Exit RX/TX, turn off frequency synthesizer
#define CC113L_SPWD     0x39    ///< Enter power down mode
#define CC113L_SFRX     0x3A    ///< Flush the RX FIFO buffer
#define CC113L_SNOP     0x3D    ///< No operation

// CC113L Status registers  
#define CC113L_PARTNUM        0x30    ///< Part number
#define CC113L_VERSION        0x31    ///< Current version number
#define CC113L_FREQEST        0x32    ///< Frequency offset estimate
#define CC113L_CRC_REG        0x33    ///< The last CRC comparison
#define CC113L_RSSI           0x34    ///< Received signal strength indication
#define CC113L_MARCSTATE      0x35    ///< Main Radio Control State Machine state
#define CC113L_PKTSTATUS      0x38    ///< Current GDOx status and packet status
#define CC113L_RXBYTES        0x3B    ///< Overflow and number of bytes in RX FIFO

// CC113L Configuration registers
#define CC113L_IOCFG2         0x00    ///< GDO2 output pin configuration
#define CC113L_IOCFG1         0x01    ///< GDO1 output pin configuration
#define CC113L_IOCFG0         0x02    ///< GDO0 output pin configuration
#define CC113L_FIFOTHR        0x03    ///< RX FIFO and TX FIFO thresholds
#define CC113L_SYNC1          0x04    ///< Sync word, high byte
#define CC113L_SYNC0          0x05    ///< Sync word, low byte
#define CC113L_PKTLEN         0x06    ///< Packet length
#define CC113L_PKTCTRL1       0x07    ///< Packet automation control
#define CC113L_PKTCTRL0       0x08    ///< Packet automation control
#define CC113L_ADDR           0x09    ///< Device address
#define CC113L_CHANNR         0x0A    ///< Channel number
#define CC113L_FSCTRL1        0x0B    ///< Frequency synthesizer control
#define CC113L_FSCTRL0        0x0C    ///< Frequency synthesizer control
#define CC113L_FREQ2          0x0D    ///< Frequency control word, high byte
#define CC113L_FREQ1          0x0E    ///< Frequency control word, middle byte
#define CC113L_FREQ0          0x0F    ///< Frequency control word, low byte
#define CC113L_MDMCFG4        0x10    ///< Modem configuration
#define CC113L_MDMCFG3        0x11    ///< Modem configuration
#define CC113L_MDMCFG2        0x12    ///< Modem configuration
#define CC113L_MDMCFG1        0x13    ///< Modem configuration
#define CC113L_MDMCFG0        0x14    ///< Modem configuration
#define CC113L_DEVIATN        0x15    ///< Modem deviation setting
#define CC113L_MCSM2          0x16    ///< Main Radio Control State Machine configuration
#define CC113L_MCSM1          0x17    ///< Main Radio Control State Machine configuration
#define CC113L_MCSM0          0x18    ///< Main Radio Control State Machine configuration
#define CC113L_FOCCFG         0x19    ///< Frequency Offset Compensation configuration
#define CC113L_BSCFG          0x1A    ///< Bit Synchronization configuration
#define CC113L_AGCCTRL2       0x1B    ///< AGC control
#define CC113L_AGCCTRL1       0x1C    ///< AGC control
#define CC113L_AGCCTRL0       0x1D    ///< AGC control
#define CC113L_RESERVED_0x20  0x20    ///< Reserved register
#define CC113L_FREND1         0x21    ///< Front end RX configuration
#define CC113L_FSCAL3         0x23    ///< Frequency synthesizer calibration
#define CC113L_FSCAL2         0x24    ///< Frequency synthesizer calibration
#define CC113L_FSCAL1         0x25    ///< Frequency synthesizer calibration
#define CC113L_FSCAL0         0x26    ///< Frequency synthesizer calibration
#define CC113L_TEST2          0x2C    ///< Various test settings
#define CC113L_TEST1          0x2D    ///< Various test settings
#define CC113L_TEST0          0x2E    ///< Various test settings
#define CC113L_RXFIFO         0x3F    ///< RX FIFO access

// Crystal frequency (26MHz)
#define CC113L_XOSC_FREQ      26000000UL

/**
 * @brief Initialize CC113L component
 * 
 * @param config Configuration structure
 * @param handle Pointer to store the handle
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if config or handle is NULL
 *     - ESP_ERR_NO_MEM if allocation failed
 *     - Other ESP error codes from SPI or GPIO initialization
 */
esp_err_t cc113l_init(const cc113l_config_t *config, cc113l_handle_t *handle);

/**
 * @brief Deinitialize CC113L component
 * 
 * @param handle CC113L handle
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t cc113l_deinit(cc113l_handle_t handle);

/**
 * @brief Reset CC113L chip
 * 
 * @param handle CC113L handle
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 *     - ESP_FAIL if chip not ready after reset
 */
esp_err_t cc113l_reset(cc113l_handle_t handle);

/**
 * @brief Configure CC113L for OOK scanning
 * 
 * @param handle CC113L handle
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t cc113l_configure_ook_scanning(cc113l_handle_t handle);
esp_err_t cc113l_configure_ook_receiving(cc113l_handle_t handle);
/**
 * @brief Set frequency
 * 
 * @param handle CC113L handle
 * @param freq_mhz Frequency in MHz
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL or frequency out of range
 */
esp_err_t cc113l_set_frequency(cc113l_handle_t handle, float freq_mhz);

/**
 * @brief Set RX mode
 * 
 * @param handle CC113L handle
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 */
esp_err_t cc113l_set_rx_mode(cc113l_handle_t handle);

/**
 * @brief Write register
 * 
 * @param handle CC113L handle
 * @param reg Register address
 * @param value Value to write
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 *     - ESP_FAIL if communication error
 */
esp_err_t cc113l_write_reg(cc113l_handle_t handle, uint8_t reg, uint8_t value);

/**
 * @brief Read register
 * 
 * @param handle CC113L handle
 * @param reg Register address
 * @param value Pointer to store read value
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or value is NULL
 *     - ESP_FAIL if communication error
 */
esp_err_t cc113l_read_reg(cc113l_handle_t handle, uint8_t reg, uint8_t *value);

/**
 * @brief Write strobe command
 * 
 * @param handle CC113L handle
 * @param strobe Strobe command
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 *     - ESP_FAIL if communication error
 */
esp_err_t cc113l_write_strobe(cc113l_handle_t handle, uint8_t strobe);

/**
 * @brief Read status register
 * 
 * @param handle CC113L handle
 * @param reg Status register address
 * @param value Pointer to store read value
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or value is NULL
 *     - ESP_FAIL if communication error
 */
esp_err_t cc113l_read_status(cc113l_handle_t handle, uint8_t reg, uint8_t *value);

/**
 * @brief Get RSSI value in dBm
 * 
 * @param handle CC113L handle
 * @param rssi_dbm Pointer to store RSSI value in dBm
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or rssi_dbm is NULL
 *     - ESP_FAIL if communication error
 */
esp_err_t cc113l_get_rssi(cc113l_handle_t handle, int16_t *rssi_dbm);

/**
 * @brief Get carrier sense status
 * 
 * @param handle CC113L handle
 * @param carrier_sense Pointer to store carrier sense status
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or carrier_sense is NULL
 *     - ESP_FAIL if communication error
 */
esp_err_t cc113l_get_carrier_sense(cc113l_handle_t handle, bool *carrier_sense);

/**
 * @brief Get RX FIFO bytes count
 * 
 * @param handle CC113L handle
 * @param fifo_bytes Pointer to store FIFO bytes count
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or fifo_bytes is NULL
 *     - ESP_FAIL if communication error
 */
esp_err_t cc113l_get_rx_fifo_bytes(cc113l_handle_t handle, uint8_t *fifo_bytes);

/**
 * @brief Scan single frequency
 * 
 * @param handle CC113L handle
 * @param freq_mhz Frequency to scan in MHz
 * @param samples Number of samples to take
 * @param dwell_time_ms Time between samples in ms
 * @param result Pointer to store scan result
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or result is NULL
 *     - ESP_FAIL if scan failed
 */
esp_err_t cc113l_scan_frequency(cc113l_handle_t handle, float freq_mhz, 
                               uint8_t samples, uint32_t dwell_time_ms, 
                               cc113l_scan_result_t *result);

/**
 * @brief Scan frequency range
 * 
 * @param handle CC113L handle
 * @param start_freq_mhz Start frequency in MHz
 * @param end_freq_mhz End frequency in MHz
 * @param step_freq_mhz Frequency step in MHz
 * @param samples_per_freq Number of samples per frequency
 * @param dwell_time_ms Time between samples in ms
 * @param results Array to store scan results
 * @param max_results Maximum number of results
 * @param actual_results Pointer to store actual number of results
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if invalid parameters
 *     - ESP_ERR_NO_MEM if results array too small
 *     - ESP_FAIL if scan failed
 */
esp_err_t cc113l_scan_range(cc113l_handle_t handle, 
                           float start_freq_mhz, float end_freq_mhz, float step_freq_mhz,
                           uint8_t samples_per_freq, uint32_t dwell_time_ms,
                           cc113l_scan_result_t *results, uint16_t max_results, 
                           uint16_t *actual_results);

#ifdef __cplusplus
}
#endif