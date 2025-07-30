#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "cc113l.h"

static const char *TAG = "RADIO_TAG_RECEIVER";

// Global variables
static QueueHandle_t packet_queue = NULL;
static cc113l_handle_t cc113l_handle = NULL;

// GPIO configuration (same as scanner)
#define CC113L_CS_PIN     GPIO_NUM_3    
#define CC113L_MOSI_PIN   GPIO_NUM_14   
#define CC113L_MISO_PIN   GPIO_NUM_18   
#define CC113L_SCK_PIN    GPIO_NUM_15   

// Radio configuration
#define TARGET_FREQUENCY    418.0       // MHz - from scan results
#define PACKET_LENGTH       16          // bytes - encrypted data
#define RSSI_THRESHOLD      -90        // dBm - minimum signal strength
#define MAX_PACKET_QUEUE    10          // Maximum packets in queue
#define RX_BUFFER_SIZE      128         // Buffer for continuous data stream
#define SYNC_TIMEOUT_MS     5000        // Timeout for packet synchronization

// Packet structure
typedef struct {
    uint8_t data[PACKET_LENGTH];
    int16_t rssi;
    uint32_t timestamp;
    bool valid;
    bool synchronized;
} radio_packet_t;

// Reception buffer for continuous stream
typedef struct {
    uint8_t buffer[RX_BUFFER_SIZE];
    uint32_t write_pos;
    uint32_t read_pos;
    uint32_t last_activity;
    bool stream_active;
} rx_buffer_t;

static rx_buffer_t rx_buffer = {0};

/**
 * @brief Configure CC113L for OOK reception at 418MHz
 */
esp_err_t configure_ook_receiver(cc113l_handle_t handle) {
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Configuring CC113L for OOK reception...");
    
    // Reset and configure for OOK
    ret = cc113l_configure_ook_scanning(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure OOK scanning: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Manual frequency calculation for 418.0 MHz
    uint64_t freq_hz = (uint64_t)(TARGET_FREQUENCY * 1000000.0);
    uint32_t freq_reg = (uint32_t)((freq_hz << 16) / CC113L_XOSC_FREQ);
    
    ESP_LOGI(TAG, "Setting frequency: %.1f MHz", TARGET_FREQUENCY);
    ESP_LOGI(TAG, "Calculated freq_reg: 0x%06lX", freq_reg);
    
    // Write frequency registers manually
    ret = cc113l_write_reg(handle, CC113L_FREQ2, (uint8_t)((freq_reg >> 16) & 0xFF));
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FREQ1, (uint8_t)((freq_reg >> 8) & 0xFF));
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_FREQ0, (uint8_t)(freq_reg & 0xFF));
    if (ret != ESP_OK) return ret;
    
    // Recalibrate frequency synthesizer
    ret = cc113l_write_strobe(handle, CC113L_SCAL);
    if (ret != ESP_OK) return ret;
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // CRITICAL: Configure for TRANSPARENT/ASYNCHRONOUS mode to receive UART data
    // STM32 sends UART serial data, not packets!
    
    // Set data rate to match STM32 UART (likely 9600 baud)
    ret = cc113l_write_reg(handle, CC113L_MDMCFG4, 0xC6); // BW ~270kHz, DRATE_E=6
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MDMCFG3, 0x47); // DRATE_M for ~9600 baud
    if (ret != ESP_OK) return ret;
    
    // OOK modulation, NO sync word, NO packet mode
    ret = cc113l_write_reg(handle, CC113L_MDMCFG2, 0x30); // OOK, no sync, no manchester
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MDMCFG1, 0x00); // No FEC, no preamble detection
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MDMCFG0, 0xF8); // Channel spacing
    if (ret != ESP_OK) return ret;
    
    // AGC settings for OOK
    ret = cc113l_write_reg(handle, CC113L_AGCCTRL2, 0x07); // Max gain, MAGN_TARGET
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_AGCCTRL1, 0x00); // Carrier sense threshold
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_AGCCTRL0, 0x91); // OOK decision boundary
    if (ret != ESP_OK) return ret;
    
    // DISABLE packet handling entirely - this is KEY!
    ret = cc113l_write_reg(handle, CC113L_PKTLEN, 0x00); // Disable length check
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_PKTCTRL1, 0x00); // No packet control
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_PKTCTRL0, 0x00); // No packet control, continuous RX
    if (ret != ESP_OK) return ret;
    
    // Sync word - DISABLE completely for transparent mode
    ret = cc113l_write_reg(handle, CC113L_SYNC1, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_SYNC0, 0x00);
    if (ret != ESP_OK) return ret;
    
    // FIFO threshold - trigger immediately on any data
    ret = cc113l_write_reg(handle, CC113L_FIFOTHR, 0x00); // RX threshold = 1 byte
    if (ret != ESP_OK) return ret;
    
    // GDO configuration
    ret = cc113l_write_reg(handle, CC113L_IOCFG2, 0x29); // CHIP_RDYn
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_IOCFG1, 0x2E); // 3-state
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_IOCFG0, 0x01); // RX FIFO filled above threshold
    if (ret != ESP_OK) return ret;
    
    // Additional settings for transparent mode
    ret = cc113l_write_reg(handle, CC113L_MCSM2, 0x07);
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MCSM1, 0x30); // Stay in RX after receiving packet
    if (ret != ESP_OK) return ret;
    
    ret = cc113l_write_reg(handle, CC113L_MCSM0, 0x18);
    if (ret != ESP_OK) return ret;
    
    // Verify configuration
    uint8_t freq2, freq1, freq0, agcctrl0, mdmcfg2;
    ret = cc113l_read_reg(handle, CC113L_FREQ2, &freq2);
    if (ret == ESP_OK) ret = cc113l_read_reg(handle, CC113L_FREQ1, &freq1);
    if (ret == ESP_OK) ret = cc113l_read_reg(handle, CC113L_FREQ0, &freq0);
    if (ret == ESP_OK) ret = cc113l_read_reg(handle, CC113L_AGCCTRL0, &agcctrl0);
    if (ret == ESP_OK) ret = cc113l_read_reg(handle, CC113L_MDMCFG2, &mdmcfg2);
    
    if (ret == ESP_OK) {
        uint32_t read_freq_reg = (freq2 << 16) | (freq1 << 8) | freq0;
        float actual_freq = (float)((uint64_t)read_freq_reg * CC113L_XOSC_FREQ) / (1ULL << 16) / 1000000.0;
        ESP_LOGI(TAG, "✅ Configuration verified:");
        ESP_LOGI(TAG, "   Target frequency: %.1f MHz", TARGET_FREQUENCY);
        ESP_LOGI(TAG, "   Actual frequency: %.3f MHz (reg: 0x%06lX)", actual_freq, read_freq_reg);
        ESP_LOGI(TAG, "   AGCCTRL0: 0x%02X (CS threshold)", agcctrl0);
        ESP_LOGI(TAG, "   MDMCFG2: 0x%02X (OOK mode)", mdmcfg2);
        
        if (fabs(actual_freq - TARGET_FREQUENCY) > 0.1) {
            ESP_LOGW(TAG, "⚠️  Frequency mismatch! Expected %.1f MHz, got %.3f MHz", 
                     TARGET_FREQUENCY, actual_freq);
        }
    }
    
    ESP_LOGI(TAG, "OOK receiver configured for %.1f MHz with enhanced sensitivity", TARGET_FREQUENCY);
    return ESP_OK;
}

/**
 * @brief Add data to continuous reception buffer
 */
void add_to_rx_buffer(uint8_t *data, uint8_t length) {
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // Check if we're starting a new stream (gap > 500ms)
    if (!rx_buffer.stream_active || (current_time - rx_buffer.last_activity) > 500) {
        ESP_LOGI(TAG, "🔄 Starting new data stream");
        rx_buffer.write_pos = 0;
        rx_buffer.read_pos = 0;
        rx_buffer.stream_active = true;
    }
    
    // Add data to buffer
    for (int i = 0; i < length && rx_buffer.write_pos < RX_BUFFER_SIZE; i++) {
        rx_buffer.buffer[rx_buffer.write_pos] = data[i];
        rx_buffer.write_pos++;
    }
    
    rx_buffer.last_activity = current_time;
    
    ESP_LOGD(TAG, "📝 Added %d bytes to buffer (pos: %lu/%d)", 
             length, rx_buffer.write_pos, RX_BUFFER_SIZE);
}

/**
 * @brief Find packet sync in buffer
 */
bool find_packet_boundary(uint32_t *start_pos) {
    // Look for potential packet starts (0xE0-0xE7 range)
    for (uint32_t i = rx_buffer.read_pos; i <= rx_buffer.write_pos - PACKET_LENGTH; i++) {
        uint8_t first_byte = rx_buffer.buffer[i];
        
        // Check if this looks like a packet start
        if (first_byte >= 0xE0 && first_byte <= 0xE7) {
            // Check if this position makes sense
            bool looks_valid = true;
            uint8_t zero_count = 0;
            for (int j = 0; j < PACKET_LENGTH; j++) {
                if (rx_buffer.buffer[i + j] == 0x00) {
                    zero_count++;
                }
            }
            
            // If more than 12 zeros in 16 bytes, probably not a real packet
            if (zero_count > 12) {
                looks_valid = false;
            }
            
            if (looks_valid) {
                *start_pos = i;
                return true;
            }
        }
    }
    
    return false;
}

/**
 * @brief Extract synchronized packet from buffer
 */
bool extract_packet_from_buffer(radio_packet_t *packet, int16_t rssi) {
    uint32_t packet_start;
    
    if (!find_packet_boundary(&packet_start)) {
        return false;
    }
    
    // Extract packet
    memcpy(packet->data, &rx_buffer.buffer[packet_start], PACKET_LENGTH);
    packet->rssi = rssi;
    packet->timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    packet->valid = true;
    packet->synchronized = true;
    
    // Update read position
    rx_buffer.read_pos = packet_start + PACKET_LENGTH;
    
    ESP_LOGI(TAG, "📦 Extracted synchronized packet from position %lu", packet_start);
    
    return true;
}

/**
 * @brief Read data from CC113L FIFO into buffer
 */
esp_err_t read_data_from_fifo(cc113l_handle_t handle, int16_t *rssi) {
    esp_err_t ret;
    uint8_t fifo_bytes;
    
    if (!handle || !rssi) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check FIFO status
    ret = cc113l_get_rx_fifo_bytes(handle, &fifo_bytes);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read FIFO status: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "FIFO has %d bytes", fifo_bytes);
    
    if (fifo_bytes == 0) {
        return ESP_ERR_NOT_FOUND;
    }
    
    // Read RSSI
    ret = cc113l_get_rssi(handle, rssi);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read RSSI: %s", esp_err_to_name(ret));
        *rssi = -127;
    }
    
    // Check if signal is strong enough
    if (*rssi < RSSI_THRESHOLD) {
        ESP_LOGD(TAG, "Signal too weak: %d dBm < %d dBm", *rssi, RSSI_THRESHOLD);
        cc113l_write_strobe(handle, CC113L_SFRX);
        vTaskDelay(pdMS_TO_TICKS(5));
        cc113l_write_strobe(handle, CC113L_SRX);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // Read all available bytes
    uint8_t temp_buffer[64];
    uint8_t bytes_to_read = (fifo_bytes > 64) ? 64 : fifo_bytes;
    
    for (int i = 0; i < bytes_to_read; i++) {
        uint8_t byte;
        ret = cc113l_read_reg(handle, CC113L_RXFIFO, &byte);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to read FIFO byte %d: %s", i, esp_err_to_name(ret));
            cc113l_write_strobe(handle, CC113L_SFRX);
            vTaskDelay(pdMS_TO_TICKS(5));
            cc113l_write_strobe(handle, CC113L_SRX);
            return ret;
        }
        temp_buffer[i] = byte;
    }
    
    // Add data to continuous buffer
    add_to_rx_buffer(temp_buffer, bytes_to_read);
    
    ESP_LOGI(TAG, "📥 Read %d bytes from FIFO (RSSI: %d dBm)", bytes_to_read, *rssi);
    
    // Flush and restart RX
    cc113l_write_strobe(handle, CC113L_SFRX);
    vTaskDelay(pdMS_TO_TICKS(5));
    cc113l_write_strobe(handle, CC113L_SRX);
    
    return ESP_OK;
}

/**
 * @brief Main receiver task with persistent monitoring
 */
void radio_receiver_task(void *pvParameters) {
    esp_err_t ret;
    radio_packet_t packet;
    uint32_t packets_received = 0;
    uint32_t last_packet_time = 0;
    uint32_t last_status_check = 0;
    uint32_t last_successful_reception = 0;
    
    ESP_LOGI(TAG, "Radio receiver task started with persistent monitoring");
    
    while (1) {
        // Faster polling for better catching of brief transmissions
        vTaskDelay(pdMS_TO_TICKS(10)); // 10ms between checks
        
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Check carrier sense
        bool carrier_sense = false;
        ret = cc113l_get_carrier_sense(cc113l_handle, &carrier_sense);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read carrier sense: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        // Check FIFO activity
        uint8_t fifo_bytes = 0;
        ret = cc113l_get_rx_fifo_bytes(cc113l_handle, &fifo_bytes);
        bool fifo_activity = (ret == ESP_OK && fifo_bytes > 0);
        
        // Activity detected
        if (carrier_sense || fifo_activity) {
            last_successful_reception = current_time;
            
            if (carrier_sense) {
                ESP_LOGI(TAG, "🔍 Carrier sense detected!");
                
                // Additional diagnostics when carrier sense is detected
                uint8_t marcstate, pktstatus;
                if (cc113l_read_status(cc113l_handle, CC113L_MARCSTATE, &marcstate) == ESP_OK) {
                    ESP_LOGI(TAG, "   MARCSTATE: 0x%02X", marcstate);
                }
                if (cc113l_read_status(cc113l_handle, CC113L_PKTSTATUS, &pktstatus) == ESP_OK) {
                    ESP_LOGI(TAG, "   PKTSTATUS: 0x%02X (CS:%s, CRC:%s)", pktstatus,
                             (pktstatus & 0x40) ? "OK" : "NO",
                             (pktstatus & 0x80) ? "OK" : "NO");
                }
            }
            if (fifo_activity) {
                ESP_LOGI(TAG, "📥 FIFO activity detected (%d bytes)!", fifo_bytes);
            }
            
            // CRITICAL: Force immediate FIFO read even if no fifo_activity
            // Sometimes carrier sense triggers but FIFO status is delayed
            if (carrier_sense && !fifo_activity) {
                ESP_LOGW(TAG, "⚠️  Carrier sense without FIFO activity - forcing transparent mode read");
                
                // Force RX mode to ensure we're receiving
                cc113l_write_strobe(cc113l_handle, CC113L_SRX);
                
                // Wait longer for UART data to accumulate (UART is slower than packet mode)
                vTaskDelay(pdMS_TO_TICKS(20)); // Increased from 5ms to 20ms
                
                // Re-check FIFO multiple times
                for (int attempts = 0; attempts < 5; attempts++) {
                    cc113l_get_rx_fifo_bytes(cc113l_handle, &fifo_bytes);
                    if (fifo_bytes > 0) {
                        ESP_LOGI(TAG, "✅ Found %d bytes in FIFO after %dms delay (attempt %d)", 
                                 fifo_bytes, 20, attempts + 1);
                        fifo_activity = true;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(5)); // Additional 5ms between attempts
                }
                
                if (!fifo_activity) {
                    // Last resort: read raw register to see if there's data
                    uint8_t rxbytes_raw;
                    if (cc113l_read_status(cc113l_handle, CC113L_RXBYTES, &rxbytes_raw) == ESP_OK) {
                        ESP_LOGW(TAG, "   Raw RXBYTES register: 0x%02X", rxbytes_raw);
                        if (rxbytes_raw & 0x7F) { // Mask overflow bit
                            fifo_bytes = rxbytes_raw & 0x7F;
                            fifo_activity = true;
                            ESP_LOGI(TAG, "✅ Found %d bytes via raw register read", fifo_bytes);
                        }
                    }
                }
            }
            
            // Read data into buffer immediately
            int16_t rssi;
            ret = read_data_from_fifo(cc113l_handle, &rssi);
            if (ret == ESP_OK) {
                // Try to extract synchronized packets from buffer
                int packets_extracted = 0;
                while (extract_packet_from_buffer(&packet, rssi)) {
                    packets_received++;
                    packets_extracted++;
                    uint32_t time_since_last = packet.timestamp - last_packet_time;
                    last_packet_time = packet.timestamp;
                    
                    ESP_LOGI(TAG, "📦 Sync packet #%lu (RSSI: %d dBm, Δt: %lu ms)", 
                             packets_received, packet.rssi, time_since_last);
                    
                    // Print packet data
                    char hex_str[64];
                    for (int i = 0; i < PACKET_LENGTH; i++) {
                        sprintf(hex_str + i*3, "%02x ", packet.data[i]);
                    }
                    ESP_LOGI(TAG, "Data: %s", hex_str);
                    
                    // Send to processing queue
                    if (packet_queue != NULL) {
                        if (xQueueSend(packet_queue, &packet, 0) != pdTRUE) {
                            ESP_LOGW(TAG, "Packet queue full, dropping packet");
                        }
                    }
                }
                
                if (packets_extracted > 0) {
                    ESP_LOGI(TAG, "✅ Extracted %d packets from this transmission", packets_extracted);
                } else {
                    ESP_LOGW(TAG, "⚠️  No valid packets extracted despite FIFO activity");
                }
            } else if (ret == ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "⚠️  Carrier sense detected but no FIFO data available");
            } else {
                ESP_LOGW(TAG, "⚠️  Error reading FIFO: %s", esp_err_to_name(ret));
            }
        }
        
        // Check for buffer timeout
        if (rx_buffer.stream_active && 
            (current_time - rx_buffer.last_activity) > SYNC_TIMEOUT_MS) {
            ESP_LOGW(TAG, "⏰ Buffer timeout - resetting stream");
            rx_buffer.stream_active = false;
            rx_buffer.write_pos = 0;
            rx_buffer.read_pos = 0;
        }
        
        // Aggressive RX mode restoration if no activity for too long
        if ((current_time - last_successful_reception) > 30000) { // 30 seconds
            ESP_LOGW(TAG, "🔄 No activity for 30s - resetting CC113L to ensure RX mode");
            cc113l_set_rx_mode(cc113l_handle);
            last_successful_reception = current_time;
        }
        
        // Status reporting every 15 seconds
        if ((current_time - last_status_check) > 15000) {
            int16_t current_rssi;
            ret = cc113l_get_rssi(cc113l_handle, &current_rssi);
            
            uint32_t time_since_last_rx = (current_time - last_successful_reception) / 1000;
            
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "📊 Stats: %lu packets, RSSI: %d dBm, buffer: %lu/%d, last RX: %lus ago", 
                         packets_received, current_rssi, rx_buffer.write_pos, RX_BUFFER_SIZE,
                         time_since_last_rx);
            }
            
            if (packets_received == 0) {
                ESP_LOGW(TAG, "🔍 Still waiting for first packet... STM32 might be in deep sleep");
                ESP_LOGI(TAG, "💡 Tip: Try pressing reset on STM32 or check if it's configured for periodic wake-up");
            }
            
            // Regular RX mode reset
            cc113l_set_rx_mode(cc113l_handle);
            
            last_status_check = current_time;
        }
    }
}

/**
 * @brief Packet processing task (simplified without AES)
 */
void packet_processor_task(void *pvParameters) {
    radio_packet_t packet;
    uint32_t total_packets = 0;
    uint32_t test_pattern_packets = 0;
    uint32_t null_packets = 0;
    
    ESP_LOGI(TAG, "Packet processor task started (no AES decryption)");
    
    while (1) {
        if (xQueueReceive(packet_queue, &packet, portMAX_DELAY) == pdTRUE) {
            total_packets++;
            
            // Check for test pattern
            const uint8_t test_pattern[16] = {
                0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
                0x21, 0x43, 0x65, 0x87, 0xa9, 0xcb, 0xed, 0x0f
            };
            
            bool is_test_pattern = (memcmp(packet.data, test_pattern, 16) == 0);
            if (is_test_pattern) {
                test_pattern_packets++;
                ESP_LOGI(TAG, "🎯 TEST PATTERN DETECTED! (#%lu of %lu total)", 
                         test_pattern_packets, total_packets);
                continue;
            }
            
            // Check for null packet
            bool is_null_packet = true;
            for (int i = 1; i < PACKET_LENGTH; i++) {
                if (packet.data[i] != 0x00) {
                    is_null_packet = false;
                    break;
                }
            }
            
            if (is_null_packet) {
                null_packets++;
                ESP_LOGI(TAG, "📭 Null packet detected (ID: 0x%02X) - #%lu", packet.data[0], null_packets);
                continue;
            }
            
            // Log sync status
            if (packet.synchronized) {
                ESP_LOGI(TAG, "🔄 Processing synchronized packet");
            } else {
                ESP_LOGW(TAG, "⚠️  Processing unsynchronized packet");
            }
            
            // Analyze raw packet structure
            ESP_LOGI(TAG, "📊 Raw packet analysis:");
            ESP_LOGI(TAG, "   First byte: 0x%02X (possible header/type)", packet.data[0]);
            ESP_LOGI(TAG, "   Second byte: 0x%02X", packet.data[1]);
            
            // Count unique bytes (entropy check)
            uint8_t unique_bytes[256] = {0};
            int unique_count = 0;
            for (int i = 0; i < PACKET_LENGTH; i++) {
                if (unique_bytes[packet.data[i]] == 0) {
                    unique_bytes[packet.data[i]] = 1;
                    unique_count++;
                }
            }
            ESP_LOGI(TAG, "   Entropy: %d unique bytes out of %d", unique_count, PACKET_LENGTH);
            
            // Look for potential patterns
            if (packet.data[0] >= 0xE0 && packet.data[0] <= 0xE7) {
                ESP_LOGI(TAG, "   🔐 Likely encrypted data (header in 0xE0-0xE7 range)");
            }
            
            // Statistics
            if (total_packets % 5 == 0) {
                ESP_LOGI(TAG, "📈 Stats: %lu total, %lu test patterns, %lu null packets", 
                         total_packets, test_pattern_packets, null_packets);
            }
        }
    }
}

/**
 * @brief Initialize radio receiver
 */
esp_err_t init_radio_receiver(void) {
    esp_err_t ret;
    
    // Initialize CC113L
    cc113l_config_t config = {
        .cs_pin = CC113L_CS_PIN,
        .mosi_pin = CC113L_MOSI_PIN,
        .miso_pin = CC113L_MISO_PIN,
        .sck_pin = CC113L_SCK_PIN,
        .spi_host = SPI2_HOST,
        .spi_clock_hz = 1000000
    };
    
    ret = cc113l_init(&config, &cc113l_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CC113L: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure for OOK reception
    ret = configure_ook_receiver(cc113l_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure receiver: %s", esp_err_to_name(ret));
        cc113l_deinit(cc113l_handle);
        return ret;
    }
    
    // Enter RX mode
    ret = cc113l_set_rx_mode(cc113l_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter RX mode: %s", esp_err_to_name(ret));
        cc113l_deinit(cc113l_handle);
        return ret;
    }
    
    // Create packet queue
    packet_queue = xQueueCreate(MAX_PACKET_QUEUE, sizeof(radio_packet_t));
    if (packet_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create packet queue");
        cc113l_deinit(cc113l_handle);
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Radio receiver initialized successfully");
    return ESP_OK;
}

void app_main(void) {
    ESP_LOGI(TAG, "🚀 Starting Radio Tag Receiver");
    ESP_LOGI(TAG, "Target frequency: %.1f MHz", TARGET_FREQUENCY);
    ESP_LOGI(TAG, "Expected packet size: %d bytes", PACKET_LENGTH);
    ESP_LOGI(TAG, "RSSI threshold: %d dBm", RSSI_THRESHOLD);
    
    // Initialize receiver
    esp_err_t ret = init_radio_receiver();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize receiver, halting");
        return;
    }
    
    // Start receiver task
    xTaskCreate(radio_receiver_task, "radio_rx", 8192, NULL, 5, NULL);
    
    // Start packet processor task
    xTaskCreate(packet_processor_task, "packet_proc", 8192, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "✅ Radio Tag Receiver started successfully!");
    ESP_LOGI(TAG, "Listening for packets from STM32 radio tag...");
}