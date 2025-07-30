#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cc113l.h"

static const char *TAG = "CC113L_RECEIVER";

// GPIO configuration (те же пины что и в сканере)
#define CC113L_CS_PIN     GPIO_NUM_3    
#define CC113L_MOSI_PIN   GPIO_NUM_14   
#define CC113L_MISO_PIN   GPIO_NUM_18   
#define CC113L_SCK_PIN    GPIO_NUM_15   

// Частота вашей метки (найденная сканером)
#define TARGET_FREQUENCY    418.0   // MHz - замените на найденную частоту

// Параметры приема
#define RX_TIMEOUT_MS       1000    // Таймаут ожидания данных
#define MAX_PACKET_SIZE     64      // Максимальный размер пакета
#define RSSI_THRESHOLD      -90     // Минимальный RSSI для обработки

// Функция чтения FIFO (добавьте в cc113l.h и cc113l.c)
esp_err_t cc113l_read_fifo_data(cc113l_handle_t handle, uint8_t *data, uint8_t length) {
    // Временная реализация через чтение регистра RXFIFO
    // В реальности нужно добавить эту функцию в cc113l.c
    for (int i = 0; i < length; i++) {
        esp_err_t ret = cc113l_read_reg(handle, CC113L_RXFIFO, &data[i]);
        if (ret != ESP_OK) return ret;
    }
    return ESP_OK;
}

void print_hex_data(const uint8_t *data, size_t len) {
    printf("Data (%zu bytes): ", len);
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
        if ((i + 1) % 16 == 0) printf("\n                 ");
    }
    printf("\n");
}

void print_binary_data(const uint8_t *data, size_t len) {
    printf("Binary: ");
    for (size_t i = 0; i < len && i < 8; i++) {  // Показываем только первые 8 байт
        for (int bit = 7; bit >= 0; bit--) {
            printf("%d", (data[i] >> bit) & 1);
        }
        printf(" ");
    }
    printf("\n");
}

// Простой анализ OOK данных
void analyze_ook_data(const uint8_t *data, size_t len, int16_t rssi) {
    if (len == 0) return;
    
    // Подсчет единиц и нулей
    int ones = 0, zeros = 0;
    for (size_t i = 0; i < len; i++) {
        for (int bit = 0; bit < 8; bit++) {
            if ((data[i] >> bit) & 1) ones++;
            else zeros++;
        }
    }
    
    // Поиск повторяющихся паттернов
    bool has_pattern = false;
    if (len >= 4) {
        // Проверяем повторение первых 2 байт
        if (memcmp(data, data + 2, 2) == 0) {
            has_pattern = true;
        }
    }
    
    ESP_LOGI(TAG, "Analysis: RSSI=%ddBm, Bits 1/0=%d/%d (%.1f%%), Pattern=%s", 
             rssi, ones, zeros, 
             (ones * 100.0) / (ones + zeros),
             has_pattern ? "YES" : "NO");
}

void ook_receiver_task(void *pvParameters) {
    cc113l_handle_t cc113l_handle = NULL;
    
    // Конфигурация CC113L (та же что в сканере)
    cc113l_config_t config = {
        .cs_pin = CC113L_CS_PIN,
        .mosi_pin = CC113L_MOSI_PIN,
        .miso_pin = CC113L_MISO_PIN,
        .sck_pin = CC113L_SCK_PIN,
        .spi_host = SPI2_HOST,
        .spi_clock_hz = 1000000  // 1MHz SPI clock
    };
    
    // Инициализация CC113L
    esp_err_t ret = cc113l_init(&config, &cc113l_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CC113L: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    // Конфигурация для приема OOK данных
    ret = cc113l_configure_ook_receiving(cc113l_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CC113L for receiving: %s", esp_err_to_name(ret));
        cc113l_deinit(cc113l_handle);
        vTaskDelete(NULL);
        return;
    }
    
    // Установка частоты вашей метки
    ret = cc113l_set_frequency(cc113l_handle, TARGET_FREQUENCY);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set frequency: %s", esp_err_to_name(ret));
        cc113l_deinit(cc113l_handle);
        vTaskDelete(NULL);
        return;
    }
    
    // Включение режима приема
    ret = cc113l_set_rx_mode(cc113l_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set RX mode: %s", esp_err_to_name(ret));
        cc113l_deinit(cc113l_handle);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "CC113L configured for OOK receiving on %.1f MHz", TARGET_FREQUENCY);
    ESP_LOGI(TAG, "Listening for radio tag transmissions...");
    
    uint8_t rx_buffer[MAX_PACKET_SIZE];
    uint32_t packets_received = 0;
    uint32_t last_activity_time = 0;
    
    while (1) {
        // Проверка RSSI
        int16_t rssi_dbm;
        ret = cc113l_get_rssi(cc113l_handle, &rssi_dbm);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read RSSI");
            break;
        }
        
        // Проверка carrier sense
        bool carrier_sense;
        ret = cc113l_get_carrier_sense(cc113l_handle, &carrier_sense);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read carrier sense");
            break;
        }
        
        // Проверка данных в FIFO
        uint8_t fifo_bytes;
        ret = cc113l_get_rx_fifo_bytes(cc113l_handle, &fifo_bytes);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read FIFO status");
            break;
        }
        
        // Если есть активность или данные в FIFO
        if (carrier_sense || fifo_bytes > 0 || rssi_dbm > RSSI_THRESHOLD) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
            // Логируем активность (не чаще чем раз в секунду)
            if (current_time - last_activity_time > 1000) {
                ESP_LOGI(TAG, "Activity detected: RSSI=%ddBm, CS=%s, FIFO=%d bytes", 
                         rssi_dbm, carrier_sense ? "YES" : "NO", fifo_bytes);
                last_activity_time = current_time;
            }
            
            // Если есть данные в FIFO, читаем их
            if (fifo_bytes > 0) {
                // Ограничиваем размер чтения
                uint8_t bytes_to_read = fifo_bytes;
                if (bytes_to_read > MAX_PACKET_SIZE) {
                    bytes_to_read = MAX_PACKET_SIZE;
                }
                
                // Читаем данные из FIFO (это упрощенная версия, нужна функция чтения FIFO)
                // В реальном коде нужно добавить функцию cc113l_read_fifo()
                if (rssi_dbm > -90) {
                    ESP_LOGI(TAG, "*** PACKET RECEIVED #%lu ***", (unsigned long)++packets_received);
                    ESP_LOGI(TAG, "FIFO contained %d bytes", fifo_bytes);
                }
                
                
                // Читаем данные из FIFO
                ret = cc113l_read_fifo_data(cc113l_handle, rx_buffer, bytes_to_read);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to read FIFO data");
                    // Очистка FIFO при ошибке
                    cc113l_write_strobe(cc113l_handle, CC113L_SFRX);
                    cc113l_write_strobe(cc113l_handle, CC113L_SRX);
                    continue;
                }
                
                if (rssi_dbm > -90) {
                // Анализ полученных данных
                    print_hex_data(rx_buffer, bytes_to_read);
                    print_binary_data(rx_buffer, bytes_to_read);
                    analyze_ook_data(rx_buffer, bytes_to_read, rssi_dbm);
                }
                
                // Очистка FIFO для следующего пакета
                cc113l_write_strobe(cc113l_handle, CC113L_SFRX);
                cc113l_write_strobe(cc113l_handle, CC113L_SRX);
                if (rssi_dbm > -90) {
                    ESP_LOGI(TAG, "FIFO flushed, ready for next packet\n");
                }
                
            }
        }
        
        // Периодическая статистика (каждые 30 секунд)
        static uint32_t last_stats_time = 0;
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_stats_time > 30000) {
            ESP_LOGI(TAG, "=== STATUS ===");
            ESP_LOGI(TAG, "Frequency: %.1f MHz", TARGET_FREQUENCY);
            ESP_LOGI(TAG, "Current RSSI: %d dBm", rssi_dbm);
            ESP_LOGI(TAG, "Packets received: %lu", (unsigned long)packets_received);
            ESP_LOGI(TAG, "Carrier sense: %s", carrier_sense ? "YES" : "NO");
            ESP_LOGI(TAG, "FIFO bytes: %d", fifo_bytes);
            ESP_LOGI(TAG, "==============\n");
            last_stats_time = current_time;
        }
        
        // Небольшая задержка между проверками
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Очистка
    cc113l_deinit(cc113l_handle);
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting CC113L OOK Data Receiver");
    ESP_LOGI(TAG, "Target frequency: %.1f MHz", TARGET_FREQUENCY);
    ESP_LOGI(TAG, "RSSI threshold: %d dBm", RSSI_THRESHOLD);
    
    ESP_LOGI(TAG, "GPIO Configuration:");
    ESP_LOGI(TAG, "CS_PIN:   GPIO%d", CC113L_CS_PIN);
    ESP_LOGI(TAG, "MOSI_PIN: GPIO%d", CC113L_MOSI_PIN);
    ESP_LOGI(TAG, "MISO_PIN: GPIO%d", CC113L_MISO_PIN);
    ESP_LOGI(TAG, "SCK_PIN:  GPIO%d", CC113L_SCK_PIN);
    
    // Запуск задачи приемника
    xTaskCreate(ook_receiver_task, "ook_receiver", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "OOK Data Receiver started!");
    ESP_LOGI(TAG, "Listening for your 418MHz radio tag...");
}