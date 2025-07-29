#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "cc113l.h"

static const char *TAG = "CC113L_EXAMPLE";

// GPIO configuration
#define CC113L_CS_PIN     GPIO_NUM_3    
#define CC113L_MOSI_PIN   GPIO_NUM_14   
#define CC113L_MISO_PIN   GPIO_NUM_18   
#define CC113L_SCK_PIN    GPIO_NUM_15   

// Scan parameters
#define SCAN_START_FREQ     417.6   // MHz
#define SCAN_END_FREQ       418.4   // MHz  
#define SCAN_STEP_FREQ      0.05    // MHz
#define SAMPLES_PER_FREQ    20      // Number of RSSI samples per frequency
#define DWELL_TIME_MS       50      // Time to spend on each frequency

void frequency_scanner_task(void *pvParameters) {
    cc113l_handle_t cc113l_handle = NULL;
    
    // Configure CC113L
    cc113l_config_t config = {
        .cs_pin = CC113L_CS_PIN,
        .mosi_pin = CC113L_MOSI_PIN,
        .miso_pin = CC113L_MISO_PIN,
        .sck_pin = CC113L_SCK_PIN,
        .spi_host = SPI2_HOST,
        .spi_clock_hz = 1000000  // 1MHz SPI clock
    };
    
    // Initialize CC113L
    esp_err_t ret = cc113l_init(&config, &cc113l_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CC113L: %s", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }
    
    // Configure for OOK scanning
    ret = cc113l_configure_ook_scanning(cc113l_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CC113L: %s", esp_err_to_name(ret));
        cc113l_deinit(cc113l_handle);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "CC113L initialized and configured for OOK scanning");
    
    while (1) {
        // Calculate maximum number of results with some extra buffer
        int max_results = (int)((SCAN_END_FREQ - SCAN_START_FREQ) / SCAN_STEP_FREQ) + 10;
        
        ESP_LOGI(TAG, "Allocating memory for %d scan results (range: %.1f-%.1f MHz, step: %.1f MHz)", 
                 max_results, SCAN_START_FREQ, SCAN_END_FREQ, SCAN_STEP_FREQ);
        
        // Allocate results array
        cc113l_scan_result_t *results = malloc(max_results * sizeof(cc113l_scan_result_t));
        if (results == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for scan results");
            break;
        }
        
        uint16_t actual_results = 0;
        
        ESP_LOGI(TAG, "Starting frequency scan...");
        ESP_LOGI(TAG, "NOTE: 416MHz has continuous activity (not your tag)");
        ESP_LOGI(TAG, "Looking for periodic activity on ~418MHz...");
        
        // Perform scan
        ret = cc113l_scan_range(cc113l_handle, 
                               SCAN_START_FREQ, SCAN_END_FREQ, SCAN_STEP_FREQ,
                               SAMPLES_PER_FREQ, DWELL_TIME_MS,
                               results, max_results, &actual_results);
        
        if (ret != ESP_OK && ret != ESP_ERR_NO_MEM) {
            ESP_LOGE(TAG, "Scan failed: %s", esp_err_to_name(ret));
            free(results);
            break;
        }
        
        if (ret == ESP_ERR_NO_MEM) {
            ESP_LOGW(TAG, "Scan was truncated due to memory limit, but continuing with available results");
        }
        
        ESP_LOGI(TAG, "=== SCAN COMPLETE ===");
        ESP_LOGI(TAG, "Scanned %d frequencies", actual_results);
        
        // Analyze results
        ESP_LOGI(TAG, "\n=== SIGNAL ANALYSIS ===");
        
        int continuous_signals = 0;
        int periodic_signals = 0;
        float best_target_freq = 0;
        int best_target_score = 0;
        
        for (int i = 0; i < actual_results; i++) {
            cc113l_scan_result_t *r = &results[i];
            
            if (r->carrier_sense_count > 0) {
                bool is_continuous = (r->carrier_sense_count > SAMPLES_PER_FREQ * 0.8);
                bool is_periodic = (r->carrier_sense_count > 0 && !is_continuous);
                
                if (is_continuous) {
                    continuous_signals++;
                    ESP_LOGI(TAG, "CONTINUOUS: %.1f MHz (RSSI=%ddBm, CS=%d%%)", 
                             r->frequency, r->max_rssi, (r->carrier_sense_count * 100) / SAMPLES_PER_FREQ);
                }
                else if (is_periodic) {
                    periodic_signals++;
                    
                    // Score potential targets (prefer frequencies near 418MHz with good signal)
                    int target_score = 0;
                    if (r->frequency >= 417.5 && r->frequency <= 418.5) target_score += 10;
                    if (r->max_rssi > -90) target_score += 5;
                    if (r->fifo_activity > 0) target_score += 3;
                    target_score += r->carrier_sense_count;
                    
                    ESP_LOGI(TAG, "PERIODIC: %.1f MHz (RSSI=%ddBm, CS=%d%%, Score=%d)", 
                             r->frequency, r->max_rssi, (r->carrier_sense_count * 100) / SAMPLES_PER_FREQ, target_score);
                    
                    if (target_score > best_target_score) {
                        best_target_score = target_score;
                        best_target_freq = r->frequency;
                    }
                }
            }
        }
        
        ESP_LOGI(TAG, "\n=== RECOMMENDATION ===");
        ESP_LOGI(TAG, "Found %d continuous signals, %d periodic signals", continuous_signals, periodic_signals);
        
        if (best_target_freq > 0) {
            ESP_LOGI(TAG, "*** BEST TARGET FREQUENCY: %.1f MHz (Score: %d) ***", best_target_freq, best_target_score);
            ESP_LOGI(TAG, "This is likely your 418MHz radio tag!");
            ESP_LOGI(TAG, "Configure receiver for %.1f MHz to receive your tag.", best_target_freq);
        } else {
            ESP_LOGI(TAG, "No clear periodic signals found near 418MHz.");
            ESP_LOGI(TAG, "Your tag might be:");
            ESP_LOGI(TAG, "1. Not transmitting during scan");
            ESP_LOGI(TAG, "2. Using different frequency than expected");
            ESP_LOGI(TAG, "3. Signal too weak to detect");
        }
        
        // Free allocated memory
        free(results);
        
        ESP_LOGI(TAG, "Scan finished. Restarting scan in 10 seconds...");
        vTaskDelay(pdMS_TO_TICKS(10000)); // Wait 10 seconds before restart
    }
    
    // Cleanup
    cc113l_deinit(cc113l_handle);
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG, "Starting CC113L Wide Frequency Scanner Example");
    ESP_LOGI(TAG, "Scanning range: %.1f - %.1f MHz", SCAN_START_FREQ, SCAN_END_FREQ);
    ESP_LOGI(TAG, "Step size: %.1f MHz", SCAN_STEP_FREQ);
    ESP_LOGI(TAG, "Samples per frequency: %d", SAMPLES_PER_FREQ);
    ESP_LOGI(TAG, "Total scan time: ~%.0f seconds", 
             ((SCAN_END_FREQ - SCAN_START_FREQ) / SCAN_STEP_FREQ) * (DWELL_TIME_MS * SAMPLES_PER_FREQ) / 1000.0);
    
    ESP_LOGI(TAG, "GPIO Configuration:");
    ESP_LOGI(TAG, "CS_PIN:   GPIO%d", CC113L_CS_PIN);
    ESP_LOGI(TAG, "MOSI_PIN: GPIO%d", CC113L_MOSI_PIN);
    ESP_LOGI(TAG, "MISO_PIN: GPIO%d", CC113L_MISO_PIN);
    ESP_LOGI(TAG, "SCK_PIN:  GPIO%d", CC113L_SCK_PIN);
    
    // Start scanner task
    xTaskCreate(frequency_scanner_task, "scanner_task", 16384, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "CC113L Wide Frequency Scanner started!");
    ESP_LOGI(TAG, "Looking for your 418MHz radio tag among all signals...");
}