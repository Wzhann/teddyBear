/*
 * TeddyBear — Application entry point
 * ESP32-S3 + ES8311 + ESP-SR + STM32 Protocol (TeddyBear)
 *
 * UART2 links ESP32 (master) to STM32 (slave) with 9-byte frames.
 */
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "config.h"
#include "audio_codec.h"
#include "sr_handler.h"
#include "uart_handler.h"
#include "protocol_handler.h"

static const char *TAG = "app";

// ---- commands ----
static const mn_command_t s_commands[] = {
    {0, "da kai dian deng"},
    {1, "guan bi dian deng"},
    {2, "da kai kong tiao"},
    {3, "guan bi kong tiao"},
    {4, "da kai feng shan"},
    {5, "guan bi feng shan"},
    {6, "yao yao tou"},
    {7, "yao wei ba"},
};

// ---- voice command -> robot action (fill in as needed) ----
static void on_command(int id, const char *phrase, float prob) {
    ESP_LOGI(TAG, "=== COMMAND: [%d] '%s' (%.3f) ===", id, phrase, prob);
    // TODO: map command IDs to robot actions, e.g.:
    //   proto_send_play_action(x);
    //   proto_send_head_angle(h, v);
    //   proto_send_emotion(state, level);
}

// ---- STM32 response callbacks ----
static void on_sensor(bool head, bool body, bool chin, bool ha, bool hb) {
    ESP_LOGI(TAG, "sensors: head=%d body=%d chin=%d human_a=%d human_b=%d", head, body, chin, ha, hb);
}

static void on_status(int mode, int battery, bool charging, int error, int pose, int t1, int t2, int e2) {
    ESP_LOGI(TAG, "status: mode=%d batt=%d%% charging=%d err=%d pose=%d temp=%d/%d servo_err=%d",
             mode, battery, charging, error, pose, t1, t2, e2);
}

static void on_response(uint8_t func, uint8_t result) {
    ESP_LOGI(TAG, "STM32 response: func=0x%02X result=%d", func, result);
}

// ---- UART RX forwarded to protocol parser ----
static void on_uart_rx(const uint8_t *data, size_t len) {
    proto_feed_bytes(data, len);
}

// ---- entry ----
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== TeddyBear SR + STM32 Protocol ===");
    ESP_LOGI(TAG, "chip=%s free_heap=%" PRIu32, CONFIG_IDF_TARGET, esp_get_free_heap_size());

    // NVS
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase(); nvs_flash_init();
    }

    // UART
    uart_cfg_t uc = {UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_BAUD_RATE, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE};
    uart_init(&uc);
    uart_set_rx_callback(on_uart_rx);

    // Protocol
    proto_init();
    proto_set_sensor_cb(on_sensor);
    proto_set_status_cb(on_status);
    proto_set_response_cb(on_response);

    // Audio codec
    audio_codec_cfg_t ac = {AUDIO_SAMPLE_RATE, AUDIO_INPUT_CHANNELS, AUDIO_OUTPUT_CHANNELS, AUDIO_BITS_PER_SAMPLE, AUDIO_INPUT_REFERENCE};
    esp_codec_dev_handle_t dev = audio_codec_init(&ac);
    if (!dev) { ESP_LOGE(TAG, "FATAL: codec"); while (1) vTaskDelay(1000); }

    // SR pipeline
    if (sr_handler_init(dev, s_commands, sizeof(s_commands) / sizeof(s_commands[0]), on_command) != 0) {
        ESP_LOGE(TAG, "FATAL: SR"); while (1) vTaskDelay(1000);
    }

    ESP_LOGI(TAG, "Ready — speak a command");

    // Heartbeat: periodic status poll
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        proto_send_query_status();
    }
}
