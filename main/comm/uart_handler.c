/*
 * UART Serial Driver — TX/RX with callback
 *
 * RX runs in a FreeRTOS task, forwarding raw bytes to the callback.
 * TX provides uart_send / uart_send_string / uart_printf.
 */

#include "uart_handler.h"
#include "config.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>
#include <stdarg.h>
#include <stdio.h>

static const char *TAG = "UartHandler";

static uart_rx_callback_t s_rx_callback = NULL;
static TaskHandle_t s_rx_task = NULL;
static int s_uart_port = -1;

static void uart_rx_task(void *arg) {
    uint8_t rx_data[256];
    ESP_LOGI(TAG, "UART RX task started on port %d", s_uart_port);

    while (true) {
        int len = uart_read_bytes(s_uart_port, rx_data, sizeof(rx_data),
                                  pdMS_TO_TICKS(100));
        if (len > 0 && s_rx_callback != NULL) {
            s_rx_callback(rx_data, len);
        }
    }
}

int uart_init(const uart_cfg_t *cfg) {
    if (cfg == NULL) {
        return -1;
    }

    s_uart_port = cfg->port;

    // Configure UART parameters
    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(cfg->port, &uart_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return -1;
    }

    // Set pins
    ret = uart_set_pin(cfg->port, cfg->tx_pin, cfg->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return -1;
    }

    // Install UART driver
    ret = uart_driver_install(cfg->port, cfg->rx_buf_size, cfg->tx_buf_size,
                              0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return -1;
    }

    // Create RX task
    BaseType_t task_ret = xTaskCreate(
        uart_rx_task,
        "uart_rx",
        UART_RX_STACK_SIZE,
        NULL,
        UART_RX_TASK_PRIO,
        &s_rx_task
    );
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create UART RX task");
        uart_driver_delete(cfg->port);
        return -1;
    }

    ESP_LOGI(TAG, "UART initialized: port=%d, TX=%d, RX=%d, baud=%d",
             cfg->port, cfg->tx_pin, cfg->rx_pin, cfg->baud_rate);
    return 0;
}

int uart_send(const uint8_t *data, size_t len) {
    if (s_uart_port < 0 || data == NULL || len == 0) {
        return -1;
    }
    int sent = uart_write_bytes(s_uart_port, data, len);
    return sent;
}

int uart_send_string(const char *str) {
    if (s_uart_port < 0 || str == NULL) {
        return -1;
    }
    size_t len = strlen(str);
    return uart_write_bytes(s_uart_port, str, len);
}

int uart_printf(const char *format, ...) {
    if (s_uart_port < 0 || format == NULL) {
        return -1;
    }

    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    if (len > 0) {
        uart_write_bytes(s_uart_port, buf, len);
    }
    return len;
}

void uart_set_rx_callback(uart_rx_callback_t callback) {
    s_rx_callback = callback;
}

void uart_deinit(void) {
    if (s_rx_task != NULL) {
        vTaskDelete(s_rx_task);
        s_rx_task = NULL;
    }

    if (s_uart_port >= 0) {
        uart_driver_delete(s_uart_port);
        s_uart_port = -1;
    }

    ESP_LOGI(TAG, "UART deinitialized");
}
