/*
 * UART Serial Communication Handler
 * Provides TX and RX over UART2 with callback-based reception
 */

#ifndef UART_HANDLER_H
#define UART_HANDLER_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief UART receive callback type
 * @param data   Pointer to received data
 * @param len    Length of received data in bytes
 */
typedef void (*uart_rx_callback_t)(const uint8_t *data, size_t len);

/**
 * @brief UART configuration structure
 */
typedef struct {
    int port;           // UART port number
    int tx_pin;         // TX GPIO pin
    int rx_pin;         // RX GPIO pin
    int baud_rate;      // Baud rate
    int rx_buf_size;    // RX buffer size
    int tx_buf_size;    // TX buffer size
} uart_cfg_t;

/**
 * @brief Initialize UART for serial communication
 *
 * Configures UART with the given parameters and starts the RX task.
 *
 * @param cfg  UART configuration
 * @return 0 on success, -1 on failure
 */
int uart_init(const uart_cfg_t *cfg);

/**
 * @brief Send data over UART
 *
 * @param data  Data buffer to send
 * @param len   Number of bytes to send
 * @return Number of bytes sent, or -1 on error
 */
int uart_send(const uint8_t *data, size_t len);

/**
 * @brief Send a null-terminated string over UART
 *
 * @param str  String to send
 * @return Number of bytes sent, or -1 on error
 */
int uart_send_string(const char *str);

/**
 * @brief Send formatted text over UART (printf-like)
 *
 * @param format  Format string
 * @param ...     Arguments
 * @return Number of bytes sent, or -1 on error
 */
int uart_printf(const char *format, ...) __attribute__((format(printf, 1, 2)));

/**
 * @brief Set callback for received data
 *
 * @param callback  Function to call when data is received
 */
void uart_set_rx_callback(uart_rx_callback_t callback);

/**
 * @brief Deinitialize UART and free resources
 */
void uart_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // UART_HANDLER_H
