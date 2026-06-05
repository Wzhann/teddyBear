/*
 * TeddyBear — Hardware & System Configuration
 * ESP32-S3 + ES8311 + ESP-SR (MultiNet command recognition)
 *
 * Pin mapping follows xiaoXiong_4G reference board.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "driver/gpio.h"
#include "driver/i2c_types.h"
#include "driver/i2s_types.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// 1. I2C — ES8311 codec control
// ============================================================
#define I2C_MASTER_PORT         I2C_NUM_0
#define I2C_MASTER_SDA_PIN      GPIO_NUM_2
#define I2C_MASTER_SCL_PIN      GPIO_NUM_38
#define I2C_MASTER_FREQ_HZ      100000

// ============================================================
// 2. I2S — ES8311 audio data (duplex)
// ============================================================
#define I2S_PORT                I2S_NUM_0
#define I2S_MCLK_PIN            GPIO_NUM_NC   // ES8311 slave mode, no MCLK
#define I2S_WS_PIN              GPIO_NUM_13
#define I2S_BCLK_PIN            GPIO_NUM_48
#define I2S_DIN_PIN             GPIO_NUM_14   // Mic input
#define I2S_DOUT_PIN            GPIO_NUM_46   // Speaker output

// ============================================================
// 3. ES8311 codec parameters
// ============================================================
#define ES8311_I2C_ADDR         0x30
#define AUDIO_SAMPLE_RATE       24000
#define AUDIO_INPUT_CHANNELS    1             // Mic channels (ref handled by codec)
#define AUDIO_OUTPUT_CHANNELS   1
#define AUDIO_BITS_PER_SAMPLE   16
#define AUDIO_INPUT_REFERENCE   true          // Enable AEC via echo reference

// ============================================================
// 4. UART — serial link to STM32 lower MCU
//     Protocol: 9-byte frames, headers 0x4141/0x4242, sum8 checksum
// ============================================================
#define UART_PORT_NUM           UART_NUM_2
#define UART_TX_PIN             GPIO_NUM_43      // ESP32 TX -> STM32 RX
#define UART_RX_PIN             GPIO_NUM_44      // ESP32 RX <- STM32 TX
#define UART_BAUD_RATE          115200
#define UART_RX_BUF_SIZE        2048
#define UART_TX_BUF_SIZE        1024

// ============================================================
// 5. ESP-SR parameters
// ============================================================
#define SR_MODEL_PARTITION      "model"
#define COMMAND_TIMEOUT_MS      6000
#define COMMAND_DET_THRESHOLD   0.15f

// ============================================================
// 6. FreeRTOS task stack sizes & priorities
// ============================================================
#define FEED_TASK_STACK_SIZE    4096
#define FEED_TASK_PRIO          5
#define FETCH_TASK_STACK_SIZE   4096
#define FETCH_TASK_PRIO         5
#define UART_RX_STACK_SIZE      3072
#define UART_RX_TASK_PRIO       2

#ifdef __cplusplus
}
#endif

#endif // CONFIG_H
