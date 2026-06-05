/*
 * SR (Speech Recognition) Pipeline Handler
 * Encapsulates: model loading, AFE, feed/fetch tasks, wake-word buffer, command dispatch
 *
 * Reference: xiaoXiong_4G AfeWakeWord + esp-sr-wzh
 */

#ifndef SR_HANDLER_H
#define SR_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_codec_dev.h"
#include "multinet_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Command action callback — called when a command is recognized
 * @param command_id  ID of the detected command
 * @param phrase      Pinyin phrase
 * @param probability Detection confidence (0.0 - 1.0)
 */
typedef void (*sr_command_callback_t)(int command_id, const char *phrase, float probability);

/**
 * @brief Initialize the full SR pipeline:
 *        1. Load models from flash
 *        2. Init AFE (HIGH_PERF, PSRAM, NS+VAD+AEC+AGC)
 *        3. Init MultiNet with commands
 *        4. Start feed & fetch FreeRTOS tasks
 *
 * @param codec_dev      Initialized ES8311 codec handle
 * @param commands       Array of pinyin command definitions
 * @param command_count  Number of commands
 * @param callback       Called when a command is detected (can be NULL)
 * @return 0 on success, -1 on failure
 */
int sr_handler_init(esp_codec_dev_handle_t codec_dev,
                    const mn_command_t *commands, int command_count,
                    sr_command_callback_t callback);

/**
 * @brief Get access to the stored wake-word PCM audio (~2 seconds)
 *
 * @param[out] data  Pointer to flattened int16_t buffer (caller must free)
 * @param[out] len   Number of samples
 * @return 0 on success, -1 if no data
 */
int sr_handler_get_wake_word(int16_t **data, int *len);

/**
 * @brief Deinitialize SR pipeline and free resources
 */
void sr_handler_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // SR_HANDLER_H
