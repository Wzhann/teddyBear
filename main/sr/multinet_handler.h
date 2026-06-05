/*
 * MultiNet Command Word Recognition Handler
 * Uses ESP-SR MultiNet for Chinese speech command recognition
 */

#ifndef MULTINET_HANDLER_H
#define MULTINET_HANDLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Command definition structure
 */
typedef struct {
    int command_id;         // Unique command ID
    const char *phrase;     // Pinyin phrase (e.g., "da kai dian deng")
} mn_command_t;

/**
 * @brief MultiNet detection result callback
 * @param command_id  ID of the detected command
 * @param phrase      Pinyin phrase of the detected command
 * @param probability Detection confidence (0.0 - 1.0)
 */
typedef void (*mn_callback_t)(int command_id, const char *phrase, float probability);

/**
 * @brief Inject a pre-created MultiNet instance.
 *        Called by sr_handler after loading models and registering commands.
 */
void multinet_set_internal(const void *mn, void *mn_data,
                           const mn_command_t *commands, int command_count);

/**
 * @brief Feed audio data to MultiNet for detection
 *
 * @param data  Audio samples (int16_t, mono, 16kHz)
 * @param len   Number of samples
 * @return Detection state:
 *         0 = detecting (ESP_MN_STATE_DETECTING)
 *         1 = command detected (ESP_MN_STATE_DETECTED)
 *         2 = timeout (ESP_MN_STATE_TIMEOUT)
 */
int multinet_detect(const int16_t *data, int len);

/**
 * @brief Get detailed results after detection
 *
 * @param[out] command_id   Detected command ID
 * @param[out] phrase       Detected command phrase
 * @param[out] probability  Detection confidence
 * @return 0 on success, -1 if no result available
 */
int multinet_get_result(int *command_id, const char **phrase, float *probability);

/**
 * @brief Set callback for command detection
 */
void multinet_set_callback(mn_callback_t callback);

/**
 * @brief Reset/clean MultiNet state (call at speech boundaries)
 */
void multinet_clean(void);

/**
 * @brief Clean up MultiNet resources
 */
void multinet_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // MULTINET_HANDLER_H
