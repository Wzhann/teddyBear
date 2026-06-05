/*
 * ES8311 Audio Codec Driver
 * Uses esp_codec_dev framework for I2C control + I2S data
 */

#ifndef AUDIO_CODEC_H
#define AUDIO_CODEC_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_codec_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Audio codec configuration parameters
 */
typedef struct {
    int sample_rate;        // Audio sample rate in Hz
    int input_channels;     // Number of input (mic) channels
    int output_channels;    // Number of output (speaker) channels
    int bits_per_sample;    // Bits per sample (typically 16)
    bool input_reference;   // Enable echo reference on input
} audio_codec_cfg_t;

/**
 * @brief Initialize the ES8311 audio codec
 *
 * Sets up I2C master bus for codec control, I2S for audio data,
 * and creates the codec device handle.
 *
 * @param cfg  Audio configuration parameters
 * @return esp_codec_dev_handle_t on success, NULL on failure
 */
esp_codec_dev_handle_t audio_codec_init(const audio_codec_cfg_t *cfg);

/**
 * @brief Read audio data from the codec (microphone input)
 *
 * @param dev   Codec device handle
 * @param data  Buffer to store read data
 * @param len   Number of bytes to read
 * @return Number of bytes read, or negative on error
 */
int audio_codec_read(esp_codec_dev_handle_t dev, void *data, int len);

/**
 * @brief Write audio data to the codec (speaker output)
 *
 * @param dev   Codec device handle
 * @param data  Buffer containing data to write
 * @param len   Number of bytes to write
 * @return Number of bytes written, or negative on error
 */
int audio_codec_write(esp_codec_dev_handle_t dev, void *data, int len);

/**
 * @brief Set the output volume
 *
 * @param dev     Codec device handle
 * @param volume  Volume level (0-100)
 * @return 0 on success
 */
int audio_codec_set_volume(esp_codec_dev_handle_t dev, int volume);

/**
 * @brief Set the input gain
 *
 * @param dev       Codec device handle
 * @param gain_db   Gain in dB
 * @return 0 on success
 */
int audio_codec_set_input_gain(esp_codec_dev_handle_t dev, float gain_db);

/**
 * @brief Close and delete the codec device, free resources
 *
 * @param dev  Codec device handle
 */
void audio_codec_delete(esp_codec_dev_handle_t dev);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_CODEC_H
