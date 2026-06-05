/*
 * ES8311 Audio Codec Driver
 * Uses esp_codec_dev framework for I2C control + I2S data
 */

#include "audio_codec.h"
#include "config.h"
#include "esp_log.h"
#include "esp_codec_dev_defaults.h"
#include "audio_codec_ctrl_if.h"
#include "audio_codec_data_if.h"
#include "audio_codec_gpio_if.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"

static const char *TAG = "AudioCodec";

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2s_chan_handle_t s_i2s_tx_chan = NULL;  // Speaker output
static i2s_chan_handle_t s_i2s_rx_chan = NULL;  // Mic input

/**
 * @brief Initialize I2C master bus for ES8311 control
 */
static esp_err_t init_i2c_bus(void) {
    i2c_master_bus_config_t i2c_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_PORT,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    esp_err_t ret = i2c_new_master_bus(&i2c_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C master bus initialized (SDA=%d, SCL=%d)",
             I2C_MASTER_SDA_PIN, I2C_MASTER_SCL_PIN);
    return ESP_OK;
}

/**
 * @brief Initialize I2S channels for audio data (duplex: TX + RX)
 */
static esp_err_t init_i2s_channels(const audio_codec_cfg_t *cfg) {
    // ---- I2S RX channel (microphone input) ----
    i2s_chan_config_t rx_chan_cfg = {
        .id = I2S_PORT,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear = false,
    };
    esp_err_t ret = i2s_new_channel(&rx_chan_cfg, NULL, &s_i2s_rx_chan);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S RX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = cfg->sample_rate,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
#if SOC_I2S_HW_VERSION_1
            .msb_right = false,
#else
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
#endif
        },
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_BCLK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,  // Not used for RX, but needs valid pin
            .din = I2S_DIN_PIN,
        },
    };
    ret = i2s_channel_init_std_mode(s_i2s_rx_chan, &rx_std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S RX std mode: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2S RX channel initialized");

    // ---- I2S TX channel (speaker output) ----
    i2s_chan_config_t tx_chan_cfg = {
        .id = I2S_PORT,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 6,
        .dma_frame_num = 240,
        .auto_clear = false,
    };
    ret = i2s_new_channel(&tx_chan_cfg, &s_i2s_tx_chan, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2S TX channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_config_t tx_std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = cfg->sample_rate,
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
            .slot_mode = I2S_SLOT_MODE_MONO,
            .slot_mask = I2S_STD_SLOT_LEFT,
            .ws_width = I2S_DATA_BIT_WIDTH_16BIT,
            .ws_pol = false,
            .bit_shift = true,
#if SOC_I2S_HW_VERSION_1
            .msb_right = false,
#else
            .left_align = false,
            .big_endian = false,
            .bit_order_lsb = false,
#endif
        },
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_BCLK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din = I2S_DIN_PIN,    // Not used for TX, but needs valid pin
        },
    };
    ret = i2s_channel_init_std_mode(s_i2s_tx_chan, &tx_std_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init I2S TX std mode: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2S TX channel initialized");

    return ESP_OK;
}

esp_codec_dev_handle_t audio_codec_init(const audio_codec_cfg_t *cfg) {
    ESP_LOGI(TAG, "Initializing ES8311 audio codec...");
    ESP_LOGI(TAG, "Sample rate: %d Hz, Input channels: %d, Output channels: %d",
             cfg->sample_rate, cfg->input_channels, cfg->output_channels);

    // 1. Initialize I2C bus
    esp_err_t ret = init_i2c_bus();
    if (ret != ESP_OK) {
        return NULL;
    }

    // 2. Initialize I2S channels
    ret = init_i2s_channels(cfg);
    if (ret != ESP_OK) {
        return NULL;
    }

    // 3. Create I2C control interface for codec
    audio_codec_i2c_cfg_t i2c_ctrl_cfg = {
        .port = I2C_MASTER_PORT,
        .addr = ES8311_I2C_ADDR,
        .bus_handle = s_i2c_bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_ctrl_cfg);
    if (ctrl_if == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C control interface");
        return NULL;
    }

    // 4. Create GPIO interface (no PA pin)
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    if (gpio_if == NULL) {
        ESP_LOGE(TAG, "Failed to create GPIO interface");
        return NULL;
    }

    // 5. Create I2S data interface
    audio_codec_i2s_cfg_t i2s_data_cfg = {
        .port = I2S_PORT,
        .rx_handle = s_i2s_rx_chan,
        .tx_handle = s_i2s_tx_chan,
        .clk_src = (int)I2S_CLK_SRC_DEFAULT,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_data_cfg);
    if (data_if == NULL) {
        ESP_LOGE(TAG, "Failed to create I2S data interface");
        return NULL;
    }

    // 6. Configure ES8311 codec
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,  // Duplex: ADC + DAC
        .pa_pin = -1,           // No external PA
        .pa_reverted = false,
        .master_mode = false,   // ESP32 is I2S master
        .use_mclk = false,      // No MCLK - ES8311 slave
        .digital_mic = false,   // Analog microphone
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain = {
            .pa_voltage = 3300, // mV
            .codec_dac_voltage = 3300,
        },
        .no_dac_ref = true,     // Reference channel empty (no DAC loopback)
        .mclk_div = 256,
    };

    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);
    if (codec_if == NULL) {
        ESP_LOGE(TAG, "Failed to create ES8311 codec interface");
        return NULL;
    }

    // 7. Create codec device
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,  // Duplex device
        .codec_if = codec_if,
        .data_if = data_if,
    };
    esp_codec_dev_handle_t dev = esp_codec_dev_new(&dev_cfg);
    if (dev == NULL) {
        ESP_LOGE(TAG, "Failed to create codec device");
        return NULL;
    }

    // 8. Open the codec device
    esp_codec_dev_sample_info_t sample_info = {
        .bits_per_sample = (uint8_t)cfg->bits_per_sample,
        .channel = (uint8_t)cfg->output_channels,  // For DAC/playback
        .channel_mask = ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0),  // Channel 0
        .sample_rate = (uint32_t)cfg->sample_rate,
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    };

    ret = esp_codec_dev_open(dev, &sample_info);
    if (ret != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "Failed to open codec device: %d", ret);
        esp_codec_dev_delete(dev);
        return NULL;
    }

    // 9. Set initial volume and gain
    esp_codec_dev_set_out_vol(dev, 60);  // Default 60% volume
    esp_codec_dev_set_in_gain(dev, 30.0f);  // 30dB input gain (like xiaoXiong_4G)

    ESP_LOGI(TAG, "ES8311 audio codec initialized successfully");
    return dev;
}

int audio_codec_read(esp_codec_dev_handle_t dev, void *data, int len) {
    if (dev == NULL) {
        return -1;
    }
    return esp_codec_dev_read(dev, data, len);
}

int audio_codec_write(esp_codec_dev_handle_t dev, void *data, int len) {
    if (dev == NULL) {
        return -1;
    }
    return esp_codec_dev_write(dev, data, len);
}

int audio_codec_set_volume(esp_codec_dev_handle_t dev, int volume) {
    if (dev == NULL) {
        return -1;
    }
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    return esp_codec_dev_set_out_vol(dev, volume);
}

int audio_codec_set_input_gain(esp_codec_dev_handle_t dev, float gain_db) {
    if (dev == NULL) {
        return -1;
    }
    return esp_codec_dev_set_in_gain(dev, gain_db);
}

void audio_codec_delete(esp_codec_dev_handle_t dev) {
    if (dev != NULL) {
        esp_codec_dev_close(dev);
        esp_codec_dev_delete(dev);
        ESP_LOGI(TAG, "Codec device deleted");
    }

    if (s_i2s_rx_chan != NULL) {
        i2s_channel_disable(s_i2s_rx_chan);
        i2s_del_channel(s_i2s_rx_chan);
        s_i2s_rx_chan = NULL;
    }

    if (s_i2s_tx_chan != NULL) {
        i2s_channel_disable(s_i2s_tx_chan);
        i2s_del_channel(s_i2s_tx_chan);
        s_i2s_tx_chan = NULL;
    }

    if (s_i2c_bus != NULL) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        ESP_LOGI(TAG, "I2C bus deleted");
    }
}
