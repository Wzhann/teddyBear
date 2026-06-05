/*
 * MultiNet Command Word Recognition Handler
 * Uses ESP-SR MultiNet for Chinese speech command recognition
 *
 * Reference: /home/wzh/program/ESP32/esp-sr-wzh/main/main.c
 */

#include "multinet_handler.h"
#include "esp_log.h"
#include "esp_mn_iface.h"

static const char *TAG = "MultiNet";

// Internal state — set by sr_handler via multinet_set_internal()
static const esp_mn_iface_t *s_multinet = NULL;
static model_iface_data_t    *s_mn_data = NULL;
static mn_callback_t          s_callback = NULL;
static const mn_command_t    *s_commands = NULL;
static int                    s_command_count = 0;

int multinet_detect(const int16_t *data, int len) {
    if (s_multinet == NULL || s_mn_data == NULL || data == NULL) {
        return -1;
    }

    esp_mn_state_t state = s_multinet->detect(s_mn_data, (int16_t *)data);

    if (state == ESP_MN_STATE_DETECTED) {
        esp_mn_results_t *results = s_multinet->get_results(s_mn_data);
        if (results != NULL && results->num > 0) {
            int cmd_id = results->command_id[0];
            float prob = results->prob[0];
            const char *phrase = (cmd_id >= 0 && cmd_id < s_command_count)
                                 ? s_commands[cmd_id].phrase : "unknown";

            ESP_LOGI(TAG, "*** COMMAND DETECTED: id=%d phrase='%s' prob=%.3f ***",
                     cmd_id, phrase, prob);

            if (s_callback != NULL) {
                s_callback(cmd_id, phrase, prob);
            }
        }
        s_multinet->clean(s_mn_data);
    } else if (state == ESP_MN_STATE_TIMEOUT) {
        ESP_LOGD(TAG, "MultiNet timeout, cleaning");
        s_multinet->clean(s_mn_data);
    }

    return (int)state;
}

int multinet_get_result(int *command_id, const char **phrase, float *probability) {
    if (s_multinet == NULL || s_mn_data == NULL) {
        return -1;
    }

    esp_mn_results_t *results = s_multinet->get_results(s_mn_data);
    if (results == NULL || results->num <= 0) {
        return -1;
    }

    if (command_id) *command_id = results->command_id[0];
    if (probability) *probability = results->prob[0];
    if (phrase && results->command_id[0] >= 0 && results->command_id[0] < s_command_count) {
        *phrase = s_commands[results->command_id[0]].phrase;
    }

    return 0;
}

void multinet_set_internal(const void *mn, void *mn_data,
                           const mn_command_t *commands, int command_count) {
    s_multinet      = (const esp_mn_iface_t *)mn;
    s_mn_data       = (model_iface_data_t *)mn_data;
    s_commands      = commands;
    s_command_count = command_count;
}

void multinet_set_callback(mn_callback_t callback) {
    s_callback = callback;
}

void multinet_clean(void) {
    if (s_multinet != NULL && s_mn_data != NULL) {
        s_multinet->clean(s_mn_data);
    }
}

void multinet_deinit(void) {
    if (s_multinet != NULL && s_mn_data != NULL) {
        s_multinet->destroy(s_mn_data);
        s_mn_data = NULL;
    }
    s_multinet = NULL;
    s_commands = NULL;
    s_command_count = 0;
    ESP_LOGI(TAG, "MultiNet deinitialized");
}
