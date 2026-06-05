/*
 * SR Pipeline — model loading, AFE, feed/fetch tasks, wake-word buffer, command dispatch
 */
#include "sr_handler.h"
#include "config.h"
#include "audio_codec.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_afe_sr_models.h"
#include "esp_afe_config.h"
#include "esp_mn_models.h"
extern "C" {
#include "esp_mn_speech_commands.h"
}

#include <cstdlib>
#include <cstring>
#include <deque>
#include <vector>
#include <mutex>

static const char *TAG = "SR";

// ---- internal state ----
static const esp_afe_sr_iface_t *s_afe_iface  = nullptr;
static esp_afe_sr_data_t       *s_afe_data    = nullptr;
static int                       s_feed_chunk  = 0;
static esp_codec_dev_handle_t    s_codec_dev   = nullptr;
static sr_command_callback_t     s_callback    = nullptr;

// ---- wake-word audio buffer (~2 seconds) ----
static std::deque<std::vector<int16_t>> s_ww_pcm;
static std::mutex s_ww_mutex;

static void store_wake_word(const int16_t *data, size_t samples) {
    std::lock_guard<std::mutex> lock(s_ww_mutex);
    s_ww_pcm.emplace_back(data, data + samples);
    while (s_ww_pcm.size() > 66) s_ww_pcm.pop_front();   // ~2s @ 16kHz / 30ms
}

// ---- feed task (core 0): codec -> AFE ----
static void feed_task(void *arg) {
    int channels = s_afe_iface->get_feed_channel_num(s_afe_data);
    int total    = s_feed_chunk * channels;
    int16_t *buf = (int16_t *)malloc(total * sizeof(int16_t));
    if (!buf) { ESP_LOGE(TAG, "feed buf alloc failed"); vTaskDelete(nullptr); return; }

    int freq     = s_afe_iface->get_samp_rate(s_afe_data);
    int period   = (s_feed_chunk * 1000) / freq;
    ESP_LOGI(TAG, "feed: chunk=%d ch=%d freq=%d period=%dms", s_feed_chunk, channels, freq, period);

    while (true) {
        int n = audio_codec_read(s_codec_dev, buf, total * sizeof(int16_t));
        if (n == total * (int)sizeof(int16_t)) {
            s_afe_iface->feed(s_afe_data, buf);
        } else if (n > 0) {
            ESP_LOGW(TAG, "partial read %d/%d", n, total * (int)sizeof(int16_t));
        }
        vTaskDelay(pdMS_TO_TICKS(period));
    }
    free(buf);
    vTaskDelete(nullptr);
}

// ---- fetch task (core 1): AFE -> MultiNet + logging + wake-word buffer ----
static void fetch_task(void *arg) {
    int chunk = s_afe_iface->get_fetch_chunksize(s_afe_data);
    ESP_LOGI(TAG, "fetch: chunk=%d samples", chunk);

    bool     was_speech = false;
    uint32_t cnt        = 0;

    while (true) {
        afe_fetch_result_t *res = s_afe_iface->fetch_with_delay(s_afe_data, portMAX_DELAY);
        if (!res || res->ret_value == ESP_FAIL) continue;

        // store wake-word audio
        if (res->data && res->data_size > 0)
            store_wake_word(res->data, res->data_size / sizeof(int16_t));

        // periodic log (xiaoXiong_4G style)
        cnt++;
        if (cnt >= 32 || res->wakeup_state == WAKENET_DETECTED) {
            cnt = 0;
            ESP_LOGI(TAG, "state: vol=%.1f vad=%d wake=%d model=%d ring=%.2f",
                     res->data_volume, res->vad_state, res->wakeup_state,
                     res->wakenet_model_index, res->ringbuff_free_pct);
        }

        // VAD -> clean MultiNet on speech start
        bool speech = (res->vad_state == VAD_SPEECH);
        if (speech && !was_speech)  multinet_clean();
        was_speech = speech;

        // feed MultiNet
        if (res->data && res->data_size > 0)
            multinet_detect(res->data, res->data_size / sizeof(int16_t));

        if (cnt % 10 == 0) vTaskDelay(1);
    }
    vTaskDelete(nullptr);
}

// ---- MultiNet -> user callback ----
static void on_mn_detected(int id, const char *phrase, float prob) {
    ESP_LOGI(TAG, "=== COMMAND: [%d] '%s' (%.3f) ===", id, phrase, prob);
    if (s_callback) s_callback(id, phrase, prob);
}

// ================================================================
// Public API
// ================================================================
int sr_handler_init(esp_codec_dev_handle_t codec_dev,
                    const mn_command_t *commands, int command_count,
                    sr_command_callback_t callback) {
    s_codec_dev = codec_dev;
    s_callback  = callback;

    // ---- 1. load models once ----
    srmodel_list_t *models = esp_srmodel_init(SR_MODEL_PARTITION);
    if (!models || models->num <= 0) {
        ESP_LOGE(TAG, "no models in '%s'", SR_MODEL_PARTITION);
        return -1;
    }
    for (int i = 0; i < models->num; i++)
        ESP_LOGI(TAG, "model[%d]: %s", i, models->model_name[i]);

    // ---- 2. AFE config (xiaoXiong_4G: HIGH_PERF + PSRAM) ----
    int  ref_num = AUDIO_INPUT_REFERENCE ? 1 : 0;
    char fmt[16] = {};
    int  pos     = 0;
    for (int i = 0; i < AUDIO_INPUT_CHANNELS; i++) fmt[pos++] = 'M';
    for (int i = 0; i < ref_num; i++)               fmt[pos++] = 'R';
    ESP_LOGI(TAG, "AFE format: '%s'", fmt);

    afe_config_t *cfg = afe_config_init(fmt, models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    cfg->aec_init               = AUDIO_INPUT_REFERENCE;
    if (cfg->aec_init) cfg->aec_mode = AEC_MODE_SR_HIGH_PERF;
    cfg->se_init                = false;
    cfg->ns_init                = true;
    cfg->agc_init               = true;
    cfg->vad_init               = true;
    cfg->wakenet_init           = true;
    cfg->afe_perferred_core     = 1;
    cfg->afe_perferred_priority = 1;
    cfg->memory_alloc_mode      = AFE_MEMORY_ALLOC_MORE_PSRAM;
    cfg = afe_config_check(cfg);
    cfg->ns_init  = true;
    cfg->agc_init = true;

    ESP_LOGI(TAG, "AFE: ns=%d agc=%d aec=%d vad=%d wake=%d gain=%.1f",
             cfg->ns_init, cfg->agc_init, cfg->aec_init,
             cfg->vad_init, cfg->wakenet_init, cfg->afe_linear_gain);

    s_afe_iface = esp_afe_handle_from_config(cfg);
    s_afe_data  = s_afe_iface ? s_afe_iface->create_from_config(cfg) : nullptr;
    if (!s_afe_data) { ESP_LOGE(TAG, "AFE create failed"); afe_config_free(cfg); return -1; }
    s_afe_iface->print_pipeline(s_afe_data);
    s_feed_chunk = s_afe_iface->get_feed_chunksize(s_afe_data);
    afe_config_free(cfg);

    // ---- 3. MultiNet init (reuse loaded models) ----
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    if (!mn_name) {
        ESP_LOGE(TAG, "no Chinese MultiNet model");
        return -1;
    }
    ESP_LOGI(TAG, "MultiNet model: %s", mn_name);

    const esp_mn_iface_t *mn = esp_mn_handle_from_name(mn_name);
    if (!mn) { ESP_LOGE(TAG, "MultiNet handle failed"); return -1; }

    model_iface_data_t *mn_data = mn->create(mn_name, COMMAND_TIMEOUT_MS);
    if (!mn_data) { ESP_LOGE(TAG, "MultiNet create failed"); return -1; }

    mn->set_det_threshold(mn_data, COMMAND_DET_THRESHOLD);

    esp_mn_commands_alloc(mn, mn_data);
    for (int i = 0; i < command_count; i++)
        esp_mn_commands_add(commands[i].command_id, commands[i].phrase);
    esp_mn_error_t *err = esp_mn_commands_update();
    if (err) { ESP_LOGE(TAG, "MultiNet cmd update: %d errors", err->num); esp_mn_commands_free(); return -1; }
    esp_mn_commands_print();

    // Wire MultiNet into C handlers (for detect / get_result)
    multinet_set_internal(mn, mn_data, commands, command_count);
    multinet_set_callback(on_mn_detected);

    // ---- 4. start tasks ----
    xTaskCreatePinnedToCore(feed_task,  "feed",  FEED_TASK_STACK_SIZE,  nullptr, FEED_TASK_PRIO,  nullptr, 0);
    xTaskCreatePinnedToCore(fetch_task, "fetch", FETCH_TASK_STACK_SIZE, nullptr, FETCH_TASK_PRIO, nullptr, 1);

    ESP_LOGI(TAG, "SR pipeline ready (models=%d, feed=%d, fetch=%d)",
             models->num, s_feed_chunk, s_afe_iface->get_fetch_chunksize(s_afe_data));
    return 0;
}

int sr_handler_get_wake_word(int16_t **data, int *len) {
    std::lock_guard<std::mutex> lock(s_ww_mutex);
    if (s_ww_pcm.empty()) return -1;

    int total = 0;
    for (auto &v : s_ww_pcm) total += v.size();
    int16_t *buf = (int16_t *)malloc(total * sizeof(int16_t));
    if (!buf) return -1;

    int off = 0;
    for (auto &v : s_ww_pcm) {
        memcpy(buf + off, v.data(), v.size() * sizeof(int16_t));
        off += v.size();
    }
    *data = buf;
    *len  = total;
    return 0;
}

void sr_handler_deinit(void) {
    // tasks are infinite loops; not joinable. Just destroy AFE.
    if (s_afe_data) { s_afe_iface->destroy(s_afe_data); s_afe_data = nullptr; }
    multinet_deinit();
}
