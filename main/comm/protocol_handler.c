/*
 * TeddyBear STM32 <-> ESP32 Communication Protocol
 *
 * Reference: STM32 PandaRobot/Users/user_communication.c
 */
#include "protocol_handler.h"
#include "config.h"
#include "uart_handler.h"
#include "esp_log.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "Proto";

// ---- internal state ----
static proto_sensor_cb_t   s_sensor_cb   = NULL;
static proto_status_cb_t   s_status_cb   = NULL;
static proto_head_cb_t     s_head_cb     = NULL;
static proto_version_cb_t  s_version_cb  = NULL;
static proto_response_cb_t s_response_cb = NULL;
static proto_raw_cb_t      s_raw_cb      = NULL;

// ---- RX frame assembly buffer ----
static uint8_t  s_rx_buf[PROTO_MAX_DATA_LEN + 9];
static uint16_t s_rx_idx = 0;

// ---- helpers ----
static uint16_t calc_checksum(const uint8_t *data, uint16_t len) {
    uint16_t sum = 0;
    while (len--) sum += *data++;
    return sum;
}

static bool validate_frame(const proto_frame_t *f, uint16_t raw_len, uint16_t expected_header, uint16_t expected_footer) {
    if (raw_len < PROTO_MIN_FRAME_LEN) return false;
    if (f->header != expected_header) return false;
    if (f->footer != expected_footer) return false;
    if (f->data_len > PROTO_MAX_DATA_LEN) return false;
    if (raw_len != (uint16_t)(PROTO_MIN_FRAME_LEN + f->data_len)) return false;
    uint16_t ck = calc_checksum(f->data, f->data_len);
    if (ck != f->checksum) {
        ESP_LOGW(TAG, "checksum mismatch: calc=0x%04X recv=0x%04X", ck, f->checksum);
        return false;
    }
    return true;
}

// ---- send a frame ----
static bool send_frame(uint8_t func, const uint8_t *data, uint16_t len) {
    if (len > PROTO_MAX_DATA_LEN) {
        ESP_LOGE(TAG, "data too long: %d > %d", len, PROTO_MAX_DATA_LEN);
        return false;
    }

    uint8_t buf[PROTO_MAX_DATA_LEN + 9];
    uint16_t pos = 0;

    // header
    buf[pos++] = (uint8_t)(FRAME_HEADER_UP & 0xFF);
    buf[pos++] = (uint8_t)((FRAME_HEADER_UP >> 8) & 0xFF);
    // func
    buf[pos++] = func;
    // data_len (LE)
    buf[pos++] = (uint8_t)(len & 0xFF);
    buf[pos++] = (uint8_t)((len >> 8) & 0xFF);
    // data
    if (len > 0) {
        memcpy(buf + pos, data, len);
        pos += len;
    }
    // checksum (LE)
    uint16_t ck = calc_checksum(data, len);
    buf[pos++] = (uint8_t)(ck & 0xFF);
    buf[pos++] = (uint8_t)((ck >> 8) & 0xFF);
    // footer
    buf[pos++] = (uint8_t)(FRAME_FOOTER_UP & 0xFF);
    buf[pos++] = (uint8_t)((FRAME_FOOTER_UP >> 8) & 0xFF);

    int sent = uart_send(buf, pos);
    if (sent != (int)pos) {
        ESP_LOGE(TAG, "send failed: %d/%d bytes", sent, pos);
        return false;
    }
    ESP_LOGD(TAG, "TX func=0x%02X len=%d", func, len);
    return true;
}

// ---- parse STM32 response frame ----
static void handle_response(const proto_frame_t *f) {
    // Simple 10-byte response: [BB BB] [func] [01 00] [result] [ck_L ck_H] [2B 2B]
    if (f->data_len == 1 && f->func >= CMD_EMOTION && f->func <= CMD_POWER) {
        uint8_t result = f->data[0];
        ESP_LOGI(TAG, "response: func=0x%02X result=%d (%s)",
                 f->func, result,
                 result == RESP_RECEIVED ? "RECEIVED" :
                 result == RESP_CHECK_ERROR ? "CHECK_ERROR" :
                 result == RESP_EXECUTING ? "EXECUTING" :
                 result == RESP_COMPLETED ? "COMPLETED" : "?");
        if (s_response_cb) s_response_cb(f->func, result);
        return;
    }

    // Larger data responses (JSON payloads)
    if (f->data_len > 1) {
        char json[PROTO_MAX_DATA_LEN + 1];
        memcpy(json, f->data, f->data_len);
        json[f->data_len] = '\0';
        ESP_LOGI(TAG, "RX func=0x%02X data: %s", f->func, json);

        switch (f->func) {
        case CMD_QUERY_SENSOR: {
            int head=0, body=0, chin=0, ha=0, hb=0;
            sscanf(json, "{\"Head\":%d,\"Body\":%d,\"Chin\":%d,\"HA\":%d,\"HB\":%d}", &head, &body, &chin, &ha, &hb);
            if (s_sensor_cb) s_sensor_cb(head, body, chin, ha, hb);
            break;
        }
        case CMD_QUERY_STATUS: {
            int mode=0, battery=0, charging=0, error=0, pose=0, temp1=0, temp2=0, error2=0;
            sscanf(json, "{\"mode\":%d,\"battery\":%d,\"charging\":%d,\"error\":%d,\"pose\":%d,\"temp1\":%d,\"temp2\":%d,\"error2\":%d}",
                   &mode, &battery, &charging, &error, &pose, &temp1, &temp2, &error2);
            if (s_status_cb) s_status_cb(mode, battery, charging, error, pose, temp1, temp2, error2);
            break;
        }
        case CMD_QUERY_HEAD: {
            int ax=0, ay=0;
            sscanf(json, "[{\"Type\":\"Head\",\"AngleX\":\"%d\",\"AngleY\":\"%d\"}]", &ax, &ay);
            if (s_head_cb) s_head_cb(ax, ay);
            break;
        }
        case CMD_QUERY_VERSION: {
            char ver[64] = {};
            sscanf(json, "{\"V\":\"%63[^\"]\"}", ver);
            if (s_version_cb) s_version_cb(ver);
            break;
        }
        default:
            if (s_raw_cb) s_raw_cb(f->func, f->data, f->data_len);
            break;
        }
    }
}

// ---- public: feed received bytes from UART ----
void proto_feed_bytes(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        s_rx_buf[s_rx_idx++] = data[i];

        // Prevent overflow
        if (s_rx_idx >= sizeof(s_rx_buf)) {
            ESP_LOGW(TAG, "RX buffer overflow, reset");
            s_rx_idx = 0;
            continue;
        }

        // Try to find a complete frame: look for header+footer pattern
        if (s_rx_idx >= PROTO_MIN_FRAME_LEN) {
            proto_frame_t *f = (proto_frame_t *)s_rx_buf;
            uint16_t exp_len = PROTO_MIN_FRAME_LEN + f->data_len;
            if (exp_len > sizeof(s_rx_buf)) {
                s_rx_idx = 0;  // bogus length
                continue;
            }

            if (s_rx_idx >= exp_len) {
                // We have enough bytes; try to validate
                if (validate_frame(f, exp_len, FRAME_HEADER_DOWN, FRAME_FOOTER_DOWN)) {
                    handle_response(f);
                } else if (validate_frame(f, exp_len, FRAME_HEADER_UP, FRAME_FOOTER_UP)) {
                    // Echo/double-check — ignore our own sent frames
                    ESP_LOGD(TAG, "ignored echo frame");
                }
                // Shift remaining bytes forward
                if (s_rx_idx > exp_len) {
                    memmove(s_rx_buf, s_rx_buf + exp_len, s_rx_idx - exp_len);
                }
                s_rx_idx -= exp_len;
            }
        }
    }
}

// ---- init ----
void proto_init(void) {
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    s_rx_idx = 0;
    ESP_LOGI(TAG, "protocol handler ready");
}

void proto_deinit(void) {
    s_sensor_cb   = NULL;
    s_status_cb   = NULL;
    s_head_cb     = NULL;
    s_version_cb  = NULL;
    s_response_cb = NULL;
    s_raw_cb      = NULL;
}

// ---- callback setters ----
void proto_set_sensor_cb(proto_sensor_cb_t cb)   { s_sensor_cb   = cb; }
void proto_set_status_cb(proto_status_cb_t cb)   { s_status_cb   = cb; }
void proto_set_head_cb(proto_head_cb_t cb)       { s_head_cb     = cb; }
void proto_set_version_cb(proto_version_cb_t cb) { s_version_cb  = cb; }
void proto_set_response_cb(proto_response_cb_t cb) { s_response_cb = cb; }
void proto_set_raw_cb(proto_raw_cb_t cb)         { s_raw_cb      = cb; }

// ---- send commands ----
bool proto_send_emotion(uint8_t state, uint8_t level) {
    uint8_t d[2] = {state, level};
    return send_frame(CMD_EMOTION, d, 2);
}
bool proto_send_play_action(uint16_t action_id) {
    uint8_t d[2] = {(uint8_t)(action_id & 0xFF), (uint8_t)((action_id >> 8) & 0xFF)};
    return send_frame(CMD_PLAY_ACTION, d, 2);
}
bool proto_send_head_angle(int16_t horizontal, int16_t vertical) {
    uint8_t d[5];
    d[0] = 0x01;  // head indicator
    d[1] = (horizontal < 0) ? 0xFF : 0x00;
    d[2] = (uint8_t)(abs(horizontal) & 0xFF);
    d[3] = (vertical < 0) ? 0xFF : 0x00;
    d[4] = (uint8_t)(abs(vertical) & 0xFF);
    return send_frame(CMD_HEAD_ANGLE, d, 5);
}
bool proto_send_query_sensor(void)  { return send_frame(CMD_QUERY_SENSOR,  NULL, 0); }
bool proto_send_query_status(void)  { return send_frame(CMD_QUERY_STATUS,  NULL, 0); }
bool proto_send_query_head(void)    { return send_frame(CMD_QUERY_HEAD,    NULL, 0); }
bool proto_send_query_version(void) { return send_frame(CMD_QUERY_VERSION, NULL, 0); }
bool proto_send_bootloader(void)    { return send_frame(CMD_BOOTLOADER,    NULL, 0); }
bool proto_send_power(proto_power_t state) {
    uint8_t d[1] = {(uint8_t)state};
    return send_frame(CMD_POWER, d, 1);
}
bool proto_send_raw(uint8_t func, const uint8_t *data, uint16_t len) {
    return send_frame(func, data, len);
}
