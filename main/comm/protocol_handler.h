/*
 * TeddyBear STM32 <-> ESP32 Communication Protocol
 *
 * Reference: STM32 PandaRobot/Users/user_communication.h
 *
 * Frame format:
 *   | Header(2B) | Func(1B) | DataLen(2B LE) | Data(NB) | Checksum(2B LE) | Footer(2B) |
 *
 *   ESP32 -> STM32: header=0x4141("AA"), footer=0x3D3D("==")
 *   STM32 -> ESP32: header=0x4242("BB"), footer=0x2B2B("++")
 *   Checksum: simple sum of data bytes (uint16 little-endian)
 */

#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_MAX_DATA_LEN   128
#define PROTO_MIN_FRAME_LEN  9     // 2H + 1F + 2L + 0D + 2C + 2F

// ---- frame markers ----
#define FRAME_HEADER_UP      0x4141  // ESP32 -> STM32  ("AA")
#define FRAME_FOOTER_UP      0x3D3D  // ESP32 -> STM32  ("==")
#define FRAME_HEADER_DOWN    0x4242  // STM32 -> ESP32  ("BB")
#define FRAME_FOOTER_DOWN    0x2B2B  // STM32 -> ESP32  ("++")

// ---- command codes (ESP32 -> STM32) ----
typedef enum {
    CMD_EMOTION       = 0x01,  // emotion / random action
    CMD_PLAY_ACTION   = 0x02,  // play action by ID
    CMD_HEAD_ANGLE    = 0x03,  // set head angle
    CMD_QUERY_SENSOR  = 0x04,  // query touch sensors
    CMD_QUERY_STATUS  = 0x05,  // query robot status
    CMD_QUERY_HEAD    = 0x06,  // query head servo params
    CMD_QUERY_VERSION = 0x07,  // query firmware version
    CMD_BOOTLOADER    = 0x08,  // jump to bootloader
    CMD_POWER         = 0x0A,  // power state control
} proto_cmd_t;

// ---- response states (STM32 -> ESP32) ----
typedef enum {
    RESP_RECEIVED    = 0x00,  // received
    RESP_CHECK_ERROR = 0x01,  // checksum error
    RESP_EXECUTING   = 0x02,  // executing
    RESP_COMPLETED   = 0x03,  // completed
} proto_resp_t;

// ---- power states ----
typedef enum {
    POWER_IDLE      = 0,
    POWER_HIBERNATE = 1,
    POWER_WAKEUP    = 2,
    POWER_SHUTDOWN  = 3,
    POWER_RESET     = 4,
} proto_power_t;

// ---- frame struct ----
typedef struct {
    uint16_t header;
    uint8_t  func;
    uint16_t data_len;
    uint8_t  data[PROTO_MAX_DATA_LEN];
    uint16_t checksum;
    uint16_t footer;
} proto_frame_t;

// ---- callback types ----
typedef void (*proto_sensor_cb_t)(bool head, bool body, bool chin, bool ha, bool hb);
typedef void (*proto_status_cb_t)(int mode, int battery, bool charging, int error,
                                   int pose, int temp1, int temp2, int error2);
typedef void (*proto_head_cb_t)(int angle_x, int angle_y);
typedef void (*proto_version_cb_t)(const char *version);
typedef void (*proto_response_cb_t)(uint8_t func, uint8_t result);
typedef void (*proto_raw_cb_t)(uint8_t func, const uint8_t *data, uint16_t len);

// ---- init / deinit ----
void proto_init(void);
void proto_deinit(void);

// ---- set callbacks ----
void proto_set_sensor_cb(proto_sensor_cb_t cb);
void proto_set_status_cb(proto_status_cb_t cb);
void proto_set_head_cb(proto_head_cb_t cb);
void proto_set_version_cb(proto_version_cb_t cb);
void proto_set_response_cb(proto_response_cb_t cb);
void proto_set_raw_cb(proto_raw_cb_t cb);  // catch-all for unhandled response func codes

// ---- send commands ----
bool proto_send_emotion(uint8_t state, uint8_t level);
bool proto_send_play_action(uint16_t action_id);
bool proto_send_head_angle(int16_t horizontal, int16_t vertical);
bool proto_send_query_sensor(void);
bool proto_send_query_status(void);
bool proto_send_query_head(void);
bool proto_send_query_version(void);
bool proto_send_bootloader(void);
bool proto_send_power(proto_power_t state);

// ---- raw send (for custom commands) ----
bool proto_send_raw(uint8_t func, const uint8_t *data, uint16_t len);

// ---- feed received bytes (called from UART RX callback) ----
void proto_feed_bytes(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // PROTOCOL_HANDLER_H
