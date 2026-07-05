#ifndef _user_tasks_h
#define _user_tasks_h

#include "main.h"
#include "time.h"

typedef enum
{
    // ===== 示教 =====
    ACTION_TEACH = 0, // 示教模式

    // ===== 站立姿态动作 (POSE_STANDING) =====
    ACTION_STAND_BOW,       // 1  鞠躬
    ACTION_STAND_DANCE1,    // 2  跳舞1
    ACTION_STAND_DANCE2,    // 3  跳舞2
    ACTION_STAND_STEPBACK,  // 4  后退
    ACTION_STAND_SALUTE,    // 5  敬礼
    ACTION_STAND_BLOWKISS,  // 6  飞吻
    ACTION_STAND_DANCE3,    // 7  跳舞3
    ACTION_STAND_HANDSHAKE, // 8  站立握手
    ACTION_STAND_PRAY,      // 9  拜一拜

    // ===== 坐姿态动作 (POSE_SITTING) =====
    ACTION_SIT_HANDSHAKE, // 10 握手
    ACTION_SIT_HELLO,     // 11 打招呼
    ACTION_SIT_STRETCH,   // 12 伸懒腰
    ACTION_SIT_SHAKEHEAD, // 13 摇头
    ACTION_SIT_CHEER,     // 14 加油
    ACTION_SIT_YAWN,      // 15 打哈欠
    ACTION_SIT_DRUM,      // 16 打鼓
    ACTION_SIT_WASHFACE,  // 17 洗脸

    // ===== 趴姿态动作 (POSE_LYING) =====
    ACTION_LIE_WAGHIPS, // 18 扭屁股
    ACTION_LIE_PUSHUP,  // 19 俯卧撑
    ACTION_LIE_CRAWL,   // 20 爬行

    // ===== 姿态切换 =====
    ACTION_SIT_TO_STAND, // 21 坐->站立
    ACTION_STAND_TO_SIT, // 22 站立->坐
    ACTION_SIT_TO_LIE,   // 23 坐->趴
    ACTION_LIE_TO_SIT,   // 24 趴->坐
    ACTION_LIE_TO_STAND, // 25 趴->站立
    ACTION_STAND_TO_LIE, // 26 站立->趴

    // ===== 系统状态 =====
    IDLE, // 27 空闲
} ACTION_STATE;

typedef struct
{
    uint8_t buzzerForvoltage;
    uint8_t buzzerForcharge;
    uint8_t buzzerForpose;
} buzzerType;

void ActionRUN(void);
void TeachmodeRUN(void);

#endif
