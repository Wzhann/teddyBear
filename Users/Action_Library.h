#ifndef __ACTION_LIBRARY_H__
#define __ACTION_LIBRARY_H__
#include "stdint.h"

#define ACTION_COUNT_MAX 30
// 示教总步数参数设置，默认定时步长为5步/秒，如果35次对应为7s的时间长度
//#define TEACH_TOTAL_STEP 30
// 动作库每步最大步数设置为60
extern int TEACH_TOTAL_STEP;

#define MAX_TOTAL_STEP 80
// 单个动作最大组合动作数为5
#define MAX_NUM_MOTION 5

#define POSE_SITTING 1   // 坐
#define POSE_LYING 2     // 趴
#define POSE_STANDING 3  // 站立
#define POSE_error 4     // 错误/未知姿态

// 三维点结构体（3维空间坐标），用于泛用
typedef struct
{
    float x;
    float y;
    float z;
} Coordinate;

// 动作情绪枚举
typedef enum
{
    EMOTION_NEUTRAL = 0, // 中性
    EMOTION_HAPPY,       // 开心
    EMOTION_SAD,         // 伤心
    EMOTION_ANGRY,       // 愤怒
    EMOTION_SURPRISED,   // 惊喜
    EMOTION_CUSTOM       // 自定义情绪
} EmotionType;

// 动作步进结构体（Flash 存储）
typedef struct
{
    const int16_t servoAngles[14]; // 该动作下目标角度值 使用 const通过外部列表初始化
} ServoActionStep;

// 动作序列结构体（Flash 存储）
typedef struct
{
    uint16_t actionId;
    const ServoActionStep *actions; // 成为指针，指向 Flash 数组
    EmotionType emotionType;        // 对应的运动情绪
    uint8_t total_step;
    float totalDuration;          // 总执行时间（单位：ms）
    uint8_t ifNeedBezier;         // 是否需要贝塞尔曲线插值
} ServoActionSeries;

typedef struct
{
    ServoActionSeries motion[MAX_NUM_MOTION];
    uint32_t point_total;
    uint32_t point_iter;
    uint8_t posestart;
    uint8_t poseend;
} Motion_t;

// ============ RAM 版本结构体（示教模式用）============
typedef struct
{
    int16_t servoAngles[14]; // 该动作下目标角度值
} ServoActionStep_ram;

typedef struct
{
    uint16_t actionId;                           // 系列动作唯一标识
    ServoActionStep_ram actions[MAX_TOTAL_STEP]; // 假定每一个动作不超过10个离散的动作点
    EmotionType emotionType;                     // 对应的运动情绪
    uint16_t total_step;
    float totalDuration;          // 总执行时间（单位：ms）
    int16_t startservoAngles[14]; // 该动作起始时14个舵机角度值
    int16_t endservoAngles[14];   // 该动作结束时14个舵机角度值
    uint8_t ifNeedBezier;         // 是否需要贝塞尔曲线插值
} ServoActionSeries_ram;

typedef struct
{
    ServoActionSeries_ram motion[MAX_NUM_MOTION];
    uint32_t point_total;
    uint32_t point_iter;
    uint8_t posestart;
    uint8_t poseend;
} Motion_t_ram;

// ============ 函数声明 ============
void Action_init(void);
void Action_Teachmode_Init(void);
void Action_Teachmode(void);
extern ServoActionSeries_ram *Action_index[ACTION_COUNT_MAX];
extern int TEACHMODE;
extern ServoActionSeries_ram Action_TEACH;
extern int TEACH_OK;
extern int TEACH_FINISH;

extern Motion_t_ram _Action_TEACH;

// ============ 姿态初始化动作 ============
extern Motion_t MsittingInit;
extern Motion_t MstandingInit;
extern Motion_t MlyingInit;

// ============ 站立姿态动作 (POSE_STANDING) ============
extern Motion_t Motion_Stand_Bow;       // 鞠躬
extern Motion_t Motion_Stand_Dance1;    // 跳舞1
extern Motion_t Motion_Stand_Dance2;    // 跳舞2
extern Motion_t Motion_Stand_StepBack;  // 后退
extern Motion_t Motion_Stand_Salute;    // 敬礼
extern Motion_t Motion_Stand_BlowKiss;  // 飞吻
extern Motion_t Motion_Stand_Dance3;    // 跳舞3
extern Motion_t Motion_Stand_Handshake; // 站立握手
extern Motion_t Motion_Stand_Pray;      // 拜一拜

// ============ 坐姿态动作 (POSE_SITTING) ============
extern Motion_t Motion_Sit_Handshake;   // 握手
extern Motion_t Motion_Sit_Hello;       // 打招呼
extern Motion_t Motion_Sit_Stretch;     // 伸懒腰
extern Motion_t Motion_Sit_ShakeHead;   // 摇头
extern Motion_t Motion_Sit_Cheer;       // 加油
extern Motion_t Motion_Sit_Yawn;        // 打哈欠
extern Motion_t Motion_Sit_Drum;        // 打鼓
extern Motion_t Motion_Sit_WashFace;    // 洗脸

// ============ 趴姿态动作 (POSE_LYING) ============
extern Motion_t Motion_Lie_WagHips;     // 扭屁股
extern Motion_t Motion_Lie_PushUp;      // 俯卧撑
extern Motion_t Motion_Lie_Crawl;       // 爬行

// ============ 姿态切换动作 ============
extern Motion_t Motion_SitToStand;      // 坐->直立
extern Motion_t Motion_StandToSit;      // 直立->坐
extern Motion_t Motion_SitToLie;        // 坐->趴
extern Motion_t Motion_LieToSit;        // 趴->坐
extern Motion_t Motion_LieToStand;      // 趴->站
extern Motion_t Motion_StandToLie;      // 站->趴

#endif
