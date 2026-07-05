# PandaRobot 🐼

基于 STM32H743 的双足熊猫机器人嵌入式固件，支持 14 舵机控制、预编排动作库、示教模式、IMU 姿态估计、上位机通信协议与 OTA 升级。

## 目录

- [硬件平台](#硬件平台)
- [目录结构](#目录结构)
- [软件架构](#软件架构)
  - [FreeRTOS 任务架构](#freertos-任务架构)
  - [数据流总览](#数据流总览)
- [动作系统详解](#动作系统详解)
  - [动作数据结构](#动作数据结构)
  - [动作执行流程](#动作执行流程)
  - [步进插补机制](#步进插补机制)
  - [动作到位判断](#动作到位判断)
  - [运动序列运行与复位](#运动序列运行与复位)
  - [姿态体系与切换](#姿态体系与切换)
  - [上位机动作指令协议](#上位机动作指令协议)
  - [示教模式](#示教模式)
  - [动作数据示例](#动作数据示例)
- [其他核心模块](#其他核心模块)
  - [舵机控制](#舵机控制)
  - [上位机通信协议](#上位机通信协议)
  - [IMU 姿态估计](#imu-姿态估计)
  - [IAP 固件升级](#iap-固件升级)
  - [定时器系统](#定时器系统)
  - [Flash 存储](#flash-存储)
  - [ADC 电池监测](#adc-电池监测)
- [构建方法](#构建方法)
- [CubeMX 重新配置](#cubemx-重新配置)
- [头文件引用关系](#头文件引用关系)

## 硬件平台

| 项目  | 参数                                                 |
| ----- | ---------------------------------------------------- |
| MCU   | STM32H743VITx (Cortex-M7, 480MHz)                    |
| Flash | 2MB 双 Bank（支持 IAP 升级）                         |
| RAM   | 1MB (AXI SRAM 512KB + DTCM 128KB + ITCM 64KB + 其他) |
| 舵机  | 14 × FEETECH 总线舵机（4 路 UART 分组控制）          |
| IMU   | MPU6050（软件 I2C，卡尔曼滤波）                      |
| 温度  | 2 × DS18B20（OneWire，GPIO PE0）                     |
| ADC   | 电池电压 + 电流监测（ADC1 + DMA）                    |
| 通信  | UART4/UART5 双通道上位机协议（DMA 双缓冲）           |
| 其他  | LED / 蜂鸣器 / 风扇 / 触摸传感器 / 人体感应          |

## 目录结构

```
PandaRobot/
├── Core/                  # STM32CubeMX 自动生成的 HAL 外设初始化
│   ├── Inc/               #   头文件（main.h, FreeRTOSConfig.h, 各外设.h）
│   └── Src/               #   源文件（main.c, freertos.c, 各外设.c, 中断处理）
├── Users/                 # ★ 自定义应用代码（主要开发区域）
│   ├── user_tasks.c/h     #   核心任务逻辑、动作执行引擎
│   ├── Action_Library.c/h #   预编排动作数据（角度序列）
│   ├── user_servo.c/h     #   FEETECH 舵机控制
│   ├── user_communication.c/h # 上位机通信协议
│   ├── user_imu_i2c.c/h   #   MPU6050 IMU + 卡尔曼滤波
│   ├── user_flash.c/h     #   Flash 持久化存储
│   ├── user_IAP.c/h       #   IAP 双 Bank 固件升级
│   ├── user_timer.c/h     #   硬件定时器回调（舵机步进/示教）
│   ├── user_adc.c/h       #   电池电压电流监测
│   ├── DS18B20.c/h        #   DS18B20 温度传感器
│   ├── user_iic.c/h       #   软件 I2C（通用）
│   ├── Myiic_IMU.c/h      #   软件 I2C（IMU 专用）
│   ├── user_led.c/h       #   LED 控制
│   ├── user_states.c/h    #   机器人状态机
│   ├── user_gait.c/h      #   步态相关
│   ├── user_comm.c/h      #   通信辅助
│   ├── user_imu.c/h       #   IMU 辅助
│   ├── resolve.c/h        #   解析辅助
│   └── user_includes.h    #   公共头文件枢纽
├── Drivers/               # STM32 HAL + CMSIS（厂商代码，勿改）
├── Middlewares/            # FreeRTOS 内核（厂商代码，勿改）
├── MDK-ARM/               # Keil 工程文件与构建输出
│   ├── Panda_Robot.uvprojx #  Keil 工程文件
│   └── startup_stm32h743xx.s # 启动文件
└── Panda_Robot.ioc        # STM32CubeMX 配置文件
```

## 软件架构

### FreeRTOS 任务架构

固件基于 FreeRTOS 运行，共 5 个任务，在 `Core/Src/freertos.c` 中创建，实现在 `Users/user_tasks.c` 中：

```
┌──────────────────────────────────────────────────┐
│                 FreeRTOS 任务                      │
├──────────────┬──────────┬──────┬─────────────────┤
│ 任务名        │ 优先级    │ 栈   │ 功能             │
├──────────────┼──────────┼──────┼─────────────────┤
│ TaskHigh     │ Realtime │ 128  │ 系统初始化        │
│              │          │      │ · UART4 初始化    │
│              │          │      │ · 通信协议初始化   │
│              │          │      │ · 定时器初始化     │
│              │          │      │ · 舵机初始化       │
│              │          │      │ · ADC 初始化       │
│              │          │      │ · 动作库加载       │
│              │          │      │ · DS18B20 初始化   │
├──────────────┼──────────┼──────┼─────────────────┤
│ TaskMid      │ Normal   │ 2048 │ 主动作循环        │
│              │          │      │ · robotRUN()      │
│              │          │      │ · TeachmodeRUN()  │
├──────────────┼──────────┼──────┼─────────────────┤
│ TaskLow      │ Low      │ 128  │ IMU 初始化与校准  │
├──────────────┼──────────┼──────┼─────────────────┤
│ defaultTask  │ Normal   │ 128  │ 空循环（预留）    │
├──────────────┼──────────┼──────┼─────────────────┤
│ myTask05     │ Low      │ 128  │ 后台任务（预留）  │
└──────────────┴──────────┴──────┴─────────────────┘
```

### 数据流总览

```
上位机 ──UART4/5──▶ [DMA双缓冲接收]
                       │
                       ▼
              user_communication.c
              ┌─────────────────┐
              │ 协议帧解析       │  帧格式: AA + func + len + data + CRC16 + ==
              │ CRC16 校验       │
              │ 命令分发         │
              └───────┬─────────┘
                      │
           ┌──────────┼──────────┐
           ▼          ▼          ▼
      动作控制     传感器查询    电源管理
           │          │          │
           ▼          ▼          ▼
    ACTION_STATE   SensorData   PowerType_T
    (枚举选择动作)  (触摸/IMU)   (休眠/唤醒/关机)
           │
           ▼
    Motion_t 动作序列执行
           │
           ▼
    user_tasks.c 动作引擎
    ┌───────────────────────┐
    │ 步进插补 (11ms 周期)   │  TIM4 定时器中断
    │ 动作切换 (170ms 步进)  │  ACTIONTIMESTEP
    │ 到位判断 → 下一步      │
    └───────────┬───────────┘
                │
                ▼
    user_servo.c → 4路UART → FEETECH 舵机
    ┌──────────────────────────┐
    │ 左腿(USART1)  右腿(USART2)│
    │ 颈部(UART5)   头部(UART7) │
    └──────────────────────────┘
```

## 动作系统详解

动作系统是本固件最核心的部分，涵盖动作数据组织、执行引擎、步进插补、到位判断、姿态切换和示教模式。相关代码主要集中在：

- `Users/user_tasks.c/h` — 动作执行引擎、状态机、示教模式入口
- `Users/Action_Library.c/h` — 动作数据定义（角度序列）
- `Users/user_timer.c/h` — 定时器中断驱动步进插补和示教采集
- `Users/user_servo.c/h` — 舵机底层驱动
- `Users/user_communication.c/h` — 上位机动作指令解析

### 动作数据结构

动作数据采用三级嵌套结构，从单个舵机角度快照到完整运动序列：

```
Motion_t（完整运动，如"鞠躬"）
  ├── posestart          // 起始姿态标签 (POSE_SITTING/POSE_LYING/POSE_STANDING)
  ├── poseend            // 结束姿态标签
  ├── point_total        // 包含的动作序列数（1~5）
  ├── point_iter         // 当前执行到第几个动作序列（运行时迭代器）
  └── motion[MAX_NUM_MOTION]  // 动作序列数组，最多5个
       │
       ▼
  ServoActionSeries（单段动作序列）
    ├── actionId          // 唯一标识（用于完成标志 Action_done[actionId]）
    ├── total_step        // 该序列包含的步数
    ├── totalDuration     // 总执行时间(ms)
    ├── emotionType       // 情绪类型 (EmotionType 枚举)
    ├── ifNeedBezier      // 是否使用贝塞尔曲线插值
    └── actions[]         // 步进数组
         │
         ▼
    ServoActionStep（单步快照）
      └── servoAngles[14] // 14个舵机的目标角度值
                           // 索引0为占位(0)
                           // 索引1~6: 左腿+右腿关节
                           // 索引7~12: 身体/颈部关节
                           // 索引13: 头部关节
```

**Flash 版本 vs RAM 版本**：

系统为动作数据提供了两套结构体，设计目的不同：

| 特性 | Flash 版本 | RAM 版本 |
|------|-----------|---------|
| 结构体 | `ServoActionSeries` + `ServoActionStep` | `ServoActionSeries_ram` + `ServoActionStep_ram` |
| 角度数据 | `const int16_t servoAngles[14]` | `int16_t servoAngles[14]`（可修改） |
| 存储位置 | Flash，编译期固化 | RAM，运行时动态写入 |
| 用途 | 预编排动作库（鞠躬、跳舞等） | 示教模式实时采集 |
| 额外字段 | 无 | `startservoAngles[14]`、`endservoAngles[14]` |
| 最大步数 | 按需定义 | `MAX_TOTAL_STEP = 80` |
| 运行函数 | `Motion_Run()` | `Motion_Run_Bezier()` |
| 到位判断 | `_SingleAction_CheckApproch()` | `_SingleAction_CheckApproch_Bezier()` |
| 复位函数 | `Motion_Reset()` | `Motion_Reset_Bezier()` |

**动作状态枚举 `ACTION_STATE`**（定义在 `user_tasks.h`）：

```c
typedef enum {
    // 示教
    ACTION_TEACH = 0,        // 示教模式回放

    // 站立姿态动作 (POSE_STANDING)
    ACTION_STAND_BOW,        // 1  鞠躬
    ACTION_STAND_DANCE1,     // 2  跳舞1
    ACTION_STAND_DANCE2,     // 3  跳舞2
    ACTION_STAND_STEPBACK,   // 4  后退
    ACTION_STAND_SALUTE,     // 5  敬礼
    ACTION_STAND_BLOWKISS,   // 6  飞吻
    ACTION_STAND_DANCE3,     // 7  跳舞3
    ACTION_STAND_HANDSHAKE,  // 8  站立握手
    ACTION_STAND_PRAY,       // 9  拜一拜

    // 坐姿态动作 (POSE_SITTING)
    ACTION_SIT_HANDSHAKE,    // 10 握手
    ACTION_SIT_HELLO,        // 11 打招呼
    ACTION_SIT_STRETCH,      // 12 伸懒腰
    ACTION_SIT_SHAKEHEAD,    // 13 摇头
    ACTION_SIT_CHEER,        // 14 加油
    ACTION_SIT_YAWN,         // 15 打哈欠
    ACTION_SIT_DRUM,         // 16 打鼓
    ACTION_SIT_WASHFACE,     // 17 洗脸

    // 趴姿态动作 (POSE_LYING)
    ACTION_LIE_WAGHIPS,      // 18 扭屁股
    ACTION_LIE_PUSHUP,       // 19 俯卧撑
    ACTION_LIE_CRAWL,        // 20 爬行

    // 姿态切换
    ACTION_SIT_TO_STAND,     // 21 坐→站
    ACTION_STAND_TO_SIT,     // 22 站→坐
    ACTION_SIT_TO_LIE,       // 23 坐→趴
    ACTION_LIE_TO_SIT,       // 24 趴→坐
    ACTION_LIE_TO_STAND,     // 25 趴→站
    ACTION_STAND_TO_LIE,     // 26 站→趴

    IDLE,                    // 27 空闲
} ACTION_STATE;
```

### 动作执行流程

动作的完整执行流程如下：

```
1. 上位机发送 func=0x02 指令，携带 action_id
       │
       ▼
2. user_communication.c 解析命令
   · TEACHMODE = 0（确保退出示教模式）
   · step_counter = 1
   · ActionNow = action_id
       │
       ▼
3. TaskMid 主循环（每10ms）检测到 ActionNow != IDLE
   · 调用 robotRun()
       │
       ▼
4. robotRun() 查表获取 Motion_t*
   · getMotionForAction(ActionNow, &actionPoseNext)
   · 返回对应动作数据指针
       │
       ▼
5. Motion_Run(motion) 执行动作序列
   · 首次进入：将第一步目标角度写入 goal_pos[1~12]
   · 发送"开始执行"应答 (0x02)
   · 每次循环调用 _SingleAction_CheckApproch() 推进步进
   · 返回 true → 动作序列完成
       │
       ▼
6. Motion_Reset(motion) 复位
   · Action_done[] 清零
   · step_counter = 1
   · point_iter = 0
   · 发送"执行完成"应答 (0x03)
   · ActionNow = IDLE
```

### 步进插补机制

舵机的平滑运动由三层定时驱动：

**第一层：TIM4 中断（11ms 周期）— 舵机角度插补**

`User_TimerServoIRQ()` 在 `user_timer.c` 中实现，每 11ms 触发一次：

```
每个 11ms 周期：
  1. 轮流发送舵机控制指令（写角度/读角度交替）
     · 奇数周期: User_AllSetAngTime() 将 goal_pos[] 写入所有舵机
     · 偶数周期: FEETECH_ReadServoPos() 依次读取 12 个舵机当前位置
  2. 在 TaskMid 的 robotRun() 中执行速度插补：
     if (goal_pos[i] <= target - speed)
         goal_pos[i] += speed;      // 向目标角度逼近
     else if (goal_pos[i] >= target + speed)
         goal_pos[i] -= speed;
     // speed 默认为 3（0.3度/步），即每 11ms 最大移动 0.3°
```

**第二层：TIM4 计数器（170ms 间隔）— 动作步进切换**

`User_TimerActionIRQ()` 累计计时，每 `ACTIONTIMESTEP`（170ms）递增 `timerStepForAction`：

```c
void User_TimerActionIRQ(void) {
    if (ActionNow != IDLE)
        countTimerStepForAction++;
    if (countTimerStepForAction >= actionSwitchTime) {  // actionSwitchTime = 170
        countTimerStepForAction = 0;
        timerStepForAction++;    // 通知动作引擎切换到下一步
    }
}
```

**第三层：动作引擎 — 步进推进**

在 `_SingleAction_CheckApproch()` 中检测 `timerStepForAction` 变化：

```c
if (timerStepForAction != timerStepForActionLast) {
    timerStepForActionLast = timerStepForAction;
    if (step_counter < action->total_step - 1) {
        step_counter++;
        // 更新 goal_pos[] 为下一步的目标角度
        for (int i = 1; i <= 12; i++)
            goal_pos[i] = action->actions[step_counter - 1].servoAngles[i];
    }
    else if (step_counter == action->total_step - 1) {
        // 最后一步：标记完成
        Action_done[action->actionId] = 1;
        return true;
    }
}
```

**时序图**：

```
时间轴 (ms)    0    11   22   ...  170  181  ...  340  ...  170×N
               │    │    │         │    │         │
TIM4 中断      ↓    ↓    ↓         ↓    ↓         ↓
              插补  插补  插补      插补  插补      插补
               │              │                   │
               │   step=1     │   step=2          │  step=N
               │   目标角度A   │   目标角度B       │  目标角度X
               │              │                   │
goal_pos[]    →A+3 →A+6 ... →A     →B+3 →B+6 ...→B    ... 完成!
               ↑                             ↑
           speed插补                    speed插补
           (每11ms逼近3单位)           (每11ms逼近3单位)
```

### 动作到位判断

系统提供两个版本的到位判断函数：

**`_SingleAction_CheckApproch()` — Flash 版本**：

1. 对 12 个舵机（索引 1~12）执行速度插补，将 `goal_pos[]` 逐步逼近 `actions[step_counter].servoAngles[]`
2. 计算当前步与下一步的角度差值 `differenceAction[]`，找出最大差值（预留自适应步进时间，当前使用固定值）
3. 设置 `actionSwitchTime = ACTIONTIMESTEP`（固定 170ms）
4. 检测 `timerStepForAction` 递增 → 推进步进计数器 → 返回 true 表示该序列完成

**`_SingleAction_CheckApproch_Bezier()` — RAM/示教版本**：

逻辑与 Flash 版本基本一致，区别在于：
- 操作 `ServoActionSeries_ram` 类型（非 const）
- 不计算差值和自适应步进时间
- 专用于示教模式的 RAM 动作数据回放

### 运动序列运行与复位

**`Motion_Run()` — Flash 版本运行**：

```c
bool Motion_Run(Motion_t *motion_) {
    // 1. 首次进入：切换舵机到位置模式 (sevroSetMode(i, 0))
    // 2. 发送"开始执行"应答
    // 3. 首次加载第一步目标角度到 goal_pos[]
    // 4. 循环调用 _SingleAction_CheckApproch() 推进步进
    // 5. 当前 ServoActionSeries 完成 → point_iter++
    // 6. 所有序列完成 → 返回 true
}
```

**`Motion_Reset()` — Flash 版本复位**：

```c
void Motion_Reset(Motion_t *motion_) {
    // 1. 清零 Action_done[] 完成标志
    // 2. step_counter = 1
    // 3. point_iter = 0
    // 4. 发送"执行完成"应答 (0x03)
    // 5. 更新工作状态为 MODE_IDLE
    // 6. ActionNow = IDLE
}
```

**`Motion_Run_Bezier()` / `Motion_Reset_Bezier()`** — RAM 版本，逻辑相同但操作 `Motion_t_ram` 类型。

### 姿态体系与切换

机器人有三种基本姿态，每种姿态对应一组初始化动作：

| 姿态 | 常量 | 初始化动作 | 含义 |
|------|------|-----------|------|
| 坐 | `POSE_SITTING = 1` | `MsittingInit` | 臀部着地，腿部弯曲 |
| 趴 | `POSE_LYING = 2` | `MlyingInit` | 四肢着地，身体水平 |
| 站 | `POSE_STANDING = 3` | `MstandingInit` | 双腿直立 |

**姿态切换函数 `switchPose()`**：

当请求的动作属于不同姿态时，需要先执行姿态切换动作：

```c
void switchPose(uint8_t lastPose, uint8_t nowPose) {
    uint8_t composeUnit = ((lastPose << 4) | (nowPose & 0x0f));
    switch (composeUnit) {
    case 0x12: ActionNow = ACTION_SIT_TO_LIE;    break;  // 坐→趴
    case 0x13: ActionNow = ACTION_SIT_TO_STAND;   break;  // 坐→站
    case 0x21: ActionNow = ACTION_LIE_TO_SIT;     break;  // 趴→坐
    case 0x23: ActionNow = ACTION_LIE_TO_STAND;   break;  // 趴→站
    case 0x31: ActionNow = ACTION_STAND_TO_SIT;   break;  // 站→坐
    case 0x32: ActionNow = ACTION_STAND_TO_LIE;   break;  // 站→趴
    }
}
```

通过将上一次姿态左移 4 位与目标姿态组合，实现 O(1) 的姿态映射。

**`actionDoprepare()`**：在执行动作前，先根据当前姿态执行对应的初始化动作（将舵机调整到该姿态的标准位置）。

### 上位机动作指令协议

上位机通过 `func=0x02` 功能码发送动作控制指令：

**帧格式**：

```
帧头(0x4141) + 0x02 + data_len(2B) + data[] + CRC16 + 帧尾(0x3D3D)
                                          │
                                          ▼
                                    data[0~1]: action_id (uint16_t, 小端)
```

**action_id 处理逻辑**（`user_communication.c`）：

| action_id | 行为 |
|-----------|------|
| 1 | 设置 `ActionNow = 1`（对应 `ACTION_STAND_BOW` 鞠躬） |
| 253 | 进入展示模式，依次执行预设动作序列 |
| 254 | 取消停止，恢复到 IDLE |
| 255 | 紧急停止：复位所有标志，`ActionNow = IDLE` |
| 其他 | `ActionNow = action_id`，执行对应编号的动作 |

**动作执行过程的状态应答**：

```
上位机                       机器人
  │                            │
  │──── 0x02 + action_id ────▶│
  │◀─── 0x02 (收到，开始执行) ──│  flag_sendExecuting
  │                            │  ┌─────── 执行动作 ───────┐
  │                            │  │ step_counter: 1→N       │
  │                            │  │ 每170ms推进一步         │
  │                            │  │ 每11ms插补逼近          │
  │                            │  └─────────────────────────┘
  │◀─── 0x03 (执行完成) ──────│  flag_sendCompleted
  │                            │  ActionNow → IDLE
```

**其他动作相关指令**：

| 功能码 | 功能 | 数据格式 |
|--------|------|----------|
| 0x01 | 情绪状态触发 | `data[0]=状态, data[1]=程度` → 随机选择匹配动作 |
| 0x03 | 关节直接控制 | `data[0]=0x01(头部), data[1~2]=水平角, data[3~4]=垂直角` |
| 0x0A | 电源/复位 | `data[0]=PowerType_T`（含 ActionReset 复位动作系统） |

### 示教模式

示教模式允许用户手动摆弄机器人各关节，系统定时采集舵机角度，形成一段动作序列后自动回放。

**涉及的文件与变量**：

| 变量/函数 | 文件 | 作用 |
|-----------|------|------|
| `TEACHMODE` | `Action_Library.c` | 示教模式标志（0=正常, 1=示教中） |
| `TEACH_OK` | `Action_Library.c` | 示教采集开始标志 |
| `TEACH_FINISH` | `Action_Library.c` | 示教采集结束标志 |
| `TEACH_TOTAL_STEP` | `Action_Library.c` | 示教总步数，默认 30（对应 6 秒） |
| `_Action_TEACH` | `Action_Library.c` | RAM 版 Motion_t，存储示教采集的角度数据 |
| `personTeachFlag` | `user_tasks.c` | 人体感应触发示教标志 |
| `T_COUNTER` | `user_timer.c` | 示教定时计数器 |
| `step_record` | `user_timer.c` | 记录示教开始的时刻 |
| `User_TimerTeachIRQ()` | `user_timer.c` | 示教定时器中断，200ms 周期 |
| `TeachmodeRUN()` | `user_tasks.c` | TaskMid 中检测示教结束并触发回放 |
| `Action_Teachmode()` | `Action_Library.c` | 示教结束后打印角度数据（调试用） |

**示教模式完整流程**：

```
阶段一：进入示教模式
━━━━━━━━━━━━━━━━━━
1. 上电时 TEACHMODE = 1（在 User_Init_HIGH 中配置）
   或通过上位机触发

2. 舵机切换到零力矩模式 (sevroSetMode(i, 2))
   → 用户可以自由摆弄各关节

3. 舵机控制被禁用：
   User_TimerServoIRQ() 中检查 TEACHMODE != 1 才发送角度指令
   → 示教期间不会驱动舵机，仅读取当前位置

阶段二：采集角度数据
━━━━━━━━━━━━━━━━━━
1. TIM6 定时器每 200ms 触发 User_TimerTeachIRQ()

2. TEACH_OK = 1 时开始采集：
   T_COUNTER 持续递增
   step_record 记录起始时刻

3. 每个周期读取 14 个舵机当前角度并存储：
   for (int i = 0; i < 14; i++)
       _Action_TEACH.motion[0].actions[T_COUNTER - step_record - 1]
           .servoAngles[i] = SERVO[i].pos_read;

4. 采集持续 TEACH_TOTAL_STEP 个周期（默认 30 步 × 200ms = 6秒）

5. 采集结束：TEACH_FINISH = 1

时序：
  TIM6中断  0ms   200ms  400ms  ...  6000ms
            │      │      │            │
  T_COUNTER 1      2      3      ...  30
            │      │      │            │
  采集      读取   读取   读取   ...  读取 → TEACH_FINISH=1
            角度1  角度2  角度3       角度30

阶段三：回放示教动作
━━━━━━━━━━━━━━━━━━
1. TaskMid 主循环检测 TEACH_FINISH == 1（TeachmodeRUN()）

2. 执行回放准备：
   · 将第一步目标角度写入 goal_pos[]
   · TEACH_OK = 0, TEACH_FINISH = 0, TEACHMODE = 0
   · ActionNow = ACTION_TEACH

3. robotRun() 检测到 ActionNow == ACTION_TEACH
   · 调用 Motion_Run_Bezier(&_Action_TEACH)
   · 使用 RAM 版本的步进插补执行

4. 回放完成后 Motion_Reset_Bezier() 复位
   · 发送执行完成应答
   · 动作引擎回到 IDLE

5. Action_Teachmode() 通过 UART7 打印采集的角度数据
   → 可用于将示教动作固化为 Flash 动作库
```

**示教数据存储结构**：

```c
// 示教用 RAM 动作容器
Motion_t_ram _Action_TEACH = {
    .point_total = 1,        // 仅1个动作序列
    .motion = {
        {
            .actionId = 0,
            .ifNeedBezier = 0,
            .emotionType = EMOTION_NEUTRAL,
            .totalDuration = 2000,  // 30步 × 200ms = 6000ms（运行时更新）
        }
    }
};
```

**示教时间参数**：

| 参数 | 值 | 含义 |
|------|-----|------|
| TIM6 周期 | 200ms | 每次采样的间隔 |
| `TEACH_TOTAL_STEP` | 30（默认） | 总采样步数 |
| 示教总时长 | 6 秒 | 30 × 200ms |
| `MAX_TOTAL_STEP` | 80 | 单序列最大步数 |

**从示教到固化动作库**：

`Action_Teachmode()` 会将采集的角度数据以 C 语言数组格式通过 UART7 打印出来：

```c
void Action_Teachmode(void) {
    printf("{\r\n");
    for (int si = 0; si < TEACH_TOTAL_STEP; si++) {
        printf("    { .servoAngles = {");
        for (int i = 0; i < 13; i++)
            printf("%d, ", _Action_TEACH.motion[0].actions[si].servoAngles[i]);
        printf("%d ", _Action_TEACH.motion[0].actions[si].servoAngles[13]);
        printf("}},\r\n");
    }
    printf("}\r\n");
}
```

输出格式可直接粘贴到 `Action_Library.c` 中作为新的 `ServoActionStep` 数组，实现从示教到永久动作库的转化。

### 动作数据示例

以"鞠躬"动作（`Motion_Stand_Bow`）为例，展示完整的数据定义：

```c
// 动作步进数据：28个关键帧，每帧14个舵机角度
static const ServoActionStep _Stand_BowActions[] = {
    { .servoAngles = {0, -1128, -1412, -626, 10, -153, 483, 831, 553, -7, 155, -1009, 175, 0 }},
    // ... 中间帧省略 ...
    { .servoAngles = {0, -1128, -1412, -626, 424, -154, 1047, 1340, 558, -420, 156, -1008, 60, 0 }},  // 最低点
    // ... 恢复帧省略 ...
    { .servoAngles = {0, -1128, -1412, -626, 8, -153, 1323, 1339, 555, -14, 156, -1013, 60, 0 }},    // 回到站姿
};

// 运动定义
Motion_t Motion_Stand_Bow = {
    .posestart = POSE_STANDING,   // 起始姿态：站立
    .poseend = POSE_STANDING,     // 结束姿态：站立
    .point_total = 1,             // 仅1个动作序列
    .motion = {
        {
            .actionId = 10,                  // 唯一标识
            .ifNeedBezier = 0,               // 不使用贝塞尔插值
            .actions = _Stand_BowActions,    // 指向步进数据
            .total_step = 28,                // 28步
            .emotionType = EMOTION_NEUTRAL,  // 中性情绪
            .totalDuration = 5000            // 总时长约5秒
        }
    }
};
```

**执行过程**：28步 × 170ms/步 ≈ 4.76秒，每步内 11ms 插补周期以 `speed=3` 逼近目标角度，实现平滑运动。

## 其他核心模块

### 舵机控制

14 个 FEETECH 总线舵机通过 4 路 UART 分组控制：

| UART   | 外设 | 舵机 ID | 部位          |
| ------ | ---- | ------- | ------------- |
| USART1 | 左腿 | 1-5     | 左腿各关节    |
| USART2 | 右腿 | 1-5     | 右腿各关节    |
| UART5  | 颈部 | 1-5     | 身体/颈部关节 |
| UART7  | 头部 | 1-5     | 头部/手部关节 |

**舵机控制周期**（TIM4，1ms 基准）：

```
1ms  → 各UART DMA发送同组5个舵机角度+执行时间（1.5ms）
3ms  → 发送第1个舵机状态读取请求
4ms  → 发送第2个舵机状态读取请求
5ms  → 发送第3个舵机状态读取请求
6ms  → 发送第4个舵机状态读取请求
7ms  → 发送第5个舵机状态读取请求

→ 舵机角度写入周期：7ms
→ 舵机角度读取周期：7ms（依次轮询12个舵机）
```

**关键函数**：

- `FEETECH_UsartSetServoPos()` — 单舵机位置+时间+速度设置
- `FEETECH_LEGSYNCWRITE()` — 腿部 5 舵机同步写
- `FEETECH_HEADSYNCWRITE()` — 头部同步写
- `FEETECH_NECKSYNCWRITE()` — 颈部同步写
- `sevroSetMode()` — 设置舵机运行模式（0=位置模式, 2=零力矩模式）
- `User_BezierCurve()` — 贝塞尔曲线插值运动
- `sevroSetZero()` — 所有舵机归零

### 上位机通信协议

自定义二进制帧协议，CRC16 校验，支持双通道（UART4 上行、UART5 下行）DMA 双缓冲接收。

**帧格式**：

```
┌──────┬──────┬──────────┬──────────┬──────────┬──────┐
│ 帧头  │功能码│ 数据长度  │  数据域   │ CRC16   │ 帧尾  │
│ 2B   │ 1B  │ 2B(小端) │ ≤128B   │ 2B(小端) │ 2B   │
│ 0x4141│ func│ data_len │ data[]  │ checksum │0x3D3D│
└──────┴──────┴──────────┴──────────┴──────────┴──────┘
上行帧头: 0x4141 ("AA")  帧尾: 0x3D3D ("==")
下行帧头: 0x4242 ("BB")  帧尾: 0x2B2B ("++")
```

**命令功能码一览**：

| 功能码 | 功能 | 数据格式 |
|--------|------|----------|
| 0x01 | 情绪状态触发 | `data[0]=状态, data[1]=程度` |
| 0x02 | 动作控制 | `data[0~1]=action_id (uint16_t)` |
| 0x03 | 关节直接控制 | `data[0]=0x01(头部), data[1~2]=水平角, data[3~4]=垂直角` |
| 0x04 | 传感器数据查询 | 无数据，返回 JSON |
| 0x05 | 工作状态查询 | 无数据，返回 JSON |
| 0x06 | 综合状态查询 | 无数据 |
| 0x07 | 固件版本查询 | 无数据，返回 `{"V":"3.12.2"}` |
| 0x08 | IAP 升级重启 | 触发跳转到 BootLoader |
| 0x0A | 电源控制 | `data[0]=PowerType_T` |

### IMU 姿态估计

基于 MPU6050 的姿态估计系统，使用软件 I2C 通信（GPIO PB6/PB7）。

```c
typedef struct {
    float Ax, Ay, Az;         // 加速度计
    float Gx, Gy, Gz;         // 陀螺仪
    float pitch, roll, yaw;   // 欧拉角
    float KalmanAngleX;       // 卡尔曼滤波后的 X 角度
    float KalmanAngleY;       // 卡尔曼滤波后的 Y 角度
    float temperature;        // 芯片温度
} MPU6050_t;
```

**姿态判断逻辑** `poseCheck()`：

- 俯仰角 48°~120° → 站立/坐（`UPRIGHT_RANGE`）
- 俯仰角 -40°~48° → 趴（`LYING_RANGE`）

### IAP 固件升级

双 Bank 在应用编程，支持 UART OTA 固件升级：

```
Flash 布局:
0x08000000 ─────────┐
                    │ Bank0 (APP1)
0x08020000 ─────────┤ ← FLASH_APP1_ADDR（主应用）
                    │
0x08100000 ─────────┤ ← FLASH_APP2_ADDR（备用应用）
                    │ Bank1 (APP2)
0x08200000 ─────────┘
```

### 定时器系统

| 定时器 | 周期  | 用途                             |
| ------ | ----- | -------------------------------- |
| TIM4   | 1ms   | 舵机控制调度（7ms一轮）+ 角度插补 |
| TIM6   | 200ms | 示教模式角度采集                  |
| TIM3   | —     | 辅助定时                         |
| TIM7   | —     | HAL 时基                         |
| TIM17  | —     | 辅助                             |

### Flash 存储

- 存储起始地址：`0x0800FC00`
- 数据容量：512 × 4 字节（`STORE_COUNT = 512`）
- 扇区大小：128KB
- 用于保存配置参数和校准数据

### ADC 电池监测

```c
typedef struct {
    uint16_t adc1_dma_buf[2];  // DMA 缓冲（电压通道 + 电流通道）
    float bat_volt;            // 电池电压(V)
    int8_t bat_power;          // 电量百分比
    float bat_current;         // 电池电流(A)
    uint8_t bat_charging;      // 充电状态
} USER_ADC_TYPE;
```

**关键阈值**：

- 最低工作电压：10V（`MINVOLT`）
- 满电电压：12.4V（`MAXVOLT`）
- 上电最低电压：11.4V（`VOLTPOWERON`）

## 构建方法

1. 安装 [Keil MDK-ARM v5](https://www.keil.com/)
2. 安装 STM32H7 DFP（`Keil.STM32H7xx_DFP.3.1.1`）
3. 打开 `MDK-ARM/Panda_Robot.uvprojx`
4. 点击 Build（F7）

## CubeMX 重新配置

如需修改外设配置：

1. 用 STM32CubeMX 打开 `Panda_Robot.ioc`
2. 修改配置后点击 Generate Code
3. **注意**：`Core/` 下的文件会被重新生成，自定义代码只在 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 之间保留

## 头文件引用关系

```
user_includes.h（枢纽）
  ├── stm32h7xx_hal.h
  ├── cmsis_os.h (FreeRTOS)
  ├── main.h
  ├── user_tasks.h
  ├── user_led.h
  ├── user_adc.h
  ├── user_timer.h
  ├── user_imu.h
  ├── user_comm.h
  ├── user_servo.h
  ├── user_gait.h
  └── Action_Library.h
```

大多数 `Users/` 文件通过 `user_includes.h` 或自身头文件间接引入所有依赖。
