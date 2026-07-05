# PandaRobot 🐼

基于 STM32H743 的双足熊猫机器人嵌入式固件，支持 14 舵机控制、预编排动作库、示教模式、IMU 姿态估计、上位机通信协议与 OTA 升级。

## 硬件平台

| 项目 | 参数 |
|------|------|
| MCU | STM32H743VITx (Cortex-M7, 480MHz) |
| Flash | 2MB 双 Bank（支持 IAP 升级） |
| RAM | 1MB (AXI SRAM 512KB + DTCM 128KB + ITCM 64KB + 其他) |
| 舵机 | 14 × FEETECH 总线舵机（4 路 UART 分组控制） |
| IMU | MPU6050（软件 I2C，卡尔曼滤波） |
| 温度 | 2 × DS18B20（OneWire，GPIO PE0） |
| ADC | 电池电压 + 电流监测（ADC1 + DMA） |
| 通信 | UART4/UART5 双通道上位机协议（DMA 双缓冲） |
| 其他 | LED / 蜂鸣器 / 风扇 / 触摸传感器 / 人体感应 |

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
│              │          │      │ · ActionRUN()     │
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

## 核心模块详解

### 1. 动作执行引擎 — `user_tasks.c`

动作执行是固件的核心逻辑，管理从动作选择到舵机驱动的完整流程。

**动作状态枚举 `ACTION_STATE`**（定义在 `user_tasks.h`）：

```c
typedef enum {
    ACTION_TEACH = 0,        // 示教模式
    ACTION_STAND_BOW,        // 鞠躬
    ACTION_STAND_DANCE1,     // 跳舞1/2/3
    ACTION_STAND_STEPBACK,   // 后退
    ACTION_STAND_SALUTE,     // 敬礼
    ACTION_STAND_BLOWKISS,   // 飞吻
    ACTION_STAND_HANDSHAKE,  // 站立握手
    ACTION_STAND_PRAY,       // 拜一拜
    ACTION_SIT_HANDSHAKE,    // 坐-握手
    ACTION_SIT_HELLO,        // 坐-打招呼
    ACTION_SIT_STRETCH,      // 坐-伸懒腰
    ACTION_SIT_SHAKEHEAD,    // 坐-摇头
    ACTION_SIT_CHEER,        // 坐-加油
    ACTION_SIT_YAWN,         // 坐-打哈欠
    ACTION_SIT_DRUM,         // 坐-打鼓
    ACTION_SIT_WASHFACE,     // 坐-洗脸
    ACTION_LIE_WAGHIPS,      // 趴-扭屁股
    ACTION_LIE_PUSHUP,       // 趴-俯卧撑
    ACTION_LIE_CRAWL,        // 趴-爬行
    ACTION_SIT_TO_STAND,     // 坐→站
    ACTION_STAND_TO_SIT,     // 站→坐
    ACTION_SIT_TO_LIE,       // 坐→趴
    ACTION_LIE_TO_SIT,       // 趴→坐
    ACTION_LIE_TO_STAND,     // 趴→站
    ACTION_STAND_TO_LIE,     // 站→趴
    IDLE,                    // 空闲
} ACTION_STATE;
```

**步进插补机制**：

- TIM4 定时器每 **11ms** 触发一次 `User_TimerServoIRQ`，对所有舵机执行速度插补
- 插补逻辑：若当前角度与目标角度差值超过 `speed`，每次增加/减少 `speed` 个单位
- 每累计 `ACTIONTIMESTEP`（170ms）个计数，动作步进一步（`step_counter++`），更新目标角度
- 到达序列末尾时标记 `Action_done`，触发完成回调

**两种到位判断**：
- `_SingleAction_CheckApproch()`：Flash 动作库版本，操作 `ServoActionSeries`（const 数据）
- `_SingleAction_CheckApproch_Bezier()`：RAM/示教版本，操作 `ServoActionSeries_ram`（可变数据，支持贝塞尔插值）

### 2. 动作库 — `Action_Library.c/h`

动作数据以结构体层级组织：

```
Motion_t（完整运动，如"鞠躬"）
  ├── posestart       // 起始姿态（坐/站/趴）
  ├── poseend         // 结束姿态
  ├── point_total     // 动作序列总数
  └── motion[5]       // 最多5个 ServoActionSeries
       ├── actionId      // 序列唯一标识
       ├── total_step    // 步数
       ├── totalDuration // 总时长(ms)
       ├── emotionType   // 情绪类型
       ├── ifNeedBezier  // 是否贝塞尔插值
       └── actions[]     // ServoActionStep 数组
            └── servoAngles[14]  // 14个舵机目标角度
```

**两种存储版本**：
- `ServoActionSeries` + `ServoActionStep`：Flash 版本，`servoAngles` 为 `const`，编译期固化
- `ServoActionSeries_ram` + `ServoActionStep_ram`：RAM 版本，用于示教模式动态写入，增加了 `startservoAngles` 和 `endservoAngles`

**动作按姿态分组**：
- 站立动作：鞠躬、跳舞1/2/3、后退、敬礼、飞吻、握手、拜一拜
- 坐姿动作：握手、打招呼、伸懒腰、摇头、加油、打哈欠、打鼓、洗脸
- 趴姿动作：扭屁股、俯卧撑、爬行
- 姿态切换：坐⇄站、坐⇄趴、站⇄趴

### 3. 舵机控制 — `user_servo.c/h`

14 个 FEETECH 总线舵机通过 4 路 UART 分组控制：

| UART | 外设 | 舵机 ID | 部位 |
|------|------|---------|------|
| USART1 | 左腿 | 1-5 | 左腿各关节 |
| USART2 | 右腿 | 1-5 | 右腿各关节 |
| UART5 | 颈部 | 1-5 | 身体/颈部关节 |
| UART7 | 头部 | 1-5 | 头部/手部关节 |

**核心数据结构**：
```c
typedef struct {
    int16_t pos_set;      // 目标位置
    uint16_t ms_set;      // 运动时间(ms)
    uint16_t speed_set;   // 目标速度
    int16_t pos_read;     // 读取位置
    uint16_t speed_read;  // 读取速度
    uint16_t temper_read; // 读取温度
    uint16_t volt_read;   // 读取电压
    int16_t zero_ang;     // 零位角度偏移
    uint8_t AsynchronousWriteFlag;  // 异步写标志
    uint8_t servoStatus;  // 舵机状态
} SERVO_INFO_TYPEDEF;

extern SERVO_INFO_TYPEDEF SERVO[14];   // 14个舵机信息
extern int16_t goal_pos[15];           // 目标角度（索引1-14）
```

**关键函数**：
- `FEETECH_UsartSetServoPos()` — 单舵机位置+时间+速度设置
- `FEETECH_LEGSYNCWRITE()` — 腿部 5 舵机同步写（一条指令控制同组所有舵机）
- `FEETECH_HEADSYNCWRITE()` — 头部同步写
- `FEETECH_NECKSYNCWRITE()` — 颈部同步写
- `sevroSetMode()` — 设置舵机运行模式（0=位置模式, 2=零力矩模式）
- `User_BezierCurve()` — 贝塞尔曲线插值运动
- `sevroSetZero()` — 所有舵机归零

### 4. 上位机通信协议 — `user_communication.c/h`

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

**协议句柄**：
```c
typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma;
    uint8_t rx_buf[2][MAX_DATA_LEN + 10]; // 双缓冲
    volatile uint8_t buf_idx;              // 当前活跃缓冲区
    CmdState cmd_state;                    // 命令执行状态
    uint8_t current_cmd;                   // 当前命令功能码
} ProtocolHandle;
```

**传感器数据结构 `SensorData`**：
- `head_touch` / `body_touch` / `chin_touch` — 触摸传感器
- `human_Abdomen` / `human_Backside` — 人体感应
- `roll` / `pitch` / `yaw` — IMU 欧拉角

**工作状态 `WorkStatus`**：
- `mode` — 工作模式（正常/空闲/动作中）
- `voltage` / `battery_level` / `is_charging` — 电池信息
- `max_temp` / `warning_code` / `error_code` — 异常监控

**电源状态 `PowerType_T`**：
- `powerIdle` — 空闲
- `hibernate` — 休眠
- `wakeup` — 唤醒
- `Shutdown` — 关机
- `ActionReset` — 动作复位

### 5. IMU 姿态估计 — `user_imu_i2c.c/h`

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
- 结合加速度计综合判断

### 6. IAP 固件升级 — `user_IAP.c/h`

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

**关键函数**：
- `JumpToApp()` — 跳转到指定地址执行
- `IAP_write_App_fromuart()` — 从 UART 接收数据写入 Flash
- `MoveCode()` — 在两个 Bank 之间搬运固件
- `Start_BootLoader()` — 启动 BootLoader 流程

### 7. 定时器系统 — `user_timer.c/h`

| 定时器 | 周期 | 用途 |
|--------|------|------|
| TIM4 | 11ms | 舵机位置插补 `User_TimerServoIRQ` |
| TIM6 | 200ms | 示教模式步进 `User_TimerTeachIRQ` |
| TIM3 | — | 辅助定时 |
| TIM7 | — | HAL 时基 |
| TIM17 | — | 辅助 |

### 8. Flash 存储 — `user_flash.c/h`

- 存储起始地址：`0x0800FC00`
- 数据容量：512 × 4 字节（`STORE_COUNT = 512`）
- 扇区大小：128KB
- 支持读写、擦除、校验操作
- 用于保存配置参数和校准数据

### 9. ADC 电池监测 — `user_adc.c/h`

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
