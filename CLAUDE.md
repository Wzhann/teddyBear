# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

PandaRobot 是一个基于 STM32 的双足熊猫机器人嵌入式固件，配备 14 个舵机。机器人支持坐、站、趴三种姿态，拥有预编排动作库、示教模式、IMU 姿态估计，以及基于 UART 的通信协议用于上位机控制。代码注释以中文为主。

## 构建系统与工具链

- **IDE**：Keil MDK-ARM（µVision 5）
- **工程文件**：`MDK-ARM/Panda_Robot.uvprojx`
- **编译器**：ARMCLANG V6.19（AC6）
- **目标 MCU**：STM32H743VITx（Cortex-M7, 480MHz, 双 Bank Flash）
- **HAL 配置生成工具**：STM32CubeMX（仓库根目录 `.ioc` 文件：`Panda_Robot.ioc`）
- **构建输出**：HEX 文件在 `MDK-ARM/` 目录下

构建方式：在 Keil 中打开 `Panda_Robot.uvprojx` 后编译。无命令行 Makefile 或 CMake 构建。

## 代码架构

### 目录结构

- **`Core/`** — STM32CubeMX 生成的 HAL 外设初始化代码（自动生成，仅在 `USER CODE BEGIN/END` 标记之间编辑）
  - `Src/`：main.c, freertos.c, gpio.c, adc.c, dma.c, tim.c, usart.c, stm32h7xx_it.c, stm32h7xx_hal_msp.c
  - `Inc/`：对应头文件 + FreeRTOSConfig.h, stm32h7xx_hal_conf.h
- **`Users/`** — 所有自定义应用代码（主要开发区域）
- **`Drivers/`** — STM32 HAL 和 CMSIS 库（不要修改）
- **`Middlewares/Third_Party/FreeRTOS/`** — FreeRTOS 内核（不要修改）

### FreeRTOS 任务结构

在 `Core/Src/freertos.c` 中创建了 5 个任务，实际实现在 `Users/user_tasks.c` 中：

| 任务 | 优先级 | 栈大小 | 功能 |
|------|--------|--------|------|
| `TaskHigh` | osPriorityRealtime | 128 | 舵机/UART 初始化、ADC 初始化、动作库初始化（`User_Init_HIGH`） |
| `defaultTask` | osPriorityNormal | 128 | 默认任务（weak 函数，空循环） |
| `TaskMid` | osPriorityNormal | 2048 | 主动作循环（`ActionRUN`/`TeachmodeRUN`） |
| `TaskLow` | osPriorityLow | 128 | IMU 初始化（`User_Init_LOW`） |
| `myTask05` | osPriorityLow | 128 | 低优先级后台任务 |

### 核心应用模块（Users/）

- **`user_tasks.c/h`** — 核心任务逻辑：动作执行引擎、运动步进、单步动作到位判断（Flash 版本和 RAM/贝塞尔版本）、动作复位。包含 `ACTION_STATE` 枚举，覆盖所有机器人动作（示教、坐/站/趴姿态、姿态切换、空闲）。
- **`Action_Library.c/h`** — 预编排动作数据：舵机角度序列存储为 `ServoActionSeries`（Flash）和 `ServoActionSeries_ram`（RAM，用于示教）。动作按姿态分组（坐、站、趴），带情绪类型。`Motion_t` 将最多 5 个动作序列组合为完整运动。
- **`user_servo.c/h`** — FEETECH 舵机控制，通过 UART 通信。14 个舵机（12 个腿/身体 + 2 个头部），分为 4 路 UART 总线（左腿、右腿、颈部、头部）。支持位置/速度/时间写入、同步写、贝塞尔曲线插值和模式切换。
- **`user_communication.c/h`** — 自定义 UART 通信协议，CRC16 校验。帧格式：`帧头(2B) + 功能码(1B) + 数据长度(2B) + 数据(最大128B) + 校验(2B) + 帧尾(2B)`。双缓冲 DMA 接收。管理传感器数据（触摸、IMU、温度）、电源状态（休眠/唤醒/关机）、工作状态上报和命令分发。
- **`user_imu_i2c.c/h`** — MPU6050 IMU，软件 I2C 通信，卡尔曼滤波角度估计，姿态检测（基于俯仰角范围判断坐/站/趴）。
- **`user_flash.c/h`** — Flash 存储，起始地址 0x0800FC00，用于持久化数据（128KB 扇区）。
- **`user_IAP.c/h`** — 在应用编程（IAP）：双 Bank 固件更新（APP1 地址 0x08020000，APP2 地址 0x08100000），支持 UART OTA。
- **`user_timer.c/h`** — 硬件定时器回调：舵机步进（TIM4，11ms 周期）和示教模式（TIM6，200ms 周期）。`ACTIONTIMESTEP = 170` 控制动作步进时间。
- **`user_adc.c/h`** — ADC+DMA 电池电压和电流监测。
- **`DS18B20.c/h`** — DS18B20 温度传感器（最多 2 个），OneWire 协议，GPIO PE0。
- **`user_iic.c/h`** / **`Myiic_IMU.c/h`** — 软件 I2C 实现，用于 IMU 通信。

### 数据流

1. 上位机通过 UART4/UART5 发送命令（DMA 双缓冲接收）
2. `user_communication` 解析协议帧，分发到动作/传感器/电源处理器
3. 通过 `ACTION_STATE` 枚举选择动作，以 `Motion_t` 序列执行
4. 定时器中断（`User_TimerServoIRQ`，11ms）更新舵机位置，基于速度插值
5. 动作步进由 `timerStepForAction` 计数器（TIM4）驱动，间隔 `ACTIONTIMESTEP`

### 头文件引用约定

`user_includes.h` 是公共头文件枢纽——包含 HAL、FreeRTOS 和所有用户模块头文件。大多数 `Users/` 文件通过 `user_includes.h` 或自身头文件（间接链入）来引用。

## STM32CubeMX 代码生成

`Core/` 下的文件由 CubeMX 自动生成。自定义代码**只能**放在 `/* USER CODE BEGIN */` 和 `/* USER CODE END */` 标记之间，否则重新生成时会被覆盖。可在 STM32CubeMX 中打开 `.ioc` 文件重新配置外设。

## 文件编码

源文件使用 **GB2312/GBK** 编码，而非 UTF-8。编辑源文件时需注意编码一致性，避免中文注释乱码。

## 重要常量

- **舵机数量**：14 个（索引 1–14；1–6 腿部，7–12 身体，13–14 头部）
- **动作步进定时**：11ms 舵机插补，170ms（`ACTIONTIMESTEP`）动作步进
- **示教模式定时**：200ms 每步
- **Flash 应用地址**：APP1 = 0x08020000，APP2 = 0x08100000
- **电池电压范围**：10V–12.4V，上电阈值 11.4V
- **姿态检测俯仰角范围**：直立 48°–120°，趴 -40°–48°
