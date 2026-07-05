#include "user_communication.h"
#include <string.h>
#include "main.h"
#include "user_imu_i2c.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "cmsis_os.h"
#include "user_servo.h"
#include "user_flash.h"
#include "user_IAP.h"
#include "DS18B20.h"
#include "usart.h"

USART_SERVO_TYPEDEF USART_ONE = {0};
ProtocolHandle ph;        // 协议句柄实例
uint8_t actionStop = 0;   // 动作停止标志
uint8_t actioninIdle = 0; // 空闲默认动作
WorkStatus stateRobot;
ACTION_STATE ActionReceive;
IOfunctionState ioState;
PowerType_T powerState_t;
uint32_t flagForUpdate[8] = {0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa};

uint8_t actionINDEXForshow1[13] = {43, 47, 48, 49, 45, 53, 58, 55, 56, 73, 75, 167, 29};
uint8_t showModeFlag = 0;

void SoftwareReset(void)
{
    __set_FAULTMASK(1); // 关闭所有中断
    NVIC_SystemReset(); // 触发系统复位
}

// ====== IO 控制模块 ======
/*LED:0=熄灭,1=点亮*/
void ledSet(uint8_t mode)
{
    if (mode == 1)
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    else if (mode == 0)
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    else
        ;
}
/*Buzzer:0=停止,1=响*/
void buzzerSet(uint8_t mode)
{
    if (mode == 1)
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    else if (mode == 0)
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    else
        ;
}
/*FAN:0=关,1=开*/
void fanSet(uint8_t mode)
{
    if (mode == 1)
        HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
    else if (mode == 0)
        HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
    else
        ;
}

void RGB_Flash_InOneSecond(uint8_t times)
{
    if (times == 0)
    {
        ledSet(LEDLIGHT_OFF);
        osDelay(pdMS_TO_TICKS(1000));
        return;
    }
    if (times > 10)
    {
        // 若times过大，可能导致每次闪烁时间过短，影响视觉效果
        // 建议在上层限制times值或优化调度逻辑
        times = 10; // 默认闪烁次数
    }

    uint32_t flash_duration = pdMS_TO_TICKS(1000 / times); // 每次闪烁总持续时间(tick)
    uint32_t half_flash = flash_duration / 2;              // 熄灭和点亮各占一半时间

    for (int i = 0; i < times; i++)
    {
        ledSet(LEDLIGHT_OFF);
        osDelay(half_flash);
        ledSet(LEDLIGHT_ON);
        osDelay(half_flash);
    }
    ledSet(LEDLIGHT_OFF);
    osDelay(pdMS_TO_TICKS(1000));
}

void BUZZER_Flash_InOneSecond(uint8_t times)
{
    if (times == 0)
    {
        buzzerSet(BUZZER_OFF);
        osDelay(pdMS_TO_TICKS(1000));
        return;
    }
    if (times > 10)
    {
        // 若times过大，可能导致每次闪烁时间过短，影响视觉效果
        // 建议在上层限制times值或优化调度逻辑
        times = 10; // 默认闪烁次数
        buzzerSet(BUZZER_ON);
    }
    else
    {
        uint32_t flash_duration = pdMS_TO_TICKS(1000 / times); // 每次闪烁总持续时间(tick)
        uint32_t half_flash = flash_duration / 2;              // 熄灭和点亮各占一半时间

        for (int i = 0; i < times; i++)
        {
            buzzerSet(BUZZER_OFF);
            osDelay(half_flash);
            buzzerSet(BUZZER_ON);
            osDelay(half_flash);
        }
        buzzerSet(BUZZER_OFF);
        osDelay(pdMS_TO_TICKS(1000));
    }
}

int getHorizontalAng(void)
{
    int angleX;
    angleX = (int)(SERVO[11].pos_read - servo11_mid) / 4096.0 * 360.0;
    //	if(angleX > 200) angleX -=360;
    return angleX;
}

int getVerticalAng(void)
{
    int angleY;
    angleY = (int)(SERVO[12].pos_read - servo12_mid) / 4096.0 * 360.0;
    return angleY;
}

/**
 * @brief 通信模块初始化
 * @note 配置UART和DMA，开启接收中断
 */
void User_CommunicationInit(void)
{
    ph.huart = &huart4;       // UART句柄
    ph.hdma = &hdma_uart4_rx; // DMA句柄
    ph.buf_idx = 0;           // DMA环缓冲使用 rx_buf[0]

    // 使用 HAL IDLE-To-DMA API：自动处理 IDLE 中断 + DMA 持续接收
    HAL_UARTEx_ReceiveToIdle_DMA(ph.huart, (uint8_t *)ph.rx_buf[ph.buf_idx], MAX_DATA_LEN + 9);

    //	HAL_Delay(1);
    // USART_ONE.p_usart_n = &huart7;
    // USART_ONE.p_hdma_usart_n_rx = &hdma_uart7_rx;

    // //旧版按键中断检测(已废弃)
    // __HAL_UART_ENABLE_IT(USART_ONE.p_usart_n, UART_IT_IDLE);
    // HAL_UART_Receive_DMA(USART_ONE.p_usart_n, (uint8_t*)USART_ONE.usart_rx_buf, USART_SERVO_RX_SIZE);
}

void Single_Key_Record(uint8_t *key, uint8_t *last_key, uint8_t *key_downside, uint8_t *key_upside)
{
    // 下降沿检测
    if ((*last_key == 1) && (*key == 0))
    {
        *key_downside = 1;
    }
    else
    {
        *key_downside = 0;
    }
    // 上升沿检测
    if ((*last_key == 0) && (*key == 1))
    {
        *key_upside = 1;
    }
    else
    {
        *key_upside = 0;
    }

    *last_key = *key;
}

uint8_t touchTopofHead_Downside, touchTopofHead_Upside, touchTopofHead, Last_touchTopofHead;                                 // 头顶触摸
uint8_t touchChin_Downside, touchChin_Upside, touchChin, Last_touchChin;                                                     // 下巴触摸
uint8_t touchBody_Downside, touchBody_Upside, touchBody, Last_touchBody;                                                     // 身体触摸
uint8_t humanDetectionAbdomen_Downside, humanDetectionAbdomen_Upside, humanDetectionAbdomen, Last_humanDetectionAbdomen;     // 人体感应-腹部
uint8_t humanDetectionBackside_Downside, humanDetectionBackside_Upside, humanDetectionBackside, Last_humanDetectionBackside; // 人体感应-背部
uint8_t IOForCharging_Downside, IOForCharging_Upside, IOForCharging, Last_IOForCharging;                                     // 充电IO
void Key_Downside_Record(void)
{

    touchTopofHead = HAL_GPIO_ReadPin(TOUCH0_HEAD_GPIO_Port, TOUCH0_HEAD_Pin);
    touchChin = HAL_GPIO_ReadPin(TOUCH1_MOUTH_GPIO_Port, TOUCH1_MOUTH_Pin);
    touchBody = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    humanDetectionBackside = !(HAL_GPIO_ReadPin(Abdomen_GPIO_Port, Abdomen_Pin));
    humanDetectionAbdomen = !(HAL_GPIO_ReadPin(Backside_GPIO_Port, Backside_Pin));

    Single_Key_Record(&touchTopofHead, &Last_touchTopofHead, &touchTopofHead_Downside, &touchTopofHead_Upside);
    Single_Key_Record(&touchChin, &Last_touchChin, &touchChin_Downside, &touchChin_Upside);
    Single_Key_Record(&touchBody, &Last_touchBody, &touchBody_Downside, &touchBody_Upside);
    Single_Key_Record(&humanDetectionAbdomen, &Last_humanDetectionAbdomen, &humanDetectionAbdomen_Downside, &humanDetectionAbdomen_Upside);
    Single_Key_Record(&humanDetectionBackside, &Last_humanDetectionBackside, &humanDetectionBackside_Downside, &humanDetectionBackside_Upside);
}

void IOForChargingDownside(void)
{
    IOForCharging = USER_ADC.bat_charging;
    Single_Key_Record(&IOForCharging, &Last_IOForCharging, &IOForCharging_Downside, &IOForCharging_Upside);
}
/*------------------------ 简单校验和 ------------------------*/

/**
 * @brief 计算简单和校验(16位)
 * @param data 待校验数据指针
 * @param len 校验数据长度(字节数)
 * @return 计算得到的16位校验值
 */
uint16_t Calculate_SumCheck(uint8_t *data, uint16_t len)
{
    uint16_t sum = 0;
    while (len--)
    {
        sum += *data++;
    }
    return sum;
}

// ====================== 工作模式相关函数 ======================
/**
 * @brief 获取当前工作模式
 * @return 工作模式枚举值
 */
WorkMode Get_WorkMode(void)
{
    if (ActionNow == IDLE)
        return MODE_IDLE;
    else
        return MODE_ACTION;
}

uint8_t sevroErrorsum;
uint8_t sevroErrorID;
uint8_t sevroErrorNum;
uint8_t sevroErrorPara()
{
    sevroErrorsum = 0;
    for (uint8_t i = 1; i <= 12; i++)
    {
        if (SERVO[i].servoStatus != 0)
        {
            sevroErrorID = i;
            sevroErrorsum++;
        }
    }
    if (sevroErrorsum != 0)
    {
        sevroErrorNum = SERVO[sevroErrorID].servoStatus;
    }
    return sevroErrorNum;
}

/**
 * @brief 获取电源电压
 * @return 当前电压值(单位:伏特)
 */
float Power_GetVoltage(void)
{
    // 临时返回0电压
    //    return USER_ADC.bat_volt;
    return 0;
}

/**
 * @brief 获取电池电量百分比
 * @return 电池百分比(0-100)
 */
uint8_t Get_BatteryLevel(void)
{
    // 临时返回0%电量
    return USER_ADC.bat_power;
    //	return 0;
}

/**
 * @brief 检查充电状态
 * @return true:正在充电 false:未充电
 */
bool Is_Charging(void)
{
    // 临时返回未充电状态
    return USER_ADC.bat_charging;
}

extern uint8_t poseNow;
uint8_t Get_LastPandaPose(void)
{
    if (poseNow == 1)
        return 1;
    else if (poseNow == 2 || poseNow == 3)
        return 2;
    else
        return 3;
}
/**
 * @brief 获取系统最高温度
 * @return 最高温度值(单位:摄氏度)
 */
extern SERVO_INFO_TYPEDEF SERVO[14];
int Get_MaxTemperature(void)
{
    float tempMax = 0;
    for (uint8_t i = 1; i <= 12; i++)
    {
        if (SERVO[i].temper_read >= tempMax)
            tempMax = SERVO[i].temper_read;
    }
    return tempMax;
}

/**
 * @brief 获取当前警告码
 * @return 警告码(0表示无警告)
 */
uint16_t Get_Warning(void)
{
    // 临时返回无警告
    return 0;
}

extern uint8_t poseNow;
/**
 * @brief 获取最后错误码
 * @return 错误码(0表示无错误)
 */
uint16_t Get_LastError(void)
{
    uint8_t lasterror;
    if (poseNow == 4)
        lasterror = 1;
    else
        lasterror = 0;

    // 临时返回无错误
    return lasterror;
}

// ====================== 触摸传感器函数 ======================
/**
 * @brief 获取最后错误码ֵ
 * @param touch_type 触摸类型(HEAD/BODY/CHIN)
 * @return 触摸状态:0/1或ADC原始值
 */
uint8_t Get_TouchValue(TouchType touch_type)
{
    uint8_t value_Touch;
    switch (touch_type)
    {
    case TOUCH_HEAD:
        value_Touch = !touchTopofHead;
        break;

    case TOUCH_BODY:
        value_Touch = !touchBody;
        break;

    case TOUCH_CHIN:
        value_Touch = !touchChin;
        break;
    }
    // 临时返回无触摸信号
    //    (void)touch_type; // 参数暂未使用
    return 0;
}

/**
 * @brief 获取IMU俯仰角
 * @return 横滚角度(单位:度,-180~180)
 */
float IMU_GetRoll(void)
{
    // 临时返回0度横滚角
    return 0.0f;
}

/**
 * @brief 获取IMU俯仰角
 * @return 俯仰角度(单位:度,-90~90)
 */
float IMU_GetPitch(void)
{
    // 临时返回0度俯仰角
    return 0.0f;
}

/**
 * @brief 获取IMU偏航角
 * @return 偏航角度(单位:度,0~360)
 */
float IMU_GetYaw(void)
{
    // 临时返回0度偏航角
    return 0.0f;
}

/* 警告/错误码转换可读提示 */
/**
 * @brief 错误码转字符串
 * @param code 错误码
 * @return 可读的警告信息
 */
char *WarningCodeToString(uint16_t code)
{
    switch (code)
    {
    case 0:
        return "none";
    case 1:
        return "low battery";
    case 2:
        return "high temp";
    default:
        return "unknown";
    }
}

/**
 * @brief 错误码转字符串
 * @param code 错误码
 * @return 可读的错误信息
 */
char *ErrorCodeToString(uint16_t code)
{
    switch (code)
    {
    case 0:
        return "none";
    case 1:
        return "motor fault";
    case 2:
        return "sensor error";
    default:
        return "unknown";
    }
}

uint8_t tx_buf_[136];
uint8_t zeroBuf[136] = {0};
extern uint8_t dma_done;
/**
 * @brief 发送带传感器数据的响应帧
 * @param ph 协议句柄指针
 * @param data 要发送的JSON数据字符串
 * @param len JSON数据长度
 * @note 帧结构:帧头(2B)|功能码(1B)|数据长度(2B)|JSON数据(NB)|校验(1B)|帧尾(2B)
 */
uint8_t Send_Sensor_Data(ProtocolHandle *ph, const char *data, uint16_t len, uint8_t code)
{
    // 校验数据长度合法性
    if (len > MAX_DATA_LEN + 8)
    {
        return 0;
    }

    // 动态分配发送缓冲区(计算帧长度)
    //    uint8_t tx_buf[2 + 2 + 5 + len]; // 帧头帧尾4+功能码1+数据长度2+校验1
    uint16_t frame_len = 2 + 2 + 5 + len;

    // 帧头(2字节)
    tx_buf_[0] = FRAME_HEADER_DOWN >> 8;
    tx_buf_[1] = FRAME_HEADER_DOWN & 0xFF;

    // 功能码(1字节)
    tx_buf_[2] = code;

    // 数据长度(小端格式,2字节)
    tx_buf_[3] = len & 0xFF;
    tx_buf_[4] = (len >> 8) & 0xFF;

    // 数据负载
    memcpy(&tx_buf_[5], data, len);

    // 计算校验和(功能码+数据长度+数据)
    uint16_t checksum = Calculate_SumCheck(&tx_buf_[3], len);
    tx_buf_[5 + len] = checksum;

    // 帧尾(2字节)
    tx_buf_[6 + len] = FRAME_FOOTER_DOWN >> 8;
    tx_buf_[7 + len] = FRAME_FOOTER_DOWN & 0xFF;

    // DMA发送
    HAL_UART_Transmit(ph->huart, tx_buf_, 8 + len, 100);

    return 1;
    //	memcpy(tx_buf_, zeroBuf, 8+len);
}

/*------------------------ 舵机参数提取函数 ------------------------*/
/**
 * @brief 获取所有舵机参数
 * @param params 舵机参数结构体数组
 * @note SERVO[1]-SERVO[12] 对应 params[0]-params[11]
 */
void Servo_GetAllParams(SERVO_INFO_TYPEDEF *params)
{
    // SERVO[1]-SERVO[12] 对应 params[0]-params[11]
    for (int i = 0; i < 12; i++)
    {
        params[i] = SERVO[i + 1]; // 舵机编号偏移
    }
}

/**
 * @brief 发送标准响应帧
 * @param ph 协议句柄指针
 * @param result 响应结果码(0=成功,1=校验失败,2=开始执行,3=执行完毕)
 * @note 帧结构:BB BB|func|01 00|result|校验|++ ++
 */
uint8_t _tx_buf_[10] = {0x42, 0x42, 0, 1, 0, 0, 0, 0, 0x2B, 0x2B};
void Send_Response(ProtocolHandle *ph, uint8_t result)
{
    //    uint8_t tx_buf[10] = {
    //        FRAME_HEADER_DOWN >> 8, FRAME_HEADER_DOWN & 0xFF, // ֡ͷBB BB
    //        ph->current_cmd,                                  // 原样回传功能码
    //        1, 0,                                             // 数据长度小端,固定1字节
    //        result,                                           // 结果码
    //        0, 0,                                             // 校验占位
    //        FRAME_FOOTER_DOWN >> 8, FRAME_FOOTER_DOWN & 0xFF  // ֡β++ ++
    //    };
    _tx_buf_[2] = ph->current_cmd;
    _tx_buf_[5] = result;
    // 计算校验和(功能码+数据长度+结果码)
    uint16_t checksum = Calculate_SumCheck(&_tx_buf_[5], 1);
    _tx_buf_[6] = checksum & 0xFF; // 校验低字节在前
    _tx_buf_[7] = checksum >> 8;   // 高字节在后

    HAL_UART_Transmit(ph->huart, _tx_buf_, sizeof(_tx_buf_), 50);
}

void sendStateActive(ProtocolHandle *ph, WorkStatus status)
{
    /* 获取工作状态数据 */
    status.mode = Get_WorkMode();              // ģʽ
    status.voltage = Power_GetVoltage();       // 电压值(float)
    status.battery_level = Get_BatteryLevel(); // 电池百分比(0~100)
    status.is_charging = Is_Charging();        // 充电状态
    status.error_code = Get_LastError();
    status.posePanda = Get_LastPandaPose();
    status.max_temp = Get_MaxTemperature();
    status.tempBoard = (int)DS18B20.temper[1];
    status.sevroerror = sevroErrorPara();

    /* 构建精简JSON格式状态数据 */
    char json_buf[128]; // 适当大小的缓冲区
    snprintf(json_buf, sizeof(json_buf),
             "{\"mode\":%d,\"battery\":%d,\"charging\":%s,\"error\":%d,\"pose\":%d,\"temp1\":%d,\"temp2\":%d,\"error2\":%d}",
             status.mode,
             status.battery_level, //
             status.is_charging ? "true" : "false",
             status.error_code,
             status.posePanda,
             status.max_temp,
             status.tempBoard,
             status.sevroerror);

    ph->cmd_state = CMD_RECEIVED;

    /* 发送状态数据 */
    Send_Sensor_Data(ph, json_buf, strlen(json_buf), 5);
}

void sendSensorActive(ProtocolHandle *ph, uint8_t head, uint8_t body, uint8_t chin, uint8_t abdomen, uint8_t backside)
{
    // 构建传感器数据
    SensorData sensor = {
        .head_touch = head,
        .body_touch = body,
        .chin_touch = chin,
        .human_Abdomen = abdomen,
        .human_Backside = backside};
    //            .roll = IMU_GetRoll(),
    //            .pitch = IMU_GetPitch(),
    //            .yaw = IMU_GetYaw()};

    // 转换为JSON字符串
    char json_buf[128];
    snprintf(json_buf, sizeof(json_buf),
             //                 "{\"touch\":[%u,%u,%u],\"pose\":[%.1f,%.1f,%.1f]}",
             "{\"Head\":%d,\"Body\":%d,\"Chin\":%d,\"HA\":%d,\"HB\":%d}",
             //		"{\"touch\":[%d,%d,%d]}",
             sensor.head_touch, sensor.body_touch, sensor.chin_touch,
             sensor.human_Abdomen, sensor.human_Backside);
    ph->cmd_state = CMD_RECEIVED; // 标记命令已接收状态
                                  //        Send_Response(ph, ph->cmd_state); // 响应接收成功
    // 发送响应帧
    Send_Sensor_Data(ph, json_buf, strlen(json_buf), 4);
}

///* 映射id */
// void mappingID(uint16_t receivedID,uint16_t )
//{
//
// }
extern uint8_t PoweronAction;
extern uint8_t actionPoseLast;
extern uint8_t step_counter;
extern uint8_t ifStartAct;
extern uint8_t flag_sendCompleted;
extern uint8_t flag_sendExecuting;
extern uint8_t sendmodework;
extern uint16_t actionSwitchTime;
extern uint8_t actionNeedReturn;
extern uint8_t releaseSevroFlag;
extern uint8_t debugUse;
uint8_t indexFortrulData;
uint8_t actionFromemotion = 0;
ProtocolFrame *frame;
/**
 * @brief 协议帧处理主逻辑
 * @param ph 协议句柄指针
 * @note 执行顺序:帧结构校验 -> 和校验 -> 命令分发
 */
void Parse_Protocol(ProtocolHandle *ph)
{
    for (uint8_t i = 0; i < 30; i++)
    {
        if (ph->rx_buf[ph->buf_idx][i] == 0x41 && ph->rx_buf[ph->buf_idx][i + 1] == 0x41)
        {
            indexFortrulData = i;
            break;
        }
        //		else
    }
    frame = (ProtocolFrame *)&ph->rx_buf[ph->buf_idx][indexFortrulData];
    frame->checksum = (frame->data[frame->data_len + 1] << 8) | frame->data[frame->data_len];
    frame->footer = *(uint16_t *)&frame->data[frame->data_len + 2];

    // 初步检查帧头和帧尾
    //    for (int i = 0; i < 4; i++)
    //        frame->data[frame->data_len + i] = 0;

    /* 帧结构校验 */
    if (frame->header != FRAME_HEADER_UP || // 验证上位机帧头
        frame->footer != FRAME_FOOTER_UP || // 验证上位机帧尾
        frame->data_len > MAX_DATA_LEN)     // 数据长度合法性检查
    {
        return;
    }

    /* 和校验验证 */
    uint16_t calc_sum = Calculate_SumCheck(frame->data, frame->data_len);
    if (calc_sum != frame->checksum)
    {
        ph->cmd_state = CMD_CHECK_ERROR;  // 标记校验错误状态
        Send_Response(ph, ph->cmd_state); // 响应校验失败
        return;
    }

    ph->current_cmd = frame->func; // 记录当前命令功能码

    /* 命令分发处理 */
    switch (frame->func)
    {
    case 0x01: // 情绪状态触发动作
        if (frame->data_len == 2)
        {
            actionFromemotion = 1;
            // 设置情绪状态
            ph->cmd_state = CMD_RECEIVED;
            Send_Response(ph, ph->cmd_state); // 首次响应接收成功
                                              // 预期2字节数据
            // 解析状态和程度值
            uint8_t state = frame->data[0];
            uint8_t level = frame->data[1];

            //					// 旧版随机数种子
            //						srand((unsigned)time(NULL));
            // 生成随机数
            int random_number = (rand() / 10) % 10;

            // 随机选择
            switch (random_number)
            {
            case 0:
            {
                if (state == 1)
                    ActionNow = IDLE;
                else if (state == 2)
                    ActionNow = IDLE;
                else if (state == 3)
                    ActionNow = IDLE;
                else if (state == 4)
                    ActionNow = IDLE;
                else if (state == 5)
                    ActionNow = IDLE;
                else if (state == 6)
                    ActionNow = IDLE;
                else if (state == 7)
                    ActionNow = IDLE;
                else if (state == 8)
                    ActionNow = IDLE;
                else if (state == 9)
                    ActionNow = IDLE;
                break;
            }
            case 1:
            {
                if (state == 1)
                    ActionNow = IDLE;
                else if (state == 2)
                    ActionNow = IDLE;
                else if (state == 3)
                    ActionNow = IDLE;
                else if (state == 4)
                    ActionNow = IDLE;
                else if (state == 5)
                    ActionNow = IDLE;
                else if (state == 6)
                    ActionNow = IDLE;
                else if (state == 7)
                    ActionNow = IDLE;
                else if (state == 8)
                    ActionNow = IDLE;
                else if (state == 9)
                    ActionNow = IDLE;
                break;
            }
            case 2:
            {
                if (state == 1)
                    ActionNow = IDLE;
                else if (state == 2)
                    ActionNow = IDLE;
                else if (state == 3)
                    ActionNow = IDLE;
                else if (state == 4)
                    ActionNow = IDLE;
                else if (state == 5)
                    ActionNow = IDLE;
                else if (state == 6)
                    ActionNow = IDLE;
                else if (state == 7)
                    ActionNow = IDLE;
                else if (state == 8)
                    ActionNow = IDLE;
                else if (state == 9)
                    ActionNow = IDLE;
                break;
            }
            case 3:
            {
                if (state == 1)
                    ActionNow = IDLE;
                else if (state == 2)
                    ActionNow = IDLE;
                else if (state == 3)
                    ActionNow = IDLE;
                else if (state == 4)
                    ActionNow = IDLE;
                else if (state == 5)
                    ActionNow = IDLE;
                else if (state == 6)
                    ActionNow = IDLE;
                else if (state == 7)
                    ActionNow = IDLE;
                else if (state == 8)
                    ActionNow = IDLE;
                else if (state == 9)
                    ActionNow = IDLE;
                break;
            }
            case 4:
            {
                if (state == 1)
                    ActionNow = IDLE;
                else if (state == 2)
                    ActionNow = IDLE;
                else if (state == 3)
                    ActionNow = IDLE;
                else if (state == 4)
                    ActionNow = IDLE;
                else if (state == 5)
                    ActionNow = IDLE;
                else if (state == 6)
                    ActionNow = IDLE;
                else if (state == 7)
                    ActionNow = IDLE;
                else if (state == 8)
                    ActionNow = IDLE;
                else if (state == 9)
                    ActionNow = IDLE;
                break;
            }
                //						case 5:
                //							{
                //								if(state ==1)ActionNow = IDLE;
                //								else if(state == 2)ActionNow = IDLE;
                //								else if(state == 3)ActionNow = IDLE;
                //								else if(state == 4)ActionNow = IDLE;
                //								else if(state == 5)ActionNow = IDLE;
                //								else if(state == 6)ActionNow = IDLE;
                //								else if(state == 7)ActionNow = IDLE;
                //								else if(state == 8)ActionNow = IDLE;
                //								else if(state == 9)ActionNow = IDLE;
                //								break;
                //							}
                //							case 6:
                //							{
                //								if(state ==1)ActionNow = IDLE;
                //								else if(state == 2)ActionNow = IDLE;
                //								else if(state == 3)ActionNow = IDLE;
                //								else if(state == 4)ActionNow = IDLE;
                //								else if(state == 5)ActionNow = IDLE;
                //								else if(state == 6)ActionNow = IDLE;
                //								else if(state == 7)ActionNow = IDLE;
                //								else if(state == 8)ActionNow = IDLE;
                //								else if(state == 9)ActionNow = IDLE;
                //								break;
                //							}
                //							case 7:
                //							{
                //								if(state ==1)ActionNow = IDLE;
                //								else if(state == 2)ActionNow = IDLE;
                //								else if(state == 3)ActionNow = IDLE;
                //								else if(state == 4)ActionNow = IDLE;
                //								else if(state == 5)ActionNow = IDLE;
                //								else if(state == 6)ActionNow = IDLE;
                //								else if(state == 7)ActionNow = IDLE;
                //								else if(state == 8)ActionNow = IDLE;
                //								else if(state == 9)ActionNow = IDLE;
                //								break;
                //							}
                //							case 8:
                //							{
                //								if(state ==1)ActionNow = IDLE;
                //								else if(state == 2)ActionNow = IDLE;
                //								else if(state == 3)ActionNow = IDLE;
                //								else if(state == 4)ActionNow = IDLE;
                //								else if(state == 5)ActionNow = IDLE;
                //								else if(state == 6)ActionNow = IDLE;
                //								else if(state == 7)ActionNow = IDLE;
                //								else if(state == 8)ActionNow = IDLE;
                //								else if(state == 9)ActionNow = IDLE;
                //								break;
                //							}
                //							case 9:
                //							{
                //								if(state ==1)ActionNow = IDLE;
                //								else if(state == 2)ActionNow = IDLE;
                //								else if(state == 3)ActionNow = IDLE;
                //								else if(state == 4)ActionNow = IDLE;
                //								else if(state == 5)ActionNow = IDLE;
                //								else if(state == 6)ActionNow = IDLE;
                //								else if(state == 7)ActionNow = IDLE;
                //								else if(state == 8)ActionNow = IDLE;
                //								else if(state == 9)ActionNow = IDLE;
                //								break;
                //							}
            }
        }
        break;

    case 0x02: // 动作控制
    {
        //		releaseSevroFlag = 1;
        actionFromemotion = 0;
        TEACHMODE = 0; // 退出示教模式标志(确保执行)
        actionSwitchTime = ACTIONTIMESTEP;
        actionNeedReturn = 0;
        flag_sendExecuting = 0;
        // 首次响应接收成功
        //		if(debugUse >1)
        ph->cmd_state = CMD_RECEIVED;
        Send_Response(ph, ph->cmd_state);

        // 解析动作编号(范围0-255)
        uint16_t action_id = *(uint16_t *)&frame->data[0];

        // 验证动作编号有效性(示例:有效范围0-121)
        //        else if (action_id > 150)
        //        {
        //            ph->cmd_state = CMD_CHECK_ERROR;
        //            Send_Response(ph, ph->cmd_state);
        //            break;
        //        }
        actionPoseLast = poseCheck();

        if (action_id == 255)
        {
            step_counter = 1;
            PoweronAction = 0;
            actionStop = 1;
            ifStartAct = 0;
            flag_sendCompleted = 0;
            flag_sendExecuting = 0;
            sendmodework = 0;
            ActionNow = IDLE;
            Send_Response(ph, 0x03); // 发送执行完成应答
        }
        else if (action_id == 254)
        {
            PoweronAction = 0;
            actionStop = 0;
            ph->cmd_state = CMD_RECEIVED;
            Send_Response(ph, ph->cmd_state); // 再次响应
            ActionNow = IDLE;
        }
        else if (action_id == 1)
        {
            PoweronAction = 0;
            // 重置动作控制参数
            actionStop = 0;
            ph->cmd_state = CMD_RECEIVED;
            Send_Response(ph, ph->cmd_state); // 再次响应
            step_counter = 1;
            ActionNow = action_id; // 设置当前动作
        }
        else if (action_id == 253)
        {
            actionStop = 0;
            if (showModeFlag == 0)
            {
                showModeFlag = 1;
                ActionNow = actionINDEXForshow1[0];
            }
            //			if(PoweronAction == 0)
            //			{
            //				PoweronAction = 1;
            //				ActionNow = actionINDEXForshow1[0];
            //			}
        }
        //		else if(action_id == 2 ||action_id == 6||action_id == 26||action_id == 14||action_id == 23
        //			||action_id == 196||action_id == 59||action_id == 211||action_id == 77||action_id == 83||action_id == 1)
        else
        {
            // 执行动作
            int random_number = (rand() / 10) % 5;
            //			switch (random_number)
            //			{
            //				case 0:
            //					action_id = 2;
            //				break;
            //				case 1:
            //					action_id = 6;
            //				break;
            //				case 2:
            //					action_id = 26;
            //				break;
            //				case 3:
            //					action_id = 14;
            //				break;
            //				case 4:
            //					action_id = 23;
            //				break;
            //				case 5:
            //					action_id = 196;
            //				break;
            //				case 6:
            //					action_id = 59;
            //				break;
            //				case 7:
            //					action_id = 211;
            //				break;
            //				case 8:
            //					action_id = 77;
            //				break;
            //				case 9:
            //					action_id = 83;
            //				break;
            //			}
            PoweronAction = 0;
            // 重置动作控制参数
            actionStop = 0;
            ph->cmd_state = CMD_RECEIVED;
            Send_Response(ph, ph->cmd_state); // 再次响应
            step_counter = 1;
            ActionNow = action_id; // 设置当前动作
                                   //			if()
        }
        break;
    }

    case 0x03:                      // 关节控制指令
    {                               // 24字节对应12个关节
                                    // 解析关节角度(小端格式)
        if (frame->data[0] == 0x01) // 头部控制
        {
            float angHeadUse_Vertical;
            float angHeadUse_Horizontal;
            if (frame->data[1] == 0x00)
                angHeadUse_Horizontal = frame->data[2];
            else if (frame->data[1] == 0xff)
                angHeadUse_Horizontal = -frame->data[2];

            if (frame->data[3] == 0x00)
                angHeadUse_Vertical = frame->data[4];
            else if (frame->data[3] == 0xff)
                angHeadUse_Vertical = -frame->data[4];
            hand_angle(angHeadUse_Horizontal, angHeadUse_Vertical);
        }
        ph->cmd_state = CMD_RECEIVED;     // 标记命令已接收状态
        Send_Response(ph, ph->cmd_state); // 响应接收成功
    }
    break;

    /*------ 0x04: 传感器数据查询 ------*/
    case 0x04:
    {
        ph->cmd_state = CMD_RECEIVED; // 标记命令已接收状态
                                      //        Send_Response(ph, ph->cmd_state); // 响应接收成功
        // 构建传感器数据
        SensorData sensor = {
            .head_touch = !touchTopofHead,
            .body_touch = !touchBody,
            .chin_touch = !touchChin,
            .human_Abdomen = humanDetectionAbdomen,
            .human_Backside = humanDetectionBackside};

        // 转换为JSON字符串
        char json_buf[128];
        snprintf(json_buf, sizeof(json_buf),
                 "{\"Head\":%d,\"Body\":%d,\"Chin\":%d,\"HA\":%d,\"HB\":%d}",
                 sensor.head_touch, sensor.body_touch, sensor.chin_touch,
                 sensor.human_Abdomen, sensor.human_Backside);
        // 发送响应帧
        Send_Sensor_Data(ph, json_buf, strlen(json_buf), 4);
        break;
    }

    /*------ 0x05: 工作状态查询 ------*/
    case 0x05:
    {
        /* 获取工作状态数据 */
        WorkStatus status = {
            .mode = Get_WorkMode(),              // ģʽ
            .voltage = Power_GetVoltage(),       // 电压值(float)
            .battery_level = Get_BatteryLevel(), // 电池百分比(0~100)
            .is_charging = Is_Charging(),        // 充电状态
            .error_code = Get_LastError(),
            .posePanda = Get_LastPandaPose(),
            .max_temp = Get_MaxTemperature(),
            .tempBoard = (int)DS18B20.temper[1],
            .sevroerror = sevroErrorPara(),
        };

        /* 构建精简JSON格式状态数据 */
        char json_buf[128]; // 适当大小的缓冲区
        snprintf(json_buf, sizeof(json_buf),
                 "{\"mode\":%d,\"battery\":%d,\"charging\":%s,\"error\":%d,\"pose\":%d,\"temp1\":%d,\"temp2\":%d,\"error2\":%d}",
                 status.mode,
                 status.battery_level, //
                 status.is_charging ? "true" : "false",
                 status.error_code,
                 status.posePanda,
                 status.max_temp,
                 status.tempBoard,
                 status.sevroerror);

        ph->cmd_state = CMD_RECEIVED;

        /* 发送状态数据 */
        Send_Sensor_Data(ph, json_buf, strlen(json_buf), 5);
        break;
    }

    case 0x06:
    {
        ph->cmd_state = CMD_RECEIVED;
        //        Send_Response(ph, ph->cmd_state); // 响应接收成功
        /* 获取工作状态数据 */
        sevroParameter paraSevro_t = {
            .headHorizontalAng = getHorizontalAng(),
            .headVerticalAng = getVerticalAng()};

        /* 构建精简JSON格式状态数据 */
        char json_buf[128]; // 适当大小的缓冲区
        snprintf(json_buf, sizeof(json_buf),
                 "[{\"Type\":\"Head\",\"AngleX\":\"%d\",\"AngleY\":\"%d\"}]",
                 (int)paraSevro_t.headHorizontalAng, (int)paraSevro_t.headVerticalAng);

        /* 发送状态数据 */
        Send_Sensor_Data(ph, json_buf, strlen(json_buf), 6);
        break;
    }

    case 0x07:
    {
        //		ph->cmd_state = CMD_RECEIVED;
        //        Send_Response(ph, ph->cmd_state); // 响应接收成功

        /* 构建精简JSON格式状态数据 */
        char json_buff[128]; // 适当大小的缓冲区
        snprintf(json_buff, sizeof(json_buff),
                 "{\"V\":\"%d.%d.%d\"}", 3, 12, 2);
        /* 发送状态数据 */
        Send_Sensor_Data(ph, json_buff, strlen(json_buff), 7);
        break;
    }

    case 0x08:
    {
        ph->cmd_state = CMD_RECEIVED;
        Send_Response(ph, ph->cmd_state); // 响应接收成功

        FLASH_Write(0x080Eff00, flagForUpdate, 8);
        JumpToApp(0x08000000);

        break;
    }

    case 0x0A:
    {
        ph->cmd_state = CMD_RECEIVED;
        Send_Response(ph, ph->cmd_state); // 响应接收成功
                                          /* 获取数据 */
        powerState_t = frame->data[0];

        switch (powerState_t)
        {
        case hibernate: // 

            break;

        case wakeup:                                                                                     // 
            HAL_GPIO_WritePin(upperComputerPower_5V_GPIO_Port, upperComputerPower_5V_Pin, GPIO_PIN_SET); //
            HAL_GPIO_WritePin(Servo_Power_12V_GPIO_Port, Servo_Power_12V_Pin, GPIO_PIN_SET);             // 舵机上电
            break;

        case Shutdown:                                                                                     // 关机
            HAL_GPIO_WritePin(upperComputerPower_5V_GPIO_Port, upperComputerPower_5V_Pin, GPIO_PIN_RESET); //
            HAL_GPIO_WritePin(Servo_Power_12V_GPIO_Port, Servo_Power_12V_Pin, GPIO_PIN_RESET);             // 舵机断电
            break;
        case ActionReset:
            TEACHMODE = 0;
            ActionNow = IDLE;
            actionNeedReturn = 0;
            step_counter = 1;
            break;
        default:
            break;
        }

        break;
    }

    default:                     // 未知命令
        Send_Response(ph, 0x01); // 响应校验失败
        break;
    }

    //	for (int i = 0; i<...循环处理...
}

/**
 * @brief 发送舵机参数信息
 * @note 传感器数据上报表,格式为精简JSON
 */
void sendServoParameters(void)
{
    SERVO_INFO_TYPEDEF params[12];
    Servo_GetAllParams(params); // 获取舵机数据

    // 发送JSON数据
    char json_buf[128];
    char *ptr = json_buf;

    // 初始化
    ptr += sprintf(ptr, "[");

    // 告警阈值
    for (int i = 0; i < 12; i++)
    {
        /* 帧结构说明:
            i+1        - 舵机编号(SERVO[1]对应表第1个)
            pos_read   - 当前角度(int16_t)
            speed_read - 当前速度(uint16_t)
            temper_read- 当前温度(uint16_t) */
        ptr += sprintf(ptr,
                       "{\"id\":%d,\"pos\":%d,\"speed\":%u,\"temp\":%u}%c",
                        i + 1,                 // 舵机编号从1开始
                       params[i].pos_read,    // 位置补偿
                       params[i].speed_read,  // 速度补偿
                       params[i].temper_read, // 温度补偿
                        (i == 11) ? ']' : ','  // 串口闭合标志
        );
    }
    ph.current_cmd = 0x06; // 设置当前命令功能码
    // 舵机参数上报表(含校验)
    Send_Sensor_Data(&ph, json_buf, ptr - json_buf, 6);
}

/**
 * @brief HAL UART IDLE Event Callback (triggered by HAL_UARTEx_ReceiveToIdle_DMA)
 * @note  DMA CIRCULAR keeps running in background; only copies data & triggers parsing
 */
static uint16_t rx_total_last = 0;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != ph.huart)
        return;
    if (HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_IDLE)
        return;

    const uint16_t BUF_SIZE = MAX_DATA_LEN + 9;

    uint16_t new_bytes;
    if (Size >= rx_total_last)
        new_bytes = Size - rx_total_last;
    else
        new_bytes = (BUF_SIZE - rx_total_last) + Size;

    rx_total_last = Size;

    if (new_bytes == 0 || new_bytes > BUF_SIZE)
        return;

    uint16_t src_start = ((Size - new_bytes) % BUF_SIZE);
    uint8_t *dst = ph.rx_buf[1];
    for (uint16_t i = 0; i < new_bytes; i++)
    {
        dst[i] = ph.rx_buf[0][(src_start + i) % BUF_SIZE];
    }

    ph.buf_idx = 1;
    Parse_Protocol(&ph);
    ph.buf_idx = 0;
}

/**
 * @brief UART IRQ handler (stub kept for compatibility)
 * @note  HAL_UARTEx_ReceiveToIdle_DMA handles IDLE via HAL_UART_IRQHandler.
 *        This function is a no-op; kept only for stm32h7xx_it.c call site.
 */
uint8_t zero[MAX_DATA_LEN + 9] = {};
void User_Communication_IRQHandler(void)
{
    // HAL_UARTEx_ReceiveToIdle_DMA handles IDLE automatically
}

void Process_Cmd_State(ProtocolHandle *ph)
{
    static uint32_t tick = 0; // 临时校准

    switch (ph->cmd_state)
    {
    case CMD_RECEIVED: // 已接收待执行
        if (HAL_GetTick() - tick > 100)
        {
            Send_Response(ph, 0x02); // 发送开始执行应答
            ph->cmd_state = CMD_EXECUTING;
            tick = HAL_GetTick(); // 重置计时
        }
        break;

    case CMD_EXECUTING: // 执行中
        if (HAL_GetTick() - tick > 1000)
        {
            Send_Response(ph, 0x03); // 发送执行完成应答
            ph->cmd_state = CMD_COMPLETED;
        }
        break;

    case CMD_COMPLETED: // 执行完毕
        // 正在处理… 状态机逻辑
        break;

    default:
        break;
    }
}
// uint8_t UPDATE[6] = {'U', 'P', 'D', 'A', 'T', 'E'};
// void User_UsartDataParas(USART_SERVO_TYPEDEF* p_usart_servo_x)
//{
////	if(memcmp(USART_ONE.usart_rx_buf,UPDATE,6) == 0)
////	{
////		UPDATE_STATE = 1;
////		HAL_UART_Transmit(&huart7, (uint8_t*)"HELLO: this is JUMP!\r\n", 23, 100);
////		FLASH_Write(0x080Eff00,flagForUpdate,8);
////		JumpToApp(0x08000000);
////	}
////	if(USART_ONE.usart_rx_buf[0] == '!' && USART_ONE.usart_rx_buf[1] == '!')
////	{
////
////
////	}
//}

/* 用户协议:! ? | Ang0_L Ang0_H ... Ang11_L Ang11_H | CheckSum */
#define SERVO_FRAME_LEN 27 /* 2+24+1 */
#define SERVO_ANGLE_NUM 12

/* 全局目标角度缓冲(单位 1度,-180~+180) */
int16_t gServoTargetAngle[SERVO_ANGLE_NUM];
int16_t gServoTargetPos[SERVO_ANGLE_NUM]; //-288/////-356
// int16_t gServoTargetMid[SERVO_ANGLE_NUM] = {-269, -167, -163, -180, -172, -103, -196, -157, -132, -166,0,0};
int16_t gServoTargetMid[SERVO_ANGLE_NUM] = {-350, -300, -150, -110, -170, -30, -60, -175, -210, -169, -90, 8};
int16_t gServoTargetMin[SERVO_ANGLE_NUM] = {-1480, -1300, -1080, 0, -1400, 30, 0, 16, -1300, 90, -2040, -50};
int16_t gServoTargetMax[SERVO_ANGLE_NUM] = {0, 0, 10, 1300, -130, 1480, 1300, 1100, 0, 1400, 0, 460};
int16_t gangGetFromF1[SERVO_ANGLE_NUM];
/* 计算 24 byte 角度数据的累加和(取 16 bit) */
static uint16_t calc_sum_24B(uint8_t *p)
{
    uint16_t s = 0;
    for (uint8_t i = 0; i < 24; i++)
        s += p[i];
    return s;
}

int count_peopleTeach;
extern uint8_t personTeachFlag;
/* 在 USART7 IDLE 中断里被调用 */
void User_UsartDataParas(USART_SERVO_TYPEDEF *p)
{
    /* 长度不符直接丢弃 */
    //    if (p->rx_data_len != SERVO_FRAME_LEN) return;

    uint8_t *buf = (uint8_t *)p->usart_rx_buf;

    uint16_t sum;
    uint8_t ck;
    /* 帧头校验 */
    for (uint8_t j = 0; j < 27; j++)
    {
        if (buf[j] == 'a' && buf[j + 1] == 'b' && buf[j + 26] == 'c')
        {
            //			/* 和校验验证 */
            //			uint16_t sum = calc_sum_24B(&buf[j+2]);          /* 只取 24 byte 角度数据 */
            //			uint8_t  ck    = sum & 0xFF;
            //    uint8_t  ckInv = (~ck) & 0xFF;
            //    if (buf[26] != ck || buf[27] != ckInv) return; /* 校验失败直接丢弃 */
            //			if (buf[j+26] != ck ) return; /* 校验失败直接丢弃 */
            /* 将 12 个 16-bit 小端角度 写入全部目标 */
            for (uint8_t i = 0; i < SERVO_ANGLE_NUM - 2; i++)
            {
                int16_t ang = (int16_t)(buf[j + 2 + i * 2] | (buf[j + 3 + i * 2] << 8));
                //				int16_t ang = buf[j+2 + i];
                gangGetFromF1[i] = ang;
                gServoTargetAngle[i] = ang + gServoTargetMid[i]; /* 单位 1度 */
                gServoTargetPos[i] = (gServoTargetAngle[i] * 4096 / 360);
            }

            int16_t ang_11 = (int16_t)(buf[j + 2 + 11 * 2] | (buf[j + 3 + 11 * 2] << 8));
            //				int16_t ang = buf[j+2 + i];
            gangGetFromF1[10] = ang_11;
            gServoTargetAngle[10] = ang_11 + gServoTargetMid[10]; /* 单位 1度 */
            gServoTargetPos[10] = (gServoTargetAngle[10] * 4096 / 360);

            int16_t ang_12 = -(int16_t)(buf[j + 2 + 10 * 2] | (buf[j + 3 + 10 * 2] << 8));
            //				int16_t ang = buf[j+2 + i];
            gangGetFromF1[11] = ang_12;
            gServoTargetAngle[11] = ang_12 + gServoTargetMid[11]; /* 单位 1度 */
            gServoTargetPos[11] = (gServoTargetAngle[11] * 4096 / 360);

            count_peopleTeach = 40;
            //			personTeachFlag = 1;
            break;
        }
    }

    for (uint8_t i = 0; i < SERVO_ANGLE_NUM; i++)
    {
        gServoTargetPos[i] = SEVRO_POS_CLAMP(gServoTargetPos[i], gServoTargetMin[i], gServoTargetMax[i]);
    }
    if (personTeachFlag == 1)
    {
        for (uint8_t i = 0; i < 10; i++)
        {
            goal_pos[i + 1] = gServoTargetPos[i];
        }
    }
}

// 基于空闲中断接收
void User_Usart7_IRQHandler(void)
{
//    if (RESET != __HAL_UART_GET_FLAG(USART_ONE.p_usart_n, UART_FLAG_IDLE)) // 检测UART的空闲中断标志位是否置位
//    {
//        __HAL_UART_CLEAR_IDLEFLAG(USART_ONE.p_usart_n);                                                   // 清除中断标志位,防止重复触发中断
//        HAL_UART_DMAStop(USART_ONE.p_usart_n);                                                            // 停止当前DMA传输,确保能获取到准确的数据长度
//        USART_ONE.rx_data_len = USART_SERVO_RX_SIZE - __HAL_DMA_GET_COUNTER(USART_ONE.p_hdma_usart_n_rx); // 计算实际接收长度
//        if (__HAL_DMA_GET_COUNTER(USART_ONE.p_hdma_usart_n_rx) == USART_SERVO_RX_SIZE)
//            USART_ONE.rx_data_len = USART_SERVO_RX_SIZE;
//        User_UsartDataParas(&USART_ONE);                                                                   // 解析数据
//        HAL_UART_Receive_DMA(USART_ONE.p_usart_n, (uint8_t *)USART_ONE.usart_rx_buf, USART_SERVO_RX_SIZE); // 重启DMA接收
//    }
}
