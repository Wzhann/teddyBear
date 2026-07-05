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
ProtocolHandle ph;        // Э����ʵ��
uint8_t actionStop = 0;   // ����ͣ�¶���
uint8_t actioninIdle = 0; // ��������Ĭ�϶���
WorkStatus stateRobot;
ACTION_STATE ActionReceive;
IOfunctionState ioState;
PowerType_T powerState_t;
uint32_t flagForUpdate[8] = {0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa};

uint8_t actionINDEXForshow1[13] = {43, 47, 48, 49, 45, 53, 58, 55, 56, 73, 75, 167, 29};
uint8_t showModeFlag = 0;

void SoftwareReset(void)
{
    __set_FAULTMASK(1); // �ر������ж�
    NVIC_SystemReset(); // ����ϵͳ��λ
}

// IOģ��//
/*LED:0--����1--��*/
void ledSet(uint8_t mode)
{
    if (mode == 1)
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    else if (mode == 0)
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    else
        ;
}
/*buzzer:0--ͣ��1--��*/
void buzzerSet(uint8_t mode)
{
    if (mode == 1)
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
    else if (mode == 0)
        HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
    else
        ;
}
/*FAN:0--�أ�1--��*/
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
        // ���times���󣬿��ܵ���ÿ����˸ʱ����̣�Ӱ���Ӿ�Ч��
        // �����������times�����ߵ����߼�
        times = 10; // ���������˸����
    }

    uint32_t flash_duration = pdMS_TO_TICKS(1000 / times); // ÿ����˸���ܳ���ʱ�䣨tick��
    uint32_t half_flash = flash_duration / 2;              // Ϩ��͵�����ռһ��ʱ��

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
        // ���times���󣬿��ܵ���ÿ����˸ʱ����̣�Ӱ���Ӿ�Ч��
        // �����������times�����ߵ����߼�
        times = 10; // ���������˸����
        buzzerSet(BUZZER_ON);
    }
    else
    {
        uint32_t flash_duration = pdMS_TO_TICKS(1000 / times); // ÿ����˸���ܳ���ʱ�䣨tick��
        uint32_t half_flash = flash_duration / 2;              // Ϩ��͵�����ռһ��ʱ��

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
 * @brief ͨ��ģ���ʼ��
 * @note ����UART��DMA�����ÿ����ж�
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

    // //���������жϽ���
    // __HAL_UART_ENABLE_IT(USART_ONE.p_usart_n, UART_IT_IDLE);
    // HAL_UART_Receive_DMA(USART_ONE.p_usart_n, (uint8_t*)USART_ONE.usart_rx_buf, USART_SERVO_RX_SIZE);
}

void Single_Key_Record(uint8_t *key, uint8_t *last_key, uint8_t *key_downside, uint8_t *key_upside)
{
    // �½��ؼ��
    if ((*last_key == 1) && (*key == 0))
    {
        *key_downside = 1;
    }
    else
    {
        *key_downside = 0;
    }
    // �����ؼ��
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

uint8_t touchTopofHead_Downside, touchTopofHead_Upside, touchTopofHead, Last_touchTopofHead;                                 // ����ͷ��
uint8_t touchChin_Downside, touchChin_Upside, touchChin, Last_touchChin;                                                     // �����°�
uint8_t touchBody_Downside, touchBody_Upside, touchBody, Last_touchBody;                                                     // ��������
uint8_t humanDetectionAbdomen_Downside, humanDetectionAbdomen_Upside, humanDetectionAbdomen, Last_humanDetectionAbdomen;     // ������
uint8_t humanDetectionBackside_Downside, humanDetectionBackside_Upside, humanDetectionBackside, Last_humanDetectionBackside; // ������
uint8_t IOForCharging_Downside, IOForCharging_Upside, IOForCharging, Last_IOForCharging;                                     // �����
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
/*------------------------ �������� ------------------------*/

/**
 * @brief ����򵥺�У�飨16λ��
 * @param data ��У������ָ��
 * @param len У�����ݳ��ȣ��ֽ�����
 * @return ����õ���16λУ��ֵ
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

// ====================== ����ģʽ��غ��� ======================
/**
 * @brief ��ȡ��ǰ����ģʽ
 * @return ����ģʽö��ֵ
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
 * @brief ��ȡ��Դ��ѹ
 * @return ��ǰ��ѹֵ����λ�����أ�
 */
float Power_GetVoltage(void)
{
    // ��ʱ����0��ѹ
    //    return USER_ADC.bat_volt;
    return 0;
}

/**
 * @brief ��ȡ��ص����ٷֱ�
 * @return �����ٷֱȣ�0-100��
 */
uint8_t Get_BatteryLevel(void)
{
    // ��ʱ����0%����
    return USER_ADC.bat_power;
    //	return 0;
}

/**
 * @brief �����״̬
 * @return true:���ڳ�� false:δ���
 */
bool Is_Charging(void)
{
    // ��ʱ����δ���״̬
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
 * @brief ��ȡϵͳ����¶�
 * @return ����¶�ֵ����λ�����϶ȣ�
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
 * @brief ��ȡ��ǰ�������
 * @return ������루0��ʾ�޾��棩
 */
uint16_t Get_Warning(void)
{
    // ��ʱ�����޾���
    return 0;
}

extern uint8_t poseNow;
/**
 * @brief ��ȡ����������
 * @return ������루0��ʾ�޴���
 */
uint16_t Get_LastError(void)
{
    uint8_t lasterror;
    if (poseNow == 4)
        lasterror = 1;
    else
        lasterror = 0;

    // ��ʱ�����޴���
    return lasterror;
}

// ====================== ��������غ��� ======================
/**
 * @brief ��ȡ����������ֵ
 * @param touch_type �������ͣ�HEAD/BODY/CHIN��
 * @return ����״̬��0/1 �� ADCԭʼֵ��
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
    // ��ʱ�����޴����ź�
    //    (void)touch_type; // ����δʹ�ò�������
    return 0;
}

/**
 * @brief ��ȡIMU������
 * @return �����Ƕȣ���λ���ȣ�-180~180��
 */
float IMU_GetRoll(void)
{
    // ��ʱ����0�Ⱥ����
    return 0.0f;
}

/**
 * @brief ��ȡIMU������
 * @return �����Ƕȣ���λ���ȣ�-90~90��
 */
float IMU_GetPitch(void)
{
    // ��ʱ����0�ȸ�����
    return 0.0f;
}

/**
 * @brief ��ȡIMUƫ����
 * @return ƫ���Ƕȣ���λ���ȣ�0~360��
 */
float IMU_GetYaw(void)
{
    // ��ʱ����0��ƫ����
    return 0.0f;
}

/* ����/�������ת������ʾ�� */
/**
 * @brief �������ת�ַ���
 * @param code �������
 * @return �ɶ��ľ�������
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
 * @brief �������ת�ַ���
 * @param code �������
 * @return �ɶ��Ĵ�������
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
 * @brief ���ʹ�����������Ӧ֡
 * @param ph Э����ָ��
 * @param data Ҫ���͵�JSON�����ַ���
 * @param len JSON���ݳ���
 * @note ֡�ṹ��֡ͷ(2B) | ������(1B) | ���ݳ���(2B) | JSON����(NB) | У���(1B) | ֡β(2B)
 */
uint8_t Send_Sensor_Data(ProtocolHandle *ph, const char *data, uint16_t len, uint8_t code)
{
    // У�����ݳ��ȺϷ���
    if (len > MAX_DATA_LEN + 8)
    {
        return 0;
    }

    // ������������֡����̬����֡���ȣ�
    //    uint8_t tx_buf[2 + 2 + 5 + len]; // ͷ֡β֡4+������1+���ݳ���2+У��1
    uint16_t frame_len = 2 + 2 + 5 + len;

    // ֡ͷ��2�ֽڣ�
    tx_buf_[0] = FRAME_HEADER_DOWN >> 8;
    tx_buf_[1] = FRAME_HEADER_DOWN & 0xFF;

    // �����루1�ֽڣ�
    tx_buf_[2] = code;

    // ���ݳ��ȣ�С�˸�ʽ��2�ֽڣ�
    tx_buf_[3] = len & 0xFF;
    tx_buf_[4] = (len >> 8) & 0xFF;

    // ��������
    memcpy(&tx_buf_[5], data, len);

    // �����У�飨������+����+���ݣ�
    uint16_t checksum = Calculate_SumCheck(&tx_buf_[3], len);
    tx_buf_[5 + len] = checksum;

    // ֡β��2�ֽڣ�
    tx_buf_[6 + len] = FRAME_FOOTER_DOWN >> 8;
    tx_buf_[7 + len] = FRAME_FOOTER_DOWN & 0xFF;

    // DMA����
    HAL_UART_Transmit(ph->huart, tx_buf_, 8 + len, 100);

    return 1;
    //	memcpy(tx_buf_, zeroBuf, 8+len);
}

/*------------------------ ���������ȡ���� ------------------------*/
/**
 * @brief ��ȡ���ж������
 * @param params ��������ṹ������
 * @note SERVO[1]-SERVO[12] ��Ӧ params[0]-params[11]
 */
void Servo_GetAllParams(SERVO_INFO_TYPEDEF *params)
{
    // SERVO[1]-SERVO[12] ��Ӧ params[0]-params[11]
    for (int i = 0; i < 12; i++)
    {
        params[i] = SERVO[i + 1]; // ��������ƫ��
    }
}

/**
 * @brief ���ͱ�׼��Ӧ֡
 * @param ph Э����ָ��
 * @param result ��Ӧ����루0��ʾ�ɹ���1��ʾУ��ʧ�ܣ�2��ʾ��ʼִ�У�3��ʾִ����ϣ�
 * @note ֡�ṹ��BB BB | func | 01 00 | result | ��У�� | ++ ++
 */
uint8_t _tx_buf_[10] = {0x42, 0x42, 0, 1, 0, 0, 0, 0, 0x2B, 0x2B};
void Send_Response(ProtocolHandle *ph, uint8_t result)
{
    //    uint8_t tx_buf[10] = {
    //        FRAME_HEADER_DOWN >> 8, FRAME_HEADER_DOWN & 0xFF, // ֡ͷBB BB
    //        ph->current_cmd,                                  // ԭ���ش�������
    //        1, 0,                                             // ���ݳ���С�ˣ��̶�1�ֽڣ�
    //        result,                                           // �����
    //        0, 0,                                             // ��У��ռλ
    //        FRAME_FOOTER_DOWN >> 8, FRAME_FOOTER_DOWN & 0xFF  // ֡β++ ++
    //    };
    _tx_buf_[2] = ph->current_cmd;
    _tx_buf_[5] = result;
    // ����У��ͣ�������������У�飩
    uint16_t checksum = Calculate_SumCheck(&_tx_buf_[5], 1);
    _tx_buf_[6] = checksum & 0xFF; // У����ֽ���ǰ
    _tx_buf_[7] = checksum >> 8;   // ���ֽ��ں�

    HAL_UART_Transmit(ph->huart, _tx_buf_, sizeof(_tx_buf_), 50);
}

void sendStateActive(ProtocolHandle *ph, WorkStatus status)
{
    /* ��ȡ����״̬���� */
    status.mode = Get_WorkMode();              // ����ģʽ
    status.voltage = Power_GetVoltage();       // ��ѹֵ��float��
    status.battery_level = Get_BatteryLevel(); // �����ٷֱȣ�0~100��
    status.is_charging = Is_Charging();        // ���״̬
    status.error_code = Get_LastError();
    status.posePanda = Get_LastPandaPose();
    status.max_temp = Get_MaxTemperature();
    status.tempBoard = (int)DS18B20.temper[1];
    status.sevroerror = sevroErrorPara();

    /* ���ɾ����JSON��ʽ״̬���� */
    char json_buf[128]; // �ʵ���С�Ļ�����
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

    /* ����״̬���� */
    Send_Sensor_Data(ph, json_buf, strlen(json_buf), 5);
}

void sendSensorActive(ProtocolHandle *ph, uint8_t head, uint8_t body, uint8_t chin, uint8_t abdomen, uint8_t backside)
{
    // ����������
    SensorData sensor = {
        .head_touch = head,
        .body_touch = body,
        .chin_touch = chin,
        .human_Abdomen = abdomen,
        .human_Backside = backside};
    //            .roll = IMU_GetRoll(),
    //            .pitch = IMU_GetPitch(),
    //            .yaw = IMU_GetYaw()};

    // ת��ΪJSON�ַ���
    char json_buf[128];
    snprintf(json_buf, sizeof(json_buf),
             //                 "{\"touch\":[%u,%u,%u],\"pose\":[%.1f,%.1f,%.1f]}",
             "{\"Head\":%d,\"Body\":%d,\"Chin\":%d,\"HA\":%d,\"HB\":%d}",
             //		"{\"touch\":[%d,%d,%d]}",
             sensor.head_touch, sensor.body_touch, sensor.chin_touch,
             sensor.human_Abdomen, sensor.human_Backside);
    ph->cmd_state = CMD_RECEIVED; // �����������״̬
                                  //        Send_Response(ph, ph->cmd_state); // ��Ӧ���ճɹ�
    // ������Ӧ֡
    Send_Sensor_Data(ph, json_buf, strlen(json_buf), 4);
}

///*ӳ��id*/
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
 * @brief Э��������߼�
 * @param ph Э����ָ��
 * @note ִ��˳��֡�ṹУ�� -> ��У�� -> ������ַ�
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

    // ���У��ͼ�֡β����
    //    for (int i = 0; i < 4; i++)
    //        frame->data[frame->data_len + i] = 0;

    /* �����ṹУ�� */
    if (frame->header != FRAME_HEADER_UP || // ��֤��λ��֡ͷ
        frame->footer != FRAME_FOOTER_UP || // ��֤��λ��֡β
        frame->data_len > MAX_DATA_LEN)     // ���ݳ��ȺϷ��Լ��
    {
        return;
    }

    /* ��У����֤ */
    uint16_t calc_sum = Calculate_SumCheck(frame->data, frame->data_len);
    if (calc_sum != frame->checksum)
    {
        ph->cmd_state = CMD_CHECK_ERROR;  // ����У�����״̬
        Send_Response(ph, ph->cmd_state); // ��ӦУ��ʧ��
        return;
    }

    ph->current_cmd = frame->func; // ��¼��ǰ�������

    /* ������ַ����� */
    switch (frame->func)
    {
    case 0x01: // ����״̬��������
        if (frame->data_len == 2)
        {
            actionFromemotion = 1;
            // ����״̬��
            ph->cmd_state = CMD_RECEIVED;
            Send_Response(ph, ph->cmd_state); // ������Ӧ���ճɹ�
                                              // Ԥ��2�ֽ�����
            // ����״̬�ͳ̶�ֵ��ʾ����
            uint8_t state = frame->data[0];
            uint8_t level = frame->data[1];

            //					// �������������
            //						srand((unsigned)time(NULL));
            // ���������
            int random_number = (rand() / 10) % 10;

            // �����ж�
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

    case 0x02: // ���������
    {
        //		releaseSevroFlag = 1;
        actionFromemotion = 0;
        TEACHMODE = 0; // �����ѧģʽ��־�����У�
        actionSwitchTime = ACTIONTIMESTEP;
        actionNeedReturn = 0;
        flag_sendExecuting = 0;
        // �״���Ӧ�����ճɹ���
        //		if(debugUse >1)
        ph->cmd_state = CMD_RECEIVED;
        Send_Response(ph, ph->cmd_state);

        // ����������ţ���Χ0-255��
        uint16_t action_id = *(uint16_t *)&frame->data[0];

        // ��֤���������Ч�ԣ�ʾ������Ч��Χ0-121��
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
            Send_Response(ph, 0x03); // ����ִ�������Ӧ
        }
        else if (action_id == 254)
        {
            PoweronAction = 0;
            actionStop = 0;
            ph->cmd_state = CMD_RECEIVED;
            Send_Response(ph, ph->cmd_state); // �ٴ���Ӧ
            ActionNow = IDLE;
        }
        else if (action_id == 1)
        {
            PoweronAction = 0;
            // ���¶������Ʋ���
            actionStop = 0;
            ph->cmd_state = CMD_RECEIVED;
            Send_Response(ph, ph->cmd_state); // �ٴ���Ӧ
            step_counter = 1;
            ActionNow = action_id; // ���õ�ǰ����
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
            // ���������
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
            // ���¶������Ʋ���
            actionStop = 0;
            ph->cmd_state = CMD_RECEIVED;
            Send_Response(ph, ph->cmd_state); // �ٴ���Ӧ
            step_counter = 1;
            ActionNow = action_id; // ���õ�ǰ����
                                   //			if()
        }
        break;
    }

    case 0x03:                      // �ؽڿ�������
    {                               // 24�ֽڶ�Ӧ12���ؽ�
                                    // �����ؽڽǶȣ�С�˸�ʽ��
        if (frame->data[0] == 0x01) // ͷ������
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
        ph->cmd_state = CMD_RECEIVED;     // �����������״̬
        Send_Response(ph, ph->cmd_state); // ��Ӧ���ճɹ�
    }
    break;

    /*------ 0x04: ��������ѯ ------*/
    case 0x04:
    {
        ph->cmd_state = CMD_RECEIVED; // �����������״̬
                                      //        Send_Response(ph, ph->cmd_state); // ��Ӧ���ճɹ�
        // ����������
        SensorData sensor = {
            .head_touch = !touchTopofHead,
            .body_touch = !touchBody,
            .chin_touch = !touchChin,
            .human_Abdomen = humanDetectionAbdomen,
            .human_Backside = humanDetectionBackside};

        // ת��ΪJSON�ַ���
        char json_buf[128];
        snprintf(json_buf, sizeof(json_buf),
                 "{\"Head\":%d,\"Body\":%d,\"Chin\":%d,\"HA\":%d,\"HB\":%d}",
                 sensor.head_touch, sensor.body_touch, sensor.chin_touch,
                 sensor.human_Abdomen, sensor.human_Backside);
        // ������Ӧ֡
        Send_Sensor_Data(ph, json_buf, strlen(json_buf), 4);
        break;
    }

    /*------ 0x05: ����״̬��ѯ ------*/
    case 0x05:
    {
        /* ��ȡ����״̬���� */
        WorkStatus status = {
            .mode = Get_WorkMode(),              // ����ģʽ
            .voltage = Power_GetVoltage(),       // ��ѹֵ��float��
            .battery_level = Get_BatteryLevel(), // �����ٷֱȣ�0~100��
            .is_charging = Is_Charging(),        // ���״̬
            .error_code = Get_LastError(),
            .posePanda = Get_LastPandaPose(),
            .max_temp = Get_MaxTemperature(),
            .tempBoard = (int)DS18B20.temper[1],
            .sevroerror = sevroErrorPara(),
        };

        /* ���ɾ����JSON��ʽ״̬���� */
        char json_buf[128]; // �ʵ���С�Ļ�����
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

        /* ����״̬���� */
        Send_Sensor_Data(ph, json_buf, strlen(json_buf), 5);
        break;
    }

    case 0x06:
    {
        ph->cmd_state = CMD_RECEIVED;
        //        Send_Response(ph, ph->cmd_state); // ��Ӧ���ճɹ�
        /* ��ȡ����״̬���� */
        sevroParameter paraSevro_t = {
            .headHorizontalAng = getHorizontalAng(),
            .headVerticalAng = getVerticalAng()};

        /* ���ɾ����JSON��ʽ״̬���� */
        char json_buf[128]; // �ʵ���С�Ļ�����
        snprintf(json_buf, sizeof(json_buf),
                 "[{\"Type\":\"Head\",\"AngleX\":\"%d\",\"AngleY\":\"%d\"}]",
                 (int)paraSevro_t.headHorizontalAng, (int)paraSevro_t.headVerticalAng);

        /* ����״̬���� */
        Send_Sensor_Data(ph, json_buf, strlen(json_buf), 6);
        break;
    }

    case 0x07:
    {
        //		ph->cmd_state = CMD_RECEIVED;
        //        Send_Response(ph, ph->cmd_state); // ��Ӧ���ճɹ�

        /* ���ɾ����JSON��ʽ״̬���� */
        char json_buff[128]; // �ʵ���С�Ļ�����
        snprintf(json_buff, sizeof(json_buff),
                 "{\"V\":\"%d.%d.%d\"}", 3, 12, 2);
        /* ����״̬���� */
        Send_Sensor_Data(ph, json_buff, strlen(json_buff), 7);
        break;
    }

    case 0x08:
    {
        ph->cmd_state = CMD_RECEIVED;
        Send_Response(ph, ph->cmd_state); // ��Ӧ���ճɹ�

        FLASH_Write(0x080Eff00, flagForUpdate, 8);
        JumpToApp(0x08000000);

        break;
    }

    case 0x0A:
    {
        ph->cmd_state = CMD_RECEIVED;
        Send_Response(ph, ph->cmd_state); // ��Ӧ���ճɹ�
                                          /* ��ȡ���� */
        powerState_t = frame->data[0];

        switch (powerState_t)
        {
        case hibernate: // ����

            break;

        case wakeup:                                                                                     // ����
            HAL_GPIO_WritePin(upperComputerPower_5V_GPIO_Port, upperComputerPower_5V_Pin, GPIO_PIN_SET); //
            HAL_GPIO_WritePin(Servo_Power_12V_GPIO_Port, Servo_Power_12V_Pin, GPIO_PIN_SET);             // �������
            break;

        case Shutdown:                                                                                     // �ػ�
            HAL_GPIO_WritePin(upperComputerPower_5V_GPIO_Port, upperComputerPower_5V_Pin, GPIO_PIN_RESET); //
            HAL_GPIO_WritePin(Servo_Power_12V_GPIO_Port, Servo_Power_12V_Pin, GPIO_PIN_RESET);             // �������
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

    default:                     // δ֪������
        Send_Response(ph, 0x01); // ��ӦУ��ʧ��
        break;
    }

    //	for (int i = 0; i��������
}

/**
 * @brief ���Ͷ��������Ϣ
 * @note �����ϱ����ж��������JSON�����ʽ
 */
void sendServoParameters(void)
{
    SERVO_INFO_TYPEDEF params[12];
    Servo_GetAllParams(params); // ��ȡ�������

    // ����JSON����
    char json_buf[128];
    char *ptr = json_buf;

    // ��ʼ���
    ptr += sprintf(ptr, "[");

    // �������ж��
    for (int i = 0; i < 12; i++)
    {
        /* ����˵����
           i+1        - ���������ţ�SERVO[1]��Ӧ���1��
           pos_read   - ���λ�ã�int16_t��
           speed_read - ����ٶȣ�uint16_t��
           temper_read- ����¶ȣ�uint16_t�� */
        ptr += sprintf(ptr,
                       "{\"id\":%d,\"pos\":%d,\"speed\":%u,\"temp\":%u}%c",
                       i + 1,                 // ���������Ŵ�1��ʼ
                       params[i].pos_read,    // λ�ò���
                       params[i].speed_read,  // �ٶȲ���
                       params[i].temper_read, // �¶Ȳ���
                       (i == 11) ? ']' : ','  // ����պϱ��
        );
    }
    ph.current_cmd = 0x06; // ���õ�ǰ�������
    // ���������ϱ���������У�飩
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
    static uint32_t tick = 0; // ��ʱ��׼

    switch (ph->cmd_state)
    {
    case CMD_RECEIVED: // �ѽ��մ�ִ��
        if (HAL_GetTick() - tick > 100)
        {
            Send_Response(ph, 0x02); // ���Ϳ�ʼִ����Ӧ
            ph->cmd_state = CMD_EXECUTING;
            tick = HAL_GetTick(); // ���ü�ʱ
        }
        break;

    case CMD_EXECUTING: // ִ����
        if (HAL_GetTick() - tick > 1000)
        {
            Send_Response(ph, 0x03); // ����ִ�������Ӧ
            ph->cmd_state = CMD_COMPLETED;
        }
        break;

    case CMD_COMPLETED: // ִ�����
        // ���ڴ�����״̬�����߼�
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

/* �û�Э�飺! ? | Ang0_L Ang0_H �� Ang11_L Ang11_H | CheckSum */
#define SERVO_FRAME_LEN 27 /* 2+24+1 */
#define SERVO_ANGLE_NUM 12

/* ȫ��Ŀ��ǶȻ��棨��λ 1�㣬-180~+180�� */
int16_t gServoTargetAngle[SERVO_ANGLE_NUM];
int16_t gServoTargetPos[SERVO_ANGLE_NUM]; //-288/////-356
// int16_t gServoTargetMid[SERVO_ANGLE_NUM] = {-269, -167, -163, -180, -172, -103, -196, -157, -132, -166,0,0};
int16_t gServoTargetMid[SERVO_ANGLE_NUM] = {-350, -300, -150, -110, -170, -30, -60, -175, -210, -169, -90, 8};
int16_t gServoTargetMin[SERVO_ANGLE_NUM] = {-1480, -1300, -1080, 0, -1400, 30, 0, 16, -1300, 90, -2040, -50};
int16_t gServoTargetMax[SERVO_ANGLE_NUM] = {0, 0, 10, 1300, -130, 1480, 1300, 1100, 0, 1400, 0, 460};
int16_t gangGetFromF1[SERVO_ANGLE_NUM];
/* ���� 24 byte �Ƕ����ݵ��ۼӺͣ��� 16 bit�� */
static uint16_t calc_sum_24B(uint8_t *p)
{
    uint16_t s = 0;
    for (uint8_t i = 0; i < 24; i++)
        s += p[i];
    return s;
}

int count_peopleTeach;
extern uint8_t personTeachFlag;
/* �� USART7 IDLE �ж��ﱻ���� */
void User_UsartDataParas(USART_SERVO_TYPEDEF *p)
{
    /* ���Ȳ���ֱ�Ӷ��� */
    //    if (p->rx_data_len != SERVO_FRAME_LEN) return;

    uint8_t *buf = (uint8_t *)p->usart_rx_buf;

    uint16_t sum;
    uint8_t ck;
    /* ֡ͷ��� */
    for (uint8_t j = 0; j < 27; j++)
    {
        if (buf[j] == 'a' && buf[j + 1] == 'b' && buf[j + 26] == 'c')
        {
            //			/* У�����֤ */
            //			uint16_t sum = calc_sum_24B(&buf[j+2]);          /* ֻ�� 24 byte �Ƕ���� */
            //			uint8_t  ck    = sum & 0xFF;
            //    uint8_t  ckInv = (~ck) & 0xFF;
            //    if (buf[26] != ck || buf[27] != ckInv) return; /* У��ʧ��ֱ�Ӷ��� */
            //			if (buf[j+26] != ck ) return; /* У��ʧ��ֱ�Ӷ��� */
            /* ���� 12 �� 16-bit С�˽Ƕ� �� д��ȫ��Ŀ�� */
            for (uint8_t i = 0; i < SERVO_ANGLE_NUM - 2; i++)
            {
                int16_t ang = (int16_t)(buf[j + 2 + i * 2] | (buf[j + 3 + i * 2] << 8));
                //				int16_t ang = buf[j+2 + i];
                gangGetFromF1[i] = ang;
                gServoTargetAngle[i] = ang + gServoTargetMid[i]; /* ��λ 1�� */
                gServoTargetPos[i] = (gServoTargetAngle[i] * 4096 / 360);
            }

            int16_t ang_11 = (int16_t)(buf[j + 2 + 11 * 2] | (buf[j + 3 + 11 * 2] << 8));
            //				int16_t ang = buf[j+2 + i];
            gangGetFromF1[10] = ang_11;
            gServoTargetAngle[10] = ang_11 + gServoTargetMid[10]; /* ��λ 1�� */
            gServoTargetPos[10] = (gServoTargetAngle[10] * 4096 / 360);

            int16_t ang_12 = -(int16_t)(buf[j + 2 + 10 * 2] | (buf[j + 3 + 10 * 2] << 8));
            //				int16_t ang = buf[j+2 + i];
            gangGetFromF1[11] = ang_12;
            gServoTargetAngle[11] = ang_12 + gServoTargetMid[11]; /* ��λ 1�� */
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

// ���ڿ����жϽ���
void User_Usart7_IRQHandler(void)
{
//    if (RESET != __HAL_UART_GET_FLAG(USART_ONE.p_usart_n, UART_FLAG_IDLE)) // ���UART�Ŀ����жϱ�־λ�Ƿ���λ
//    {
//        __HAL_UART_CLEAR_IDLEFLAG(USART_ONE.p_usart_n);                                                   // ����жϱ�־λ����ֹ�ظ������ж�
//        HAL_UART_DMAStop(USART_ONE.p_usart_n);                                                            // ��ֹ��ǰDMA���䣬ȷ��������������������ݳ��ȣ���׼ȷ��
//        USART_ONE.rx_data_len = USART_SERVO_RX_SIZE - __HAL_DMA_GET_COUNTER(USART_ONE.p_hdma_usart_n_rx); // ����ʵ�ʽ��ճ���
//        if (__HAL_DMA_GET_COUNTER(USART_ONE.p_hdma_usart_n_rx) == USART_SERVO_RX_SIZE)
//            USART_ONE.rx_data_len = USART_SERVO_RX_SIZE;
//        User_UsartDataParas(&USART_ONE);                                                                   // ��������
//        HAL_UART_Receive_DMA(USART_ONE.p_usart_n, (uint8_t *)USART_ONE.usart_rx_buf, USART_SERVO_RX_SIZE); // ����DMA����
//    }
}
