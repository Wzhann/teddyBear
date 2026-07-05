/**
 ****************************************************************************************************
 * @file        ds18b20.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-27
 * @brief       DS18B20数字温度传感器 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 探索者 F407开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211027
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "DS18B20.h"
#include "tim.h"



#define _USER_HTIM_1US		htim7
#define _USER_TIM_1US			TIM7

 
USER_DS18B20_TYPE DS18B20 = {0}; 
 

/* TIM7 init function */
void TimerDelay_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */
	TIM_HandleTypeDef htim7;
  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 240-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}


/**
us级延时，该延时函数将阻塞程序执行，非必要不使用
用于IO口模拟IIC通信使用（stm32f1硬件IIC尚存在问题）
**/
void TimerDelayUs(uint16_t us) 
{
	_USER_TIM_1US->CNT = 0;
	HAL_TIM_Base_Start(&_USER_HTIM_1US); 	
	while(_USER_TIM_1US->CNT < us);
	HAL_TIM_Base_Stop(&_USER_HTIM_1US); 
}
/**
ms级延时，该延时函数将阻塞程序执行，非必要不使用
**/
void User_DelayMs(uint16_t ms)
{
	while(ms--)
		TimerDelayUs(1000);
}
   
//void TimerDelayUs(uint16_t time) 
//{
//  time *= 10;
//	while(time--);
//}
/**
 * @brief       复位DS18B20
 * @param       data : 要写入的数据
 * @retval      无
 */
static void ds18b20_reset(void)
{
	DS18B20_DQ_OUT_0;  /* 拉低DQ, 复位 */
	TimerDelayUs(750);      /* 拉低750us */
	DS18B20_DQ_OUT_1;  /* DQ=1, 释放复位 */
	TimerDelayUs(15);       /* 延迟15us */
}

/**
 * @brief       等待DS18B20的回应
 * @param       无
 * @retval      0, DS18B20正常
 *              1, DS18B20异常/不存在
 */
uint8_t ds18b20_answer_check(void)
{
	uint8_t retry = 0;
	uint8_t rval = 0;

	while (DS18B20_DQ_IN && (retry < 200))        /* 等待DQ变低, 等待200us */
	{
		retry++;
		TimerDelayUs(1);
	}
	if (retry >= 200)
	{
		rval = 1;
	}
	else
	{
		retry = 0;

		while ((!DS18B20_DQ_IN) && (retry < 240))   /* 等待DQ变高, 等待240us */
		{
			retry++;
			TimerDelayUs(1);
		}

		if (retry >= 240) rval = 1;
	}
	return rval;
}

/**
 * @brief       从DS18B20读取一个位
 * @param       无
 * @retval      读取到的位值: 0 / 1
 */
static uint8_t ds18b20_read_bit(void)
{
	uint8_t data = 0;
	
	DS18B20_DQ_OUT_0;
	TimerDelayUs(2);
	DS18B20_DQ_OUT_1;
	TimerDelayUs(12);

	if (DS18B20_DQ_IN)
	{
		data = 1;
	}
	TimerDelayUs(60);
	return data;
}

/**
 * @brief       从DS18B20读取一个字节
 * @param       无
 * @retval      读到的数据
 */
static uint8_t ds18b20_read_byte(void)
{
	uint8_t i, b, data = 0;

	for (i = 0; i < 8; i++)
	{
		b = ds18b20_read_bit(); /* DS18B20先输出低位数据, 高位数据后输出 */	
		data |= b << i;         /* 填充data的每一位 */ 
	}
	return data;
}

// 写1位到DS18B20  

void DS18B20_Write_Bit(uint8_t data)  
{  
  if (data & 0x01)
	{
		DS18B20_DQ_OUT_0;  /*  Write 1 */
		TimerDelayUs(2);
		DS18B20_DQ_OUT_1;
		TimerDelayUs(60);
	}
	else
	{
		DS18B20_DQ_OUT_0;  /*  Write 0 */
		TimerDelayUs(60);
		DS18B20_DQ_OUT_1;
		TimerDelayUs(2);
	}
}  

/**
 * @brief       写一个字节到DS18B20
 * @param       data: 要写入的字节
 * @retval      无
 */
static void ds18b20_write_byte(uint8_t data)
{
	uint8_t j;

	for (j = 1; j <= 8; j++)
	{
		if (data & 0x01)
		{
				DS18B20_DQ_OUT_0;  /*  Write 1 */
				TimerDelayUs(2);
				DS18B20_DQ_OUT_1;
				TimerDelayUs(60);
		}
		else
		{
				DS18B20_DQ_OUT_0;  /*  Write 0 */
				TimerDelayUs(60);
				DS18B20_DQ_OUT_1;
				TimerDelayUs(2);
		}
		data >>= 1;             /* 右移,获取高一位数据 */
	}
}

/**
 * @brief       初始化DS18B20的IO口 DQ 同时检测DS18B20的存在
 * @param       无
 * @retval      0, 正常
 *              1, 不存在/不正常
 */
uint8_t ds18b20_init(void)
{
	TimerDelay_Init();
	
	GPIO_InitTypeDef gpio_init_struct;
	
	DS18B20_DQ_GPIO_CLK_ENABLE();   /* 开启DQ引脚时钟 */

	gpio_init_struct.Pin = DS18B20_DQ_GPIO_PIN;
	gpio_init_struct.Mode = GPIO_MODE_OUTPUT_OD;            /* 开漏输出 */
	gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
	HAL_GPIO_Init(DS18B20_DQ_GPIO_PORT, &gpio_init_struct); /* 初始化DS18B20_DQ引脚 */
	/* DS18B20_DQ引脚模式设置,开漏输出,上拉, 这样就不用再设置IO方向了, 开漏输出的时候(=1), 也可以读取外部信号的高低电平 */

	ds18b20_reset();
	return ds18b20_answer_check();
}


/**

 */
static void ds18b20_start(void)
{
	ds18b20_reset();
	ds18b20_answer_check();
	ds18b20_write_byte(0xcc);   /*  skip rom */
	ds18b20_write_byte(0x44);   /*  convert */
	ds18b20_reset();
	ds18b20_answer_check();
}

/**
总线只连接一个传感器时使用
 */
void ds18b20_get_temperature_single(void)
{
	uint8_t flag = 1;           /* 默认温度为正数 */
	uint8_t data_l,data_h;
	int16_t temp;
	
	ds18b20_start();            /*  ds1820 start convert */

	ds18b20_write_byte(0xcc);   /*  skip rom */
	ds18b20_write_byte(0xbe);   /*  convert */

	data_l = ds18b20_read_byte();   /*  LSB */
	data_h = ds18b20_read_byte();   /*  MSB */
	
	temp = ((uint16_t)data_h << 8) | data_l;

	/* 转换成实际温度 */
	if (temp < 0)
	{/* 将温度转换成负温度，这里的+1参考前面的说明 */
			DS18B20.temper[0] = (~temp + 1) * 0.0625f;
	}
	else
	{
			DS18B20.temper[0] = temp * 0.0625f;
	}
}

/**
总线只连接多个传感器时使用
 */
void ds18b20_get_temperature_multiple(void)
{
  static uint8_t sener_i = 0;  
	int16_t temp;
	uint8_t j;
	uint8_t data_l,data_h;
	
	ds18b20_start();

	//匹配ID
	ds18b20_write_byte(0x55);
	for (j = 0; j < 8; j++)
	{
		ds18b20_write_byte(DS18B20.id[sener_i][j]);
	}
	ds18b20_write_byte(0xbe);// convert
	
	data_l=ds18b20_read_byte(); // LSB
	data_h=ds18b20_read_byte(); // MSB  
	
	temp = ((uint16_t)data_h << 8) | data_l;
	
	/* 转换成实际温度 */
	if (temp < 0)
	{/* 将温度转换成负温度，这里的+1参考前面的说明 */
			DS18B20.temper[sener_i] = (~temp + 1) * 0.0625f;
	}
	else
	{
			DS18B20.temper[sener_i] = temp * 0.0625f;
	}
	
	sener_i++;
	if(sener_i == _DS18_NUM)
	{
		sener_i = 0;
	}
} 

void ds18b20_search_ID(void)
{
	uint8_t rom[64];
	uint8_t i = 0,j = 0,R1 = 0,R2 = 0,a = 0,b = 0,c = 0,d = 0;
	uint8_t _00wbit[2] = {0}; //初始化00写位组全部为填充位2

	for(i=0;i<2;i++)
	{
		_00wbit[i] = 2;
	}
	
	for (i = 0, c = 0; i < _DS18_NUM; i++)
	{
		ds18b20_reset(); //复位所有从机
		TimerDelayUs(420); 
		ds18b20_write_byte(0xf0); //主机发布搜索命令

		for (j = 0; j < 64; j++)
		{
			R1 = ds18b20_read_bit(); //读一位
			R2 = ds18b20_read_bit(); //读该位补码
			if (R1 == 0 && R2 == 1) //未出现数据冲突，主机写0
			{
				rom[j] = 0;
				DS18B20_Write_Bit(0);
			}
			else if (R1 == 1 && R2 == 0) //未出现数据冲突，主机写1
			{
				rom[j] = 1;
				DS18B20_Write_Bit(1);
			}
			else if(R1 == 0 && R2 == 0)
			{
				if (_00wbit[c] == 2) //出现新00写位
				{
					DS18B20_Write_Bit(0);
					rom[j] = 0;
					_00wbit[c] = 0; //新00写位赋值为0
					c += 1;
				}
				else if (_00wbit[c] == 1)//00写位组中00写位为1，主机写1
				{
					DS18B20_Write_Bit(1);
					rom[j] = 1;
					c += 1;
				}
				else if (_00wbit[c] == 0) //00写位组中00写位为0，主机写0
				{
					DS18B20_Write_Bit(0);
					rom[j] = 0;
					c += 1;
				}
			}
		}
		
		for (j = 0; j < 64; j += 8) //将64位ROM编码整理成8字节存入RomID[n][8]中
		{
			for (d = 0; d < 8; d++)
			{
				if (rom[j + d] & 0x01)
				{
					DS18B20.id[i][j/8] >>= 1;
					DS18B20.id[i][j/8] |= 0x80;
				}
				else DS18B20.id[i][j/8] >>= 1;
			}
		}

		for (a = 0, c = 1; c >= 0; c--) //更新00写位数组
		{
			if (_00wbit[c] == 2) //跳过00写位组中的填充位
				continue;
			if (_00wbit[c] == 0 && a == 0)//更改最高00写位并跳出
			{
				_00wbit[c] = 1;
				break;
			}
			else if (_00wbit[c] == 1)//最高00写位为1
			{
				if (c != 0) //为1的00写位不为00写位组的最低位
				{
					a += 1;//记录不为00写位组的最低位且为1的连续00写位个数
					continue;
				}
				else
				{
					b = 1; //00写位组全部为1，搜索结束置标志位
					break;
				}
			}
			else if (_00wbit[c] == 0 && a != 0)//连续为1的00写位后第一个为0的00写位
			{
				_00wbit[c] = 1;//赋次高00写位为1
				for (; a > 0; a--)//连续弃去为1的最高00写位
				{
					c += 1;
					_00wbit[c] = 2;
				}
				break;
			}
		}
		if (b == 1) break; //搜索结束标志位为1跳出
	}
	DS18B20.online_num = i + 1;//返回总线上器件个数
}






