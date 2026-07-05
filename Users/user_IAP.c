#include "user_IAP.h"
#include "string.h"
#include "user_flash.h"
#include "usart.h"
#define SRAM_START  0x20000000
#define SRAM_END    0x2004FFFF
#define AXI_SRAM_START 0x24000000
#define AXI_SRAM_END   0x2407FFFF
AppEntry AppStart;
uint32_t XXX = 0;
//uint32_t temp[APP_BUF_LEN];
void JumpToApp(uint32_t App_address) {
    
	//XXX = ((*(__IO uint32_t*)App_address) & 0x2FFE0000);
	XXX = (*(__IO uint32_t*)App_address);
	if ((XXX >= SRAM_START && XXX <= SRAM_END) || (XXX >= AXI_SRAM_START && XXX <= AXI_SRAM_END)) //检查栈顶地址是否合法，STM32 型号的 SRAM 地址范围是0x2000000--0x3FFF
	{
		//AppStart即为指向复位函数地址的函数指针
		AppStart = (AppEntry)(*(__IO uint32_t*)(App_address + 4));//用户代码区第二个位置为用户程序的复位函数地址（用户程序入口）
		__disable_irq(); // 禁用全局中断
		__set_FAULTMASK(1); // 关闭所有中断
		HAL_RCC_DeInit();
		HAL_DeInit();
		SysTick->CTRL = 0;             /* 关 SysTick           */
		SysTick->LOAD = 0;
		SysTick->VAL  = 0;
		for (int i = 0; i < 8; i++) {
		  NVIC->ICER[i] = 0xFFFFFFFF;  // 禁用所有中断
		  NVIC->ICPR[i] = 0xFFFFFFFF;  // 清除所有挂起的中断
		}
		NVIC_SystemReset();            /* 触发系统复位         */
//		SCB->VTOR = App_address & 0xFFFFFF80;
		//HAL_UART_DeInit(&huart7);
		__set_MSP(*(__IO uint32_t*)App_address);//切换主栈指针到用户代码区的栈顶地址
		AppStart();
	}
}

uint32_t IAPbuf[512]; //按2KByte合并接收到的数据，然后写入flash
uint32_t TOTAL_addr = 0;
void IAP_write_App_fromuart(uint32_t addr,uint8_t *buf,uint32_t len)
{
	uint32_t t;
	uint32_t i = 0;
	uint32_t temp;
	uint8_t *tempbuf = buf;
	//len是传入的字节数，处理后默认是32的整数倍
	for(t=0;t<len;t+=4)
	{
		//将4个8位数据合并为32位数据
		temp = (uint32_t)(tempbuf[3]<<24);
		temp += (uint32_t)(tempbuf[2]<<16);
		temp += (uint32_t)(tempbuf[1]<<8);
		temp += (uint32_t)tempbuf[0];
		tempbuf += 4;
		
		IAPbuf[i++] = temp; //将合并完成的16位数据储存在数组中
			
		//1024字节为256个32bit
		if(i == 256) //合并的数据填满iapbuf缓冲区后，开始写入falsh
		{
			i=0;
			FLASH_Write(addr + TOTAL_addr,IAPbuf,256);
			TOTAL_addr += 1024;//每次处理1024字节
		}
		
	}
	
	if(i)FLASH_Write(addr + TOTAL_addr,IAPbuf,i); //将最后一些内容写入flash
	TOTAL_addr += i*4;

}

/**
 * @bieaf 读若干个数据
 *
 * @param addr       读数据的地址
 * @param buff       读出数据的数组指针
 * @param word_size  长度
 * @return 
 */
static void ReadFlash(uint32_t addr, uint32_t * buff, uint16_t word_size)
{
	uint32_t redeBuff;
	for(int i =0; i < word_size; i++)
	{
		redeBuff = *(__IO uint32_t*)(addr);
	}
	if(redeBuff == 0xffffffff) *buff = 0;
	else *buff = 1;
	return;
}

/* 读取启动模式 */
unsigned int Read_Start_Mode(void)
{
	unsigned int mode = 0;
	ReadFlash(FLASH_APP2_ADDR, &mode, 1);
	return mode;
}

void MoveCode(uint32_t src_addr, uint32_t des_addr, uint32_t byte_size)
{
	/*1.擦除目的地址*/
//	UserFLASH_EraseSector(0x08020000);
//	UserFLASH_EraseSector(0x08040000);
//	UserFLASH_EraseSector(0x08060000);
//	UserFLASH_EraseSector(0x08080000);
//	UserFLASH_EraseSector(0x080A0000);
//	UserFLASH_EraseSector(0x080C0000);
//	UserFLASH_EraseSector(0x080E0000);
//	for(uint8_t number = 1;number<=7;number++) 
//	{
//		UserFLASH_EraseSector(number*SECTOR_SIZE+FLASH_BASE);//整页擦除
//		HAL_Delay(1);
//	}

//	/*2.开始拷贝*/	
//	unsigned int temp[APP_BUF_LEN];
//	FLASH_Read(FLASH_APP2_ADDR,temp,APP_BUF_LEN);
//	FLASH_Write(FLASH_APP1_ADDR,temp,APP_BUF_LEN);
//	
//	/*3.擦除源地址*/
//	for(uint8_t j = 8;j<=14;j++) 
//	{
//	UserFLASH_EraseSector(j*SECTOR_SIZE+FLASH_BASE);//整页擦除
//		HAL_Delay(1);
//	}
//	
}


void Start_BootLoader(void)
{
	switch(Read_Start_Mode())									///< 读取是否启动应用程序 */
	{
		case Startup_Normol:										///< 正常启动 */
		{
			break;
		}
		case Startup_Update:										///< 升级再启动 */
		{
			MoveCode(FLASH_APP2_ADDR, FLASH_APP1_ADDR, Application_Size);
			break;
		}
		default:				
		{
			return;			
		}
	}
	/*跳转到应用程序 */
	UPDATE_STATE = 0;
	HAL_Delay(10);
	JumpToApp(FLASH_APP1_ADDR);
}
