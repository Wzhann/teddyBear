/*
	STM32F103C8T6 的 Flash大小为 64KB，
	地址范围：0x08000000 -- 0x08010000-1，
	单个扇区大小：1KB=0x400，
	最后一个扇区起始地址：0x0800FC00
*/
#include "user_flash.h"
#include "string.h"
#include "user_IAP.h"
#include "usart.h"

#define DATA_32                 ((uint32_t)0x12345678)
/* 要擦除内部FLASH的起始地址 */
#define FLASH_USER_START_ADDR   ((uint32_t)0x08020000)
/* 要擦除内部FLASH的结束地址 */
#define FLASH_USER_END_ADDR     ((uint32_t)0x0803FFFF)

#define TEST_SECTOR             1
#define TEST_SECTOR_ADDR        ((uint32_t)0x08020000)


#define WriteFlashAddress    ((uint32_t)0x08002800)//读写起始地址（内部flash的主存储块地址从0x08000000开始）


uint32_t Store_Data[STORE_COUNT];				//定义SRAM数组
uint32_t FLASH_BUF[SECTOR_SIZE/4]; 				//128KByte
uint32_t FLASHWORD_BUF[SECTOR_SIZE/4]; 				//128KByte

HAL_StatusTypeDef status_erase;
HAL_StatusTypeDef status_write;
uint8_t Len = 0;
const uint32_t testData[8] = {
    0x12345611, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
};


/**
  * 函    数：FLASH读取一个32位的字
  * 参    数：Address 要读取数据的字地址
  * 返 回 值：指定地址下的数据
  */
uint32_t UserFLASH_ReadWord(uint32_t Address)
{
	return *((__IO uint32_t *)(Address));	//使用指针访问指定地址下的数据并返回
}


/**
  * 函    数：FLASH读取一个16位的半字
  * 参    数：Address 要读取数据的半字地址
  * 返 回 值：指定地址下的数据
  */
uint16_t UserFLASH_ReadHalfWord(uint32_t Address)
{
	return *((__IO uint16_t *)(Address));	//使用指针访问指定地址下的数据并返回
}


/**
  * 函    数：FLASH读取一个8位的字节
  * 参    数：Address 要读取数据的字节地址
  * 返 回 值：指定地址下的数据
  */
uint8_t UserFLASH_ReadByte(uint32_t Address)
{
	return *((__IO uint8_t *)(Address));	//使用指针访问指定地址下的数据并返回
}


/**
  * 函    数：FLASH全擦除
  * 参    数：无
  * 返 回 值：status
  * 说    明：调用此函数后，FLASH的所有页都会被擦除，包括程序文件本身，擦除后，程序将不复存在
  */
HAL_StatusTypeDef UserFLASH_EraseAllPages(void)
{
	FLASH_EraseInitTypeDef EraseInit;
    uint32_t SectorError = 0;
	HAL_StatusTypeDef status;
	
	//解锁
	HAL_FLASH_Unlock();				
	//配置为全片擦除
    EraseInit.TypeErase = FLASH_TYPEERASE_MASSERASE;
	//执行擦除
    status = HAL_FLASHEx_Erase(&EraseInit, &SectorError);
    if (status != HAL_OK) {
		//擦除失败处理
        Error_Handler();
    }
	//加锁
	HAL_FLASH_Lock();
	
	return status;
}


uint32_t GetFlashAddressSector(uint32_t SectorAddress)
{
	if ((SectorAddress < FLASH_BANK1_BASE) || (SectorAddress > FLASH_END)) {
        return 10; // 地址无效
    }
	if (SectorAddress < FLASH_BANK2_BASE) {
        if(SectorAddress < 0x0801FFFF)
			return 0;
		else if(SectorAddress < 0x0803FFFF)
			return 1;
		else if(SectorAddress < 0x0805FFFF)
			return 2;
		else if(SectorAddress < 0x0807FFFF)
			return 3;
		else if(SectorAddress < 0x0809FFFF)
			return 4;
		else if(SectorAddress < 0x080BFFFF)
			return 5;
		else if(SectorAddress < 0x080DFFFF)
			return 6;
		else if(SectorAddress < 0x080FFFFF)
			return 7;
    } else {
        if(SectorAddress < 0x0811FFFF)
			return 0;
		else if(SectorAddress < 0x0813FFFF)
			return 1;
		else if(SectorAddress < 0x0815FFFF)
			return 2;
		else if(SectorAddress < 0x0817FFFF)
			return 3;
		else if(SectorAddress < 0x0819FFFF)
			return 4;
		else if(SectorAddress < 0x081BFFFF)
			return 5;
		else if(SectorAddress < 0x081DFFFF)
			return 6;
		else if(SectorAddress < 0x081FFFFF)
			return 7;
    }
	return 10;
}


/**
  * 函    数：FLASH扇区擦除
  * 参    数：SectorAddress 要擦除扇区的扇区地址
  * 返 回 值：status
  */
HAL_StatusTypeDef UserFLASH_EraseSector(uint32_t SectorAddress)
{
	FLASH_EraseInitTypeDef EraseInit;
    uint32_t SectorError = 0;
	HAL_StatusTypeDef status;
	uint32_t bankNumber;
	uint32_t sectorNumber;
	
	// 确定页地址所在的bank
    if (SectorAddress< FLASH_BANK2_BASE) {
        bankNumber = FLASH_BANK_1;
    } else {
        bankNumber = FLASH_BANK_2;
    }
	
	sectorNumber = GetFlashAddressSector(SectorAddress);
	if(sectorNumber  > 7)
		return HAL_ERROR;

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK2);
	//解锁
	status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status; // 解锁失败直接返回
    }
		
	//配置为全片擦除
	EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;  // H7使用扇区擦除
	EraseInit.Banks = bankNumber;
    EraseInit.Sector = sectorNumber;              // 示例：擦除扇区5（需根据实际地址计算）
    EraseInit.NbSectors = 1;                        // 擦除1个扇区
    EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; // H7必须指定电压范围（3.3V）
	//执行擦除
	status = HAL_FLASHEx_Erase(&EraseInit, &SectorError);
    if (status != HAL_OK) {
		//擦除失败处理
        Error_Handler();
    }
	//加锁
	HAL_FLASH_Lock();	
	
	return status;
}


/**
  * @brief  向指定地址写入32位4字节数据（HAL库版本）
  * @param  Address: 目标地址（必须4字节对齐，地址末位需为0）
  * @param  Data: 要写入的32位数据
  * @retval HAL_StatusTypeDef: HAL_OK表示成功，其他值为错误状态
  */
HAL_StatusTypeDef CHECKC;
uint32_t CHECKD = 0;
HAL_StatusTypeDef UserFLASH_ProgramWord(uint32_t Address, uint32_t* DataAddress)
{
    HAL_StatusTypeDef status;
    
    // 1. 解锁Flash（硬件强制要求）
    status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status; // 解锁失败直接返回
    }
	if((uint32_t)Address%4!=0)
		CHECKD = 1;
	else if((uint32_t)DataAddress%4!=0)
		CHECKD = 2;
    // 2. 执行16位编程（注意强制类型转换）
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, (uint32_t)DataAddress);
    
	CHECKC = status;
    // 3. 锁定Flash（防止意外修改）
    HAL_FLASH_Lock();
    
    return status;
}


HAL_StatusTypeDef User_Store_Init(void)
{
	HAL_StatusTypeDef status;
	/*判断是不是第一次使用*/
	if (UserFLASH_ReadHalfWord(STORE_START_ADDRESS) != 0xACAC)	//读取第一个半字的标志位，if成立，则执行第一次使用的初始化
	{
		status = UserFLASH_EraseSector(STORE_START_ADDRESS);					//擦除指定页
		if (status != HAL_OK) {
			return status; // 失败直接返回
		}
		uint32_t pData = 0xACACACAC;
		//注意STM32采用小端模式，低字节 存储在 低地址，高字节 存储在 高地址。
		status = UserFLASH_ProgramWord(STORE_START_ADDRESS, &pData);	//在第一个半字写入自己规定的标志位，用于判断是不是第一次使用
		if (status != HAL_OK) {
			return status; // 失败直接返回
		}
		for (uint16_t i = 1; i < STORE_COUNT; i ++)				//循环STORE_COUNT次，除了第一个标志位
		{
			//每个半字占据两个字节，一个字节8位
			//每个扇区1024个字节，所以STORE_COUNT = 512，1024 = 512 * 2
			status = UserFLASH_ProgramWord(STORE_START_ADDRESS + i * 4, 0x00000000);		//除了标志位的有效数据全部清0
			if (status != HAL_OK) {
				return status; // 失败直接返回
			}
		}
	}
	
	/*上电时，将闪存数据加载回SRAM数组，实现SRAM数组的掉电不丢失*/
	for (uint16_t i = 0; i < STORE_COUNT; i ++)					//循环STORE_COUNT次，包括第一个标志位
	{
		Store_Data[i] = UserFLASH_ReadHalfWord(STORE_START_ADDRESS + i * 2);		//将闪存的数据加载回SRAM数组
	}
	
	return status;
}


uint8_t UPDATE_STATE = 0;//0:NO_UPDATE 1:NEED_UPDATE 2:FINISH_UPDATE
HAL_StatusTypeDef User_Update_Check(void)
{
	HAL_StatusTypeDef status;
	/*判断是不是需要更新*/
	if (UserFLASH_ReadHalfWord(FLASH_APP1_ADDR - 2) == 0xACAC)	//读取标志位
	{
		UPDATE_STATE = 0;
	}
	else if (UserFLASH_ReadHalfWord(FLASH_APP1_ADDR - 2) == 0xAAAA)	//读取标志位
	{
		UPDATE_STATE = 1;
	}
	
	return status;
}
/**
  * @brief  将SRAM数组数据保存到Flash（HAL库版本）
  * @param  无
  * @retval HAL_StatusTypeDef: 操作状态（HAL_OK为成功）
  */
HAL_StatusTypeDef User_Store_Save(void)
{
    HAL_StatusTypeDef status;

    status = UserFLASH_EraseSector(STORE_START_ADDRESS);
	if (status != HAL_OK) {
				return status; // 失败直接返回
	}
	//写入
	for (uint32_t i = 0; i < STORE_COUNT; i ++)			//循环STORE_COUNT次，包括第一个标志位
	{
		status = UserFLASH_ProgramWord(STORE_START_ADDRESS + i * 2, Store_Data + i);	//将SRAM数组的数据备份保存到闪存
		if (status != HAL_OK) {
			return status; // 失败返回
		}
	}
    //锁定Flash
    HAL_FLASH_Lock();
    return status;
}


HAL_StatusTypeDef User_Store_Clear(void)
{
	HAL_StatusTypeDef status;
	for (uint32_t i = 1; i < STORE_COUNT; i ++)			//循环STORE_COUNT次，除了第一个标志位
	{
		Store_Data[i] = 0x00000000;							//SRAM数组有效数据清0
	}
	status = User_Store_Save();										//保存数据到闪存
	return status;
}


//读出一段数据（32位）
void FLASH_Read(uint32_t Addr,uint32_t *pBuff,uint32_t len)
{
	uint32_t i;
	for(i=0;i<len;i++)
	{
		pBuff[i] = UserFLASH_ReadWord(Addr);
		Addr += 4;
	}
}


/**
  * uint32_t Addr：要写入的 Flash 起始地址（例如 0x08008000）。
  * uint16_t *pBuff：指向要写入数据的缓冲区（16 位数据数组）。
  * uint32_t len：要写入的数据长度（单位：16 位，即 len=1 表示写入 2 字节）
  */
HAL_StatusTypeDef FLASH_Write(uint32_t Addr,uint32_t *pBuff,uint32_t len)
{
	//Flash只能按扇区擦除
	uint32_t Sector_Number;  //第几扇区
	uint32_t Sector_Relative;   //扇区内偏移地址（按16位计算）
	uint32_t Sector_Remain;     //扇区内剩余地址（按16位计算）
	uint32_t OffAddr;           //减去0x08000000后的地址
	uint32_t i;
	uint32_t counter = 0;
	HAL_StatusTypeDef status;
	
	//判断地址合理性
	if(Addr<FLASH_BASE || Addr>FLASH_END)
		return HAL_ERROR;
	
	//判断len是否符合要求，保证是整数个FLASHWORD
	if(len % 8 != 0)
		return HAL_ERROR;
	
	//判断起始写入地址是否符合要求，地址要32字节对齐
	if(Addr % 32 != 0)
		return HAL_ERROR;
	
	counter = len / 8;
	
    //解锁Flash（硬件强制要求）
    status = HAL_FLASH_Unlock();
	if (status != HAL_OK)
	{
		//HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"NONO1", 5);
		return status; // 失败返回
	}
	
	OffAddr = Addr - FLASH_BASE;//偏移地址
	Sector_Number = OffAddr/SECTOR_SIZE;//扇区编号
	Sector_Relative = (OffAddr%SECTOR_SIZE)/32;//扇区内第几个FLASHWORD，一个FLASHWORD32字节
	Sector_Remain = SECTOR_SIZE/32-Sector_Relative;//扇区内剩余几个FLASHWORD
	
	//如果一页能够写完
	if(counter <= Sector_Remain) Sector_Remain = counter;
	
	//写入
	while(1)
	{
		//整页读出（每次读出4字节，读取SECTOR_SIZE/4次）
		//Flash 擦除会清空整个扇区，因此必须先读旧数据到缓冲区，替换新数据后整体写回，避免丢失扇区内其他有效数据
		FLASH_Read(Sector_Number*SECTOR_SIZE+FLASH_BASE,FLASH_BUF,SECTOR_SIZE/4);
		//每个FLASHWORD有8个32位
		for(i=0;i<Sector_Remain*8;i++)
		{
			if(FLASH_BUF[Sector_Relative*8+i] != 0xFFFFFFFF) break; //需要擦除
		}
		if(i<Sector_Remain*8) //需要擦除
		{
			UserFLASH_EraseSector(Sector_Number*SECTOR_SIZE+FLASH_BASE);//整页擦除
			
			//将需要写入的数据替换进入，此时FLAH_BUF内就是需要写入的所有字节
			for(i=0;i<Sector_Remain*8;i++)
			{
				FLASH_BUF[Sector_Relative*8+i] = pBuff[i];
			}
			
			//整个扇区按FLASHWORD写入FLASH
			for(i=0;i<SECTOR_SIZE/32;i++)
				UserFLASH_ProgramWord(Sector_Number*SECTOR_SIZE+FLASH_BASE + i * 32, (uint32_t *)(FLASH_BUF+i*8));
		}
		else
		{
			//按剩余FLASHWORD数
			for(i=0;i<Sector_Remain;i++)
				UserFLASH_ProgramWord(Addr + i * 32, (uint32_t *)(pBuff + i*8));
		}
		
		if(counter == Sector_Remain) break; //写入结束
		else
		{
			Sector_Number++;          //扇区+1
			Sector_Relative = 0;      //扇区内偏移0
			pBuff += Sector_Remain * 8;   //更新传入的数组指针
			Addr += Sector_Remain*32;    //写地址偏移
			counter -= Sector_Remain;     //更新剩余需要写入的FLASHWORD数
			
			if(counter>(SECTOR_SIZE/32)) Sector_Remain = SECTOR_SIZE/32;
			else Sector_Remain = counter;
		}
	}
	
	status = HAL_FLASH_Lock();
	//HAL_UART_Transmit_DMA(&huart1, (uint8_t*)"OKOK!", 5);
	return status; // 返回
}



//HAL_StatusTypeDef Flash_EraseSector(uint32_t sector, uint32_t bank) {
//    FLASH_EraseInitTypeDef eraseInit;
//    uint32_t sectorError = 0;

//    /* 解锁 Flash */
//    HAL_FLASH_Unlock();

//    /* 清除所有错误标志 */
//    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS_BANK1);

//    /* 配置擦除参数（STM32H7 使用 Sector 擦除） */
//    eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
//    eraseInit.Banks = bank;           // 选择 Bank1 或 Bank2
//    eraseInit.Sector = sector;                // 扇区号（0~7 或 0~15，取决于型号）
//    eraseInit.NbSectors = 1;                  // 擦除 1 个扇区
//    eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 电压范围（H7 必须设置）

//    /* 执行擦除 */
//    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);

//    /* 锁定 Flash */
//    HAL_FLASH_Lock();
//	
//	return status;
//}


/**
 * @brief 写入数据到 Flash
 * @param addr: 起始地址（需已擦除）
 * @param data: 数据指针
 * @param len: 数据长度（字节）
 * @return HAL_StatusTypeDef
 */
//HAL_StatusTypeDef Flash_Write(uint32_t addr, uint32_t *data, uint32_t len) {
//    HAL_StatusTypeDef status = HAL_OK;
//    uint8_t k=0;
//    uint32_t Address;

//    Address = WriteFlashAddress;
//    FLASHStatus = FLASH_COMPLETE;
//    FLASH_Unlock();//解锁
//    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);//清除所有标志
//    FLASHStatus = FLASH_ErasePage(WriteFlashAddress);//扇区擦除
//    if(FLASHStatus == FLASH_COMPLETE)
//    {
//        for(k=0;(k<len) && (FLASHStatus == FLASH_COMPLETE);k++)
//        {
//            FLASHStatus = FLASH_ProgramWord(Address, buff[k]);//写入一个字（32位）的数据入指定地址
//            Address = Address + 4;//地址偏移4个字节
//        }        
//        FLASH_Lock();//重新上锁，防止误写入
//    }
//    else
//    {
//        return 0;
//    }
//    if(FLASHStatus == FLASH_COMPLETE)
//    {
//        return 1;
//    }
//    return 0;
//    return status;
//}

/**
 * @brief 验证 Flash 数据
 * @param addr: 起始地址
 * @param data: 预期数据
 * @param len: 数据长度（字节）
 * @return HAL_StatusTypeDef
 */
uint8_t flashData[64] = {0}; 
void Flash_Verify(uint32_t addr) {
    memcpy(flashData, (void*)addr, 32);
}

/*
	//使用样例
	status_erase = Flash_EraseSector(FLASH_SECTOR_1, FLASH_BANK_1);
	Len = sizeof(testData);
	status_write = Flash_Write(TEST_SECTOR_ADDR, (uint32_t*)testData, sizeof(testData));
	Flash_Verify(TEST_SECTOR_ADDR);
*/



