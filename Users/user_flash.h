#ifndef _user_flash_h
#define _user_flash_h

#include "main.h"

#define STORE_START_ADDRESS		0x0800FC00		//存储的起始地址
#define STORE_COUNT				512				//存储数据的个数
#define SECTOR_SIZE (1024*128)       //扇区大小，根据型号改变

extern uint8_t UPDATE_STATE;//0:NO_UPDATE 1:NEED_UPDATE 2:FINISH_UPDATE

HAL_StatusTypeDef User_Update_Check(void);
HAL_StatusTypeDef User_Store_Init(void);
HAL_StatusTypeDef User_Store_Save(void);
HAL_StatusTypeDef User_Store_Clear(void);
void FLASH_Read(uint32_t Addr,uint32_t *pBuff,uint32_t len);
HAL_StatusTypeDef FLASH_Write(uint32_t Addr,uint32_t *pBuff,uint32_t len);
HAL_StatusTypeDef UserFLASH_EraseSector(uint32_t SectorAddress);
HAL_StatusTypeDef UserFLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
HAL_StatusTypeDef UserFLASH_ProgramWord(uint32_t Address, uint32_t* DataAddress);
extern uint32_t Store_Data[STORE_COUNT];
#endif



