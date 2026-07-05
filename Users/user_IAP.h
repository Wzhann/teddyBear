#ifndef _user_IAP_h
#define _user_IAP_h

#include "main.h"

#define FLASH_APP1_ADDR 0x08020000 //第一个应用程序起始地址//第二个扇区开始
#define FLASH_APP2_ADDR 0x08100000 //第二个应用程序起始地址
//#define FLASH_UPDATE_ADDR 0x81ffff00 //判断是否有新程序


/* 启动的步骤 */
#define Startup_Normol 0	///< 正常启动
#define Startup_Update 1	///< 升级再启动

#define Application_Size		0x00080000U			///< 应用程序的大小
//#define APP_BUF_LEN (Application_Size / sizeof(uint32_t))

///* --------------  Flash 基地址 -------------- */
//#define FLASH_BASE          0x08000000UL

///* --------------  每个扇区大小 -------------- */
//#define FLASH_SECTOR_SIZE   0x20000UL       /* 128 KB */

/* --------------  Bank0（扇区 0~7） -------------- */
#define FLASH_BANK0_SECTOR0_ADDR  (FLASH_BASE + 0*FLASH_SECTOR_SIZE)   /* 0x0800 0000 */
#define FLASH_BANK0_SECTOR1_ADDR  (FLASH_BASE + 1*FLASH_SECTOR_SIZE)   /* 0x0802 0000 */
#define FLASH_BANK0_SECTOR2_ADDR  (FLASH_BASE + 2*FLASH_SECTOR_SIZE)   /* 0x0804 0000 */
#define FLASH_BANK0_SECTOR3_ADDR  (FLASH_BASE + 3*FLASH_SECTOR_SIZE)   /* 0x0806 0000 */
#define FLASH_BANK0_SECTOR4_ADDR  (FLASH_BASE + 4*FLASH_SECTOR_SIZE)   /* 0x0808 0000 */
#define FLASH_BANK0_SECTOR5_ADDR  (FLASH_BASE + 5*FLASH_SECTOR_SIZE)   /* 0x080A 0000 */
#define FLASH_BANK0_SECTOR6_ADDR  (FLASH_BASE + 6*FLASH_SECTOR_SIZE)   /* 0x080C 0000 */
#define FLASH_BANK0_SECTOR7_ADDR  (FLASH_BASE + 7*FLASH_SECTOR_SIZE)   /* 0x080E 0000 */

/* --------------  Bank1（扇区 8~15） -------------- */
#define FLASH_BANK1_SECTOR8_ADDR  (FLASH_BASE + 8*FLASH_SECTOR_SIZE)   /* 0x0810 0000 */
#define FLASH_BANK1_SECTOR9_ADDR  (FLASH_BASE + 9*FLASH_SECTOR_SIZE)   /* 0x0812 0000 */
#define FLASH_BANK1_SECTOR10_ADDR (FLASH_BASE + 10*FLASH_SECTOR_SIZE)  /* 0x0814 0000 */
#define FLASH_BANK1_SECTOR11_ADDR (FLASH_BASE + 11*FLASH_SECTOR_SIZE)  /* 0x0816 0000 */
#define FLASH_BANK1_SECTOR12_ADDR (FLASH_BASE + 12*FLASH_SECTOR_SIZE)  /* 0x0818 0000 */
#define FLASH_BANK1_SECTOR13_ADDR (FLASH_BASE + 13*FLASH_SECTOR_SIZE)  /* 0x081A 0000 */
#define FLASH_BANK1_SECTOR14_ADDR (FLASH_BASE + 14*FLASH_SECTOR_SIZE)  /* 0x081C 0000 */
#define FLASH_BANK1_SECTOR15_ADDR (FLASH_BASE + 15*FLASH_SECTOR_SIZE)  /* 0x081E 0000 */

/* --------------  快速索引宏 -------------- */
#define FLASH_SECTOR_ADDR(n)  (FLASH_BASE + (n)*FLASH_SECTOR_SIZE)

//extern uint32_t temp[APP_BUF_LEN];
extern uint32_t TOTAL_addr;
typedef void (*AppEntry)(void);//定义了一个名为 AppEntry 的函数指针类型，指向无参数、无返回值的函数
void JumpToApp(uint32_t App_address);
void IAP_write_App_fromuart(uint32_t addr,uint8_t *buf,uint32_t len);
void MoveCode(uint32_t src_addr, uint32_t des_addr, uint32_t byte_size);
void Start_BootLoader(void);

#endif



