/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_10
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_11
#define DATA_32                 ((uint32_t)0x12345678)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t GetSector(uint32_t Address);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  printf("test Demo15_7 read and write Flash_In_Chip, and used for USBD MSC.\r\n");


//  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
//                    FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);
//  HAL_FLASH_Unlock();

  // 强制擦除目标扇区（以STM32F4为例）
//  FLASH_EraseInitTypeDef erase = {
//          .TypeErase = FLASH_TYPEERASE_SECTORS,
//          .Sector = FLASH_SECTOR_10,  // 对应0x080A0000的扇区
//          .NbSectors = 1,
//          .VoltageRange = FLASH_VOLTAGE_RANGE_3
//  };
//  uint32_t sector_error;
//  if(HAL_FLASHEx_Erase(&erase, &sector_error) !=HAL_OK)
//	  printf("erase failure.\r\n");
//  else
//  	  printf("erase success.\r\n");
//
//  // 写入测试数据（带硬件级等待）
//  uint32_t addr = 0x080C0000;
//  uint32_t data = 0x12345678;
//  HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, data);
//  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)); // 必须等待！
//
//  // 内存屏障+强制读取
//  __DSB();
//  volatile uint32_t read_data = *(volatile uint32_t*)addr;
//  HAL_FLASH_Lock();
//
//  printf("Addr:0x%08lX 写入:0x%08lX 读取:0x%08lX\n",
//             addr, data, read_data);


// 测试单个双字写入读出
//  uint64_t test_data = 0x1122334455667788;
//  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x080A0000, test_data);
//  volatile uint64_t read_back = *(volatile uint64_t*)0x080A0000;

//  Test1_FlashInChip();
  Test2_FlashInChip();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/*
*	 @brief:get sector
*/
/* start test 6 */
uint32_t Addr_Index[12+1] = {ADDR_FLASH_SECTOR_0,ADDR_FLASH_SECTOR_1,ADDR_FLASH_SECTOR_2,\
                          ADDR_FLASH_SECTOR_3,ADDR_FLASH_SECTOR_4,ADDR_FLASH_SECTOR_5,\
                          ADDR_FLASH_SECTOR_6,ADDR_FLASH_SECTOR_7,ADDR_FLASH_SECTOR_8,\
                          ADDR_FLASH_SECTOR_9,ADDR_FLASH_SECTOR_10,ADDR_FLASH_SECTOR_11,ADDR_FLASH_SECTOR_12};

uint8_t FlashSector_Index[12] = {FLASH_SECTOR_0,FLASH_SECTOR_1,FLASH_SECTOR_2,
                                FLASH_SECTOR_3,FLASH_SECTOR_4,FLASH_SECTOR_5,
                                FLASH_SECTOR_6,FLASH_SECTOR_7,FLASH_SECTOR_8,
                                FLASH_SECTOR_9,FLASH_SECTOR_10,FLASH_SECTOR_11,};
/**
  * @brief  give the sector according to the address
  *			such as:
			BeginSector =GetSector(start_addr);
			EndSector =GetSector(start_addr +bytes_tobe_written -1);
  * @param  Address：address
  * @retval the sector where the address is located
  */
static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;
	for(uint8_t i=0; i<12; i++)
		if((Address >=Addr_Index[i]) && (Address <Addr_Index[i+1]))
			sector =(uint32_t)(FlashSector_Index[i]);

	return sector;
}


void Flash_Write(uint8_t *RAM_Buffer, uint32_t start_addr, uint32_t bytes_tobe_written)
{
	HAL_FLASH_Unlock();

/* erase user area.
 * The user area refers to the space not utilized by the program itself,
 * which can be customized.
*/
/* clear FLASH flags*/
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	uint32_t StartSector = GetSector(start_addr);
	uint32_t EndSector = GetSector(start_addr +bytes_tobe_written -1);

/* erase sectors or mass********************************/
	FLASH_EraseInitTypeDef *eraseinit ={};
	eraseinit->Banks =FLASH_BANK_1;
	eraseinit->NbSectors =EndSector -StartSector +1;
//	eraseinit->Sector =FLASH_SECTOR_8 |FLASH_SECTOR_9 |FLASH_SECTOR_10 |FLASH_SECTOR_11;
	eraseinit->Sector =StartSector;
	eraseinit->TypeErase =FLASH_TYPEERASE_SECTORS;
	eraseinit->VoltageRange =FLASH_VOLTAGE_RANGE_3;
	uint32_t *SectorError =0;
	HAL_FLASHEx_Erase(eraseinit, SectorError);

/* Write into FLASH in words ********************************/
  	uint16_t i =0;
  	uint16_t j =bytes_tobe_written/4;

/* The dynamic pointer points to the first address of the static array.*/
  	uint8_t buf_write[] ={};
  	RAM_Buffer =buf_write;

	for(i=0; i<j; i++)
	{
		uint32_t add = (uint32_t)(start_addr +i*4);
//		uint32_t dat = RAM_Buffer[i*8 +0]|(RAM_Buffer[i*8 +1]<<8)|(RAM_Buffer[i*8 +2]<<16)|(RAM_Buffer[i*8 +3]<<24);	//RAM_Buffer[i]=buf_write[i]
		uint32_t dat = buf_write[i*8 +0]|(buf_write[i*8 +1]<<8)|(buf_write[i*8 +2]<<16)|(buf_write[i*8 +3]<<24);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, add, dat);
	}


	HAL_FLASH_Lock();
}

/*
*  @brief:read to RAM_Buffer flash according to byte address.
*  @prama start_addr: physic address begin to read,such as 0x08020100.
*  @param bytes_tobe_read: total bytes to be read,bytes.
*  @retval RAM_Bufferֵ
*/
void Flash_Read(uint8_t *RAM_Buffer, uint32_t start_addr, uint32_t bytes_tobe_read)
{
  	uint16_t i =0;
  	uint16_t j =bytes_tobe_read/4;

  	for(i=0;i<j;i++)
  	{
  		uint32_t add = (uint32_t)(start_addr +i*4);
  		uint32_t dat = *(__IO uint32_t *)(add);

  		RAM_Buffer[i*8 +0]=(uint8_t)(dat&0xFF);
  		RAM_Buffer[i*8 +1]=(uint8_t)((dat>>8)&0xFF);
  		RAM_Buffer[i*8 +2]=(uint8_t)((dat>>16)&0xFF);
  		RAM_Buffer[i*8 +3]=(uint8_t)((dat>>24)&0xFF);
  	}
}

/* end test 6 */

/**
  * @brief  Test1 read and write flash,
  * @brief  directly call HAL library functions.
  * @param  None
  * @retval 0:OK,-1,ERROR
  */
void Test1_FlashInChip(void)
{
	uint32_t StartSector = 0;
	uint32_t EndSector = 0;
	uint32_t Address = 0;

	__IO uint32_t Data32 = 0;
	__IO uint32_t MemoryProgramStatus = 0;

	HAL_FLASH_Unlock();

/* erase user area.
 * The user area refers to the space not utilized by the program itself,
 * which can be customized.
*/
/* clear FLASH flags*/
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	StartSector = GetSector(FLASH_USER_START_ADDR);
	EndSector = GetSector(FLASH_USER_END_ADDR);

/* erase sectors or mass********************************/
//	FLASH_EraseInitTypeDef *eraseinit ={};
//	eraseinit->NbSectors =EndSector -StartSector +1;
//	eraseinit->Sector =StartSector;	//this 2 code have same effect
//	eraseinit->TypeErase =FLASH_TYPEERASE_SECTORS;
//	eraseinit->VoltageRange =FLASH_VOLTAGE_RANGE_3;
//	uint32_t *SectorError =0;
//	HAL_FLASHEx_Erase(eraseinit, SectorError);

	FLASH_EraseInitTypeDef eraseinit ={};
	eraseinit.NbSectors =EndSector -StartSector +1;
	eraseinit.Sector =StartSector;	//this 2 code have same effect
	eraseinit.TypeErase =FLASH_TYPEERASE_SECTORS;
	eraseinit.VoltageRange =FLASH_VOLTAGE_RANGE_3;
	uint32_t SectorError =0;
	if(HAL_FLASHEx_Erase(&eraseinit, &SectorError) !=HAL_OK){
		printf("erase failure.\r\n");
		//此处写入异常处理
	}
	else
		printf("erase success.\r\n");

/* Write into FLASH in words *******************************
 * Write a constant to each address in the specified address space
 */
	Address =FLASH_USER_START_ADDR;
	while (Address <FLASH_USER_END_ADDR)
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32) ==HAL_OK)
		{
	  		printf("write Address %08lx =%08lx,\r\n",Address,DATA_32);	//test
			Address =Address +4;
		}
    }

	HAL_FLASH_Lock();

/* read from FLASH and check***************************************/
/* MemoryProgramStatus  =0: write in correct
 * MemoryProgramStatus !=0:write in error
*/
  	Address =FLASH_USER_START_ADDR;
  	MemoryProgramStatus = 0;

  	//Check if the data read out is equal to the data written in.
  	while (Address <FLASH_USER_END_ADDR)
  	{
  		Data32 = *(__IO uint32_t*)Address;
  		printf("read Address %08lx =%08lx,\r\n",Address,Data32);	//test

  		//record data checked failed
  		if(Data32 != DATA_32)
  			MemoryProgramStatus++;
  		Address +=4;												//Address alignment
  	}
  	printf("check error times =%08lx,\r\n",MemoryProgramStatus);	//test
}



void Test2_FlashInChip(void)
{
	uint32_t StartSector = 0;
	uint32_t EndSector = 0;
	uint32_t Address = 0;
	uint16_t bytes_tobe_rw =64;	//bytes

/* erase user area.
 * The user area refers to the space not utilized by the program itself,
 * which can be customized.
 */
/* clear FLASH flags*/
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR);

	HAL_FLASH_Unlock();

	StartSector = GetSector(FLASH_USER_START_ADDR);
	EndSector = GetSector(FLASH_USER_START_ADDR +bytes_tobe_rw -1);

/* erase sectors or mass********************************/
	FLASH_EraseInitTypeDef erase = {
			.TypeErase =FLASH_TYPEERASE_SECTORS,
	        .Sector =StartSector,
	        .NbSectors =(EndSector -StartSector +1),
	        .VoltageRange =FLASH_VOLTAGE_RANGE_3
	};
	uint32_t sector_error;
	if(HAL_FLASHEx_Erase(&erase, &sector_error) !=HAL_OK){
		printf("erase failure.\r\n");
		//Here is exception handling.
	}
	else
	  	printf("erase success.\r\n");

/* Write into FLASH in bytes,8bits ********************************/
	uint8_t DATA8[bytes_tobe_rw] ={};
	for(uint32_t i=0; i <bytes_tobe_rw;i++)
		DATA8[i] =i;

//	Address =FLASH_USER_START_ADDR;
//	uint8_t j =0;
//	while (Address <(uint32_t)(FLASH_USER_START_ADDR +bytes_tobe_rw))
//	{
//		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, DATA8[j]) == HAL_OK)
//		{
//	  		printf("写入Address %08lx =%02x,\r\n",Address,DATA8[j]);	//test
//			Address ++;
//			j ++;
//		}
//    }
//	while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));

/* Write into FLASH in half word,16bits********************************/
//	uint16_t DATA16[bytes_tobe_rw/2] ={};
//	for(uint32_t j=0; j <bytes_tobe_rw/2; j++)
//	{
//		DATA16[j] =DATA8[j*2+0]|(DATA8[j*2+1]<<8);
//		printf("data ready, DATA16[0x%08lx] =0x%04x,\r\n",j,DATA16[j]);//test
//	}
//
//	Address =FLASH_USER_START_ADDR;
//	uint32_t j =0;
//	while (Address <(uint32_t)(FLASH_USER_START_ADDR +bytes_tobe_rw))
//	{
//		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, DATA16[j]) == HAL_OK)
//		{
//			printf("写入Address %08lx =%04x,\r\n",Address,DATA16[j]);	//test
//			Address +=2;
//			j ++;
//		}
//	}
//	while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));

/* Write into FLASH in words, 32bits ********************************/
//	uint32_t DATA32[bytes_tobe_rw/4] ={};
//	for(uint32_t j=0; j <bytes_tobe_rw/4; j++)
//	{
//		DATA32[j] =DATA8[j*4+0] |(DATA8[j*4+1]<<8) |(DATA8[j*4+2]<<16) |(DATA8[j*4+3]<<24);
//		printf("data ready, DATA32[0x%08lx] =0x%08lx,\r\n",j,DATA32[j]);//test
//	}
//
//	Address =FLASH_USER_START_ADDR;
//	uint32_t j =0;
//	while (Address <(uint32_t)(FLASH_USER_START_ADDR +bytes_tobe_rw))
//	{
//		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA32[j]) == HAL_OK)
//		{
//			printf("写入Address %08lx =%08lx,\r\n",Address,DATA32[j]);	//test
//			Address +=4;
//			j ++;
//		}
//	}
//	while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));

/* Write into FLASH in double words, 64bits********************************/
	uint64_t DATA64[bytes_tobe_rw/8] ={};
	uint32_t Hi_DATA64[bytes_tobe_rw/8] ={};
	uint32_t Li_DATA64[bytes_tobe_rw/8] ={};

	for(uint32_t j=0; j <bytes_tobe_rw/8; j++)
	{
		/* When shifting left more than 31 bits,
		 * ensure the operand is 64-bit (such as using uint64_t typecasting)
		 * avoid 32-bit integer shift truncation.
		 */
		DATA64[j] =DATA8[j*8+0] |(DATA8[j*8+1]<<8) |(DATA8[j*8+2]<<16) |(DATA8[j*8+3]<<24) |
				((uint64_t)DATA8[j*8+4]<<32) |
				((uint64_t)DATA8[j*8+5]<<40) |
				((uint64_t)DATA8[j*8+6]<<48) |
				((uint64_t)DATA8[j*8+7]<<56);
		Hi_DATA64[j] =DATA64[j] >>32;
		Li_DATA64[j] =DATA64[j] &(0xFFFFFFFF);
		printf("data ready, DATA64[%lx] =0x%08lx%08lx,\r\n",j, Hi_DATA64[j], Li_DATA64[j]);	//test
	}

	Address =FLASH_USER_START_ADDR;
	uint32_t j =0;
	HAL_StatusTypeDef ret1,ret2;

	if(*(volatile uint32_t*)Address != 0xFFFFFFFF) {
		HAL_FLASHEx_Erase(&erase, &sector_error);	//Re-erase
	}
/* When TypeProgram is double word,
 * HAL_FLASH_Program write fails,
 * and it can only be written successfully after being split into word
 */
 while (Address <(uint32_t)(FLASH_USER_START_ADDR +bytes_tobe_rw))
 {
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, DATA64[j]);	//Unavailable

	ret2 =HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Li_DATA64[j]);
	ret1 =HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address+4, Hi_DATA64[j]);

	if((ret1 ==HAL_OK) &&(ret2 ==HAL_OK))
	{
		printf("写入Address %08lx =0x%08lx%08lx,\r\n",Address, Hi_DATA64[j], Li_DATA64[j]);	//test
	}

	Address +=8;
	j ++;
 }
 while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY));
 HAL_FLASH_Lock();


/* read from FLASH in bytes, 8bits***************************************/
//  Address =FLASH_USER_START_ADDR;
//  volatile uint8_t Data8[bytes_tobe_rw] ={};
//	j=0;
//
//  while (Address <(FLASH_USER_START_ADDR +bytes_tobe_rw))
//  {
//	__DSB();
//  	Data8[j] = *(__IO uint8_t*)Address;
//  	printf("read Address %08lx =%02x,\r\n", Address, Data8[j]);		//test
//  	Address++;
//  	j++;
//  }

/* read from FLASH in half words, 16bits***************************************/
//  Address =FLASH_USER_START_ADDR;
//  volatile uint16_t Data16[bytes_tobe_rw/2] ={};
//  j=0;
//
//  while (Address <(FLASH_USER_START_ADDR +bytes_tobe_rw))
//  {
//	  __DSB();
//	  Data16[j] = *(__IO uint16_t*)Address;
//	  printf("read Address %08lx =0x%04x,\r\n", Address, Data16[j]);		//test
//	  Address +=2;
//	  j++;
//  }


/* read from FLASH in words, 32bits***************************************/
//  Address =FLASH_USER_START_ADDR;
//  volatile uint32_t Data32[bytes_tobe_rw/4] ={};
//	volatile uint32_t *pFlash;
//	j=0;
//
//	while (Address <(FLASH_USER_START_ADDR +bytes_tobe_rw))
//	{
//		__DSB();
//		pFlash = (volatile uint32_t*)Address;
//		Data32[j] = *pFlash;  //Forced read from physical address
//
//		printf("read Address %08lx =%08lx,\r\n", Address, Data32[j]);		//test
//		Address +=4;
//		j++;
//	}


/* read from FLASH in double words, 64bits split into two 32bits*******
 * method 1
 */
//  Address =FLASH_USER_START_ADDR;
//  volatile uint32_t Hi_Data64[bytes_tobe_rw/8] ={};
//  volatile uint32_t Li_Data64[bytes_tobe_rw/8] ={};
//  j=0;
//
//  while (Address <(FLASH_USER_START_ADDR +bytes_tobe_rw))
//  {
//	  __DSB();
//	  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_5); //Adjust according to the clock frequency
//
//	  //Read out 64 bits data at two times
//	  Li_Data64[j] =*(volatile uint32_t*)Address;
//	  Address +=4;
//	  Hi_Data64[j] =*(volatile uint32_t*)(Address);
//
//    printf("read Address, %08lx =0x%08lx%08lx,\r\n",Address-4, Hi_Data64[j], Li_Data64[j]);	//test
//	  Address +=4;
//    j++;
//  }

/* read from FLASH in double words, 64bits split into two 32bits*******
 * method 2 is more convenient to read.
 */
//  Address =FLASH_USER_START_ADDR;
//  volatile uint32_t Hi_Data64[bytes_tobe_rw/8] ={};
//  volatile uint32_t Li_Data64[bytes_tobe_rw/8] ={};
//  j=0;
//
//  while (Address <(FLASH_USER_START_ADDR +bytes_tobe_rw))
//  {
//	  __DSB();
//  	  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_5); //Adjust according to the clock frequency
//
//  	  //Read out 64 bits data at two times
//  	  Li_Data64[j] =*(volatile uint32_t*)Address;
//  	  Hi_Data64[j] =*(volatile uint32_t*)(Address +4);
//
//      printf("read Address, %08lx =0x%08lx%08lx,\r\n",Address, Hi_Data64[j], Li_Data64[j]);	//test
//  	  Address +=8;
//      j++;
//  }

/* read from FLASH in double words, 64bits************************
 * method 3
 */
  Address =FLASH_USER_START_ADDR;
  volatile uint64_t Data64[bytes_tobe_rw/8] ={};
  uint32_t Hi_Data64[bytes_tobe_rw/8] ={};
  uint32_t Li_Data64[bytes_tobe_rw/8] ={};
  j=0;

  volatile uint32_t Li_data64 = *(__IO uint32_t*)Address;
  printf("read first Address, %08lx =%08lx,\r\n",Address, Li_data64);	//test

  while (Address <(FLASH_USER_START_ADDR +bytes_tobe_rw))
  {
	  __DSB();
  	  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_5); //Adjust according to the clock frequency

  	  //Read out 64 bits data at a time
  	  Data64[j] = *(__IO uint64_t*)Address;
  	  Hi_Data64[j] =(Data64[j] >>32) &(0xFFFFFFFF);
  	  Li_Data64[j] =Data64[j] &(0xFFFFFFFF);

  	  printf("read Address, %08lx =0x%08lx%08lx,\r\n",Address, Hi_Data64[j], Li_Data64[j]);	//test
  	  Address +=8;
  	  j++;
  }

}//void Test2_FlashInChip(void)


int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart3,(uint8_t*)&ch,1,0xFFFF);
	return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
