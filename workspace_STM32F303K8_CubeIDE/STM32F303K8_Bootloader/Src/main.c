/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "usart.h"
//#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define main bootloader_main


extern int _main_data_start; // リンカスクリプトで定義したバイナリ転送先アドレス

// OK
typedef void(*pFunction)(void);
pFunction JumpToApplication;
#define APP_ADDRESS 0x08002000
#define APP_STACK   (APP_ADDRESS + 0)
#define APP_RESET_HANDLER (APP_ADDRESS + 4)

void JumpToAPP(void)
{
    uint32_t JumpAddress = *(__IO uint32_t*)APP_RESET_HANDLER;
    JumpToApplication = (pFunction)JumpAddress;
    __set_MSP(*(__IO uint32_t*)APP_STACK);
    JumpToApplication();
}

#define RAMAPP_ADDRESS 0x20000800
#define RAMAPP_STACK   (RAMAPP_ADDRESS + 0)
#define RAMAPP_RESET_HANDLER (RAMAPP_ADDRESS + 4)
void JumpToRAMApp(void)
{
    uint32_t JumpAddress = *(__IO uint32_t*)RAMAPP_RESET_HANDLER;
    JumpToApplication = (pFunction)JumpAddress;
    __set_MSP(*(__IO uint32_t*)RAMAPP_STACK);
    JumpToApplication();
}



////NXP func
////*****************************************************************************
//// Functions to carry out the initialization of RW and BSS data sections. These
//// are written as separate functions rather than being inlined within the
//// ResetISR() function in order to cope with MCUs with multiple banks of
//// memory.
////*****************************************************************************
//__attribute__ ((section(".after_vectors.init_data")))
//void data_init(unsigned int romstart, unsigned int start, unsigned int len) {
//    unsigned int *pulDest = (unsigned int*) start;
//    unsigned int *pulSrc = (unsigned int*) romstart;
//    unsigned int loop;
//    for (loop = 0; loop < len; loop = loop + 4)
//        *pulDest++ = *pulSrc++;
//}
//
//__attribute__ ((section(".after_vectors.init_bss")))
//void bss_init(unsigned int start, unsigned int len) {
//    unsigned int *pulDest = (unsigned int*) start;
//    unsigned int loop;
//    for (loop = 0; loop < len; loop = loop + 4)
//        *pulDest++ = 0;
//}
//
//
//extern unsigned int __data_section_table;
//extern unsigned int __data_section_table_end;
//extern unsigned int __bss_section_table;
//extern unsigned int __bss_section_table_end;
//void cpdata() {
//    //
//    // Copy the data sections from flash to SRAM.
//    //
//    unsigned int LoadAddr, ExeAddr, SectionLen;
//    unsigned int *SectionTableAddr;
//
//    // Load base address of Global Section Table
//    SectionTableAddr = &__data_section_table;
//
//    // Copy the data sections from flash to SRAM.
//    while (SectionTableAddr < &__data_section_table_end) {
//        LoadAddr = *SectionTableAddr++;
//        ExeAddr = *SectionTableAddr++;
//        SectionLen = *SectionTableAddr++;
//        data_init(LoadAddr, ExeAddr, SectionLen);
//    }
//
//    // At this point, SectionTableAddr = &__bss_section_table;
//    // Zero fill the bss segment
//    while (SectionTableAddr < &__bss_section_table_end) {
//        ExeAddr = *SectionTableAddr++;
//        SectionLen = *SectionTableAddr++;
//        bss_init(ExeAddr, SectionLen);
//    }
//}
//
////

//example code as to how to do this copy and then jump to start of the application
#define DefROM_BIN_TOP    (0x08002000)
#define DefDST_RAM_TOP    (0x20000800)

__attribute__((always_inline)) static inline void JumpApplication(uint32_t topOfMainStack, uint32_t AppliAddr)
{
	volatile uint32_t test = AppliAddr;
 __ASM volatile ("mov r13, %0" : : "r" (topOfMainStack) : );
 __ASM volatile ("mov r15, %0" : : "r" (AppliAddr) : );
}

_Bool ExampleCheckROM(uint32_t u32BinSize)
{
	memset((void*)DefDST_RAM_TOP, 0, u32BinSize);

    memcpy((void*)DefDST_RAM_TOP, (void*)(DefROM_BIN_TOP), u32BinSize);

 /** Verify */
    if(memcmp((void*)(DefROM_BIN_TOP), (void*)DefDST_RAM_TOP, u32BinSize) == 0) {
//        printf("memcpy OK\r\n");
//        vTaskDelay(100u);

//        NVIC_DisableIRQ();
//        ARM_MPU_Disable();
//        SCB_DisableDCache();
//        SCB_DisableICache();


    	JumpToRAMApp();
/** Set Stack Pointer and PC */
        JumpApplication(*(uint32_t*)DefDST_RAM_TOP, *(uint32_t*)(DefDST_RAM_TOP + 4));
    }

    return 1;
}

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
//  MX_GPIO_Init();
//  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  	//8k Byte = 8Byte * 1024 = 8192 byte
	ExampleCheckROM(8192);

//	memcpy((void*)DefDST_RAM_TOP, (void*)(DefROM_BIN_TOP), 1280);
//	RAM_JumpToAPP();
	JumpToAPP();


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
