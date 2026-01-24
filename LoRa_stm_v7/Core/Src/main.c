/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (clean RX model)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "LoRa.h"
#include <stdint.h>
#include <ctype.h>
#include <stdlib.h>
#include "stm32f1xx.h"
#include "stm32f1xx_hal_flash.h"
#include "stm32f1xx_hal_flash_ex.h"
#include "Mesh.h"
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
LoRa myLoRa;
uint16_t LoRa_status;
uint32_t uid[3];
uint8_t Master = 1;
int nodeId = 0;
uint8_t rxBuf[256];
uint8_t txBuf[256];
char payloadBuf[128];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef USE_UART
	void printu(const char *msg);
	void print_dbg(const char *format, ...);
	void print_node_id(uint8_t nodeId);
	void print_reset_reason(void);
#endif
void STM32_GetUID(uint32_t uid[3]);
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
  HAL_Init();

  lcg_seed = HAL_GetTick();

  /* USER CODE BEGIN Init */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

    /* ---------- SX127x RESET ---------- */
    Mesh_hard_reset_radio();
    HAL_Delay(100);

    /* ---------- INIT LoRa STRUCT ---------- */
    myLoRa = newLoRa();
    myLoRa.CS_port    = NSS_GPIO_Port;
    myLoRa.CS_pin     = NSS_Pin;
    myLoRa.reset_port = RESET_GPIO_Port;
    myLoRa.reset_pin  = RESET_Pin;
    myLoRa.DIO0_port  = DIO0_GPIO_Port;
    myLoRa.DIO0_pin   = DIO0_Pin;
    myLoRa.hSPIx      = &hspi1;

    LoRa_status = LoRa_init(&myLoRa);

    if (LoRa_status != LORA_OK)
    {
      Mesh_hard_reset_radio();
      HAL_Delay(100);

      LoRa_status = LoRa_init(&myLoRa);
      if (LoRa_status != LORA_OK) {
      }
    }

    // Configure LoRa parameters to match ESP32 master
    LoRa_setSyncWord(&myLoRa, 0x34);
    LoRa_setSpreadingFactor(&myLoRa, 7);
    LoRa_enableCRC(&myLoRa, 1);

    /* ---------- SAFE FIFO SETUP ---------- */
    LoRa_write(&myLoRa, RegFiFoRxBaseAddr, 0x00);
    LoRa_write(&myLoRa, RegFiFoTxBaseAddr, 0x80);

    /* ---------- START RX (single start) ---------- */
    LoRa_startReceiving(&myLoRa);

    STM32_GetUID(uid);
    Mesh_reset_watchdog();
    lastResetTime = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
      Mesh_check_watchdog();

      if (nodeId == 0)
      {
          Mesh_request_registration(&myLoRa,uid,&nodeId,payloadBuf,sizeof(payloadBuf),txBuf,sizeof(txBuf),rxBuf,sizeof(rxBuf));
      }
      else
      {
          Mesh_poll(&myLoRa,nodeId,rxBuf,sizeof(rxBuf),payloadBuf,sizeof(payloadBuf));
      }

      Mesh_check_radio_health(&myLoRa);
      HAL_Delay(100);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
#ifdef USE_UART
void printu(const char *msg)
{
  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 200);
}

void print_dbg(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int len = vsnprintf(dbg_buf, DBG_BUF_SIZE, format, args);
    va_end(args);

    if (len > 0) {
        printu(dbg_buf);
    }
}

void print_node_id(uint8_t nodeId)
{
    char buf[4];   // only 4 bytes on stack
    buf[0] = '0' + (nodeId / 10);
    buf[1] = '0' + (nodeId % 10);
    buf[2] = '\r';
    buf[3] = '\n';

    printu("Node ID: ");
    printu(buf);
}

void print_reset_reason(void)
{
    uint32_t flags = RCC->CSR;
    if(flags & RCC_CSR_LPWRRSTF) {
        printu("Reset Reason: Low Power (Brown-out)\r\n");
    } else if(flags & RCC_CSR_WWDGRSTF) {
        printu("Reset Reason: Window Watchdog\r\n");
    } else if(flags & RCC_CSR_IWDGRSTF) {
        printu("Reset Reason: Independent Watchdog\r\n");
    } else if(flags & RCC_CSR_SFTRSTF) {
        printu("Reset Reason: Software Reset\r\n");
    } else if(flags & RCC_CSR_PORRSTF) {
        printu("Reset Reason: Power-on Reset\r\n");
    } else if(flags & RCC_CSR_PINRSTF) {
        printu("Reset Reason: External Pin Reset\r\n");
    } else {
        printu("Reset Reason: Unknown\r\n");
    }
    RCC->CSR |= RCC_CSR_RMVF;
}
#endif

void STM32_GetUID(uint32_t uid_out[3])
{
    uid_out[0] = *(uint32_t*)0x1FFFF7E8;
    uid_out[1] = *(uint32_t*)0x1FFFF7EC;
    uid_out[2] = *(uint32_t*)0x1FFFF7F0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
