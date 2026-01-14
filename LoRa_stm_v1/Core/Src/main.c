/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32F103 + SX127x LoRa Transmitter
  ******************************************************************************
  */
 /* USER CODE END Header */

#include "main.h"
#include "LoRa.h"
#include <stdio.h>
#include <string.h>

/* ===== EXTERN PERIPHERAL HANDLES (FROM CubeMX) ===== */
extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart1;

/* ===== USER VARIABLES ===== */
LoRa myLoRa;
uint16_t LoRa_status;

/* ===== FUNCTION PROTOTYPES ===== */
void SystemClock_Config(void);
void printu(const char *msg);

/* ===== UART PRINT FUNCTION ===== */
void printu(const char *msg)
{
  HAL_UART_Transmit(
      &huart1,
      (uint8_t*)msg,
      strlen(msg),
      HAL_MAX_DELAY
  );
}

/* ===== MAIN ===== */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* CubeMX init functions */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  printu("\r\n===============================\r\n");
  printu(" STM32F103 LoRa TRANSMITTER\r\n");
  printu("===============================\r\n");

  /* ---------- SX127x RESET ---------- */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  /* ---------- NSS HIGH (IDLE) ---------- */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);

  /* ---------- INIT LoRa STRUCT ---------- */
  myLoRa = newLoRa();
  myLoRa.CS_port    = NSS_GPIO_Port;
  myLoRa.CS_pin     = NSS_Pin;
  myLoRa.reset_port = RESET_GPIO_Port;
  myLoRa.reset_pin  = RESET_Pin;
  myLoRa.DIO0_port  = DIO0_GPIO_Port;
  myLoRa.DIO0_pin   = DIO0_Pin;
  myLoRa.hSPIx      = &hspi1;

  printu("Initializing LoRa...\r\n");
  LoRa_status = LoRa_init(&myLoRa);

  if (LoRa_status != LORA_OK)
  {
    char err[64];
    sprintf(err, "LoRa INIT FAILED (%d)\r\n", LoRa_status);
    printu(err);

    while (1)
    {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
      HAL_Delay(200);
    }
  }

  printu("LoRa INIT SUCCESS ✅\r\n");
  printu("Frequency: 433 MHz\r\n");
  printu("SF: 7 | BW: 125 kHz\r\n\r\n");

  /* ---------- TRANSMISSION LOOP ---------- */
  int counter = 0;

  while (1)
  {
    char payload[64];
    sprintf(payload, "Hello from STM32 | #%d", ++counter);

    printu("TX -> ");
    printu(payload);
    printu("\r\n");

    uint16_t tx = LoRa_transmit(
                    &myLoRa,
                    (uint8_t*)payload,
                    strlen(payload),
                    200
                  );

    if (tx == 1)
      printu("TX OK\r\n\r\n");
    else
    {
      char e[40];
      sprintf(e, "TX FAIL (%d)\r\n\r\n", tx);
      printu(e);
    }

    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2); // heartbeat LED
    HAL_Delay(5000);
  }
}

/* ===== SYSTEM CLOCK ===== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK |
      RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 |
      RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    Error_Handler();
}

/* ===== ERROR HANDLER ===== */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
    HAL_Delay(100);
  }
}
