/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for BNO085 test using I2C
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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bno085.h"
#include <stdio.h>
#include <string.h>
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
static BNO085_t bno085;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Retarget printf ke USART2 (PA2 TX)
 */
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  printf("\r\n========================================\r\n");
  printf("     BNO085 I2C TEST (STM32F103C8)     \r\n");
  printf("========================================\r\n");
  printf("I2C1: SCL=PB6, SDA=PB7 | INT=PB0, RST=PB1\r\n");
  printf("UART2: 115200 baud (PA2=TX)\r\n\r\n");

  /* 1. Inisialisasi handle driver BNO085 (0 = auto-detect alamat I2C 0x4A/0x4B) */
  printf("[1] Inisialisasi BNO085...\r\n");
  if (BNO085_Init(&bno085, &hi2c1, 0, BNO_RST_GPIO_Port, BNO_RST_Pin, BNO_INT_GPIO_Port, BNO_INT_Pin) != BNO085_OK) {
      printf("[ERROR] Inisialisasi parameter gagal!\r\n");
  } else {
      printf("[OK] Alamat I2C terdeteksi: 0x%02X (8-bit: 0x%02X)\r\n", bno085.i2c_addr >> 1, bno085.i2c_addr);
  }

  /* 2. Hardware Reset sensor */
  printf("[2] Melakukan Hardware Reset sensor...\r\n");
  if (BNO085_HardwareReset(&bno085, 1000) != BNO085_OK) {
      printf("[WARN] Hardware reset timeout atau sinyal HINTN (PB0) belum LOW.\r\n");
      printf("       Pastikan kabel INT ke PB0, RST ke PB1, serta pull-up SDA/SCL terpasang.\r\n");
  } else {
      printf("[OK] BNO085 berhasil di-reset dan boot selesai.\r\n");
  }

  /* 3. Aktifkan report sensor (50 Hz = interval 20 ms) */
  printf("[3] Mengaktifkan Sensor Reports...\r\n");

  /* Game Rotation Vector (6-Axis: Accel + Gyro, tidak terganggu medan magnet) */
  BNO085_EnableGameRotationVector(&bno085, 20);
  HAL_Delay(50);

  /* Linear / Calibrated Accelerometer */
  BNO085_EnableAccelerometer(&bno085, 20);
  HAL_Delay(50);

  /* Calibrated Gyroscope */
  BNO085_EnableGyroscope(&bno085, 20);
  HAL_Delay(50);

  printf("[OK] Konfigurasi selesai. Memulai pembacaan sensor...\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 1. Update & proses paket SHTP dari BNO085 jika data siap (HINTN LOW) */
    BNO085_Update(&bno085);

    /* 2. Cetak data telemetri secara periodik ke serial monitor (10 Hz / setiap 100 ms) */
    static uint32_t last_print = 0;
    if ((HAL_GetTick() - last_print) >= 100) {
        last_print = HAL_GetTick();

        if (bno085.data.has_new_data) {
            printf("[IMU] Yaw:%+6.1f Pitch:%+6.1f Roll:%+6.1f deg | Q:[%+5.2f %+5.2f %+5.2f %+5.2f] | Acc:[%+5.2f %+5.2f %+5.2f] | Accu:%u\r\n",
                   bno085.data.yaw, bno085.data.pitch, bno085.data.roll,
                   bno085.data.q_r, bno085.data.q_i, bno085.data.q_j, bno085.data.q_k,
                   bno085.data.accel_x, bno085.data.accel_y, bno085.data.accel_z,
                   bno085.data.accuracy);
            bno085.data.has_new_data = false;
        }
    }
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
