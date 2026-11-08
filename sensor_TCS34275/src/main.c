/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Program utama pengujian sensor warna TCS34725
  *                   menggunakan STM32F103C8T6 (I2C2 & USART1).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "tcs34725.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static TCS34725_RawData raw_color;
static TCS34725_ColorInfo color_info;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  Entry point aplikasi.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes Flash interface and Systick */
  HAL_Init();

  /* Konfigurasi System Clock 72 MHz dari HSE 8 MHz */
  SystemClock_Config();

  /* Inisialisasi periferal */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();

  HAL_Delay(100);

  /* Banner Informasi Startup */
  printf("\r\n=======================================================\r\n");
  printf("      PENGUJIAN SENSOR WARNA TCS34725 (CJMCU-34725)    \r\n");
  printf("           STM32F103C8T6 (STM32Cube HAL)              \r\n");
  printf("=======================================================\r\n");
  printf("I2C2   : PB10 (SCL), PB11 (SDA) - Standard Mode 100 kHz\r\n");
  printf("USART2 : PA2 (TX) -> RX USB-TTL, PA3 (RX) - 115200 baud\r\n");
  printf("ST-Link: Header 4-Pin SWD (GND, CLK, DIO, 3.3V)\r\n");
  printf("GPIO   : PB0 (LED Kontrol Sensor), PC13 (Heartbeat LED)\r\n");
  printf("-------------------------------------------------------\r\n");

  /* Matikan LED sensor pada modul melalui pin PB0 */
  TCS34725_SetLED(GPIO_PIN_RESET);
  printf("[INFO] LED iluminasi sensor dimatikan (PB0 LOW)...\r\n");

  /* Inisialisasi sensor TCS34725: ATIME 50ms (0xEB), Gain 4X */
  printf("[INFO] Menghubungi TCS34725 pada bus I2C2...\r\n");
  uint8_t init_attempts = 0;
  while (TCS34725_Init(&hi2c2, TCS34725_INTEGRATION_50MS, TCS34725_GAIN_4X) != HAL_OK)
  {
      init_attempts++;
      printf("[ERROR] Gagal inisialisasi sensor (Percobaan #%u)!\r\n", init_attempts);
      printf("        Pastikan:\r\n");
      printf("        1. Kabel SCL terhubung ke PB10, SDA ke PB11.\r\n");
      printf("        2. Catu daya 3.3V dan GND terhubung dengan baik.\r\n");
      printf("        3. Pasang resistor pull-up 4.7 kOhm jika bus I2C belum memiliki pull-up.\r\n");

      /* Kedipkan LED onboard PC13 menandakan peringatan error */
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(1000);
  }

  printf("[OK] TCS34725 terdeteksi & berhasil diinisialisasi!\r\n");
  printf("[INFO] Konfigurasi: ATIME = 50ms, Gain = 4X, Alamat I2C = 0x29\r\n");
  printf("[INFO] Memulai pembacaan data warna...\r\n\r\n");

  uint32_t last_read_tick = 0;

  /* Infinite loop */
  while (1)
  {
    /* Pembacaan periodik setiap 200 ms */
    if ((HAL_GetTick() - last_read_tick) >= 200)
    {
      last_read_tick = HAL_GetTick();

      HAL_StatusTypeDef status = TCS34725_ReadRaw(&hi2c2, &raw_color);

      if (status == HAL_OK)
      {
        /* Olah data: Normalisasi 0-255, Lux, CCT, dan Klasifikasi Warna */
        TCS34725_ProcessColor(&raw_color, &color_info);

        /* Tampilkan telemetri warna ke Serial Monitor */
        printf("[TCS34725] RAW:[C:%5u R:%5u G:%5u B:%5u] | RGB:[%3.0f,%3.0f,%3.0f] | Lux:%6.1f | CCT:%5.0f K | Warna: %s\r\n",
               raw_color.c, raw_color.r, raw_color.g, raw_color.b,
               color_info.r, color_info.g, color_info.b,
               color_info.lux, color_info.cct,
               color_info.color_name);

        /* Toggle LED PC13 sebagai indikator detak jantung (heartbeat) */
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      }
      else if (status == HAL_BUSY)
      {
        /* Integrasi ADC masih berlangsung, tunggu siklus berikutnya */
      }
      else
      {
        printf("[WARN] Error membaca bus I2C2 sensor TCS34725.\r\n");
      }
    }
  }
}

/**
  * @brief System Clock Configuration
  *        HSE 8 MHz -> PLL 9x -> SYSCLK 72 MHz
  *        APB1 = 36 MHz, APB2 = 72 MHz
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators */
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

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Fungsi penanganan error kritis.
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

