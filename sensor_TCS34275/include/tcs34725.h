/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tcs34725.h
  * @brief   Header file for TCS34725 / CJMCU-34725 color sensor driver.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef TCS34725_H
#define TCS34725_H

#include "main.h"
#include <stdbool.h>

/* I2C Address (7-bit 0x29 -> 8-bit for HAL: 0x52) */
#define TCS34725_ADDR               (0x29 << 1)

/* Command bits */
#define TCS34725_CMD                0x80
#define TCS34725_CMD_AUTO           0xA0

/* Registers */
#define TCS34725_ENABLE             0x00
#define TCS34725_ATIME              0x01
#define TCS34725_WTIME              0x03
#define TCS34725_AILTL              0x04
#define TCS34725_AILTH              0x05
#define TCS34725_AIHTL              0x06
#define TCS34725_AIHTH              0x07
#define TCS34725_PERS               0x0C
#define TCS34725_CONFIG             0x0D
#define TCS34725_CONTROL            0x0F
#define TCS34725_ID                 0x12
#define TCS34725_STATUS             0x13
#define TCS34725_CDATAL             0x14
#define TCS34725_CDATAH             0x15
#define TCS34725_RDATAL             0x16
#define TCS34725_RDATAH             0x17
#define TCS34725_GDATAL             0x18
#define TCS34725_GDATAH             0x19
#define TCS34725_BDATAL             0x1A
#define TCS34725_BDATAH             0x1B

/* ENABLE Register Bits */
#define TCS34725_ENABLE_PON         0x01  /* Power ON */
#define TCS34725_ENABLE_AEN         0x02  /* RGBC ADC Enable */
#define TCS34725_ENABLE_WEN         0x08  /* Wait Enable */
#define TCS34725_ENABLE_AIEN        0x10  /* RGBC Interrupt Enable */

/* STATUS Register Bits */
#define TCS34725_STATUS_AVALID      0x01  /* RGBC Valid */

/* Gain settings (CONTROL Register) */
#define TCS34725_GAIN_1X            0x00
#define TCS34725_GAIN_4X            0x01
#define TCS34725_GAIN_16X           0x02
#define TCS34725_GAIN_60X           0x03

/* Integration time presets (ATIME) */
#define TCS34725_INTEGRATION_2_4MS  0xFF  /* 1 cycle  - max 1024  */
#define TCS34725_INTEGRATION_24MS   0xF6  /* 10 cycles - max 10240 */
#define TCS34725_INTEGRATION_50MS   0xEB  /* 20 cycles - max 20480 */
#define TCS34725_INTEGRATION_101MS  0xD5  /* 42 cycles - max 43008 */
#define TCS34725_INTEGRATION_154MS  0xC0  /* 64 cycles - max 65535 */
#define TCS34725_INTEGRATION_700MS  0x00  /* 256 cycles - max 65535 */

/* Raw RGBC 16-bit data */
typedef struct {
    uint16_t c;
    uint16_t r;
    uint16_t g;
    uint16_t b;
} TCS34725_RawData;

/* Processed color information */
typedef struct {
    float r;                /* Normalized Red (0 - 255) */
    float g;                /* Normalized Green (0 - 255) */
    float b;                /* Normalized Blue (0 - 255) */
    float lux;              /* Estimated illuminance in Lux */
    float cct;              /* Correlated Color Temperature in Kelvin */
    const char *color_name; /* Detected color label */
} TCS34725_ColorInfo;

/* Driver Functions */
HAL_StatusTypeDef TCS34725_Init(I2C_HandleTypeDef *hi2c, uint8_t atime, uint8_t gain);
HAL_StatusTypeDef TCS34725_ReadRaw(I2C_HandleTypeDef *hi2c, TCS34725_RawData *data);
void TCS34725_SetLED(GPIO_PinState state);
void TCS34725_ProcessColor(const TCS34725_RawData *raw, TCS34725_ColorInfo *info);

float TCS34725_CalculateLux(uint16_t r, uint16_t g, uint16_t b);
float TCS34725_CalculateCCT(uint16_t r, uint16_t g, uint16_t b);
const char* TCS34725_ClassifyColor(const TCS34725_RawData *raw, float r, float g, float b);

#endif /* TCS34725_H */

