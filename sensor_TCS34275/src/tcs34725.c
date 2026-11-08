/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tcs34725.c
  * @brief   Source file for TCS34725 / CJMCU-34725 color sensor driver.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "tcs34725.h"
#include <math.h>

static HAL_StatusTypeDef write8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { (uint8_t)(TCS34725_CMD | reg), val };
    return HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, buf, 2, 100);
}

static HAL_StatusTypeDef read8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val)
{
    uint8_t cmd = TCS34725_CMD | reg;
    HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, &cmd, 1, 100);
    if (st != HAL_OK) return st;
    return HAL_I2C_Master_Receive(hi2c, TCS34725_ADDR, val, 1, 100);
}

/**
 * @brief Inisialisasi sensor TCS34725
 */
HAL_StatusTypeDef TCS34725_Init(I2C_HandleTypeDef *hi2c, uint8_t atime, uint8_t gain)
{
    uint8_t id = 0;

    /* 1. Baca chip ID: 0x44 (TCS34725/1) atau 0x4D (TCS34727/3) */
    if (read8(hi2c, TCS34725_ID, &id) != HAL_OK) return HAL_ERROR;
    if (id != 0x44 && id != 0x4D) return HAL_ERROR;

    /* 2. Set waktu integrasi (ATIME) dan Gain (CONTROL) */
    if (write8(hi2c, TCS34725_ATIME, atime) != HAL_OK) return HAL_ERROR;
    if (write8(hi2c, TCS34725_CONTROL, gain) != HAL_OK) return HAL_ERROR;

    /* 3. Aktifkan Power (PON), tunggu minimal 2.4 ms */
    if (write8(hi2c, TCS34725_ENABLE, TCS34725_ENABLE_PON) != HAL_OK) return HAL_ERROR;
    HAL_Delay(3);

    /* 4. Aktifkan Power + RGBC ADC (PON | AEN) */
    if (write8(hi2c, TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief Kontrol LED putih pada modul (pin PB0)
 */
void TCS34725_SetLED(GPIO_PinState state)
{
    HAL_GPIO_WritePin(TCS_LED_GPIO_Port, TCS_LED_Pin, state);
}

/**
 * @brief Membaca data RAW 16-bit RGBC
 */
HAL_StatusTypeDef TCS34725_ReadRaw(I2C_HandleTypeDef *hi2c, TCS34725_RawData *data)
{
    uint8_t status = 0;
    uint8_t cmd = TCS34725_CMD_AUTO | TCS34725_CDATAL;
    uint8_t buf[8];

    /* Cek status data valid */
    if (read8(hi2c, TCS34725_STATUS, &status) != HAL_OK) return HAL_ERROR;
    if (!(status & TCS34725_STATUS_AVALID)) return HAL_BUSY;

    /* Baca 8 byte (Clear, Red, Green, Blue low & high byte) */
    if (HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, &cmd, 1, 100) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Receive(hi2c, TCS34725_ADDR, buf, 8, 100) != HAL_OK) return HAL_ERROR;

    data->c = (uint16_t)(buf[1] << 8 | buf[0]);
    data->r = (uint16_t)(buf[3] << 8 | buf[2]);
    data->g = (uint16_t)(buf[5] << 8 | buf[4]);
    data->b = (uint16_t)(buf[7] << 8 | buf[6]);

    return HAL_OK;
}

/**
 * @brief Menghitung estimasi iluminansi (Lux)
 */
float TCS34725_CalculateLux(uint16_t r, uint16_t g, uint16_t b)
{
    /* Formula empiris TAOS / Adafruit untuk TCS34725 */
    float lux = (-0.32466f * (float)r) + (1.57837f * (float)g) + (-0.73191f * (float)b);
    if (lux < 0.0f) lux = 0.0f;
    return lux;
}

/**
 * @brief Menghitung perkiraan Suhu Warna (CCT Kelvin) dengan McCamy's Formula
 */
float TCS34725_CalculateCCT(uint16_t r, uint16_t g, uint16_t b)
{
    /* 1. Konversi RGB ke Tristimulus XYZ */
    float X = (-0.14282f * (float)r) + (1.54924f * (float)g) + (-0.95641f * (float)b);
    float Y = (-0.32466f * (float)r) + (1.57837f * (float)g) + (-0.73191f * (float)b);
    float Z = (-0.68202f * (float)r) + (0.77073f * (float)g) + (0.56332f * (float)b);

    float sum = X + Y + Z;
    if (sum <= 0.0001f) return 0.0f;

    /* 2. Hitung koordinat kromatisitas (x, y) */
    float xc = X / sum;
    float yc = Y / sum;

    /* 3. Formula McCamy */
    float denom = 0.1858f - yc;
    if (fabsf(denom) < 0.00001f) return 0.0f;

    float n = (xc - 0.3320f) / denom;
    float cct = (449.0f * n * n * n) + (3525.0f * n * n) + (6823.3f * n) + 5520.33f;

    if (cct < 1000.0f) cct = 1000.0f;
    if (cct > 40000.0f) cct = 40000.0f;

    return cct;
}

/**
 * @brief Algoritma klasifikasi warna objek
 */
const char* TCS34725_ClassifyColor(const TCS34725_RawData *raw, float r, float g, float b)
{
    /* 1. Kondisi gelap / objek tidak ada atau terlalu jauh */
    if (raw->c < 45) {
        return "HITAM / GELAP";
    }

    /* 2. Kondisi saturasi */
    if (raw->c >= 21500) {
        return "SATURASI (TERLALU TERANG)";
    }

    /* 3. Putih: ketiga kanal tinggi dan seimbang */
    if (r > 70.0f && g > 70.0f && b > 70.0f) {
        float max_val = fmaxf(r, fmaxf(g, b));
        float min_val = fminf(r, fminf(g, b));
        if ((max_val - min_val) < 28.0f && raw->c > 350) {
            return "PUTIH (WHITE)";
        }
    }

    /* 4. Kuning: Merah & Hijau tinggi, Biru rendah */
    if (r > 80.0f && g > 75.0f && b < 60.0f && fabsf(r - g) < 35.0f) {
        return "KUNING (YELLOW)";
    }

    /* 5. Cyan: Hijau & Biru tinggi, Merah rendah */
    if (g > 80.0f && b > 80.0f && r < 60.0f) {
        return "CYAN";
    }

    /* 6. Magenta / Ungu: Merah & Biru tinggi, Hijau rendah */
    if (r > 80.0f && b > 80.0f && g < 60.0f) {
        return "MAGENTA / UNGU";
    }

    /* 7. Warna Dasar Utama */
    if (r > g && r > b) {
        if (r > (g + 20.0f) && r > (b + 20.0f)) {
            return "MERAH (RED)";
        }
        return "MERAH (DOMINAN)";
    }
    else if (g > r && g > b) {
        if (g > (r + 15.0f) && g > (b + 15.0f)) {
            return "HIJAU (GREEN)";
        }
        return "HIJAU (DOMINAN)";
    }
    else if (b > r && b > g) {
        if (b > (r + 15.0f) && b > (g + 15.0f)) {
            return "BIRU (BLUE)";
        }
        return "BIRU (DOMINAN)";
    }

    return "TERDETEKSI";
}

/**
 * @brief Memproses data warna lengkap (Normalisasi 0-255, Lux, CCT, Klasifikasi)
 */
void TCS34725_ProcessColor(const TCS34725_RawData *raw, TCS34725_ColorInfo *info)
{
    if (raw->c == 0) {
        info->r = 0.0f;
        info->g = 0.0f;
        info->b = 0.0f;
        info->lux = 0.0f;
        info->cct = 0.0f;
        info->color_name = "GELAP / OFF";
        return;
    }

    /* Normalisasi RGB ke skala 0 - 255 terhadap Clear */
    info->r = ((float)raw->r / (float)raw->c) * 255.0f;
    info->g = ((float)raw->g / (float)raw->c) * 255.0f;
    info->b = ((float)raw->b / (float)raw->c) * 255.0f;

    /* Batasi nilai ke 255 */
    if (info->r > 255.0f) info->r = 255.0f;
    if (info->g > 255.0f) info->g = 255.0f;
    if (info->b > 255.0f) info->b = 255.0f;

    /* Hitung Lux dan CCT */
    info->lux = TCS34725_CalculateLux(raw->r, raw->g, raw->b);
    info->cct = TCS34725_CalculateCCT(raw->r, raw->g, raw->b);

    /* Tentukan klasifikasi warna */
    info->color_name = TCS34725_ClassifyColor(raw, info->r, info->g, info->b);
}

