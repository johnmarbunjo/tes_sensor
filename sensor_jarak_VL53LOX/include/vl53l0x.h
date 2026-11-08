/*
 * vl53l0x.h - Driver VL53L0X untuk STM32 HAL (I2C polling)
 *
 * Port ke C/HAL dari library Pololu "vl53l0x-arduino"
 * (https://github.com/pololu/vl53l0x-arduino), yang berbasis ST VL53L0X API.
 * Lisensi mengikuti library asli (MIT).
 *
 * Catatan: perhitungan timing budget (get/setMeasurementTimingBudget) tidak
 * dipindahkan. Sensor memakai pengaturan waktu pengukuran bawaan.
 */
#ifndef VL53L0X_H
#define VL53L0X_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* Nilai kembalian saat timeout */
#define VL53L0X_TIMEOUT_VALUE 65535U

/* Inisialisasi sensor. hi2c = handle I2C (mis. &hi2c1).
 * Mengembalikan false jika sensor tidak merespons / kalibrasi gagal. */
bool VL53L0X_Init(I2C_HandleTypeDef *hi2c);

/* Batas waktu tunggu operasi (ms). 0 = tanpa batas. Default 500 ms. */
void VL53L0X_SetTimeout(uint16_t timeout_ms);

/* Mode continuous. period_ms = 0 -> back-to-back (secepat mungkin). */
void VL53L0X_StartContinuous(uint32_t period_ms);
void VL53L0X_StopContinuous(void);

/* Baca jarak (mm) saat mode continuous aktif. */
uint16_t VL53L0X_ReadRangeContinuousMM(void);

/* Satu kali pengukuran (single-shot), mengembalikan jarak dalam mm. */
uint16_t VL53L0X_ReadRangeSingleMM(void);

/* true jika terjadi timeout sejak pemanggilan terakhir fungsi ini. */
bool VL53L0X_TimeoutOccurred(void);

#endif /* VL53L0X_H */