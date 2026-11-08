/*
 * vl53l0x.c - Driver VL53L0X untuk STM32 HAL (I2C polling)
 *
 * Port ke C/HAL dari library Pololu "vl53l0x-arduino" (MIT License),
 * yang berbasis ST VL53L0X API (STSW-IMG005).
 * Blok tuning settings disalin dari sumber asli.
 */
#include "vl53l0x.h"

#define VL53L0X_ADDR (0x29 << 1) /* alamat 7-bit 0x29, digeser untuk HAL */
#define I2C_TIMEOUT_MS 100U

/* Register */
#define SYSRANGE_START                              0x00
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_INTERMEASUREMENT_PERIOD              0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define SYSTEM_INTERRUPT_CLEAR                      0x0B
#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14
#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84
#define MSRC_CONFIG_CONTROL                         0x60
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define IDENTIFICATION_MODEL_ID                     0xC0
#define OSC_CALIBRATE_VAL                           0xF8
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89

static I2C_HandleTypeDef *s_i2c;
static uint8_t  s_stop_variable;
static uint16_t s_io_timeout = 500;
static uint32_t s_timeout_start;
static bool     s_did_timeout;

#define startTimeout()        (s_timeout_start = HAL_GetTick())
#define checkTimeoutExpired() (s_io_timeout > 0 && \
        (uint16_t)(HAL_GetTick() - s_timeout_start) > s_io_timeout)

/* ---------------- Akses register I2C ---------------- */

static void wr8(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(s_i2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                      &value, 1, I2C_TIMEOUT_MS);
}

static void wr16(uint8_t reg, uint16_t value)
{
    uint8_t b[2] = { (uint8_t)(value >> 8), (uint8_t)value };
    HAL_I2C_Mem_Write(s_i2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                      b, 2, I2C_TIMEOUT_MS);
}

static void wr32(uint8_t reg, uint32_t value)
{
    uint8_t b[4] = { (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                     (uint8_t)(value >> 8),  (uint8_t)value };
    HAL_I2C_Mem_Write(s_i2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                      b, 4, I2C_TIMEOUT_MS);
}

static void wrMulti(uint8_t reg, const uint8_t *src, uint8_t count)
{
    HAL_I2C_Mem_Write(s_i2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                      (uint8_t *)src, count, I2C_TIMEOUT_MS);
}

static uint8_t rd8(uint8_t reg)
{
    uint8_t v = 0;
    HAL_I2C_Mem_Read(s_i2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                     &v, 1, I2C_TIMEOUT_MS);
    return v;
}

static uint16_t rd16(uint8_t reg)
{
    uint8_t b[2] = {0, 0};
    HAL_I2C_Mem_Read(s_i2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                     b, 2, I2C_TIMEOUT_MS);
    return ((uint16_t)b[0] << 8) | b[1];
}

static void rdMulti(uint8_t reg, uint8_t *dst, uint8_t count)
{
    HAL_I2C_Mem_Read(s_i2c, VL53L0X_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                     dst, count, I2C_TIMEOUT_MS);
}

/* ---------------- Fungsi internal ---------------- */

/* berdasarkan VL53L0X_get_info_from_device() */
static bool getSpadInfo(uint8_t *count, bool *type_is_aperture)
{
    uint8_t tmp;

    wr8(0x80, 0x01);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);

    wr8(0xFF, 0x06);
    wr8(0x83, rd8(0x83) | 0x04);
    wr8(0xFF, 0x07);
    wr8(0x81, 0x01);

    wr8(0x80, 0x01);

    wr8(0x94, 0x6b);
    wr8(0x83, 0x00);
    startTimeout();
    while (rd8(0x83) == 0x00)
    {
        if (checkTimeoutExpired()) { return false; }
    }
    wr8(0x83, 0x01);
    tmp = rd8(0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    wr8(0x81, 0x00);
    wr8(0xFF, 0x06);
    wr8(0x83, rd8(0x83) & ~0x04);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x01);

    wr8(0xFF, 0x00);
    wr8(0x80, 0x00);

    return true;
}

/* berdasarkan VL53L0X_perform_single_ref_calibration() */
static bool performSingleRefCalibration(uint8_t vhv_init_byte)
{
    wr8(SYSRANGE_START, 0x01 | vhv_init_byte);

    startTimeout();
    while ((rd8(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (checkTimeoutExpired()) { return false; }
    }

    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);
    wr8(SYSRANGE_START, 0x00);

    return true;
}

/* ---------------- API publik ---------------- */

void VL53L0X_SetTimeout(uint16_t timeout_ms)
{
    s_io_timeout = timeout_ms;
}

bool VL53L0X_TimeoutOccurred(void)
{
    bool tmp = s_did_timeout;
    s_did_timeout = false;
    return tmp;
}

/* Urutan berdasarkan VL53L0X_DataInit(), VL53L0X_StaticInit(), dan
 * VL53L0X_PerformRefCalibration(). Mode I/O 2V8 diaktifkan (sama dengan
 * default library Pololu, cocok untuk modul GY-530). */
bool VL53L0X_Init(I2C_HandleTypeDef *hi2c)
{
    s_i2c = hi2c;

    /* periksa Model ID (datasheet: 0xEE) */
    if (rd8(IDENTIFICATION_MODEL_ID) != 0xEE) { return false; }

    /* VL53L0X_DataInit() begin */

    /* sensor memakai I/O 1V8 secara default; pindah ke mode 2V8 */
    wr8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
        rd8(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);

    /* "Set I2C standard mode" */
    wr8(0x88, 0x00);

    wr8(0x80, 0x01);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);
    s_stop_variable = rd8(0x91);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
    wr8(0x80, 0x00);

    /* nonaktifkan pemeriksaan batas SIGNAL_RATE_MSRC (bit 1) dan
     * SIGNAL_RATE_PRE_RANGE (bit 4) */
    wr8(MSRC_CONFIG_CONTROL, rd8(MSRC_CONFIG_CONTROL) | 0x12);

    /* batas signal rate final range = 0.25 MCPS (Q9.7: 0.25 * 128 = 32) */
    wr16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);

    wr8(SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* VL53L0X_DataInit() end */

    /* VL53L0X_StaticInit() begin */

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

    uint8_t ref_spad_map[6];
    rdMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* VL53L0X_set_reference_spads() begin (asumsi nilai NVM valid) */

    wr8(0xFF, 0x01);
    wr8(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    wr8(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    wr8(0xFF, 0x00);
    wr8(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    wrMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* VL53L0X_set_reference_spads() end */

    /* VL53L0X_load_tuning_settings() begin
     * DefaultTuningSettings dari vl53l0x_tuning.h (disalin dari Pololu) */

    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);

    wr8(0xFF, 0x00);
    wr8(0x09, 0x00);
    wr8(0x10, 0x00);
    wr8(0x11, 0x00);

    wr8(0x24, 0x01);
    wr8(0x25, 0xFF);
    wr8(0x75, 0x00);

    wr8(0xFF, 0x01);
    wr8(0x4E, 0x2C);
    wr8(0x48, 0x00);
    wr8(0x30, 0x20);

    wr8(0xFF, 0x00);
    wr8(0x30, 0x09);
    wr8(0x54, 0x00);
    wr8(0x31, 0x04);
    wr8(0x32, 0x03);
    wr8(0x40, 0x83);
    wr8(0x46, 0x25);
    wr8(0x60, 0x00);
    wr8(0x27, 0x00);
    wr8(0x50, 0x06);
    wr8(0x51, 0x00);
    wr8(0x52, 0x96);
    wr8(0x56, 0x08);
    wr8(0x57, 0x30);
    wr8(0x61, 0x00);
    wr8(0x62, 0x00);
    wr8(0x64, 0x00);
    wr8(0x65, 0x00);
    wr8(0x66, 0xA0);

    wr8(0xFF, 0x01);
    wr8(0x22, 0x32);
    wr8(0x47, 0x14);
    wr8(0x49, 0xFF);
    wr8(0x4A, 0x00);

    wr8(0xFF, 0x00);
    wr8(0x7A, 0x0A);
    wr8(0x7B, 0x00);
    wr8(0x78, 0x21);

    wr8(0xFF, 0x01);
    wr8(0x23, 0x34);
    wr8(0x42, 0x00);
    wr8(0x44, 0xFF);
    wr8(0x45, 0x26);
    wr8(0x46, 0x05);
    wr8(0x40, 0x40);
    wr8(0x0E, 0x06);
    wr8(0x20, 0x1A);
    wr8(0x43, 0x40);

    wr8(0xFF, 0x00);
    wr8(0x34, 0x03);
    wr8(0x35, 0x44);

    wr8(0xFF, 0x01);
    wr8(0x31, 0x04);
    wr8(0x4B, 0x09);
    wr8(0x4C, 0x05);
    wr8(0x4D, 0x04);

    wr8(0xFF, 0x00);
    wr8(0x44, 0x00);
    wr8(0x45, 0x20);
    wr8(0x47, 0x08);
    wr8(0x48, 0x28);
    wr8(0x67, 0x00);
    wr8(0x70, 0x04);
    wr8(0x71, 0x01);
    wr8(0x72, 0xFE);
    wr8(0x76, 0x00);
    wr8(0x77, 0x00);

    wr8(0xFF, 0x01);
    wr8(0x0D, 0x01);

    wr8(0xFF, 0x00);
    wr8(0x80, 0x01);
    wr8(0x01, 0xF8);

    wr8(0xFF, 0x01);
    wr8(0x8E, 0x01);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
    wr8(0x80, 0x00);

    /* VL53L0X_load_tuning_settings() end */

    /* "Set interrupt config to new sample ready" */
    wr8(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    wr8(GPIO_HV_MUX_ACTIVE_HIGH, rd8(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    /* "Disable MSRC and TCC by default" */
    wr8(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* VL53L0X_StaticInit() end */

    /* VL53L0X_PerformRefCalibration() begin */

    wr8(SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(0x40)) { return false; }

    wr8(SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(0x00)) { return false; }

    /* "restore the previous Sequence Config" */
    wr8(SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* VL53L0X_PerformRefCalibration() end */

    return true;
}

/* berdasarkan VL53L0X_StartMeasurement() */
void VL53L0X_StartContinuous(uint32_t period_ms)
{
    wr8(0x80, 0x01);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);
    wr8(0x91, s_stop_variable);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
    wr8(0x80, 0x00);

    if (period_ms != 0)
    {
        /* continuous timed mode */
        uint16_t osc_calibrate_val = rd16(OSC_CALIBRATE_VAL);
        if (osc_calibrate_val != 0)
        {
            period_ms *= osc_calibrate_val;
        }
        wr32(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        wr8(SYSRANGE_START, 0x04);
    }
    else
    {
        /* continuous back-to-back mode */
        wr8(SYSRANGE_START, 0x02);
    }
}

/* berdasarkan VL53L0X_StopMeasurement() */
void VL53L0X_StopContinuous(void)
{
    wr8(SYSRANGE_START, 0x01);

    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);
    wr8(0x91, 0x00);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
}

uint16_t VL53L0X_ReadRangeContinuousMM(void)
{
    startTimeout();
    while ((rd8(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    {
        if (checkTimeoutExpired())
        {
            s_did_timeout = true;
            return VL53L0X_TIMEOUT_VALUE;
        }
    }

    /* asumsi: Linearity Corrective Gain = 1000 (default),
     * fractional ranging tidak aktif */
    uint16_t range = rd16(RESULT_RANGE_STATUS + 10);

    wr8(SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}

/* berdasarkan VL53L0X_PerformSingleRangingMeasurement() */
uint16_t VL53L0X_ReadRangeSingleMM(void)
{
    wr8(0x80, 0x01);
    wr8(0xFF, 0x01);
    wr8(0x00, 0x00);
    wr8(0x91, s_stop_variable);
    wr8(0x00, 0x01);
    wr8(0xFF, 0x00);
    wr8(0x80, 0x00);

    wr8(SYSRANGE_START, 0x01);

    /* "Wait until start bit has been cleared" */
    startTimeout();
    while (rd8(SYSRANGE_START) & 0x01)
    {
        if (checkTimeoutExpired())
        {
            s_did_timeout = true;
            return VL53L0X_TIMEOUT_VALUE;
        }
    }

    return VL53L0X_ReadRangeContinuousMM();
}