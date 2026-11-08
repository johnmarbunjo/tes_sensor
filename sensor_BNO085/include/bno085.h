#ifndef __BNO085_H
#define __BNO085_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Default BNO085 I2C Addresses (7-bit left-shifted by 1 for STM32 HAL)
 *        - SA0 LOW (GND)  -> 0x4A (0x94 in 8-bit HAL format)
 *        - SA0 HIGH (VDD) -> 0x4B (0x96 in 8-bit HAL format)
 */
#define BNO085_I2C_ADDR_DEF     (0x4A << 1)
#define BNO085_I2C_ADDR_ALT     (0x4B << 1)

/**
 * @brief Driver status & error codes
 */
typedef enum {
    BNO085_OK             = 0,
    BNO085_NO_DATA        = 1,   /* HINTN pin is HIGH (no data available) */
    BNO085_ERR_TIMEOUT    = -1,  /* Timeout waiting for HINTN / hardware */
    BNO085_ERR_I2C        = -2,  /* HAL I2C transfer error */
    BNO085_ERR_HEADER     = -3,  /* Corrupt or invalid SHTP header */
    BNO085_ERR_PARAM      = -4   /* Invalid function argument */
} BNO085_Status_t;

/* SHTP Logic Channels */
#define SHTP_CHAN_COMMAND       0
#define SHTP_CHAN_CONTROL       1
#define SHTP_CHAN_SH2_CONFIG    2
#define SHTP_CHAN_SH2_INPUT     3

/* SHTP Command & Report IDs */
#define SHTP_REPORT_COMMAND_RESPONSE     0xF1
#define SHTP_REPORT_COMMAND_REQUEST      0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE    0xF3
#define SHTP_REPORT_FRS_READ_REQUEST     0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE  0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST   0xF9
#define SHTP_REPORT_TIMESTAMP_BASE       0xFB
#define SHTP_REPORT_SET_FEATURE          0xFD

/* Sensor Feature Report IDs */
#define SHTP_REPORT_ACCELEROMETER           0x01
#define SHTP_REPORT_GYROSCOPE_CALIBRATED    0x02
#define SHTP_REPORT_ROTATION_VECTOR         0x05
#define SHTP_REPORT_GAME_ROTATION_VECTOR    0x08
#define SHTP_REPORT_ARVR_STABILIZED_GRV     0x28

/**
 * @brief Measurement Data Structure
 */
typedef struct {
    /* Quaternions (Normalized: -1.0f to +1.0f) */
    float q_r;  /* Real (W) */
    float q_i;  /* X */
    float q_j;  /* Y */
    float q_k;  /* Z */

    /* Euler angles in degrees (Derived from quaternion) */
    float yaw;
    float pitch;
    float roll;

    /* Accelerometer in m/s^2 */
    float accel_x;
    float accel_y;
    float accel_z;

    /* Calibrated Gyroscope in rad/s */
    float gyro_x;
    float gyro_y;
    float gyro_z;

    uint8_t  accuracy;           /* 0: Unreliable, 1: Low, 2: Medium, 3: High */
    uint8_t  sequence;           /* SHTP sensor report sequence number */
    uint32_t last_update_tick;   /* HAL_GetTick() when data was received */
    bool     has_new_data;       /* Set to true when fresh data arrived */
} BNO085_Data_t;

/**
 * @brief BNO085 Device Instance Handle for I2C
 */
typedef struct {
    I2C_HandleTypeDef   *hi2c;
    uint16_t            i2c_addr;
    GPIO_TypeDef        *rst_port;
    uint16_t            rst_pin;
    GPIO_TypeDef        *hintn_port;
    uint16_t            hintn_pin;

    /* SHTP sequence counters for channels 0..5 */
    uint8_t             sequence_number[6];

    /* Output measurement data */
    BNO085_Data_t       data;
} BNO085_t;

/**
 * @brief SHTP 4-Byte Header
 */
typedef struct {
    uint16_t length;
    uint8_t  channel;
    uint8_t  sequence;
    bool     has_continuation;
} SHTP_Header_t;

/* Public Driver API */

/**
 * @brief Initialize device handle and verify I2C communication
 * @param i2c_addr 8-bit I2C address (e.g. BNO085_I2C_ADDR_DEF) or 0 for auto-detect
 */
BNO085_Status_t BNO085_Init(BNO085_t *dev, I2C_HandleTypeDef *hi2c, uint16_t i2c_addr,
                            GPIO_TypeDef *rst_port,   uint16_t rst_pin,
                            GPIO_TypeDef *hintn_port, uint16_t hintn_pin);

/**
 * @brief Pulse hardware reset and wait for initial boot packet
 */
BNO085_Status_t BNO085_HardwareReset(BNO085_t *dev, uint32_t timeout_ms);

/**
 * @brief Non-blocking check if sensor has asserted HINTN (data ready)
 */
bool BNO085_HasData(BNO085_t *dev);

/**
 * @brief Configure and enable a specific feature report
 * @param report_id SHTP report ID (e.g. SHTP_REPORT_GAME_ROTATION_VECTOR)
 * @param report_interval_ms Update period in milliseconds (e.g., 20 = 50 Hz, 10 = 100 Hz)
 */
BNO085_Status_t BNO085_SetFeature(BNO085_t *dev, uint8_t report_id, uint16_t report_interval_ms);

/**
 * @brief Enable Game Rotation Vector at a specified report interval
 *        (6-axis gyro+accel fusion, immune to magnetic distortion)
 */
BNO085_Status_t BNO085_EnableGameRotationVector(BNO085_t *dev, uint16_t report_interval_ms);

/**
 * @brief Enable Absolute Rotation Vector at a specified report interval
 *        (9-axis gyro+accel+mag fusion, absolute orientation referenced to North)
 */
BNO085_Status_t BNO085_EnableRotationVector(BNO085_t *dev, uint16_t report_interval_ms);

/**
 * @brief Enable Linear / Calibrated Accelerometer
 */
BNO085_Status_t BNO085_EnableAccelerometer(BNO085_t *dev, uint16_t report_interval_ms);

/**
 * @brief Enable Calibrated Gyroscope
 */
BNO085_Status_t BNO085_EnableGyroscope(BNO085_t *dev, uint16_t report_interval_ms);

/**
 * @brief Non-blocking poll & update function.
 *        Call periodically in control loop or from EXTI ISR context.
 * @return BNO085_OK if packet was read and parsed,
 *         BNO085_NO_DATA if no packet was waiting (0 delay),
 *         or an error code.
 */
BNO085_Status_t BNO085_Update(BNO085_t *dev);

/**
 * @brief Send an SHTP packet to the sensor over I2C
 */
BNO085_Status_t BNO085_SendPacket(BNO085_t *dev, uint8_t channel, const uint8_t *payload, uint16_t payload_len);

/**
 * @brief Read a raw SHTP packet from the sensor over I2C
 */
uint16_t BNO085_ReadPacket(BNO085_t *dev, uint8_t *buffer, uint16_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* __BNO085_H */

