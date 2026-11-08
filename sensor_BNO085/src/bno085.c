#include "bno085.h"
#include <math.h>
#include <string.h>

BNO085_Status_t BNO085_Init(BNO085_t *dev, I2C_HandleTypeDef *hi2c, uint16_t i2c_addr,
                            GPIO_TypeDef *rst_port,   uint16_t rst_pin,
                            GPIO_TypeDef *hintn_port, uint16_t hintn_pin) {
    if (!dev || !hi2c || !rst_port || !hintn_port) {
        return BNO085_ERR_PARAM;
    }

    dev->hi2c       = hi2c;
    dev->rst_port   = rst_port;
    dev->rst_pin    = rst_pin;
    dev->hintn_port = hintn_port;
    dev->hintn_pin  = hintn_pin;

    memset(&dev->data, 0, sizeof(dev->data));
    memset(dev->sequence_number, 0, sizeof(dev->sequence_number));

    /* Initialize control line states: RST high (inactive / run mode) */
    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);

    /* Determine I2C target address */
    if (i2c_addr != 0) {
        dev->i2c_addr = i2c_addr;
    } else {
        /* Auto-probe default address 0x4A, then 0x4B */
        if (HAL_I2C_IsDeviceReady(dev->hi2c, BNO085_I2C_ADDR_DEF, 2, 50) == HAL_OK) {
            dev->i2c_addr = BNO085_I2C_ADDR_DEF;
        } else if (HAL_I2C_IsDeviceReady(dev->hi2c, BNO085_I2C_ADDR_ALT, 2, 50) == HAL_OK) {
            dev->i2c_addr = BNO085_I2C_ADDR_ALT;
        } else {
            /* Fallback to default */
            dev->i2c_addr = BNO085_I2C_ADDR_DEF;
        }
    }

    return BNO085_OK;
}

BNO085_Status_t BNO085_HardwareReset(BNO085_t *dev, uint32_t timeout_ms) {
    if (!dev) return BNO085_ERR_PARAM;

    /* 1. Pulse hardware reset line LOW for 15 ms */
    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_RESET);
    HAL_Delay(15);
    HAL_GPIO_WritePin(dev->rst_port, dev->rst_pin, GPIO_PIN_SET);

    /* 2. Wait for HINTN to assert LOW (sensor booting) */
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(dev->hintn_port, dev->hintn_pin) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return BNO085_ERR_TIMEOUT;
        }
    }

    /* 3. Drain all boot packets until SH-2 firmware reports initialized (0xF1) and bus settles */
    uint8_t boot_buf[256];
    bool sh2_ready = false;
    uint32_t drain_start = HAL_GetTick();

    while ((HAL_GetTick() - drain_start) < 2000) {
        if (HAL_GPIO_ReadPin(dev->hintn_port, dev->hintn_pin) == GPIO_PIN_RESET) {
            uint16_t plen = BNO085_ReadPacket(dev, boot_buf, sizeof(boot_buf));
            if (plen > 0) {
                /* Check if this packet is the SH-2 System Initialized notification (CH2, report 0xF1) */
                if (plen >= 5 && boot_buf[2] == SHTP_CHAN_SH2_CONFIG && boot_buf[4] == 0xF1) {
                    sh2_ready = true;
                }
                drain_start = HAL_GetTick();
            }
        } else if (sh2_ready && ((HAL_GetTick() - drain_start) > 100)) {
            /* SH-2 confirmed initialized and bus is quiet */
            break;
        } else if ((HAL_GetTick() - drain_start) > 500) {
            /* Fallback timeout: FIFO drained */
            break;
        }
    }

    HAL_Delay(50);
    return BNO085_OK;
}

bool BNO085_HasData(BNO085_t *dev) {
    if (!dev) return false;
    return (HAL_GPIO_ReadPin(dev->hintn_port, dev->hintn_pin) == GPIO_PIN_RESET);
}

uint16_t BNO085_ReadPacket(BNO085_t *dev, uint8_t *buffer, uint16_t buffer_size) {
    if (!dev || !buffer || buffer_size < 4) return 0;

    /* Fast check: sensor only has data ready when HINTN is active LOW */
    if (HAL_GPIO_ReadPin(dev->hintn_port, dev->hintn_pin) == GPIO_PIN_SET) {
        return 0;
    }

    /* 1. Read 4-byte SHTP Header */
    uint8_t header[4] = {0};
    if (HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr, header, 4, 100) != HAL_OK) {
        return 0;
    }

    uint16_t packet_len = (header[0] | ((uint16_t)(header[1] & 0x7F) << 8));
    if (packet_len < 4 || packet_len == 0x7FFF) {
        return 0;
    }

    /* If packet only contains header */
    if (packet_len == 4) {
        memcpy(buffer, header, 4);
        return 4;
    }

    /* 2. Read full packet.
          In SHTP over I2C, terminating the previous 4-byte read with STOP
          causes the sensor to output the packet from byte 0 (including the header). */
    uint16_t to_read = (packet_len <= buffer_size) ? packet_len : buffer_size;
    if (HAL_I2C_Master_Receive(dev->hi2c, dev->i2c_addr, buffer, to_read, 100) != HAL_OK) {
        return 0;
    }

    /* If sensor replayed the 4-byte header at byte 0 */
    if (buffer[0] == header[0] && (buffer[1] & 0x7F) == (header[1] & 0x7F)) {
        return to_read;
    }

    /* In case the sensor continued reading from byte 4 directly without repeating the header */
    if ((to_read + 4) <= buffer_size) {
        memmove(&buffer[4], buffer, to_read);
        memcpy(buffer, header, 4);
        return to_read + 4;
    }

    return to_read;
}

BNO085_Status_t BNO085_SendPacket(BNO085_t *dev, uint8_t channel, const uint8_t *payload, uint16_t payload_len) {
    if (!dev || (!payload && payload_len > 0)) return BNO085_ERR_PARAM;

    uint16_t total_len = payload_len + 4;
    uint8_t tx_buf[64];
    if (total_len > sizeof(tx_buf)) return BNO085_ERR_PARAM;

    /* SHTP 4-Byte Header */
    tx_buf[0] = (uint8_t)(total_len & 0xFF);
    tx_buf[1] = (uint8_t)((total_len >> 8) & 0x7F);
    tx_buf[2] = channel;
    tx_buf[3] = (channel < 6) ? dev->sequence_number[channel]++ : 0;

    if (payload_len > 0) {
        memcpy(&tx_buf[4], payload, payload_len);
    }

    if (HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr, tx_buf, total_len, 100) != HAL_OK) {
        return BNO085_ERR_I2C;
    }

    return BNO085_OK;
}

BNO085_Status_t BNO085_SetFeature(BNO085_t *dev, uint8_t report_id, uint16_t report_interval_ms) {
    if (!dev) return BNO085_ERR_PARAM;

    uint32_t interval_us = (uint32_t)report_interval_ms * 1000;
    uint8_t cmd[17] = {0};

    cmd[0] = SHTP_REPORT_SET_FEATURE; /* 0xFD: Set Feature Command */
    cmd[1] = report_id;               /* Report ID */
    cmd[2] = 0x00;                    /* Feature Flags */
    cmd[3] = 0x00;                    /* Change Sensitivity LSB */
    cmd[4] = 0x00;                    /* Change Sensitivity MSB */

    /* Report Interval in microseconds (32-bit unsigned little-endian) */
    cmd[5] = (uint8_t)(interval_us & 0xFF);
    cmd[6] = (uint8_t)((interval_us >> 8) & 0xFF);
    cmd[7] = (uint8_t)((interval_us >> 16) & 0xFF);
    cmd[8] = (uint8_t)((interval_us >> 24) & 0xFF);

    /* Batch Interval = 0 (no batching) */
    cmd[9]  = 0x00;
    cmd[10] = 0x00;
    cmd[11] = 0x00;
    cmd[12] = 0x00;

    /* Sensor-specific configuration word = 0 */
    cmd[13] = 0x00;
    cmd[14] = 0x00;
    cmd[15] = 0x00;
    cmd[16] = 0x00;

    return BNO085_SendPacket(dev, SHTP_CHAN_SH2_CONFIG, cmd, sizeof(cmd));
}

BNO085_Status_t BNO085_EnableGameRotationVector(BNO085_t *dev, uint16_t report_interval_ms) {
    return BNO085_SetFeature(dev, SHTP_REPORT_GAME_ROTATION_VECTOR, report_interval_ms);
}

BNO085_Status_t BNO085_EnableRotationVector(BNO085_t *dev, uint16_t report_interval_ms) {
    return BNO085_SetFeature(dev, SHTP_REPORT_ROTATION_VECTOR, report_interval_ms);
}

BNO085_Status_t BNO085_EnableAccelerometer(BNO085_t *dev, uint16_t report_interval_ms) {
    return BNO085_SetFeature(dev, SHTP_REPORT_ACCELEROMETER, report_interval_ms);
}

BNO085_Status_t BNO085_EnableGyroscope(BNO085_t *dev, uint16_t report_interval_ms) {
    return BNO085_SetFeature(dev, SHTP_REPORT_GYROSCOPE_CALIBRATED, report_interval_ms);
}

BNO085_Status_t BNO085_Update(BNO085_t *dev) {
    if (!dev) return BNO085_ERR_PARAM;

    /* Fast check: If HINTN is HIGH, sensor has no data ready */
    if (HAL_GPIO_ReadPin(dev->hintn_port, dev->hintn_pin) == GPIO_PIN_SET) {
        return BNO085_NO_DATA;
    }

    uint8_t packet[128];
    uint16_t len = BNO085_ReadPacket(dev, packet, sizeof(packet));
    if (len < 4) {
        return BNO085_NO_DATA;
    }

    uint8_t channel = packet[2];

    /* Channel 3: Sensor Input Reports */
    if (channel == SHTP_CHAN_SH2_INPUT) {
        uint16_t offset = 4;
        while (offset < len) {
            uint8_t report_id = packet[offset];

            /* Timestamp reference report is 5 bytes: ID (0xFB) + 4 bytes tick */
            if (report_id == SHTP_REPORT_TIMESTAMP_BASE) {
                offset += 5;
                continue;
            }

            /* Rotation vector reports (Game Rotation Vector, Absolute, ARVR Stabilized) */
            if (report_id == SHTP_REPORT_GAME_ROTATION_VECTOR ||
                report_id == SHTP_REPORT_ROTATION_VECTOR ||
                report_id == SHTP_REPORT_ARVR_STABILIZED_GRV) {

                if ((offset + 12) <= len) {
                    int16_t q_i = (int16_t)(packet[offset + 4]  | (packet[offset + 5]  << 8));
                    int16_t q_j = (int16_t)(packet[offset + 6]  | (packet[offset + 7]  << 8));
                    int16_t q_k = (int16_t)(packet[offset + 8]  | (packet[offset + 9]  << 8));
                    int16_t q_r = (int16_t)(packet[offset + 10] | (packet[offset + 11] << 8));

                    /* Scale Q14 fixed-point to float [-1.0f, +1.0f] */
                    dev->data.q_i = q_i * (1.0f / 16384.0f);
                    dev->data.q_j = q_j * (1.0f / 16384.0f);
                    dev->data.q_k = q_k * (1.0f / 16384.0f);
                    dev->data.q_r = q_r * (1.0f / 16384.0f);

                    dev->data.sequence = packet[offset + 1];
                    dev->data.accuracy = packet[offset + 2] & 0x03;

                    /* Compute Euler angles (Yaw, Pitch, Roll in degrees) */
                    float siny_cosp = 2.0f * (dev->data.q_r * dev->data.q_k + dev->data.q_i * dev->data.q_j);
                    float cosy_cosp = 1.0f - 2.0f * (dev->data.q_j * dev->data.q_j + dev->data.q_k * dev->data.q_k);
                    dev->data.yaw   = 57.2957795f * atan2f(siny_cosp, cosy_cosp);

                    float sinp = 2.0f * (dev->data.q_r * dev->data.q_j - dev->data.q_k * dev->data.q_i);
                    if (fabsf(sinp) >= 1.0f) {
                        dev->data.pitch = (sinp >= 0.0f) ? 90.0f : -90.0f;
                    } else {
                        dev->data.pitch = 57.2957795f * asinf(sinp);
                    }

                    float sinr_cosp = 2.0f * (dev->data.q_r * dev->data.q_i + dev->data.q_j * dev->data.q_k);
                    float cosr_cosp = 1.0f - 2.0f * (dev->data.q_i * dev->data.q_i + dev->data.q_j * dev->data.q_j);
                    dev->data.roll  = 57.2957795f * atan2f(sinr_cosp, cosr_cosp);

                    dev->data.last_update_tick = HAL_GetTick();
                    dev->data.has_new_data = true;

                    uint16_t report_size = (report_id == SHTP_REPORT_ROTATION_VECTOR) ? 14 : 12;
                    offset += report_size;
                    continue;
                }
            }

            /* Accelerometer report (0x01) */
            if (report_id == SHTP_REPORT_ACCELEROMETER) {
                if ((offset + 10) <= len) {
                    int16_t raw_x = (int16_t)(packet[offset + 4] | (packet[offset + 5] << 8));
                    int16_t raw_y = (int16_t)(packet[offset + 6] | (packet[offset + 7] << 8));
                    int16_t raw_z = (int16_t)(packet[offset + 8] | (packet[offset + 9] << 8));

                    /* Scale Q8 fixed-point to float m/s^2 */
                    dev->data.accel_x = raw_x * (1.0f / 256.0f);
                    dev->data.accel_y = raw_y * (1.0f / 256.0f);
                    dev->data.accel_z = raw_z * (1.0f / 256.0f);

                    dev->data.last_update_tick = HAL_GetTick();
                    dev->data.has_new_data = true;
                    offset += 10;
                    continue;
                }
            }

            /* Calibrated Gyroscope report (0x02) */
            if (report_id == SHTP_REPORT_GYROSCOPE_CALIBRATED) {
                if ((offset + 10) <= len) {
                    int16_t raw_x = (int16_t)(packet[offset + 4] | (packet[offset + 5] << 8));
                    int16_t raw_y = (int16_t)(packet[offset + 6] | (packet[offset + 7] << 8));
                    int16_t raw_z = (int16_t)(packet[offset + 8] | (packet[offset + 9] << 8));

                    /* Scale Q9 fixed-point to float rad/s */
                    dev->data.gyro_x = raw_x * (1.0f / 512.0f);
                    dev->data.gyro_y = raw_y * (1.0f / 512.0f);
                    dev->data.gyro_z = raw_z * (1.0f / 512.0f);

                    dev->data.last_update_tick = HAL_GetTick();
                    dev->data.has_new_data = true;
                    offset += 10;
                    continue;
                }
            }

            /* Advance past unhandled report or break */
            break;
        }
        return BNO085_OK;
    }

    return BNO085_OK;
}

