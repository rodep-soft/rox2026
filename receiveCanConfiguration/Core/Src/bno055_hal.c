#include "bno055_hal.h"

#define BNO055_REG_CHIP_ID       0x00U
#define BNO055_REG_PAGE_ID       0x07U
#define BNO055_REG_EULER_H_LSB   0x1AU
#define BNO055_REG_QUAT_W_LSB    0x20U
#define BNO055_REG_TEMP          0x34U
#define BNO055_REG_CALIB_STAT    0x35U
#define BNO055_REG_SELFTEST      0x36U
#define BNO055_REG_SYS_STATUS    0x39U
#define BNO055_REG_SYS_ERR       0x3AU
#define BNO055_REG_UNIT_SEL      0x3BU
#define BNO055_REG_OPR_MODE      0x3DU
#define BNO055_REG_PWR_MODE      0x3EU
#define BNO055_REG_SYS_TRIGGER   0x3FU
#define BNO055_REG_ACC_OFFSET_X_LSB 0x55U

#define BNO055_CHIP_ID_VALUE     0xA0U
#define BNO055_PWR_MODE_NORMAL   0x00U
#define BNO055_RESET_COMMAND     0x20U
#define BNO055_TIMEOUT_MS        5U

volatile uint8_t debug_data[8];

static BNO055_Status bno055_from_hal(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return BNO055_OK;
    }
    if (status == HAL_TIMEOUT) {
        return BNO055_TIMEOUT;
    }
    return BNO055_ERROR;
}

static BNO055_Status bno055_read(BNO055_Handle *dev,
                                 uint8_t reg,
                                 uint8_t *data,
                                 uint16_t length)
{
    if ((dev == NULL) || (dev->hi2c == NULL) || (data == NULL)) {
        return BNO055_ERROR;
    }

    return bno055_from_hal(HAL_I2C_Mem_Read(dev->hi2c,
                                            dev->address,
                                            reg,
                                            I2C_MEMADD_SIZE_8BIT,
                                            data,
                                            length,
                                            BNO055_TIMEOUT_MS));
}

static BNO055_Status bno055_write_u8(BNO055_Handle *dev, uint8_t reg, uint8_t value)
{
    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return BNO055_ERROR;
    }

    return bno055_from_hal(HAL_I2C_Mem_Write(dev->hi2c,
                                             dev->address,
                                             reg,
                                             I2C_MEMADD_SIZE_8BIT,
                                             &value,
                                             1U,
                                             BNO055_TIMEOUT_MS));
}

static int16_t bno055_decode_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

BNO055_Status BNO055_SetMode(BNO055_Handle *dev, BNO055_OperationMode mode)
{
    BNO055_Status status = bno055_write_u8(dev, BNO055_REG_OPR_MODE, (uint8_t)mode);
    if (status != BNO055_OK) {
        return status;
    }

    /* Datasheet requires a mode-switch delay. */
    HAL_Delay((mode == BNO055_MODE_CONFIG) ? 25U : 20U);
    dev->mode = mode;
    return BNO055_OK;
}

BNO055_Status BNO055_Init(BNO055_Handle *dev,
                          I2C_HandleTypeDef *hi2c,
                          uint16_t address,
                          BNO055_OperationMode mode)
{
    uint8_t chip_id = 0U;
    BNO055_Status status;

    if ((dev == NULL) || (hi2c == NULL)) {
        return BNO055_ERROR;
    }

    dev->hi2c = hi2c;
    dev->address = address;
    dev->mode = BNO055_MODE_CONFIG;

    HAL_Delay(700U);

    status = bno055_read(dev, BNO055_REG_CHIP_ID, &chip_id, 1U);
    if (status != BNO055_OK) {
        return status;
    }
    if (chip_id != BNO055_CHIP_ID_VALUE) {
        return BNO055_WRONG_CHIP_ID;
    }

    status = BNO055_SetMode(dev, BNO055_MODE_CONFIG);
    if (status != BNO055_OK) {
        return status;
    }

    status = bno055_write_u8(dev, BNO055_REG_SYS_TRIGGER, BNO055_RESET_COMMAND);
    if (status != BNO055_OK) {
        return status;
    }

    HAL_Delay(700U);

    /* After reset, wait until CHIP_ID becomes readable again. */
    for (uint32_t retry = 0U; retry < 20U; ++retry) {
        if ((bno055_read(dev, BNO055_REG_CHIP_ID, &chip_id, 1U) == BNO055_OK) &&
            (chip_id == BNO055_CHIP_ID_VALUE)) {
            break;
        }
        HAL_Delay(50U);
    }
    if (chip_id != BNO055_CHIP_ID_VALUE) {
        return BNO055_WRONG_CHIP_ID;
    }

    status = bno055_write_u8(dev, BNO055_REG_PWR_MODE, BNO055_PWR_MODE_NORMAL);
    if (status != BNO055_OK) {
        return status;
    }
    HAL_Delay(10U);

    status = bno055_write_u8(dev, BNO055_REG_PAGE_ID, 0x00U);
    if (status != BNO055_OK) {
        return status;
    }

    /* Windows orientation, Celsius, degrees, m/s^2, dps. */
    status = bno055_write_u8(dev, BNO055_REG_UNIT_SEL, 0x00U);
    if (status != BNO055_OK) {
        return status;
    }

    return BNO055_SetMode(dev, mode);
}

//BNO055_Status BNO055_ReadQuaternion(BNO055_Handle *dev, BNO055_Quaternion *quat)
//{
//    uint8_t data[8];
//    BNO055_Status status;
//
//    if (quat == NULL) {
//        return BNO055_ERROR;
//    }
//
//    status = bno055_read(dev, BNO055_REG_QUAT_W_LSB, data, sizeof(data));
//    if (status != BNO055_OK) {
//        return status;
//    }
//
//    //CAN経由で送信するためint16のスケールなしに変更
////    const float scale = 1.0f / 16384.0f;
////    quat->w = (float)bno055_decode_i16_le(&data[0]) * scale;
////    quat->x = (float)bno055_decode_i16_le(&data[2]) * scale;
////    quat->y = (float)bno055_decode_i16_le(&data[4]) * scale;
////    quat->z = (float)bno055_decode_i16_le(&data[6]) * scale;
//    for (uint8_t i = 0; i < 8; i++) {
//        debug_data[i] = data[i];
//    }
//      quat->w = bno055_decode_i16_le(&data[0]);
//      quat->x = bno055_decode_i16_le(&data[2]);
//      quat->y = bno055_decode_i16_le(&data[4]);
//      quat->z = bno055_decode_i16_le(&data[6]);
//    return BNO055_OK;
//}

BNO055_Status BNO055_ReadQuaternion(
    BNO055_Handle *dev,
    BNO055_Quaternion *quat)
{
    uint8_t data[8];

    if ((dev == NULL) || (quat == NULL)) {
        return BNO055_ERROR;
    }
    for (uint8_t i = 0U; i < sizeof(data); ++i) {
        const BNO055_Status status =
            bno055_read(dev,
                        (uint8_t)(BNO055_REG_QUAT_W_LSB + i),
                        &data[i],
                        1U);
        if (status != BNO055_OK) {
            return status;
        }
    }

    const int16_t w = bno055_decode_i16_le(&data[0]);
    const int16_t x = bno055_decode_i16_le(&data[2]);
    const int16_t y = bno055_decode_i16_le(&data[4]);
    const int16_t z = bno055_decode_i16_le(&data[6]);

    quat->w = w;
    quat->x = x;
    quat->y = y;
    quat->z = z;

    return BNO055_OK;
}

BNO055_Status BNO055_ReadEuler(BNO055_Handle *dev, BNO055_Euler *euler)
{
    uint8_t data[6];
    BNO055_Status status;

    if (euler == NULL) {
        return BNO055_ERROR;
    }

    status = bno055_read(dev, BNO055_REG_EULER_H_LSB, data, sizeof(data));
    if (status != BNO055_OK) {
        return status;
    }

    const float scale = 1.0f / 16.0f;
    euler->heading_deg = (float)bno055_decode_i16_le(&data[0]) * scale;
    euler->roll_deg    = (float)bno055_decode_i16_le(&data[2]) * scale;
    euler->pitch_deg   = (float)bno055_decode_i16_le(&data[4]) * scale;
    return BNO055_OK;
}

BNO055_Status BNO055_ReadCalibration(BNO055_Handle *dev, BNO055_Calibration *calib)
{
    uint8_t value = 0U;
    BNO055_Status status;

    if (calib == NULL) {
        return BNO055_ERROR;
    }

    status = bno055_read(dev, BNO055_REG_CALIB_STAT, &value, 1U);
    if (status != BNO055_OK) {
        return status;
    }

    calib->system = (value >> 6) & 0x03U;
    calib->gyro   = (value >> 4) & 0x03U;
    calib->accel  = (value >> 2) & 0x03U;
    calib->mag    = value & 0x03U;
    return BNO055_OK;
}

BNO055_Status BNO055_ReadTemperature(BNO055_Handle *dev, int8_t *temperature_c)
{
    uint8_t value = 0U;
    BNO055_Status status;

    if (temperature_c == NULL) {
        return BNO055_ERROR;
    }

    status = bno055_read(dev, BNO055_REG_TEMP, &value, 1U);
    if (status == BNO055_OK) {
        *temperature_c = (int8_t)value;
    }
    return status;
}

BNO055_Status BNO055_ReadSystemStatus(BNO055_Handle *dev,
                                      uint8_t *system_status,
                                      uint8_t *self_test,
                                      uint8_t *system_error)
{
    BNO055_Status status;

    if ((system_status == NULL) || (self_test == NULL) || (system_error == NULL)) {
        return BNO055_ERROR;
    }

    status = bno055_read(dev, BNO055_REG_SYS_STATUS, system_status, 1U);
    if (status != BNO055_OK) {
        return status;
    }
    status = bno055_read(dev, BNO055_REG_SELFTEST, self_test, 1U);
    if (status != BNO055_OK) {
        return status;
    }
    return bno055_read(dev, BNO055_REG_SYS_ERR, system_error, 1U);
}

bool BNO055_IsFullyCalibrated(const BNO055_Calibration *calib)
{
    return (calib != NULL) &&
           (calib->system == 3U) &&
           (calib->gyro == 3U) &&
           (calib->accel == 3U) &&
           (calib->mag == 3U);
}

BNO055_Status BNO055_ReadOperationMode(
    BNO055_Handle *dev,
    uint8_t *mode)
{
    if (mode == NULL) {
        return BNO055_ERROR;
    }

    return bno055_read(
        dev,
        BNO055_REG_OPR_MODE,
        mode,
        1U
    );
}

BNO055_Status BNO055_ReadCalibrationProfile(
    BNO055_Handle *dev,
    uint8_t profile[BNO055_CALIBRATION_PROFILE_SIZE])
{
    BNO055_Status status;
    BNO055_OperationMode previous_mode;

    if ((dev == NULL) || (profile == NULL)) {
        return BNO055_ERROR;
    }

    previous_mode = dev->mode;
    status = BNO055_SetMode(dev, BNO055_MODE_CONFIG);
    if (status != BNO055_OK) {
        return status;
    }

    /* Single-register reads are retained because burst reads are unstable. */
    for (uint8_t i = 0U; i < BNO055_CALIBRATION_PROFILE_SIZE; ++i) {
        status = bno055_read(dev,
            (uint8_t)(BNO055_REG_ACC_OFFSET_X_LSB + i), &profile[i], 1U);
        if (status != BNO055_OK) {
            (void)BNO055_SetMode(dev, previous_mode);
            return status;
        }
    }

    return BNO055_SetMode(dev, previous_mode);
}

BNO055_Status BNO055_ApplyCalibrationProfile(
    BNO055_Handle *dev,
    const uint8_t profile[BNO055_CALIBRATION_PROFILE_SIZE])
{
    BNO055_Status status;
    BNO055_OperationMode previous_mode;

    if ((dev == NULL) || (profile == NULL)) {
        return BNO055_ERROR;
    }

    previous_mode = dev->mode;
    status = BNO055_SetMode(dev, BNO055_MODE_CONFIG);
    if (status != BNO055_OK) {
        return status;
    }

    for (uint8_t i = 0U; i < BNO055_CALIBRATION_PROFILE_SIZE; ++i) {
        status = bno055_write_u8(dev,
            (uint8_t)(BNO055_REG_ACC_OFFSET_X_LSB + i), profile[i]);
        if (status != BNO055_OK) {
            (void)BNO055_SetMode(dev, previous_mode);
            return status;
        }
    }

    return BNO055_SetMode(dev, previous_mode);
}
