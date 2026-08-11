#ifndef BNO055_HAL_H
#define BNO055_HAL_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BNO055_I2C_ADDR_LOW   (0x28U << 1)
#define BNO055_I2C_ADDR_HIGH  (0x29U << 1)
#define BNO055_CALIBRATION_PROFILE_SIZE 22U

typedef enum {
    BNO055_OK = 0,
    BNO055_ERROR,
    BNO055_TIMEOUT,
    BNO055_WRONG_CHIP_ID
} BNO055_Status;

typedef enum {
    BNO055_MODE_CONFIG       = 0x00,
    BNO055_MODE_ACCONLY      = 0x01,
    BNO055_MODE_MAGONLY      = 0x02,
    BNO055_MODE_GYRONLY      = 0x03,
    BNO055_MODE_ACCMAG       = 0x04,
    BNO055_MODE_ACCGYRO      = 0x05,
    BNO055_MODE_MAGGYRO      = 0x06,
    BNO055_MODE_AMG          = 0x07,
    BNO055_MODE_IMUPLUS      = 0x08,
    BNO055_MODE_COMPASS      = 0x09,
    BNO055_MODE_M4G          = 0x0A,
    BNO055_MODE_NDOF_FMC_OFF = 0x0B,
    BNO055_MODE_NDOF         = 0x0C
} BNO055_OperationMode;

typedef struct {
    int16_t w;
    int16_t x;
    int16_t y;
    int16_t z;
} BNO055_Quaternion;

typedef struct {
    float heading_deg;
    float roll_deg;
    float pitch_deg;
} BNO055_Euler;
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} BNO055_Vector3Int16;

typedef struct {
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} BNO055_Calibration;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t address;
    BNO055_OperationMode mode;
} BNO055_Handle;

BNO055_Status BNO055_Init(BNO055_Handle *dev,
                          I2C_HandleTypeDef *hi2c,
                          uint16_t address,
                          BNO055_OperationMode mode);
BNO055_Status BNO055_SetMode(BNO055_Handle *dev, BNO055_OperationMode mode);
BNO055_Status BNO055_ReadQuaternion(BNO055_Handle *dev, BNO055_Quaternion *quat);
BNO055_Status BNO055_ReadEuler(BNO055_Handle *dev, BNO055_Euler *euler);
BNO055_Status BNO055_ReadGyroscope(BNO055_Handle *dev, BNO055_Vector3Int16 *gyro);
BNO055_Status BNO055_ReadLinearAcceleration(BNO055_Handle *dev, BNO055_Vector3Int16 *accel);
BNO055_Status BNO055_ReadCalibration(BNO055_Handle *dev, BNO055_Calibration *calib);
BNO055_Status BNO055_ReadTemperature(BNO055_Handle *dev, int8_t *temperature_c);
BNO055_Status BNO055_ReadSystemStatus(BNO055_Handle *dev,
                                      uint8_t *system_status,
                                      uint8_t *self_test,
                                      uint8_t *system_error);
bool BNO055_IsFullyCalibrated(const BNO055_Calibration *calib);
BNO055_Status BNO055_ReadOperationMode(BNO055_Handle *dev,uint8_t *mode);
BNO055_Status BNO055_ReadCalibrationProfile(
    BNO055_Handle *dev,
    uint8_t profile[BNO055_CALIBRATION_PROFILE_SIZE]);
BNO055_Status BNO055_ApplyCalibrationProfile(
    BNO055_Handle *dev,
    const uint8_t profile[BNO055_CALIBRATION_PROFILE_SIZE]);

#ifdef __cplusplus
}
#endif

#endif
