#ifndef BNO055_CALIBRATION_H
#define BNO055_CALIBRATION_H

#include "bno055_hal.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Last 2 KiB Flash page, reserved in STM32F303K8TX_FLASH.ld. */
#define BNO055_CALIBRATION_FLASH_ADDRESS 0x0800F800U

bool BNO055_CalibrationStorageLoad(
    uint8_t profile[BNO055_CALIBRATION_PROFILE_SIZE]);
bool BNO055_CalibrationStorageSave(
    const uint8_t profile[BNO055_CALIBRATION_PROFILE_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_CALIBRATION_H */
