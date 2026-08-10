#include "bno055_calibration.h"
#include "stm32f3xx_hal_flash.h"
#include "stm32f3xx_hal_flash_ex.h"
#include <stddef.h>
#include <string.h>

#define CALIBRATION_MAGIC   0x434F4E42UL /* "BNOC" in little endian */
#define CALIBRATION_VERSION 1U

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t profile_size;
    uint8_t profile[BNO055_CALIBRATION_PROFILE_SIZE];
    uint8_t reserved[2];
    uint32_t crc32;
} CalibrationRecord;

_Static_assert((sizeof(CalibrationRecord) % 2U) == 0U,
               "Flash record must be half-word aligned");

static uint32_t calculate_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (uint32_t i = 0U; i < length; ++i) {
        crc ^= data[i];
        for (uint32_t bit = 0U; bit < 8U; ++bit) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1U);
            crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static bool record_is_valid(const CalibrationRecord *record)
{
    if ((record->magic != CALIBRATION_MAGIC) ||
        (record->version != CALIBRATION_VERSION) ||
        (record->profile_size != BNO055_CALIBRATION_PROFILE_SIZE)) {
        return false;
    }

    return record->crc32 == calculate_crc32(
        (const uint8_t *)record, offsetof(CalibrationRecord, crc32));
}

bool BNO055_CalibrationStorageLoad(
    uint8_t profile[BNO055_CALIBRATION_PROFILE_SIZE])
{
    const CalibrationRecord *record =
        (const CalibrationRecord *)BNO055_CALIBRATION_FLASH_ADDRESS;

    if ((profile == NULL) || !record_is_valid(record)) {
        return false;
    }

    memcpy(profile, record->profile, BNO055_CALIBRATION_PROFILE_SIZE);
    return true;
}

bool BNO055_CalibrationStorageSave(
    const uint8_t profile[BNO055_CALIBRATION_PROFILE_SIZE])
{
    CalibrationRecord record = {0};
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    HAL_StatusTypeDef status;

    if (profile == NULL) {
        return false;
    }

    record.magic = CALIBRATION_MAGIC;
    record.version = CALIBRATION_VERSION;
    record.profile_size = BNO055_CALIBRATION_PROFILE_SIZE;
    memcpy(record.profile, profile, BNO055_CALIBRATION_PROFILE_SIZE);
    record.crc32 = calculate_crc32(
        (const uint8_t *)&record, offsetof(CalibrationRecord, crc32));

    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = BNO055_CALIBRATION_FLASH_ADDRESS;
    erase.NbPages = 1U;

    status = HAL_FLASH_Unlock();
    if (status == HAL_OK) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR |
                               FLASH_FLAG_WRPERR);
        status = HAL_FLASHEx_Erase(&erase, &page_error);
    }

    for (uint32_t offset = 0U;
         (status == HAL_OK) && (offset < sizeof(record));
         offset += sizeof(uint16_t)) {
        uint16_t value;
        memcpy(&value, ((const uint8_t *)&record) + offset, sizeof(value));
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
            BNO055_CALIBRATION_FLASH_ADDRESS + offset, value);
    }

    (void)HAL_FLASH_Lock();
    if (status != HAL_OK) {
        return false;
    }

    return record_is_valid(
        (const CalibrationRecord *)BNO055_CALIBRATION_FLASH_ADDRESS);
}
