#ifndef LIBBNO055_LINUX_BNO055_C_H
#define LIBBNO055_LINUX_BNO055_C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque handle type for BNO055 instance
typedef struct bno055_handle* bno055_handle_t;

// Standard 3D vector for physical readings
typedef struct {
    float x;
    float y;
    float z;
} bno055_vector3_t;

// Standard Quaternion for rotation representation
typedef struct {
    float w;
    float x;
    float y;
    float z;
} bno055_quaternion_t;

// Calibration status
typedef struct {
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} bno055_calibration_status_t;

// Diagnostics telemetry
typedef struct {
    uint32_t write_failures;
    uint32_t read_failures;
    uint32_t reconnect_attempts;
} bno055_diagnostics_t;

// Raw 18-byte burst sensor data
typedef struct {
    bno055_vector3_t accel;
    bno055_vector3_t mag;
    bno055_vector3_t gyro;
} bno055_raw_sensor_data_t;

// Operating modes
typedef enum {
    BNO055_OPMODE_CONFIG = 0X00,
    BNO055_OPMODE_ACC_ONLY = 0X01,
    BNO055_OPMODE_MAG_ONLY = 0X02,
    BNO055_OPMODE_GYRO_ONLY = 0X03,
    BNO055_OPMODE_ACC_MAG = 0X04,
    BNO055_OPMODE_ACC_GYRO = 0X05,
    BNO055_OPMODE_MAG_GYRO = 0X06,
    BNO055_OPMODE_AMG = 0X07,
    BNO055_OPMODE_IMU_PLUS = 0X08,
    BNO055_OPMODE_COMPASS = 0X09,
    BNO055_OPMODE_M4G = 0X0A,
    BNO055_OPMODE_NDOF_FMC_OFF = 0X0B,
    BNO055_OPMODE_NDOF = 0X0C
} bno055_opmode_t;

// Constructors & Destructors
bno055_handle_t bno055_create_i2c(uint8_t i2c_address, const char* i2c_device);
bno055_handle_t bno055_create_uart(const char* port, uint32_t baudrate);
void bno055_destroy(bno055_handle_t handle);

// Lifecycle
bool bno055_begin(bno055_handle_t handle, bno055_opmode_t mode);
bool bno055_reset(bno055_handle_t handle);

// Mode Configuration
void bno055_set_mode(bno055_handle_t handle, bno055_opmode_t mode);
bno055_opmode_t bno055_get_mode(bno055_handle_t handle);
void bno055_set_ext_crystal_use(bno055_handle_t handle, bool use_xtal);

// Exception-free Sensor Data Queries (returns true on success, false on failure)
bool bno055_get_accelerometer(bno055_handle_t handle, bno055_vector3_t* out_accel);
bool bno055_get_magnetometer(bno055_handle_t handle, bno055_vector3_t* out_mag);
bool bno055_get_gyroscope(bno055_handle_t handle, bno055_vector3_t* out_gyro);
bool bno055_get_euler_angles(bno055_handle_t handle, bno055_vector3_t* out_euler);
bool bno055_get_linear_acceleration(bno055_handle_t handle, bno055_vector3_t* out_accel);
bool bno055_get_gravity(bno055_handle_t handle, bno055_vector3_t* out_gravity);
bool bno055_get_quaternion(bno055_handle_t handle, bno055_quaternion_t* out_quat);
bool bno055_get_temperature(bno055_handle_t handle, int8_t* out_temp);
bool bno055_get_raw_sensor_data(bno055_handle_t handle, bno055_raw_sensor_data_t* out_raw);

// Diagnostics & Calibration
bool bno055_get_calibration_status(bno055_handle_t handle, bno055_calibration_status_t* out_status);
bool bno055_is_fully_calibrated(bno055_handle_t handle);
bool bno055_get_diagnostics(bno055_handle_t handle, bno055_diagnostics_t* out_diag);
bool bno055_save_calibration_file(bno055_handle_t handle, const char* filepath);
bool bno055_load_calibration_file(bno055_handle_t handle, const char* filepath);
void bno055_enable_auto_calibration(bno055_handle_t handle, const char* filepath);
void bno055_disable_auto_calibration(bno055_handle_t handle);

// Asynchronous & Interrupt Driven Reading
typedef void (*bno055_raw_async_callback_t)(const bno055_raw_sensor_data_t* raw_data, void* user_data);
bool bno055_start_interrupt_driven_reading(bno055_handle_t handle, int gpio_pin, bno055_raw_async_callback_t callback, void* user_data);
void bno055_stop_interrupt_driven_reading(bno055_handle_t handle);

// Power Management
void bno055_enter_suspend_mode(bno055_handle_t handle);
void bno055_enter_normal_mode(bno055_handle_t handle);

// Utilities
bno055_vector3_t bno055_to_euler_degrees(const bno055_quaternion_t* q);

#ifdef __cplusplus
}
#endif

#endif  // LIBBNO055_LINUX_BNO055_C_H
