#include "libbno055-linux/bno055_c.h"

#include <utility>

#include "libbno055-linux/bno055.hpp"

struct bno055_handle {
    bno055lib::BNO055 cpp_imu;

    template <typename... Args>
    explicit bno055_handle(Args&&... args) : cpp_imu(std::forward<Args>(args)...) {}
};

bno055_handle_t bno055_create_i2c(uint8_t i2c_address, const char* i2c_device) {
    try {
        const char* dev = i2c_device ? i2c_device : "/dev/i2c-1";
        return new bno055_handle(i2c_address, dev);
    } catch (...) {
        return nullptr;
    }
}

bno055_handle_t bno055_create_uart(const char* port, uint32_t baudrate) {
    try {
        bno055lib::BNO055::UARTConfig cfg;
        cfg.port = port ? port : "/dev/ttyUSB0";
        cfg.baudrate = baudrate;
        return new bno055_handle(cfg);
    } catch (...) {
        return nullptr;
    }
}

void bno055_destroy(bno055_handle_t handle) {
    delete handle;
}

bool bno055_begin(bno055_handle_t handle, bno055_opmode_t mode) {
    if (!handle) return false;
    return handle->cpp_imu.begin(static_cast<bno055lib::OpMode>(mode));
}

bool bno055_reset(bno055_handle_t handle) {
    if (!handle) return false;
    return handle->cpp_imu.reset();
}

void bno055_set_mode(bno055_handle_t handle, bno055_opmode_t mode) {
    if (!handle) return;
    try {
        handle->cpp_imu.setMode(static_cast<bno055lib::OpMode>(mode));
    } catch (...) {
    }
}

bno055_opmode_t bno055_get_mode(bno055_handle_t handle) {
    if (!handle) return BNO055_OPMODE_CONFIG;
    try {
        return static_cast<bno055_opmode_t>(handle->cpp_imu.getMode());
    } catch (...) {
        return BNO055_OPMODE_CONFIG;
    }
}

void bno055_set_ext_crystal_use(bno055_handle_t handle, bool use_xtal) {
    if (!handle) return;
    try {
        handle->cpp_imu.setExtCrystalUse(use_xtal);
    } catch (...) {
    }
}

static inline bno055_vector3_t to_c_vec3(const bno055lib::Vector3& v) {
    return bno055_vector3_t{v.x, v.y, v.z};
}

static inline bno055_quaternion_t to_c_quat(const bno055lib::Quaternion& q) {
    return bno055_quaternion_t{q.w, q.x, q.y, q.z};
}

bool bno055_get_accelerometer(bno055_handle_t handle, bno055_vector3_t* out_accel) {
    if (!handle || !out_accel) return false;
    auto val = handle->cpp_imu.getAccelerometerNoexcept();
    if (!val) return false;
    *out_accel = to_c_vec3(*val);
    return true;
}

bool bno055_get_magnetometer(bno055_handle_t handle, bno055_vector3_t* out_mag) {
    if (!handle || !out_mag) return false;
    auto val = handle->cpp_imu.getMagnetometerNoexcept();
    if (!val) return false;
    *out_mag = to_c_vec3(*val);
    return true;
}

bool bno055_get_gyroscope(bno055_handle_t handle, bno055_vector3_t* out_gyro) {
    if (!handle || !out_gyro) return false;
    auto val = handle->cpp_imu.getGyroscopeNoexcept();
    if (!val) return false;
    *out_gyro = to_c_vec3(*val);
    return true;
}

bool bno055_get_euler_angles(bno055_handle_t handle, bno055_vector3_t* out_euler) {
    if (!handle || !out_euler) return false;
    auto val = handle->cpp_imu.getEulerAnglesNoexcept();
    if (!val) return false;
    *out_euler = to_c_vec3(*val);
    return true;
}

bool bno055_get_linear_acceleration(bno055_handle_t handle, bno055_vector3_t* out_accel) {
    if (!handle || !out_accel) return false;
    auto val = handle->cpp_imu.getLinearAccelerationNoexcept();
    if (!val) return false;
    *out_accel = to_c_vec3(*val);
    return true;
}

bool bno055_get_gravity(bno055_handle_t handle, bno055_vector3_t* out_gravity) {
    if (!handle || !out_gravity) return false;
    auto val = handle->cpp_imu.getGravityNoexcept();
    if (!val) return false;
    *out_gravity = to_c_vec3(*val);
    return true;
}

bool bno055_get_quaternion(bno055_handle_t handle, bno055_quaternion_t* out_quat) {
    if (!handle || !out_quat) return false;
    auto val = handle->cpp_imu.getQuaternionNoexcept();
    if (!val) return false;
    *out_quat = to_c_quat(*val);
    return true;
}

bool bno055_get_temperature(bno055_handle_t handle, int8_t* out_temp) {
    if (!handle || !out_temp) return false;
    auto val = handle->cpp_imu.getTemperatureNoexcept();
    if (!val) return false;
    *out_temp = *val;
    return true;
}

bool bno055_get_raw_sensor_data(bno055_handle_t handle, bno055_raw_sensor_data_t* out_raw) {
    if (!handle || !out_raw) return false;
    auto val = handle->cpp_imu.getRawSensorDataNoexcept();
    if (!val) return false;
    out_raw->accel = to_c_vec3(val->accel);
    out_raw->mag = to_c_vec3(val->mag);
    out_raw->gyro = to_c_vec3(val->gyro);
    return true;
}

bool bno055_get_calibration_status(bno055_handle_t handle, bno055_calibration_status_t* out_status) {
    if (!handle || !out_status) return false;
    try {
        auto cal = handle->cpp_imu.getCalibrationStatus();
        out_status->sys = cal.sys;
        out_status->gyro = cal.gyro;
        out_status->accel = cal.accel;
        out_status->mag = cal.mag;
        return true;
    } catch (...) {
        return false;
    }
}

bool bno055_is_fully_calibrated(bno055_handle_t handle) {
    if (!handle) return false;
    try {
        return handle->cpp_imu.getCalibrationStatus().isFullyCalibrated();
    } catch (...) {
        return false;
    }
}

bool bno055_get_diagnostics(bno055_handle_t handle, bno055_diagnostics_t* out_diag) {
    if (!handle || !out_diag) return false;
    auto diag = handle->cpp_imu.getDiagnostics();
    out_diag->write_failures = diag.write_failures;
    out_diag->read_failures = diag.read_failures;
    out_diag->reconnect_attempts = diag.reconnect_attempts;
    return true;
}

bool bno055_save_calibration_file(bno055_handle_t handle, const char* filepath) {
    if (!handle || !filepath) return false;
    return handle->cpp_imu.saveCalibrationFile(filepath);
}

bool bno055_load_calibration_file(bno055_handle_t handle, const char* filepath) {
    if (!handle || !filepath) return false;
    return handle->cpp_imu.loadCalibrationFile(filepath);
}

void bno055_enable_auto_calibration(bno055_handle_t handle, const char* filepath) {
    if (!handle || !filepath) return;
    handle->cpp_imu.enableAutoCalibration(filepath);
}

void bno055_disable_auto_calibration(bno055_handle_t handle) {
    if (!handle) return;
    handle->cpp_imu.disableAutoCalibration();
}

bool bno055_start_interrupt_driven_reading(bno055_handle_t handle, int gpio_pin, bno055_raw_async_callback_t callback,
                                           void* user_data) {
    if (!handle || !callback) return false;
    return handle->cpp_imu.startInterruptDrivenReading(
        gpio_pin, [callback, user_data](const bno055lib::BNO055::RawSensorData& raw) {
            bno055_raw_sensor_data_t c_raw;
            c_raw.accel = to_c_vec3(raw.accel);
            c_raw.mag = to_c_vec3(raw.mag);
            c_raw.gyro = to_c_vec3(raw.gyro);
            callback(&c_raw, user_data);
        });
}

void bno055_stop_interrupt_driven_reading(bno055_handle_t handle) {
    if (!handle) return;
    handle->cpp_imu.stopInterruptDrivenReading();
}

void bno055_enter_suspend_mode(bno055_handle_t handle) {
    if (!handle) return;
    try {
        handle->cpp_imu.enterSuspendMode();
    } catch (...) {
    }
}

void bno055_enter_normal_mode(bno055_handle_t handle) {
    if (!handle) return;
    try {
        handle->cpp_imu.enterNormalMode();
    } catch (...) {
    }
}

bno055_vector3_t bno055_to_euler_degrees(const bno055_quaternion_t* q) {
    if (!q) return bno055_vector3_t{0.0f, 0.0f, 0.0f};
    bno055lib::Quaternion cpp_q{q->w, q->x, q->y, q->z};
    auto euler = bno055lib::toEulerDegrees(cpp_q);
    return bno055_vector3_t{euler.x, euler.y, euler.z};
}
