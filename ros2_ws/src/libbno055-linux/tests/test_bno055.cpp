#include <gtest/gtest.h>

#include <cmath>
#include <condition_variable>
#include <mutex>

#include "libbno055-linux/bno055.hpp"
#include "libbno055-linux/mock_transport.hpp"

class BNO055MockTest : public ::testing::Test {
protected:
    void SetUp() override {
        auto transport = std::make_unique<bno055lib::MockTransport>();
        mock_ = transport.get();  // Keep raw pointer to control mock
        imu_ = std::make_unique<bno055lib::BNO055>(std::move(transport));
        ASSERT_TRUE(imu_->begin(bno055lib::OpMode::NDOF));
    }
    bno055lib::MockTransport* mock_{nullptr};
    std::unique_ptr<bno055lib::BNO055> imu_;
};

TEST_F(BNO055MockTest, ConstructorAndInitialization) {
    // Check initialization status
    EXPECT_EQ(mock_->getRegister(0x3D), static_cast<uint8_t>(bno055lib::OpMode::NDOF));

    // Initial diagnostics check
    auto diag = imu_->getDiagnostics();
    EXPECT_EQ(diag.write_failures, 0u);
    EXPECT_EQ(diag.read_failures, 0u);
    EXPECT_EQ(diag.reconnect_attempts, 0u);
}

TEST_F(BNO055MockTest, SensorDataReading) {
    // 1. Accelerometer (Scale: 1 m/s^2 = 100 LSB)
    mock_->setRegister16LE(0x08, 100);   // X
    mock_->setRegister16LE(0x0A, -200);  // Y
    mock_->setRegister16LE(0x0C, 300);   // Z
    auto accel = imu_->getAccelerometer();
    EXPECT_NEAR(accel.x, 1.0, 1e-4);
    EXPECT_NEAR(accel.y, -2.0, 1e-4);
    EXPECT_NEAR(accel.z, 3.0, 1e-4);

    // 2. Magnetometer (Scale: 1 uT = 16 LSB)
    mock_->setRegister16LE(0x0E, 16);   // X
    mock_->setRegister16LE(0x10, -32);  // Y
    mock_->setRegister16LE(0x12, 48);   // Z
    auto mag = imu_->getMagnetometer();
    EXPECT_NEAR(mag.x, 1.0, 1e-4);
    EXPECT_NEAR(mag.y, -2.0, 1e-4);
    EXPECT_NEAR(mag.z, 3.0, 1e-4);

    // 3. Gyroscope (Scale: 1 dps = 16 LSB. Convert to rad/s)
    double gyro_scale = (1.0 / 16.0) * (M_PI / 180.0);
    mock_->setRegister16LE(0x14, 16);   // X
    mock_->setRegister16LE(0x16, -32);  // Y
    mock_->setRegister16LE(0x18, 48);   // Z
    auto gyro = imu_->getGyroscope();
    EXPECT_NEAR(gyro.x, 16 * gyro_scale, 1e-4);
    EXPECT_NEAR(gyro.y, -32 * gyro_scale, 1e-4);
    EXPECT_NEAR(gyro.z, 48 * gyro_scale, 1e-4);

    // 4. Quaternion (Scale: 1 = 16384 LSB)
    mock_->setRegister16LE(0x20, 16384);  // W
    mock_->setRegister16LE(0x22, 0);      // X
    mock_->setRegister16LE(0x24, 0);      // Y
    mock_->setRegister16LE(0x26, 0);      // Z
    auto quat = imu_->getQuaternion();
    EXPECT_NEAR(quat.w, 1.0, 1e-4);
    EXPECT_NEAR(quat.x, 0.0, 1e-4);
    EXPECT_NEAR(quat.y, 0.0, 1e-4);
    EXPECT_NEAR(quat.z, 0.0, 1e-4);

    // 5. Temperature
    mock_->setRegister(0x34, 25);
    EXPECT_EQ(imu_->getTemperature(), 25);
}

TEST_F(BNO055MockTest, NoexceptGetters) {
    // Success path
    mock_->setRegister(0x34, 28);
    auto temp_opt = imu_->getTemperatureNoexcept();
    ASSERT_TRUE(temp_opt.has_value());
    EXPECT_EQ(*temp_opt, 28);

    // Failure path
    mock_->setFailReads(true);
    auto temp_opt_fail = imu_->getTemperatureNoexcept();
    EXPECT_FALSE(temp_opt_fail.has_value());
}

TEST_F(BNO055MockTest, OrDefaultGetters) {
    // Failure path should return defaults
    mock_->setFailReads(true);
    auto accel = imu_->getAccelerometerOrDefault();
    EXPECT_EQ(accel.x, 0.0);
    EXPECT_EQ(accel.y, 0.0);
    EXPECT_EQ(accel.z, 0.0);

    auto quat = imu_->getQuaternionOrDefault();
    EXPECT_EQ(quat.w, 1.0);
    EXPECT_EQ(quat.x, 0.0);
    EXPECT_EQ(quat.y, 0.0);
    EXPECT_EQ(quat.z, 0.0);
}

TEST_F(BNO055MockTest, ThrowingGetters) {
    mock_->setFailReads(true);
    EXPECT_THROW(imu_->getAccelerometer(), bno055lib::IMUError);
}

TEST_F(BNO055MockTest, CalibrationStatus) {
    // Set SYS=3, GYRO=2, ACCEL=1, MAG=0 => 0b11100100 = 0xE4
    mock_->setRegister(0x35, 0xE4);
    auto calib = imu_->getCalibrationStatus();
    EXPECT_EQ(calib.sys, 3);
    EXPECT_EQ(calib.gyro, 2);
    EXPECT_EQ(calib.accel, 1);
    EXPECT_EQ(calib.mag, 0);
    EXPECT_FALSE(calib.isFullyCalibrated());

    // Fully calibrated
    mock_->setRegister(0x35, 0xFF);
    calib = imu_->getCalibrationStatus();
    EXPECT_TRUE(calib.isFullyCalibrated());
}

TEST_F(BNO055MockTest, ModeConfiguration) {
    imu_->setMode(bno055lib::OpMode::IMUPlus);
    EXPECT_EQ(mock_->getRegister(0x3D), static_cast<uint8_t>(bno055lib::OpMode::IMUPlus));
    EXPECT_EQ(imu_->getMode(), bno055lib::OpMode::IMUPlus);
}

TEST_F(BNO055MockTest, AxisRemap) {
    imu_->setAxisRemap(bno055lib::AxisMapConfig::P0);
    EXPECT_EQ(mock_->getRegister(0x41), static_cast<uint8_t>(bno055lib::AxisMapConfig::P0));

    imu_->setAxisSign(bno055lib::AxisMapSign::P3);
    EXPECT_EQ(mock_->getRegister(0x42), static_cast<uint8_t>(bno055lib::AxisMapSign::P3));
}

TEST_F(BNO055MockTest, ExtCrystalUse) {
    imu_->setExtCrystalUse(true);
    EXPECT_EQ(mock_->getRegister(0x3F), 0x80);  // SYS_TRIGGER bit 7 should be set

    imu_->setExtCrystalUse(false);
    EXPECT_EQ(mock_->getRegister(0x3F), 0x00);
}

TEST_F(BNO055MockTest, SensorOffsets) {
    bno055lib::Offsets offsets;
    offsets.accel_offset_x = 10;
    offsets.accel_offset_y = -20;
    offsets.accel_offset_z = 30;
    offsets.mag_offset_x = 40;
    offsets.mag_offset_y = -50;
    offsets.mag_offset_z = 60;
    offsets.gyro_offset_x = 70;
    offsets.gyro_offset_y = -80;
    offsets.gyro_offset_z = 90;
    offsets.accel_radius = 1000;
    offsets.mag_radius = 2000;

    imu_->setSensorOffsets(offsets);

    bno055lib::Offsets read_back;
    ASSERT_TRUE(imu_->getSensorOffsets(read_back));
    EXPECT_EQ(read_back.accel_offset_x, 10);
    EXPECT_EQ(read_back.accel_offset_y, -20);
    EXPECT_EQ(read_back.accel_offset_z, 30);
    EXPECT_EQ(read_back.mag_offset_x, 40);
    EXPECT_EQ(read_back.mag_offset_y, -50);
    EXPECT_EQ(read_back.mag_offset_z, 60);
    EXPECT_EQ(read_back.gyro_offset_x, 70);
    EXPECT_EQ(read_back.gyro_offset_y, -80);
    EXPECT_EQ(read_back.gyro_offset_z, 90);
    EXPECT_EQ(read_back.accel_radius, 1000);
    EXPECT_EQ(read_back.mag_radius, 2000);
}

TEST(BNO055Test, VectorBoundsChecking) {
    bno055lib::Vector3 vec{1.0, 2.0, 3.0};
    EXPECT_EQ(vec[0], 1.0);
    EXPECT_EQ(vec[1], 2.0);
    EXPECT_EQ(vec[2], 3.0);
    EXPECT_THROW(vec[3], std::out_of_range);

    vec[0] = 10.0;
    EXPECT_EQ(vec[0], 10.0);
    EXPECT_THROW(vec[3] = 4.0, std::out_of_range);
}

TEST(BNO055Test, QuaternionToEulerDegrees) {
    bno055lib::Quaternion q_rot;
    // Rotate 90 degrees around Yaw (Z-axis)
    q_rot.w = std::cos(M_PI / 4.0);
    q_rot.x = 0.0;
    q_rot.y = 0.0;
    q_rot.z = std::sin(M_PI / 4.0);

    auto euler = bno055lib::toEulerDegrees(q_rot);
    EXPECT_NEAR(euler.x, 0.0, 1e-4);   // Roll
    EXPECT_NEAR(euler.y, 0.0, 1e-4);   // Pitch
    EXPECT_NEAR(euler.z, 90.0, 1e-4);  // Yaw
}

TEST_F(BNO055MockTest, AsyncReadingAndAutoCalibration) {
    // 1. Configure registers for valid data
    mock_->setRegister16LE(0x08, 100);  // Accel X
    mock_->setRegister(0x34, 25);       // Temp

    std::mutex mtx;
    std::condition_variable cv;
    bool data_received = false;
    bno055lib::BNO055::AllData received_data;

    // Start async polling
    bool success = imu_->startAsyncReading(100.0, [&](const bno055lib::BNO055::AllData& data) {
        std::lock_guard<std::mutex> lock(mtx);
        received_data = data;
        data_received = true;
        cv.notify_one();
    });

    ASSERT_TRUE(success);

    // Wait for callback to execute
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait_for(lock, std::chrono::milliseconds(200), [&]() { return data_received; });

    EXPECT_TRUE(data_received);
    EXPECT_NEAR(received_data.accel.x, 1.0, 1e-4);
    EXPECT_EQ(received_data.temp, 25);

    // Stop async reading
    imu_->stopAsyncReading();

    // 2. Test auto-calibration configuration methods
    imu_->enableAutoCalibration("/tmp/test_auto_calib.bin");
    imu_->disableAutoCalibration();
}

TEST_F(BNO055MockTest, EKFRawBurstReadingAndAsync) {
    // Set Accelerometer (0x08), Magnetometer (0x0E), Gyroscope (0x14)
    mock_->setRegister16LE(0x08, 100);  // Accel X (1.0 m/s^2)
    mock_->setRegister16LE(0x0E, 16);   // Mag X (1.0 uT)
    mock_->setRegister16LE(0x14, 16);   // Gyro X (1 dps -> converted to rad/s)

    // Verify burst read
    auto raw_data = imu_->getRawSensorData();
    EXPECT_NEAR(raw_data.accel.x, 1.0f, 1e-4);
    EXPECT_NEAR(raw_data.mag.x, 1.0f, 1e-4);
    double gyro_scale = (1.0 / 16.0) * (M_PI / 180.0);
    EXPECT_NEAR(raw_data.gyro.x, static_cast<float>(16 * gyro_scale), 1e-4);

    // Verify raw async reader
    std::mutex mtx;
    std::condition_variable cv;
    bool data_received = false;
    bno055lib::BNO055::RawSensorData async_raw;

    bool success = imu_->startRawAsyncReading(100.0, [&](const bno055lib::BNO055::RawSensorData& data) {
        std::lock_guard<std::mutex> lock(mtx);
        async_raw = data;
        data_received = true;
        cv.notify_one();
    });

    ASSERT_TRUE(success);

    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait_for(lock, std::chrono::milliseconds(200), [&]() { return data_received; });

        EXPECT_TRUE(data_received);
        EXPECT_NEAR(async_raw.accel.x, 1.0f, 1e-4);
        EXPECT_NEAR(async_raw.mag.x, 1.0f, 1e-4);
    }

    imu_->stopRawAsyncReading();

    // Verify GPIO Interrupt driven mode (mock fallback mode)
    data_received = false;
    success = imu_->startInterruptDrivenReading(24, [&](const bno055lib::BNO055::RawSensorData& data) {
        std::lock_guard<std::mutex> lock(mtx);
        async_raw = data;
        data_received = true;
        cv.notify_one();
    });

    ASSERT_TRUE(success);

    {
        std::unique_lock<std::mutex> lock2(mtx);
        cv.wait_for(lock2, std::chrono::milliseconds(200), [&]() { return data_received; });

        EXPECT_TRUE(data_received);
        EXPECT_NEAR(async_raw.accel.x, 1.0f, 1e-4);
    }

    imu_->stopInterruptDrivenReading();
}

TEST_F(BNO055MockTest, HardwareOverclockingInAMGMode) {
    // Reinitialize a fresh MockTransport and IMU in AMG Mode to trigger overclocking
    auto local_transport = std::make_unique<bno055lib::MockTransport>();
    auto* local_mock = local_transport.get();

    bool page1_selected = false;
    bool accel_overclocked = false;
    bool gyro_overclocked = false;

    local_mock->setOnWrite([&](uint8_t reg, uint8_t value) {
        if (reg == 0x07) {  // PAGE_ID NOLINT(readability-magic-numbers)
            if (value == 1)
                page1_selected = true;
            else if (value == 0)
                page1_selected = false;
        }
        if (page1_selected) {
            if (reg == 0x08 && value == 0x1D) {  // ACC_CONFIG 1000Hz BW + ±4g (datasheet Tables 4-3, 4-4)
                accel_overclocked = true;
            }
            if (reg == 0x0A && value == 0x00) {  // GYR_CONFIG_0 NOLINT(readability-magic-numbers)
                gyro_overclocked = true;
            }
        }
    });

    // Boot IMU in AMG Mode
    bno055lib::BNO055 amg_imu(std::move(local_transport));
    ASSERT_TRUE(amg_imu.begin(bno055lib::OpMode::AMG));

    EXPECT_TRUE(accel_overclocked);
    EXPECT_TRUE(gyro_overclocked);
}

#include "libbno055-linux/bno055_c.h"

TEST(BNO055CAPITest, HandlesAndUtilities) {
    bno055_handle_t handle = bno055_create_i2c(0x28, "mock_device");
    ASSERT_NE(handle, nullptr);

    // Begin IMU in NDOF mode
    bno055_begin(handle, BNO055_OPMODE_NDOF);

    bno055_quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};
    bno055_vector3_t euler = bno055_to_euler_degrees(&q);
    EXPECT_FLOAT_EQ(euler.x, 0.0f);
    EXPECT_FLOAT_EQ(euler.y, 0.0f);
    EXPECT_FLOAT_EQ(euler.z, 0.0f);

    bno055_destroy(handle);
}
