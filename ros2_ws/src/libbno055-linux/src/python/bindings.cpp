#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "libbno055-linux/bno055.hpp"

namespace py = pybind11;
using namespace bno055lib;

PYBIND11_MODULE(libbno055, m) {
    m.doc() = "Python bindings for libbno055-linux C++ library";

    // OpMode enum
    py::enum_<OpMode>(m, "OpMode")
        .value("Config", OpMode::Config)
        .value("AccOnly", OpMode::AccOnly)
        .value("MagOnly", OpMode::MagOnly)
        .value("GyroOnly", OpMode::GyroOnly)
        .value("AccMag", OpMode::AccMag)
        .value("AccGyro", OpMode::AccGyro)
        .value("MagGyro", OpMode::MagGyro)
        .value("AMG", OpMode::AMG)
        .value("IMUPlus", OpMode::IMUPlus)
        .value("Compass", OpMode::Compass)
        .value("M4G", OpMode::M4G)
        .value("NDOF_FMC_Off", OpMode::NDOF_FMC_Off)
        .value("NDOF", OpMode::NDOF)
        .export_values();

    // AxisMapConfig enum (datasheet Table 3-37 placements P0-P7)
    py::enum_<AxisMapConfig>(m, "AxisMapConfig")
        .value("P0", AxisMapConfig::P0)
        .value("P1", AxisMapConfig::P1)
        .value("P2", AxisMapConfig::P2)
        .value("P3", AxisMapConfig::P3)
        .value("P4", AxisMapConfig::P4)
        .value("P5", AxisMapConfig::P5)
        .value("P6", AxisMapConfig::P6)
        .value("P7", AxisMapConfig::P7)
        .export_values();

    // AxisMapSign enum (datasheet Table 3-37 placements P0-P7)
    py::enum_<AxisMapSign>(m, "AxisMapSign")
        .value("P0", AxisMapSign::P0)
        .value("P1", AxisMapSign::P1)
        .value("P2", AxisMapSign::P2)
        .value("P3", AxisMapSign::P3)
        .value("P4", AxisMapSign::P4)
        .value("P5", AxisMapSign::P5)
        .value("P6", AxisMapSign::P6)
        .value("P7", AxisMapSign::P7)
        .export_values();

    // Vector3 struct
    py::class_<Vector3>(m, "Vector3")
        .def(py::init<float, float, float>(), py::arg("x") = 0.0f, py::arg("y") = 0.0f, py::arg("z") = 0.0f)
        .def_readwrite("x", &Vector3::x)
        .def_readwrite("y", &Vector3::y)
        .def_readwrite("z", &Vector3::z)
        .def("__repr__", [](const Vector3& v) {
            return "<Vector3 x=" + std::to_string(v.x) + " y=" + std::to_string(v.y) + " z=" + std::to_string(v.z) +
                   ">";
        });

    // Quaternion struct
    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<float, float, float, float>(), py::arg("w") = 1.0f, py::arg("x") = 0.0f, py::arg("y") = 0.0f,
             py::arg("z") = 0.0f)
        .def_readwrite("w", &Quaternion::w)
        .def_readwrite("x", &Quaternion::x)
        .def_readwrite("y", &Quaternion::y)
        .def_readwrite("z", &Quaternion::z)
        .def("__repr__", [](const Quaternion& q) {
            return "<Quaternion w=" + std::to_string(q.w) + " x=" + std::to_string(q.x) + " y=" + std::to_string(q.y) +
                   " z=" + std::to_string(q.z) + ">";
        });

    // CalibrationStatus struct
    py::class_<CalibrationStatus>(m, "CalibrationStatus")
        .def_readwrite("sys", &CalibrationStatus::sys)
        .def_readwrite("gyro", &CalibrationStatus::gyro)
        .def_readwrite("accel", &CalibrationStatus::accel)
        .def_readwrite("mag", &CalibrationStatus::mag)
        .def("is_fully_calibrated", &CalibrationStatus::isFullyCalibrated)
        .def("__repr__", [](const CalibrationStatus& c) {
            return "<CalibrationStatus sys=" + std::to_string(c.sys) + " gyro=" + std::to_string(c.gyro) +
                   " accel=" + std::to_string(c.accel) + " mag=" + std::to_string(c.mag) + ">";
        });

    // Diagnostics struct
    py::class_<Diagnostics>(m, "Diagnostics")
        .def_readwrite("write_failures", &Diagnostics::write_failures)
        .def_readwrite("read_failures", &Diagnostics::read_failures)
        .def_readwrite("reconnect_attempts", &Diagnostics::reconnect_attempts)
        .def("__repr__", [](const Diagnostics& d) {
            return "<Diagnostics write_failures=" + std::to_string(d.write_failures) +
                   " read_failures=" + std::to_string(d.read_failures) +
                   " reconnect_attempts=" + std::to_string(d.reconnect_attempts) + ">";
        });

    // RawSensorData struct
    py::class_<BNO055::RawSensorData>(m, "RawSensorData")
        .def_readwrite("accel", &BNO055::RawSensorData::accel)
        .def_readwrite("mag", &BNO055::RawSensorData::mag)
        .def_readwrite("gyro", &BNO055::RawSensorData::gyro)
        .def("__repr__", [](const BNO055::RawSensorData& r) {
            return "<RawSensorData accel=(" + std::to_string(r.accel.x) + "," + std::to_string(r.accel.y) + "," +
                   std::to_string(r.accel.z) + ") mag=(" + std::to_string(r.mag.x) + "," + std::to_string(r.mag.y) +
                   "," + std::to_string(r.mag.z) + ") gyro=(" + std::to_string(r.gyro.x) + "," +
                   std::to_string(r.gyro.y) + "," + std::to_string(r.gyro.z) + ")>";
        });

    // AllData struct — full snapshot of all sensor outputs from a single burst read
    py::class_<BNO055::AllData>(m, "AllData")
        .def(py::init<>())
        .def_readwrite("accel", &BNO055::AllData::accel)
        .def_readwrite("mag", &BNO055::AllData::mag)
        .def_readwrite("gyro", &BNO055::AllData::gyro)
        .def_readwrite("euler", &BNO055::AllData::euler)
        .def_readwrite("linear_accel", &BNO055::AllData::linear_accel)
        .def_readwrite("gravity", &BNO055::AllData::gravity)
        .def_readwrite("quat", &BNO055::AllData::quat)
        .def_readwrite("temp", &BNO055::AllData::temp)
        .def("__repr__", [](const BNO055::AllData& d) {
            return "<AllData accel=(" + std::to_string(d.accel.x) + "," + std::to_string(d.accel.y) + "," +
                   std::to_string(d.accel.z) + ") temp=" + std::to_string(static_cast<int>(d.temp)) + ">";
        });

    // BNO055 class
    py::class_<BNO055>(m, "BNO055")
        .def(py::init<uint8_t, std::string_view>(), py::arg("address") = 0x28, py::arg("device") = "/dev/i2c-1")
        .def("begin", &BNO055::begin, py::arg("mode") = OpMode::NDOF)
        .def("reset", &BNO055::reset)
        .def("set_mode", &BNO055::setMode, py::arg("mode"))
        .def("get_mode", &BNO055::getMode)
        .def("set_ext_crystal_use", &BNO055::setExtCrystalUse, py::arg("use_xtal"))
        .def("set_axis_remap", &BNO055::setAxisRemap, py::arg("config"),
             "Set axis remapping (P0-P7 per datasheet Table 3-37). Must call in CONFIGMODE or will auto-switch.")
        .def("set_axis_sign", &BNO055::setAxisSign, py::arg("sign"),
             "Set axis sign remapping (P0-P7 per datasheet Table 3-37).")
        // Sensor data getters (noexcept std::optional bindings)
        .def("get_accelerometer", &BNO055::getAccelerometerNoexcept)
        .def("get_magnetometer", &BNO055::getMagnetometerNoexcept)
        .def("get_gyroscope", &BNO055::getGyroscopeNoexcept)
        .def("get_euler_angles", &BNO055::getEulerAnglesNoexcept)
        .def("get_linear_acceleration", &BNO055::getLinearAccelerationNoexcept)
        .def("get_gravity", &BNO055::getGravityNoexcept)
        .def("get_quaternion", &BNO055::getQuaternionNoexcept)
        .def("get_temperature", &BNO055::getTemperatureNoexcept)
        .def("get_raw_sensor_data", &BNO055::getRawSensorDataNoexcept)
        .def("get_all_data", &BNO055::getAllDataNoexcept,
             "Single 45-byte burst read returning all sensor outputs atomically. "
             "8x fewer I2C transactions vs reading sensors individually.")
        // Calibration & Diagnostics
        .def("get_calibration_status",
             [](BNO055& self) {
                 try {
                     return std::optional<CalibrationStatus>(self.getCalibrationStatus());
                 } catch (...) {
                     return std::optional<CalibrationStatus>(std::nullopt);
                 }
             })
        .def("get_diagnostics", &BNO055::getDiagnostics)
        .def("save_calibration_file", &BNO055::saveCalibrationFile, py::arg("filepath"))
        .def("load_calibration_file", &BNO055::loadCalibrationFile, py::arg("filepath"))
        .def("enable_auto_calibration", &BNO055::enableAutoCalibration, py::arg("filepath"))
        .def("disable_auto_calibration", &BNO055::disableAutoCalibration)
        .def("get_sensor_offsets",
             [](BNO055& self) {
                 std::array<uint8_t, 22> calib_data;
                 if (self.getSensorOffsets(calib_data)) {
                     return calib_data;
                 }
                 throw std::runtime_error("Failed to get sensor offsets");
             })
        .def(
            "set_sensor_offsets",
            [](BNO055& self, const std::array<uint8_t, 22>& data) { self.setSensorOffsets(data); }, py::arg("data"))
        .def("enter_suspend_mode",
             [](BNO055& self) {
                 try {
                     self.enterSuspendMode();
                 } catch (...) {
                 }
             })
        .def("enter_normal_mode",
             [](BNO055& self) {
                 try {
                     self.enterNormalMode();
                 } catch (...) {
                 }
             })
        // ------------------------------------------------------------------
        // Async / Interrupt-driven reading APIs
        // NOTE: Callbacks are invoked from a C++ background thread.
        //       GIL is acquired inside the lambda before calling Python.
        // ------------------------------------------------------------------
        .def(
            "start_async_reading",
            [](BNO055& self, double rate_hz, py::object callback) -> bool {
                auto py_cb = std::make_shared<py::object>(std::move(callback));
                return self.startAsyncReading(rate_hz, [py_cb](const BNO055::AllData& data) {
                    py::gil_scoped_acquire acquire;
                    try {
                        (*py_cb)(data);
                    } catch (py::error_already_set& e) {
                        e.restore();
                    }
                });
            },
            py::arg("rate_hz"), py::arg("callback"), py::call_guard<py::gil_scoped_release>(),
            "Start background async polling at rate_hz Hz. Callback(AllData) called from C++ thread with GIL held.")
        .def("stop_async_reading", &BNO055::stopAsyncReading, "Stop the background async reading thread.")
        .def(
            "start_raw_async_reading",
            [](BNO055& self, double rate_hz, py::object callback) -> bool {
                auto py_cb = std::make_shared<py::object>(std::move(callback));
                return self.startRawAsyncReading(rate_hz, [py_cb](const BNO055::RawSensorData& data) {
                    py::gil_scoped_acquire acquire;
                    try {
                        (*py_cb)(data);
                    } catch (py::error_already_set& e) {
                        e.restore();
                    }
                });
            },
            py::arg("rate_hz"), py::arg("callback"), py::call_guard<py::gil_scoped_release>(),
            "Start background raw burst async reading at rate_hz Hz. Callback(RawSensorData) called from C++ thread.")
        .def("stop_raw_async_reading", &BNO055::stopRawAsyncReading, "Stop the background raw async reading thread.")
        .def(
            "start_interrupt_driven_reading",
            [](BNO055& self, int gpio_pin, py::object callback) -> bool {
                auto py_cb = std::make_shared<py::object>(std::move(callback));
                return self.startInterruptDrivenReading(gpio_pin, [py_cb](const BNO055::RawSensorData& data) {
                    py::gil_scoped_acquire acquire;
                    try {
                        (*py_cb)(data);
                    } catch (py::error_already_set& e) {
                        e.restore();
                    }
                });
            },
            py::arg("gpio_pin"), py::arg("callback"), py::call_guard<py::gil_scoped_release>(),
            "Start GPIO IRQ-driven reading on gpio_pin (libgpiod). Callback(RawSensorData) fires on rising edge.")
        .def("stop_interrupt_driven_reading", &BNO055::stopInterruptDrivenReading,
             "Stop the GPIO IRQ-driven reading thread.");

    // Utility function
    m.def("to_euler_degrees", &toEulerDegrees, py::arg("q"), "Convert Quaternion to Euler angles in degrees");
}
