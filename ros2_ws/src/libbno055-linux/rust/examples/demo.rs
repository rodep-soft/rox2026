use libbno055::{BNO055, OpMode};

fn main() -> Result<(), &'static str> {
    println!("Initializing BNO055 via Rust crate...");

    // Initialize IMU via I2C (Address 0x28, /dev/i2c-1)
    let mut imu = BNO055::new_i2c(0x28, "/dev/i2c-1")?;

    // OR Initialize via USB-to-UART bridge
    // let mut imu = BNO055::new_uart("/dev/ttyUSB0", 115200)?;

    // Boot sensor in NDOF 9-DOF Fusion mode
    if imu.begin(OpMode::NDOF) {
        println!("BNO055 Initialized Successfully!");

        // Read Orientation Quaternion & Convert to Euler Degrees
        if let Some(q) = imu.get_quaternion() {
            let euler = BNO055::to_euler_degrees(&q);
            println!("Euler (deg): Roll={:.2}, Pitch={:.2}, Yaw={:.2}", euler.x, euler.y, euler.z);
        }

        // Read Accelerometer
        if let Some(acc) = imu.get_accelerometer() {
            println!("Accel (m/s^2): X={:.2}, Y={:.2}, Z={:.2}", acc.x, acc.y, acc.z);
        }

        // Check Calibration Status (0 = Uncalibrated, 3 = Fully Calibrated)
        if let Some(calib) = imu.get_calibration_status() {
            println!("Calibration: SYS={}, GYRO={}, ACCEL={}, MAG={}",
                     calib.sys, calib.gyro, calib.accel, calib.mag);
            println!("Is Fully Calibrated? {}", calib.is_fully_calibrated());
        }

        // Check Telemetry Health Diagnostics
        let diag = imu.get_diagnostics();
        println!("Telemetry: Reconnects={}, WriteFailures={}, ReadFailures={}",
                 diag.reconnect_attempts, diag.write_failures, diag.read_failures);
    } else {
        println!("Initialization failed (Device missing or unreadable).");
    }

    Ok(())
}
