import time
import libbno055

def main():
    print("Initializing BNO055 via Python...")

    # Initialize IMU via I2C (Address 0x28, /dev/i2c-1)
    imu = libbno055.BNO055(address=0x28, device="/dev/i2c-1")

    # OR Initialize via USB-to-UART bridge
    # imu = libbno055.BNO055(port="/dev/ttyUSB0", baudrate=115200, timeout=0.1)

    # Boot sensor in NDOF 9-DOF Fusion mode
    if not imu.begin(libbno055.OpMode.NDOF):
        print("Error: BNO055 initialization failed. Check wiring and permissions.")
        return

    print("BNO055 Initialized Successfully! Reading data...")
    print("-" * 50)

    try:
        # Loop to read data periodically
        for i in range(10):
            # Read Orientation Quaternion & Convert to Euler Degrees
            q = imu.get_quaternion()
            if q:
                euler = libbno055.to_euler_degrees(q)
                print(f"[Angle] Roll: {euler.x:>7.2f}, Pitch: {euler.y:>7.2f}, Yaw: {euler.z:>7.2f}")

            # Read Linear Acceleration (Acceleration minus gravity)
            lin_acc = imu.get_linear_acceleration()
            if lin_acc:
                print(f"[Accel] X: {lin_acc.x:>7.2f}, Y: {lin_acc.y:>7.2f}, Z: {lin_acc.z:>7.2f} (m/s^2)")

            # Check Calibration Status (0 = Uncalibrated, 3 = Fully Calibrated)
            calib = imu.get_calibration_status()
            print(f"[Calib] SYS:{calib.sys}, GYR:{calib.gyro}, ACC:{calib.accel}, MAG:{calib.mag}")
            
            print("-" * 50)
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    # Check Telemetry Health Diagnostics (Useful for debugging I2C/UART issues)
    diag = imu.get_diagnostics()
    print(f"\n[Diagnostics] Reconnects: {diag.reconnect_attempts}, "
          f"Write Errors: {diag.write_failures}, Read Errors: {diag.read_failures}")

if __name__ == "__main__":
    main()
