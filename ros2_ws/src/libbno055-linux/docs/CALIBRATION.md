# BNO055 Calibration Best Practices

Calibration is one of the most critical and often misunderstood aspects of using the BNO055 IMU in production robotics (e.g., drones, autonomous rovers).

While the sensor performs background auto-calibration, writing these values to a persistent file during high-speed control loops can cause severe latency spikes (jitter) or even crash the I2C bus.

To avoid these "deadlocks" and ensure real-time performance, `libbno055-linux` strongly recommends the following **Pro Workflow**.

## The Pro Workflow

### 1. Disable Auto-Save in Flight
Do **not** use `enableAutoCalibration()` during actual autonomous operation. 
Saving 22 bytes of calibration data to the Linux filesystem requires blocking the I2C bus, acquiring thread mutexes, and executing slow disk I/O. This can block the sensor reading thread for several milliseconds, throwing off your EKF (Extended Kalman Filter) and causing erratic robot behavior.

### 2. Use a Master Calibration File

Instead of saving on the fly, rely on a pre-saved "master" calibration file.

#### Offline (Maintenance Phase)
During maintenance or robot setup, physically calibrate the robot:
1. **Gyroscope**: Keep the robot perfectly still.
2. **Accelerometer**: Rest the robot on different faces (if possible), or keep it still.
3. **Magnetometer**: Move the robot in a figure-8 motion to sample the local magnetic field.

Once the sensor reports it is fully calibrated (`isFullyCalibrated() == true`), manually trigger the save function to write `master_calib.bin`. 
* **C++**: Call `imu.saveCalibrationFile("master_calib.bin")`.
* **Python**: Call `imu.save_calibration_file("master_calib.bin")`.
* **ROS 2**: Call the `~/save_calibration` service (`ros2 service call /bno055/save_calibration std_srvs/srv/Trigger`).

#### Online (Production Phase)
On production startup, simply load the master file into the sensor. The BNO055 will use this as a highly accurate baseline.
* **C++**: Call `imu.loadCalibrationFile("master_calib.bin")`.
* **Python**: Call `imu.load_calibration_file("master_calib.bin")`.
* **ROS 2**: Set the `calibration_file` parameter in your launch file or `params.yaml`. The node will automatically load it on startup.

### 3. Never Block on Calibration During Startup
After loading the master calibration file, wait a few seconds for the gyroscope to stabilize (which happens automatically while the robot is stationary on the ground).
**Immediately start your control loop.** 

*Do not wait for the magnetometer status to reach `3/3` before taking off or driving.* Local magnetic interference (e.g., metal rebar in the floor, motors) might temporarily lower the reported magnetometer calibration status. As long as you loaded a valid master file, the sensor's fusion algorithm will perform optimally.

## Reading Offsets Manually
If you need to read or write the 22-byte offset arrays manually, `libbno055-linux` provides thread-safe access:
```cpp
// C++
auto offsets = imu.getSensorOffsets();
// ...
imu.setSensorOffsets(offsets);
```
```python
# Python
offsets = imu.get_sensor_offsets() # Returns a list of 22 bytes
# ...
imu.set_sensor_offsets(offsets)
```
The driver automatically handles the strict requirement of temporarily switching to `CONFIG` mode to access the offset registers, meaning you don't have to write error-prone sleep or mode-switching logic yourself.
