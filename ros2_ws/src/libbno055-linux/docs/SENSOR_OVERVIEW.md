# Sensor Overview and Operation Modes

The Bosch BNO055 is a System-in-Package (SiP) integrating a triaxial accelerometer, a triaxial gyroscope, a triaxial geomagnetic sensor, and a 32-bit ARM Cortex-M0 microcontroller running Bosch Sensortec sensor fusion software. 

Selecting the appropriate Operation Mode (OpMode) is critical for the stability of your estimation loops.

## Key Fusion Modes

*   **NDOF (9-DoF Fusion)**: Uses accelerometer, gyroscope, and magnetometer. It outputs absolute orientation relative to the Earth's magnetic field (Yaw is referenced to magnetic North). This mode is suitable for outdoor navigation but highly susceptible to magnetic interference (distortion from iron structures, electric motors, or wiring).
*   **IMUPlus (6-DoF Fusion)**: Uses accelerometer and gyroscope only. It outputs relative orientation (Yaw starts at 0 on boot and will slowly drift over time). This mode is highly recommended for indoor robotics, autonomous mobile robots (AMRs), and industrial environments where magnetic disturbances are constant.
*   **AMG (Non-fusion Raw Mode)**: Bypasses the internal fusion processor and outputs raw sensor readings from the Accelerometer, Magnetometer, and Gyroscope. Use this mode if you intend to implement custom state estimation filters (such as EKF or complementary filters) on the host CPU.

## Orientation Formats (Quaternion and Euler Angles)

The library provides two formats for retrieving the 3D orientation computed by the sensor:

*   **Quaternions (De-facto Standard for Robotics)**: Highly recommended for robotics applications (such as ROS 2 navigation, TF2 transforms, and state estimation) to avoid gimbal lock. The BNO055 internal fusion coprocessor computes unit quaternions (w, x, y, z) at 100Hz. The library automatically normalizes this data to a unit quaternion format, making it directly compatible with ROS 2 geometry_msgs/msg/Quaternion and sensor_msgs/msg/Imu messages.
*   **Euler Angles**: Convenient for human-readable display or simpler projects. The library returns Roll, Pitch, and Yaw in radians via a Vector3 struct (where x = Roll, y = Pitch, and z = Yaw).

## Sensor Calibration

The BNO055 calibrates itself dynamically in the background. The calibration status for each sensor ranges from 0 (uncalibrated) to 3 (fully calibrated). To achieve full calibration:
1.  **Gyroscope**: Keep the sensor completely still in a stable position for a few seconds.
2.  **Magnetometer**: Move the sensor in a figure-8 pattern through the air.
3.  **Accelerometer**: Rotate the sensor into 6 different stable positions, holding it still for a few seconds in each orientation (similar to placing a cube on each of its 6 faces).
