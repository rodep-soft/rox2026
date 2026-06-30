#include <linux/i2c-dev.h>//ROS2でI2C通信をするためのライブラリ
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <memory>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <utility>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/imu.hpp"  //message型

#include <functional>

class BNO055Node : public rclcpp::Node
{
public :
    BNO055Node()
    :Node("BNO055Node")
    {
        fd_ = open("/dev/i2c-1", O_RDWR);

        if (fd_ < 0) {           
            RCLCPP_FATAL(this->get_logger(), "Cannot open I2C");  //I2Cが開けたかの確認
            rclcpp::shutdown();
            return;
        }

        if (ioctl(fd_, I2C_SLAVE, 0x28) < 0) {                    //BNO055につながったかの確認
            RCLCPP_FATAL(this->get_logger(), "Cannot connect BNO055");
            rclcpp::shutdown();
            return;
        }

        initializeBNO055();


        pub_ = this -> create_publisher <sensor_msgs::msg::Imu>("/imu/data" ,10);

        using namespace std::chrono_literals;

        timer_ = this ->create_wall_timer(10ms, std::bind(&BNO055Node::timer_callback, this));
    }  
    ~BNO055Node() {
        close(fd_);
    }
private :
    void timer_callback()
{
    sensor_msgs::msg::Imu msg;
        //姿勢
        int16_t qw_raw = read16(0x20);
        int16_t qx_raw = read16(0x22);
        int16_t qy_raw = read16(0x24);
        int16_t qz_raw = read16(0x26);
        
        msg.orientation.w = qw_raw / 16384.0;
        msg.orientation.x = qx_raw / 16384.0;
        msg.orientation.y = qy_raw / 16384.0;
        msg.orientation.z = qz_raw / 16384.0;
         
        //角速度(rad/s)
        int16_t gx = read16(0x14);
        int16_t gy = read16(0x16);
        int16_t gz = read16(0x18);

        msg.angular_velocity.x = gx / 900.0;
        msg.angular_velocity.y = gy / 900.0;
        msg.angular_velocity.z = gz / 900.0;

        //加速度(m/s^2)
        int16_t ax = read16(0x28);
        int16_t ay = read16(0x2A);
        int16_t az = read16(0x2C);

        msg.linear_acceleration.x = ax / 100.0;
        msg.linear_acceleration.y = ay / 100.0;
        msg.linear_acceleration.z = az / 100.0;
        
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "imu_link";

        pub_->publish(msg);

}


    void initializeBNO055()
    {
    std::this_thread::sleep_for(std::chrono::milliseconds(700));

    write8(0x3D, 0x0C);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

        void write8(uint8_t reg, uint8_t value) //8bitの書き込み
    {
        uint8_t buf[2] = {reg, value};
        write(fd_, buf, 2);
    }


    int16_t read16(uint8_t reg)
    {
        write(fd_, &reg, 1);

        uint8_t data[2];
        read(fd_, data, 2);

        return (int16_t)((data[1] << 8) | data[0]);
    }

    int fd_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<BNO055Node>());

    rclcpp::shutdown();

    return 0;
}

