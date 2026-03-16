#include "tauv_core/imu_converter.h"

ImuConverter::ImuConverter(std::string prefix) : Node("imu_converter"), prefix_(prefix) {
    sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu/data",
                                                      rclcpp::SensorDataQoS(),
                                                      std::bind(&ImuConverter::imuCallback,
                                                                this,
                                                                std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::Imu>(prefix_ + "/sensors/imu_xsens", 10);
}

void ImuConverter::imuCallback(sensor_msgs::msg::Imu::SharedPtr msg) const {
    // 1. Orientation Variance (Derived from typical Xsens 0.5 deg Roll/Pitch, 1.0 deg Yaw)
    msg->orientation_covariance = {
        0.000076, 0.0,      0.0,
        0.0,      0.000076, 0.0,
        0.0,      0.0,      0.000300
    };

    // 2. Angular Velocity Variance (From spec sheet: 1.1e-6)
    msg->angular_velocity_covariance = {
        1.1e-6, 0.0,    0.0,
        0.0,    1.1e-6, 0.0,
        0.0,    0.0,    1.1e-6
    };

    // 3. Linear Acceleration Variance (From spec sheet: 1.0e-5)
    msg->linear_acceleration_covariance = {
        1.0e-5, 0.0,    0.0,
        0.0,    1.0e-5, 0.0,
        0.0,    0.0,    1.0e-5
    };

    // Republish the fixed message
    pub_->publish(*msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuConverter>("os");
    rclcpp::spin(node);
    rclcpp::shutdown();
}
