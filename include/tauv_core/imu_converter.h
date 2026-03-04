#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class ImuConverter : public rclcpp::Node {
   public:
    ImuConverter(std::string prefix);

   private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) const;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

    std::string prefix_;
};
