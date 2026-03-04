#pragma once

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <dvl_msgs/msg/dvl.hpp>

class DvlConverter : public rclcpp::Node {
   public:
    DvlConverter(std::string prefix);

   private:
    void dvlCallback(const dvl_msgs::msg::DVL::SharedPtr msg);

    rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_;

    std::string prefix_;
};