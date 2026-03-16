#include "tauv_core/dvl_converter.h"

DvlConverter::DvlConverter(std::string prefix) : Node("dvl_converter"), prefix_(prefix) {
    sub_ = create_subscription<
        dvl_msgs::msg::DVL>("/dvl/data",
                                         rclcpp::SensorDataQoS(),
                                         std::bind(&DvlConverter::dvlCallback,
                                                   this,
                                                   std::placeholders::_1));

    pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(prefix_ + "/sensors/dvl", 10);
}

void DvlConverter::dvlCallback(const dvl_msgs::msg::DVL::SharedPtr msg) {
    geometry_msgs::msg::TwistWithCovarianceStamped twist;

    twist.header = msg->header;
    twist.header.frame_id = "dvl_link";

    twist.twist.twist.linear.x = msg->velocity.x;
    twist.twist.twist.linear.y = msg->velocity.y;
    twist.twist.twist.linear.z = msg->velocity.z;
    twist.twist.twist.angular.x = 0.0;
    twist.twist.twist.angular.y = 0.0;
    twist.twist.twist.angular.z = 0.0;

    // Twist covariance is a 36-element array. Linear velocities are in the top-left 3x3 block.
    twist.twist.covariance.fill(1e6);  // Large default covariance for unmeasured variables
    twist.twist.covariance[0] = msg->covariance[0];
    twist.twist.covariance[1] = msg->covariance[1];
    twist.twist.covariance[2] = msg->covariance[2];
    twist.twist.covariance[6] = msg->covariance[3];
    twist.twist.covariance[7] = msg->covariance[4];
    twist.twist.covariance[8] = msg->covariance[5];
    twist.twist.covariance[12] = msg->covariance[6];
    twist.twist.covariance[13] = msg->covariance[7];
    twist.twist.covariance[14] = msg->covariance[8];

    pub_->publish(twist);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DvlConverter>("os");
    rclcpp::spin(node);
    rclcpp::shutdown();
}
