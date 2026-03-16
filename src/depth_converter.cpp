#include "tauv_core/depth_converter.h"

DepthConverter::DepthConverter(std::string prefix) : Node("depth_converter"), prefix_(prefix) {
    sub_ = create_subscription<
        tauv_msgs::msg::Depth>("/vehicle/depth",
                                         rclcpp::SensorDataQoS(),
                                         std::bind(&DepthConverter::depthCallback,
                                                   this,
                                                   std::placeholders::_1));

    pub_ = create_publisher<nav_msgs::msg::Odometry>(prefix_ + "/sensors/depth", 10);
}

void DepthConverter::depthCallback(const tauv_msgs::msg::Depth::SharedPtr msg) {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "depth_link";

    odom.pose.pose.position.z = -msg->depth;  // Use the depth directly from the message

    odom.pose.covariance.fill(1e6);
    odom.twist.covariance.fill(1e6);
    odom.pose.covariance[14] = msg->variance;

    pub_->publish(odom);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthConverter>("os");
    rclcpp::spin(node);
    rclcpp::shutdown();
}
