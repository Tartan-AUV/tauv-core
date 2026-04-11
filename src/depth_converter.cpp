/*
 * Converts the output of the depth sensor into a Z position
 * and publishes it to the Odometry node
*/

#include "tauv_core/depth_converter.h"

// Initializes the DepthConverter node:
// - Subscribes to raw depth data
// - Publishes converted odometry messages
DepthConverter::DepthConverter(std::string prefix) : Node("depth_converter"), prefix_(prefix) {
     // Subscribe to raw depth measurements from the vehicle
    sub_ = create_subscription<
        tauv_msgs::msg::Depth>("/vehicle/depth",
                                         rclcpp::SensorDataQoS(),
                                         std::bind(&DepthConverter::depthCallback,
                                                   this,
                                                   std::placeholders::_1));
    // Publish depth as a Z-position in Odometry format
    pub_ = create_publisher<nav_msgs::msg::Odometry>(prefix_ + "/sensors/depth", 10);
}

// Converts a depth message into an Odometry message:
// - Maps depth -> Z position
// - Assigns covariance for use in state estimation
void DepthConverter::depthCallback(const tauv_msgs::msg::Depth::SharedPtr msg) {
    // Create a new odometry message to store converted data
    nav_msgs::msg::Odometry odom;

     // Copy timestamp and define coordinate frames
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "depth_link";

    // Convert depth (positive down) to ROS Z (positive up)
    odom.pose.pose.position.z = -msg->depth;  // Use the depth directly from the message

    // Fills covariances
    odom.pose.covariance.fill(1e6);
    odom.twist.covariance.fill(1e6);
    odom.pose.covariance[14] = msg->variance;

    // Publish the converted odometry message
    pub_->publish(odom);
}

// Entry point: initializes ROS, creates the node, and starts spinning
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthConverter>("os");
    rclcpp::spin(node);
    rclcpp::shutdown();
}
