#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_localization/ros_filter_types.hpp"

namespace robot_localization {
// EkfComponent wrapper to satisfy RosEkf's requirement for a specific node name in the arguments
// list while maintaining compatibility with rclcpp_components.
class EkfComponent : public RosEkf {
   public:
    explicit EkfComponent(const rclcpp::NodeOptions& options)
        : RosEkf(rclcpp::NodeOptions(options).arguments({"ekf_filter_node"})) {}
};
}  // namespace robot_localization

RCLCPP_COMPONENTS_REGISTER_NODE(robot_localization::EkfComponent)