#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "robot_localization/ros_filter_types.hpp"

namespace robot_localization {
class EkfComponent : public RosEkf {
   public:
    explicit EkfComponent(const rclcpp::NodeOptions& options) : RosEkf(patch_options(options)) {
        // Robot Localization requires us to call an initialize method. To ensure this happens after
        // the constructor, we jankily add a timer for 1ms later that calls the initialize method
        // and then cancels itself so it only runs once.
        init_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
            this->init_timer_->cancel();  // Cancel immediately so it only runs once
            this->initialize();
        });
    }

   private:
    rclcpp::TimerBase::SharedPtr init_timer_;

    static rclcpp::NodeOptions patch_options(const rclcpp::NodeOptions& options) {
        rclcpp::NodeOptions patched_options = options;
        std::vector<std::string> args = options.arguments();

        // Satisfy the legacy name check while keeping parameter paths intact
        args.insert(args.begin(), "ekf_filter_node");

        patched_options.arguments(args);
        return patched_options;
    }
};
}  // namespace robot_localization

RCLCPP_COMPONENTS_REGISTER_NODE(robot_localization::EkfComponent)