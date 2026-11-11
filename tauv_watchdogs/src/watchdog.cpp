// C++17. Normal build uses the project's existing main that constructs Watchdog.
// Define WATCHDOG_STANDALONE for an executable from this file alone.
// Define WATCHDOG_SELF_TEST for dependency-free core regression tests.
#include "tauv_watchdogs/watchdog.h"

#ifdef WATCHDOG_SELF_TEST
#include <cassert>
#include <iostream>
int main() {
  using namespace watchdog;
  const auto t = [](double s) {
    return Time{} + std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(s));
  };
  Config c;
  c.ids = {100, 101}; c.esc_timeout = 2; c.imu_timeout = 2;
  c.startup_grace = 1; c.mission_timeout = 10;
  auto feed = [&](Core& w, double s) {
    w.esc(100, 30, 16, t(s)); w.esc(101, 30, 16, t(s)); w.imu({}, t(s));
  };
  Core w(c, t(0));
  assert(w.state(t(0)) == "kill" && !w.latched());
  assert(!w.reset(t(0)).success); // Never-seen sensors cannot be reset away.
  feed(w, 0.1);
  assert(w.state(t(0.1)) == "continue");
  w.esc(100, 95, 16, t(0.2));
  assert(w.latched() && !w.reset(t(0.2)).success);
  feed(w, 0.3);
  assert(w.state(t(0.3)) == "kill"); // Recovery alone does not clear latch.
  assert(w.reset(t(0.3)).success && w.state(t(0.3)) == "continue");
  w.tick(t(2.4));
  assert(w.latched() && !w.reset(t(2.4)).success);
  feed(w, 2.5);
  assert(w.reset(t(2.5)).success);
  w.imu("bad quaternion", t(2.6));
  assert(!w.reset(t(2.6)).success);
  feed(w, 2.7); assert(w.reset(t(2.7)).success);
  w.esc(100, 30, 11, t(2.8)); assert(!w.reset(t(2.8)).success);
  feed(w, 2.9); assert(w.reset(t(2.9)).success);
  w.esc(100, std::numeric_limits<double>::quiet_NaN(), 16, t(3));
  assert(!w.reset(t(3)).success);
  feed(w, 3.1); assert(w.reset(t(3.1)).success);

  Core missing(c, t(0));
  missing.esc(100, 30, 16, t(0.1)); missing.imu({}, t(0.1));
  missing.esc(255, 30, 16, t(0.1)); // Unexpected ID cannot satisfy ESC 101.
  missing.tick(t(1)); assert(missing.latched());
  assert(!missing.reset(t(1)).success);

  Core dead_imu(c, t(0)); feed(dead_imu, 0.1);
  dead_imu.esc(100, 30, 16, t(2)); dead_imu.esc(101, 30, 16, t(2));
  dead_imu.tick(t(2.2));
  assert(dead_imu.latched() && dead_imu.reason() == "IMU missing/stale");

  Core late(c, t(0)); feed(late, 0.1); feed(late, 3);
  assert(late.latched()); // Arrival before next timer tick still exposes gap.

  Core mission(c, t(0)); feed(mission, 0.1);
  for (int i = 1; i <= 9; ++i) feed(mission, i);
  assert(mission.reset(t(9)).success); // Healthy reset must not extend deadline.
  feed(mission, 10);
  assert(mission.latched() && mission.reason() == "mission timeout");
  assert(mission.reset(t(10)).success && mission.state(t(10)) == "continue");
  for (int i = 11; i <= 19; ++i) feed(mission, i);
  assert(!mission.latched());
  feed(mission, 20); assert(mission.latched());
  assert(mission.reset(t(20)).success);
  mission.tick(t(22)); // Reset grants no new startup grace.
  assert(mission.latched());

  assert(imuFault(c, 0, 0, 0, 1, 0, 0, 0, true, true).empty());
  assert(!imuFault(c, 0, 0, 0, 0, 0, 0, 0, true, true).empty());
  assert(!imuFault(c, 0, 0, 0, 1, 6, 0, 0, true, true).empty());
  assert(!imuFault(c, .5, 0, 0, std::sqrt(.75), 0, 0, 0, true, true).empty());
  assert(!imuFault(c, 0, 0, 0, 1, 0, 0, 0, false, true).empty());
  assert(!imuFault(c, 0, 0, 0, 1, 0, 0, 0, true, false).empty());
  bool rejected = false;
  try { Config bad = c; bad.ids.clear(); Core invalid(bad, t(0)); }
  catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
  rejected = false;
  try { Config bad = c; bad.esc_timeout = std::numeric_limits<double>::infinity();
        Core invalid(bad, t(0)); }
  catch (const std::invalid_argument&) { rejected = true; }
  assert(rejected);
  std::cout << "All watchdog core assertions passed\n";
}
#else
Watchdog::Watchdog(std::string prefix) : Node("watchdog") {
    (void)prefix;  // ROS namespace/remapping controls topic resolution.
    watchdog::Config c;
    c.ids = declare_parameter<std::vector<int64_t>>("expected_esc_ids", c.ids);
    auto param = [this](const char* name, double& v) {
      v = declare_parameter<double>(name, v);
    };
    param("esc_timeout_s", c.esc_timeout);
    param("imu_timeout_s", c.imu_timeout);
    param("stale_startup_grace_s", c.startup_grace);
    param("warning_temperature_c", c.warning_temperature);
    param("error_temperature_c", c.error_temperature);
    param("error_voltage_v", c.min_voltage);
    param("roll_threshold_deg", c.max_roll);
    param("pitch_threshold_deg", c.max_pitch);
    param("angular_velocity_threshold_radps", c.max_rate);
    param("mission_timeout_s", c.mission_timeout);
    c.validate();
    const double hz = declare_parameter<double>("heartbeat_frequency_hz", 2.0);
    if (!std::isfinite(hz) || hz < 0.1 || hz > 1000.0)
      throw std::invalid_argument("heartbeat_frequency_hz must be in 0.1..1000");
    core_ = std::make_unique<watchdog::Core>(c, watchdog::Clock::now());
    const auto esc_topic = declare_parameter<std::string>("esc_topic", "esc_telemetry");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "os/sensors/imu_xsens");
    const auto state_topic = declare_parameter<std::string>("system_state_topic", "watchdog/system_state");
    const auto reset_service = declare_parameter<std::string>("reset_service", "watchdog/reset");
    pub_ = create_publisher<std_msgs::msg::String>(state_topic, rclcpp::QoS(1).reliable().durability_volatile());
    // All callbacks use the default mutually exclusive callback group.
    esc_sub_ = create_subscription<tauv_msgs::msg::EscTelemetry>(
        esc_topic, rclcpp::SensorDataQoS(),
        [this](tauv_msgs::msg::EscTelemetry::ConstSharedPtr m) {
          const auto now = watchdog::Clock::now();
          core_->esc(m->id, m->temperature, m->voltage, now);
          if (core_->expected(m->id) && std::isfinite(m->temperature) &&
              m->temperature > core_->config().warning_temperature) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "ESC %u temperature %.1f C", static_cast<unsigned>(m->id),
                static_cast<double>(m->temperature));
          }
          publish(now);
        });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
          const auto now = watchdog::Clock::now();
          core_->imu(watchdog::imuFault(core_->config(), m->orientation.x,
              m->orientation.y, m->orientation.z, m->orientation.w,
              m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z,
              m->orientation_covariance[0] != -1,
              m->angular_velocity_covariance[0] != -1), now);
          publish(now);
        });
    reset_ = create_service<std_srvs::srv::Trigger>(reset_service,
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          const auto now = watchdog::Clock::now();
          const auto result = core_->reset(now);
          response->success = result.success;
          response->message = result.message;
          RCLCPP_INFO(get_logger(), "%s", result.message.c_str());
          publish(now);
        });
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / hz));
    timer_ = create_wall_timer(period, [this] { publish(watchdog::Clock::now()); });
    publish(watchdog::Clock::now());
  }
void Watchdog::publish(watchdog::Time now) {
    std_msgs::msg::String message;
    message.data = core_->state(now);
    if (core_->latched() && !was_latched_)
      RCLCPP_ERROR(get_logger(), "Watchdog latched: %s", core_->reason().c_str());
    was_latched_ = core_->latched();
    pub_->publish(message);
  }

#ifdef WATCHDOG_STANDALONE
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<Watchdog>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("watchdog"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown();
  return result;
}
#endif
#endif
