// New ROS 2 watchdog implementation based only on the supplied snippet.
// Requires C++17, rclcpp, sensor_msgs, std_msgs, std_srvs, tauv_msgs.
// Assumes EscTelemetry contains numeric id, temperature (C), voltage (V).
// No ESC fault-bit field was provided, so no such field is invented here.
// Core-only tests: g++ -std=c++17 -Wall -Wextra -Werror
//   -DWATCHDOG_SELF_TEST watchdog_reset.cpp -o watchdog_test && ./watchdog_test
// ROS reset: ros2 service call /watchdog/reset std_srvs/srv/Trigger "{}"
// Reset acknowledges the fault and begins a new mission timer; it does not arm motors.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace watchdog {
using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
inline double age(Time now, Time then) {
  return std::chrono::duration<double>(now - then).count();
}

struct Config {
  std::vector<int64_t> ids{100, 101, 102, 103, 104, 105, 106, 107};
  double esc_timeout = 10.0;
  double imu_timeout = 1.0;  // New: do not silently accept a dead IMU.
  double startup_grace = 10.0;
  double warning_temperature = 70.0;
  double error_temperature = 90.0;
  double min_voltage = 12.0;
  double max_roll = 45.0;
  double max_pitch = 45.0;
  double max_rate = 5.0;
  double mission_timeout = 480.0;  // Zero disables this timer.

  void validate() const {
    for (double v : {esc_timeout, imu_timeout, startup_grace,
                     warning_temperature, error_temperature, min_voltage,
                     max_roll, max_pitch, max_rate, mission_timeout}) {
      if (!std::isfinite(v)) throw std::invalid_argument("Non-finite parameter");
    }
    if (esc_timeout <= 0 || imu_timeout <= 0 || startup_grace < 0 ||
        warning_temperature >= error_temperature || min_voltage <= 0 ||
        max_roll < 0 || max_roll > 180 || max_pitch < 0 || max_pitch > 90 ||
        max_rate < 0 || mission_timeout < 0 || ids.empty()) {
      throw std::invalid_argument("Invalid watchdog parameter range");
    }
    auto sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    if (sorted.front() < 0 || sorted.back() > 255 ||
        std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end()) {
      throw std::invalid_argument("ESC IDs must be unique and within 0..255");
    }
  }
};

// Validates and converts IMU inputs without depending on ROS or tf2.
// Assumes the orientation already describes the vehicle in the intended frame.
inline std::string imuFault(const Config& c, double x, double y, double z,
                            double w, double wx, double wy, double wz,
                            bool orientation_available, bool rate_available) {
  if (!orientation_available || !rate_available) return "IMU data unavailable";
  for (double v : {x, y, z, w, wx, wy, wz}) {
    if (!std::isfinite(v)) return "IMU contains NaN or infinity";
  }
  const double norm = std::hypot(std::hypot(x, y), std::hypot(z, w));
  // Reject malformed quaternions, normalize small floating-point drift.
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 0.01)
    return "IMU quaternion is not approximately unit length";
  x /= norm; y /= norm; z /= norm; w /= norm;
  constexpr double rad_to_deg = 57.2957795130823208768;
  const double roll = std::atan2(2 * (w*x + y*z), 1 - 2*(x*x + y*y));
  const double pitch = std::asin(std::clamp(2 * (w*y - z*x), -1.0, 1.0));
  if (std::abs(roll) * rad_to_deg > c.max_roll ||
      std::abs(pitch) * rad_to_deg > c.max_pitch) return "IMU attitude limit";
  if (std::max({std::abs(wx), std::abs(wy), std::abs(wz)}) > c.max_rate)
    return "IMU angular velocity limit";
  return {};
}

class Core {
 public:
  struct ResetResult { bool success; std::string message; };
  explicit Core(Config config, Time now)
      : c_(std::move(config)), startup_(now), mission_start_(now) {
    c_.validate();
    for (auto id : c_.ids) esc_.emplace(id, Sample{});
  }
  const Config& config() const { return c_; }
  bool expected(int64_t id) const { return esc_.count(id) != 0; }
  bool latched() const { return latched_; }
  const std::string& reason() const { return reason_; }

  void esc(int64_t id, double temperature, double voltage, Time now) {
    if (!expected(id)) return;
    // Check the OLD timestamps first so late recovery cannot hide a timeout.
    tick(now);
    std::string fault;
    if (!std::isfinite(temperature) || !std::isfinite(voltage))
      fault = "invalid telemetry";
    else {
      if (temperature > c_.error_temperature) fault += "over temperature; ";
      if (voltage < c_.min_voltage) fault += "under voltage; ";
    }
    esc_.at(id) = Sample{now, std::move(fault)};
    tick(now);
  }

  void imu(std::string fault, Time now) {
    tick(now);
    imu_ = Sample{now, std::move(fault)};
    tick(now);
  }

  void tick(Time now) {
    // Startup grace suppresses missing/stale trips only, never measured faults.
    const auto fault = healthFault(now, age(now, startup_) >= c_.startup_grace);
    if (!fault.empty()) trip(fault);
    if (c_.mission_timeout > 0 && age(now, mission_start_) >= c_.mission_timeout)
      trip("mission timeout");
  }

  std::string state(Time now) {
    tick(now);
    // Keep the original two-string wire protocol. ERROR during startup means
    // not ready; it becomes latched only on a fault or expiration of grace.
    return latched_ || !healthFault(now, true).empty() ? "ERROR" : "OK";
  }

  ResetResult reset(Time now) {
    tick(now);
    // Strict check: startup grace does NOT allow a reset with missing data.
    const auto fault = healthFault(now, true);
    if (!fault.empty()) return {false, "Reset refused: " + fault};
    if (!latched_) return {true, "Already healthy; mission timer unchanged"};
    latched_ = false;
    reason_.clear();
    mission_start_ = now;
    // Keep all readings/timestamps and the ORIGINAL startup deadline.
    return {true, "Watchdog reset; new mission timer started; motors not armed"};
  }

 private:
  struct Sample {
    std::optional<Time> received;
    std::string fault;
  };
  std::string healthFault(Time now, bool require_fresh) const {
    for (const auto& item : esc_) {
      const auto& sample = item.second;
      const std::string name = "ESC " + std::to_string(item.first) + ": ";
      if (!sample.fault.empty()) return name + sample.fault;
      if (require_fresh && (!sample.received ||
          age(now, *sample.received) >= c_.esc_timeout)) return name + "missing/stale";
    }
    if (!imu_.fault.empty()) return imu_.fault;
    if (require_fresh && (!imu_.received ||
        age(now, *imu_.received) >= c_.imu_timeout)) return "IMU missing/stale";
    return {};
  }
  void trip(const std::string& reason) {
    if (!latched_) reason_ = reason; // Preserve the first trip for diagnosis.
    latched_ = true;
  }
  Config c_;
  Time startup_, mission_start_;
  std::map<int64_t, Sample> esc_;
  Sample imu_;
  bool latched_ = false;
  std::string reason_;
};
}  // namespace watchdog

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
  assert(w.state(t(0)) == "ERROR" && !w.latched());
  assert(!w.reset(t(0)).success); // Never-seen sensors cannot be reset away.
  feed(w, 0.1);
  assert(w.state(t(0.1)) == "OK");
  w.esc(100, 95, 16, t(0.2));
  assert(w.latched() && !w.reset(t(0.2)).success);
  feed(w, 0.3);
  assert(w.state(t(0.3)) == "ERROR"); // Recovery alone does not clear latch.
  assert(w.reset(t(0.3)).success && w.state(t(0.3)) == "OK");
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
  assert(mission.reset(t(10)).success && mission.state(t(10)) == "OK");
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
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tauv_msgs/msg/esc_telemetry.hpp>

class ResettableWatchdog final : public rclcpp::Node {
 public:
  ResettableWatchdog() : Node("watchdog") {
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
    const auto esc_topic = declare_parameter<std::string>("esc_topic", "/esc_telemetry");
    const auto imu_topic = declare_parameter<std::string>("imu_topic", "os/sensors/imu_xsens");
    const auto state_topic = declare_parameter<std::string>("system_state_topic", "watchdog/system_state");
    const auto reset_service = declare_parameter<std::string>("reset_service", "watchdog/reset");
    pub_ = create_publisher<std_msgs::msg::String>(state_topic, 10);
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
 private:
  void publish(watchdog::Time now) {
    std_msgs::msg::String message;
    message.data = core_->state(now);
    if (core_->latched() && !was_latched_)
      RCLCPP_ERROR(get_logger(), "Watchdog latched: %s", core_->reason().c_str());
    was_latched_ = core_->latched();
    pub_->publish(message);
  }
  std::unique_ptr<watchdog::Core> core_;
  bool was_latched_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<tauv_msgs::msg::EscTelemetry>::SharedPtr esc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try { rclcpp::spin(std::make_shared<ResettableWatchdog>()); }
  catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("watchdog"), "%s", e.what()); result = 1;
  }
  rclcpp::shutdown();
  return result;
}
#endif
