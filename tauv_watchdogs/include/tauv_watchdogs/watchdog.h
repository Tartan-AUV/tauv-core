#pragma once

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
    // Canonical CAN-driver protocol. kill during startup means
    // not ready; it becomes latched only on a fault or expiration of grace.
    return latched_ || !healthFault(now, true).empty() ? "kill" : "continue";
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


#ifndef WATCHDOG_SELF_TEST
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tauv_msgs/msg/esc_telemetry.hpp>

// Existing callers may continue constructing Watchdog(prefix). Topics are
// relative to the ROS namespace; prefix remains a compatibility argument.
class Watchdog : public rclcpp::Node {
 public:
  explicit Watchdog(std::string prefix = "");
 private:
  void publish(watchdog::Time now);
  std::unique_ptr<watchdog::Core> core_;
  bool was_latched_ = false;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::Subscription<tauv_msgs::msg::EscTelemetry>::SharedPtr esc_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif
