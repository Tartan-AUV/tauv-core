#include "tauv_core/watchdog.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

// Compile-time expected ESC IDs for watchdog monitoring.
#define WATCHDOG_EXPECTED_ESC_IDS {0, 1, 2, 3, 4, 5, 6, 7}

// Watchdog configuration constants.
#define WATCHDOG_ESC_TELEMETRY_TOPIC "esc_telemetry"
#define WATCHDOG_IMU_TOPIC_SUFFIX "/sensors/imu_xsens"
#define WATCHDOG_SYSTEM_STATE_TOPIC "watchdog/system_state"
#define WATCHDOG_HEARTBEAT_CHECK_HZ 2.0
#define WATCHDOG_ESC_TIMEOUT_S 1.0
#define WATCHDOG_STALE_STARTUP_GRACE_S 5.0
#define WATCHDOG_WARNING_TEMPERATURE_C 70.0
#define WATCHDOG_ERROR_TEMPERATURE_C 90.0
#define WATCHDOG_ERROR_VOLTAGE_V 12.0
#define WATCHDOG_ERROR_ROLL_DEG 45.0
#define WATCHDOG_ERROR_PITCH_DEG 45.0
#define WATCHDOG_ERROR_ANGULAR_VELOCITY_RADPS 5.0

// ESC fault bit masks.
#define ESC_FAULT_OVER_TEMPERATURE (1U << 0)
#define ESC_FAULT_BUS_OVER_CURRENT (1U << 1)
#define ESC_FAULT_PHASE_OVER_CURRENT (1U << 2)
#define ESC_FAULT_OVER_VOLTAGE (1U << 3)
#define ESC_FAULT_UNDER_VOLTAGE (1U << 4)
#define ESC_FAULT_EXCESS_VOLTAGE_RIPPLE (1U << 5)
#define ESC_FAULT_SIGNAL_LOSS (1U << 6)
#define ESC_FAULT_MOTOR_SATURATED (1U << 7)
#define ESC_FAULT_MOTOR_OVER_TEMPERATURE (1U << 8)
#define ESC_FAULT_RPM_LIMIT_REACHED (1U << 9)
#define ESC_FAULT_ERROR_ACTIVE (1U << 10)
#define ESC_FAULT_OUTPUT_SHORTED (1U << 11)
#define ESC_FAULT_STARTUP_CHECK_FAILED (1U << 12)

// Choose which faults should affect watchdog system state.
#define WATCHDOG_INTEREST_FAULT_MASK \
    (ESC_FAULT_SIGNAL_LOSS | ESC_FAULT_ERROR_ACTIVE | ESC_FAULT_OUTPUT_SHORTED | ESC_FAULT_STARTUP_CHECK_FAILED)

using namespace std::chrono_literals;

namespace {

struct EscFaultInfo {
    bool voltage_fault;
    bool temperature_fault;
    const char* description;
};

/**
 * @brief Classifies ESC telemetry against simple electrical safety thresholds.
 *
 * @param temperature_c ESC temperature in Celsius.
 * @param voltage_v ESC bus voltage in Volts.
 * @return EscFaultInfo Struct containing fault flags and an operator-facing summary.
 */
EscFaultInfo faultDetect(const double temperature_c, const double voltage_v) {
    const bool temperature_fault = temperature_c > WATCHDOG_ERROR_TEMPERATURE_C;
    const bool voltage_fault = voltage_v < WATCHDOG_ERROR_VOLTAGE_V;

    if (temperature_fault && voltage_fault) {
        return {voltage_fault, temperature_fault, "Over Temperature + Under Voltage"};
    }
    if (temperature_fault) {
        return {voltage_fault, temperature_fault, "Over Temperature"};
    }
    if (voltage_fault) {
        return {voltage_fault, temperature_fault, "Under Voltage"};
    }
    return {false, false, "OK"};
}

}  // namespace

Watchdog::Watchdog(std::string prefix)
    : Node("watchdog"),
      prefix_(std::move(prefix)),
      system_in_error_(false),
      imu_attitude_fault_(false),
      imu_angular_velocity_fault_(false),
      startup_time_(this->now()) {
    // Configuration: topics, watchdog timing, and warning thresholds.
    esc_topic_ = WATCHDOG_ESC_TELEMETRY_TOPIC;
    imu_topic_ = prefix_ + WATCHDOG_IMU_TOPIC_SUFFIX;
    system_state_topic_ = WATCHDOG_SYSTEM_STATE_TOPIC;
    heartbeat_check_hz_ = WATCHDOG_HEARTBEAT_CHECK_HZ;
    esc_timeout_s_ = WATCHDOG_ESC_TIMEOUT_S;
    stale_startup_grace_s_ = WATCHDOG_STALE_STARTUP_GRACE_S;
    warning_temperature_c_ = WATCHDOG_WARNING_TEMPERATURE_C;
    roll_threshold_deg_ = WATCHDOG_ERROR_ROLL_DEG;
    pitch_threshold_deg_ = WATCHDOG_ERROR_PITCH_DEG;
    angular_velocity_threshold_radps_ = WATCHDOG_ERROR_ANGULAR_VELOCITY_RADPS;

    // Watchdog identity set: use compile-time ESC ID list.
    expected_esc_ids_ = std::vector<uint8_t> WATCHDOG_EXPECTED_ESC_IDS;

    esc_sub_ = this->create_subscription<tauv_msgs::msg::EscTelemetry>(
        esc_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&Watchdog::escTelemetryCallback, this, std::placeholders::_1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&Watchdog::imuCallback, this, std::placeholders::_1));

    system_state_pub_ = this->create_publisher<std_msgs::msg::String>(system_state_topic_, 10);

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(heartbeat_check_hz_, 1e-3)));
    heartbeat_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&Watchdog::heartbeatCheckCallback, this));

    publishSystemState();

    RCLCPP_INFO(this->get_logger(),
                "Watchdog started. esc_topic=%s imu_topic=%s state_topic=%s timeout=%.2fs check_hz=%.2f expected_esc_count=%zu",
                esc_topic_.c_str(),
                imu_topic_.c_str(),
                system_state_topic_.c_str(),
                esc_timeout_s_,
                heartbeat_check_hz_,
                expected_esc_ids_.size());
}

void Watchdog::escTelemetryCallback(const tauv_msgs::msg::EscTelemetry::SharedPtr msg) {
    // Telemetry ingestion: update freshness state and emit actionable alerts.
    last_esc_telemetry_time_[msg->id] = this->now();

    if (stale_esc_ids_.erase(msg->id) > 0) {
        RCLCPP_INFO(this->get_logger(), "ESC %u telemetry stream recovered", msg->id);
    }

    const EscFaultInfo fault_info = faultDetect(msg->temperature, msg->voltage);
    const bool has_fault = fault_info.temperature_fault || fault_info.voltage_fault;

    if (has_fault) {
        escs_with_cared_faults_.insert(msg->id);
        RCLCPP_ERROR_THROTTLE(this->get_logger(),
                              *this->get_clock(),
                              2000,
                              "ESC %u faultDetect error: %s",
                              msg->id,
                              fault_info.description);

        // Publish ERROR immediately on detection instead of waiting for the next heartbeat tick.
        system_in_error_ = true;
        std_msgs::msg::String msg_state;
        msg_state.data = "ERROR";
        system_state_pub_->publish(msg_state);
    } else {
        escs_with_cared_faults_.erase(msg->id);
    }
}

void Watchdog::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // Attitude safety gate: derive roll/pitch from quaternion and enforce limits.
    const tf2::Quaternion q(msg->orientation.x,
                            msg->orientation.y,
                            msg->orientation.z,
                            msg->orientation.w);

    double roll_rad = 0.0;
    double pitch_rad = 0.0;
    double yaw_rad = 0.0;
    tf2::Matrix3x3(q).getRPY(roll_rad, pitch_rad, yaw_rad);

    const double roll_deg = std::abs(roll_rad) * 180.0 / M_PI;
    const double pitch_deg = std::abs(pitch_rad) * 180.0 / M_PI;

    const bool attitude_exceeds_threshold = roll_deg > roll_threshold_deg_ || pitch_deg > pitch_threshold_deg_;
    const bool attitude_state_changed = attitude_exceeds_threshold != imu_attitude_fault_;
    imu_attitude_fault_ = attitude_exceeds_threshold;

    if (attitude_state_changed && imu_attitude_fault_) {
        RCLCPP_ERROR(this->get_logger(),
                     "IMU attitude limit exceeded: |roll|=%.2f deg (limit=%.2f), |pitch|=%.2f deg (limit=%.2f)",
                     roll_deg,
                     roll_threshold_deg_,
                     pitch_deg,
                     pitch_threshold_deg_);

        // Publish ERROR immediately when attitude exceeds configured limits.
        system_in_error_ = true;
        std_msgs::msg::String msg_state;
        msg_state.data = "ERROR";
        system_state_pub_->publish(msg_state);
    }

    if (attitude_state_changed && !imu_attitude_fault_) {
        RCLCPP_INFO(this->get_logger(),
                    "IMU attitude returned within limits: |roll|=%.2f deg, |pitch|=%.2f deg",
                    roll_deg,
                    pitch_deg);
    }

    // Body-rate safety gate: monitor absolute angular velocity on all IMU axes.
    const double omega_x_radps = std::abs(msg->angular_velocity.x);
    const double omega_y_radps = std::abs(msg->angular_velocity.y);
    const double omega_z_radps = std::abs(msg->angular_velocity.z);

    const bool angular_velocity_exceeds_threshold =
        omega_x_radps > angular_velocity_threshold_radps_ ||
        omega_y_radps > angular_velocity_threshold_radps_ ||
        omega_z_radps > angular_velocity_threshold_radps_;
    const bool angular_velocity_state_changed = angular_velocity_exceeds_threshold != imu_angular_velocity_fault_;
    imu_angular_velocity_fault_ = angular_velocity_exceeds_threshold;

    if (angular_velocity_state_changed && imu_angular_velocity_fault_) {
        RCLCPP_ERROR(this->get_logger(),
                     "IMU angular velocity limit exceeded: |wx|=%.2f rad/s, |wy|=%.2f rad/s, |wz|=%.2f rad/s (limit=%.2f)",
                     omega_x_radps,
                     omega_y_radps,
                     omega_z_radps,
                     angular_velocity_threshold_radps_);

        // Publish ERROR immediately when any body-rate exceeds configured limits.
        system_in_error_ = true;
        std_msgs::msg::String msg_state;
        msg_state.data = "ERROR";
        system_state_pub_->publish(msg_state);
    }

    if (angular_velocity_state_changed && !imu_angular_velocity_fault_) {
        RCLCPP_INFO(this->get_logger(),
                    "IMU angular velocity returned within limits: |wx|=%.2f rad/s, |wy|=%.2f rad/s, |wz|=%.2f rad/s",
                    omega_x_radps,
                    omega_y_radps,
                    omega_z_radps);
    }
}

void Watchdog::heartbeatCheckCallback() {
    // Freshness monitoring: each monitored ESC must publish at least once per timeout.
    const auto now = this->now();

    if ((now - startup_time_).seconds() >= stale_startup_grace_s_) {
        for (const uint8_t esc_id : monitoredEscIds()) {
            const auto it = last_esc_telemetry_time_.find(esc_id);
            // stale if never seen, or if last seen is too old.
            const bool stale = it == last_esc_telemetry_time_.end() || (now - it->second).seconds() > esc_timeout_s_;

            // log error if this is first time we have seen it stale, but do not log again until it recovers and goes stale again.
            if (stale && stale_esc_ids_.insert(esc_id).second) {
                RCLCPP_WARN(this->get_logger(),
                            "ESC %u telemetry stale or missing (timeout: %.2fs)",
                            esc_id,
                            esc_timeout_s_);
            }
        }
    }

    // Heartbeat output: publish watchdog state every tick.
    publishSystemState();
}

std::vector<uint8_t> Watchdog::monitoredEscIds() const {
    if (!expected_esc_ids_.empty()) {
        return expected_esc_ids_;
    }

    std::vector<uint8_t> observed_ids;
    observed_ids.reserve(last_esc_telemetry_time_.size());
    for (const auto& [esc_id, _] : last_esc_telemetry_time_) {
        (void)_;
        observed_ids.push_back(esc_id);
    }
    return observed_ids;
}

void Watchdog::publishSystemState() {
    const bool has_error =
        !escs_with_cared_faults_.empty() || !stale_esc_ids_.empty() || imu_attitude_fault_ || imu_angular_velocity_fault_;
    const bool state_changed = has_error != system_in_error_;
    system_in_error_ = has_error;

    std_msgs::msg::String msg;
    msg.data = system_in_error_ ? "ERROR" : "OK";
    system_state_pub_->publish(msg);

    if (state_changed) {
        RCLCPP_INFO(this->get_logger(),
                    "Watchdog system state changed: %s",
                    msg.data.c_str());
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Watchdog>("os");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
