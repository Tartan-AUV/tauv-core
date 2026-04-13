#include "tauv_watchdogs/watchdog.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

// Compile-time expected ESC IDs for watchdog monitoring.
#define WATCHDOG_EXPECTED_ESC_IDS {100, 101, 102, 103, 104, 105, 106, 107}

// Watchdog configuration constants.
#define WATCHDOG_ESC_TELEMETRY_TOPIC "/esc_telemetry"
#define WATCHDOG_IMU_TOPIC_SUFFIX "/sensors/imu_xsens"
#define WATCHDOG_SYSTEM_STATE_TOPIC "watchdog/system_state"
#define WATCHDOG_HEARTBEAT_CHECK_HZ 2.0
#define WATCHDOG_HEARTBEAT_FREQUENCY_PARAM "heartbeat_frequency_hz"
#define WATCHDOG_ESC_TIMEOUT_S 1.0
#define WATCHDOG_STALE_STARTUP_GRACE_S 5.0
#define WATCHDOG_WARNING_TEMPERATURE_C 70.0
#define WATCHDOG_ERROR_TEMPERATURE_C 90.0
#define WATCHDOG_ERROR_VOLTAGE_V 12.0
#define WATCHDOG_ERROR_ROLL_DEG 45.0
#define WATCHDOG_ERROR_PITCH_DEG 45.0
#define WATCHDOG_ERROR_ANGULAR_VELOCITY_RADPS 5.0
#define WATCHDOG_ESC_TOPIC_PARAM "esc_topic"
#define WATCHDOG_IMU_TOPIC_PARAM "imu_topic"
#define WATCHDOG_SYSTEM_STATE_TOPIC_PARAM "system_state_topic"
#define WATCHDOG_ESC_TIMEOUT_PARAM "esc_timeout_s"
#define WATCHDOG_STALE_STARTUP_GRACE_PARAM "stale_startup_grace_s"
#define WATCHDOG_WARNING_TEMPERATURE_PARAM "warning_temperature_c"
#define WATCHDOG_ERROR_TEMPERATURE_PARAM "error_temperature_c"
#define WATCHDOG_ERROR_VOLTAGE_PARAM "error_voltage_v"
#define WATCHDOG_ROLL_THRESHOLD_PARAM "roll_threshold_deg"
#define WATCHDOG_PITCH_THRESHOLD_PARAM "pitch_threshold_deg"
#define WATCHDOG_ANGULAR_VELOCITY_THRESHOLD_PARAM "angular_velocity_threshold_radps"
#define WATCHDOG_EXPECTED_ESC_IDS_PARAM "expected_esc_ids"

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

Watchdog::Watchdog(std::string prefix)
    : Node("watchdog"),
      prefix_(std::move(prefix)),
      system_in_error_(false),
      imu_attitude_fault_(false),
      imu_angular_velocity_fault_(false),
      startup_time_(this->now()) {
    const auto ensure_positive_param = [this](const char* name, const double value, const double fallback) {
        if (value > 0.0) {
            return value;
        }
        RCLCPP_WARN(this->get_logger(),
                    "Invalid %s=%.3f. Falling back to default %.3f.",
                    name,
                    value,
                    fallback);
        return fallback;
    };

    const auto ensure_non_negative_param = [this](const char* name, const double value, const double fallback) {
        if (value >= 0.0) {
            return value;
        }
        RCLCPP_WARN(this->get_logger(),
                    "Invalid %s=%.3f. Falling back to default %.3f.",
                    name,
                    value,
                    fallback);
        return fallback;
    };

    // Configuration: topics, watchdog timing, and warning thresholds.
    esc_topic_ = this->declare_parameter<std::string>(
        WATCHDOG_ESC_TOPIC_PARAM,
        WATCHDOG_ESC_TELEMETRY_TOPIC);
    imu_topic_ = this->declare_parameter<std::string>(
        WATCHDOG_IMU_TOPIC_PARAM,
        prefix_ + WATCHDOG_IMU_TOPIC_SUFFIX);
    system_state_topic_ = this->declare_parameter<std::string>(
        WATCHDOG_SYSTEM_STATE_TOPIC_PARAM,
        WATCHDOG_SYSTEM_STATE_TOPIC);
    heartbeat_check_hz_ = ensure_positive_param(
        WATCHDOG_HEARTBEAT_FREQUENCY_PARAM,
        this->declare_parameter<double>(WATCHDOG_HEARTBEAT_FREQUENCY_PARAM, WATCHDOG_HEARTBEAT_CHECK_HZ),
        WATCHDOG_HEARTBEAT_CHECK_HZ);
    esc_timeout_s_ = ensure_positive_param(
        WATCHDOG_ESC_TIMEOUT_PARAM,
        this->declare_parameter<double>(WATCHDOG_ESC_TIMEOUT_PARAM, WATCHDOG_ESC_TIMEOUT_S),
        WATCHDOG_ESC_TIMEOUT_S);
    stale_startup_grace_s_ = ensure_non_negative_param(
        WATCHDOG_STALE_STARTUP_GRACE_PARAM,
        this->declare_parameter<double>(WATCHDOG_STALE_STARTUP_GRACE_PARAM, WATCHDOG_STALE_STARTUP_GRACE_S),
        WATCHDOG_STALE_STARTUP_GRACE_S);
    warning_temperature_c_ = this->declare_parameter<double>(
        WATCHDOG_WARNING_TEMPERATURE_PARAM,
        WATCHDOG_WARNING_TEMPERATURE_C);
    error_temperature_c_ = this->declare_parameter<double>(
        WATCHDOG_ERROR_TEMPERATURE_PARAM,
        WATCHDOG_ERROR_TEMPERATURE_C);
    error_voltage_v_ = this->declare_parameter<double>(
        WATCHDOG_ERROR_VOLTAGE_PARAM,
        WATCHDOG_ERROR_VOLTAGE_V);
    roll_threshold_deg_ = ensure_non_negative_param(
        WATCHDOG_ROLL_THRESHOLD_PARAM,
        this->declare_parameter<double>(WATCHDOG_ROLL_THRESHOLD_PARAM, WATCHDOG_ERROR_ROLL_DEG),
        WATCHDOG_ERROR_ROLL_DEG);
    pitch_threshold_deg_ = ensure_non_negative_param(
        WATCHDOG_PITCH_THRESHOLD_PARAM,
        this->declare_parameter<double>(WATCHDOG_PITCH_THRESHOLD_PARAM, WATCHDOG_ERROR_PITCH_DEG),
        WATCHDOG_ERROR_PITCH_DEG);
    angular_velocity_threshold_radps_ = ensure_non_negative_param(
        WATCHDOG_ANGULAR_VELOCITY_THRESHOLD_PARAM,
        this->declare_parameter<double>(
            WATCHDOG_ANGULAR_VELOCITY_THRESHOLD_PARAM,
            WATCHDOG_ERROR_ANGULAR_VELOCITY_RADPS),
        WATCHDOG_ERROR_ANGULAR_VELOCITY_RADPS);

    const std::vector<int64_t> default_expected_esc_ids = WATCHDOG_EXPECTED_ESC_IDS;
    const auto expected_esc_ids_param = this->declare_parameter<std::vector<int64_t>>(
        WATCHDOG_EXPECTED_ESC_IDS_PARAM,
        default_expected_esc_ids);
    expected_esc_ids_.clear();
    expected_esc_ids_.reserve(expected_esc_ids_param.size());
    for (const auto esc_id : expected_esc_ids_param) {
        if (esc_id < 0 || esc_id > std::numeric_limits<uint8_t>::max()) {
            RCLCPP_WARN(this->get_logger(),
                        "Ignoring out-of-range ESC id in %s: %ld",
                        WATCHDOG_EXPECTED_ESC_IDS_PARAM,
                        esc_id);
            continue;
        }
        expected_esc_ids_.push_back(static_cast<uint8_t>(esc_id));
    }

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

    const bool temperature_fault = msg->temperature > error_temperature_c_;
    const bool voltage_fault = msg->voltage < error_voltage_v_;
    const bool has_fault = temperature_fault || voltage_fault;

    if (has_fault) {
        escs_with_cared_faults_.insert(msg->id);
        const char* fault_description = "OK";
        if (temperature_fault && voltage_fault) {
            fault_description = "Over Temperature + Under Voltage";
        } else if (temperature_fault) {
            fault_description = "Over Temperature";
        } else if (voltage_fault) {
            fault_description = "Under Voltage";
        }

        RCLCPP_ERROR_THROTTLE(this->get_logger(),
                              *this->get_clock(),
                              2000,
                              "ESC %u faultDetect error: %s",
                              msg->id,
                              fault_description);

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
    const bool has_error_now =
        !escs_with_cared_faults_.empty() || !stale_esc_ids_.empty() || imu_attitude_fault_ || imu_angular_velocity_fault_;

    // Once an error is observed, latch watchdog output in ERROR until restart.
    const bool previous_state = system_in_error_;
    system_in_error_ = system_in_error_ || has_error_now;
    const bool state_changed = system_in_error_ != previous_state;

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
