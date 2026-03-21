#include "tauv_core/watchdog.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <sstream>
#include <utility>

// Compile-time expected ESC IDs for watchdog monitoring.
#define WATCHDOG_EXPECTED_ESC_IDS {0, 1, 2, 3, 4, 5, 6, 7}

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
    uint32_t mask;
    const char* name;
};

const std::array<EscFaultInfo, 13> kEscFaultInfo = {{
    {ESC_FAULT_OVER_TEMPERATURE, "Over Temperature"},
    {ESC_FAULT_BUS_OVER_CURRENT, "Bus Over Current"},
    {ESC_FAULT_PHASE_OVER_CURRENT, "Phase Over Current"},
    {ESC_FAULT_OVER_VOLTAGE, "Over Voltage"},
    {ESC_FAULT_UNDER_VOLTAGE, "Under Voltage"},
    {ESC_FAULT_EXCESS_VOLTAGE_RIPPLE, "Excess Voltage Ripple"},
    {ESC_FAULT_SIGNAL_LOSS, "Signal Loss"},
    {ESC_FAULT_MOTOR_SATURATED, "Motor Saturated"},
    {ESC_FAULT_MOTOR_OVER_TEMPERATURE, "Motor Over Temperature"},
    {ESC_FAULT_RPM_LIMIT_REACHED, "RPM Limit Reached"},
    {ESC_FAULT_ERROR_ACTIVE, "Error Active"},
    {ESC_FAULT_OUTPUT_SHORTED, "Output Shorted"},
    {ESC_FAULT_STARTUP_CHECK_FAILED, "Startup Check Failed"},
}};

bool faultMaskSet(const int32_t fault_code, const uint32_t fault_mask) {
    return (static_cast<uint32_t>(fault_code) & fault_mask) > 0U;
}

}  // namespace

Watchdog::Watchdog(std::string prefix)
    : Node("watchdog"),
      prefix_(std::move(prefix)),
      system_in_error_(false),
      startup_time_(this->now()) {
    // Configuration: topics, watchdog timing, and warning thresholds.
    esc_topic_ = this->declare_parameter<std::string>("esc_telemetry_topic", "esc_telemetry");
    system_state_topic_ = this->declare_parameter<std::string>("system_state_topic", "watchdog/system_state");
    heartbeat_check_hz_ = this->declare_parameter<double>("heartbeat_check_hz", 2.0);
    esc_timeout_s_ = this->declare_parameter<double>("esc_timeout_s", 1.0);
    stale_startup_grace_s_ = this->declare_parameter<double>("stale_startup_grace_s", 5.0);
    warning_temperature_c_ = this->declare_parameter<double>("warning_temperature_c", 70.0);

    // Watchdog identity set: use compile-time ESC ID list.
    expected_esc_ids_ = std::vector<uint8_t> WATCHDOG_EXPECTED_ESC_IDS;

    esc_sub_ = this->create_subscription<tauv_msgs::msg::EscTelemetry>(
        esc_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&Watchdog::escTelemetryCallback, this, std::placeholders::_1));

    system_state_pub_ = this->create_publisher<std_msgs::msg::String>(system_state_topic_, 10);

    const auto timer_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(heartbeat_check_hz_, 1e-3)));
    heartbeat_timer_ = this->create_wall_timer(
        timer_period,
        std::bind(&Watchdog::heartbeatCheckCallback, this));

    publishSystemState();

    RCLCPP_INFO(this->get_logger(),
                "Watchdog started. topic=%s state_topic=%s timeout=%.2fs check_hz=%.2f expected_esc_count=%zu",
                esc_topic_.c_str(),
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

    const uint32_t cared_fault_code = static_cast<uint32_t>(msg->fault_code) & WATCHDOG_INTEREST_FAULT_MASK;

    if (cared_fault_code > 0U) {
        // Fault decoding: only include faults selected by WATCHDOG_INTEREST_FAULT_MASK.
        std::ostringstream active_faults;
        bool first_fault = true;
        for (const EscFaultInfo& fault : kEscFaultInfo) {
            if ((fault.mask & WATCHDOG_INTEREST_FAULT_MASK) == 0U || !faultMaskSet(msg->fault_code, fault.mask)) {
                continue;
            }

            if (!first_fault) {
                active_faults << ", ";
            }
            active_faults << fault.name;
            first_fault = false;
        }

        escs_with_cared_faults_.insert(msg->id);

        RCLCPP_ERROR_THROTTLE(this->get_logger(),
                              *this->get_clock(),
                              2000,
                              "ESC %u cared_fault_code=%u active_faults=[%s]",
                              msg->id,
                              cared_fault_code,
                              active_faults.str().c_str());
    } else {
        escs_with_cared_faults_.erase(msg->id);
    }

    publishSystemState();

    if (msg->temperature >= warning_temperature_c_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(),
                             *this->get_clock(),
                             2000,
                             "ESC %u temperature high: %.2fC (threshold: %.2fC)",
                             msg->id,
                             msg->temperature,
                             warning_temperature_c_);
    }
}

void Watchdog::heartbeatCheckCallback() {
    // Freshness monitoring: each monitored ESC must publish at least once per timeout.
    const auto now = this->now();
    if ((now - startup_time_).seconds() < stale_startup_grace_s_) {
        return;
    }

    for (const uint8_t esc_id : monitoredEscIds()) {
        const auto it = last_esc_telemetry_time_.find(esc_id);
        const bool stale = it == last_esc_telemetry_time_.end() || (now - it->second).seconds() > esc_timeout_s_;

        if (stale && stale_esc_ids_.insert(esc_id).second) {
            RCLCPP_WARN(this->get_logger(),
                        "ESC %u telemetry stale or missing (timeout: %.2fs)",
                        esc_id,
                        esc_timeout_s_);
        }
    }
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
    const bool has_cared_fault = !escs_with_cared_faults_.empty();
    if (has_cared_fault == system_in_error_) {
        return;
    }

    system_in_error_ = has_cared_fault;

    std_msgs::msg::String msg;
    msg.data = system_in_error_ ? "ERROR" : "OK";
    system_state_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "Watchdog system state changed: %s",
                msg.data.c_str());
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Watchdog>("os");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
