#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tauv_msgs/msg/esc_telemetry.hpp>

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/**
 * @brief Monitors core actuator health indicators from ESC telemetry.
 *
 * This node subscribes to ESC telemetry and IMU attitude, tracks telemetry
 * freshness, and publishes watchdog system state based on configured fault
 * interests and IMU safety limits.
 */
class Watchdog : public rclcpp::Node {
   public:
    /**
     * @brief Construct a watchdog node.
     *
     * @param prefix Vehicle prefix retained for compatibility with other core nodes.
     */
    explicit Watchdog(std::string prefix);

   private:
    /**
     * @brief Handles incoming telemetry from a single ESC.
     *
     * @param msg ESC telemetry sample containing identity, electrical state, and fault code.
     */
    void escTelemetryCallback(const tauv_msgs::msg::EscTelemetry::SharedPtr msg);

    /**
     * @brief Tracks IMU attitude and angular velocity limits.
     *
     * @param msg IMU sample used to derive roll/pitch from quaternion and angular rates.
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /**
     * @brief Periodic watchdog sweep that checks ESC telemetry freshness.
     */
    void heartbeatCheckCallback();

    /**
     * @brief Resolves the list of ESC IDs that must be monitored.
     *
     * If expected_esc_ids is configured, that list is used. Otherwise, all
     * observed ESC IDs are monitored.
     */
    std::vector<uint8_t> monitoredEscIds() const;

    /**
     * @brief Publishes the current watchdog system state.
     *
     * State is ERROR when any monitored ESC has a fault that matches
     * WATCHDOG_INTEREST_FAULT_MASK, when IMU attitude exceeds configured
     * roll/pitch thresholds, or when IMU angular velocity exceeds configured
     * body-rate thresholds. Otherwise state is OK.
     */
    void publishSystemState();

    rclcpp::Subscription<tauv_msgs::msg::EscTelemetry>::SharedPtr esc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_state_pub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    std::unordered_map<uint8_t, rclcpp::Time> last_esc_telemetry_time_;
    std::unordered_set<uint8_t> stale_esc_ids_;
    std::unordered_set<uint8_t> escs_with_cared_faults_;

    std::vector<uint8_t> expected_esc_ids_;

    std::string prefix_;
    std::string esc_topic_;
    std::string imu_topic_;
    std::string system_state_topic_;

    double heartbeat_check_hz_;
    double esc_timeout_s_;
    double stale_startup_grace_s_;
    double warning_temperature_c_;
    double error_temperature_c_;
    double error_voltage_v_;
    double roll_threshold_deg_;
    double pitch_threshold_deg_;
    double angular_velocity_threshold_radps_;

    bool system_in_error_;
    bool imu_attitude_fault_;
    bool imu_angular_velocity_fault_;
    rclcpp::Time startup_time_;
};
