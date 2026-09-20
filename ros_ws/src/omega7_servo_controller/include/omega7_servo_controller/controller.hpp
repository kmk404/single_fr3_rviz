#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include "omega7_servo_controller/fault_diagnostics.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ruckig/ruckig.hpp>

namespace omega7_servo_controller {

class Omega7ServoController : public controller_interface::ControllerInterface {
 public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

 private:
  static constexpr size_t kJoints = 7;
  struct Target {
    std::array<double, kJoints> position{};
    int64_t received_ns = 0;
    uint64_t sequence = 0;
  };
  static int64_t steady_now_ns();
  bool read_measured(std::array<double, kJoints> &measured) const;
  void latch_fault(const FaultDetail &detail);
  bool has_fault() const;

  std::array<std::string, kJoints> names_{};
  std::array<double, kJoints> lower_{{-2.9007, -1.8361, -2.9007, -3.0770,
                                       -2.8763, 0.4398, -3.0508}};
  std::array<double, kJoints> upper_{{2.9007, 1.8361, 2.9007, -0.1169,
                                       2.8763, 4.6216, 3.0508}};
  std::array<double, kJoints> command_position_{};
  std::array<double, kJoints> command_velocity_{};
  std::array<double, kJoints> command_acceleration_{};
  std::array<double, kJoints> measured_velocity_{};
  std::array<double, kJoints> filtered_velocity_{};
  std::array<double, kJoints> k_gains_{};
  std::array<double, kJoints> d_gains_{};
  ruckig::Ruckig<kJoints> trajectory_{0.001};
  ruckig::InputParameter<kJoints> trajectory_input_{};
  ruckig::OutputParameter<kJoints> trajectory_output_{};
  double max_velocity_ = 0.18;
  double max_acceleration_ = 0.60;
  double max_jerk_ = 3.0;
  double target_timeout_ = 0.25;
  double following_error_ = 0.08;
  double max_target_jump_ = 0.04;
  double position_margin_ = 0.02;
  bool effort_control_ = true;
  bool initialized_ = false;
  bool was_enabled_ = false;
  bool following_ = false;
  std::atomic<bool> fault_claimed_{false};
  std::atomic<FaultReason> fault_reason_{FaultReason::None};
  std::atomic<size_t> fault_joint_{kNoFaultJoint};
  std::atomic<double> fault_observed_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<double> fault_limit_{std::numeric_limits<double>::quiet_NaN()};
  std::atomic<bool> fault_reported_{false};
  uint64_t rearm_after_sequence_ = 0;
  uint64_t accepted_sequence_ = 0;
  bool rearm_pending_ = true;
  std::atomic<uint64_t> received_sequence_{0};
  std::array<double, kJoints> last_target_{};
  std::atomic<bool> enabled_{false};
  realtime_tools::RealtimeBuffer<Target> target_buffer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_pub_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
};

}  // namespace omega7_servo_controller
