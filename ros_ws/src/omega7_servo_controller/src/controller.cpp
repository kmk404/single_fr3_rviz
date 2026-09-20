#include "omega7_servo_controller/controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

namespace omega7_servo_controller {

int64_t Omega7ServoController::steady_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

void Omega7ServoController::latch_fault(const FaultDetail &detail) {
  bool expected = false;
  if (!fault_claimed_.compare_exchange_strong(expected, true,
                                              std::memory_order_acq_rel)) return;
  fault_joint_.store(detail.joint_index, std::memory_order_relaxed);
  fault_observed_.store(detail.observed, std::memory_order_relaxed);
  fault_limit_.store(detail.limit, std::memory_order_relaxed);
  fault_reason_.store(detail.reason, std::memory_order_release);
}

bool Omega7ServoController::has_fault() const {
  return fault_reason_.load(std::memory_order_acquire) != FaultReason::None;
}

controller_interface::InterfaceConfiguration
Omega7ServoController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const std::string interface = effort_control_ ? "/effort" : "/position";
  for (const auto &name : names_) config.names.push_back(name + interface);
  return config;
}

controller_interface::InterfaceConfiguration
Omega7ServoController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto &name : names_) {
    config.names.push_back(name + "/position");
    if (effort_control_) config.names.push_back(name + "/velocity");
  }
  return config;
}

controller_interface::CallbackReturn Omega7ServoController::on_init() {
  for (size_t i = 0; i < kJoints; ++i) names_[i] = "fr3_joint" + std::to_string(i + 1);
  try {
    auto_declare<double>("max_velocity", 0.18);
    auto_declare<double>("max_acceleration", 0.60);
    auto_declare<double>("max_jerk", 3.0);
    auto_declare<double>("target_timeout", 0.25);
    auto_declare<double>("max_following_error", 0.08);
    auto_declare<double>("max_target_jump", 0.04);
    auto_declare<double>("position_margin", 0.02);
    auto_declare<bool>("effort_control", true);
    auto_declare<std::vector<double>>(
      "k_gains", {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
    auto_declare<std::vector<double>>(
      "d_gains", {30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0});
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Omega7ServoController::on_configure(
    const rclcpp_lifecycle::State &) {
  const auto node = get_node();
  max_velocity_ = node->get_parameter("max_velocity").as_double();
  max_acceleration_ = node->get_parameter("max_acceleration").as_double();
  max_jerk_ = node->get_parameter("max_jerk").as_double();
  target_timeout_ = node->get_parameter("target_timeout").as_double();
  following_error_ = node->get_parameter("max_following_error").as_double();
  max_target_jump_ = node->get_parameter("max_target_jump").as_double();
  position_margin_ = node->get_parameter("position_margin").as_double();
  effort_control_ = node->get_parameter("effort_control").as_bool();
  const auto k_gains = node->get_parameter("k_gains").as_double_array();
  const auto d_gains = node->get_parameter("d_gains").as_double_array();
  if (max_velocity_ <= 0 || max_acceleration_ <= 0 || max_jerk_ <= 0 ||
      target_timeout_ <= 0 || following_error_ <= 0 || max_target_jump_ <= 0 ||
      position_margin_ < 0) {
    RCLCPP_ERROR(node->get_logger(), "Servo limits must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (k_gains.size() != kJoints || d_gains.size() != kJoints) {
    RCLCPP_ERROR(node->get_logger(), "k_gains and d_gains must each contain 7 values");
    return controller_interface::CallbackReturn::ERROR;
  }
  std::copy(k_gains.begin(), k_gains.end(), k_gains_.begin());
  std::copy(d_gains.begin(), d_gains.end(), d_gains_.begin());
  RCLCPP_INFO(node->get_logger(), "Configured %s control mode",
              effort_control_ ? "effort joint impedance" : "fake position");
  target_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "~/target_joint_states", rclcpp::QoS(10),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      if (msg->name.size() != kJoints || msg->position.size() != kJoints) {
        latch_fault({FaultReason::MessageStructure});
        return;
      }
      Target target;
      for (size_t i = 0; i < kJoints; ++i) {
        if (msg->name[i] != names_[i]) {
          latch_fault({FaultReason::JointNameMismatch, i});
          return;
        }
        if (!std::isfinite(msg->position[i])) {
          latch_fault({FaultReason::NonFiniteTarget, i, msg->position[i], 0.0});
          return;
        }
        target.position[i] = msg->position[i];
      }
      const auto limit_fault = position_margin_fault(target.position, lower_, upper_,
                                                      position_margin_);
      if (limit_fault.reason != FaultReason::None) {
        latch_fault(limit_fault);
        return;
      }
      target.received_ns = steady_now_ns();
      target.sequence = received_sequence_.fetch_add(1, std::memory_order_relaxed) + 1;
      target_buffer_.writeFromNonRT(target);
    });
  enable_sub_ = node->create_subscription<std_msgs::msg::Bool>(
    "~/enable", rclcpp::QoS(10),
    [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
      enabled_.store(msg->data, std::memory_order_release);
    });
  fault_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "~/fault_latched", rclcpp::QoS(1).transient_local());
  fault_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(200), [this]() {
      std_msgs::msg::Bool status;
      const auto reason = fault_reason_.load(std::memory_order_acquire);
      status.data = reason != FaultReason::None;
      fault_pub_->publish(status);
      if (status.data && !fault_reported_.exchange(true, std::memory_order_acq_rel)) {
        const auto joint = fault_joint_.load(std::memory_order_relaxed);
        const auto observed = fault_observed_.load(std::memory_order_relaxed);
        const auto limit = fault_limit_.load(std::memory_order_relaxed);
        if (joint != kNoFaultJoint &&
            (std::isfinite(observed) || reason == FaultReason::NonFiniteTarget) &&
            std::isfinite(limit)) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Servo fault latched: %s; joint %zu observed=%.6g limit=%.6g",
                       fault_reason_name(reason), joint + 1, observed, limit);
        } else if (joint != kNoFaultJoint && std::isfinite(observed)) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Servo fault latched: %s; joint %zu observed=%.6g",
                       fault_reason_name(reason), joint + 1, observed);
        } else if (joint == kNoFaultJoint && std::isfinite(observed) &&
                   std::isfinite(limit)) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Servo fault latched: %s; observed=%.6g limit=%.6g",
                       fault_reason_name(reason), observed, limit);
        } else if (joint == kNoFaultJoint && std::isfinite(observed)) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       "Servo fault latched: %s; observed=%.6g",
                       fault_reason_name(reason), observed);
        } else {
          RCLCPP_ERROR(get_node()->get_logger(), "Servo fault latched: %s",
                       fault_reason_name(reason));
        }
      }
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

bool Omega7ServoController::read_measured(std::array<double, kJoints> &measured) const {
  const size_t stride = effort_control_ ? 2 : 1;
  if (state_interfaces_.size() < kJoints * stride) return false;
  for (size_t i = 0; i < kJoints; ++i) {
    const auto value = state_interfaces_[i * stride].get_optional<double>();
    if (!value.has_value() || !std::isfinite(value.value())) return false;
    measured[i] = value.value();
  }
  return true;
}

controller_interface::CallbackReturn Omega7ServoController::on_activate(
    const rclcpp_lifecycle::State &) {
  fault_reason_.store(FaultReason::None, std::memory_order_release);
  fault_joint_.store(kNoFaultJoint, std::memory_order_relaxed);
  fault_observed_.store(std::numeric_limits<double>::quiet_NaN(), std::memory_order_relaxed);
  fault_limit_.store(std::numeric_limits<double>::quiet_NaN(), std::memory_order_relaxed);
  fault_reported_.store(false, std::memory_order_release);
  fault_claimed_.store(false, std::memory_order_release);
  std::array<double, kJoints> measured;
  if (command_interfaces_.size() != kJoints || !read_measured(measured)) {
    latch_fault({FaultReason::InvalidMeasuredState});
    return controller_interface::CallbackReturn::ERROR;
  }
  for (size_t i = 0; i < kJoints; ++i) {
    command_position_[i] = measured[i];
    command_velocity_[i] = 0.0;
    command_acceleration_[i] = 0.0;
    trajectory_input_.current_position[i] = measured[i];
    trajectory_input_.current_velocity[i] = 0.0;
    trajectory_input_.current_acceleration[i] = 0.0;
    trajectory_input_.target_position[i] = measured[i];
    trajectory_input_.target_velocity[i] = 0.0;
    trajectory_input_.target_acceleration[i] = 0.0;
    trajectory_input_.max_velocity[i] = max_velocity_;
    trajectory_input_.max_acceleration[i] = max_acceleration_;
    trajectory_input_.max_jerk[i] = max_jerk_;
    measured_velocity_[i] = 0.0;
    filtered_velocity_[i] = 0.0;
    const double initial_command = effort_control_ ? 0.0 : measured[i];
    if (!command_interfaces_[i].set_value(initial_command)) {
      latch_fault({FaultReason::CommandWriteFailure, i, initial_command, 0.0});
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  const auto *latest = target_buffer_.readFromRT();
  rearm_after_sequence_ = latest ? latest->sequence : received_sequence_.load(std::memory_order_relaxed);
  accepted_sequence_ = rearm_after_sequence_;
  enabled_.store(false, std::memory_order_release);
  initialized_ = true;
  was_enabled_ = following_ = false;
  rearm_pending_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn Omega7ServoController::on_deactivate(
    const rclcpp_lifecycle::State &) {
  enabled_.store(false, std::memory_order_release);
  initialized_ = following_ = false;
  rearm_pending_ = true;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type Omega7ServoController::update(
    const rclcpp::Time &, const rclcpp::Duration &period) {
  if (!initialized_) return controller_interface::return_type::ERROR;
  const double dt = period.seconds();
  if (!std::isfinite(dt) || dt <= 0 || dt > 0.02) {
    latch_fault({FaultReason::InvalidControlPeriod, kNoFaultJoint, dt, 0.02});
    return controller_interface::return_type::ERROR;
  }
  std::array<double, kJoints> measured;
  if (!read_measured(measured)) {
    latch_fault({FaultReason::InvalidMeasuredState});
    return controller_interface::return_type::ERROR;
  }
  if (effort_control_) {
    for (size_t i = 0; i < kJoints; ++i) {
      const auto velocity = state_interfaces_[2 * i + 1].get_optional<double>();
      if (!velocity.has_value() || !std::isfinite(velocity.value())) {
        latch_fault({FaultReason::InvalidMeasuredState, i,
                     velocity.has_value() ? velocity.value() :
                       std::numeric_limits<double>::quiet_NaN(), 0.0});
        return controller_interface::return_type::ERROR;
      }
      measured_velocity_[i] = velocity.value();
      filtered_velocity_[i] = 0.01 * filtered_velocity_[i] + 0.99 * measured_velocity_[i];
    }
  }
  const auto *latest = target_buffer_.readFromRT();
  const bool enabled = enabled_.load(std::memory_order_acquire);
  if (enabled && !was_enabled_) {
    rearm_after_sequence_ = latest ? latest->sequence : received_sequence_.load(std::memory_order_relaxed);
    following_ = false;
    rearm_pending_ = true;
  }
  was_enabled_ = enabled;
  bool fresh = false;
  if (latest && latest->sequence > rearm_after_sequence_) {
    const int64_t age_ns = steady_now_ns() - latest->received_ns;
    fresh = age_ns >= 0 && age_ns <= static_cast<int64_t>(target_timeout_ * 1e9);
  }
  if (!has_fault() && enabled && fresh && latest->sequence != accepted_sequence_) {
    const auto &reference = following_ ? last_target_ : measured;
    const auto jump_fault = largest_joint_delta_fault(
      latest->position, reference, max_target_jump_, FaultReason::TargetJump);
    if (jump_fault.reason != FaultReason::None) latch_fault(jump_fault);
    if (!has_fault()) {
      if (rearm_pending_) {
        // A fresh target after disable/timeout is checked against measured q above.
        // Restart the trajectory from the measured robot state before taking it over.
        for (size_t i = 0; i < kJoints; ++i) {
          const double velocity = effort_control_ ? measured_velocity_[i] : 0.0;
          command_position_[i] = measured[i];
          command_velocity_[i] = velocity;
          command_acceleration_[i] = 0.0;
          trajectory_input_.current_position[i] = measured[i];
          trajectory_input_.current_velocity[i] = velocity;
          trajectory_input_.current_acceleration[i] = 0.0;
          trajectory_input_.target_position[i] = measured[i];
          trajectory_input_.target_velocity[i] = 0.0;
          trajectory_input_.target_acceleration[i] = 0.0;
        }
      }
      last_target_ = latest->position;
      accepted_sequence_ = latest->sequence;
      following_ = true;
      rearm_pending_ = false;
    }
  }
  if ((!fresh || !enabled || has_fault()) && following_) {
    // Freeze the stop request once. Ruckig may overshoot this point slightly while producing
    // a continuous jerk-limited stop, then returns to it without moving the target forward.
    trajectory_input_.target_position = command_position_;
  }
  if (!fresh || !enabled || has_fault()) {
    following_ = false;
    rearm_pending_ = true;
    if (latest) rearm_after_sequence_ = latest->sequence;
  }
  if (following_) trajectory_input_.target_position = last_target_;
  const auto trajectory_result = trajectory_.update(trajectory_input_, trajectory_output_);
  if (trajectory_result < ruckig::Result::Working) {
    latch_fault({FaultReason::RuckigError, kNoFaultJoint,
                 static_cast<double>(static_cast<int>(trajectory_result)), 0.0});
    return controller_interface::return_type::ERROR;
  }
  command_position_ = trajectory_output_.new_position;
  command_velocity_ = trajectory_output_.new_velocity;
  command_acceleration_ = trajectory_output_.new_acceleration;
  trajectory_output_.pass_to_input(trajectory_input_);
  const auto following_fault = largest_joint_delta_fault(
    command_position_, measured, following_error_, FaultReason::FollowingError);
  if (following_fault.reason != FaultReason::None) latch_fault(following_fault);
  for (size_t i = 0; i < kJoints; ++i) {
    const double command = effort_control_ ?
      k_gains_[i] * (command_position_[i] - measured[i]) -
        d_gains_[i] * filtered_velocity_[i] :
      command_position_[i];
    if (!std::isfinite(command_position_[i]) || !std::isfinite(command)) {
      latch_fault({FaultReason::NonFiniteCommand, i, command, 0.0});
      return controller_interface::return_type::ERROR;
    }
    if (!command_interfaces_[i].set_value(command)) {
      latch_fault({FaultReason::CommandWriteFailure, i, command, 0.0});
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace omega7_servo_controller

PLUGINLIB_EXPORT_CLASS(omega7_servo_controller::Omega7ServoController,
                       controller_interface::ControllerInterface)
