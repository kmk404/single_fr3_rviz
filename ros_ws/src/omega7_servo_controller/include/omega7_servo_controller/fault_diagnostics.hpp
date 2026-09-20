#pragma once

#include <array>
#include <cstddef>
#include <limits>

namespace omega7_servo_controller {

enum class FaultReason : int {
  None = 0,
  MessageStructure,
  JointNameMismatch,
  NonFiniteTarget,
  TargetNearPositionLimit,
  TargetJump,
  RuckigError,
  FollowingError,
  CommandWriteFailure,
  NonFiniteCommand,
  InvalidControlPeriod,
  InvalidMeasuredState,
};

constexpr const char *fault_reason_name(FaultReason reason) {
  switch (reason) {
    case FaultReason::None: return "none";
    case FaultReason::MessageStructure: return "message structure mismatch";
    case FaultReason::JointNameMismatch: return "joint name/order mismatch";
    case FaultReason::NonFiniteTarget: return "non-finite target";
    case FaultReason::TargetNearPositionLimit: return "target near position limit";
    case FaultReason::TargetJump: return "target jump";
    case FaultReason::RuckigError: return "Ruckig error";
    case FaultReason::FollowingError: return "following error";
    case FaultReason::CommandWriteFailure: return "command interface write failure";
    case FaultReason::NonFiniteCommand: return "non-finite command";
    case FaultReason::InvalidControlPeriod: return "invalid control period";
    case FaultReason::InvalidMeasuredState: return "invalid measured state";
  }
  return "unknown";
}

constexpr size_t kNoFaultJoint = static_cast<size_t>(-1);

struct FaultDetail {
  FaultReason reason = FaultReason::None;
  size_t joint_index = kNoFaultJoint;
  double observed = std::numeric_limits<double>::quiet_NaN();
  double limit = std::numeric_limits<double>::quiet_NaN();
};

template <size_t N>
FaultDetail largest_joint_delta_fault(const std::array<double, N> &values,
                                      const std::array<double, N> &reference,
                                      double threshold, FaultReason reason) {
  size_t largest_index = kNoFaultJoint;
  double largest_delta = threshold;
  for (size_t i = 0; i < N; ++i) {
    const double delta = values[i] > reference[i] ?
      values[i] - reference[i] : reference[i] - values[i];
    if (delta > largest_delta) {
      largest_delta = delta;
      largest_index = i;
    }
  }
  if (largest_index == kNoFaultJoint) return {};
  return {reason, largest_index, largest_delta, threshold};
}

template <size_t N>
FaultDetail position_margin_fault(const std::array<double, N> &values,
                                  const std::array<double, N> &lower,
                                  const std::array<double, N> &upper,
                                  double margin) {
  size_t worst_index = kNoFaultJoint;
  double worst_violation = 0.0;
  double observed = std::numeric_limits<double>::quiet_NaN();
  double boundary = std::numeric_limits<double>::quiet_NaN();
  for (size_t i = 0; i < N; ++i) {
    const double low_boundary = lower[i] + margin;
    const double high_boundary = upper[i] - margin;
    const double violation = values[i] < low_boundary ? low_boundary - values[i] :
      (values[i] > high_boundary ? values[i] - high_boundary : 0.0);
    if (violation > worst_violation) {
      worst_violation = violation;
      worst_index = i;
      observed = values[i];
      boundary = values[i] < low_boundary ? low_boundary : high_boundary;
    }
  }
  if (worst_index == kNoFaultJoint) return {};
  return {FaultReason::TargetNearPositionLimit, worst_index, observed, boundary};
}

}  // namespace omega7_servo_controller
