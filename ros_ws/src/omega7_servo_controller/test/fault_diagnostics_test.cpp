#include <array>

#include <gtest/gtest.h>

#include "omega7_servo_controller/fault_diagnostics.hpp"

namespace omega7_servo_controller {

TEST(FaultDiagnostics, IdentifiesTargetJumpJointAndMagnitude) {
  const std::array<double, 3> target{{0.0, 0.051, 0.0}};
  const std::array<double, 3> reference{{0.0, 0.0, 0.0}};
  const auto fault = largest_joint_delta_fault(
    target, reference, 0.04, FaultReason::TargetJump);
  EXPECT_EQ(fault.reason, FaultReason::TargetJump);
  EXPECT_EQ(fault.joint_index, 1U);
  EXPECT_DOUBLE_EQ(fault.observed, 0.051);
  EXPECT_DOUBLE_EQ(fault.limit, 0.04);
}

TEST(FaultDiagnostics, IdentifiesFollowingErrorSeparately) {
  const std::array<double, 2> command{{0.0, 0.09}};
  const std::array<double, 2> measured{{0.0, 0.0}};
  const auto fault = largest_joint_delta_fault(
    command, measured, 0.08, FaultReason::FollowingError);
  EXPECT_EQ(fault.reason, FaultReason::FollowingError);
  EXPECT_EQ(fault.joint_index, 1U);
  EXPECT_DOUBLE_EQ(fault.observed, 0.09);
  EXPECT_DOUBLE_EQ(fault.limit, 0.08);
}

TEST(FaultDiagnostics, IdentifiesPositionMarginBoundary) {
  const std::array<double, 2> target{{0.0, 0.99}};
  const std::array<double, 2> lower{{-1.0, -1.0}};
  const std::array<double, 2> upper{{1.0, 1.0}};
  const auto fault = position_margin_fault(target, lower, upper, 0.02);
  EXPECT_EQ(fault.reason, FaultReason::TargetNearPositionLimit);
  EXPECT_EQ(fault.joint_index, 1U);
  EXPECT_DOUBLE_EQ(fault.observed, 0.99);
  EXPECT_DOUBLE_EQ(fault.limit, 0.98);
}

TEST(FaultDiagnostics, ExposesHumanReadableFaultNames) {
  EXPECT_STREQ(fault_reason_name(FaultReason::TargetJump), "target jump");
  EXPECT_STREQ(fault_reason_name(FaultReason::FollowingError), "following error");
}

}  // namespace omega7_servo_controller
