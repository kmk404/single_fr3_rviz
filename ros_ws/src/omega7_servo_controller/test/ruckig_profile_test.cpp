#include <algorithm>
#include <array>
#include <cmath>

#include <gtest/gtest.h>
#include <ruckig/ruckig.hpp>

TEST(RuckigProfile, RespectsConfiguredLimitsAcrossTargetReversal) {
  constexpr size_t joints = 7;
  constexpr double dt = 0.001;
  constexpr double max_velocity = 0.18;
  constexpr double max_acceleration = 0.60;
  constexpr double max_jerk = 3.0;

  ruckig::Ruckig<joints> trajectory(dt);
  ruckig::InputParameter<joints> input;
  ruckig::OutputParameter<joints> output;
  input.current_position.fill(0.0);
  input.current_velocity.fill(0.0);
  input.current_acceleration.fill(0.0);
  input.target_position.fill(0.0);
  input.target_velocity.fill(0.0);
  input.target_acceleration.fill(0.0);
  input.max_velocity.fill(max_velocity);
  input.max_acceleration.fill(max_acceleration);
  input.max_jerk.fill(max_jerk);

  double previous_acceleration = 0.0;
  for (int tick = 0; tick < 8000; ++tick) {
    input.target_position[1] = tick < 3500 ? 0.08 : -0.02;
    const auto result = trajectory.update(input, output);
    ASSERT_GE(result, ruckig::Result::Working);
    EXPECT_LE(std::abs(output.new_velocity[1]), max_velocity + 1.0e-9);
    EXPECT_LE(std::abs(output.new_acceleration[1]), max_acceleration + 1.0e-9);
    EXPECT_LE(std::abs(output.new_acceleration[1] - previous_acceleration),
              max_jerk * dt + 1.0e-8);
    previous_acceleration = output.new_acceleration[1];
    output.pass_to_input(input);
  }
  EXPECT_NEAR(output.new_position[1], -0.02, 1.0e-4);
}
