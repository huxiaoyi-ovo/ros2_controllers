// Copyright (c) 2026 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <memory>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

namespace joint_trajectory_controller
{
namespace
{

class RealtimeGoalHandleAccessor : public JointTrajectoryController
{
public:
  using RealtimeGoalHandleForTest = RealtimeGoalHandle;
};

class RealtimeGoalHandleTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

// cppcheck-suppress syntaxError
TEST_F(RealtimeGoalHandleTest, feedback_buffers_rotate_after_successful_handoff)
{
  using Action = control_msgs::action::FollowJointTrajectory;
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using RealtimeGoalHandle = RealtimeGoalHandleAccessor::RealtimeGoalHandleForTest;

  std::shared_ptr<ServerGoalHandle> goal_handle;
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);

  auto first_buffer = rt_goal->preallocated_feedback_;
  ASSERT_NE(first_buffer, nullptr);

  // Mirror the goal callback: size the primary feedback before execute() clones the RT buffer.
  first_buffer->joint_names = {"joint1", "joint2"};
  first_buffer->actual.positions.resize(2);
  first_buffer->desired.positions.resize(2);
  first_buffer->error.positions.resize(2);

  // Use the same calls as the controller so this also verifies the compatibility adapters.
  rt_goal->execute();
  ASSERT_TRUE(rt_goal->setFeedback(first_buffer));

  auto second_buffer = rt_goal->preallocated_feedback_;
  ASSERT_NE(second_buffer, nullptr);
  EXPECT_NE(first_buffer.get(), second_buffer.get());
  EXPECT_EQ(first_buffer->joint_names, second_buffer->joint_names);
  EXPECT_EQ(first_buffer->actual.positions.size(), second_buffer->actual.positions.size());
  EXPECT_EQ(first_buffer->desired.positions.size(), second_buffer->desired.positions.size());
  EXPECT_EQ(first_buffer->error.positions.size(), second_buffer->error.positions.size());

  ASSERT_TRUE(rt_goal->setFeedback(second_buffer));
  EXPECT_EQ(first_buffer.get(), rt_goal->preallocated_feedback_.get());
}

// cppcheck-suppress syntaxError
TEST_F(RealtimeGoalHandleTest, uninitialized_feedback_buffers_reject_handoff)
{
  using Action = control_msgs::action::FollowJointTrajectory;
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  using RealtimeGoalHandle = RealtimeGoalHandleAccessor::RealtimeGoalHandleForTest;

  std::shared_ptr<ServerGoalHandle> goal_handle;
  auto rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);

  EXPECT_FALSE(rt_goal->set_feedback_from_rt());
}

}  // namespace
}  // namespace joint_trajectory_controller
