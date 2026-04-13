// Copyright 2022 TIER IV, Inc.
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

#include "arrival_checker.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <tf2/utils.hpp>

#include <cmath>
#include <functional>

namespace autoware::mission_planner_universe
{

ArrivalChecker::ArrivalChecker(rclcpp::Node * node)
{
  using std::placeholders::_1;

  const double angle_deg = node->declare_parameter<double>("arrival_check_angle_deg");
  angle_ = autoware_utils::deg2rad(angle_deg);
  arrival_check_lateral_distance_ =
    node->declare_parameter<double>("arrival_check_lateral_distance");
  arrival_check_longitudinal_undershoot_distance_ =
    node->declare_parameter<double>("arrival_check_longitudinal_undershoot_distance");
  arrival_check_longitudinal_overshoot_distance_ =
    node->declare_parameter<double>("arrival_check_longitudinal_overshoot_distance");
  duration_ = node->declare_parameter<double>("arrival_check_duration");
  arrival_check_stopped_velocity_mps_ =
    node->declare_parameter<double>("arrival_check_stopped_velocity_mps", 0.05);
  clock_ = node->get_clock();
  sub_odometry_ = node->create_subscription<Odometry>(
    "/localization/kinematic_state", rclcpp::QoS(1),
    std::bind(&ArrivalChecker::on_odometry, this, _1));
}

void ArrivalChecker::set_goal()
{
  // Ignore the modified goal after the route is cleared.
  goal_with_uuid_ = std::nullopt;
}

void ArrivalChecker::set_goal(const PoseWithUuidStamped & goal)
{
  // Ignore the modified goal for the previous route using uuid.
  goal_with_uuid_ = goal;
}

bool ArrivalChecker::is_arrived(const PoseStamped & pose) const
{
  if (!goal_with_uuid_) {
    return false;
  }
  const auto goal = goal_with_uuid_.value();

  // Check frame id
  if (goal.header.frame_id != pose.header.frame_id) {
    return false;
  }

  // Compute position difference in goal frame
  const double dx = pose.pose.position.x - goal.pose.position.x;
  const double dy = pose.pose.position.y - goal.pose.position.y;

  const double yaw_goal = tf2::getYaw(goal.pose.orientation);
  const double longitudinal_offset_to_goal = std::cos(yaw_goal) * dx + std::sin(yaw_goal) * dy;
  const double lateral_offset_to_goal = -std::sin(yaw_goal) * dx + std::cos(yaw_goal) * dy;

  const double yaw_pose = tf2::getYaw(pose.pose.orientation);
  const double yaw_diff = autoware_utils::normalize_radian(yaw_pose - yaw_goal);

  // Check lateral distance.
  const bool is_within_lateral_range =
    std::abs(lateral_offset_to_goal) <= arrival_check_lateral_distance_;

  // Check longitudinal distance.
  // The acceptable range is [-arrival_check_longitudinal_undershoot_distance,
  // +arrival_check_longitudinal_overshoot_distance_].
  const bool is_within_longitudinal_range =
    longitudinal_offset_to_goal >= -arrival_check_longitudinal_undershoot_distance_ &&
    longitudinal_offset_to_goal <= arrival_check_longitudinal_overshoot_distance_;

  // Check angle.
  const bool is_within_angle_range = std::fabs(yaw_diff) <= angle_;

  if (!is_within_lateral_range || !is_within_longitudinal_range || !is_within_angle_range) {
    return false;
  }

  // Check vehicle stopped.
  return is_vehicle_stopped();
}

void ArrivalChecker::on_odometry(const Odometry::ConstSharedPtr msg)
{
  TwistStamped twist_msg;
  twist_msg.header = msg->header;
  twist_msg.twist = msg->twist.twist;
  if (twist_msg.header.stamp.sec == 0 && twist_msg.header.stamp.nanosec == 0) {
    twist_msg.header.stamp = clock_->now();
  }

  twist_buffer_.push_back(twist_msg);

  const auto latest_stamp = rclcpp::Time(twist_msg.header.stamp);
  const auto cutoff = latest_stamp - rclcpp::Duration::from_seconds(velocity_buffer_time_sec_);
  while (!twist_buffer_.empty() && rclcpp::Time(twist_buffer_.front().header.stamp) < cutoff) {
    twist_buffer_.pop_front();
  }
}

bool ArrivalChecker::is_vehicle_stopped() const
{
  if (twist_buffer_.empty()) {
    return false;
  }

  const auto latest_stamp = rclcpp::Time(twist_buffer_.back().header.stamp);
  const auto required_start = latest_stamp - rclcpp::Duration::from_seconds(duration_);
  if (rclcpp::Time(twist_buffer_.front().header.stamp) > required_start) {
    return false;
  }

  const double velocity_threshold_sq =
    arrival_check_stopped_velocity_mps_ * arrival_check_stopped_velocity_mps_;
  for (auto it = twist_buffer_.rbegin(); it != twist_buffer_.rend(); ++it) {
    const auto stamp = rclcpp::Time(it->header.stamp);
    const auto & linear = it->twist.linear;
    const double velocity_sq =
      linear.x * linear.x + linear.y * linear.y + linear.z * linear.z;
    if (velocity_sq > velocity_threshold_sq) {
      return false;
    }
    if (stamp <= required_start) {
      return true;
    }
  }

  return false;
}

}  // namespace autoware::mission_planner_universe
