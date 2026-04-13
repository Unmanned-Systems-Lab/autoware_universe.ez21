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

#ifndef MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
#define MISSION_PLANNER__ARRIVAL_CHECKER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/pose_with_uuid_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <optional>

namespace autoware::mission_planner_universe
{

class ArrivalChecker
{
public:
  using PoseWithUuidStamped = autoware_planning_msgs::msg::PoseWithUuidStamped;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  explicit ArrivalChecker(rclcpp::Node * node);
  void set_goal();
  void set_goal(const PoseWithUuidStamped & goal);
  bool is_arrived(const PoseStamped & pose) const;

private:
  using Odometry = nav_msgs::msg::Odometry;
  using TwistStamped = geometry_msgs::msg::TwistStamped;

  double angle_;
  double duration_;
  double arrival_check_lateral_distance_;
  double arrival_check_longitudinal_undershoot_distance_;
  double arrival_check_longitudinal_overshoot_distance_;
  double arrival_check_stopped_velocity_mps_;
  std::optional<PoseWithUuidStamped> goal_with_uuid_;
  rclcpp::Subscription<PoseWithUuidStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odometry_;
  rclcpp::Clock::SharedPtr clock_;
  std::deque<TwistStamped> twist_buffer_;

  void on_odometry(const Odometry::ConstSharedPtr msg);
  bool is_vehicle_stopped() const;

  static constexpr double velocity_buffer_time_sec_ = 10.0;
};

}  // namespace autoware::mission_planner_universe

#endif  // MISSION_PLANNER__ARRIVAL_CHECKER_HPP_
