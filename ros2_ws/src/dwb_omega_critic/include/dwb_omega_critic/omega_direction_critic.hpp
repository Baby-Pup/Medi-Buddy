#pragma once

#include <vector>
#include <mutex>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_2d_msgs/msg/twist2_d.hpp"
#include "nav_2d_msgs/msg/path2_d.hpp"
#include "dwb_msgs/msg/trajectory2_d.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"

#include "dwb_core/trajectory_critic.hpp"

namespace dwb_omega_critic
{

/**
 * @brief Critic that penalizes trajectories going toward risky directions
 *        based on external omega_weights (Float32MultiArray).
 *
 * omega_weights[i] ∈ [0, 1]
 *  - 1.0 : 아주 안전한 방향
 *  - 0.0 : 매우 위험한 방향
 *
 * score = 1 - omega (0이 최상, 1이 최악)
 * 실제 planner에서는 이 score에 critic weight(scale)가 곱해진다.
 */
class OmegaDirectionCritic : public dwb_core::TrajectoryCritic
{
public:
  OmegaDirectionCritic() = default;
  ~OmegaDirectionCritic() override = default;

  // Called once after initialize()
  void onInit() override;

  // Called when a new global plan is set
  void reset() override;

  // Called once per planning cycle before scoring all trajectories
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose,
    const nav_2d_msgs::msg::Twist2D & vel,
    const geometry_msgs::msg::Pose2D & goal,
    const nav_2d_msgs::msg::Path2D & global_plan) override;

  // Main scoring function
  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj) override;

  // We don't need debrief / visualization, keep defaults
  void debrief(const nav_2d_msgs::msg::Twist2D &) override {}

protected:
  void weightsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // Convert a trajectory to heading angle (rad, [-pi, pi]) relative to robot
  double computeHeadingFromTrajectory(const dwb_msgs::msg::Trajectory2D & traj) const;

  // Map heading angle to sector index [0, sector_count_-1]
  int headingToSectorIndex(double heading) const;

protected:
  int sector_count_{72};
  std::string omega_topic_{"/future_bias/omega_weights"};

  // latest omega weights (size = sector_count_)
  std::vector<float> omega_weights_;
  bool has_weights_{false};

  std::mutex mutex_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr omega_sub_;
};

}  // namespace dwb_omega_critic
