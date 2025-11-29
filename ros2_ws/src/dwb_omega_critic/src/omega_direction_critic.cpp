#include "dwb_omega_critic/omega_direction_critic.hpp"

#include <algorithm>

namespace dwb_omega_critic
{

using std::placeholders::_1;

void OmegaDirectionCritic::onInit()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("OmegaDirectionCritic: Lifecycle node is expired in onInit()");
  }

  // critic 이름(namespace) 붙여서 파라미터 선언
  // 예: DWBLocalPlanner.OmegaDirectionCritic.sector_count
  const std::string full_name = dwb_plugin_name_ + "." + name_;

  // 파라미터 선언 + get
  node->declare_parameter<int>(full_name + ".sector_count", sector_count_);
  node->declare_parameter<std::string>(full_name + ".omega_topic", omega_topic_);

  node->get_parameter(full_name + ".sector_count", sector_count_);
  node->get_parameter(full_name + ".omega_topic", omega_topic_);

  if (sector_count_ <= 0) {
    RCLCPP_WARN(node->get_logger(),
      "OmegaDirectionCritic: sector_count <= 0 (%d), force to 18", sector_count_);
    sector_count_ = 18;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    omega_weights_.assign(sector_count_, 1.0f);  // 초기에는 전방향 안전하다고 가정
    has_weights_ = false;
  }

  // omega_weights 구독
  omega_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
    omega_topic_,
    rclcpp::QoS(1).best_effort(),
    std::bind(&OmegaDirectionCritic::weightsCallback, this, _1));

  RCLCPP_INFO(node->get_logger(),
    "OmegaDirectionCritic initialized. sector_count=%d, topic='%s'",
    sector_count_, omega_topic_.c_str());
}

void OmegaDirectionCritic::reset()
{
  // 새로운 global plan이 들어와도 weight는 그대로 유지.
  // 필요하면 여기서 상태 초기화 가능.
}

bool OmegaDirectionCritic::prepare(
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Twist2D &,
  const geometry_msgs::msg::Pose2D &,
  const nav_2d_msgs::msg::Path2D &)
{
  // 이 critic은 준비 단계에서 할 일이 없음
  return true;
}

void OmegaDirectionCritic::weightsCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  auto node = node_.lock();
  if (!node) {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (static_cast<int>(msg->data.size()) != sector_count_) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(),
      *node->get_clock(),
      5000,
      "OmegaDirectionCritic: received %zu weights, expected %d",
      msg->data.size(), sector_count_);
    // 그래도 가능한 범위만 사용
    omega_weights_.assign(sector_count_, 1.0f);
    const int n_copy = std::min(sector_count_, static_cast<int>(msg->data.size()));
    for (int i = 0; i < n_copy; ++i) {
      omega_weights_[i] = msg->data[i];
    }
  } else {
    omega_weights_.assign(msg->data.begin(), msg->data.end());
  }

  has_weights_ = true;
}

double OmegaDirectionCritic::computeHeadingFromTrajectory(
  const dwb_msgs::msg::Trajectory2D & traj) const
{
  if (traj.poses.empty()) {
    return 0.0;  // no movement -> 0 rad
  }

  const auto & start = traj.poses.front();
  const auto & end = traj.poses.back();

  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double dist = std::hypot(dx, dy);

  double heading = 0.0;

  if (dist < 1e-3) {
    // 제자리 회전 같은 경우: 끝 자세의 yaw 사용
    heading = end.theta;
  } else {
    // 이동 방향 기준 heading
    heading = std::atan2(dy, dx);
  }

  // [-pi, pi] 범위로 정규화
  while (heading > M_PI) {
    heading -= 2.0 * M_PI;
  }
  while (heading < -M_PI) {
    heading += 2.0 * M_PI;
  }
  return heading;
}

int OmegaDirectionCritic::headingToSectorIndex(double heading) const
{
  // 전방 180도만 사용: FOV = [-pi/2, +pi/2]
  const double fov_min = -M_PI / 2.0;
  const double fov_max = +M_PI / 2.0;

  // heading이 전방 FOV 바깥이면 -1 리턴해서 scoreTrajectory에서 최대 패널티
  if (heading < fov_min || heading > fov_max) {
    return -1;
  }

  const double fov = fov_max - fov_min; // = pi
  // heading ∈ [fov_min, fov_max] → normalized ∈ [0,1]
  double normalized = (heading - fov_min) / fov;  // 0 ~ 1

  // 0~1 → 0~sector_count_-1
  int idx = static_cast<int>(normalized * static_cast<double>(sector_count_));

  if (idx < 0) {
    idx = 0;
  }
  if (idx >= sector_count_) {
    idx = sector_count_ - 1;
  }
  return idx;
}

double OmegaDirectionCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D & traj)
{
  // 아직 weight를 받은 적 없으면 bias를 주지 않는다
  bool has_weights_copy = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    has_weights_copy = has_weights_;
  }

  if (!has_weights_copy) {
    return 0.0;  // neutral
  }

  const double heading = computeHeadingFromTrajectory(traj);
  const int sector_idx = headingToSectorIndex(heading);

  // 전방 FOV 바깥(뒤쪽 등)은 이 critic에서 "최대 패널티"
  if (sector_idx < 0) {
    // 0~1 스케일에서 최악 값. 필요하면 파라미터로 조정 가능.
    return 1.0;
  }

  float w = 1.0f;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!omega_weights_.empty() &&
        sector_idx >= 0 &&
        static_cast<size_t>(sector_idx) < omega_weights_.size())
    {
      w = omega_weights_[sector_idx];
    }
  }

  // w ∈ [0,1] (1: 안전, 0: 위험)
  // critic score는 "높을수록 나쁨"이므로 penalty = 1 - w
  double penalty = 1.0 - static_cast<double>(w);

  if (penalty < 0.0) {
    penalty = 0.0;
  }
  // 음수는 invalid trajectory 의미라 쓰면 안 됨
  return penalty;
}

}  // namespace dwb_omega_critic

#include "pluginlib/class_list_macros.hpp"

// pluginlib 등록
PLUGINLIB_EXPORT_CLASS(dwb_omega_critic::OmegaDirectionCritic, dwb_core::TrajectoryCritic)
