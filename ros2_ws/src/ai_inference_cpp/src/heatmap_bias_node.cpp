#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstring> // for memcpy

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp" // [중요] UInt8 사용

using std::placeholders::_1;
using std::vector;

static constexpr int GRID_SIZE   = 128;
static constexpr int T_OUT       = 16;
static constexpr int NUM_SECTORS = 72;

static constexpr float ALPHA      = 3.0f;
static constexpr float CELL_SIZE  = 0.1f;
static constexpr float DIST_DECAY = 0.4f;

// 가중치 인덱스
static constexpr int MID_START = 4;
static constexpr int MID_END   = 10;
static constexpr int MID_COUNT = (MID_END - MID_START + 1);

class HeatmapBiasNode : public rclcpp::Node
{
public:
  HeatmapBiasNode()
  : Node("heatmap_bias_node"),
    H_(GRID_SIZE),
    W_(GRID_SIZE),
    cy_(GRID_SIZE / 2),
    cx_(GRID_SIZE / 2),
    max_ray_steps_(std::min(GRID_SIZE, GRID_SIZE) / 2)
  {
    // [수정] UInt8MultiArray 구독
    sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/future_occupancy", 10,
      std::bind(&HeatmapBiasNode::onFutureOccupancy, this, _1));

    pub_risk_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/future_bias/risk_map", 10);

    pub_omega_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/future_bias/omega_weights", 10);

    RCLCPP_INFO(this->get_logger(),
      "🔥 Heatmap Bias Node (Optimized UInt8 Input) Started");
  }

private:
  void onFutureOccupancy(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
  {
    // 1. 데이터 검증 (Byte Size 체크)
    const size_t byte_size = msg->data.size();
    const size_t expected_floats = static_cast<size_t>(T_OUT) * H_ * W_;
    const size_t expected_bytes = expected_floats * sizeof(float);

    if (byte_size != expected_bytes) {
      RCLCPP_WARN(this->get_logger(),
        "Size mismatch! Got %zu bytes, expected %zu bytes",
        byte_size, expected_bytes);
      return;
    }

    // 2. [핵심] 바이트 포인터를 Float 포인터로 재해석 (Zero-copy logic)
    const float* raw_data = reinterpret_cast<const float*>(msg->data.data());

    // 3. Risk Map 계산 (이전과 로직 동일)
    vector<float> risk_map(H_ * W_, 0.0f);
    const float weights[MID_COUNT] = {0.3f, 0.25f, 0.2f, 0.15f, 0.1f, 0.05f, 0.05f};

    // raw_data[idx]는 float처럼 접근 가능
    for (int y = 0; y < H_; ++y) {
      for (int x = 0; x < W_; ++x) {
        float acc = 0.0f;
        int w_idx = 0;
        for (int t = MID_START; t <= MID_END; ++t, ++w_idx) {
          size_t idx = static_cast<size_t>(t) * H_ * W_
                     + static_cast<size_t>(y) * W_
                     + static_cast<size_t>(x);
          acc += weights[w_idx] * raw_data[idx];
        }
        risk_map[y * W_ + x] = acc;
      }
    }

    // 4. 섹터 위험도 계산
    vector<float> sector_risk(NUM_SECTORS, 0.0f);
    computeSectorRisk(risk_map, sector_risk);

    // 5. 가중치 변환
    vector<float> omega(NUM_SECTORS, 1.0f);
    riskToWeights(sector_risk, omega);

    // 6. Publish (결과는 다시 Float32로 보냄 - 시각화용)
    std_msgs::msg::Float32MultiArray risk_msg;
    risk_msg.data = risk_map; 
    pub_risk_->publish(risk_msg);

    std_msgs::msg::Float32MultiArray omega_msg;
    omega_msg.data = omega;
    pub_omega_->publish(omega_msg);
  }

  void computeSectorRisk(const vector<float> & risk_map,
                         vector<float> & sector_vals)
  {
    const float two_pi = 2.0f * static_cast<float>(M_PI);

    for (int i = 0; i < NUM_SECTORS; ++i) {
      float angle_center = -static_cast<float>(M_PI)
                         + (static_cast<float>(i) + 0.5f) * (two_pi / NUM_SECTORS);
      float cos_a = std::cos(angle_center);
      float sin_a = std::sin(angle_center);

      float ray_max = 0.0f;

      for (int step = 2; step < max_ray_steps_; ++step) {
        float row = static_cast<float>(cy_) - step * cos_a;
        float col = static_cast<float>(cx_) - step * sin_a;

        int iy = static_cast<int>(std::round(row));
        int ix = static_cast<int>(std::round(col));

        if (ix < 0 || ix >= W_ || iy < 0 || iy >= H_) break;

        float base = risk_map[iy * W_ + ix];
        if (base <= 0.0f) continue;

        float dist_m = static_cast<float>(step) * CELL_SIZE;
        float eff    = base * std::exp(-DIST_DECAY * dist_m);

        if (eff > ray_max) ray_max = eff;
      }
      sector_vals[i] = ray_max;
    }
  }

  void riskToWeights(const vector<float> & sector_risk,
                     vector<float> & weights)
  {
    float max_w = 0.0f;
    for (int i = 0; i < NUM_SECTORS; ++i) {
      float w = std::exp(-ALPHA * sector_risk[i]);
      weights[i] = w;
      if (w > max_w) max_w = w;
    }

    if (max_w < 1e-6f) {
      std::fill(weights.begin(), weights.end(), 1.0f);
      return;
    }
    for (float & w : weights) {
      w /= max_w;
    }
  }

private:
  int H_, W_, cy_, cx_, max_ray_steps_;
  
  // [수정] 구독 타입
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
  
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_risk_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_omega_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HeatmapBiasNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}