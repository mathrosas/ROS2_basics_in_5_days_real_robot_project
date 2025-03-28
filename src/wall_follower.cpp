#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "wall_follower/srv/find_wall.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;
using std::placeholders::_1;

class WallFollower : public rclcpp::Node {
public:
  WallFollower() : Node("wall_follower_node") {
    declare_parameters();
    initialize_components();
  }

private:
  struct ControlParams {
    float safe_distance{0.35f};
    float target_gap{0.3f};
    float min_gap{0.25f};
    float linear_speed{0.1f};
    float angular_speed{0.3f};
    int smoothing_window{5};
  } params_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex scan_mutex_;
  float front_dist_{0.0f};
  float right_dist_{0.0f};

  void declare_parameters() {
    this->declare_parameter("safe_distance", 0.35);
    this->declare_parameter("target_gap", 0.3);
    this->declare_parameter("min_gap", 0.25);
    this->declare_parameter("linear_speed", 0.1);
    this->declare_parameter("angular_speed", 0.3);
    this->declare_parameter("smoothing_window", 5);
  }

  void update_parameters() {
    params_.safe_distance = this->get_parameter("safe_distance").as_double();
    params_.target_gap = this->get_parameter("target_gap").as_double();
    params_.min_gap = this->get_parameter("min_gap").as_double();
    params_.linear_speed = this->get_parameter("linear_speed").as_double();
    params_.angular_speed = this->get_parameter("angular_speed").as_double();
    params_.smoothing_window = this->get_parameter("smoothing_window").as_int();
  }

  void initialize_components() {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&WallFollower::laser_callback, this, _1));
    timer_ = this->create_wall_timer(
        100ms, std::bind(&WallFollower::control_loop, this));
  }

  int angle_to_index(const sensor_msgs::msg::LaserScan &scan,
                     float angle_deg) const {
    const float angle_rad = angle_deg * M_PI / 180.0f;
    const int index = static_cast<int>(
        std::round((angle_rad - scan.angle_min) / scan.angle_increment));
    return (index + scan.ranges.size()) % scan.ranges.size();
  }

  float get_min_range(const std::vector<float> &ranges, int center) const {
    float min_val = std::numeric_limits<float>::infinity();
    const int start = std::max(0, center - params_.smoothing_window);
    const int end = std::min(static_cast<int>(ranges.size()),
                             center + params_.smoothing_window + 1);

    for (int i = start; i < end; ++i) {
      if (std::isfinite(ranges[i])) {
        min_val = std::min(min_val, ranges[i]);
      }
    }
    return min_val;
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;

    if (!msg)
      return;

    const int front_idx = angle_to_index(*msg, 0);
    const int right_idx = angle_to_index(*msg, -90);

    front_dist_ = get_min_range(msg->ranges, front_idx);
    right_dist_ = get_min_range(msg->ranges, right_idx);

    RCLCPP_DEBUG(get_logger(), "Front: %.2fm | Right: %.2fm", front_dist_,
                 right_dist_);
  }

  void control_loop() {
    update_parameters();
    std::unique_lock<std::mutex> lock(scan_mutex_, std::try_to_lock);
    if (!lock || !latest_scan_)
      return;

    auto msg = geometry_msgs::msg::Twist();

    if (front_dist_ <= params_.safe_distance) {
      // Obstacle avoidance
      msg.linear.x = 0.0;
      msg.angular.z = params_.angular_speed;
    } else if (right_dist_ > params_.target_gap) {
      // Close gap with wall
      msg.linear.x = params_.linear_speed;
      msg.angular.z = -params_.angular_speed;
    } else if (right_dist_ <= params_.min_gap) {
      // Avoid wall collision
      msg.linear.x = params_.linear_speed;
      msg.angular.z = params_.angular_speed;
    } else {
      // Maintain course
      msg.linear.x = params_.linear_speed;
      msg.angular.z = 0.0;
    }

    publisher_->publish(msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<rclcpp::Node>("wall_follower_client");
  auto client =
      client_node->create_client<wall_follower::srv::FindWall>("find_wall");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(client_node->get_logger(), "Service interrupted");
      return 1;
    }
    RCLCPP_INFO(client_node->get_logger(), "Awaiting service...");
  }

  auto request = std::make_shared<wall_follower::srv::FindWall::Request>();
  auto future_result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(client_node, future_result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(client_node->get_logger(), "Service response: %s",
                future_result.get()->wallfound ? "Success" : "Failure");
  }

  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
  return 0;
}