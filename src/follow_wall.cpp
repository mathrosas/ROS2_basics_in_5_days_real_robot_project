#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define ANGULAR_SPEED 0.3
#define LINEAR_SPEED 0.1
#define STOP 0.0
#define PI 3.14159

class WallFollower : public rclcpp::Node {
public:
  WallFollower() : Node("wall_follower_node") {
    // Initialize publisher
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Initialize subscription
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&WallFollower::laser_callback, this, _1));

    // Initialize timer
    timer_ = this->create_wall_timer(
        100ms, std::bind(&WallFollower::move_callback, this));
  }

private:
  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                     float desired_angle_deg) {
    float desired_angle_rad = desired_angle_deg * PI / 180.0;
    int index = static_cast<int>(std::round(
        (desired_angle_rad - scan->angle_min) / scan->angle_increment));
    index =
        (index % static_cast<int>(scan->ranges.size()) + scan->ranges.size()) %
        scan->ranges.size();
    return index;
  }

  float min_range(const std::vector<float> &ranges, int center,
                  int window = 5) {
    float min_val = std::numeric_limits<float>::infinity();
    int start = std::max(0, center - window);
    int end = std::min(static_cast<int>(ranges.size()), center + window);

    for (int i = start; i < end; ++i) {
      if (std::isfinite(ranges[i]) && ranges[i] < min_val) {
        min_val = ranges[i];
      }
    }
    return min_val;
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    this->latest_scan_ = msg;

    this->front_idx = angle_to_index(msg, 0);   // 0 degrees
    this->right_idx = angle_to_index(msg, -90); // 90 degrees

    this->front_dist = min_range(msg->ranges, front_idx, 5);
    this->right_dist = min_range(msg->ranges, right_idx, 5);

    RCLCPP_INFO(this->get_logger(), "front: %.2f m, right: %.2f m",
                this->front_dist, this->right_dist);
  }

  void move_callback() {
    auto pub_msg = geometry_msgs::msg::Twist();

    if (!this->latest_scan_)
      return;

    // Obstacle avoidance logic
    if (this->front_dist <= 0.35) {
      pub_msg.linear.x = STOP;
      pub_msg.angular.z = ANGULAR_SPEED;
    } else if (this->right_dist > 0.3) {
      pub_msg.linear.x = LINEAR_SPEED;
      pub_msg.angular.z = -ANGULAR_SPEED;
    } else if (this->right_dist <= 0.25) {
      pub_msg.linear.x = LINEAR_SPEED;
      pub_msg.angular.z = ANGULAR_SPEED;
    } else {
      pub_msg.linear.x = LINEAR_SPEED;
      pub_msg.angular.z = STOP;
    }

    publisher_->publish(pub_msg);
  }

  // Member variables
  int front_idx;
  int right_idx;
  float front_dist;
  float right_dist;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
  return 0;
}
