#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "wall_follower/srv/find_wall.hpp"
#include <chrono>
#include <memory>

using FindWall = wall_follower::srv::FindWall;
using namespace std::chrono_literals;
using std::placeholders::_1;

#define ANGULAR_SPEED 0.3
#define LINEAR_SPEED 0.1
#define STOP 0.0
#define PI 3.14159

class WallFollower : public rclcpp::Node {
public:
  WallFollower() : Node("wall_follower_node") {
    // Publisher to send velocity commands.
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // Subscriber for laser scan data.
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&WallFollower::laser_callback, this, _1));
    // Timer for executing the wall following control loop.
    timer_ = this->create_wall_timer(
        100ms, std::bind(&WallFollower::move_callback, this));
  }

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
    latest_scan_ = msg;
    front_idx = angle_to_index(msg, 0);   // 0° (front)
    right_idx = angle_to_index(msg, -90); // -90° (right)
    front_dist = min_range(msg->ranges, front_idx, 5);
    right_dist = min_range(msg->ranges, right_idx, 5);
    RCLCPP_INFO(this->get_logger(), "front: %.2f m, right: %.2f m", front_dist,
                right_dist);
  }

  void move_callback() {
    auto pub_msg = geometry_msgs::msg::Twist();
    if (!latest_scan_)
      return;
    // Basic wall following logic
    if (front_dist <= 0.35) {
      pub_msg.linear.x = STOP;
      pub_msg.angular.z = ANGULAR_SPEED;
    } else if (right_dist > 0.3) {
      pub_msg.linear.x = LINEAR_SPEED;
      pub_msg.angular.z = -ANGULAR_SPEED;
    } else if (right_dist <= 0.25) {
      pub_msg.linear.x = LINEAR_SPEED;
      pub_msg.angular.z = ANGULAR_SPEED;
    } else {
      pub_msg.linear.x = LINEAR_SPEED;
      pub_msg.angular.z = STOP;
    }
    publisher_->publish(pub_msg);
  }

private:
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

  std::shared_ptr<rclcpp::Node> client_node =
      rclcpp::Node::make_shared("wall_follower_node");

  rclcpp::Client<FindWall>::SharedPtr client =
      client_node->create_client<FindWall>("find_wall");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
  }

  auto request = std::make_shared<FindWall::Request>();
  auto result_future = client->async_send_request(request);

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(client_node, result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    if (result->wallfound == true) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned success");
    } else if (result->wallfound == false) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned false");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                 "Failed to call service /rotate");
  }

  // Now that the robot is aligned, spin the wall following node.
  std::shared_ptr<rclcpp::Node> node = std::make_shared<WallFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
