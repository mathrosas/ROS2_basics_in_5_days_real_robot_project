#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "wall_follower/srv/find_wall.hpp"
#include <algorithm>
#include <limits>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

const float PI = 3.14159265359;
const float DISTANCE = 0.3;
const float TOLERANCE = 5 * PI / 180.0; // Convert degrees to radians explicitly

class WallFinder : public rclcpp::Node {
public:
  WallFinder() : Node("wall_finder_node") {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&WallFinder::laser_callback, this, _1));

    callback_group_service_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    find_wall_srv_ = create_service<wall_follower::srv::FindWall>(
        "find_wall", std::bind(&WallFinder::find_wall_callback, this, _1, _2),
        rmw_qos_profile_services_default, callback_group_service_);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Service<wall_follower::srv::FindWall>::SharedPtr find_wall_srv_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;

  int angle_to_index(const sensor_msgs::msg::LaserScan::SharedPtr &scan,
                     float desired_angle_deg) {
    float desired_angle_rad = desired_angle_deg * PI / 180.0;
    int index = static_cast<int>(std::round(
        (desired_angle_rad - scan->angle_min) / scan->angle_increment));
    return (index + scan->ranges.size()) % scan->ranges.size();
  }

  std::pair<float, int> min_dist(const std::vector<float> &ranges) {
    float min_val = std::numeric_limits<float>::infinity();
    int min_idx = 0;
    for (size_t i = 0; i < ranges.size(); i++) {
      if (std::isfinite(ranges[i]) && ranges[i] < min_val) {
        min_val = ranges[i];
        min_idx = i;
      }
    }
    return {min_val, min_idx};
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
  }

  void find_wall_callback(
      const std::shared_ptr<wall_follower::srv::FindWall::Request>,
      std::shared_ptr<wall_follower::srv::FindWall::Response> response) {

    geometry_msgs::msg::Twist twist;
    rclcpp::Rate rate(10);
    int retries = 50; // wait explicitly up to 5 seconds for laser data

    // Explicitly wait for laser data before proceeding
    while (rclcpp::ok() && !latest_scan_ && retries-- > 0) {
      RCLCPP_WARN(get_logger(), "Waiting for laser scan data...");
      rate.sleep();
    }

    if (!latest_scan_) {
      RCLCPP_WARN(get_logger(), "No laser data received yet!");
      response->wallfound = false;
      return;
    }

    RCLCPP_INFO(get_logger(), "Laser data received. Starting wall alignment.");

    // Step 1: Rotate until facing the closest wall
    while (rclcpp::ok()) {
      auto [_, min_idx] = min_dist(latest_scan_->ranges);
      int front_idx = angle_to_index(latest_scan_, 0);
      float angle_diff_front =
          (min_idx - front_idx) * latest_scan_->angle_increment;

      if (std::abs(angle_diff_front) <= TOLERANCE)
        break;

      twist.angular.z = (angle_diff_front > 0) ? 0.2 : -0.2;
      twist.linear.x = 0;
      vel_pub_->publish(twist);
      rate.sleep();
    }

    twist.angular.z = 0;
    vel_pub_->publish(twist);
    rate.sleep();

    // Step 2: Move forward until at distance = 0.3m from wall
    while (rclcpp::ok()) {
      int front_idx = angle_to_index(latest_scan_, 0);
      float front_dist = latest_scan_->ranges[front_idx];

      if (front_dist <= DISTANCE)
        break;

      twist.linear.x = 0.1;
      twist.angular.z = 0;
      vel_pub_->publish(twist);
      rate.sleep();
    }

    twist.linear.x = 0;
    vel_pub_->publish(twist);
    rate.sleep();

    // Step 3: Rotate until wall is exactly on the right side (-90 degrees)
    while (rclcpp::ok()) {
      auto [_, min_idx] = min_dist(latest_scan_->ranges);
      int right_idx = angle_to_index(latest_scan_, -90);
      // It aligns better to the wall using my angle to idx function than simply
      // using right_idx as 270
      //   int right_idx = 270;
      float angle_diff_right =
          (min_idx - right_idx) * latest_scan_->angle_increment;

      if (std::abs(angle_diff_right) <= TOLERANCE)
        break;

      twist.angular.z = (angle_diff_right > 0) ? 0.2 : -0.2;
      twist.linear.x = 0;
      vel_pub_->publish(twist);
      rate.sleep();
    }

    twist.angular.z = 0;
    vel_pub_->publish(twist);
    rate.sleep();

    response->wallfound = true;
    RCLCPP_INFO(get_logger(), "Wall found and aligned successfully.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WallFinder>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}