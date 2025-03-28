#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "wall_follower/srv/find_wall.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>

using std::placeholders::_1;
using std::placeholders::_2;

class WallFinder : public rclcpp::Node {
public:
  WallFinder() : Node("wall_finder_node") {
    declare_parameters();
    initialize_components();
  }

private:
  struct Parameters {
    float target_distance{0.3f};
    float angular_tolerance{5.0f};
    float approach_speed{0.1f};
    float rotation_speed{0.2f};
    int max_retries{50};
  } params_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Service<wall_follower::srv::FindWall>::SharedPtr find_wall_srv_;
  rclcpp::CallbackGroup::SharedPtr callback_group_service_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex scan_mutex_;

  void declare_parameters() {
    this->declare_parameter("target_distance", 0.3);
    this->declare_parameter("angular_tolerance", 5.0);
    this->declare_parameter("approach_speed", 0.1);
    this->declare_parameter("rotation_speed", 0.2);
    this->declare_parameter("max_retries", 50);
  }

  void update_parameters() {
    params_.target_distance =
        this->get_parameter("target_distance").as_double();
    params_.angular_tolerance =
        this->get_parameter("angular_tolerance").as_double() * M_PI / 180.0;
    params_.approach_speed = this->get_parameter("approach_speed").as_double();
    params_.rotation_speed = this->get_parameter("rotation_speed").as_double();
    params_.max_retries = this->get_parameter("max_retries").as_int();
  }

  void initialize_components() {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&WallFinder::laser_callback, this, _1));

    callback_group_service_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    find_wall_srv_ = create_service<wall_follower::srv::FindWall>(
        "find_wall", std::bind(&WallFinder::find_wall_callback, this, _1, _2),
        rmw_qos_profile_services_default, callback_group_service_);
  }

  int angle_to_index(const sensor_msgs::msg::LaserScan &scan,
                     float desired_angle_deg) const {
    const float desired_angle_rad = desired_angle_deg * M_PI / 180.0f;
    const int index = static_cast<int>(std::round(
        (desired_angle_rad - scan.angle_min) / scan.angle_increment));
    return (index + scan.ranges.size()) % scan.ranges.size();
  }

  std::pair<float, int>
  find_min_distance(const std::vector<float> &ranges) const {
    float min_val = std::numeric_limits<float>::infinity();
    int min_idx = 0;
    for (size_t i = 0; i < ranges.size(); ++i) {
      if (std::isfinite(ranges[i]) && ranges[i] < min_val) {
        min_val = ranges[i];
        min_idx = i;
      }
    }
    return {min_val, min_idx};
  }

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    latest_scan_ = msg;
  }

  void execute_rotation(geometry_msgs::msg::Twist &twist, int target_idx,
                        rclcpp::Rate &rate, const std::string &debug_msg) {
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        if (!latest_scan_)
          continue;

        const auto [_, min_idx] = find_min_distance(latest_scan_->ranges);
        const float angle_diff =
            (min_idx - target_idx) * latest_scan_->angle_increment;

        if (std::abs(angle_diff) <= params_.angular_tolerance)
          break;

        twist.angular.z =
            (angle_diff > 0) ? params_.rotation_speed : -params_.rotation_speed;
      }

      vel_pub_->publish(twist);
      rate.sleep();
    }
    twist.angular.z = 0.0;
    vel_pub_->publish(twist);
    RCLCPP_DEBUG(get_logger(), "%s", debug_msg.c_str());
  }

  void find_wall_callback(
      const std::shared_ptr<wall_follower::srv::FindWall::Request>,
      std::shared_ptr<wall_follower::srv::FindWall::Response> response) {

    update_parameters();
    geometry_msgs::msg::Twist twist;
    rclcpp::Rate rate(10);
    int retries = params_.max_retries;

    // Wait for initial scan data
    {
      std::unique_lock<std::mutex> lock(scan_mutex_);
      while (rclcpp::ok() && !latest_scan_ && retries-- > 0) {
        lock.unlock();
        RCLCPP_DEBUG(get_logger(), "Awaiting laser scan data...");
        rate.sleep();
        lock.lock();
      }

      if (!latest_scan_) {
        RCLCPP_ERROR(get_logger(), "Laser data unavailable");
        response->wallfound = false;
        return;
      }
    }

    // Phase 1: Align with nearest wall
    execute_rotation(twist, angle_to_index(*latest_scan_, 0), rate,
                     "Front alignment complete");

    // Phase 2: Approach wall
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        const float front_dist =
            latest_scan_->ranges[angle_to_index(*latest_scan_, 0)];
        if (front_dist <= params_.target_distance)
          break;
      }
      twist.linear.x = params_.approach_speed;
      vel_pub_->publish(twist);
      rate.sleep();
    }
    twist.linear.x = 0.0;
    vel_pub_->publish(twist);

    // Phase 3: Align to right side
    execute_rotation(twist, angle_to_index(*latest_scan_, -90), rate,
                     "Right alignment complete");

    response->wallfound = true;
    RCLCPP_INFO(get_logger(), "Wall alignment successful");
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