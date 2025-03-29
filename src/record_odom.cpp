#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32.hpp"
#include "wall_follower/action/odom_record.hpp"

class OdomRecordServer : public rclcpp::Node {
public:
  using OdomRecord = wall_follower::action::OdomRecord;
  using GoalHandleOdomRecord = rclcpp_action::ServerGoalHandle<OdomRecord>;

  OdomRecordServer() : Node("record_odom") {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<OdomRecord>(
        this, "record_odom",
        std::bind(&OdomRecordServer::handle_goal, this, _1, _2),
        std::bind(&OdomRecordServer::handle_cancel, this, _1),
        std::bind(&OdomRecordServer::handle_accepted, this, _1));

    distance_pub_ =
        this->create_publisher<std_msgs::msg::Float32>("/total_distance", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&OdomRecordServer::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<OdomRecord>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Pose last_pose_;
  bool first_odom_{true};
  double total_distance_{0.0};
  std::vector<geometry_msgs::msg::Point32> recorded_poses_;
  bool recording_{false};
  geometry_msgs::msg::Point32 start_point;
  bool start_point_set = false;
  geometry_msgs::msg::Point32 current_point;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (!recording_)
      return;

    current_point.x = msg->pose.pose.position.x;
    current_point.y = msg->pose.pose.position.y;
    current_point.z = msg->pose.pose.orientation.z;

    if (first_odom_) {
      last_pose_ = msg->pose.pose;
      first_odom_ = false;
    } else {
      double dx = current_point.x - last_pose_.position.x;
      double dy = current_point.y - last_pose_.position.y;
      double dtheta = current_point.z - last_pose_.orientation.z;
      total_distance_ += std::sqrt(dx * dx + dy * dy);
      last_pose_ = msg->pose.pose;
    }

    recorded_poses_.push_back(current_point);
    if (!start_point_set) {
      start_point = current_point;
      start_point_set = true;
    }
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const OdomRecord::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received odometry recording goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Cancel request received");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void
  handle_accepted(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
    std::thread{std::bind(&OdomRecordServer::execute, this, goal_handle)}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleOdomRecord> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Starting odometry recording...");

    auto feedback = std::make_shared<OdomRecord::Feedback>();
    auto result = std::make_shared<OdomRecord::Result>();

    recorded_poses_.clear();
    total_distance_ = 0.0;
    first_odom_ = true;
    recording_ = true;

    rclcpp::Rate rate(1);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        recording_ = false;
        result->list_of_odoms = recorded_poses_;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Recording canceled");
        return;
      }

      feedback->current_total = total_distance_;
      goal_handle->publish_feedback(feedback);

      std_msgs::msg::Float32 msg;
      msg.data = total_distance_;
      distance_pub_->publish(msg);

      rate.sleep();

      if (start_point_set) {
        double dx = current_point.x - start_point.x;
        double dy = current_point.y - start_point.y;
        double dist_from_start = std::sqrt(dx * dx + dy * dy);

        if (dist_from_start < 0.2 && total_distance_ > 3.0) {
          RCLCPP_INFO(this->get_logger(),
                      "Lap complete. Distance from start: %.2f | Total: %.2f",
                      dist_from_start, total_distance_);
          break;
        }
      }
    }

    recording_ = false;
    result->list_of_odoms = recorded_poses_;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Recording succeeded. Total distance: %.2f",
                total_distance_);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomRecordServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
