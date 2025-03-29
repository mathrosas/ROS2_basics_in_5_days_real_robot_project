#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "wall_follower/action/odom_record.hpp"
#include "wall_follower/srv/find_wall.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class WallFollower : public rclcpp::Node {
public:
  using OdomRecord = wall_follower::action::OdomRecord;
  using GoalHandleOdomRecord = rclcpp_action::ClientGoalHandle<OdomRecord>;

  WallFollower() : Node("wall_follower_node") {
    declare_parameters();
    initialize_components();
    init_clients();
  }

private:
  struct ControlParams {
    float safe_distance{0.35f};
    float target_gap{0.3f};
    float min_gap{0.25f};
    float linear_speed{0.1f};
    float angular_speed{0.3f};
    int smoothing_window{15};
  } params_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<wall_follower::srv::FindWall>::SharedPtr find_wall_client_;
  rclcpp_action::Client<OdomRecord>::SharedPtr odom_action_client_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  std::mutex scan_mutex_;
  float front_dist_{0.0f};
  float right_dist_{0.0f};
  bool ready_to_move_{false};
  bool alignment_complete_{false};

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

  void init_clients() {
    find_wall_client_ =
        this->create_client<wall_follower::srv::FindWall>("find_wall");
    odom_action_client_ =
        rclcpp_action::create_client<OdomRecord>(this, "record_odom");
    std::thread{[this]() { perform_initialization_sequence(); }}.detach();
  }

  void perform_initialization_sequence() {
    if (!align_to_wall()) {
      RCLCPP_ERROR(get_logger(), "Initialization failed - shutting down");
      rclcpp::shutdown();
      return;
    }

    if (!start_odometry_recording()) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to start odometry recording - shutting down");
      rclcpp::shutdown();
      return;
    }

    ready_to_move_ = true;
    RCLCPP_INFO(get_logger(),
                "Initialization complete - starting wall following");
  }

  bool align_to_wall() {
    RCLCPP_INFO(get_logger(), "Starting wall alignment...");
    if (!find_wall_client_->wait_for_service(5s)) {
      RCLCPP_ERROR(get_logger(), "FindWall service not available");
      return false;
    }

    auto request = std::make_shared<wall_follower::srv::FindWall::Request>();
    auto future = find_wall_client_->async_send_request(request);

    try {
      auto response = future.get(); // Blocks, but only inside the thread
      if (!response->wallfound) {
        RCLCPP_ERROR(get_logger(), "Wall alignment failed");
        return false;
      }
      RCLCPP_INFO(get_logger(), "Wall alignment successful");
      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Service call exception: %s", e.what());
      return false;
    }

    auto response = future.get();
    if (!response->wallfound) {
      RCLCPP_ERROR(get_logger(), "Wall alignment failed");
      return false;
    }

    RCLCPP_INFO(get_logger(), "Wall alignment successful");
    return true;
  }

  bool start_odometry_recording() {
    RCLCPP_INFO(get_logger(), "Starting odometry recording...");

    if (!odom_action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "OdomRecord action server not available");
      return false;
    }

    auto goal_msg = OdomRecord::Goal();
    rclcpp_action::Client<OdomRecord>::SendGoalOptions options;

    options.goal_response_callback =
        [this](std::shared_future<GoalHandleOdomRecord::SharedPtr> future) {
          auto goal_handle = future.get();
          if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected by server");
          } else {
            RCLCPP_INFO(this->get_logger(), "Odometry recording goal accepted");
          }
        };

    options.feedback_callback =
        [this](GoalHandleOdomRecord::SharedPtr,
               const std::shared_ptr<const OdomRecord::Feedback> feedback) {
          RCLCPP_INFO(this->get_logger(), "Feedback: current distance = %.2f",
                      feedback->current_total);
        };

    options.result_callback =
        [this](const GoalHandleOdomRecord::WrappedResult &result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(),
                        "Odometry recording completed successfully");
            RCLCPP_INFO(this->get_logger(), "Recorded %zu positions",
                        result.result->list_of_odoms.size());

            // Format and print the list of odometries
            std::ostringstream oss;
            oss << "List of odometries:\n";
            for (const auto &pt : result.result->list_of_odoms) {
              oss << "  (" << pt.x << ", " << pt.y << ", " << pt.z << ")\n";
            }
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

          } else {
            RCLCPP_ERROR(this->get_logger(), "Odometry recording failed");
          }
        };

    odom_action_client_->async_send_goal(goal_msg, options);
    return true;
  }

  int angle_to_index(const sensor_msgs::msg::LaserScan &scan,
                     float angle_deg) const {
    float angle_rad = angle_deg * M_PI / 180.0f;
    int index =
        static_cast<int>((angle_rad - scan.angle_min) / scan.angle_increment);
    return (index + scan.ranges.size()) % scan.ranges.size();
  }

  float get_min_range(const std::vector<float> &ranges, int center) const {
    float min_val = std::numeric_limits<float>::infinity();
    int start = std::max(0, center - params_.smoothing_window);
    int end = std::min(static_cast<int>(ranges.size()),
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

    int front_idx = angle_to_index(*msg, 0);
    int right_idx = angle_to_index(*msg, -90);

    front_dist_ = get_min_range(msg->ranges, front_idx);
    right_dist_ = get_min_range(msg->ranges, right_idx);

    RCLCPP_DEBUG(get_logger(), "Front: %.2fm | Right: %.2fm", front_dist_,
                 right_dist_);
  }

  void control_loop() {
    if (!ready_to_move_)
      return;

    update_parameters();
    std::unique_lock<std::mutex> lock(scan_mutex_, std::try_to_lock);
    if (!lock || !latest_scan_)
      return;

    auto msg = geometry_msgs::msg::Twist();

    if (front_dist_ <= params_.safe_distance) {
      msg.linear.x = 0.0;
      msg.angular.z = params_.angular_speed;
    } else if (right_dist_ > params_.target_gap) {
      msg.linear.x = params_.linear_speed;
      msg.angular.z = -params_.angular_speed;
    } else if (right_dist_ <= params_.min_gap) {
      msg.linear.x = params_.linear_speed;
      msg.angular.z = params_.angular_speed;
    } else {
      msg.linear.x = params_.linear_speed;
      msg.angular.z = 0.0;
    }

    publisher_->publish(msg);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WallFollower>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
