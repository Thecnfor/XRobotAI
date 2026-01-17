#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace
{
double normalize_yaw(double yaw)
{
  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
}

double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  tf2::Quaternion quat;
  tf2::fromMsg(q, quat);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  return yaw;
}

geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  q.normalize();
  return tf2::toMsg(q);
}
}  // namespace

class AutoNav final : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  AutoNav()
  : rclcpp::Node("auto_nav")
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    laser_in_topic_ = declare_parameter<std::string>("laser_in", "/laser_scan");
    laser_out_topic_ = declare_parameter<std::string>("laser_out", "/scan");
    odom_frame_override_ = declare_parameter<std::string>("odom_frame", "");
    base_frame_override_ = declare_parameter<std::string>("base_frame", "");
    publish_static_laser_tf_ = declare_parameter<bool>("publish_static_laser_tf", true);

    publish_initial_pose_ = declare_parameter<bool>("publish_initial_pose", true);
    initial_pose_source_ = declare_parameter<std::string>("initial_pose_source", "odom");
    initial_pose_x_ = declare_parameter<double>("initial_pose_x", 0.0);
    initial_pose_y_ = declare_parameter<double>("initial_pose_y", 0.0);
    initial_pose_yaw_ = declare_parameter<double>("initial_pose_yaw", 0.0);

    goal_mode_ = declare_parameter<std::string>("goal_mode", "relative");
    goal_x_ = declare_parameter<double>("goal_x", 0.0);
    goal_y_ = declare_parameter<double>("goal_y", 0.0);
    goal_yaw_ = declare_parameter<double>("goal_yaw", 0.0);
    relative_dx_ = declare_parameter<double>("relative_dx", 1.0);
    relative_dy_ = declare_parameter<double>("relative_dy", 0.0);
    relative_dyaw_ = declare_parameter<double>("relative_dyaw", 0.0);

    action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");
    action_server_timeout_sec_ = declare_parameter<double>("action_server_timeout_sec", 60.0);
    odom_timeout_sec_ = declare_parameter<double>("odom_timeout_sec", 20.0);

    initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::QoS(1).reliable());

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(laser_out_topic_, rclcpp::QoS(10));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      laser_in_topic_, rclcpp::QoS(10),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->on_scan(*msg);
        scan_pub_->publish(*msg);
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        {
          std::lock_guard<std::mutex> lock(odom_mutex_);
          last_odom_ = *msg;
          has_odom_ = true;
        }
        this->on_odom(*msg);
        odom_cv_.notify_all();
      });

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);

    worker_thread_ = std::thread([this]() {this->run();});
  }

  ~AutoNav() override
  {
    stop_requested_.store(true);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  bool wait_for_odom(const rclcpp::Duration & timeout)
  {
    std::unique_lock<std::mutex> lock(odom_mutex_);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::nanoseconds(timeout.nanoseconds());

    while (!has_odom_ && rclcpp::ok() && !stop_requested_.load()) {
      if (odom_cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
        break;
      }
    }
    return has_odom_;
  }

  geometry_msgs::msg::PoseWithCovarianceStamped make_initial_pose_from_odom()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.pose.pose = last_odom_.pose.pose;

    for (auto & v : msg.pose.covariance) {
      v = 0.0;
    }
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.06853891945200942;
    return msg;
  }

  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const std::string odom_frame =
      odom_frame_override_.empty() ? msg.header.frame_id : odom_frame_override_;
    const std::string base_frame =
      base_frame_override_.empty() ? msg.child_frame_id : base_frame_override_;

    if (odom_frame.empty() || base_frame.empty()) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg.header.stamp;
    tf.header.frame_id = odom_frame;
    tf.child_frame_id = base_frame;
    tf.transform.translation.x = msg.pose.pose.position.x;
    tf.transform.translation.y = msg.pose.pose.position.y;
    tf.transform.translation.z = msg.pose.pose.position.z;
    tf.transform.rotation = msg.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);

    {
      std::lock_guard<std::mutex> lock(frames_mutex_);
      last_base_frame_ = base_frame;
    }
    maybe_publish_static_laser_tf();
  }

  void on_scan(const sensor_msgs::msg::LaserScan & msg)
  {
    if (msg.header.frame_id.empty()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(frames_mutex_);
      last_laser_frame_ = msg.header.frame_id;
    }
    maybe_publish_static_laser_tf();
  }

  void maybe_publish_static_laser_tf()
  {
    if (!publish_static_laser_tf_ || static_laser_tf_sent_.load()) {
      return;
    }

    std::string base_frame;
    std::string laser_frame;
    {
      std::lock_guard<std::mutex> lock(frames_mutex_);
      base_frame = last_base_frame_;
      laser_frame = last_laser_frame_;
    }

    if (base_frame.empty() || laser_frame.empty() || base_frame == laser_frame) {
      return;
    }

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = base_frame;
    tf.child_frame_id = laser_frame;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = quat_from_yaw(0.0);
    static_tf_broadcaster_->sendTransform(tf);
    static_laser_tf_sent_.store(true);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped make_initial_pose_from_params()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = initial_pose_x_;
    msg.pose.pose.position.y = initial_pose_y_;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = quat_from_yaw(initial_pose_yaw_);

    for (auto & v : msg.pose.covariance) {
      v = 0.0;
    }
    msg.pose.covariance[0] = 0.25;
    msg.pose.covariance[7] = 0.25;
    msg.pose.covariance[35] = 0.06853891945200942;
    return msg;
  }

  geometry_msgs::msg::PoseStamped make_goal_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped & initial)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = map_frame_;
    goal.header.stamp = now();

    if (goal_mode_ == "absolute") {
      goal.pose.position.x = goal_x_;
      goal.pose.position.y = goal_y_;
      goal.pose.orientation = quat_from_yaw(goal_yaw_);
      return goal;
    }

    const double base_yaw = yaw_from_quat(initial.pose.pose.orientation);
    const double goal_yaw = normalize_yaw(base_yaw + relative_dyaw_);

    const double c = std::cos(base_yaw);
    const double s = std::sin(base_yaw);
    const double dx_map = relative_dx_ * c - relative_dy_ * s;
    const double dy_map = relative_dx_ * s + relative_dy_ * c;

    goal.pose.position.x = initial.pose.pose.position.x + dx_map;
    goal.pose.position.y = initial.pose.pose.position.y + dy_map;
    goal.pose.orientation = quat_from_yaw(goal_yaw);
    return goal;
  }

  void run()
  {
    const auto odom_timeout = rclcpp::Duration::from_seconds(std::max(0.1, odom_timeout_sec_));
    while (rclcpp::ok() && !stop_requested_.load()) {
      if (wait_for_odom(odom_timeout)) {
        break;
      }
      RCLCPP_WARN(
        get_logger(), "Waiting for odom on %s (timeout %.1fs)", odom_topic_.c_str(),
        odom_timeout_sec_);
    }
    if (!rclcpp::ok() || stop_requested_.load()) {
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    if (publish_initial_pose_) {
      if (initial_pose_source_ == "odom") {
        initial_pose = make_initial_pose_from_odom();
      } else {
        initial_pose = make_initial_pose_from_params();
      }
      initialpose_pub_->publish(initial_pose);
      std::this_thread::sleep_for(1s);
    } else {
      initial_pose = make_initial_pose_from_odom();
    }

    const auto server_timeout =
      std::chrono::duration<double>(std::max(0.1, action_server_timeout_sec_));
    while (rclcpp::ok() && !stop_requested_.load()) {
      if (action_client_->wait_for_action_server(server_timeout)) {
        break;
      }
      RCLCPP_WARN(get_logger(), "Waiting for action server: %s", action_name_.c_str());
    }
    if (!rclcpp::ok() || stop_requested_.load()) {
      return;
    }

    const auto goal_pose = make_goal_pose(initial_pose);

    NavigateToPose::Goal goal_msg;
    goal_msg.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
      [this](const GoalHandleNavigateToPose::WrappedResult & result) {
        last_result_code_ = result.code;
        result_cv_.notify_all();
      };

    auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(get_node_base_interface(), goal_handle_future, 10s) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Failed to send goal");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
      return;
    }

    {
      std::unique_lock<std::mutex> lock(result_mutex_);
      while (rclcpp::ok() && !stop_requested_.load()) {
        if (result_cv_.wait_for(lock, 500ms) == std::cv_status::timeout) {
          continue;
        }
        break;
      }
    }

    if (last_result_code_ == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
    } else {
      RCLCPP_WARN(
        get_logger(), "Navigation finished with code %d",
        static_cast<int>(last_result_code_));
    }
  }

  std::string map_frame_;
  std::string odom_topic_;
  std::string laser_in_topic_;
  std::string laser_out_topic_;
  std::string odom_frame_override_;
  std::string base_frame_override_;
  bool publish_static_laser_tf_{true};

  bool publish_initial_pose_{true};
  std::string initial_pose_source_;
  double initial_pose_x_{0.0};
  double initial_pose_y_{0.0};
  double initial_pose_yaw_{0.0};

  std::string goal_mode_;
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  double relative_dx_{1.0};
  double relative_dy_{0.0};
  double relative_dyaw_{0.0};

  std::string action_name_;
  double action_server_timeout_sec_{60.0};
  double odom_timeout_sec_{20.0};

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  std::mutex frames_mutex_;
  std::string last_base_frame_;
  std::string last_laser_frame_;
  std::atomic<bool> static_laser_tf_sent_{false};

  std::mutex odom_mutex_;
  std::condition_variable odom_cv_;
  nav_msgs::msg::Odometry last_odom_;
  bool has_odom_{false};

  std::mutex result_mutex_;
  std::condition_variable result_cv_;
  rclcpp_action::ResultCode last_result_code_{rclcpp_action::ResultCode::UNKNOWN};

  std::thread worker_thread_;
  std::atomic<bool> stop_requested_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoNav>());
  rclcpp::shutdown();
  return 0;
}
