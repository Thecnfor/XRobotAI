#include <algorithm>
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CmdVelDriverNode final : public rclcpp::Node
{
public:
  CmdVelDriverNode()
  : rclcpp::Node("cmd_vel_driver")
  {
    topic_ = declare_parameter<std::string>("topic", "/cmd_vel");
    linear_x_ = declare_parameter<double>("linear_x", 20);
    angular_z_ = declare_parameter<double>("angular_z", 0.2);
    rate_hz_ = declare_parameter<double>("rate_hz", 10.0);

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(topic_, rclcpp::QoS(10));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / std::max(0.1, rate_hz_)));
    timer_ = create_wall_timer(period, std::bind(&CmdVelDriverNode::on_timer, this));
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = linear_x_;
    msg.angular.z = angular_z_;
    publisher_->publish(msg);
  }

  std::string topic_;
  double linear_x_{0.0};
  double angular_z_{0.0};
  double rate_hz_{10.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelDriverNode>());
  rclcpp::shutdown();
  return 0;
}
