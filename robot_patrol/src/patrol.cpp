#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>
#include <vector>

class Patrol : public rclcpp::Node {
public:
  Patrol(const std::string &node_name = "patrol")
      : Node(node_name), node_name_(node_name) {

    // QoS settings
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", qos,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    publisher_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);

    auto timer_period = std::chrono::milliseconds(500);
    timer_ = this->create_wall_timer(timer_period,
                                     std::bind(&Patrol::timer_callback, this));
  }

private:
  std::string node_name_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {}

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Increment: %2d", msg->angle_increment);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}