#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TestService : public rclcpp::Node {
public:
  TestService(const std::string &node_name = "test_service") : Node(node_name) {

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    // service client
    std::string name_service = "/direction_service";
    client_ =
        this->create_client<robot_patrol::srv::GetDirection>(name_service);

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        std::bind(&TestService::laserscan_callback, this,
                  std::placeholders::_1));

    timer_ = this->create_wall_timer(
        200ms, std::bind(&TestService::timer_callback, this));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    laser_scan_ = msg;
  }

  void send_request() {

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();

    request->laser_data = *laser_scan_;

    // hacemos que la peticion sea asincrona para no bloquear el hilo principal
    client_->async_send_request(request,
                                std::bind(&TestService::response_callback, this,
                                          std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Request Sent");
  }

  // recibimos la respuesta del server
  void response_callback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {

    auto response = future.get();

    RCLCPP_INFO(this->get_logger(), "Response Received: %s",
                response->direction.c_str());
  }

  void timer_callback() { send_request(); }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}