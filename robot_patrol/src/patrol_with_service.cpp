#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>

using namespace std::chrono_literals;

class PatrolWithService : public rclcpp::Node {
public:
  PatrolWithService(const std::string &node_name = "patrol_with_service")
      : Node(node_name) {

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    // service client
    std::string name_service = "/direction_service";
    client_ =
        this->create_client<robot_patrol::srv::GetDirection>(name_service);

    while (!client_->wait_for_service(1s)) {

      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the Service");

        return;
      }

      RCLCPP_INFO(this->get_logger(),
                  "Service not available, waiting again...");
    }

    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        std::bind(&PatrolWithService::laserscan_callback, this,
                  std::placeholders::_1));

    publisher_cmd_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&PatrolWithService::timer_callback, this));

    // iniciamos movimiento hacia delante
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.1;
    msg.angular.z = 0.0;

    publisher_cmd_vel_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "%s Patrol with service Ready...",
                name_service.c_str());
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  std::string direction_;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    laser_scan_ = msg;
  }

  void send_request() {

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();

    request->laser_data = *laser_scan_;

    // hacemos que la peticion sea asincrona para no bloquear el hilo principal
    client_->async_send_request(request,
                                std::bind(&PatrolWithService::response_callback,
                                          this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Request Sent");
  }

  // recibimos la respuesta del server
  void response_callback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {

    auto response = future.get();

    RCLCPP_INFO(this->get_logger(), "Response Received: %s",
                response->direction.c_str());

    direction_ = response->direction;
  }

  void timer_callback() {

    if (!laser_scan_) {
      return;
    }
    auto msg = geometry_msgs::msg::Twist();

    if (direction_ == "forward") {
      msg.linear.x = 0.1;
      msg.angular.z = 0.0;
    } else if (direction_ == "left") {
      msg.linear.x = 0.1;
      msg.angular.z = 0.5;
    } else if (direction_ == "right") {
      msg.linear.x = 0.1;
      msg.angular.z = -0.5;
    } else {
      msg.linear.x = 0.1;
      msg.angular.z = 0.0;
    }

    publisher_cmd_vel_->publish(msg);

    send_request();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PatrolWithService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}