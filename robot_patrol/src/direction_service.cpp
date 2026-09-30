#include <rclcpp/rclcpp.hpp>

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node('/node_direction_service') {
    std::string name_service = "/direction_service";

    service_ = this->create_service<robot_patrol::srv::GetDirection>(
        name_service, std::bind(&DirectionService::get_direction_request, this,
                                std::placeholders::_1, std::placeholders::_2));
  }

private:
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;

  void handle_text_recognition_request(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {}
}