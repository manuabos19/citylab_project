#include "robot_patrol/srv/get_direction.hpp"
#include <cmath>
#include <limits>
#include <rclcpp/rclcpp.hpp>

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("node_direction_service") {
    std::string name_service = "/direction_service";

    service_ = this->create_service<robot_patrol::srv::GetDirection>(
        name_service, std::bind(&DirectionService::get_direction_request, this,
                                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
                name_service.c_str());
  }

private:
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
  bool girando_ = false;
  std::string lado_ = "";

  // umbrales
  const float DISTANCIA_GIRO_ = 0.35;
  const float DISTANCIA_LIBRE_ = 0.50;

  void get_direction_request(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {

    RCLCPP_INFO(this->get_logger(), " Request Received...");

    std::vector<float> sector_central;
    std::vector<float> sector_izquierdo;
    std::vector<float> sector_derecho;

    // indices frontal
    int i_30 = angle_to_index(request->laser_data.angle_min,
                              request->laser_data.angle_increment, 30);
    int i_330 = angle_to_index(request->laser_data.angle_min,
                               request->laser_data.angle_increment, 330);
    // indices izquierda
    int i_90 = angle_to_index(request->laser_data.angle_min,
                              request->laser_data.angle_increment, 90);
    // indices derecha
    int i_270 = angle_to_index(request->laser_data.angle_min,
                               request->laser_data.angle_increment, 270);

    sector_central.insert(sector_central.end(),
                          request->laser_data.ranges.begin(),
                          request->laser_data.ranges.begin() + (i_30 + 1));

    sector_central.insert(sector_central.end(),
                          request->laser_data.ranges.begin() + i_330,
                          request->laser_data.ranges.end());

    sector_izquierdo.insert(sector_izquierdo.end(),
                            request->laser_data.ranges.begin() + (i_30 + 1),
                            request->laser_data.ranges.begin() + (i_90 + 1));

    sector_derecho.insert(sector_derecho.end(),
                          request->laser_data.ranges.begin() + i_270,
                          request->laser_data.ranges.begin() + i_330);

    // buscamos la distancia minima del sector frontal
    float min_distancia_frontal_obstaculos =
        std::numeric_limits<float>::infinity();

    float total_dist_sec_front = 0.0;

    for (float distancia : sector_central) {
      if (std::isnan(distancia)) {
        continue;
      } else if (std::isinf(distancia)) {
        distancia = request->laser_data.range_max;
      }

      if (distancia < min_distancia_frontal_obstaculos) {
        min_distancia_frontal_obstaculos = distancia;
      }

      total_dist_sec_front = total_dist_sec_front + distancia;
    }

    if (girando_ == true) {
      if (min_distancia_frontal_obstaculos < DISTANCIA_LIBRE_) {
        response->direction = lado_;
        return;
      }
    }

    // verificamos si la distancia del sector frontal es superior al umbral
    // minimo para evaluar el giro
    if (min_distancia_frontal_obstaculos > DISTANCIA_GIRO_) {
      RCLCPP_INFO(this->get_logger(), "Request Completed...");
      RCLCPP_INFO(this->get_logger(), "Response: forward");
      response->direction = "forward";
      lado_ = "";
      girando_ = false;
      return;
    }

    girando_ = true;

    // sumamos las distancias del sector izquierdo y derecho
    float total_dist_sec_right = 0.0;
    float total_dist_sec_left = 0.0;

    // sector izquierdo
    for (float distancia : sector_izquierdo) {
      if (std::isnan(distancia)) {
        continue;
      } else if (std::isinf(distancia)) {
        distancia = request->laser_data.range_max;
      }

      total_dist_sec_left = total_dist_sec_left + distancia;
    }

    // sector derecho
    for (float distancia : sector_derecho) {
      if (std::isnan(distancia)) {
        continue;
      } else if (std::isinf(distancia)) {
        distancia = request->laser_data.range_max;
      }

      total_dist_sec_right = total_dist_sec_right + distancia;
    }

    RCLCPP_INFO(this->get_logger(), "Request Completed...");
    if (total_dist_sec_left > total_dist_sec_right) {
      RCLCPP_INFO(this->get_logger(), "Response: left");
      response->direction = "left";
      lado_ = "left";
    } else {
      RCLCPP_INFO(this->get_logger(), "Response: right");
      response->direction = "right";
      lado_ = "right";
    }
  }

  // convertimos de angulo a indices, previamente convertimos a radianes
  int angle_to_index(float angle_min, float angle_increment, float angle) {

    float rad = angle * M_PI / 180.0f;

    return static_cast<int>(std::round((rad - angle_min) / angle_increment));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}