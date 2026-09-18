#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

class Patrol : public rclcpp::Node {
public:
  Patrol(const std::string &node_name = "patrol")
      : Node(node_name), node_name_(node_name) {

    // QoS settings
    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    subscriber_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", qos,
        std::bind(&Patrol::laserscan_callback, this, std::placeholders::_1));

    publisher_cmd_vel_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto timer_period = std::chrono::milliseconds(100);
    timer_ = this->create_wall_timer(timer_period,
                                     std::bind(&Patrol::timer_callback, this));

    // iniciamos movimiento hacia delante
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = 0.1;
    msg.angular.z = 0.0;

    publisher_cmd_vel_->publish(msg);
  }

private:
  std::string node_name_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
  rclcpp::TimerBase::SharedPtr timer_;

  // inicializamos todo lo necesario
  bool hayObstaculos_ = false;
  bool girando_ = false;
  float lado_ = 0.0;

  // umbrales
  const float DISTANCIA_GIRO_ = 0.35;
  const float DISTANCIA_LIBRE_ = 0.60;

  void timer_callback() {
    auto cmd_vel = geometry_msgs::msg::Twist();

    if (girando_) {
      cmd_vel.angular.z = lado_;
      cmd_vel.linear.x = 0.05;
      RCLCPP_WARN(this->get_logger(), "Girando %s",
                  (lado_ > 0) ? "izquierda" : "derecha");
    } else {
      cmd_vel.angular.z = 0.0;
      cmd_vel.linear.x = 0.1;
      RCLCPP_INFO(this->get_logger(), "Sigue de frente");
    }

    publisher_cmd_vel_->publish(cmd_vel);
  }

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    hayObstaculos_ = false;

    // obtenemos los indice del angulo 90  y 270. Tambien obtendremos los
    // indices de 20 y 340 para la deteccion de obstaculos
    int i_90 = angle_to_index(msg->angle_min, msg->angle_increment, 90);
    int i_270 = angle_to_index(msg->angle_min, msg->angle_increment, 270);
    int i_20 = angle_to_index(msg->angle_min, msg->angle_increment, 20);
    int i_340 = angle_to_index(msg->angle_min, msg->angle_increment, 340);

    std::vector<float> sector_180;
    std::vector<float> sector_obstaculos;

    // añadimos a los vectores las lecturas de los laseres. 20º y 180º
    sector_180.insert(sector_180.end(), msg->ranges.begin(),
                      msg->ranges.begin() + (i_90 + 1));

    sector_180.insert(sector_180.end(), msg->ranges.begin() + i_270,
                      msg->ranges.end());

    sector_obstaculos.insert(sector_obstaculos.end(), msg->ranges.begin(),
                             msg->ranges.begin() + (i_20 + 1));

    sector_obstaculos.insert(sector_obstaculos.end(),
                             msg->ranges.begin() + i_340, msg->ranges.end());

    float min_distancia_obstaculos = std::numeric_limits<float>::infinity();

    // distancia minima del sector de obstaculos (±20º)
    // aqui si ignoramos inf: inf = no hay obstaculo
    for (float distancia : sector_obstaculos) {
      if (std::isnan(distancia) || std::isinf(distancia))
        continue;

      if (distancia < min_distancia_obstaculos) {
        min_distancia_obstaculos = distancia;
      }
    }

    // maximo de los 180º para ver donde girar
    float max_distancia_obstaculos = 0.0;
    int indice_laser_maxima_distancia = 0;
    for (size_t i = 0; i < sector_180.size(); ++i) {
      if (std::isnan(sector_180[i]) || std::isinf(sector_180[i]))
        continue;

      if (sector_180[i] > max_distancia_obstaculos) {
        max_distancia_obstaculos = sector_180[i];
        indice_laser_maxima_distancia = i;
      }
    }

    int mitad_indice = sector_180.size() / 2;

    // el lado solo se decide mientras NO giramos; al girar queda bloqueado
    if (girando_ == false) {
      if (indice_laser_maxima_distancia > mitad_indice) {
        lado_ = -0.5;
      } else {
        lado_ = 0.5;
      }
    }

    if (min_distancia_obstaculos <= DISTANCIA_GIRO_) {
      if (!girando_) {
        RCLCPP_ERROR(this->get_logger(), "ALERTA: CERCA DE OBSTACULO O PARED");
      }
      hayObstaculos_ = true;
      girando_ = true;
    }

    if (min_distancia_obstaculos > DISTANCIA_LIBRE_) {
      girando_ = false;
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
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}