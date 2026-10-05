#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <thread>

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseInterface = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseInterface>;
  GoToPose() : Node("go_to_pose_server") {

    using namespace std::placeholders;

    // creamos la action server
    this->action_server_ = rclcpp_action::create_server<GoToPoseInterface>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Action Server Ready.");

    auto qos = rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);

    // nos subscribimos al odom
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot_1/odom", qos,
        std::bind(&GoToPose::odom_callback, this, std::placeholders::_1));

    // publicador del cmd_vel
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);
  }

private:
  rclcpp_action::Server<GoToPoseInterface>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;
  std::atomic_bool navegacion_terminada_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // cuaternión de tf2 con las cuatro componentes para obtener el yaw y
    // guardar los datos
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // almacenamos la posicion actual
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.theta = yaw;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseInterface::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with x: %.2f, y: %.2f, theta: %.2f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;

    // guardamos la posicion deseada
    desired_pos_.x = goal->goal_pos.x;
    desired_pos_.y = goal->goal_pos.y;
    desired_pos_.theta = goal->goal_pos.theta * M_PI / 180.0;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;

    navegacion_terminada_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    using namespace std::placeholders;
    // This needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {

    RCLCPP_INFO(this->get_logger(), "Executing goal");

    bool en_posicion = false;
    navegacion_terminada_ = false;

    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<GoToPoseInterface::Feedback>();

    auto result = std::make_shared<GoToPoseInterface::Result>();

    // añadimos un loop de 10Hz o 100ms
    rclcpp::Rate loop_rate(10);

    int ciclos = 0;

    auto msg = geometry_msgs::msg::Twist();

    while (!navegacion_terminada_) {

      // se pide que el feedback sea solo cada 1 segundo
      if (ciclos % 10 == 0) {
        feedback->current_pos = current_pos_;
        goal_handle->publish_feedback(feedback);
      }
      ciclos++;

      // obtenemos los datos de distancia con el objetivo y errores
      float distancia_restante = calcular_distancia_restante();
      float angulo_objetivo = obtener_angulo_objetivo();
      float error_objetivo =
          obtener_error_objetivo(angulo_objetivo, current_pos_.theta);

      if (distancia_restante <= 0.075)
        en_posicion = true;
      // una vez que tenemos el error, comprobamos si esta muy lejos y giramos
      // si es asi

      if (!en_posicion) {

        if (std::abs(error_objetivo) > 0.05) {
          // giramos
          if (error_objetivo < 0) {

            msg.linear.x = 0;
            msg.angular.z = -0.2;
          } else {

            msg.linear.x = 0;
            msg.angular.z = 0.2;
          }
        } else {
          msg.linear.x = 0.2;
          msg.angular.z = 0;
        }

      } else {
        double error_final =
            obtener_error_objetivo(desired_pos_.theta, current_pos_.theta);

        if (std::abs(error_final) <= 0.1745) { // 10 grados en radianes
          navegacion_terminada_ = true;
          break;
        }

        // giramos en el sitio hacia la orientacion final
        msg.linear.x = 0.0;
        msg.angular.z = (error_final < 0) ? -0.2 : 0.2;
      }
      cmd_vel_publisher_->publish(msg);

      // duerme lo que falte para completar los 100 ms de esta vuelta
      loop_rate.sleep();
    }

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_publisher_->publish(msg);

    if (goal_handle->is_canceling()) {
      result->status = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal Canceled");
    } else {
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal Completed");
    }
  }

  float calcular_distancia_restante() {

    float dx = desired_pos_.x - current_pos_.x;
    float dy = desired_pos_.y - current_pos_.y;

    return std::sqrt(dx * dx + dy * dy);
  }

  float obtener_angulo_objetivo() {

    float dx = desired_pos_.x - current_pos_.x;
    float dy = desired_pos_.y - current_pos_.y;

    return std::atan2(dy, dx);
  }

  float normalizar_angulo(float angulo) {
    return std::atan2(std::sin(angulo), std::cos(angulo));
  }
  float obtener_error_objetivo(float angulo_objetivo, float theta_actual) {
    return normalizar_angulo(angulo_objetivo - theta_actual);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}