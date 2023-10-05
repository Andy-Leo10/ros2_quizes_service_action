#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"

#include <memory>

using Spin = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;

class ServerNode : public rclcpp::Node {
public:
  ServerNode() : Node("service_rotating") {

    srv_ = create_service<Spin>(
        "rotate", std::bind(&ServerNode::moving_callback, this, _1, _2));
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  rclcpp::Service<Spin>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::string direction_ = "";
  float angular_velocity_ = 0;
  int time_ = 0;
  bool success_ = false;
  void moving_callback(const std::shared_ptr<Spin::Request> request,
                       const std::shared_ptr<Spin::Response> response) {

    auto message = geometry_msgs::msg::Twist();
    direction_ = request->direction;
    angular_velocity_ = request->angular_velocity;
    time_ = request->time;

    if (direction_ == "right") {
      message.linear.x = 0.0;
      message.angular.z = -1 * angular_velocity_;
      publisher_->publish(message);
      success_ = true;
    } else if (direction_ == "left") {
      message.linear.x = 0.0;
      message.angular.z = 1 * angular_velocity_;
      publisher_->publish(message);
      success_ = true;
    } else {
      message.linear.x = 0.0;
      message.angular.z = 0;
      publisher_->publish(message);
      success_ = false;
    }
    response->success = success_;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<ServerNode>();  // Crear una instancia de ServerNode
  rcutils_logging_set_logger_level(server_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  RCLCPP_DEBUG(server_node->get_logger(), "SERVICE = /rotate");
  RCLCPP_INFO(server_node->get_logger(), "FOR TEST USE = ros2 service call /rotate services_quiz_srv/srv/Spin \"{direction: 'right', angular_velocity: 1.0, time: 5}\"");
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  return 0;
}

// ros2 service call /rotate services_quiz_srv/srv/Spin "{direction: 'right', angular_velocity: 1.0, time: 5}"
