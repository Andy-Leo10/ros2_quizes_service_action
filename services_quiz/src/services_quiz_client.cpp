#include "rclcpp/rclcpp.hpp"
#include "services_quiz_srv/srv/spin.hpp"

#include <memory>

using namespace std::chrono_literals;
using Spin = services_quiz_srv::srv::Spin;

class ServiceClient : public rclcpp::Node
{
private:
  rclcpp::Client<Spin>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool service_done_ = false;

  void timer_callback(const std::string& direction, float angular_velocity, int time)
  {
    if (!client_->wait_for_service(1s))
    {
      RCLCPP_ERROR(this->get_logger(), "Service Unavailable. Terminating...");
      return;
    }

    auto request = std::make_shared<Spin::Request>();
    request->direction = direction;
    request->angular_velocity = angular_velocity;
    request->time = time;

    service_done_ = false;
    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_callback, this,
                           std::placeholders::_1));
    //necessary cancel
    timer_->cancel();
  }

  void response_callback(rclcpp::Client<Spin>::SharedFuture future)
  {
    RCLCPP_INFO(this->get_logger(), "Service Response Received !!!");
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready)
    {
      auto result = future.get(); // obtain the result of the service call
      if (result->success)
      {
        RCLCPP_INFO(this->get_logger(), "result->success = true !!!");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "result->success = false");
      }
      service_done_ = true;
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Service call failed to finish properly");
    }
    // only for assurance 
    timer_->cancel();
  }

public:
  ServiceClient(const std::string& service_name, const std::string& direction, float angular_velocity, int time) : Node("client_rotating")
    {
    client_ = this->create_client<Spin>(service_name);
    timer_ = this->create_wall_timer(
        1s, [this, direction, angular_velocity, time]() {
            timer_callback(direction, angular_velocity, time);
        });
    }

  bool is_service_done() const { return this->service_done_; }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto service_client1 = std::make_shared<ServiceClient>("rotate", "right", 0.2, 2);
  rcutils_logging_set_logger_level(service_client1->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_DEBUG); // set logger level to debug
  while (!service_client1->is_service_done())
  {
    RCLCPP_INFO(service_client1->get_logger(), "Service1 working...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::spin_some(service_client1);
  }

    // delete the service client
  service_client1.reset();

  auto service_client2 = std::make_shared<ServiceClient>("rotate", "left", 0.5, 3);
  rcutils_logging_set_logger_level(service_client2->get_logger().get_name(),
                                   RCUTILS_LOG_SEVERITY_DEBUG); // set logger level to debug
  while (!service_client2->is_service_done())
  {
    RCLCPP_INFO(service_client2->get_logger(), "Service2 working...");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    rclcpp::spin_some(service_client2);
  }

  rclcpp::shutdown();
  return 0;
}