#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "actions_quiz_msg/action/distance.hpp"
#include "std_msgs/msg/float64.hpp"

#include "nav_msgs/msg/odometry.hpp"

class MyActionServer : public rclcpp::Node
{
public:
  using Distance = actions_quiz_msg::action::Distance;
  using GoalHandleDistance = rclcpp_action::ServerGoalHandle<Distance>;

  explicit MyActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("my_distance_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Distance>(
        this,
        "distance_as",
        std::bind(&MyActionServer::handle_goal, this, _1, _2),
        std::bind(&MyActionServer::handle_cancel, this, _1),
        std::bind(&MyActionServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float64>("total_distance", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&MyActionServer::odomCallback, this, _1));
    distance_ = 0.0;
  }

private:
  rclcpp_action::Server<Distance>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  //definition of odom subscriber & definition of tracking distance variables
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  double distance_;
  nav_msgs::msg::Odometry prev_pose_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // calculate distance traveled using Euclidean distance formula
    double dx = msg->pose.pose.position.x - prev_pose_.pose.pose.position.x;
    double dy = msg->pose.pose.position.y - prev_pose_.pose.pose.position.y;
    double dz = msg->pose.pose.position.z - prev_pose_.pose.pose.position.z;
    double distance = sqrt(dx * dx + dy * dy + dz * dz);

    // add distance traveled to total distance
    distance_ += distance;

    // update previous pose
    prev_pose_.pose = msg->pose;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const Distance::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->seconds);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleDistance> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDistance> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&MyActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleDistance> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Distance::Feedback>();
    auto &tracking_distance_FB = feedback->current_dist;
    tracking_distance_FB = distance_;
    auto result = std::make_shared<Distance::Result>();
    std_msgs::msg::Float64 distance;
    //the topic name is "total_distance"
    distance.data = tracking_distance_FB;
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < goal->seconds) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handle->is_canceling())
      {
        result->status = false;
        result->total_dist = tracking_distance_FB;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // UPDATE send feedback
      tracking_distance_FB = distance_;
      distance.data = tracking_distance_FB;
      publisher_->publish(distance);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
      result->total_dist = tracking_distance_FB;
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded \n");
    }
  }
}; // class MyActionServer

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<MyActionServer>();
  rcutils_logging_set_logger_level(action_server->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  RCLCPP_INFO(action_server->get_logger(), "ACTION = /distance_as READY!");
  RCLCPP_INFO(action_server->get_logger(), "FOR TEST USE = ros2 action send_goal -f /distance_as actions_quiz_msg/action/Distance \"{seconds: 2}\"");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
