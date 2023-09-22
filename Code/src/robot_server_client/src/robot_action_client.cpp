#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_interface/action/position.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace robot_server_client
{
class RobotActionClient : public rclcpp::Node
{
public:
  using Position = action_interface::action::Position;
  using GoalHandlePosition = rclcpp_action::ClientGoalHandle<Position>;

  explicit RobotActionClient(const rclcpp::NodeOptions & options)
  : Node("robot_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Position>(
      this,
      "position");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RobotActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Position::Goal();
    // goal_msg.order = 10;
    goal_msg.order = "1234";
    
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Position>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&RobotActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&RobotActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&RobotActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Position>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandlePosition::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandlePosition::SharedPtr,
    const std::shared_ptr<const Position::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandlePosition::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->sequence) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    // rclcpp::shutdown();
  }
};  // class RobotActionClient

}  // namespace robot_server_client

RCLCPP_COMPONENTS_REGISTER_NODE(robot_server_client::RobotActionClient)