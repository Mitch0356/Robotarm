#include <functional>
#include <memory>
#include <thread>

#include "action_interface/action/position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "../include/action_server_client/visibility_control.h"

namespace robot_server_client
{
class RobotActionServer : public rclcpp::Node
{
public:
  using Position = action_interface::action::Position;
  using GoalHandlePosition = rclcpp_action::ServerGoalHandle<Position>;

  ROBOT_SERVER_CLIENT_PUBLIC
  explicit RobotActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Position>(
      this,
      "position",
      std::bind(&RobotActionServer::handle_goal, this, _1, _2),
      std::bind(&RobotActionServer::handle_cancel, this, _1),
      std::bind(&RobotActionServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Position>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Position::Goal> goal)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "Received goal request with order " << goal->order);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePosition> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePosition> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&RobotActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePosition> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Position::Feedback>();
    auto & sequence = feedback->partial_sequence;
    // sequence.push_back(0);
    // sequence.push_back(1);
    std::vector <std::vector<int32_t, std::allocator<int32_t>>>  positions = {
      {0, 1500, 1, 1900, 2, 1900, 3, 1500, 4, 1500, 5, 1500, 5000},   //Ready
      {0, 1500, 1, 1500, 2, 700, 3, 1500, 4, 1500, 5, 1500, 5000},    //Straight up
      {0, 1500, 1, 2100, 2, 2100, 3, 1000, 4, 1500, 5, 1500, 5000},   //Park
      {0, 1500, 1, 1500, 2, 1500, 3, 1500, 4, 1500, 5, 1500, 5000},   //Init
      {0, 0, 1, 0, 2, 0, 3, 0, 4, 0, 5, 0, 0}                         //Stop
    };

    auto result = std::make_shared<Position::Result>();

    // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    std::vector<int32_t, std::allocator<int32_t>> previous_sequence;
    sequence = positions.at(0);
    int i = 0;
    while (true) {
      if (sequence != previous_sequence) {
        // Check if there is a cancel request
        if (goal_handle->is_canceling()) {
          result->sequence = sequence;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
        // Update sequence
        // sequence.push_back(sequence[i] + sequence[i - 1]);
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
        previous_sequence = sequence;
        i++;
        sequence = update_sequence(positions, i);
      }
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  std::vector<int32_t, std::allocator<int32_t>> update_sequence(std::vector <std::vector<int32_t, std::allocator<int32_t>>> & positions, int index) {
    //getValues
    index %= 5;
    return positions.at(index);
  }
};  // class RobotActionServer

}  // namespace robot_server_client

RCLCPP_COMPONENTS_REGISTER_NODE(robot_server_client::RobotActionServer)