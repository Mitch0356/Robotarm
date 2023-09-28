#include <functional>
#include <memory>
#include <thread>
#include <chrono>

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
  std::vector<std::thread> thread_queue;
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
    if (!thread_queue.empty()) {
      thread_queue.front().join();
    }
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // std::thread{std::bind(&RobotActionServer::execute, this, _1), goal_handle}.detach();
    thread_queue.push_back(std::thread{std::bind(&RobotActionServer::execute, this, _1), goal_handle});
  }

 std::vector<uint16_t> parse(std::string order) {
  std::string delimiter = " ";
  std::vector<uint16_t> total;

  size_t pos = 0;
  std::string token;
  while ((pos = order.find(delimiter)) != std::string::npos) {
      token = order.substr(0, pos);
      total.push_back(stoi(token));
      order.erase(0, pos + delimiter.length());
  }
  total.push_back(stoi(order));

  return total;
 }

  void execute(const std::shared_ptr<GoalHandlePosition> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");

    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Position::Feedback>();
    auto & sequence = feedback->partial_sequence;
    
    auto result = std::make_shared<Position::Result>();

    std::vector<uint16_t> totalOrder = parse(goal->order);
    const auto startTime{std::chrono::steady_clock::now()};
    uint32_t maxTime = 2300;
    uint32_t givenTime = 0;
    std::cout << "Moving towards: " << std::endl;
    for (uint16_t i = 0; i < totalOrder.size() - 1; ++i) {
      std::cout << totalOrder.at(i) << " ";
    }
    if (totalOrder.size() % 2 == 1) {
      givenTime = totalOrder.at(totalOrder.size() - 1);
    }
    std::cout << std::endl;
    if (totalOrder.size() % 2 == 0) {
      std::cout << "As fast as possible" << std:: endl;
    } else {
      std::cout << "In " << givenTime << " milliseconds" << std::endl;
    }

    // for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {

    bool goalCompleted = false;
    while (!goalCompleted) {
      const auto currentTime{std::chrono::steady_clock::now()};
      auto timePassed = (std::chrono::duration_cast<std::chrono::microseconds>(currentTime - startTime).count()) /1000.0;
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update sequence for feedback
      if (timePassed > maxTime) {
        sequence.push_back(0);
        goalCompleted = true;
      } else if (timePassed > givenTime) {
        sequence.push_back(1);
        goalCompleted = true;
      } else {
        sequence.push_back(1);
      }
    
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");
      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class RobotActionServer

}  // namespace robot_server_client

RCLCPP_COMPONENTS_REGISTER_NODE(robot_server_client::RobotActionServer)