#include <functional>
#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include "../include/serial_interface/driver.hpp"


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
    explicit RobotActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("robot_action_server", options), d("/dev/ttyUSB0")
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
    driver d;
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
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
      auto &sequence = feedback->partial_sequence;
      auto result = std::make_shared<Position::Result>();
      std::vector<uint16_t> totalOrder = {1};
      int i = 0;
      bool goalCompleted = false;
      while (!goalCompleted)
      {
        i++;
        if (goal_handle->is_canceling())
        {
          result->sequence = sequence;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          return;
        }
        sequence.push_back(1);
        goal_handle->publish_feedback(feedback);
        std::stringstream ss;
        ss << feedback->partial_sequence.back();
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        if (i == 1)
        {
          d.move_arm_posture(READY, 1000);
          goalCompleted = true;
        }
      }

      // Check if goal is done
      if (rclcpp::ok())
      {
        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
      }
    }
  }; // class RobotActionServer

} // namespace robot_server_client

RCLCPP_COMPONENTS_REGISTER_NODE(robot_server_client::RobotActionServer)