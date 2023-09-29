#include <functional>
#include <memory>
#include <thread>
#include <iostream>
#include <ctype.h>
#include <string>
#include <sstream>
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
        : Node("robot_action_server", options), d("/dev/pts/2")
    {
      RCLCPP_INFO(this->get_logger(), "STATE: {INITIATING}");
      d.initialize();
      using namespace std::placeholders;
      this->action_server_ = rclcpp_action::create_server<Position>(
          this,
          "position",
          std::bind(&RobotActionServer::handle_goal, this, _1, _2),
          std::bind(&RobotActionServer::handle_cancel, this, _1),
          std::bind(&RobotActionServer::handle_accepted, this, _1));
      RCLCPP_INFO(this->get_logger(), "STATE: {IDLE}");
    }

  private:
    std::vector<std::thread> thread_queue;
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
      std::thread{std::bind(&RobotActionServer::execute, this, _1), goal_handle}.detach();
    }
    /**
    * @brief returns the asked time for a command if provided
    * 
    * 
    * @param command Command received from RobotActionClient as string
    */
    uint32_t getTime(std::string order)
    {
      RCLCPP_INFO(this->get_logger(), "STATE: {SENDING_COMMAND}");
      std::string temp;
      std::stringstream ss(order);
      std::vector<std::string> result;
      while (getline(ss, temp, ' '))
      {
        result.push_back(temp);
      }
      if (std::isdigit(result.at(0).at(0)))
      {
        if (result.size() % 2 == 0)
        {
          return 0;
        }
        else
        {
          return stoi(result.back());
        }
      }
      else
      {
        if (result.size() % 2 == 1)
        {
          return 0;
        }
        else
        {
          return stoi(result.back());
        }
      }
    }
    /**
    * @brief reads the given command and calls the function to move the arm from the driver in the dynamicly linked library
    * 
    *
    * @param command the command received from RobotActionClient 
    * @param interval The amount of milliseconds in which the movement needs to be completed, by default 0 unless another time is given
    */
    void moveArm(const std::string &command, const long &interval = 0)
    {
      if (command == "0")
      {
        d.stop_movement();
        return;
      }
      std::string positionCommand, speedStr;
      std::istringstream iss(command);
      iss >> positionCommand >> speedStr;
      size_t foundPosition = positionCommand.find_first_of("PRSIprsi");
      if (foundPosition != std::string::npos)
      {
        std::cout << "Moving to position: " << positionCommand << ", with speed " << interval << " ms." << std::endl;
        if (positionCommand == "P" || positionCommand == "p")
        {
          d.move_arm_posture(PARK, interval);
        }
        else if (positionCommand == "R" || positionCommand == "r")
        {
          d.move_arm_posture(READY, interval);
        }
        else if (positionCommand == "S" || positionCommand == "s")
        {
          d.move_arm_posture(STRAIGHT, interval);
        }
        else if (positionCommand == "I" || positionCommand == "i")
        {
          d.move_arm_posture(INIT, interval);
        }
      }
      else
      {
        d.move_multiple(command, interval);
      }
    }
    /**
    * @brief receives the goal for the movement goal and sends feedback and the result back
    * 
    * 
    * @param goal_handle the goal to be achieved through the messages, in this case a position the arm needs to move towards 
    */
    void execute(const std::shared_ptr<GoalHandlePosition> goal_handle)
    {
      RCLCPP_DEBUG(this->get_logger(), "EVENT: {EXECUTING_ACTION}");
      rclcpp::Rate loop_rate(1);
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<Position::Feedback>();
      auto &sequence = feedback->partial_sequence;
      auto result = std::make_shared<Position::Result>();
      uint32_t givenTime = getTime(goal->order);
      moveArm(goal->order, givenTime);
      bool goalCompleted = false;
      while (!goalCompleted)
      {
        RCLCPP_DEBUG(this->get_logger(), "EVENT: {SEND_FEEDBACK}");
        RCLCPP_INFO(this->get_logger(), "STATE: {MOVING}");
        if (goal_handle->is_canceling())
        {
          result->sequence = sequence;
          goal_handle->canceled(result);
          RCLCPP_INFO(this->get_logger(), "STATE: {IDLE}");
          return;
        }
        sequence.push_back(1);
        goalCompleted = true;
        if (rclcpp::ok())
        {
          result->sequence = sequence;
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "STATE: {IDLE}");
        }
      }
    }
  }; // class RobotActionServer

} // namespace robot_server_client

RCLCPP_COMPONENTS_REGISTER_NODE(robot_server_client::RobotActionServer)