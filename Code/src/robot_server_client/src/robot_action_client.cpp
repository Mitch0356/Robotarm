#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "action_interface/action/position.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// ros2 run robot_server_client robot_action_client

namespace robot_server_client
{
  class RobotActionClient : public rclcpp::Node
  {
  public:
    using Position = action_interface::action::Position;
    using GoalHandlePosition = rclcpp_action::ClientGoalHandle<Position>;

    explicit RobotActionClient(const rclcpp::NodeOptions &options)
        : Node("robot_action_client", options)
    {

      RCLCPP_INFO(this->get_logger(), "STATE: {INITIATING}");
      this->client_ptr_ = rclcpp_action::create_client<Position>(this, "position");
      std::cout << "Robot interface, Kars en Mitchel 2023" << std::endl;
      std::cout << "\nSelect an option:\n";
      std::cout << "1. Move the arm to a fixed position (P(ark)/R(eady)/S(traight)/I(nit) followed by an optional speed in ms)(Type '1', followed by a enter, after that type the command.)\n";
      std::cout << "2. Move a specific joint to a certain degree (joint number, degrees, and interval)(Type '2',  followed by a enter, after that type the command.)\n";
      std::cout << "3. Emergency stop(Type '3')\n";
      std::cout << "4. Exit (Type '4')\n";
      handle_input();
    }

    void send_goal(const std::string &command)
    {
      using namespace std::placeholders;
      if (!this->client_ptr_->wait_for_action_server())
      {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }

      auto goal_msg = Position::Goal();
      goal_msg.order = command;
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

    void moveArmToPosition(const std::string &command)
    {
      RCLCPP_INFO(this->get_logger(), "STATE: {SENDING}");
      std::string positionCommand, speedStr;
      std::istringstream iss(command);
      long speed;
      iss >> positionCommand >> speedStr;
      size_t foundPosition = positionCommand.find_first_of("PRSIprsi");
      if (foundPosition != std::string::npos && positionCommand.length() == 1)
      {
        if (!speedStr.empty())
        {
          speed = std::atoi(speedStr.c_str());
          std::cout << "Moving to position: " << positionCommand << ", with speed " << speed << " ms." << std::endl;
        }
        else
        {
          std::cout << "Moving to position: " << positionCommand << std::endl;
        }
        this->send_goal(command);
      }
      else
      {
        std::cout << "Requested position '" << positionCommand << "' is unknown." << std::endl;
        handle_input();
      }
    }

    std::vector<std::string> parseJointToDegree(std::string command)
    {
      std::string temp;
      std::stringstream ss(command);
      std::vector<std::string> result;
      while (getline(ss, temp, ' '))
      {
        result.push_back(temp);
      }
      return result;
    }

    bool isNegative(std::string input)
    {
      return input[0] == '-';
    }

    bool check_digits(std::vector<std::string> commandNumbers)
    {
      for (std::string word : commandNumbers)
      {
        if (!isNegative(word))
        {
          for (char character : word)
          {
            if (!isdigit(character))
            {
              return false;
            }
          }
        }
        else
        {
          for (std::string::size_type character = 1; character < word.size(); ++character)
          {
            if (!isdigit(word.at(character)))
            {
              return false;
            }
          }
        }
      }
      return true;
    }

    void QoSMessage(const std::string time)
    {

      if (std::stoi(time) >= 2300)
      {
        std::stringstream ss;
        ss << "QoS-Warning: {QoS won't be reached with interval " << time << "}";
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
      }
    }

    void moveJointToDegree(const std::string &command)
    {
      RCLCPP_INFO(this->get_logger(), "STATE: {SENDING}");
      std::vector<std::string> commandNumbers = parseJointToDegree(command);
      if (!check_digits(commandNumbers) || commandNumbers.size() < 2)
      {
        std::cout << "Invalid syntax" << std::endl;
        handle_input();
      }
      else
      {
        std::cout << "Moving towards: " << std::endl;
        for (uint16_t i = 0; i < commandNumbers.size() - 1; ++i)
        {
          std::cout << commandNumbers.at(i) << " ";
        }
        std::cout << std::endl;
        if (commandNumbers.size() % 2 == 0)
        {
          std::cout << "As fast as possible" << std::endl;
        }
        else
        {
          QoSMessage(commandNumbers.back());
        }
        this->send_goal(command);
      }
    }

    void makeStop()
    {
      this->send_goal("0");
      handle_input();
    }

    void handle_input()
    {
      RCLCPP_INFO(this->get_logger(), "STATE: {IDLE}");
      std::string userInput;
      std::cin >> userInput;

      if (userInput == "1")
      {
        std::string command;
        std::cout << "Enter position command: " << std::endl;
        std::cin.ignore();
        std::getline(std::cin, command);
        moveArmToPosition(command);
        RCLCPP_INFO(this->get_logger(), "EVENT: {POSITION_CHANGE}");
      }
      else if (userInput == "2")
      {
        std::string command;
        std::cout << "Enter joint command: " << std::endl;
        std::cin.ignore();
        std::getline(std::cin, command);
        moveJointToDegree(command);
        RCLCPP_INFO(this->get_logger(), "EVENT: {SPECIFIC_JOINT_CHANGE}");
      }
      else if (userInput == "3")
      {
        std::cout << "Emergency stop initiated." << std::endl;
        RCLCPP_INFO(this->get_logger(), "EVENT: {STOP}");

        makeStop();
      }
      else if (userInput == "4")
      {
        std::cout << "Exiting program." << std::endl;
        rclcpp::shutdown();
      }
    }

  private:
    rclcpp_action::Client<Position>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(const GoalHandlePosition::SharedPtr &goal_handle)
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

    void feedback_callback(
        GoalHandlePosition::SharedPtr,
        const std::shared_ptr<const Position::Feedback> feedback)
    {
      RCLCPP_INFO(this->get_logger(), "STATE: {RECEIVING}");
      std::stringstream ss;
      ss << feedback->partial_sequence.back();
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandlePosition::WrappedResult &result)
    {
      switch (result.code)
      {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "STATE: {DONE_RECEIVING}");
        handle_input();
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
      for (auto number : result.result->sequence)
      {
        ss << number << " ";
      }
      RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

  }; // class RobotActionClient

} // namespace robot_server_client

RCLCPP_COMPONENTS_REGISTER_NODE(robot_server_client::RobotActionClient)