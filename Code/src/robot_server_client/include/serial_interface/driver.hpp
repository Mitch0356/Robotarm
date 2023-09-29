#include "postures.hpp"
#include "joints.hpp"
#include "serial_port.hpp"
#include <boost/bind/bind.hpp>
#include <vector>
#include <string>


class driver
{
private:
/**
 * @brief contains the serial port.
 * 
 */
serial_port robot_port;
/**
 * @brief contains all the joints that are available on the robot
 * 
 */
std::vector<joint> joint_container;
/**
 * @brief contains path to the configuration file
 * 
 */
const std::string config_path = "src/robot_server_client/include/serial_interface/config.json";
public:
/**
 * @brief Construct a new driver object
 * 
 * @param a_port contains the address of the serial port
 */
driver(std::string a_port);
/**
 * @brief Destroy the driver object
 * 
 */
~driver();
/**
 * @brief initalizes all basic functions. Reading the config file, and setting the robot to Park.
 * 
 */
void initialize();
/**
 * @brief Moves arm position conform given values. Can only accept these argument, and only move 1 servo.
 * 
 * @param degrees 
 * @param a_joint 
 * @param interval 
 */
void move_arm_pos(const long degrees, const joint a_joint, const long interval);
/**
 * @brief Moves arm to position conform given values.
 * 
 * 
 * @param a_posture 
 * @param interval 
 */
void move_arm_posture(const postures a_posture, const long interval);
/**
 * @brief Moves arm to position conform given values. Can move multiple servos in the same command.
 * 
 * @param command 
 * @param interval 
 */
void move_multiple(const std::string& command, const long& interval);
/**
 * @brief stops all movement.
 * 
 */
void stop_movement();
/**
 * @brief Get the joint object at given index (or identifier)
 * 
 * @param index 
 * @return joint 
 */
joint get_joint(const long& index) const;
};
