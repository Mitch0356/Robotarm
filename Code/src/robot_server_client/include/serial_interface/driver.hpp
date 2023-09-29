#include "postures.hpp"
#include "joints.hpp"
#include "serial_port.hpp"
#include <boost/bind/bind.hpp>
#include <vector>
#include <string>


class driver
{
private:
    serial_port robot_port;
    std::vector<joint> joint_container;
    const std::string config_path = "src/robot_server_client/include/serial_interface/config.json";
public:
    driver(std::string a_port);
    ~driver();
    void initialize();
    void move_arm_pos(const long degrees, const joint a_joint, const long interval);
    void move_arm_posture(const postures a_posture, const long interval);
    void move_multiple(const std::string& command, const long& interval);
    void stop_movement();
    joint get_joint(const long& index) const;
};
