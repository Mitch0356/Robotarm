#include "driver.hpp"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <filesystem>

driver::driver(std::string a_port): robot_port(a_port)
{
    std::cout << __PRETTY_FUNCTION__ << ": " << config_path << std::endl;
    initialize();
}

driver::~driver()
{
}

void driver::move_arm_pos(const long degrees, const joint a_joint, const long interval) {
 
if(degrees >= a_joint.get_min_angle() && degrees <= a_joint.get_max_angle())
{
    long target_pwm = a_joint.map_pwm(-degrees);
    std::cout << a_joint.get_max_angle() << " " << a_joint.get_min_angle() << " " << a_joint.get_min_pwm() << " " << a_joint.get_max_pwm() << std::endl;
    std::cout << "Target: " << target_pwm << std::endl;
    std::stringstream message;
    message << "#" << a_joint.get_identifier() << "P" << target_pwm;
    if(interval > 0)
    {
        message << "T" << interval;
    }
    message << "\r\n";
    std::cout << message.str() << std::endl;
    robot_port.easy_write_to_port(message.str());
} else {
    std::cerr << "This action is not allowed." << std::endl;
}
}

void driver::move_arm_posture(const postures a_posture, const long interval = 0)
{
    std::stringstream message;
    switch (a_posture)
    {
    case postures::PARK:
        message << "#0 P1500 #1 P2100 #2 P2100 #3 P1000 #4 P1500 #5 P1500";
        break;
    case postures::READY:
        message << "#0 P1500 #1 P1900 #2 P1900 #3 P1500 #4 P1500 #5 P1500";
        break;
    case postures::STRAIGHT:
        message << "#0 P1500 #1 P1500 #2 P700 #3 P1500 #4 P1500 #5 P1500";
        break;
    case postures::INIT:
        message << "#0 P1500 #1 P1500 #2 P1500 #3 P1500 #4 P1500 #5 P1500";
    default:
        break;
    }
    if(interval > 0)
    {
        message << " T" << interval;
    }
    message << "\r\n";
    robot_port.easy_write_to_port(message.str());
}


void driver::initialize() 
{
try {
    namespace fs = std::filesystem;
        boost::property_tree::ptree pt;
        std::cout << "Current path is " << fs::current_path() << '\n'; // (1)        
           boost::property_tree::read_json(config_path, pt);
        const boost::property_tree::ptree& joint_tree = pt.get_child("joints");
        for (const auto& item : joint_tree) {
            joint_type identifier =  static_cast<joint_type>(item.second.get<int>("identifier"));
            int min_angle = item.second.get<int>("min_angle");
            int max_angle = item.second.get<int>("max_angle");
            int min_pwm = item.second.get<int>("min_pwm");
            int max_pwm = item.second.get<int>("max_pwm");
            joint_container.push_back(joint(max_angle, min_angle, min_pwm, max_pwm, identifier));
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    move_arm_posture(PARK);
}

void driver::stop_movement()
{
    robot_port.easy_write_to_port("#0P0\r\n");
}

joint driver::get_joint(const long& index) const
{
    return joint_container.at(index);
}

