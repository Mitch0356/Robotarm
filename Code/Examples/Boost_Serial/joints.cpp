#include "joints.hpp"

joint::joint(const long a_max_angle, const long a_min_angle, const long a_min_pwm, const long a_max_pwm, const joint_type a_identfier): max_angle(a_max_angle), min_angle(a_min_angle), min_pwm(a_min_pwm), max_pwm(a_max_pwm), identfier(a_identfier) {}

joint::~joint(){}

long joint::map_pwm(const signed long incoming_degrees) const
{
    return (incoming_degrees - min_angle) * (max_pwm - min_pwm) / (max_angle - min_angle) + min_pwm;
}

long joint::get_max_angle() const
{
    return max_angle;
}

long joint::get_min_angle() const
{
    return min_angle;
}
long joint::get_min_pwm() const
{
    return min_pwm;
}
long joint::get_max_pwm() const
{
    return max_pwm;
}
long joint::get_identifier() const
{
    return static_cast<joint_type>(identfier);
}