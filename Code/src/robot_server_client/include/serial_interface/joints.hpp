/**
 * @brief contains all joints in the form of enums.
 * 
 */
enum joint_type {
    BASE = 0,
    SHOULDER = 1,
    ELBOW = 2,
    WRIST = 3,
    GRIPPER = 4,
    WRIST_ROTATE = 5
};

class joint {
private:
/**
 * @brief maximum angle that can be reached. Originating from a configuration file
 * 
 */
const long max_angle;
/**
* @brief minimal angle that can be reached. Originating from a configuration file
* 
*/
const long min_angle;
/**
 * @brief minimal pwm that is given. Originating from a configuration file
 * 
 */
const long min_pwm;
/**
 * @brief maximum pwm that is given. Originating from a configuration file
 * 
 */
const long max_pwm;
/**
 * @brief Identifies what type of joint this instance represents. Originating from a configuration file
 * 
 */
joint_type identfier;
public:
/**
 * @brief Construct a new joint object
 * 
 * @param a_max_angle 
 * @param a_min_angle 
 * @param a_min_pwm 
 * @param a_max_pwm 
 * @param a_identfier 
 */
joint(const long a_max_angle, const long a_min_angle, const long a_min_pwm, const long a_max_pwm, const joint_type a_identfier);
/**
 * @brief Destroy the joint object
 * 
 */
~joint();
/**
 * @brief Get the max angle 
 * 
 * @return long 
 */
long get_max_angle() const;
/**
 * @brief Get the min angle 
 * 
 * @return long 
 */
long get_min_angle() const;
/**
 * @brief Get the min pwm 
 * 
 * @return long 
 */
long get_min_pwm() const;
/**
 * @brief Get the max pwm 
 * 
 * @return long 
 */
long get_max_pwm() const;
/**
 * @brief Get the identifier 
 * 
 * @return long 
 */
long get_identifier() const;
/**
 * @brief maps degrees to a usable PWM. 
 * 
 * @param incoming_degrees 
 * @return long 
 */
long map_pwm(const signed long incoming_degrees) const;
};




