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
    const long max_angle;
    const long min_angle;
    const long min_pwm;
    const long max_pwm;
    joint_type identfier;
    public:
    joint(const long a_max_angle, const long a_min_angle, const long a_min_pwm, const long a_max_pwm, const joint_type a_identfier);
    ~joint();
    long get_max_angle() const;
    long get_min_angle() const;
    long get_min_pwm() const;
    long get_max_pwm() const;
    long get_identifier() const;
    long map_pwm(const signed long incoming_degrees) const;
};




