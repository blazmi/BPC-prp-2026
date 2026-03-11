#ifndef PRP_PROJECT_KINEMATIC_HPP
#define PRP_PROJECT_KINEMATIC_HPP

#include <cmath>

struct RobotSpeed{
    float v; // linear velocity [m/s]
    float w; // angular velocity [rad/s]
};

struct WheelSpeed{
    float l; // left wheel angular velocity [rad/s]
    float r; // right wheel angular velocity [rad/s]
};

struct Encoders{
    int l; // left ticks
    int r; // right ticks
};

struct Pose{
    float x;
    float y;
    float theta; // orientation
};

class Kinematics{
public:
    Kinematics(double wheel_radius, double wheel_base, int ticks_revolution);

    RobotSpeed forward(WheelSpeed x) const;
    WheelSpeed inverse(RobotSpeed x) const;

    RobotSpeed forward(Encoders x) const;
    RobotSpeed inverse( Encoders x) const;

    Pose updateOdometry(WheelSpeed ws, float dt);

private:
    double r_;     // wheel radius
    double L_;     // wheel base
    int ticks_;    // pulses per revolution

    Pose pose_{0,0,0}; // internal robot pose
};

#endif