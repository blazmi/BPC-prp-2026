#ifndef ENCODER_ODOMETRY_HPP
#define ENCODER_ODOMETRY_HPP

#include <cmath>

struct EncoderTicks {
    int left;
    int right;
};

struct Pose {
    double x;
    double y;
    double theta;
};

class EncoderOdometry {
public:
    EncoderOdometry(double wheel_radius,
                    double wheel_base,
                    int ticks_per_rev);

    void update(const EncoderTicks& ticks);

    Pose getPose() const;

    void reset();

    // Korekce parametrů
    void setWheelCorrection(double left_scale, double right_scale);
    void setBaseCorrection(double base_scale);

private:
    double r_;         // wheel radius
    double L_;         // wheel base
    int ticks_;        // ticks per revolution

    double left_scale_  = 1.0;
    double right_scale_ = 1.0;
    double base_scale_  = 1.0;

    EncoderTicks last_ticks_{0,0};
    bool first_update_ = true;

    Pose pose_{0,0,0};

    double normalizeAngle(double angle);
};

#endif