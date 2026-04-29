#pragma once
#include <cmath>
#include <cstdint>

struct EncoderTicks {
    uint32_t left;
    uint32_t right;
};

struct Pose {
    double x;
    double y;
    double theta;
};

class EncoderOdometry {
public:
    EncoderOdometry(double wheel_radius, double wheel_base, int ticks_per_rev);

    void update(const EncoderTicks& ticks);

    Pose   getPose() const;
    void   setPose(double x, double y, double theta); // nové: relocalizace
    void   reset();

    void   setWheelCorrection(double left_scale, double right_scale);
    void   setBaseCorrection(double base_scale);

private:
    double normalizeAngle(double angle);

    double r_;
    double L_;
    int    ticks_;

    double left_scale_  = 1.0;
    double right_scale_ = 1.0;
    double base_scale_  = 1.0;

    Pose         pose_         = {0.0, 0.0, 0.0};
    EncoderTicks last_ticks_   = {0, 0};
    bool         first_update_ = true;
};