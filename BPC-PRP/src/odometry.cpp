//
// Created by student on 04.03.26.
#include "odometry.hpp"

EncoderOdometry::EncoderOdometry(double wheel_radius,
                                 double wheel_base,
                                 int ticks_per_rev)
    : r_(wheel_radius),
      L_(wheel_base),
      ticks_(ticks_per_rev)
{
}

void EncoderOdometry::update(const EncoderTicks& ticks)
{
    if (first_update_) {
        last_ticks_ = ticks;
        first_update_ = false;
        return;
    }

    int dL = ticks.left  - last_ticks_.left;
    int dR = ticks.right - last_ticks_.right;

    last_ticks_ = ticks;

    // převod ticků → úhlová změna
    double dtheta_l = (2.0 * M_PI * dL) / ticks_;
    double dtheta_r = (2.0 * M_PI * dR) / ticks_;

    // aplikace korekce kol
    double ds_l = r_ * dtheta_l * left_scale_;
    double ds_r = r_ * dtheta_r * right_scale_;

    double ds = (ds_r + ds_l) / 2.0;
    double dtheta = (ds_r - ds_l) / (L_ * base_scale_);

    // midpoint integration (lepší než Euler)
    double theta_mid = pose_.theta + dtheta / 2.0;

    pose_.x += ds * std::cos(theta_mid);
    pose_.y += ds * std::sin(theta_mid);
    pose_.theta += dtheta;

    pose_.theta = normalizeAngle(pose_.theta);
}

Pose EncoderOdometry::getPose() const
{
    return pose_;
}

void EncoderOdometry::reset()
{
    pose_ = {0,0,0};
    first_update_ = true;
}

void EncoderOdometry::setWheelCorrection(double left_scale,
                                         double right_scale)
{
    left_scale_  = left_scale;
    right_scale_ = right_scale;
}

void EncoderOdometry::setBaseCorrection(double base_scale)
{
    base_scale_ = base_scale;
}

double EncoderOdometry::normalizeAngle(double angle)
{
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}