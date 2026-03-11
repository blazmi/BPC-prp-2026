//
// Created by student on 04.03.26.
//
#include "kinematic.hpp"

Kinematics::Kinematics(double wheel_radius,
                       double wheel_base,
                       int ticks_revolution)
    : r_(wheel_radius),
      L_(wheel_base),
      ticks_(ticks_revolution)
{
}

/* ============================= */
/*        FORWARD (wheel→robot)  */
/* ============================= */

RobotSpeed Kinematics::forward(WheelSpeed ws) const
{
    float v = r_ * (ws.l + ws.r) / 2.0f;
    float w = r_ * (ws.r - ws.l) / L_;

    return {v, w};
}

/* ============================= */
/*        INVERSE (robot→wheel)  */
/* ============================= */

WheelSpeed Kinematics::inverse(RobotSpeed rs) const
{
    float wl = (rs.v - (L_/2.0f) * rs.w) / r_;
    float wr = (rs.v + (L_/2.0f) * rs.w) / r_;

    return {wl, wr};
}

/* ============================= */
/*        ENCODERS → ROBOT       */
/* ============================= */

RobotSpeed Kinematics::forward(Encoders enc) const
{
    // převod ticků na úhlovou změnu
    float dtheta_l = (2.0f * M_PI * enc.l) / ticks_;
    float dtheta_r = (2.0f * M_PI * enc.r) / ticks_;

    // lineární dráha kol
    float ds_l = r_ * dtheta_l;
    float ds_r = r_ * dtheta_r;

    float v = (ds_l + ds_r) / 2.0f;
    float w = (ds_r - ds_l) / L_;

    return {v, w};
}

/* ============================= */
/*        ROBOT → ENCODERS       */
/* ============================= */

RobotSpeed Kinematics::inverse(Encoders rs) const
{
    WheelSpeed ws = this->inverse(rs);

    int ticks_l = static_cast<int>((ws.l * ticks_) / (2.0f * M_PI));
    int ticks_r = static_cast<int>((ws.r * ticks_) / (2.0f * M_PI));

    return {ticks_l, ticks_r};
}

/* ============================= */
/*        ODOMETRY (dead reck.)  */
/* ============================= */

Pose Kinematics::updateOdometry(WheelSpeed ws, float dt)
{
    RobotSpeed rs = forward(ws);

    float dx = rs.v * std::cos(pose_.theta) * dt;
    float dy = rs.v * std::sin(pose_.theta) * dt;
    float dtheta = rs.w * dt;

    pose_.x += dx;
    pose_.y += dy;
    pose_.theta += dtheta;

    return pose_;
}