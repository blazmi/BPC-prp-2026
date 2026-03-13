#include "../include/kinematics.hpp"
#include <cmath>

namespace algorithms {

    Kinematics::Kinematics(double wheel_radius,
                           double wheel_base,
                           int ticks_revolution)
        : r_(wheel_radius),
          L_(wheel_base),
          ticks_(ticks_revolution)
    {
    }

    RobotSpeed Kinematics::forward(WheelSpeed ws) const
    {
        float v = r_ * (ws.l + ws.r) / 2.0f;
        float w = r_ * (ws.r - ws.l) / L_;
        return {v, w};
    }

    WheelSpeed Kinematics::inverse(RobotSpeed rs) const
    {
        float wl = (rs.v - (L_/2.0f) * rs.w) / r_;
        float wr = (rs.v + (L_/2.0f) * rs.w) / r_;
        return {wl, wr};
    }

    Coordinates Kinematics::forward(Encoders enc) const
    {
        // Převod tiků na úhel natočení kola v radiánech
        float dtheta_l = (2.0f * M_PI * enc.l) / (float)ticks_;
        float dtheta_r = (2.0f * M_PI * enc.r) / (float)ticks_;

        // Převod na ujetou vzdálenost levého a pravého kola
        float ds_l = r_ * dtheta_l;
        float ds_r = r_ * dtheta_r;

        // Vypočítáme lineární posun (dx) a natočení (dtheta) robota
        float dx = (ds_l + ds_r) / 2.0f;
        float dtheta = (ds_r - ds_l) / L_;

        // Vracíme to jako Coordinates (x = posun, y = rotace)
        return {dx, dtheta};
    }

    Encoders Kinematics::inverse(Coordinates c) const
    {
        // c.x je lineární posun, c.y je natočení
        float ds_l = c.x - (c.y * L_) / 2.0f;
        float ds_r = c.x + (c.y * L_) / 2.0f;

        int ticks_l = std::round((ds_l / (2.0f * M_PI * r_)) * ticks_);
        int ticks_r = std::round((ds_r / (2.0f * M_PI * r_)) * ticks_);

        return {ticks_l, ticks_r};
    }

} // namespace algorithms