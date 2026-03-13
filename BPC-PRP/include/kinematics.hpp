#ifndef PRP_PROJECT_KINEMATIC_HPP
#define PRP_PROJECT_KINEMATIC_HPP

#include <cmath>

namespace algorithms {

    struct RobotSpeed {
        float v; // linear
        float w; // angular
    };

    struct WheelSpeed {
        float l; // left
        float r; // right
    };

    struct Encoders {
        int l; // left
        int r; // right
    };

    struct Coordinates {
        float x;
        float y;
    };

    class Kinematics {
    public:
        Kinematics(double wheel_radius, double wheel_base, int ticks_revolution);

        RobotSpeed forward(WheelSpeed x) const;
        WheelSpeed inverse(RobotSpeed x) const;

        // Přesně podle zadání z webu
        Coordinates forward(Encoders x) const;
        Encoders inverse(Coordinates x) const;

    private:
        double r_;     // wheel radius
        double L_;     // wheel base
        int ticks_;    // pulses per revolution
    };

} // namespace algorithms
#endif