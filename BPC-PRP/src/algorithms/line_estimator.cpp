#include "../include/algorithms/line_estimator.hpp"

namespace algorithms {

    float LineEstimator::estimate_continuous(float left_val, float right_val) {
        // Jednoduchý rozdíl senzorů. Vrací hodnoty v rozsahu [-1.0, 1.0]
        // Kladné číslo = čára je vlevo, Záporné číslo = čára je vpravo, 0 = uprostřed
        return (left_val - right_val)*0.015f;// je nutne doplnit vzdalenost senzoru po co oni caru uvidi
    }

    DiscreteLinePose LineEstimator::estimate_discrete(float left_val, float right_val)
    {
        const float threshold = 0.3f;

        bool left_detected = left_val > threshold;
        bool right_detected = right_val > threshold;

        if (left_detected && right_detected)
            return DiscreteLinePose::LineBoth;
        if (left_detected)
            return DiscreteLinePose::LineOnLeft;
        if (right_detected)
            return DiscreteLinePose::LineOnRight;

        return DiscreteLinePose::LineNone;
    }

} // namespace algorithms//
// Created by blazmi on 13.03.26.
//