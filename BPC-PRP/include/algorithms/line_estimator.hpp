#ifndef PRP_PROJECT_LINE_ESTIMATOR_HPP
#define PRP_PROJECT_LINE_ESTIMATOR_HPP

#include <cstdint>

namespace algorithms {

    enum class DiscreteLinePose {
        LineOnLeft,
        LineOnRight,
        LineNone,
        LineBoth,
    };

    class LineEstimator {
    public:
        static float estimate_continuous(float left_val, float right_val);
        static DiscreteLinePose estimate_discrete(float left_val, float right_val);
    };

} // namespace algorithms
#endif