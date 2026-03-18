//
// Created by blazmi on 15.03.26.
//
#include "algorithms/pid.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace algorithms;

bool nearly_equal(float a, float b, float eps = 1e-5f) {
    return std::fabs(a - b) < eps;
}

// Unit step input (constant error = 1.0)
void test_unit_step() {
    Pid pid(2.0f, 0.0f, 0.0f); // P-only
    float dt = 0.1f;

    float error = 1.0f;

    for (int i = 0; i < 5; ++i) {
        float output = pid.step(error, dt);
        assert(nearly_equal(output, 2.0f));
    }

    std::cout << "[PASS]\n";
}

int main() {
    test_unit_step();

    std::cout << "All P-controller tests passed.\n";
    return 0;
}

