#include <gtest/gtest.h>
#include "../include/algorithms/line_estimator.hpp"

// Použijeme tvůj jmenný prostor, abychom ho nemuseli všude vypisovat
using namespace algorithms;

// --- 1. TEST DISKRÉTNÍ ESTIMACE ---
TEST(LineEstimatorTest, BasicDiscreteEstimation) {
    // A) Žádná čára: obě hodnoty jsou pod prahem 0.3
    EXPECT_EQ(LineEstimator::estimate_discrete(0.1f, 0.2f), DiscreteLinePose::LineNone);

    // B) Čára vlevo: levý senzor nad prahem, pravý pod prahem
    EXPECT_EQ(LineEstimator::estimate_discrete(0.8f, 0.1f), DiscreteLinePose::LineOnLeft);

    // C) Čára vpravo: levý pod prahem, pravý nad prahem
    EXPECT_EQ(LineEstimator::estimate_discrete(0.2f, 0.9f), DiscreteLinePose::LineOnRight);

    // D) Čára na obou senzorech: oba jsou nad prahem
    EXPECT_EQ(LineEstimator::estimate_discrete(0.7f, 0.8f), DiscreteLinePose::LineBoth);
}

// --- 2. TEST SPOJITÉ ESTIMACE ---
TEST(LineEstimatorTest, BasicContinuousEstimation) {
    // A) Robot je přesně uprostřed čáry (senzory vidí stejnou hodnotu)
    // Očekáváme 0.0 m
    EXPECT_FLOAT_EQ(LineEstimator::estimate_continuous(0.5f, 0.5f), 0.0f);

    // B) Čára je úplně vlevo (levý senzor na maximu 1.0, pravý na minimu 0.0)
    // Očekáváme kladnou odchylku: (1.0 - 0.0) * 0.015 = +0.015 m
    EXPECT_FLOAT_EQ(LineEstimator::estimate_continuous(1.0f, 0.0f), 0.015f);

    // C) Čára je úplně vpravo (levý na minimu 0.0, pravý na maximu 1.0)
    // Očekáváme zápornou odchylku: (0.0 - 1.0) * 0.015 = -0.015 m
    EXPECT_FLOAT_EQ(LineEstimator::estimate_continuous(0.0f, 1.0f), -0.015f);
}

// Hlavní funkce, která spustí všechny testy
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}//
// Created by blazmi on 13.03.26.
//