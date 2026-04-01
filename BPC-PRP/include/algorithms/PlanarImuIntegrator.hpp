#pragma once
#include <vector>
#include <numeric>
#include <cmath>

namespace algorithms {

    class PlanarImuIntegrator {
    public:
        PlanarImuIntegrator() : theta_(0.0f), gyro_offset_(0.0f) {}

        void update(float gyro_z, double dt) {
            // Odečteme kalibrovaný offset a přičteme přírůstek (integrace)
            float corrected_gyro = gyro_z - gyro_offset_;
            theta_ += corrected_gyro * static_cast<float>(dt);

            // Normalizace úhlu do rozsahu -PI až PI (volitelné, ale užitečné)
            if (theta_ > M_PI) theta_ -= 2.0f * M_PI;
            if (theta_ < -M_PI) theta_ += 2.0f * M_PI;
        }

        void setCalibration(const std::vector<float>& samples) {
            if (samples.empty()) return;
            // Průměr ze vzorků určí klidový šum (drift)
            gyro_offset_ = std::accumulate(samples.begin(), samples.end(), 0.0f) / samples.size();
        }

        [[nodiscard]] float getYaw() const { return theta_; }

        void reset() {
            theta_ = 0.0f;
        }

    private:
        float theta_;       // Aktuální otočení (v radiánech)
        float gyro_offset_; // Vypočtený drift
    };
}