#pragma once
#include <cmath>
#include <vector>
#include <numeric>
#include <limits>

namespace algorithms {

    struct LidarFilterResults {
        float front;
        float back;
        float left;
        float right;
    };

    class LidarFilter {
    public:
        LidarFilter() = default;

        LidarFilterResults apply_filter(const std::vector<float>& points, float angle_start, float angle_increment) {

            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            constexpr float angle_range = M_PI / 4.0f;

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + (i * angle_increment);

                while (angle < 0) angle += 2 * M_PI;
                while (angle >= 2 * M_PI) angle -= 2 * M_PI;

                float dist = points[i];

                // 1. Ošetření chybného měření (Not a Number)
                if (std::isnan(dist)) {
                    continue;
                }

                // 2. MOC DALEKO: Pokud je to dál než 6m nebo senzor vrátil inf, bod ignorujeme.
                // Pokud budou ignorovány všechny, průměr se spočítá jako 'inf' (cesta je čistá).
                if (std::isinf(dist) || dist > 6.0f) {
                    continue;
                }

                // 3. MOC BLÍZKO: Pokud je objekt blíž než 0.15m, nastavíme vzdálenost na 0.0f.
                // Tím zajistíme, že node nahlásí nebezpečnou blízkost a robot nenabourá.
                if (dist < 0.15f) {
                    dist = 0.0f;
                }

                // Rozřazení do sektorů
                if (angle <= angle_range / 2.0f || angle >= 2 * M_PI - angle_range / 2.0f) {
                    front.push_back(dist);
                }
                else if (angle >= M_PI / 2.0f - angle_range / 2.0f && angle <= M_PI / 2.0f + angle_range / 2.0f) {
                    left.push_back(dist);
                }
                else if (angle >= M_PI - angle_range / 2.0f && angle <= M_PI + angle_range / 2.0f) {
                    back.push_back(dist);
                }
                else if (angle >= 3 * M_PI / 2.0f - angle_range / 2.0f && angle <= 3 * M_PI / 2.0f + angle_range / 2.0f) {
                    right.push_back(dist);
                }
            }

            auto calc_mean = [](const std::vector<float>& vec) -> float {
                if (vec.empty()) {
                    return std::numeric_limits<float>::infinity();
                }
                return std::accumulate(vec.begin(), vec.end(), 0.0f) / vec.size();
            };

            // Využití tvých opravených směrů
            return LidarFilterResults{
                .front = calc_mean(back),
                .back  = calc_mean(front),
                .left  = calc_mean(right),
                .right = calc_mean(left),
            };
        }
    };

} // namespace algorithms