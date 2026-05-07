#pragma once
#include <cmath>
#include <vector>
#include <numeric>
#include <limits>
#include <algorithm>

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

            constexpr float angle_range = M_PI / 8.0f;

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + (i * angle_increment);
                while (angle < 0)          angle += 2.0f * M_PI;
                while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;

                float dist = points[i];

                // 1. Ignorovat NaN
                if (std::isnan(dist)) continue;

                // 2. Ignorovat inf a příliš vzdálené body (cesta je volná)
                if (std::isinf(dist) || dist > 6.0f) continue;

                // FIX: Místo nastavení na 0.0f bod zahazujeme — nulové hodnoty
                //      táhly průměr dolů a způsobovaly falešné detekce překážek.
                // 3. Příliš blízko: šumový bod zahazujeme
                if (dist < 0.08f) continue;

                // Rozřazení do sektorů
                if (angle <= angle_range / 2.0f || angle >= 2.0f * M_PI - angle_range / 2.0f) {
                    front.push_back(dist);
                }
                else if (angle >= M_PI / 2.0f - angle_range / 2.0f && angle <= M_PI / 2.0f + angle_range / 2.0f) {
                    left.push_back(dist);
                }
                else if (angle >= M_PI - angle_range / 2.0f && angle <= M_PI + angle_range / 2.0f) {
                    back.push_back(dist);
                }
                else if (angle >= 3.0f * M_PI / 2.0f - angle_range / 2.0f && angle <= 3.0f * M_PI / 2.0f + angle_range / 2.0f) {
                    right.push_back(dist);
                }
            }

            // FIX: Minimum místo průměru — zajímá nás nejbližší překážka v sektoru,
            //      ne průměrná vzdálenost. Průměr mohl skrýt blízkou překážku.
            /*auto calc_min = [](const std::vector<float>& vec) -> float {
                if (vec.empty()) return std::numeric_limits<float>::infinity();
                return *std::min_element(vec.begin(), vec.end());
            };*/
            auto calc_median = [](std::vector<float> vec) -> float {  // pozor: kopie, ne reference!
                if (vec.empty()) return std::numeric_limits<float>::infinity();
                size_t mid = vec.size() / 2;
                std::nth_element(vec.begin(), vec.begin() + mid, vec.end());
                return vec[mid];
            };
            // Pozn: směry jsou záměrně swapované — LiDAR je na robotovi fyzicky otočený.
            return LidarFilterResults{
                .front = calc_median(back),
                .back  = calc_median(front),
                .left  = calc_median(right),
                .right = calc_median(left),
            };
        }
    };

} // namespace algorithms