#pragma once
#include <cmath>
#include <vector>
#include <numeric>
#include <limits> // Přidáno pro std::numeric_limits

namespace algorithms {

    // Structure to store filtered average distances in key directions
    struct LidarFilterResults {
        float front;
        float back;
        float left;
        float right;
    };

    class LidarFilter {
    public:
        LidarFilter() = default;

        // Vektor předáváme referencí (const &), abychom ho zbytečně nekopírovali
        LidarFilterResults apply_filter(const std::vector<float>& points, float angle_start, float angle_end) {

            // Create containers for values in different directions
            std::vector<float> left{};
            std::vector<float> right{};
            std::vector<float> front{};
            std::vector<float> back{};

            // NASTAVITELNÝ PARAMETR: Šířka každého sektoru
            // Zde používáme 45 stupňů (PI / 4). Můžeš zvětšit na PI/2 (90 stupňů),
            // pokud chceš pokrýt větší prostor.
            constexpr float angle_range = M_PI / 4.0f;

            // Compute the angular step between each range reading
            auto angle_step = (angle_end - angle_start) / points.size();

            for (size_t i = 0; i < points.size(); ++i) {
                auto angle = angle_start + i * angle_step;

                // Normalizace úhlu do rozsahu 0 až 2*PI (0 až 360 stupňů)
                while (angle < 0) angle += 2 * M_PI;
                while (angle >= 2 * M_PI) angle -= 2 * M_PI;

                float dist = points[i];

                // FILTRACE: Přeskočení neplatných čtení
                // Datasheet říká, že senzor měří spolehlivě od 0.15m do 6m.
                if (std::isinf(dist) || std::isnan(dist) || dist < 0.15f || dist > 6.0f) {
                    continue;
                }

                // TŘÍDĚNÍ: Rozřazení do směrových sektorů
                // Přední (Front): kolem úhlu 0 (nebo 2*PI)
                if (angle <= angle_range / 2.0f || angle >= 2 * M_PI - angle_range / 2.0f) {
                    front.push_back(dist);
                }
                // Levý (Left): kolem úhlu PI/2 (90 stupňů)
                else if (angle >= M_PI / 2.0f - angle_range / 2.0f && angle <= M_PI / 2.0f + angle_range / 2.0f) {
                    left.push_back(dist);
                }
                // Zadní (Back): kolem úhlu PI (180 stupňů)
                else if (angle >= M_PI - angle_range / 2.0f && angle <= M_PI + angle_range / 2.0f) {
                    back.push_back(dist);
                }
                // Pravý (Right): kolem úhlu 3*PI/2 (270 stupňů)
                else if (angle >= 3 * M_PI / 2.0f - angle_range / 2.0f && angle <= 3 * M_PI / 2.0f + angle_range / 2.0f) {
                    right.push_back(dist);
                }
            }

            // Pomocná lambda funkce pro výpočet průměru z vektoru
            auto calc_mean = [](const std::vector<float>& vec) -> float {
                if (vec.empty()) {
                    // Pokud vektor neobsahuje žádné body, vrátíme nekonečno (cesta je volná)
                    return std::numeric_limits<float>::infinity();
                }
                return std::accumulate(vec.begin(), vec.end(), 0.0f) / vec.size();
            };

            // Návrat výsledků s využitím lambda funkce
            return LidarFilterResults{
                .front = calc_mean(front),
                .back  = calc_mean(back),
                .left  = calc_mean(left),
                .right = calc_mean(right),
            };
        }
    };

} // namespace algorithms