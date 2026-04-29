//
// odometry.cpp – upravená verze
// Změny oproti originálu:
//   1. Ošetření přetečení 32-bit enkodéru (wraparound)
//   2. base_scale_ opravena: dělíme L_ * base_scale_, kde base_scale_ > 1
//      znamená „změřená základna je větší než zadaná" → méně otočení.
//   3. Přidána metoda setPose() pro relocalizaci.
//   4. dt guard při update je přesunut sem (odometrie počítá pouze z ticků,
//      takže time není potřeba – ponecháno pro kompatibilitu).
//
#include "odometry.hpp"
#include <cmath>

// ---------------------------------------------------------------------------
//  Konstanta: rozsah uint32_t enkodéru z /bpc_prp_robot/encoders
//  Pokud je enkodér uint16_t, změň na 65536.
// ---------------------------------------------------------------------------
static constexpr int32_t ENCODER_WRAP = 1L << 32; // 4 294 967 296

EncoderOdometry::EncoderOdometry(double wheel_radius,
                                 double wheel_base,
                                 int ticks_per_rev)
    : r_(wheel_radius),
      L_(wheel_base),
      ticks_(ticks_per_rev)
{
}

void EncoderOdometry::update(const EncoderTicks& ticks)
{
    if (first_update_) {
        last_ticks_ = ticks;
        first_update_ = false;
        return;
    }

    // --- Delta s ošetřením přetečení ---
    // Enkodér je uint32_t, ale roboti jezdí i pozpátku → přetečení je reálné.
    int64_t raw_dL = static_cast<int64_t>(ticks.left)  - static_cast<int64_t>(last_ticks_.left);
    int64_t raw_dR = static_cast<int64_t>(ticks.right) - static_cast<int64_t>(last_ticks_.right);

    // Pokud delta překračuje půlku rozsahu, pravděpodobně přeteklo
    if (raw_dL >  ENCODER_WRAP / 2) raw_dL -= ENCODER_WRAP;
    if (raw_dL < -ENCODER_WRAP / 2) raw_dL += ENCODER_WRAP;
    if (raw_dR >  ENCODER_WRAP / 2) raw_dR -= ENCODER_WRAP;
    if (raw_dR < -ENCODER_WRAP / 2) raw_dR += ENCODER_WRAP;

    last_ticks_ = ticks;

    int dL = static_cast<int>(raw_dL);
    int dR = static_cast<int>(raw_dR);

    // --- Převod ticků → ujetá vzdálenost [m] ---
    // ds = r * (2π * ticky / ticky_na_otáčku)
    double ds_l = r_ * (2.0 * M_PI * dL / ticks_) * left_scale_;
    double ds_r = r_ * (2.0 * M_PI * dR / ticks_) * right_scale_;

    // --- Výpočet posunu a otočení ---
    double ds     = (ds_l + ds_r) / 2.0;
    // base_scale_: korekční faktor pro efektivní šířku podvozku.
    // base_scale_ > 1 → robot se otáčí méně než model čeká (kolej je širší)
    // base_scale_ < 1 → robot se otáčí více
    double dtheta = (ds_r - ds_l) / (L_ * base_scale_);

    // --- Midpoint integrace (přesnější než Euler pro velké dt) ---
    double theta_mid = pose_.theta + dtheta / 2.0;

    pose_.x     += ds * std::cos(theta_mid);
    pose_.y     += ds * std::sin(theta_mid);
    pose_.theta += dtheta;
    pose_.theta  = normalizeAngle(pose_.theta);
}

Pose EncoderOdometry::getPose() const
{
    return pose_;
}

void EncoderOdometry::setPose(double x, double y, double theta)
{
    pose_.x     = x;
    pose_.y     = y;
    pose_.theta = normalizeAngle(theta);
}

void EncoderOdometry::reset()
{
    pose_        = {0.0, 0.0, 0.0};
    first_update_ = true;
}

void EncoderOdometry::setWheelCorrection(double left_scale, double right_scale)
{
    left_scale_  = left_scale;
    right_scale_ = right_scale;
}

void EncoderOdometry::setBaseCorrection(double base_scale)
{
    // Ochrana: nulový scale by způsobil dělení nulou
    if (std::abs(base_scale) < 1e-6) return;
    base_scale_ = base_scale;
}

double EncoderOdometry::normalizeAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}