//
// corridor_node.cpp
//
// Stavový diagram:
//
//   CALIBRATING ──► (imu ok + enable) ──────────────────────────► CORRIDOR
//
//   CORRIDOR ──── obě zdi zmizí ──────────────────────────────── ► OPEN
//            └─── přední zeď ──► decide_turn() ──► TURN_* ──────► LEAVING
//
//   OPEN ─────── přední zeď ──► decide_turn() ──► TURN_* ───────► LEAVING
//         └───── zeď se vrátí (po settle_dist) ────────────────── ► LEAVING
//                                                                   (krátký
//   LEAVING ──── ujeto settle_dist ─────────────────────────────►  CORRIDOR)
//
//   CORRIDOR/OPEN ◄──────────────────────────────────────────────┘
//
// ArUco:
//   on_aruco() ukládá příkaz 0/1/2 kdykoliv přijde ze CameraNode.
//   decide_turn() ho spotřebuje a resetuje na default.
//   Pokud žádná značka nebyla, použije se p_.default_aruco (výchozí: vpravo).
//
// Ochranná vzdálenost (settle_dist = 15 cm):
//   Platí na DVOU místech:
//   1. OPEN: robot nesmí přepnout zpět do CORRIDOR dříve než ujede settle_dist
//      (chrání před okamžitým návratem po vstupu do křižovatky)
//   2. LEAVING: po každém otočení robot jede rovně (IMU kurz) settle_dist,
//      pak teprve přejde do CORRIDOR – chrání před "viděním" staré křižovatky
//
// PWM konvence:
//   255 = plný vpřed
//   127 = stop  ← střed
//   0   = plný dozadu
//

#include "loops/hokus_pokus.hpp"
#include <cmath>
#include <algorithm>

namespace nodes {

static constexpr float INF  = std::numeric_limits<float>::infinity();
static constexpr int   PWM_CENTER = 127;   // stop = 127, ne 128!

// ============================================================================
//  Konstruktor
// ============================================================================
CorridorNode::CorridorNode(const CorridorParams& p)
    : rclcpp::Node("corridor_node"),
      p_(p),
      pid_wall_(p.pid_kp, p.pid_ki, p.pid_kd),
      pid_hdg_ (p.hdg_kp, p.hdg_ki, p.hdg_kd),
      odom_(p.wheel_radius, p.wheel_base, p.ticks_per_rev)
{
    lidar_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "filtered_distances", 10,
        std::bind(&CorridorNode::on_lidar, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/bpc_prp_robot/imu", rclcpp::SensorDataQoS(),
        std::bind(&CorridorNode::on_imu, this, std::placeholders::_1));

    enc_sub_ = create_subscription<std_msgs::msg::UInt32MultiArray>(
        "/bpc_prp_robot/encoders", 10,
        std::bind(&CorridorNode::on_encoders, this, std::placeholders::_1));

    en_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/robot/enable", 10,
        std::bind(&CorridorNode::on_enable, this, std::placeholders::_1));

    aruco_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
        "/camera/command", 10,
        [this](const std_msgs::msg::UInt8::SharedPtr msg) { this->on_aruco(msg); });

    motor_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/corridor_loop/motor_cmds", rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&CorridorNode::on_timer, this));

    last_t_     = now();
    last_imu_t_ = now();

    RCLCPP_INFO(get_logger(), "CorridorNode spuštěn – kalibruji IMU...");
}

// ============================================================================
//  Callbacky
// ============================================================================
void CorridorNode::on_lidar(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 4) return;
    lf_ = msg->data[0];
    ll_ = msg->data[2];
    lr_ = msg->data[3];
    lidar_ok_ = true;
}

void CorridorNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const float gz = static_cast<float>(msg->angular_velocity.z);
    rclcpp::Time t = now();
    double dt = (t - last_imu_t_).seconds();
    last_imu_t_ = t;
    if (dt <= 0.0 || dt > 0.5) dt = 0.01;

    if (!imu_ok_) {
        calib_.push_back(gz);
        if ((int)calib_.size() >= p_.imu_calib_n) {
            imu_.setCalibration(calib_);
            imu_ok_ = true;
            RCLCPP_INFO(get_logger(), "IMU zkalibrováno. Čekám na enable...");
        }
        return;
    }
    imu_.update(gz, dt);
}

void CorridorNode::on_encoders(const std_msgs::msg::UInt32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2) return;
    odom_.update({ msg->data[0], msg->data[1] });
}

void CorridorNode::on_enable(const std_msgs::msg::Bool::SharedPtr msg)
{
    enabled_ = msg->data;
    if (!enabled_) {
        stop_motors();
        state_ = RobotState::STOPPED;
        pid_wall_.reset();
        pid_hdg_.reset();
        RCLCPP_INFO(get_logger(), "STOP.");
    } else {
        if (imu_ok_) {
            state_ = RobotState::CORRIDOR;
            RCLCPP_INFO(get_logger(), "START → CORRIDOR.");
        } else {
            RCLCPP_WARN(get_logger(), "IMU ještě není zkalibrováno!");
        }
    }
}

void CorridorNode::on_aruco(const std_msgs::msg::UInt8::SharedPtr msg)
{
    uint8_t id = msg->data;
    if (id <= 2) {
        aruco_cmd_   = id;
        aruco_fresh_ = true;
        const char* dir[] = {"ROVNE", "VLEVO", "VPRAVO"};
        RCLCPP_INFO(get_logger(), "ArUco příkaz: %s (id=%d)", dir[id], id);
    }
    // ID 10, 11, 12 ignorujeme
}

// ============================================================================
//  Výpočet scény
// ============================================================================
Scene CorridorNode::compute_scene() const
{
    return {
        .wall_front = std::isfinite(lf_) && lf_ <= p_.wall_stop_dist,
        .wall_left  = std::isfinite(ll_) && ll_ <  p_.junction_thr,
        .wall_right = std::isfinite(lr_) && lr_ <  p_.junction_thr,
    };
}

// ============================================================================
//  Hlavní smyčka 50 Hz
// ============================================================================
void CorridorNode::on_timer()
{
    if (!enabled_ || !imu_ok_ || !lidar_ok_) return;

    rclcpp::Time t = now();
    float dt = static_cast<float>((t - last_t_).seconds());
    last_t_ = t;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;

    Scene s = compute_scene();

    switch (state_) {
        case RobotState::CORRIDOR:  process_corridor(dt, s); break;
        case RobotState::OPEN:      process_open    (dt, s); break;
        case RobotState::TURN_LEFT:
        case RobotState::TURN_RIGHT:
        case RobotState::TURN_180:  process_turn();          break;
        case RobotState::LEAVING:   process_leaving (dt);    break;
        case RobotState::STOPPED:
        case RobotState::CALIBRATING: stop_motors();         break;
    }
}

// ============================================================================
//  CORRIDOR
// ============================================================================
void CorridorNode::process_corridor(float dt, const Scene& s)
{
    if (s.wall_front) {
        stop_motors();
        decide_turn(s);
        return;
    }

    // Obě boční zdi zmizely → přejdi do OPEN
    if (!s.wall_left && !s.wall_right) {
        enter_open();
        return;
    }

    // PID centrování
    bool hL = std::isfinite(ll_) && ll_ > p_.sensor_min;
    bool hR = std::isfinite(lr_) && lr_ > p_.sensor_min;
    float error = 0.0f;

    if      (hL && hR) error =  ll_ - lr_;
    else if (hL)       error =  (ll_ - p_.wall_follow_dist);
    else if (hR)       error = -(lr_ - p_.wall_follow_dist);

    float corr = std::clamp(pid_wall_.step(error, dt), -80.0f, 80.0f);
    auto [vl, vr] = blend(p_.base_speed, corr);
    set_motors(vl, vr);
}

// ============================================================================
//  OPEN – chybí obě boční zdi
// ============================================================================
void CorridorNode::process_open(float dt, const Scene& s)
{
    float dist = odometry_dist_from(open_start_);

    // Přední zeď — bezpečnost nade vše
    if (s.wall_front) {
        stop_motors();
        decide_turn(s);
        return;
    }

    // Fáze 1: settle – první settle_dist jedeme slepě rovně
    if (dist < p_.settle_dist) {
        drive_heading(dt, open_ref_yaw_);
        return;
    }

    // Fáze 2: vrátila se zeď → LEAVING
    if (s.wall_left || s.wall_right) {
        pid_hdg_.reset();
        pid_wall_.reset();
        enter_leaving();
        RCLCPP_INFO(get_logger(), "Zeď zpět po %.3f m → LEAVING", dist);
        return;
    }

    // Stále otevřeno: IMU drží kurz
    drive_heading(dt, open_ref_yaw_);
    RCLCPP_DEBUG(get_logger(), "OPEN: dist=%.3f yaw=%.3f", dist, imu_.getYaw());
}

// ============================================================================
//  TURN – točení na místě
// ============================================================================
void CorridorNode::process_turn()
{
    float diff = turn_target_yaw_ - imu_.getYaw();
    while (diff >  M_PI) diff -= 2.0f * M_PI;
    while (diff < -M_PI) diff += 2.0f * M_PI;

    float tol = p_.turn_tol_deg * static_cast<float>(M_PI) / 180.0f;

    if (std::abs(diff) <= tol) {
        stop_motors();
        pid_wall_.reset();
        pid_hdg_.reset();
        enter_leaving();
        RCLCPP_INFO(get_logger(), "Otočení hotovo → LEAVING");
        return;
    }

    // PWM střed = 127, spd = offset
    // diff > 0 → točit vlevo: levé kolo dozadu, pravé vpřed
    // diff < 0 → točit vpravo: levé kolo vpřed, pravé dozadu
    int spd = static_cast<int>(p_.turn_speed);
    if (diff > 0) {
        set_motors(
            static_cast<uint8_t>(std::clamp(PWM_CENTER - spd, 0, 255)),
            static_cast<uint8_t>(std::clamp(PWM_CENTER + spd, 0, 255))
        );
    } else {
        set_motors(
            static_cast<uint8_t>(std::clamp(PWM_CENTER + spd, 0, 255)),
            static_cast<uint8_t>(std::clamp(PWM_CENTER - spd, 0, 255))
        );
    }
}

// ============================================================================
//  LEAVING – ochranná vzdálenost po otočení
// ============================================================================
void CorridorNode::process_leaving(float dt)
{
    float dist = odometry_dist_from(leaving_start_);

    if (dist >= p_.settle_dist) {
        pid_hdg_.reset();
        state_ = RobotState::CORRIDOR;
        RCLCPP_INFO(get_logger(), "LEAVING hotovo (%.3f m) → CORRIDOR", dist);
        return;
    }

    drive_heading(dt, leaving_ref_yaw_);
}

// ============================================================================
//  Přechody
// ============================================================================
void CorridorNode::enter_open()
{
    open_ref_yaw_ = imu_.getYaw();
    open_start_   = odom_.getPose();
    pid_hdg_.reset();
    state_ = RobotState::OPEN;
    RCLCPP_INFO(get_logger(), "→ OPEN (ref_yaw=%.3f)", open_ref_yaw_);
}

void CorridorNode::enter_turn(float delta_rad, RobotState ts)
{
    float target = imu_.getYaw() + delta_rad;
    while (target >  M_PI) target += M_PI/2;
    while (target < -M_PI) target -= M_PI/2;
    turn_target_yaw_ = target;
    state_ = ts;
}

void CorridorNode::enter_leaving()
{
    leaving_ref_yaw_ = imu_.getYaw();
    leaving_start_   = odom_.getPose();
    state_ = RobotState::LEAVING;
}

// ============================================================================
//  decide_turn – rozhodování o směru
// ============================================================================
void CorridorNode::decide_turn(const Scene& s)
{
    float rad = p_.turn_target_deg * static_cast<float>(M_PI) / 180.0f;

    bool can_left  = !s.wall_left;
    bool can_right = !s.wall_right;

    // Slepá ulička
    if (!can_left && !can_right) {
        aruco_fresh_ = false;
        aruco_cmd_   = p_.default_aruco;
        enter_turn(static_cast<float>(M_PI), RobotState::TURN_180);
        RCLCPP_INFO(get_logger(), "Slepá ulička → TURN_180");
        return;
    }

    // Přečtení příkazu
    uint8_t cmd;
    if (aruco_fresh_) {
        cmd          = aruco_cmd_;
        aruco_fresh_ = false;
        aruco_cmd_   = p_.default_aruco;
        RCLCPP_INFO(get_logger(), "Použit ArUco příkaz: %d", cmd);
    } else {
        cmd = p_.default_aruco;
        RCLCPP_INFO(get_logger(), "Žádná ArUco značka → výchozí: %d", cmd);
    }

    // Provedení příkazu
    if (cmd == 0) {
        if (!s.wall_front) {
            enter_leaving();
            RCLCPP_INFO(get_logger(), "ArUco: ROVNE");
        } else {
            cmd = p_.default_aruco;
            RCLCPP_WARN(get_logger(), "ArUco: ROVNE ale zeď vpředu → fallback");
            goto apply_lr;
        }
    } else {
        apply_lr:
        if (cmd == 1 && can_left) {
            enter_turn(-rad, RobotState::TURN_LEFT);
            RCLCPP_INFO(get_logger(), "→ TURN_LEFT");
        } else if (cmd == 2 && can_right) {
            enter_turn(+rad, RobotState::TURN_RIGHT);
            RCLCPP_INFO(get_logger(), "→ TURN_RIGHT");
        } else if (can_left) {
            enter_turn(-rad, RobotState::TURN_LEFT);
            RCLCPP_WARN(get_logger(), "Požadovaná strana není volná → TURN_LEFT");
        } else {
            enter_turn(+rad, RobotState::TURN_RIGHT);
            RCLCPP_WARN(get_logger(), "Požadovaná strana není volná → TURN_RIGHT");
        }
    }
}

// ============================================================================
//  Pomocná: jede rovně, IMU drží zadaný yaw
// ============================================================================
void CorridorNode::drive_heading(float dt, float ref_yaw)
{
    float yaw_err = ref_yaw - imu_.getYaw();
    while (yaw_err >  M_PI) yaw_err -= 2.0f * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0f * M_PI;

    float corr = std::clamp(pid_hdg_.step(yaw_err, dt), -60.0f, 60.0f);
    auto [vl, vr] = blend(p_.base_speed, corr);
    set_motors(vl, vr);
}

// ============================================================================
//  Pomocné
// ============================================================================
float CorridorNode::odometry_dist_from(const Pose& ref) const
{
    Pose cur = odom_.getPose();
    float dx = static_cast<float>(cur.x - ref.x);
    float dy = static_cast<float>(cur.y - ref.y);
    return std::sqrt(dx*dx + dy*dy);
}

// blend: PWM střed = 127, base_speed = offset vpřed, corr = korekce
// výsledek: L = 127 + base - corr, R = 127 + base + corr
std::pair<uint8_t, uint8_t> CorridorNode::blend(uint8_t base, float corr)
{
    int l = PWM_CENTER + static_cast<int>(base) - static_cast<int>(corr);
    int r = PWM_CENTER + static_cast<int>(base) + static_cast<int>(corr);
    return { static_cast<uint8_t>(std::clamp(l, 0, 255)),
             static_cast<uint8_t>(std::clamp(r, 0, 255)) };
}

void CorridorNode::set_motors(uint8_t l, uint8_t r)
{
    RCLCPP_DEBUG(get_logger(), "Motors: L=%d R=%d", l, r);
    std_msgs::msg::UInt8MultiArray msg;
    msg.data = { l, r };
    motor_pub_->publish(msg);
}

void CorridorNode::stop_motors()
{
    // 127 = PWM střed = stop
    set_motors(static_cast<uint8_t>(PWM_CENTER),
               static_cast<uint8_t>(PWM_CENTER));
}

} // namespace nodes