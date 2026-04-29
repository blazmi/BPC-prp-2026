#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/imu.hpp>
//#include <std_msgs/msg/detail/u_int32_multi_array__struct.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "algorithms/pid.hpp"
#include "algorithms/PlanarImuIntegrator.hpp"
#include "odometry.hpp"

namespace nodes {

// ---------------------------------------------------------------------------
//  Scéna – co robot vidí tento tick
// ---------------------------------------------------------------------------
struct Scene {
    bool wall_front = false;
    bool wall_left  = false;
    bool wall_right = false;
};

// ---------------------------------------------------------------------------
//  Stavy
// ---------------------------------------------------------------------------
enum class RobotState {
    CALIBRATING,

    STOPPED,

    // Jede rovně, PID centruje bočními zdmi.
    // Přechod do OPEN jakmile zmizí obě boční zdi.
    CORRIDOR,

    // Chybí obě boční zdi (křižovatka/zatáčka).
    // IMU drží kurz. Odometrie hlídá:
    //   a) min. vjezdovou vzdálenost (settle_dist) – robot nesmí hned reagovat
    //   b) přední zeď nebo návrat zdi → decide_turn() / → CORRIDOR
    OPEN,

    // Točení na místě, IMU měří úhel.
    // Po dokončení → LEAVING (ochranná vzdálenost od místa otočení).
    TURN_LEFT,
    TURN_RIGHT,
    TURN_180,

    // Robot právě vyjel z otočení nebo z OPEN zpět do koridoru.
    // Jede rovně (IMU drží kurz) dokud neujede settle_dist.
    // Pak teprve přejde do CORRIDOR.
    LEAVING,
};

// ---------------------------------------------------------------------------
//  Parametry
// ---------------------------------------------------------------------------
struct CorridorParams {
    // Kinematika
    double  wheel_radius     = 0.033;
    double  wheel_base       = 0.160;
    int     ticks_per_rev    = 585;

    // Rychlosti 0–255
    uint8_t base_speed       = 20;
    uint8_t turn_speed       = 25;
    uint8_t stop_speed       = 0;   // 128 = PWM-center, nebo 0

    // PID centrování zdmi (CORRIDOR) – vstup: L-R [m]
    float   pid_kp           = 5.0f;
    float   pid_ki           =  0.5f;
    float   pid_kd           = 5.0f;

    // PID kurzu (OPEN + LEAVING) – vstup: yaw error [rad]
    float   hdg_kp           = 5.0f;
    float   hdg_ki           =  0.5f;
    float   hdg_kd           = 5.0f;

    // Vzdálenosti [m]
    float   wall_stop_dist   = 0.22f; // přední zeď → zastav a rozhoduj
    float   junction_thr     = 0.50f; // práh "zeď existuje" na straně – VYLADIT
    float   wall_follow_dist = 0.20f; // cílová vzdálenost od jedné zdi
    float   sensor_min       = 0.15f; // slepá zóna senzoru

    // Ochranná vzdálenost po přechodu stavu [m]
    // – po otočení: ujede settle_dist než začne vyhodnocovat scénu
    // – po vstupu do OPEN: čeká settle_dist než přepne zpět do CORRIDOR
    float   settle_dist      = 0.15f; // VYLADIT

    // Točení
    float   turn_target_deg  = 90.0f;
    float   turn_tol_deg     =  3.0f;

    // ArUco: výchozí příkaz když žádná značka není vidět
    // 0 = rovně, 1 = vlevo, 2 = vpravo
    uint8_t default_aruco    = 2;     // výchozí: vpravo

    // IMU kalibrace
    int     imu_calib_n      = 200;
};

// ---------------------------------------------------------------------------
//  Node
// ---------------------------------------------------------------------------
class CorridorNode : public rclcpp::Node {
public:
    explicit CorridorNode(const CorridorParams& p = CorridorParams{});

private:
    // Callbacky
    void on_lidar   (const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void on_imu     (const sensor_msgs::msg::Imu::SharedPtr msg);
    void on_encoders(const std_msgs::msg::UInt32MultiArray::SharedPtr msg);
    void on_enable  (const std_msgs::msg::Bool::SharedPtr msg);
    void on_aruco   (const std_msgs::msg::UInt8::SharedPtr msg);
    void on_timer   ();

    Scene compute_scene() const;

    // Stavová logika
    void process_corridor(float dt, const Scene& s);
    void process_open    (float dt, const Scene& s);
    void process_turn    ();
    void process_leaving (float dt);

    // Přechody
    void enter_open   ();
    void enter_turn   (float delta_rad, RobotState ts);
    void enter_leaving();                        // vždy po otočení
    void decide_turn  (const Scene& s);          // použije aruco_cmd_

    // Pomocné
    void  drive_heading(float dt, float ref_yaw);  // jede rovně, IMU drží kurz
    void  set_motors (uint8_t l, uint8_t r);
    void  stop_motors();
    static std::pair<uint8_t,uint8_t> blend(uint8_t base, float corr);
    float odometry_dist_from(const Pose& ref) const;

    // -----------------------------------------------------------------------
    CorridorParams   p_;
    algorithms::Pid  pid_wall_;
    algorithms::Pid  pid_hdg_;
    algorithms::PlanarImuIntegrator imu_;
    EncoderOdometry  odom_;

    RobotState state_ = RobotState::CALIBRATING;

    // LiDAR
    float lf_ = std::numeric_limits<float>::infinity();
    float ll_ = std::numeric_limits<float>::infinity();
    float lr_ = std::numeric_limits<float>::infinity();
    bool  lidar_ok_ = false;

    // IMU
    std::vector<float> calib_;
    bool imu_ok_ = false;
    rclcpp::Time last_imu_t_;

    // ArUco
    // Ukládáme poslední platný příkaz (0/1/2).
    // Resetujeme na default po každém použití.
    uint8_t aruco_cmd_      = 255;   // 255 = žádná značka
    bool    aruco_fresh_    = false; // true = přišel nový příkaz od posledního vjezdu

    // Točení
    float turn_target_yaw_ = 0.0f;

    // Referenční pose/yaw pro OPEN a LEAVING
    float open_ref_yaw_  = 0.0f;
    Pose  open_start_    = {0,0,0};

    float leaving_ref_yaw_ = 0.0f;
    Pose  leaving_start_   = {0,0,0};

    // ROS
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr            imu_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32MultiArray>::SharedPtr  enc_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              en_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr             aruco_sub_;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr      motor_pub_;
    rclcpp::TimerBase::SharedPtr                                      timer_;

    rclcpp::Time last_t_;
    bool         enabled_ = false;
};

} // namespace nodes