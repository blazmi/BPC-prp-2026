#include "nodes/imu_node.hpp"

namespace nodes {

    ImuNode::ImuNode() : Node("imu_node"), mode(ImuNodeMode::CALIBRATE) {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10, std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "IMU Node spuštěn. Probíhá kalibrace (nehybejte s robotem!)...");
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        rclcpp::Time current_time = this->now();

        // FIX: last_time jako member proměnná (ne static), inicializovaná na nulu.
        //      Při prvním volání pouze nastavíme čas a vrátíme se — žádné dt nespočítáme.
        if (last_time_.nanoseconds() == 0) {
            last_time_ = current_time;
            return;
        }

        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // FIX: Guard na nesmyslné dt — příliš malé (duplikát) nebo příliš velké (skok)
        if (dt <= 0.0 || dt > 0.1) return;

        if (mode == ImuNodeMode::CALIBRATE) {
            gyro_calibration_samples_.push_back(msg->angular_velocity.z);

            if (gyro_calibration_samples_.size() >= 200) {
                planar_integrator_.setCalibration(gyro_calibration_samples_);
                mode = ImuNodeMode::INTEGRATE;

                // FIX: Kritický reset last_time_ po kalibraci.
                //      Bez tohoto resetu by první dt po přepnutí na INTEGRATE
                //      bylo rovno celé době kalibrace (~2-4 s), což by způsobilo
                //      obrovský skok v integrovaném yaw.
                last_time_ = this->now();

                RCLCPP_INFO(this->get_logger(), "Kalibrace hotova. Přepínám na integraci.");
            }
        } else {
            // dt je nyní vždy v rozumném rozsahu (0, 0.1] s
            planar_integrator_.update(msg->angular_velocity.z, dt);

            // Zde bys mohl publikovat aktuální yaw na nový topic
        }
    }

} // namespace nodes