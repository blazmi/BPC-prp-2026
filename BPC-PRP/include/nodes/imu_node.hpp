#ifndef PRP_PROJECT_MOTOR_NODE_HPP
#define PRP_PROJECT_MOTOR_NODE_HPP


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "algorithms/planar_imu_integrator.hpp"

    namespace nodes {

        enum class ImuNodeMode {
            CALIBRATE,
            INTEGRATE,
        };

        class ImuNode : public rclcpp::Node {
        public:
            ImuNode();
            ~ImuNode() override = default;

            // Set the IMU mode
            void setMode(ImuNodeMode mode);

            // Get the current IMU mode
            ImuNodeMode getMode();

            // Get the results after integration
            auto getIntegratedResults();

            // Reset the class
            void reset_imu();

        private:

            void calibrate();
            void integrate();

            ImuNodeMode mode = ImuNodeMode::INTEGRATE;

            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
            algorithms::PlanarImuIntegrator planar_integrator_;

            std::vector<float> gyro_calibration_samples_;

            void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
        };
    }
#endif