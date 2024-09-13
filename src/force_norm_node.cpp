#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/sensors_norm.hpp"
#include <cmath>
#include <memory>

using std::placeholders::_1;

class ForceNorm : public rclcpp::Node
{
public:
    std::string sensor_state_topic_;
    std::string norm_forces_topic_;

    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr subscription_;
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr publisher_;

    ForceNorm()
    : Node("force_norm"),
      sensor_state_topic_(this->declare_parameter<std::string>("sensor_state_topic", "sensor_state")),
      norm_forces_topic_(this->declare_parameter<std::string>("norm_forces_topic", "norm_forces"))
    {
        subscription_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            sensor_state_topic_, 10, std::bind(&ForceNorm::sensorStateCallback, this, _1));

        publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            norm_forces_topic_, 10);
    }

private:
    void sensorStateCallback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        uclv_seed_robotics_ros_interfaces::msg::SensorsNorm norm_msg;
        norm_msg.header.stamp = this->now();
        norm_msg.ids = msg->ids;
        norm_msg.norms.resize(msg->forces.size());

        for (size_t i = 0; i < msg->forces.size(); ++i)
        {
            // Calcolo della norma della forza 3D
            norm_msg.norms[i] = std::sqrt(std::pow(msg->forces[i].x, 2) + std::pow(msg->forces[i].y, 2) + std::pow(msg->forces[i].z, 2))/(1000.0);

            RCLCPP_INFO(this->get_logger(), "Calculated norm for sensor ID: %d, Norm: %f", msg->ids[i], norm_msg.norms[i]);
        }

        publisher_->publish(norm_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto force_norm_node = std::make_shared<ForceNorm>();
    rclcpp::spin(force_norm_node);
    rclcpp::shutdown();
    return 0;
}
