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
    // Crea la subscription al topic 'sensor_state'
    subscription_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
        sensor_state_topic_, 10, std::bind(&ForceNorm::sensorStateCallback, this, _1));

    // Crea il publisher al topic 'norm_forces'
    publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
        norm_forces_topic_, 10);
}


private:
    // Callback function to process the incoming sensor state message
    void sensorStateCallback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        // Create the message to publish
        uclv_seed_robotics_ros_interfaces::msg::SensorsNorm norm_msg;
        norm_msg.ids = msg->ids;

        // Iterate through each force vector in the received message
        for (size_t i = 0; i < msg->forces.size(); ++i)
        {
            const auto &force = msg->forces[i];

            norm_msg.norm[i].header = msg->header;

            // Calculate the norm (magnitude) of the force vector
            // The values are assumed to be in mN, so they are converted to N by dividing by 1000.0
            double norm_value = std::sqrt(std::pow(force.x, 2) + std::pow(force.y, 2) + std::pow(force.z, 2)) / 1000.0;

            // Add the computed norm to the array
            norm_msg.norm[i].data = norm_value;
        }

        // Publish the message with all norms and corresponding IDs
        publisher_->publish(norm_msg);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto force_norm_node = std::make_shared<ForceNorm>();
        rclcpp::spin(force_norm_node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception caught: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
