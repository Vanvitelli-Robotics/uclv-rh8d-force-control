#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/sensors_norm.hpp"
#include <cmath>
#include <memory>

using std::placeholders::_1;

class ForceNorm : public rclcpp::Node
{
public:
    // Topic names for sensor state and measured norms
    std::string sensor_state_topic_;
    std::string measured_norm_topic_;

    // Subscription to sensor state messages
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr subscription_;
    // Publisher for sensor norms messages
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr publisher_;

    // Constructor of the class
    ForceNorm()
    : Node("force_norm"),
      // Declare and initialize parameters for the node
      sensor_state_topic_(this->declare_parameter<std::string>("sensor_state_topic", "sensor_state")),
      measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", "norm_forces"))
    {
        // Create a subscription to receive sensor state messages
        subscription_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            sensor_state_topic_, 10, std::bind(&ForceNorm::sensorStateCallback, this, _1));

        // Create a publisher to send sensor norm messages
        publisher_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            measured_norm_topic_, 10);
    }

private:
    // Callback function for processing received sensor state messages
    void sensorStateCallback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        // Create a message for the calculated norms
        uclv_seed_robotics_ros_interfaces::msg::SensorsNorm norm_msg;
        norm_msg.header.stamp = this->now();  // Set the current time stamp
        norm_msg.ids = msg->ids;  // Copy sensor IDs
        norm_msg.norms.resize(msg->forces.size());  // Resize norms vector to match the size of forces

        // Calculate the norm of each 3D force
        for (size_t i = 0; i < msg->forces.size(); ++i)
        {
            // Compute the 3D force norm
            norm_msg.norms[i] = std::sqrt(std::pow(msg->forces[i].x, 2) + 
                                          std::pow(msg->forces[i].y, 2) + 
                                          std::pow(msg->forces[i].z, 2)) / 1000.0;

            // Log the calculated norm for debugging purposes
            RCLCPP_INFO(this->get_logger(), "Calculated norm for sensor ID: %d, Norm: %f", msg->ids[i], norm_msg.norms[i]);
        }

        // Publish the calculated norms
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
