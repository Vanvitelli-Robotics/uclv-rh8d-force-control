#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_interfaces/msg/fts3_sensors.hpp"
#include "uclv_seed_robotics_interfaces/msg/float64_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <cmath>
#include <memory>

using std::placeholders::_1;

// Node class definition for force norm calculation
class ForceNorme : public rclcpp::Node
{
public:
    ForceNorme()
    : Node("force_norm")
    {
        // Subscriber to the 'sensor_state' topic
        subscription_ = this->create_subscription<uclv_seed_robotics_interfaces::msg::FTS3Sensors>(
            "sensor_state", 10, std::bind(&ForceNorme::sensorStateCallback, this, _1));

        // Publisher to the 'norm_forces' topic
        publisher_ = this->create_publisher<uclv_seed_robotics_interfaces::msg::Float64Stamped>(
            "norm_forces", 10);
    }

private:
    // Callback function to process the incoming sensor state message
    void sensorStateCallback(const uclv_seed_robotics_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        for (const auto& force : msg->forces)
        {
            // Calculate the norm (magnitude) of the force vector
            double norm = std::sqrt(std::pow(force.x, 2) + (std::pow(force.y, 2) + (std::pow(force.y, 2))/1000.0; // /1000.0 cause mN -> N

            // Create the message to publish
            uclv_seed_robotics_interfaces::msg::Float64Stamped norm_msg;
            norm_msg.header = msg->header; // Keep the same header as the incoming message
            norm_msg.data = norm;

            // Publish the calculated norm
            publisher_->publish(norm_msg);
        }
    }

    // Subscriber and Publisher members
    rclcpp::Subscription<uclv_seed_robotics_interfaces::msg::FTS3Sensors>::SharedPtr subscription_;
    rclcpp::Publisher<uclv_seed_robotics_interfaces::msg::Float64Stamped>::SharedPtr publisher_;
};

// Main function for running the node
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceNorme>());
    rclcpp::shutdown();
    return 0;
}
