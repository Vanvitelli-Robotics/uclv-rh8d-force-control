#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_interfaces/msg/fts3_sensors.hpp"
#include "uclv_seed_robotics_interfaces/msg/float64_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <cmath>
#include <memory>

using std::placeholders::_1;

class ForceNorme : public rclcpp::Node
{
public:

    // Subscription to receive sensor state
    rclcpp::Subscription<uclv_seed_robotics_interfaces::msg::FTS3Sensors>::SharedPtr subscription_;

    // Publisher to publish the result of norm calculation
    rclcpp::Publisher<uclv_seed_robotics_interfaces::msg::Float64Stamped>::SharedPtr publisher_;

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
        // Iterate through each force vector in the received message
        for (const auto& force : msg->forces)
        {
            // Calculate the norm (magnitude) of the force vector
            // The values are assumed to be in mN, so they are converted to N by dividing by 1000.0
            double norm = std::sqrt(std::pow(force.x, 2) + std::pow(force.y, 2) + std::pow(force.z, 2)) / 1000.0;

            // Create the message to publish
            uclv_seed_robotics_interfaces::msg::Float64Stamped norm_msg;
            norm_msg.header = msg->header; // Copy the header from the incoming message
            norm_msg.data = norm; // Set the computed norm as the data

            // Publish the calculated norm
            publisher_->publish(norm_msg);
        }
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto force_norm_node = std::make_shared<ForceNorme>();
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
