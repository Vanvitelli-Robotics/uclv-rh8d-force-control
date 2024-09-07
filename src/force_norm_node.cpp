#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_interfaces/msg/fts3_sensors.hpp"
#include "uclv_seed_robotics_interfaces/msg/sensors_norm.hpp"
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
    rclcpp::Publisher<uclv_seed_robotics_interfaces::msg::SensorsNorm>::SharedPtr publisher_;

    ForceNorme()
    : Node("force_norm")
    {
        // Subscriber to the 'sensor_state' topic
        subscription_ = this->create_subscription<uclv_seed_robotics_interfaces::msg::FTS3Sensors>(
            "sensor_state", 10, std::bind(&ForceNorme::sensorStateCallback, this, _1));

        // Publisher to the 'norm_forces' topic
        publisher_ = this->create_publisher<uclv_seed_robotics_interfaces::msg::SensorsNorm>(
            "norm_forces", 10);
    }

private:
    // Callback function to process the incoming sensor state message
    void sensorStateCallback(const uclv_seed_robotics_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        // Create the message to publish
        uclv_seed_robotics_interfaces::msg::SensorsNorm norm_msg;
        norm_msg.ids = msg->ids; // Copy the sensor IDs from the incoming message
        norm_value.norm.header = msg->header; // Copy the header from the incoming message


        // Initialize a map to store norms for each ID
        std::unordered_map<uint16_t, double> norms_map;

        // Iterate through each force vector in the received message
        for (size_t i = 0; i < msg->forces.size(); ++i)
        {
            const auto& force = msg->forces[i];
            uint16_t sensor_id = msg->ids[i];

            // Calculate the norm (magnitude) of the force vector
            // The values are assumed to be in mN, so they are converted to N by dividing by 1000.0
            double norm = std::sqrt(std::pow(force.x, 2) + std::pow(force.y, 2) + std::pow(force.z, 2)) / 1000.0;

            // Store the computed norm in the map
            norms_map[sensor_id] = norm;
        }

        // Populate the norm array with norms corresponding to the IDs
        for (auto id : norm_msg.ids)
        {
            // Set the norm for the current ID (or 0 if ID not found)
            auto it = norms_map.find(id);
            // Set the norm value for the current ID; use the value from the map if the ID is found, otherwise set it to 0.0
            norm_value.data = (it != norms_map.end()) ? it->second : 0.0;

            norm_msg.norm.push_back(norm_value);
        }

        // Publish the message with all norms and corresponding IDs
        publisher_->publish(norm_msg);
    }
};

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
