#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/fts3_sensors.hpp"
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <geometry_msgs/msg/vector3.hpp>

class ProportionalController : public rclcpp::Node
{
public:

    double gain_; // Gain value for the proportional controller
    std::vector<int64_t> motor_ids_;    // List of motor IDs that the controller will manage

    
    // Messages to store desired and measured forces
    uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors desired_forces_;
    uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors measured_forces_;
    
    // Flags to track whether desired and measured forces have been received
    bool desired_forces_received_ = false;
    bool measured_forces_received_ = false;

    // Map to associate motors with their corresponding sensors
    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;

    // Subscriptions to receive sensor state and desired forces
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr sensor_state_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr desired_forces_sub_;

    // Publisher to publish the result of the proportional control
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>::SharedPtr result_pub_;

    ProportionalController()
        : Node("proportional_controller"),
          gain_(this->declare_parameter<double>("gain", 1.0)),
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>()))
    {
        // Check if the gain parameter is set correctly (non-negative)
        if (gain_ < 0.0)
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'gain' must be non-negative. Exiting...");
            throw std::invalid_argument("Parameter 'gain' must be non-negative");
        }

        // Ensure that the list of motor IDs is not empty
        if (motor_ids_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_ids' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_ids' is empty or not set");
        }

        // Initialize the mapping between motors and sensors
        initialize_motor_to_sensor_map();

        // Subscribe to sensor state and desired forces topics
        sensor_state_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            "/sensor_state", 1, std::bind(&ProportionalController::sensor_state_callback, this, std::placeholders::_1));

        desired_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            "/cmd/desired_forces", 1, std::bind(&ProportionalController::desired_forces_callback, this, std::placeholders::_1));

        // Create publisher to publish control results
        result_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors>(
            "/result_proportional_controller", 1);
    }

private:
    // Function to initialize the mapping between motors and sensors
    void initialize_motor_to_sensor_map()
    {
        // Example mapping where motor IDs are associated with sensor IDs
        motor_to_sensor_map_[35] = {0};
        motor_to_sensor_map_[36] = {1};
        motor_to_sensor_map_[37] = {2};
        motor_to_sensor_map_[38] = {3, 4};
    }

    // Callback function for receiving sensor state messages
    void sensor_state_callback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        measured_forces_ = *msg; // Update measured forces
        measured_forces_received_ = true;
        compute_and_publish_result(); // Attempt to compute and publish the control result
    }

    // Callback function for receiving desired forces messages
    void desired_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors::SharedPtr msg)
    {
        desired_forces_ = *msg; // Update desired forces
        desired_forces_received_ = true;
        // RCLCPP_INFO(this->get_logger(), "Received desired forces:");
        // for (size_t i = 0; i < msg->forces.size(); ++i)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Sensor ID: %ld, Force: (%f, %f, %f)",
        //                 msg->ids[i], msg->forces[i].x, msg->forces[i].y, msg->forces[i].z);
        // }
        compute_and_publish_result(); // Attempt to compute and publish the control result
    }

    // Function to compute the control result and publish it
    void compute_and_publish_result()
    {
        // Check if desired or measured forces data is missing
        if (desired_forces_.forces.empty() || measured_forces_.forces.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Desired or measured forces vectors are empty.");
            return;
        }
        
        // Initialize the message to publish
        uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors result_msg;
        result_msg.header.stamp = this->now(); // Timestamp the result message

        // Iterate over the motor IDs that the controller manages
        for (int64_t motor_id : motor_ids_)
        {
            // Find the motor_id in the sensor map
            auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id);
            if (sensor_ids_iter == motor_to_sensor_map_.end())
            {
                RCLCPP_FATAL(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
                continue;
            }

            // If the id is found in the map, get the second element of the map
            const auto &sensor_ids = sensor_ids_iter->second;
            
            // Log sensor_ids for the current motor_id
            std::string sensor_ids_str;
            for (auto sensor_id : sensor_ids)
            {
                sensor_ids_str += std::to_string(sensor_id) + " ";
            }
            // RCLCPP_INFO(this->get_logger(), "Motor ID: %ld maps to Sensor IDs: %s", motor_id, sensor_ids_str.c_str());

            // For each sensor id associated with the motor
            for (int64_t sensor_id : sensor_ids)
            {
                auto measured_force_iter = std::find(measured_forces_.ids.begin(), measured_forces_.ids.end(), sensor_id);
                auto desired_force_iter = std::find(desired_forces_.ids.begin(), desired_forces_.ids.end(), sensor_id);

                // Check if the id is found in both the measured forces and desired forces lists
                if (measured_force_iter == measured_forces_.ids.end())
                {
                    RCLCPP_FATAL(this->get_logger(), "Sensor ID: %ld not found in measured forces", sensor_id);
                    continue;
                }
                if (desired_force_iter == desired_forces_.ids.end())
                {
                    RCLCPP_FATAL(this->get_logger(), "Sensor ID: %ld not found in desired forces", sensor_id);
                    continue;
                }

                // For each found sensor, get the id within the measured and desired forces
                size_t measured_idx = std::distance(measured_forces_.ids.begin(), measured_force_iter);
                size_t desired_idx = std::distance(desired_forces_.ids.begin(), desired_force_iter);

                // Calculate the error between desired and measured forces
                double error_x = desired_forces_.forces[desired_idx].x - measured_forces_.forces[measured_idx].x;
                double error_y = desired_forces_.forces[desired_idx].y - measured_forces_.forces[measured_idx].y;
                double error_z = desired_forces_.forces[desired_idx].z - measured_forces_.forces[measured_idx].z;

                RCLCPP_INFO(this->get_logger(), "Error for Sensor ID: %ld - X: %f, Y: %f, Z: %f",
                            sensor_id, error_x, error_y, error_z);

                // Apply the proportional control law
                geometry_msgs::msg::Vector3 result_force;
                result_force.x = gain_ * error_x;
                result_force.y = gain_ * error_y;
                result_force.z = gain_ * error_z;

                // Append the computed force to the result message
                result_msg.ids.push_back(motor_id);
                result_msg.forces.push_back(result_force);

                RCLCPP_INFO(this->get_logger(), "Computed result for Motor ID: %ld - X: %f, Y: %f, Z: %f",
                            motor_id, result_force.x, result_force.y, result_force.z);
            }
        }

        // Publish the computed control result
        result_pub_->publish(result_msg);

        // Reset flags to ensure fresh data for next computation
        desired_forces_received_ = false;
        measured_forces_received_ = false;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto proportional_controller_node = std::make_shared<ProportionalController>();
        rclcpp::spin(proportional_controller_node);
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
