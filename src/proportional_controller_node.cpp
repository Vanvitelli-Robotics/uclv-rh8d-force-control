#include "rclcpp/rclcpp.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/sensors_norm.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/float64_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/msg/motor_error.hpp" // New message for error publishing
#include "uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp"
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <algorithm> // Needed for std::find

// Class definition for the proportional controller
class ProportionalController : public rclcpp::Node
{
public:
    double gain_; // Proportional control gain value
    std::vector<int64_t> motor_ids_; // List of motor IDs managed by the controller

    // Messages to store the desired and measured forces
    uclv_seed_robotics_ros_interfaces::msg::SensorsNorm desired_norm_forces_;
    uclv_seed_robotics_ros_interfaces::msg::SensorsNorm measured_norm_forces_;
    
    // Flags to track whether the desired and measured forces have been received
    bool desired_norm_forces_received_ = false;
    bool measured_norm_forces_received_ = false;

    // Map to associate motors with their corresponding sensors
    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;

    // Subscriptions to receive sensor states and desired forces
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr desired_norm_forces_sub_;

    // Publisher to publish the error result
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorError>::SharedPtr error_pub_;

    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SetGain>::SharedPtr set_gain_service_;

    // Constructor for the ProportionalController class
    ProportionalController()
        : Node("proportional_controller"),
        gain_(1.0), // Default gain value
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())) // Declare and initialize the motor IDs parameter
    {
        // Check that the gain parameter is set correctly (non-negative)
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

        // Initialize the map between motors and sensors
        initialize_motor_to_sensor_map();

        // Subscribe to topics to receive sensor states and desired forces
        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            "norm_forces", 1, std::bind(&ProportionalController::measured_norm_forces_callback, this, std::placeholders::_1));

        desired_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            "/cmd/desired_norm_forces", 1, std::bind(&ProportionalController::desired_norm_forces_callback, this, std::placeholders::_1));

        // Create a publisher to publish the control error result
        error_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorError>(
            "/result_proportional_controller", 1);
        
        // Create a service to dynamically set the gain
        set_gain_service_ = this->create_service<uclv_seed_robotics_ros_interfaces::srv::SetGain>(
            "set_gain", std::bind(&ProportionalController::set_gain_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Function to initialize the map between motors and sensors
    void initialize_motor_to_sensor_map()
    {
        motor_to_sensor_map_[35] = {0}; // Motor ID 35 is associated with sensor ID 0
        motor_to_sensor_map_[36] = {1}; // Motor ID 36 is associated with sensor ID 1
        motor_to_sensor_map_[37] = {2}; // Motor ID 37 is associated with sensor ID 2
        motor_to_sensor_map_[38] = {3, 4}; // Motor ID 38 is associated with sensor IDs 3 and 4
    }

    // Callback to receive sensor state messages
    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::SensorsNorm::SharedPtr msg)
    {
        measured_norm_forces_ = *msg; // Update measured forces with the received message
        measured_norm_forces_received_ = true; // Set the flag indicating that measured forces have been received
        compute_and_publish_error(); // Attempt to compute and publish the control error
    }

    // Callback to receive desired force messages
    void desired_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::SensorsNorm::SharedPtr msg)
    {
        desired_norm_forces_ = *msg; // Update desired forces with the received message
        desired_norm_forces_received_ = true; // Set the flag indicating that desired forces have been received
        compute_and_publish_error(); // Attempt to compute and publish the control error
    }

    // Callback for the service to set the gain
    void set_gain_callback(const std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Request> request,
                           std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Response> response)
    {
        // Check if the gain value is non-negative
        if (request->gain < 0.0)
        {
            response->success = false;
            response->message = "Gain must be non-negative.";
            RCLCPP_WARN(this->get_logger(), "Attempted to set a negative gain.");
        }
        else
        {
            gain_ = request->gain; // Update the gain value
            response->success = true;
            response->message = "Gain updated successfully.";
            RCLCPP_INFO(this->get_logger(), "Gain updated to: %f", gain_);
        }
    }
    
    // Function to compute the control error and publish it
    void compute_and_publish_error()
    {
        // Check if the desired or measured forces data is missing
        if (!desired_norm_forces_received_ || !measured_norm_forces_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Desired or measured forces data not received yet.");
            return;
        }

        // Create a result message to publish the error
        uclv_seed_robotics_ros_interfaces::msg::MotorError error_msg;
        error_msg.header.stamp = this->now(); // Add timestamp to the message

        // Iterate over the motor IDs managed by the controller
        for (int64_t motor_id : motor_ids_)
        {
            // Find the motor ID in the sensor map
            auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id);
            if (sensor_ids_iter == motor_to_sensor_map_.end())
            {
                RCLCPP_FATAL(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
                continue; // Continue to the next motor if no mapping is found
            }

            // Retrieve the sensor IDs associated with the motor
            const auto &sensor_ids = sensor_ids_iter->second;

            // Iterate over the sensor IDs associated with the motor
            for (int64_t sensor_id : sensor_ids)
            {
                // Find the index of the sensor ID in the measured and desired force lists
                auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
                auto desired_force_iter = std::find(desired_norm_forces_.ids.begin(), desired_norm_forces_.ids.end(), sensor_id);

                // Check if the sensor ID is found in both the measured and desired forces lists
                if (measured_force_iter == measured_norm_forces_.ids.end())
                {
                    RCLCPP_FATAL(this->get_logger(), "Sensor ID: %ld not found in measured forces", sensor_id);
                    continue; // Continue to the next sensor if not found in measured forces
                }
                if (desired_force_iter == desired_norm_forces_.ids.end())
                {
                    RCLCPP_FATAL(this->get_logger(), "Sensor ID: %ld not found in desired forces", sensor_id);
                    continue; // Continue to the next sensor if not found in desired forces
                }

                // Obtain indices for the measured and desired forces
                size_t measured_id = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
                size_t desired_id = std::distance(desired_norm_forces_.ids.begin(), desired_force_iter);

                // Extract measured and desired norms
                double measured_norm = measured_norm_forces_.norm[measured_id].data; // Extract the `data` field from measured norm
                double desired_norm = desired_norm_forces_.norm[desired_id].data; // Extract the `data` field from desired norm

                // Calculate the error between desired and measured norms
                double error = desired_norm - measured_norm;

                RCLCPP_INFO(this->get_logger(), "Error for Sensor ID: %ld - Error: %f", sensor_id, error);

                // Add the error to the error message
                error_msg.motor_ids.push_back(motor_id);
                error_msg.errors.push_back(gain_ * error); // Scale error by proportional gain
            }
        }

        // Publish the error message
        error_pub_->publish(error_msg);

        // Reset the flags to ensure data is fresh for the next computation
        desired_norm_forces_received_ = false;
        measured_norm_forces_received_ = false;
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
