#include "rclcpp/rclcpp.hpp"  // Include the main header for ROS 2 C++ client library
#include "uclv_seed_robotics_ros_interfaces/msg/sensors_norm.hpp"  // Include custom message type for sensor data
#include "uclv_seed_robotics_ros_interfaces/msg/motor_error.hpp"  // Include custom message type for motor error data
#include "uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp"  // Include custom service type for setting the gain value
#include <stdexcept>  // Include standard library for handling exceptions
#include <unordered_map>  // Include standard library for using hash maps
#include <vector>  // Include standard library for using vectors
#include <algorithm>  // Include standard library for common algorithms like std::find

// Definition of the ProportionalController class, which inherits from rclcpp::Node
class ProportionalController : public rclcpp::Node
{
public:
    double gain_; // Proportional gain value for the controller
    std::vector<int64_t> motor_ids_; // List of motor IDs managed by the controller
    std::vector<std::string> motor_sensor_mappings_; // Mapping motor ID - Sensors

    uclv_seed_robotics_ros_interfaces::msg::SensorsNorm desired_norm_forces_;  // Message for desired normalized forces
    uclv_seed_robotics_ros_interfaces::msg::SensorsNorm measured_norm_forces_; // Message for measured normalized forces

    bool desired_norm_forces_received_ = false;  // Flag indicating if desired forces data has been received
    bool measured_norm_forces_received_ = false; // Flag indicating if measured forces data has been received

    // Mapping of motor IDs to their corresponding sensor IDs
    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;

    // ROS 2 subscriptions to receive sensor data
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>::SharedPtr desired_norm_forces_sub_;

    // ROS 2 publisher to publish motor errors
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::MotorError>::SharedPtr error_pub_;

    // ROS 2 service to handle requests for setting the gain value
    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SetGain>::SharedPtr set_gain_service_;

    // Constructor for initializing the node
    ProportionalController()
        : Node("proportional_controller"),  // Initialize the node with the name "proportional_controller"
          gain_(this->declare_parameter<double>("gain", 1.0)), // Initialize gain from the ROS parameter (default: 1.0)
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())),  // Initialize motor IDs from the ROS parameter
          motor_sensor_mappings_(this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings", std::vector<std::string>()))
    {
        // Check if the gain is non-negative; terminate if not
        if (gain_ < 0.0)
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'gain' must be non-negative. Exiting...");
            throw std::invalid_argument("Parameter 'gain' must be non-negative");
        }

        // Check if motor IDs have been provided; terminate if not
        if (motor_ids_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_ids' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_ids' is empty or not set");
        }

        // Initialize the mapping between motors and their associated sensors
        initialize_motor_to_sensor_map();

        // Create subscription to measured normalized forces
        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            "norm_forces", 10, std::bind(&ProportionalController::measured_norm_forces_callback, this, std::placeholders::_1));

        // Create subscription to desired normalized forces
        desired_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::SensorsNorm>(
            "/cmd/desired_norm_forces", 10, std::bind(&ProportionalController::desired_norm_forces_callback, this, std::placeholders::_1));

        // Create publisher for motor errors
        error_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::MotorError>(
            "/result_proportional_controller", 10);

        // Create service for setting the gain
        set_gain_service_ = this->create_service<uclv_seed_robotics_ros_interfaces::srv::SetGain>(
            "set_gain", std::bind(&ProportionalController::set_gain_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    // Initialize the mapping between motor IDs and their associated sensor IDs
//    void initialize_motor_to_sensor_map()
//    {
//        motor_to_sensor_map_[35] = {0};  // Motor ID 35 is associated with Sensor ID 0
//        motor_to_sensor_map_[36] = {1};  // Motor ID 36 is associated with Sensor ID 1
//        motor_to_sensor_map_[37] = {2};  // Motor ID 37 is associated with Sensor ID 2
//        motor_to_sensor_map_[38] = {3, 4}; // Motor ID 38 is associated with Sensor IDs 3 and 4
//    }

void initialize_motor_to_sensor_map()
    {
   //     std::vector<std::string> motor_sensor_mappings_ = 
   //         this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings_", std::vector<std::string>());

        if (motor_sensor_mappings_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "Parameter 'motor_sensor_mappings_' is empty or not set. Exiting...");
            throw std::runtime_error("Parameter 'motor_sensor_mappings_' is empty or not set");
        }

        for (const auto& mapping : motor_sensor_mappings_)
        {
            std::istringstream iss(mapping);
            int64_t motor_id;
            std::vector<int64_t> sensor_ids;
            char delimiter;

            if (!(iss >> motor_id >> delimiter))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid mapping format: %s", mapping.c_str());
                continue;
            }

            int64_t sensor_id;
            while (iss >> sensor_id)
            {
                sensor_ids.push_back(sensor_id);
                iss >> delimiter; // consume the comma if present
            }

            if (sensor_ids.empty())
            {
                RCLCPP_ERROR(this->get_logger(), "No sensor IDs found for motor ID: %ld", motor_id);
                continue;
            }

            motor_to_sensor_map_[motor_id] = sensor_ids;
            RCLCPP_INFO(this->get_logger(), "Mapped motor ID %ld to sensor IDs: %s", 
                        motor_id, [&sensor_ids]() {
                            std::ostringstream oss;
                            for (size_t i = 0; i < sensor_ids.size(); ++i) {
                                if (i > 0) oss << ", ";
                                oss << sensor_ids[i];
                            }
                            return oss.str();
                        }().c_str());
        }

        if (motor_to_sensor_map_.empty())
        {
            RCLCPP_FATAL(this->get_logger(), "No valid motor-to-sensor mappings were created. Exiting...");
            throw std::runtime_error("No valid motor-to-sensor mappings were created");
        }
    }

    // Callback function for receiving measured normalized forces
    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::SensorsNorm::SharedPtr msg)
    {
        measured_norm_forces_ = *msg;  // Store the received message
        measured_norm_forces_received_ = true;  // Set the flag indicating that data has been received
        compute_and_publish_error();  // Compute and publish the motor errors
    }

    // Callback function for receiving desired normalized forces
    void desired_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::SensorsNorm::SharedPtr msg)
    {
        desired_norm_forces_ = *msg;  // Store the received message
        desired_norm_forces_received_ = true;  // Set the flag indicating that data has been received
        compute_and_publish_error();  // Compute and publish the motor errors
    }

    // Callback function for setting the gain value
    void set_gain_callback(const std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Request> request,
                           std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Response> response)
    {
        if (request->gain < 0.0)  // Check if the requested gain is negative
        {
            response->success = false;
            response->message = "Gain must be non-negative.";
            RCLCPP_WARN(this->get_logger(), "Attempted to set a negative gain.");
        }
        else  // Update the gain value if it is valid
        {
            gain_ = request->gain;
            response->success = true;
            response->message = "Gain updated successfully.";
            RCLCPP_INFO(this->get_logger(), "Gain updated to: %f", gain_);
        }
    }

    // Function to compute and publish the motor errors
    void compute_and_publish_error()
    {
        // Check if both desired and measured forces data have been received
        if (!desired_norm_forces_received_ || !measured_norm_forces_received_)
        {
            RCLCPP_WARN(this->get_logger(), "Desired or measured forces data not received yet.");
            return;
        }

        // Initialize the motor error message
        uclv_seed_robotics_ros_interfaces::msg::MotorError error_msg;
        error_msg.header.stamp = this->now();  // Set the timestamp for the message

        // Iterate over each motor ID to compute the error
        for (int64_t motor_id : motor_ids_)
        {
            auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id);  // Find the associated sensor IDs for the motor
            if (sensor_ids_iter == motor_to_sensor_map_.end())  // Check if the motor has a valid mapping
            {
                RCLCPP_ERROR(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
                continue;
            }

            const auto &sensor_ids = sensor_ids_iter->second;  // Get the associated sensor IDs for the motor

            // Iterate over each sensor ID to compute the error
            for (int64_t sensor_id : sensor_ids)
            {
                // Find the index of the sensor ID in the measured forces message
                auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
                auto desired_force_iter = std::find(desired_norm_forces_.ids.begin(), desired_norm_forces_.ids.end(), sensor_id);

                // Check if the sensor ID is found in the measured forces
                if (measured_force_iter == measured_norm_forces_.ids.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in measured forces", sensor_id);
                    continue;
                }
                // Check if the sensor ID is found in the desired forces
                if (desired_force_iter == desired_norm_forces_.ids.end())
                {
                    RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in desired forces", sensor_id);
                    continue;
                }

                // Compute the indices of the sensor ID in the measured and desired forces
                size_t measured_id = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
                size_t desired_id = std::distance(desired_norm_forces_.ids.begin(), desired_force_iter);

                // Retrieve the measured and desired normalized forces
                double measured_norm = measured_norm_forces_.norms[measured_id];
                double desired_norm = desired_norm_forces_.norms[desired_id];

                // Compute the error for the sensor
                double error = desired_norm - measured_norm;

                RCLCPP_INFO(this->get_logger(), "Error for Sensor ID: %ld - Error: %f", sensor_id, error);

                // Add the motor ID and the computed error to the error message
                error_msg.motor_ids.push_back(motor_id);
                error_msg.errors.push_back(gain_ * error);  // Apply the proportional gain to the error
            }
        }

        // Publish the computed motor errors
        error_pub_->publish(error_msg);

        // Reset the flags for receiving new data
        desired_norm_forces_received_ = false;
        measured_norm_forces_received_ = false;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // Initialize the ROS 2 client library
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
