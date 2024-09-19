
#include "rclcpp/rclcpp.hpp" // Include the main header for ROS 2 C++ client library
#include "uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp"
#include "uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp" // Include custom service type for setting the gain value
#include <stdexcept>                                          // Include standard library for handling exceptions
#include <unordered_map>                                      // Include standard library for using hash maps
#include <vector>                                             // Include standard library for using vectors
#include <algorithm>                                          // Include standard library for common algorithms like std::find

// Definition of the ProportionalController class, which inherits from rclcpp::Node
class ProportionalController : public rclcpp::Node
{
public:
    double gain_;                                     // Proportional gain value for the controller
    std::vector<int64_t> motor_ids_;                  // List of motor IDs managed by the controller
    std::vector<std::string> motor_sensor_mappings_;  // Mapping motor ID - sensors ID
    std::vector<std::string> sensor_weight_mappings_; // Mapping sensor ID - weight

    std::string measured_norm_topic_;       // Name of the topic for measured normalized forces
    std::string desired_norm_topic_;        // Name of the topic for desired normalized forces
    std::string proportional_result_topic_; // Name of the topic for publishing errors
    std::string set_gain_service_name_;     // Name of the service to set gain

    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped desired_norm_forces_;  // Message for desired normalized forces
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped measured_norm_forces_; // Message for measured normalized forces

    bool desired_norm_forces_received_ = false;  // Flag indicating if desired forces data has been received
    bool measured_norm_forces_received_ = false; // Flag indicating if measured forces data has been received

    // Mapping of motor IDs to their corresponding sensor IDs
    std::unordered_map<int64_t, std::vector<int64_t>> motor_to_sensor_map_;
    // Mapping of sensor IDs to their corresponding weights
    std::unordered_map<int64_t, std::vector<double>> sensor_to_weight_map_;

    // ROS 2 subscriptions to receive sensor data
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr measured_norm_forces_sub_;
    rclcpp::Subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr desired_norm_forces_sub_;

    // ROS 2 publisher to publish motor errors
    rclcpp::Publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>::SharedPtr error_pub_;

    // ROS 2 service to handle requests for setting the gain value
    rclcpp::Service<uclv_seed_robotics_ros_interfaces::srv::SetGain>::SharedPtr set_gain_service_;

    // Constructor for initializing the node
    ProportionalController()
        : Node("proportional_controller"),                                                                // Initialize the node with the name "proportional_controller"
          gain_(this->declare_parameter<double>("gain", 1.0)),                                            // Initialize gain from the ROS parameter (default: 1.0)
          motor_ids_(this->declare_parameter<std::vector<int64_t>>("motor_ids", std::vector<int64_t>())), // Initialize motor IDs from the ROS parameter
          motor_sensor_mappings_(this->declare_parameter<std::vector<std::string>>("motor_sensor_mappings", std::vector<std::string>())),
          sensor_weight_mappings_(this->declare_parameter<std::vector<std::string>>("sensor_weight_mappings", std::vector<std::string>())),
          measured_norm_topic_(this->declare_parameter<std::string>("measured_norm_topic", "norm_forces")),
          desired_norm_topic_(this->declare_parameter<std::string>("desired_norm_topic", "/cmd/desired_norm_forces")),
          proportional_result_topic_(this->declare_parameter<std::string>("proportional_result_topic", "result_proportional_controller")),
          set_gain_service_name_(this->declare_parameter<std::string>("set_gain_service_name", "set_gain"))
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
        initialize_sensor_to_weight_map();

        // Create subscription to measured normalized forces
        measured_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            measured_norm_topic_, 10, std::bind(&ProportionalController::measured_norm_forces_callback, this, std::placeholders::_1));

        // Create subscription to desired normalized forces
        desired_norm_forces_sub_ = this->create_subscription<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            desired_norm_topic_, 10, std::bind(&ProportionalController::desired_norm_forces_callback, this, std::placeholders::_1));

        // Create publisher for motor errors
        error_pub_ = this->create_publisher<uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped>(
            proportional_result_topic_, 10);

        // Create service for setting the gain
        set_gain_service_ = this->create_service<uclv_seed_robotics_ros_interfaces::srv::SetGain>(
            set_gain_service_name_, std::bind(&ProportionalController::set_gain_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
   template <typename KeyType, typename ValueType>
void initialize_map_from_mappings(
    const std::vector<std::string> &mappings,
    std::unordered_map<KeyType, std::vector<ValueType>> &map,
    const std::string &map_type)
{
    if (mappings.empty())
    {
        RCLCPP_FATAL(this->get_logger(), "Parameter '%s' is empty or not set. Exiting...", map_type.c_str());
        throw std::runtime_error("Parameter '" + map_type + "' is empty or not set");
    }

    for (const auto &mapping : mappings)
    {
        std::istringstream iss(mapping);
        KeyType key;
        std::vector<ValueType> values;
        char delimiter;

        if (!(iss >> key >> delimiter))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid mapping format: %s", mapping.c_str());
            continue;
        }

        ValueType value;
        while (iss >> value)
        {
            values.push_back(value);
            iss >> delimiter; // consume the comma if present
        }

        if (values.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No values found for key: %ld", static_cast<int64_t>(key));
            continue;
        }

        map[key] = values;
        RCLCPP_INFO(this->get_logger(), "Mapped key %ld to values: %s",
                    static_cast<int64_t>(key), [&values]()
                    {
                        std::ostringstream oss;
                        for (size_t i = 0; i < values.size(); ++i)
                        {
                            if (i > 0)
                                oss << ", ";
                            oss << values[i];
                        }
                        return oss.str();
                    }()
                    .c_str());
    }

    if (map.empty())
    {
        RCLCPP_FATAL(this->get_logger(), "No valid mappings were created for '%s'. Exiting...", map_type.c_str());
        throw std::runtime_error("No valid mappings were created for '" + map_type + "'");
    }
}


void initialize_motor_to_sensor_map()
{
    initialize_map_from_mappings<int64_t, int64_t>(motor_sensor_mappings_, motor_to_sensor_map_, "motor_sensor_mappings");
}

void initialize_sensor_to_weight_map()
{
    initialize_map_from_mappings<int64_t, double>(sensor_weight_mappings_, sensor_to_weight_map_, "sensor_weight_mappings");
}


    // Callback function for receiving measured normalized forces
    void measured_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        measured_norm_forces_ = *msg;          // Store the received message
        measured_norm_forces_received_ = true; // Set the flag indicating that data has been received
        compute_and_publish_error();           // Compute and publish the motor errors
    }

    // Callback function for receiving desired normalized forces
    void desired_norm_forces_callback(const uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped::SharedPtr msg)
    {
        desired_norm_forces_ = *msg;          // Store the received message
        desired_norm_forces_received_ = true; // Set the flag indicating that data has been received
        compute_and_publish_error();          // Compute and publish the motor errors
    }

    // Callback function for setting the gain value
    void set_gain_callback(const std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Request> request,
                           std::shared_ptr<uclv_seed_robotics_ros_interfaces::srv::SetGain::Response> response)
    {
        if (request->gain < 0.0) // Check if the requested gain is negative
        {
            response->success = false;
            response->message = "Gain must be non-negative.";
            RCLCPP_WARN(this->get_logger(), "Attempted to set a negative gain.");
        }
        else // Update the gain value if it is valid
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
    // Initialize the motor error message
    uclv_seed_robotics_ros_interfaces::msg::Float64WithIdsStamped error_msg;
    error_msg.header.stamp = this->now(); // Set the timestamp for the message

    // Iterate over each motor ID to compute the error
    for (int64_t motor_id : motor_ids_)
    {
        auto sensor_ids_iter = motor_to_sensor_map_.find(motor_id); // Find the associated sensor IDs for the motor
        if (sensor_ids_iter == motor_to_sensor_map_.end())          // Check if the motor has a valid mapping
        {
            RCLCPP_ERROR(this->get_logger(), "No mapping found for motor ID: %ld", motor_id);
            continue;
        }

        const auto &sensor_ids = sensor_ids_iter->second; // Get the associated sensor IDs for the motor
        double total_weighted_error = 0.0;                // Sum of weighted errors for the motor
        double total_weights = 0.0;                       // Sum of sensor weights

        // Iterate over each sensor ID to compute the weighted error
        for (int64_t sensor_id : sensor_ids)
        {
            // Find the weight(s) associated with the sensor
            auto weight_iter = sensor_to_weight_map_.find(sensor_id);
            if (weight_iter == sensor_to_weight_map_.end()) // Check if the sensor has a valid weight mapping
            {
                RCLCPP_ERROR(this->get_logger(), "No mapping found for sensor ID: %ld", sensor_id);
                continue;
            }

            const auto &weights = weight_iter->second;

            // Find the index of the sensor ID in the measured and desired forces
            auto measured_force_iter = std::find(measured_norm_forces_.ids.begin(), measured_norm_forces_.ids.end(), sensor_id);
            auto desired_force_iter = std::find(desired_norm_forces_.ids.begin(), desired_norm_forces_.ids.end(), sensor_id);

            // Check if the sensor ID is found in the measured and desired forces
            if (measured_force_iter == measured_norm_forces_.ids.end() || desired_force_iter == desired_norm_forces_.ids.end())
            {
                RCLCPP_ERROR(this->get_logger(), "Sensor ID: %ld not found in forces", sensor_id);
                continue;
            }

            // Compute the indices of the sensor ID in the measured and desired forces
            size_t measured_id = std::distance(measured_norm_forces_.ids.begin(), measured_force_iter);
            size_t desired_id = std::distance(desired_norm_forces_.ids.begin(), desired_force_iter);

            // Retrieve the measured and desired normalized forces
            double measured_norm = measured_norm_forces_.data[measured_id];
            double desired_norm = desired_norm_forces_.data[desired_id];

            // Compute the weighted error for each weight associated with the sensor
            for (const auto &weight : weights)
            {
                double error = (desired_norm - measured_norm) * weight; // Weighted error
                total_weighted_error += error;
                total_weights += weight; // Sum up the weights
            }
        }

        // Calculate the weighted average error for the motor
        if (total_weights > 0.0)
        {
            double average_error = total_weighted_error / total_weights;
            error_msg.ids.push_back(motor_id);
            error_msg.data.push_back(gain_ * average_error); // Apply the proportional gain to the average error
            RCLCPP_INFO(this->get_logger(), "Published error for motor ID: %ld - Error: %f", motor_id, average_error);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Total weight is zero for motor ID: %ld", motor_id);
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
    rclcpp::init(argc, argv); // Initialize the ROS 2 client library
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
